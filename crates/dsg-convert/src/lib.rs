//! Round-trip a Spark-DSG JSON export (Hydra's `dsg.json`) through Yggdrasil.
//!
//! `dsg.json` -> `SceneGraph` -> serde JSON bytes -> `SceneGraph` -> Spark-DSG JSON.
//!
//! The mapping is deliberately lossless so the output can be diffed against the input:
//! * every Spark (layer, partition) pair becomes one Yggdrasil layer, ordered by layer number
//!   so that higher layers get higher ids (Yggdrasil requires child layer id < parent layer id);
//! * a node's `position` becomes its single coordinate, every other attribute is kept as one
//!   JSON-string feature;
//! * same-layer edges become Yggdrasil edges, with the edge info as the description;
//! * cross-layer edges become nesting (lower layer nested under higher layer). Cross-layer
//!   edges that cannot be expressed as nesting (second parent, or same layer number) are kept
//!   as a feature on the node and counted, since those are what the port has to redesign.

pub mod compare;
pub mod incremental;
pub mod topology;
pub mod typed;

use std::collections::{BTreeMap, BTreeSet, HashMap};

use scene_graph::{Feature, NodeId, SceneGraph};
use serde_json::{Map, Value, json};

pub const F_DSG_ID: &str = "dsg_id";
pub const F_ATTRS: &str = "dsg_attrs";
pub const F_PARENT_EDGE: &str = "dsg_parent_edge";
pub const F_EXTRA_EDGES: &str = "dsg_extra_edges";
pub const F_LABEL: &str = "semantic_label";

pub type LayerKey = (i64, i64);

/// What the import did, and which Spark-DSG edges did not fit Yggdrasil's model.
#[derive(Debug, Default)]
pub struct Report {
    pub nodes: usize,
    pub intra_edges: usize,
    pub nestings: usize,
    pub extra_parent_edges: usize,
    pub same_level_cross_edges: usize,
    pub no_position: usize,
}

pub(crate) fn key_of(node: &Value) -> LayerKey {
    (
        node["layer"].as_i64().unwrap(),
        node.get("partition").and_then(Value::as_i64).unwrap_or(0),
    )
}

pub fn import(input: &Value) -> (SceneGraph, BTreeMap<LayerKey, u32>, Report) {
    let mut report = Report::default();
    let nodes = input["nodes"].as_array().expect("nodes");
    let edges = input["edges"].as_array().expect("edges");

    // Layers ordered by (layer, partition): higher Spark layers get higher Yggdrasil ids.
    let keys: BTreeSet<LayerKey> = nodes.iter().map(key_of).collect();
    let mut sg = SceneGraph::default();
    let mut layer_keys = BTreeMap::new();
    for key in &keys {
        layer_keys.insert(*key, sg.new_layer().id);
    }

    let mut ids: HashMap<u64, NodeId> = HashMap::new();
    let mut keys_by_dsg: HashMap<u64, LayerKey> = HashMap::new();
    for node in nodes {
        let dsg_id = node["id"].as_u64().unwrap();
        let key = key_of(node);
        let mut attrs = node["attributes"].as_object().cloned().unwrap_or_default();
        let position = attrs.remove("position");
        let mut features = vec![Feature::new(F_DSG_ID, dsg_id)];
        if let Some(label) = attrs.get("semantic_label") {
            features.push(Feature::new(F_LABEL, label));
        }
        features.push(Feature::new(F_ATTRS, Value::Object(attrs)));
        let anon = match position.as_ref().and_then(Value::as_array) {
            Some(p) => {
                let c: Vec<f64> = p.iter().map(|v| v.as_f64().unwrap()).collect();
                sg.new_coordinates(c[0], c[1], c[2], features)
            }
            None => {
                report.no_position += 1;
                sg.new_node(features)
            }
        };
        let nid = sg
            .layer_mut(layer_keys[&key])
            .unwrap()
            .push_node(anon)
            .expect("push node");
        ids.insert(dsg_id, nid);
        keys_by_dsg.insert(dsg_id, key);
        report.nodes += 1;
    }

    // Declare the layer DAG from the cross-layer edges that actually occur.
    for e in edges {
        let (s, t) = (e["source"].as_u64().unwrap(), e["target"].as_u64().unwrap());
        let (ks, kt) = (keys_by_dsg[&s], keys_by_dsg[&t]);
        if ks.0 != kt.0 {
            let (hi, lo) = if ks.0 > kt.0 { (ks, kt) } else { (kt, ks) };
            sg.layer_mut(layer_keys[&hi])
                .unwrap()
                .add_child(layer_keys[&lo])
                .unwrap();
        }
    }

    for e in edges {
        let (s, t) = (e["source"].as_u64().unwrap(), e["target"].as_u64().unwrap());
        let (ks, kt) = (keys_by_dsg[&s], keys_by_dsg[&t]);
        let info = e["info"].to_string();
        if ks == kt {
            sg.layer_mut(layer_keys[&ks])
                .unwrap()
                .add_edge(ids[&s], ids[&t], &info)
                .unwrap();
            report.intra_edges += 1;
            continue;
        }
        let record = json!({"source": s, "target": t, "info": e["info"]});
        if ks.0 == kt.0 {
            push_record(&mut sg, ids[&s], F_EXTRA_EDGES, record);
            report.same_level_cross_edges += 1;
            continue;
        }
        let (parent, child) = if ks.0 > kt.0 { (s, t) } else { (t, s) };
        let (pid, cid) = (ids[&parent], ids[&child]);
        let current = sg.node(cid).unwrap().pid;
        if current.is_none() || current == Some(pid) {
            if current.is_none() {
                sg.nest(cid).under(pid).expect("nest");
            }
            push_record(&mut sg, cid, F_PARENT_EDGE, record);
            report.nestings += 1;
        } else {
            push_record(&mut sg, cid, F_EXTRA_EDGES, record);
            report.extra_parent_edges += 1;
        }
    }
    (sg, layer_keys, report)
}

fn set_feature(sg: &mut SceneGraph, nid: NodeId, key: &str, value: &str) {
    sg.node_mut(nid)
        .unwrap()
        .set_feature(Feature::new(key, value));
}

fn push_record(sg: &mut SceneGraph, nid: NodeId, key: &str, record: Value) {
    let mut list: Vec<Value> = sg
        .node(nid)
        .unwrap()
        .feature(key)
        .ok()
        .map(|s| serde_json::from_str(s).unwrap())
        .unwrap_or_default();
    list.push(record);
    set_feature(sg, nid, key, &Value::Array(list).to_string());
}

pub fn export(sg: &SceneGraph, layer_keys: &BTreeMap<LayerKey, u32>, input: &Value) -> Value {
    let dsg_id = |nid: NodeId| -> u64 {
        sg.node(nid)
            .unwrap()
            .feature(F_DSG_ID)
            .unwrap()
            .parse()
            .unwrap()
    };
    let mut nodes = Vec::new();
    let mut edges = Vec::new();
    for (key, lid) in layer_keys {
        for node in sg.layer(*lid).unwrap().all_nodes() {
            let mut attrs: Map<String, Value> =
                serde_json::from_str(node.feature(F_ATTRS).unwrap()).unwrap();
            if let Some(c) = node.coordinates.first() {
                attrs.insert(
                    "position".into(),
                    json!([c.x as f64, c.y as f64, c.z as f64]),
                );
            }
            nodes.push(json!({
                "id": dsg_id(node.id),
                "layer": key.0,
                "partition": key.1,
                "attributes": attrs,
            }));
            for e in &node.edges {
                edges.push(json!({
                    "source": dsg_id(e.src),
                    "target": dsg_id(e.dst),
                    "info": serde_json::from_str::<Value>(&e.desc).unwrap(),
                }));
            }
            for key in [F_PARENT_EDGE, F_EXTRA_EDGES] {
                if let Ok(list) = node.feature(key) {
                    edges.extend(serde_json::from_str::<Vec<Value>>(list).unwrap());
                }
            }
        }
    }
    // Graph-level header fields are not scene content; carry them over from the input.
    let mut out = input.as_object().unwrap().clone();
    out.insert("nodes".into(), Value::Array(nodes));
    out.insert("edges".into(), Value::Array(edges));
    Value::Object(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn node(id: u64, layer: i64, pos: [f64; 3]) -> Value {
        json!({"id": id, "layer": layer, "partition": 0,
               "attributes": {"position": pos, "semantic_label": 5, "name": format!("n{id}")}})
    }

    fn edge(s: u64, t: u64) -> Value {
        json!({"source": s, "target": t, "info": {"type": "EdgeAttributes", "weight": 0.5}})
    }

    /// Room 1 nests places 10 and 11; object 20 is linked to both places, so its second link
    /// cannot be nesting and must survive as a feature.
    fn small_graph() -> Value {
        json!({
            "directed": false,
            "nodes": [node(1, 4, [0.0, 0.0, 0.0]), node(10, 3, [1.0, 0.0, 0.0]),
                      node(11, 3, [2.0, 0.0, 0.0]), node(20, 2, [1.5, 0.0, 0.0])],
            "edges": [edge(1, 10), edge(1, 11), edge(10, 11), edge(10, 20), edge(11, 20)],
        })
    }

    fn edge_set(v: &Value) -> Vec<(u64, u64)> {
        let mut e: Vec<_> = v["edges"]
            .as_array()
            .unwrap()
            .iter()
            .map(|e| {
                let (s, t) = (e["source"].as_u64().unwrap(), e["target"].as_u64().unwrap());
                (s.min(t), s.max(t))
            })
            .collect();
        e.sort();
        e
    }

    #[test]
    fn round_trip_is_lossless() {
        let input = small_graph();
        let (sg, keys, report) = import(&input);
        assert_eq!(report.nodes, 4);
        assert_eq!(report.intra_edges, 1);
        assert_eq!(report.nestings, 3);
        assert_eq!(report.extra_parent_edges, 1);

        let sg: SceneGraph = serde_json::from_slice(&serde_json::to_vec(&sg).unwrap()).unwrap();
        let output = export(&sg, &keys, &input);
        assert_eq!(edge_set(&input), edge_set(&output));
        let names: Vec<_> = output["nodes"]
            .as_array()
            .unwrap()
            .iter()
            .map(|n| n["attributes"]["name"].as_str().unwrap().to_string())
            .collect();
        assert_eq!(names.len(), 4);
    }

    #[test]
    fn places_nest_under_room() {
        let (sg, keys, _) = import(&small_graph());
        let room_layer = sg.layer(keys[&(4, 0)]).unwrap();
        let room = room_layer.all_nodes().next().unwrap();
        assert_eq!(room.children().len(), 2);
    }
}
