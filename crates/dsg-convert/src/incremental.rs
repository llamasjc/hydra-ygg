//! Keep a Yggdrasil graph in sync with a stream of Spark-DSG snapshots by applying only what
//! changed between them.
//!
//! Uses the same mapping as [`crate::import`], so [`crate::export`] of a mirror equals the
//! export of a fresh import of the latest snapshot. Which of several possible parents a node
//! is nested under can differ from a fresh import; the exported edges cannot.

use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};

use scene_graph::{Bounds, Coordinate, Feature, NodeId, SceneGraph};
use serde_json::{Map, Value, json};

use crate::{F_ATTRS, F_DSG_ID, F_EXTRA_EDGES, F_LABEL, F_PARENT_EDGE, LayerKey, key_of};

/// Hydra's standard layers (objects, agents, places, mesh places, rooms, buildings). They are
/// created up front so their first node later on does not force a renumbering rebuild.
pub const HYDRA_LAYERS: [LayerKey; 6] = [(2, 0), (2, 97), (3, 0), (3, 1), (4, 0), (5, 0)];

type EdgeKey = (u64, u64);

#[derive(Clone)]
struct NodeState {
    key: LayerKey,
    attrs: String,
    label: Option<String>,
    pos: Option<[f64; 3]>,
}

#[derive(Clone, Copy, PartialEq)]
enum Kind {
    /// Same (layer, partition): a Yggdrasil edge.
    Intra,
    /// Different layer number: the lower node may be nested under the higher one.
    Cross { child: u64, parent: u64 },
    /// Same layer number, different partition: kept as a record on `owner`.
    SameLevel { owner: u64 },
}

#[derive(Clone, PartialEq)]
struct EdgeState {
    source: u64,
    target: u64,
    info: Value,
    kind: Kind,
}

impl EdgeState {
    fn owner(&self) -> Option<u64> {
        match self.kind {
            Kind::Intra => None,
            Kind::Cross { child, .. } => Some(child),
            Kind::SameLevel { owner } => Some(owner),
        }
    }

    fn record(&self) -> Value {
        json!({"source": self.source, "target": self.target, "info": self.info})
    }
}

/// What one snapshot changed in the mirror.
#[derive(Debug, Default, Clone, PartialEq)]
pub struct Delta {
    pub nodes_added: usize,
    pub nodes_updated: usize,
    pub nodes_removed: usize,
    pub edges_added: usize,
    pub edges_removed: usize,
    /// True when the snapshot introduced a new (layer, partition) and the mirror was rebuilt.
    pub rebuilt: bool,
}

#[derive(Default)]
pub struct Mirror {
    pub sg: SceneGraph,
    pub layer_keys: BTreeMap<LayerKey, u32>,
    header: Value,
    ids: HashMap<u64, NodeId>,
    dsg_of: HashMap<NodeId, u64>,
    nodes: HashMap<u64, NodeState>,
    /// Last input JSON of each node; unchanged nodes are skipped without re-encoding them.
    raw: HashMap<u64, Value>,
    edges: HashMap<EdgeKey, EdgeState>,
    /// Edge keys touching each node, so removals do not scan every edge.
    adjacency: HashMap<u64, HashSet<EdgeKey>>,
}

fn edge_key(s: u64, t: u64) -> EdgeKey {
    (s.min(t), s.max(t))
}

fn node_state(node: &Value) -> NodeState {
    let mut attrs: Map<String, Value> = node["attributes"].as_object().cloned().unwrap_or_default();
    let pos = attrs.remove("position").and_then(|p| {
        let p = p
            .as_array()?
            .iter()
            .map(|v| v.as_f64().unwrap())
            .collect::<Vec<_>>();
        Some([p[0], p[1], p[2]])
    });
    NodeState {
        key: key_of(node),
        label: attrs.get("semantic_label").map(Value::to_string),
        attrs: Value::Object(attrs).to_string(),
        pos,
    }
}

impl Mirror {
    pub fn header(&self) -> &Value {
        &self.header
    }

    /// Spark-DSG ids of the nodes the mirror holds.
    pub fn node_ids(&self) -> impl Iterator<Item = u64> + '_ {
        self.raw.keys().copied()
    }

    /// Undirected (smaller id, larger id) pairs of the edges the mirror holds.
    pub fn edge_pairs(&self) -> impl Iterator<Item = (u64, u64)> + '_ {
        self.edges.keys().copied()
    }

    pub fn export(&self) -> Value {
        crate::export(&self.sg, &self.layer_keys, &self.header)
    }

    /// Bring the mirror to `input` (a full Spark-DSG JSON graph), touching only what changed.
    pub fn apply(&mut self, input: &Value) -> Delta {
        let header: Map<String, Value> = input
            .as_object()
            .into_iter()
            .flatten()
            .filter(|(k, _)| *k != "nodes" && *k != "edges")
            .map(|(k, v)| (k.clone(), v.clone()))
            .collect();
        self.header = Value::Object(header);

        let empty = Vec::new();
        let in_nodes = input["nodes"].as_array().unwrap_or(&empty);
        let in_edges = input["edges"].as_array().unwrap_or(&empty);
        let new_nodes: HashMap<u64, &Value> = in_nodes
            .iter()
            .map(|n| (n["id"].as_u64().unwrap(), n))
            .collect();
        let key = |id: &u64| key_of(new_nodes[id]);

        // Yggdrasil layer ids must grow with the layer number, so an unseen (layer, partition)
        // means renumbering: start again from an empty mirror.
        let keys: BTreeSet<LayerKey> = new_nodes.values().map(|n| key_of(n)).collect();
        let moved = new_nodes
            .keys()
            .any(|id| self.nodes.get(id).is_some_and(|o| o.key != key(id)));
        let mut rebuilt = false;
        if moved || keys.iter().any(|k| !self.layer_keys.contains_key(k)) {
            let header = std::mem::take(&mut self.header);
            *self = Mirror {
                header,
                ..Default::default()
            };
            let all: BTreeSet<LayerKey> = keys.into_iter().chain(HYDRA_LAYERS).collect();
            for key in all {
                self.layer_keys.insert(key, self.sg.new_layer().id);
            }
            rebuilt = true;
        }

        // Borrowed views of the input edges; only new or changed ones are copied below.
        let mut new_edges: HashMap<EdgeKey, (u64, u64, &Value, Kind)> = HashMap::new();
        for e in in_edges {
            let (s, t) = (e["source"].as_u64().unwrap(), e["target"].as_u64().unwrap());
            let (ks, kt) = (key(&s), key(&t));
            let kind = if ks == kt {
                Kind::Intra
            } else if ks.0 == kt.0 {
                Kind::SameLevel { owner: s }
            } else if ks.0 > kt.0 {
                Kind::Cross {
                    child: t,
                    parent: s,
                }
            } else {
                Kind::Cross {
                    child: s,
                    parent: t,
                }
            };
            new_edges.insert(edge_key(s, t), (s, t, &e["info"], kind));
        }

        let mut delta = Delta {
            rebuilt,
            ..Default::default()
        };
        let mut touched: BTreeSet<u64> = BTreeSet::new();

        // 1. Edges that are gone or changed.
        let stale: Vec<EdgeKey> = self
            .edges
            .iter()
            .filter(|(k, e)| {
                new_edges.get(*k).is_none_or(|&(s, t, info, kind)| {
                    (s, t, kind) != (e.source, e.target, e.kind) || *info != e.info
                })
            })
            .map(|(k, _)| *k)
            .collect();
        for k in stale {
            self.remove_edge(k, &mut touched);
            delta.edges_removed += 1;
        }

        // 2. Nodes that are gone. Their edges were removed in step 1.
        let gone: Vec<u64> = self
            .nodes
            .keys()
            .filter(|id| !new_nodes.contains_key(id))
            .copied()
            .collect();
        for id in gone {
            self.remove_node(id);
            touched.remove(&id);
            delta.nodes_removed += 1;
        }

        // 3. New and changed nodes.
        for (id, value) in &new_nodes {
            match self.raw.get(id) {
                Some(old) if old == *value => continue,
                Some(_) => {
                    self.update_node(*id, node_state(value));
                    delta.nodes_updated += 1;
                }
                None => {
                    self.add_node(*id, node_state(value));
                    delta.nodes_added += 1;
                }
            }
            self.raw.insert(*id, (*value).clone());
        }

        // 4. New and changed edges.
        let mut fresh: Vec<(EdgeKey, EdgeState)> = new_edges
            .into_iter()
            .filter(|(k, _)| !self.edges.contains_key(k))
            .map(|(k, (source, target, info, kind))| {
                (
                    k,
                    EdgeState {
                        source,
                        target,
                        info: info.clone(),
                        kind,
                    },
                )
            })
            .collect();
        fresh.sort_by_key(|(k, _)| *k);
        for (k, e) in fresh {
            self.add_edge(k, e, &mut touched);
            delta.edges_added += 1;
        }

        // 5. Nesting and edge records of every node whose cross-layer edges changed.
        for id in touched {
            self.refresh_owner(id);
        }
        delta
    }

    fn add_node(&mut self, id: u64, state: NodeState) {
        let mut features = vec![Feature::new(F_DSG_ID, id)];
        if let Some(label) = &state.label {
            features.push(Feature::new(F_LABEL, label));
        }
        features.push(Feature::new(F_ATTRS, &state.attrs));
        let anon = match state.pos {
            Some([x, y, z]) => self.sg.new_coordinates(x as f32, y as f32, z as f32, features),
            None => self.sg.new_node(features),
        };
        let lid = self.layer_keys[&state.key];
        let nid = self
            .sg
            .layer_mut(lid)
            .unwrap()
            .push_node(anon)
            .expect("push node");
        self.ids.insert(id, nid);
        self.dsg_of.insert(nid, id);
        self.nodes.insert(id, state);
    }

    fn update_node(&mut self, id: u64, state: NodeState) {
        let nid = self.ids[&id];
        let moved = self.nodes[&id].pos != state.pos;
        // Yggdrasil keeps parents' spatial index and bounds in sync only through nest/detach,
        // so a node that moves is taken out from under its parent and put back afterwards.
        let parent = if moved {
            self.sg.node(nid).unwrap().pid
        } else {
            None
        };
        if let Some(p) = parent {
            self.sg.detach(nid).from(p).unwrap();
        }
        // A parent's bounds also cover its children, so rebuild them the way nesting would.
        let coord = state.pos.map(|[x, y, z]| Coordinate::new(x as f32, y as f32, z as f32));
        let bounds = moved.then(|| {
            let node = self.sg.node(nid).unwrap();
            node.children()
                .iter()
                .filter_map(|c| self.sg.node(*c).ok()?.bounds)
                .fold(coord.map(Bounds::point), |acc, b| {
                    Some(*acc.unwrap_or(Bounds::Unbounded).merge(&b))
                })
        });
        let mut node = self.sg.node_mut(nid).unwrap();
        node.set_feature(Feature::new(F_ATTRS, &state.attrs));
        match &state.label {
            Some(label) => node.set_feature(Feature::new(F_LABEL, label)),
            None => node.features.retain(|f| f.key() != F_LABEL),
        }
        if let Some(bounds) = bounds {
            node.coordinates = coord.into_iter().collect();
            node.bounds = bounds;
        }
        // NodeMut re-indexes the layer's position index when it is dropped.
        drop(node);
        if let Some(p) = parent {
            self.sg.nest(nid).under(p).unwrap();
        }
        self.nodes.insert(id, state);
    }

    fn remove_node(&mut self, id: u64) {
        let nid = self.ids.remove(&id).unwrap();
        // del_node also deletes children; anything still nested here is re-parented later.
        for child in self.sg.node(nid).unwrap().children().to_vec() {
            self.sg.detach(child).from(nid).unwrap();
        }
        self.sg.del_node(nid).unwrap();
        self.dsg_of.remove(&nid);
        self.nodes.remove(&id);
        self.raw.remove(&id);
        self.adjacency.remove(&id);
    }

    fn add_edge(&mut self, k: EdgeKey, e: EdgeState, touched: &mut BTreeSet<u64>) {
        let (s, t) = (self.ids[&e.source], self.ids[&e.target]);
        match e.kind {
            Kind::Intra => {
                let lid = self.layer_keys[&self.nodes[&e.source].key];
                self.sg
                    .layer_mut(lid)
                    .unwrap()
                    .add_edge(s, t, &e.info.to_string())
                    .unwrap();
            }
            Kind::Cross { child, parent } => {
                let (pl, cl) = (
                    self.layer_keys[&self.nodes[&parent].key],
                    self.layer_keys[&self.nodes[&child].key],
                );
                self.sg.layer_mut(pl).unwrap().add_child(cl).unwrap();
                touched.insert(child);
            }
            Kind::SameLevel { owner } => {
                touched.insert(owner);
            }
        }
        for end in [e.source, e.target] {
            self.adjacency.entry(end).or_default().insert(k);
        }
        self.edges.insert(k, e);
    }

    fn remove_edge(&mut self, k: EdgeKey, touched: &mut BTreeSet<u64>) {
        let e = self.edges.remove(&k).unwrap();
        for end in [e.source, e.target] {
            if let Some(adj) = self.adjacency.get_mut(&end) {
                adj.remove(&k);
            }
        }
        match e.kind {
            Kind::Intra => {
                let lid = self.layer_keys[&self.nodes[&e.source].key];
                let (s, t) = (self.ids[&e.source], self.ids[&e.target]);
                self.sg.layer_mut(lid).unwrap().del_edge(s, t).unwrap();
            }
            _ => {
                touched.insert(e.owner().unwrap());
            }
        }
    }

    /// Re-derive a node's parent and its stored cross-layer edge records from `self.edges`.
    fn refresh_owner(&mut self, id: u64) {
        let Some(&nid) = self.ids.get(&id) else {
            return;
        };
        let mut owned: Vec<&EdgeState> = self
            .adjacency
            .get(&id)
            .into_iter()
            .flatten()
            .map(|k| &self.edges[k])
            .filter(|e| e.owner() == Some(id))
            .collect();
        owned.sort_by_key(|e| edge_key(e.source, e.target));
        let parents: Vec<u64> = owned
            .iter()
            .filter_map(|e| match e.kind {
                Kind::Cross { parent, .. } => Some(parent),
                _ => None,
            })
            .collect();

        let current = self.sg.node(nid).unwrap().pid.map(|p| self.dsg_of[&p]);
        let parent = match current {
            Some(p) if parents.contains(&p) => Some(p),
            other => {
                if let Some(p) = other {
                    self.sg.detach(nid).from(self.ids[&p]).unwrap();
                }
                let first = parents.first().copied();
                if let Some(p) = first {
                    self.sg.nest(nid).under(self.ids[&p]).unwrap();
                }
                first
            }
        };

        let (nesting, extra): (Vec<&EdgeState>, Vec<&EdgeState>) = owned
            .into_iter()
            .partition(|e| matches!(e.kind, Kind::Cross { parent: p, .. } if Some(p) == parent));
        let records = |list: Vec<&EdgeState>| -> Option<String> {
            (!list.is_empty())
                .then(|| Value::Array(list.iter().map(|e| e.record()).collect()).to_string())
        };
        let (nesting, extra) = (records(nesting), records(extra));
        let mut node = self.sg.node_mut(nid).unwrap();
        for (key, value) in [(F_PARENT_EDGE, nesting), (F_EXTRA_EDGES, extra)] {
            match value {
                Some(v) => node.set_feature(Feature::new(key, v)),
                None => node.features.retain(|f| f.key() != key),
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::compare::compare;

    fn node(id: u64, layer: i64, x: f64) -> Value {
        json!({"id": id, "layer": layer, "partition": 0,
               "attributes": {"position": [x, 0.0, 0.0], "name": format!("n{id}")}})
    }

    fn edge(s: u64, t: u64) -> Value {
        json!({"source": s, "target": t, "info": {"weight": 1.0}})
    }

    fn graph(nodes: Vec<Value>, edges: Vec<Value>) -> Value {
        json!({"directed": false, "nodes": nodes, "edges": edges})
    }

    fn assert_matches(m: &Mirror, input: &Value) {
        let d = compare(input, &m.export(), 1e-5);
        assert!(d.is_equal(), "{d:?}");
    }

    #[test]
    fn follows_adds_moves_and_deletes() {
        let mut m = Mirror::default();
        // Room 1 with places 10, 11; object 20 under place 10.
        let a = graph(
            vec![
                node(1, 4, 0.0),
                node(10, 3, 1.0),
                node(11, 3, 2.0),
                node(20, 2, 1.5),
            ],
            vec![edge(1, 10), edge(1, 11), edge(10, 11), edge(10, 20)],
        );
        m.apply(&a);
        assert_matches(&m, &a);

        // Place 10 deleted (object 20 must survive and move to place 11), place 11 moved,
        // new room 2 takes place 11, object gains a second parent.
        let mut moved = node(11, 3, 2.5);
        moved["attributes"]["name"] = json!("renamed");
        let b = graph(
            vec![
                node(1, 4, 0.0),
                node(2, 4, 5.0),
                moved,
                node(20, 2, 1.5),
                node(21, 2, 2.0),
            ],
            vec![edge(2, 11), edge(11, 20), edge(11, 21), edge(20, 21)],
        );
        let delta = m.apply(&b);
        assert_matches(&m, &b);
        assert_eq!(
            (delta.nodes_added, delta.nodes_removed, delta.nodes_updated),
            (2, 1, 1)
        );
        assert!(!delta.rebuilt);
        let obj = m.sg.node(m.ids[&20]).unwrap();
        assert_eq!(obj.pid, Some(m.ids[&11]));

        // A layer outside the predeclared Hydra set forces a rebuild, which must still match.
        let mut c = b.clone();
        c["nodes"].as_array_mut().unwrap().push(node(99, 6, 0.0));
        assert!(m.apply(&c).rebuilt);
        assert_matches(&m, &c);

        // Applying the same snapshot again changes nothing.
        assert_eq!(m.apply(&c), Delta::default());
    }
}
