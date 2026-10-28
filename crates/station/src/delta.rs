//! What a robot sends the station after each update: only what changed since the last one,
//! instead of the whole graph (Hydra-Multi's main bandwidth cost).

use std::collections::HashMap;

use serde_json::{Map, Value, json};

type EdgeKey = (u64, u64);

fn edge_key(s: u64, t: u64) -> EdgeKey {
    (s.min(t), s.max(t))
}

fn index(snapshot: &Value) -> (HashMap<u64, Value>, HashMap<EdgeKey, Value>) {
    let nodes = snapshot["nodes"]
        .as_array()
        .into_iter()
        .flatten()
        .map(|n| (n["id"].as_u64().unwrap(), n.clone()))
        .collect();
    let edges = snapshot["edges"]
        .as_array()
        .into_iter()
        .flatten()
        .map(|e| {
            let (s, t) = (e["source"].as_u64().unwrap(), e["target"].as_u64().unwrap());
            (edge_key(s, t), e.clone())
        })
        .collect();
    (nodes, edges)
}

fn header(snapshot: &Value) -> Value {
    Value::Object(
        snapshot
            .as_object()
            .into_iter()
            .flatten()
            .filter(|(k, _)| *k != "nodes" && *k != "edges")
            .map(|(k, v)| (k.clone(), v.clone()))
            .collect::<Map<_, _>>(),
    )
}

/// Robot side: turns consecutive Spark-DSG snapshots into deltas.
#[derive(Default)]
pub struct Encoder {
    seq: u64,
    nodes: HashMap<u64, Value>,
    edges: HashMap<EdgeKey, Value>,
    header: Value,
}

impl Encoder {
    /// The delta from the previous snapshot to `snapshot`, as JSON:
    /// `{seq, header?, nodes: [records added or changed], removed_nodes: [ids],
    ///   edges: [records added or changed], removed_edges: [[s, t]]}`.
    pub fn encode(&mut self, snapshot: &Value) -> Value {
        let (nodes, edges) = index(snapshot);
        let changed_nodes: Vec<&Value> = nodes
            .iter()
            .filter(|(id, n)| self.nodes.get(id) != Some(n))
            .map(|(_, n)| n)
            .collect();
        let removed_nodes: Vec<u64> = self
            .nodes
            .keys()
            .filter(|id| !nodes.contains_key(id))
            .copied()
            .collect();
        let changed_edges: Vec<&Value> = edges
            .iter()
            .filter(|(k, e)| self.edges.get(k) != Some(e))
            .map(|(_, e)| e)
            .collect();
        let removed_edges: Vec<[u64; 2]> = self
            .edges
            .keys()
            .filter(|k| !edges.contains_key(k))
            .map(|(s, t)| [*s, *t])
            .collect();

        self.seq += 1;
        let mut delta = json!({
            "seq": self.seq,
            "nodes": changed_nodes,
            "removed_nodes": removed_nodes,
            "edges": changed_edges,
            "removed_edges": removed_edges,
        });
        let h = header(snapshot);
        if h != self.header {
            delta["header"] = h.clone();
            self.header = h;
        }
        self.nodes = nodes;
        self.edges = edges;
        delta
    }
}

/// Station side: rebuilds a robot's graph from its deltas.
#[derive(Default)]
pub struct Decoder {
    pub seq: u64,
    pub nodes: HashMap<u64, Value>,
    pub edges: HashMap<EdgeKey, Value>,
    pub header: Value,
}

impl Decoder {
    pub fn apply(&mut self, delta: &Value) -> Result<(), String> {
        let seq = delta["seq"].as_u64().ok_or("delta without seq")?;
        if seq != self.seq + 1 {
            return Err(format!("expected delta {}, got {seq}", self.seq + 1));
        }
        if let Some(h) = delta.get("header") {
            self.header = h.clone();
        }
        for id in delta["removed_nodes"].as_array().into_iter().flatten() {
            self.nodes.remove(&id.as_u64().unwrap());
        }
        for pair in delta["removed_edges"].as_array().into_iter().flatten() {
            let (s, t) = (pair[0].as_u64().unwrap(), pair[1].as_u64().unwrap());
            self.edges.remove(&edge_key(s, t));
        }
        for n in delta["nodes"].as_array().into_iter().flatten() {
            self.nodes.insert(n["id"].as_u64().unwrap(), n.clone());
        }
        for e in delta["edges"].as_array().into_iter().flatten() {
            let (s, t) = (e["source"].as_u64().unwrap(), e["target"].as_u64().unwrap());
            self.edges.insert(edge_key(s, t), e.clone());
        }
        self.seq = seq;
        Ok(())
    }

    /// The robot's current graph as Spark-DSG JSON.
    pub fn graph(&self) -> Value {
        let mut g = self.header.clone();
        if !g.is_object() {
            g = json!({});
        }
        let mut nodes: Vec<&Value> = self.nodes.values().collect();
        nodes.sort_by_key(|n| n["id"].as_u64());
        let mut keys: Vec<&EdgeKey> = self.edges.keys().collect();
        keys.sort();
        g["nodes"] = json!(nodes);
        g["edges"] = json!(keys.iter().map(|k| &self.edges[k]).collect::<Vec<_>>());
        g
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use dsg_convert::compare::compare;

    fn graph(nodes: &[(u64, f64)], edges: &[(u64, u64)]) -> Value {
        json!({
            "directed": false,
            "nodes": nodes.iter().map(|(id, x)| json!({"id": id, "layer": 3, "partition": 0,
                "attributes": {"position": [x, 0.0, 0.0]}})).collect::<Vec<_>>(),
            "edges": edges.iter().map(|(s, t)| json!({"source": s, "target": t,
                "info": {"weight": 1.0}})).collect::<Vec<_>>(),
        })
    }

    #[test]
    fn decoder_rebuilds_every_snapshot() {
        let snapshots = [
            graph(&[(1, 0.0), (2, 1.0)], &[(1, 2)]),
            graph(&[(1, 0.5), (2, 1.0), (3, 2.0)], &[(1, 2), (3, 2)]),
            graph(&[(2, 1.0), (3, 2.0)], &[(2, 3)]),
        ];
        let (mut enc, mut dec) = (Encoder::default(), Decoder::default());
        for s in &snapshots {
            dec.apply(&enc.encode(s)).unwrap();
            assert!(compare(s, &dec.graph(), 0.0).is_equal());
        }
        // unchanged snapshot: nothing to send
        let d = enc.encode(&snapshots[2]);
        assert!(d["nodes"].as_array().unwrap().is_empty() && d.get("header").is_none());
        dec.apply(&d).unwrap();
        // a lost delta is detected
        enc.encode(&snapshots[0]);
        assert!(dec.apply(&enc.encode(&snapshots[1])).is_err());
    }
}
