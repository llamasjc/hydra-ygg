//! Node-by-node and edge-by-edge diff of two Spark-DSG JSON graphs.
//!
//! Same rules as `tools/compare.py`: edges are undirected pairs, floats are compared with a
//! relative/absolute tolerance, and the largest position change is reported on its own.

use std::collections::{BTreeMap, HashMap};

use serde_json::Value;

#[derive(Debug, Default)]
pub struct Diff {
    pub missing_nodes: usize,
    pub extra_nodes: usize,
    pub missing_edges: usize,
    pub extra_edges: usize,
    pub max_position_error: f64,
    /// Human-readable descriptions of the first few attribute or edge-info mismatches.
    pub mismatches: Vec<String>,
    pub mismatch_count: usize,
}

impl Diff {
    pub fn is_equal(&self) -> bool {
        self.missing_nodes == 0
            && self.extra_nodes == 0
            && self.missing_edges == 0
            && self.extra_edges == 0
            && self.mismatch_count == 0
    }
}

const MAX_REPORTED: usize = 20;

pub fn compare(a: &Value, b: &Value, tol: f64) -> Diff {
    let mut diff = Diff::default();
    let nodes = |g: &Value| -> HashMap<u64, Value> {
        g["nodes"]
            .as_array()
            .map(|ns| {
                ns.iter()
                    .map(|n| (n["id"].as_u64().unwrap(), n.clone()))
                    .collect()
            })
            .unwrap_or_default()
    };
    let (na, nb) = (nodes(a), nodes(b));
    diff.missing_nodes = na.keys().filter(|k| !nb.contains_key(k)).count();
    diff.extra_nodes = nb.keys().filter(|k| !na.contains_key(k)).count();

    for (id, x) in &na {
        let Some(y) = nb.get(id) else { continue };
        for key in ["layer", "partition"] {
            let (p, q) = (x[key].as_i64().unwrap_or(0), y[key].as_i64().unwrap_or(0));
            if p != q {
                diff.push(format!("{id}.{key}: {p} vs {q}"));
            }
        }
        let (px, py) = (&x["attributes"]["position"], &y["attributes"]["position"]);
        if let (Some(px), Some(py)) = (px.as_array(), py.as_array()) {
            for (p, q) in px.iter().zip(py) {
                let err = (p.as_f64().unwrap_or(0.0) - q.as_f64().unwrap_or(0.0)).abs();
                diff.max_position_error = diff.max_position_error.max(err);
            }
        }
        diff.values(&x["attributes"], &y["attributes"], tol, &id.to_string());
    }

    let edges = |g: &Value| -> BTreeMap<(u64, u64), Vec<Value>> {
        let mut m: BTreeMap<(u64, u64), Vec<Value>> = BTreeMap::new();
        for e in g["edges"].as_array().into_iter().flatten() {
            let (s, t) = (e["source"].as_u64().unwrap(), e["target"].as_u64().unwrap());
            m.entry((s.min(t), s.max(t)))
                .or_default()
                .push(e["info"].clone());
        }
        m
    };
    let (ea, eb) = (edges(a), edges(b));
    for (k, infos) in &ea {
        let other = eb.get(k).map_or(0, Vec::len);
        diff.missing_edges += infos.len().saturating_sub(other);
    }
    for (k, infos) in &eb {
        let other = ea.get(k).map_or(0, Vec::len);
        diff.extra_edges += infos.len().saturating_sub(other);
    }
    for (k, infos) in &ea {
        if let Some(other) = eb.get(k) {
            diff.values(&infos[0], &other[0], tol, &format!("edge{k:?}"));
        }
    }
    diff
}

impl Diff {
    fn push(&mut self, msg: String) {
        self.mismatch_count += 1;
        if self.mismatches.len() < MAX_REPORTED {
            self.mismatches.push(msg);
        }
    }

    fn values(&mut self, a: &Value, b: &Value, tol: f64, path: &str) {
        match (a, b) {
            (Value::Number(x), Value::Number(y)) if x.is_f64() || y.is_f64() => {
                let (x, y) = (x.as_f64().unwrap(), y.as_f64().unwrap());
                if (x - y).abs() > tol.max(tol * x.abs().max(y.abs())) {
                    self.push(format!("{path}: {x} vs {y}"));
                }
            }
            (Value::Object(x), Value::Object(y)) => {
                for (k, v) in x {
                    match y.get(k) {
                        Some(w) => self.values(v, w, tol, &format!("{path}.{k}")),
                        None => self.push(format!("{path}.{k}: missing on the right")),
                    }
                }
                for k in y.keys().filter(|k| !x.contains_key(*k)) {
                    self.push(format!("{path}.{k}: missing on the left"));
                }
            }
            (Value::Array(x), Value::Array(y)) => {
                if x.len() != y.len() {
                    self.push(format!("{path}: length {} vs {}", x.len(), y.len()));
                } else {
                    for (i, (v, w)) in x.iter().zip(y).enumerate() {
                        self.values(v, w, tol, &format!("{path}[{i}]"));
                    }
                }
            }
            _ if a != b => self.push(format!("{path}: {a} vs {b}")),
            _ => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn graph(x: f64, weight: f64) -> Value {
        json!({
            "nodes": [{"id": 1, "layer": 3, "attributes": {"position": [x, 0.0, 0.0]}},
                      {"id": 2, "layer": 3, "attributes": {"position": [0.0, 0.0, 0.0]}}],
            "edges": [{"source": 2, "target": 1, "info": {"weight": weight}}],
        })
    }

    #[test]
    fn equal_within_tolerance() {
        let d = compare(&graph(1.0, 0.5), &graph(1.000_000_1, 0.5), 1e-5);
        assert!(d.is_equal(), "{d:?}");
        assert!(d.max_position_error > 0.0);
    }

    #[test]
    fn reports_changes() {
        let d = compare(&graph(1.0, 0.5), &graph(2.0, 0.7), 1e-5);
        assert_eq!(d.mismatch_count, 2);
        let mut b = graph(1.0, 0.5);
        b["edges"] = json!([]);
        assert_eq!(compare(&graph(1.0, 0.5), &b, 1e-5).missing_edges, 1);
    }
}
