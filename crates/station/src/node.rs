//! Reading Spark-DSG JSON nodes and edges.

use dsg_convert::LayerKey;
use serde_json::{Value, json};

/// An undirected edge, smaller id first.
pub type EdgeKey = (u64, u64);

pub fn edge_key(s: u64, t: u64) -> EdgeKey {
    (s.min(t), s.max(t))
}

pub fn layer_key(n: &Value) -> LayerKey {
    (
        n["layer"].as_i64().unwrap_or(0),
        n["partition"].as_i64().unwrap_or(0),
    )
}

pub fn position(n: &Value) -> Option<[f64; 3]> {
    let p = n["attributes"]["position"].as_array()?;
    Some([p[0].as_f64()?, p[1].as_f64()?, p[2].as_f64()?])
}

pub fn label(n: &Value) -> String {
    n["attributes"]["semantic_label"].to_string()
}

/// An edge's info, carrying its spark_dsg record (weight and other attributes) when it has one.
pub fn edge_info(e: &Value) -> Value {
    let mut info = e["info"].clone();
    if let Some(r) = e.get("record") {
        if !info.is_object() {
            info = json!({});
        }
        info["record"] = r.clone();
    }
    info
}

pub fn distance(p: [f64; 3], q: [f64; 3]) -> f64 {
    ((p[0] - q[0]).powi(2) + (p[1] - q[1]).powi(2) + (p[2] - q[2]).powi(2)).sqrt()
}

pub fn add(p: [f64; 3], q: [f64; 3]) -> [f64; 3] {
    [p[0] + q[0], p[1] + q[1], p[2] + q[2]]
}

pub fn sub(p: [f64; 3], q: [f64; 3]) -> [f64; 3] {
    [p[0] - q[0], p[1] - q[1], p[2] - q[2]]
}
