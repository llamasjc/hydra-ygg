//! C++ entry point for mirroring Hydra's scene graph into Yggdrasil.
//!
//! Stage A: every update rebuilds the mirror from Hydra's Spark-DSG JSON. Incremental updates
//! come later, once the full rebuild is proven equal to Hydra's own output.

use std::time::Instant;

use dsg_convert::{LayerKey, compare::compare};
use scene_graph::SceneGraph;
use serde_json::Value;

#[cxx::bridge(namespace = "hydra_ygg")]
mod ffi {
    struct UpdateStats {
        nodes: usize,
        intra_edges: usize,
        nestings: usize,
        extra_parent_edges: usize,
        import_us: u64,
    }

    struct CompareResult {
        equal: bool,
        missing_nodes: usize,
        extra_nodes: usize,
        missing_edges: usize,
        extra_edges: usize,
        mismatches: usize,
        max_position_error: f64,
        summary: String,
    }

    extern "Rust" {
        type Mirror;
        fn new_mirror() -> Box<Mirror>;
        fn apply_dsg_json(self: &mut Mirror, json: &str) -> Result<UpdateStats>;
        fn export_dsg_json(self: &Mirror) -> String;
        fn compare_with(self: &Mirror, json: &str, tol: f64) -> Result<CompareResult>;
        fn save(self: &Mirror, path: &str) -> Result<()>;
    }
}

#[derive(Default)]
pub struct Mirror {
    graph: SceneGraph,
    layer_keys: std::collections::BTreeMap<LayerKey, u32>,
    /// Last input, used for header fields on export.
    header: Value,
}

pub fn new_mirror() -> Box<Mirror> {
    Box::default()
}

impl Mirror {
    pub fn apply_dsg_json(&mut self, json: &str) -> Result<ffi::UpdateStats, String> {
        let input: Value = serde_json::from_str(json).map_err(|e| e.to_string())?;
        let t = Instant::now();
        let (graph, layer_keys, report) = dsg_convert::import(&input);
        let import_us = t.elapsed().as_micros() as u64;
        self.graph = graph;
        self.layer_keys = layer_keys;
        let mut header = input;
        if let Some(h) = header.as_object_mut() {
            h.remove("nodes");
            h.remove("edges");
        }
        self.header = header;
        Ok(ffi::UpdateStats {
            nodes: report.nodes,
            intra_edges: report.intra_edges,
            nestings: report.nestings,
            extra_parent_edges: report.extra_parent_edges,
            import_us,
        })
    }

    pub fn export_value(&self) -> Value {
        let header = if self.header.is_object() {
            self.header.clone()
        } else {
            Value::Object(Default::default())
        };
        dsg_convert::export(&self.graph, &self.layer_keys, &header)
    }

    pub fn export_dsg_json(&self) -> String {
        self.export_value().to_string()
    }

    pub fn compare_with(&self, json: &str, tol: f64) -> Result<ffi::CompareResult, String> {
        let other: Value = serde_json::from_str(json).map_err(|e| e.to_string())?;
        let d = compare(&other, &self.export_value(), tol);
        Ok(ffi::CompareResult {
            equal: d.is_equal(),
            missing_nodes: d.missing_nodes,
            extra_nodes: d.extra_nodes,
            missing_edges: d.missing_edges,
            extra_edges: d.extra_edges,
            mismatches: d.mismatch_count,
            max_position_error: d.max_position_error,
            summary: d.mismatches.join("\n"),
        })
    }

    pub fn save(&self, path: &str) -> Result<(), String> {
        let bytes = serde_json::to_vec(&self.graph).map_err(|e| e.to_string())?;
        std::fs::write(path, bytes).map_err(|e| e.to_string())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mirror_matches_its_input() {
        let json = r#"{"directed":false,"nodes":[
            {"id":1,"layer":3,"partition":0,"attributes":{"position":[1.0,2.0,3.0]}},
            {"id":2,"layer":3,"partition":0,"attributes":{"position":[0.0,0.0,0.0]}},
            {"id":3,"layer":4,"partition":0,"attributes":{"position":[0.5,0.5,0.5]}}],
            "edges":[{"source":1,"target":2,"info":{"weight":1.0}},
                     {"source":3,"target":1,"info":{"weight":1.0}}]}"#;
        let mut m = new_mirror();
        let stats = m.apply_dsg_json(json).unwrap();
        assert_eq!((stats.nodes, stats.intra_edges, stats.nestings), (3, 1, 1));
        let r = m.compare_with(json, 1e-5).unwrap();
        assert!(r.equal, "{}", r.summary);
    }
}
