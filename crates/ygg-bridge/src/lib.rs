//! C++ entry point for mirroring Hydra's scene graph into Yggdrasil.
//!
//! Each update is diffed against the previous one and only the changes are applied to Yggdrasil.

use std::time::Instant;

use dsg_convert::compare::compare;
use serde_json::Value;

#[cxx::bridge(namespace = "hydra_ygg")]
mod ffi {
    struct UpdateStats {
        nodes: usize,
        nodes_added: usize,
        nodes_updated: usize,
        nodes_removed: usize,
        edges_added: usize,
        edges_removed: usize,
        rebuilt: bool,
        apply_us: u64,
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

pub struct Mirror {
    inner: dsg_convert::incremental::Mirror,
}

pub fn new_mirror() -> Box<Mirror> {
    Box::new(Mirror {
        inner: Default::default(),
    })
}

impl Mirror {
    /// Bring the mirror to Hydra's latest graph, changing only what differs from the last one.
    pub fn apply_dsg_json(&mut self, json: &str) -> Result<ffi::UpdateStats, String> {
        let input: Value = serde_json::from_str(json).map_err(|e| e.to_string())?;
        let t = Instant::now();
        let d = self.inner.apply(&input);
        Ok(ffi::UpdateStats {
            nodes: input["nodes"].as_array().map_or(0, Vec::len),
            nodes_added: d.nodes_added,
            nodes_updated: d.nodes_updated,
            nodes_removed: d.nodes_removed,
            edges_added: d.edges_added,
            edges_removed: d.edges_removed,
            rebuilt: d.rebuilt,
            apply_us: t.elapsed().as_micros() as u64,
        })
    }

    pub fn export_value(&self) -> Value {
        self.inner.export()
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
        let bytes = serde_json::to_vec(&self.inner.sg).map_err(|e| e.to_string())?;
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
        assert_eq!(
            (stats.nodes, stats.nodes_added, stats.edges_added),
            (3, 3, 2)
        );
        let r = m.compare_with(json, 1e-5).unwrap();
        assert!(r.equal, "{}", r.summary);
    }
}
