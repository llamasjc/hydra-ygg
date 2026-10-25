//! Round-trip a Spark-DSG JSON export through Yggdrasil and write it back out.
//!
//! usage: roundtrip <in dsg.json> <out dsg.json>, then diff with `compare.py`.

use std::time::Instant;

use dsg_convert::{export, import};
use scene_graph::SceneGraph;
use serde_json::Value;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() != 3 {
        eprintln!("usage: roundtrip <in dsg.json> <out dsg.json>");
        std::process::exit(2);
    }
    let input: Value =
        serde_json::from_str(&std::fs::read_to_string(&args[1]).expect("read input"))
            .expect("parse input");

    let t = Instant::now();
    let (sg, layer_keys, report) = import(&input);
    let import_time = t.elapsed();

    let t = Instant::now();
    let bytes = serde_json::to_vec(&sg).expect("serialize scene graph");
    let sg: SceneGraph = serde_json::from_slice(&bytes).expect("deserialize scene graph");
    let serde_time = t.elapsed();

    let t = Instant::now();
    let output = export(&sg, &layer_keys, &input);
    let export_time = t.elapsed();
    std::fs::write(&args[2], serde_json::to_string(&output).unwrap()).expect("write output");

    println!("layers:");
    for (key, lid) in &layer_keys {
        let layer = sg.layer(*lid).unwrap();
        println!(
            "  ygg {lid} <- dsg layer {} partition {}: {} nodes, child layers {:?}",
            key.0,
            key.1,
            layer.all_nodes().len(),
            layer.child_layer_ids()
        );
    }
    println!("nodes:                      {}", report.nodes);
    println!("same-layer edges:           {}", report.intra_edges);
    println!("cross-layer edges -> nest:  {}", report.nestings);
    println!(
        "second-parent edges (kept as feature): {}",
        report.extra_parent_edges
    );
    println!(
        "same-level cross edges (kept as feature): {}",
        report.same_level_cross_edges
    );
    println!("nodes without position:     {}", report.no_position);
    println!(
        "serialized size:            {:.1} MB",
        bytes.len() as f64 / 1e6
    );
    println!(
        "import {:?}, serde round trip {:?}, export {:?}",
        import_time, serde_time, export_time
    );
}
