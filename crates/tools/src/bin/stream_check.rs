//! Replays a saved Hydra graph as a stream of growing, changing snapshots and checks that the
//! incremental mirror matches every snapshot exactly, timing it against a full rebuild.
//!
//! usage: stream_check <dsg.json> [snapshots]

use std::collections::{BTreeSet, HashMap, HashSet};
use std::time::{Duration, Instant};

use dsg_convert::{F_DSG_ID, compare::compare, import, incremental::Mirror};
use scene_graph::SceneGraph;
use serde_json::{Value, json};

/// Small deterministic PRNG so runs are repeatable without extra dependencies.
struct Lcg(u64);
impl Lcg {
    fn next(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }
}

/// Parents whose children are the same in both graphs must have the same bounds, which only
/// holds if moves and re-nesting kept Yggdrasil's bubbled-up spatial data in sync.
fn spatial_mismatches(a: &SceneGraph, b: &SceneGraph) -> usize {
    let index = |sg: &SceneGraph| -> HashMap<u64, (BTreeSet<u64>, String)> {
        let dsg = |n: &scene_graph::Node| n.feature(F_DSG_ID).unwrap().parse::<u64>().unwrap();
        let mut out = HashMap::new();
        for layer in sg.layers() {
            for n in layer.all_nodes() {
                if n.children().is_empty() {
                    continue;
                }
                let kids = n
                    .children()
                    .iter()
                    .map(|c| dsg(sg.node(*c).unwrap()))
                    .collect();
                out.insert(dsg(n), (kids, format!("{:?}", n.bounds)));
            }
        }
        out
    };
    let (ia, ib) = (index(a), index(b));
    ia.iter()
        .filter(|(id, (kids, bounds))| {
            ib.get(*id).is_some_and(|(k2, b2)| {
                let bad = k2 == kids && b2 != bounds;
                if bad {
                    eprintln!("{id}: mirror {bounds}\n   fresh  {b2}");
                }
                bad
            })
        })
        .count()
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let input: Value = serde_json::from_str(&std::fs::read_to_string(&args[1]).unwrap()).unwrap();
    let steps: usize = args.get(2).map_or(40, |s| s.parse().unwrap());
    let nodes = input["nodes"].as_array().unwrap();
    let edges = input["edges"].as_array().unwrap();
    let mut rng = Lcg(7);

    let mut mirror = Mirror::default();
    let (mut t_inc, mut t_full) = (Duration::ZERO, Duration::ZERO);
    for step in 1..=steps {
        // Grow toward the full graph, drop ~2% of nodes, and move ~5% (like loop closures).
        let upto = nodes.len() * step / steps;
        let mut snap_nodes = Vec::new();
        for n in &nodes[..upto] {
            if step < steps && rng.next() < 0.02 {
                continue;
            }
            let mut n = n.clone();
            if step < steps
                && rng.next() < 0.05
                && let Some(p) = n["attributes"]["position"].as_array_mut()
            {
                p[0] = json!(p[0].as_f64().unwrap() + 0.1 * step as f64);
            }
            snap_nodes.push(n);
        }
        let ids: HashSet<u64> = snap_nodes
            .iter()
            .map(|n| n["id"].as_u64().unwrap())
            .collect();
        let snap_edges: Vec<Value> = edges
            .iter()
            .filter(|e| {
                ids.contains(&e["source"].as_u64().unwrap())
                    && ids.contains(&e["target"].as_u64().unwrap())
            })
            .cloned()
            .collect();
        let mut snap = input.clone();
        snap["nodes"] = Value::Array(snap_nodes);
        snap["edges"] = Value::Array(snap_edges);

        let t = Instant::now();
        let delta = mirror.apply(&snap);
        t_inc += t.elapsed();
        let t = Instant::now();
        let (fresh, _, _) = import(&snap);
        t_full += t.elapsed();

        let d = compare(&snap, &mirror.export(), 1e-5);
        println!(
            "step {step:3}: {:5} nodes  +{} ~{} -{} nodes  +{} -{} edges{}  {}",
            ids.len(),
            delta.nodes_added,
            delta.nodes_updated,
            delta.nodes_removed,
            delta.edges_added,
            delta.edges_removed,
            if delta.rebuilt { "  (rebuilt)" } else { "" },
            if d.is_equal() {
                "MATCH".to_string()
            } else {
                format!("DIFFERS {d:?}")
            }
        );
        assert!(d.is_equal());
        let spatial = spatial_mismatches(&mirror.sg, &fresh);
        assert_eq!(
            spatial, 0,
            "{spatial} parents' bounds differ from a fresh import"
        );
    }
    println!("total apply time: incremental {t_inc:?}, full rebuild {t_full:?}");
    println!("STREAM OK");
}
