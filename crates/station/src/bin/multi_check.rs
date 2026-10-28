//! Two simulated robots on one saved Hydra graph, through the station.
//!
//! Robot 0 maps the graph as saved. Robot 1 maps the same building from its own frame (a
//! known rotation and offset) with a few centimetres of position noise. Both grow their
//! graphs over `steps` updates and send deltas. The station must put robot 1's objects back
//! on robot 0's, and deltas must reproduce every robot's graph exactly.
//!
//! usage: multi_check <dsg.json> [steps] [--objects-only] [--max-matching]

use std::collections::HashSet;
use std::time::Instant;

use dsg_convert::compare::compare;
use serde_json::{Value, json};
use station::delta::{Decoder, Encoder};
use station::{Config, Station, Transform, global_id, robot_of};

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

/// The first `fraction` of the graph's nodes and their edges, minus a few nodes that come and
/// go from step to step (`churn` picks which), the way Hydra's active window does.
fn prefix(g: &Value, fraction: f64, churn: u64) -> Value {
    let nodes = g["nodes"].as_array().unwrap();
    let keep = (nodes.len() as f64 * fraction).ceil() as usize;
    let kept: Vec<Value> = nodes
        .iter()
        .take(keep)
        .enumerate()
        .filter(|(i, _)| churn == 0 || !(*i as u64 * 2654435761 + churn).is_multiple_of(50))
        .map(|(_, n)| n.clone())
        .collect();
    let ids: HashSet<u64> = kept.iter().map(|n| n["id"].as_u64().unwrap()).collect();
    let edges: Vec<Value> = g["edges"]
        .as_array()
        .unwrap()
        .iter()
        .filter(|e| {
            ids.contains(&e["source"].as_u64().unwrap())
                && ids.contains(&e["target"].as_u64().unwrap())
        })
        .cloned()
        .collect();
    let mut out = g.clone();
    out["nodes"] = json!(kept);
    out["edges"] = json!(edges);
    out
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let path = args.get(1).expect("usage: multi_check <dsg.json> [steps]");
    let steps: usize = args.get(2).map_or(20, |s| s.parse().unwrap());
    let base: Value = serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap();

    // Interleave layers so every prefix holds some of each.
    let mut nodes = base["nodes"].as_array().unwrap().clone();
    let mut rng = Lcg(42);
    let mut keyed: Vec<(f64, Value)> = nodes.drain(..).map(|n| (rng.next(), n)).collect();
    keyed.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut base = base.clone();
    base["nodes"] = json!(keyed.into_iter().map(|(_, n)| n).collect::<Vec<_>>());

    // Robot 1's frame: station = T(robot 1), so robot 1 sees T^-1(station) plus noise.
    let t1 = Transform {
        yaw: 0.6,
        translation: [4.0, -2.5, 0.0],
    };
    let inverse = |p: [f64; 3]| {
        let q = [
            p[0] - t1.translation[0],
            p[1] - t1.translation[1],
            p[2] - t1.translation[2],
        ];
        let (s, c) = t1.yaw.sin_cos();
        [c * q[0] + s * q[1], -s * q[0] + c * q[1], q[2]]
    };
    let mut robot1 = base.clone();
    for n in robot1["nodes"].as_array_mut().unwrap() {
        if let Some(p) = n["attributes"]["position"].as_array() {
            let p = [0, 1, 2].map(|i| p[i].as_f64().unwrap() + (rng.next() - 0.5) * 0.06);
            n["attributes"]["position"] = json!(inverse(p));
        }
    }

    let config = if args.iter().any(|a| a == "--objects-only") {
        Config::objects_only()
    } else {
        Config::default()
    };
    let mut config = config;
    config.max_matching = args.iter().any(|a| a == "--max-matching");
    let rules = config.rules.clone();
    let mut station = Station::new(config);
    station.add_robot(0, Transform::default());
    station.add_robot(1, t1);
    let mut encoders = [Encoder::default(), Encoder::default()];
    let mut checks = [Decoder::default(), Decoder::default()];
    let (mut delta_bytes, mut full_bytes) = (0usize, 0usize);
    let mut ok = true;
    let start = Instant::now();
    let (mut station_s, mut updates) = (0.0, 0usize);
    for step in 1..=steps {
        let fraction = step as f64 / steps as f64;
        for (r, graph) in [&base, &robot1].into_iter().enumerate() {
            let churn = if step == steps { 0 } else { step as u64 };
            let snapshot = prefix(graph, fraction, churn);
            let delta = encoders[r].encode(&snapshot);
            delta_bytes += delta.to_string().len();
            full_bytes += snapshot.to_string().len();
            checks[r].apply(&delta).unwrap();
            if !compare(&snapshot, &checks[r].graph(), 0.0).is_equal() {
                println!("step {step}: robot {r} graph from deltas differs");
                ok = false;
            }
            let t = Instant::now();
            station.receive(r as u8, &delta).unwrap();
            station_s += t.elapsed().as_secs_f64();
            updates += 1;
        }
        // the incremental station must equal a from-scratch rebuild
        let (want, merges) = station.merged_graph();
        let d = compare(&want, &station.export(), 0.0);
        if !d.is_equal() || merges != station.merges {
            println!("step {step}: station differs from rebuild: {d:?}");
            ok = false;
        }
        println!(
            "step {step:2}: station nodes={} edges={} merged objects={}",
            station.num_nodes(),
            station.num_edges(),
            station.merges.len()
        );
    }

    // Every robot 1 node a rule covers should land on the same node of robot 0.
    let same_index = |a: u64, b: u64| (a & !(0xff << 48)) == (b & !(0xff << 48));
    let mut objects_ok = true;
    for rule in &rules {
        let ids: Vec<u64> = base["nodes"]
            .as_array()
            .unwrap()
            .iter()
            .filter(|n| {
                (n["layer"].as_i64(), n["partition"].as_i64().unwrap_or(0))
                    == (Some(rule.layer.0), rule.layer.1)
            })
            .map(|n| n["id"].as_u64().unwrap())
            .collect();
        let set: HashSet<u64> = ids.iter().map(|id| global_id(1, *id)).collect();
        let right = ids
            .iter()
            .filter(|id| station.merges.get(&global_id(1, **id)) == Some(&global_id(0, **id)))
            .count();
        let wrong = station
            .merges
            .iter()
            .filter(|(from, to)| set.contains(from) && !same_index(**from, **to))
            .count();
        println!(
            "layer {:?} (radius {} m): {} per robot, {right} merged correctly, {wrong} merged wrongly",
            rule.layer,
            rule.radius,
            ids.len()
        );
        if rule.layer == station::OBJECTS {
            objects_ok = right == ids.len() && wrong == 0;
        }
    }
    let stray = station
        .merges
        .keys()
        .filter(|from| robot_of(**from) != 1)
        .count();
    println!(
        "sent: deltas {:.1} MB vs full graphs {:.1} MB ({:.0}% saved), {:.2} s",
        delta_bytes as f64 / 1e6,
        full_bytes as f64 / 1e6,
        100.0 * (1.0 - delta_bytes as f64 / full_bytes as f64),
        start.elapsed().as_secs_f64()
    );
    println!(
        "station update: {:.1} ms per delta",
        1e3 * station_s / updates as f64
    );
    ok &= objects_ok && stray == 0;
    println!("{}", if ok { "MULTI OK" } else { "MULTI FAILED" });
    std::process::exit(if ok { 0 } else { 1 });
}
