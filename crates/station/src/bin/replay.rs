//! Replays recorded robot runs through the station, in time order.
//!
//! Each Hydra run with `HYDRA_YGG_RECORD=<file>` writes one delta per backend update. This
//! reads one such file per robot (robot 0 first), feeds every delta to the station in
//! timestamp order and reports the station graph.
//!
//! Recordings ending in .gz are read through gzip.
//!
//! usage: replay <robot0.jsonl> [<robot1.jsonl> ...] [--transform R,yaw,x,y,z]...
//!               [--objects-only] [--many-to-one] [--radius L,P=R]... [--out station.json]
//!
//! --align finds each later robot's frame from its objects (ignoring any --transform given,
//! which then only sets the starting guess) and re-merges when a better fit is found. Giving
//! wrong --transform guesses on the shared-frame uHumans2 runs tests it: the answer is identity.
//! --max-matching re-pairs places so the most merge (largest one-to-one matching in the radius).
//! --deform corrects drift between robots (merges as loop closures, see station::deform) in
//! the written graph, and reports how it moves places when only object merges are used.
//! --many-to-one lets several nodes of one robot merge into the same node (the old behaviour).
//!
//! --radius 3,0=0.6 sets the merge radius for layer 3 partition 0 (places). After the replay,
//! the distance from each node of robots 1.. to the nearest node of an earlier robot in the
//! same layer is summarised, to show which radius catches the duplicates.

use std::collections::BTreeMap;
use std::fs::File;
use std::io::{BufRead, BufReader, Lines, Read};
use std::time::Instant;

use serde_json::Value;
use station::{Config, Station, Transform, robot_of};

struct Run {
    lines: Lines<BufReader<Box<dyn Read>>>,
    next: Option<Value>,
    deltas: usize,
}

impl Run {
    fn advance(&mut self) {
        // A recording cut off mid-write (Hydra stopped hard) ends at its last whole update.
        self.next = match self.lines.next() {
            Some(Ok(l)) => serde_json::from_str(&l).ok(),
            _ => None,
        };
    }
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut files = Vec::new();
    let mut transforms: BTreeMap<u8, Transform> = BTreeMap::new();
    let mut out = None;
    let mut objects_only = false;
    let mut many_to_one = false;
    let mut max_matching = false;
    let mut align = false;
    let mut deform = false;
    let mut radii: Vec<((i64, i64), f64)> = Vec::new();
    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            "--transform" => {
                let v: Vec<f64> = args[i + 1].split(',').map(|x| x.parse().unwrap()).collect();
                transforms.insert(
                    v[0] as u8,
                    Transform {
                        yaw: v[1],
                        translation: [v[2], v[3], v[4]],
                    },
                );
                i += 1;
            }
            "--out" => {
                out = Some(args[i + 1].clone());
                i += 1;
            }
            "--objects-only" => objects_only = true,
            "--many-to-one" => many_to_one = true,
            "--max-matching" => max_matching = true,
            "--align" => align = true,
            "--deform" => deform = true,
            "--radius" => {
                let (k, r) = args[i + 1].split_once('=').unwrap();
                let (l, p) = k.split_once(',').unwrap();
                radii.push(((l.parse().unwrap(), p.parse().unwrap()), r.parse().unwrap()));
                i += 1;
            }
            f => files.push(f.to_string()),
        }
        i += 1;
    }
    if files.is_empty() {
        eprintln!("usage: replay <robot0.jsonl> [<robot1.jsonl> ...] [--transform R,yaw,x,y,z]");
        std::process::exit(2);
    }

    let config = if objects_only {
        Config::objects_only()
    } else {
        Config::default()
    };
    let mut config = config;
    config.one_to_one = !many_to_one;
    config.max_matching = max_matching;
    for (layer, r) in &radii {
        for rule in config.rules.iter_mut().filter(|x| x.layer == *layer) {
            rule.radius = *r;
        }
    }
    let rules = config.rules.clone();
    let mut station = Station::new(config);
    let mut runs: Vec<Run> = Vec::new();
    for (r, f) in files.iter().enumerate() {
        station.add_robot(
            r as u8,
            transforms.get(&(r as u8)).copied().unwrap_or_default(),
        );
        let file = File::open(f).unwrap_or_else(|e| panic!("{f}: {e}"));
        let input: Box<dyn Read> = if f.ends_with(".gz") {
            Box::new(flate2::read::MultiGzDecoder::new(file))
        } else {
            Box::new(file)
        };
        let mut run = Run {
            lines: BufReader::new(input).lines(),
            next: None,
            deltas: 0,
        };
        run.advance();
        runs.push(run);
    }

    let start = Instant::now();
    let (mut total_ms, mut max_ms, mut count) = (0.0f64, 0.0f64, 0usize);
    // the robot whose next delta is earliest, until every run is done
    while let Some(r) = (0..runs.len())
        .filter(|r| runs[*r].next.is_some())
        .min_by_key(|r| runs[*r].next.as_ref().unwrap()["t"].as_u64().unwrap_or(0))
    {
        let delta = runs[r].next.take().unwrap();
        let t = Instant::now();
        station.receive(r as u8, &delta).unwrap();
        let ms = t.elapsed().as_secs_f64() * 1e3;
        total_ms += ms;
        max_ms = max_ms.max(ms);
        count += 1;
        runs[r].deltas += 1;
        runs[r].advance();
        if align && r > 0 && runs[r].deltas.is_multiple_of(50) {
            try_align(&mut station, r as u8);
        }
        if count % 200 == 0 {
            println!(
                "{count} deltas: station nodes={} edges={} merged={}",
                station.num_nodes(),
                station.num_edges(),
                station.merges.len()
            );
        }
    }

    if align {
        for r in 1..runs.len() {
            try_align(&mut station, r as u8);
        }
    }
    for (r, run) in runs.iter().enumerate() {
        println!("robot {r}: {} deltas from {}", run.deltas, files[r]);
    }
    if align {
        for r in 1..runs.len() {
            let t = station.transform_of(r as u8).unwrap();
            println!(
                "robot {r} frame: yaw {:.4} rad, translation [{:.3}, {:.3}, {:.3}] m",
                t.yaw, t.translation[0], t.translation[1], t.translation[2]
            );
        }
    }
    println!(
        "station: nodes={} edges={} merged={}",
        station.num_nodes(),
        station.num_edges(),
        station.merges.len()
    );
    for rule in &rules {
        let merged = station
            .merges
            .keys()
            .filter(|id| station.layer_of_robot_node(**id) == Some(rule.layer))
            .count();
        println!(
            "  layer {:?} (radius {} m): {merged} nodes merged into another robot's",
            rule.layer, rule.radius
        );
    }
    let per_robot: BTreeMap<u8, usize> =
        station.merges.keys().fold(BTreeMap::new(), |mut m, id| {
            *m.entry(robot_of(*id)).or_default() += 1;
            m
        });
    println!("  merged away per robot: {per_robot:?}");
    println!(
        "station update: {:.1} ms per delta (max {:.1} ms), {count} deltas in {:.1} s",
        total_ms / count.max(1) as f64,
        max_ms,
        start.elapsed().as_secs_f64()
    );
    // How far each later robot's node is from the nearest earlier robot's node, per layer.
    for rule in &rules {
        let nodes = station.robot_nodes(rule.layer);
        let mut d: Vec<f64> = nodes
            .iter()
            .filter(|(id, _)| robot_of(*id) > 0)
            .filter_map(|(id, p)| {
                nodes
                    .iter()
                    .filter(|(o, _)| robot_of(*o) < robot_of(*id))
                    .map(|(_, q)| {
                        ((p[0] - q[0]).powi(2) + (p[1] - q[1]).powi(2) + (p[2] - q[2]).powi(2))
                            .sqrt()
                    })
                    .min_by(|a, b| a.total_cmp(b))
            })
            .collect();
        if d.is_empty() {
            continue;
        }
        d.sort_by(|a, b| a.total_cmp(b));
        let pct = |q: f64| d[((d.len() - 1) as f64 * q) as usize];
        let within = |r: f64| 100.0 * d.iter().filter(|x| **x <= r).count() as f64 / d.len() as f64;
        println!(
            "  layer {:?}: nearest earlier-robot node, median {:.2} m, 90% {:.2} m; \
             within 0.3/0.5/0.75/1.0 m: {:.0}/{:.0}/{:.0}/{:.0}%",
            rule.layer,
            pct(0.5),
            pct(0.9),
            within(0.3),
            within(0.5),
            within(0.75),
            within(1.0)
        );
    }
    // Merges that share a target with another node of the same robot: two of one robot's own
    // nodes taken as one, a sign the radius is too large for that layer.
    for rule in &rules {
        let mut per_target: BTreeMap<(u64, u8), usize> = BTreeMap::new();
        for (id, target) in &station.merges {
            if station.layer_of_robot_node(*id) == Some(rule.layer) {
                *per_target.entry((*target, robot_of(*id))).or_default() += 1;
            }
        }
        let crowded: usize = per_target.values().filter(|n| **n > 1).map(|n| n - 1).sum();
        println!(
            "  layer {:?}: {crowded} merges onto a node another node of the same robot also merged onto",
            rule.layer
        );
    }

    // How many nodes each later robot has per layer, and the most of them any one-to-one
    // matching within the radius could merge (each earlier-robot node used once per robot).
    for rule in &rules {
        let nodes = station.robot_nodes(rule.layer);
        let robots: std::collections::BTreeSet<u8> =
            nodes.iter().map(|(id, _)| robot_of(*id)).collect();
        let mut line = Vec::new();
        for r in robots.iter().filter(|r| **r > 0) {
            let mine: Vec<[f64; 3]> = nodes
                .iter()
                .filter(|(id, _)| robot_of(*id) == *r)
                .map(|(_, p)| *p)
                .collect();
            let theirs: Vec<[f64; 3]> = nodes
                .iter()
                .filter(|(id, _)| robot_of(*id) < *r)
                .map(|(_, p)| *p)
                .collect();
            line.push(format!(
                "robot {r}: {} nodes, at most {} mergeable",
                mine.len(),
                matching_ceiling(&mine, &theirs, rule.radius)
            ));
        }
        println!("  layer {:?}: {}", rule.layer, line.join("; "));
    }

    if deform {
        report_deform(&station);
        // then match again on corrected positions, a few rounds (each correction uses the
        // merges the previous round found)
        let count = |st: &Station, layer| {
            st.merges
                .keys()
                .filter(|id| st.layer_of_robot_node(**id) == Some(layer))
                .count()
        };
        for round in 1..=3 {
            let t = Instant::now();
            station.correct_drift(&[station::OBJECTS, station::PLACES, station::ROOMS]);
            println!(
                "  re-merge round {round} ({:.0} ms): objects {}, places {}, rooms {} merged",
                t.elapsed().as_secs_f64() * 1e3,
                count(&station, station::OBJECTS),
                count(&station, station::PLACES),
                count(&station, station::ROOMS)
            );
        }
        let (_, reference) = station.merged_graph();
        println!(
            "  incremental merges equal the full rebuild after correction: {}",
            reference == station.merges
        );
    }
    let mut graph = if deform {
        station.export_corrected(&[station::OBJECTS, station::PLACES, station::ROOMS])
    } else {
        station.export()
    };
    let rooms = station::merge_rooms_by_places(&mut graph);
    println!("  rooms: {rooms} more rooms merged because they share places");
    if let Some(path) = out {
        std::fs::write(&path, graph.to_string()).unwrap();
        println!("wrote {path}");
    }
}

/// Maximum one-to-one matching between `a` and `b` where pairs are within `radius` (Kuhn's
/// augmenting paths, iterative).
fn matching_ceiling(a: &[[f64; 3]], b: &[[f64; 3]], radius: f64) -> usize {
    let cell = radius.max(1e-6);
    let key = |p: &[f64; 3]| {
        (
            (p[0] / cell).floor() as i64,
            (p[1] / cell).floor() as i64,
            (p[2] / cell).floor() as i64,
        )
    };
    let mut grid: std::collections::HashMap<(i64, i64, i64), Vec<usize>> = Default::default();
    for (j, q) in b.iter().enumerate() {
        grid.entry(key(q)).or_default().push(j);
    }
    let adj: Vec<Vec<usize>> = a
        .iter()
        .map(|p| {
            let (x, y, z) = key(p);
            let mut out = Vec::new();
            for dx in -1..=1 {
                for dy in -1..=1 {
                    for dz in -1..=1 {
                        for &j in grid.get(&(x + dx, y + dy, z + dz)).into_iter().flatten() {
                            let q = b[j];
                            let d = ((p[0] - q[0]).powi(2)
                                + (p[1] - q[1]).powi(2)
                                + (p[2] - q[2]).powi(2))
                            .sqrt();
                            if d <= radius {
                                out.push(j);
                            }
                        }
                    }
                }
            }
            out
        })
        .collect();
    let mut owner: Vec<Option<usize>> = vec![None; b.len()];
    let mut count = 0;
    for start in 0..a.len() {
        let mut seen = vec![false; b.len()];
        // stack of (left node, next neighbour index); path of (left, right) choices
        let mut stack: Vec<(usize, usize)> = vec![(start, 0)];
        let mut via: Vec<usize> = Vec::new();
        let mut found = false;
        while let Some(&mut (u, ref mut k)) = stack.last_mut() {
            if *k >= adj[u].len() {
                stack.pop();
                via.pop();
                continue;
            }
            let j = adj[u][*k];
            *k += 1;
            if seen[j] {
                continue;
            }
            seen[j] = true;
            via.push(j);
            match owner[j] {
                None => {
                    found = true;
                    break;
                }
                Some(v) => stack.push((v, 0)),
            }
        }
        if found {
            // flip the path: stack[i].0 takes via[i]
            for (i, (u, _)) in stack.iter().enumerate() {
                owner[via[i]] = Some(*u);
            }
            count += 1;
        }
    }
    count
}

/// Estimate robot `r`'s frame from objects and switch to it when it fits clearly more objects
/// than the current one.
fn try_align(station: &mut Station, r: u8) {
    const RADIUS: f64 = 0.3;
    const MIN_INLIERS: usize = 6;
    let (mine, theirs) = station.alignment_points(r);
    let t0 = Instant::now();
    let Some((t, n)) = station::align::estimate(&mine, &theirs, RADIUS) else {
        return;
    };
    let current = station::align::score(&mine, &theirs, &station.transform_of(r).unwrap(), RADIUS);
    if n >= MIN_INLIERS && n > current + current / 10 {
        station.set_transform(r, t);
        println!(
            "align robot {r}: yaw {:.4} rad, translation [{:.3}, {:.3}, {:.3}] m, {n} of {} objects and described places fit (was {current}), {:.0} ms",
            t.yaw,
            t.translation[0],
            t.translation[1],
            t.translation[2],
            mine.len(),
            t0.elapsed().as_secs_f64() * 1e3
        );
    }
}

/// Drift correction, checked on data it did not use: bend the maps with object merges only, then
/// see how much closer each later robot's places sit to an earlier robot's places, and how many
/// more could merge.
fn report_deform(station: &Station) {
    let rule = station
        .config
        .rules
        .iter()
        .find(|r| r.layer == station::PLACES)
        .cloned();
    let Some(rule) = rule else { return };
    let before: std::collections::HashMap<u64, [f64; 3]> =
        station.robot_nodes(station::PLACES).into_iter().collect();
    let t = Instant::now();
    let after = station.corrected_positions(&[station::OBJECTS]);
    let ms = t.elapsed().as_secs_f64() * 1e3;
    let summary = |pos: &dyn Fn(u64) -> [f64; 3]| {
        let ids: Vec<u64> = before.keys().copied().collect();
        let mut d = Vec::new();
        let mut ceiling = 0;
        let robots: std::collections::BTreeSet<u8> = ids.iter().map(|id| robot_of(*id)).collect();
        for r in robots.iter().filter(|r| **r > 0) {
            let mine: Vec<[f64; 3]> = ids
                .iter()
                .filter(|id| robot_of(**id) == *r)
                .map(|id| pos(*id))
                .collect();
            let theirs: Vec<[f64; 3]> = ids
                .iter()
                .filter(|id| robot_of(**id) < *r)
                .map(|id| pos(*id))
                .collect();
            for p in &mine {
                let m = theirs
                    .iter()
                    .map(|q| {
                        ((p[0] - q[0]).powi(2) + (p[1] - q[1]).powi(2) + (p[2] - q[2]).powi(2))
                            .sqrt()
                    })
                    .fold(f64::INFINITY, f64::min);
                if m.is_finite() {
                    d.push(m);
                }
            }
            ceiling += matching_ceiling(&mine, &theirs, rule.radius);
        }
        d.sort_by(|a, b| a.total_cmp(b));
        let med = if d.is_empty() { 0.0 } else { d[d.len() / 2] };
        let within =
            |x: f64| 100.0 * d.iter().filter(|v| **v <= x).count() as f64 / d.len().max(1) as f64;
        format!(
            "median {med:.2} m, within 0.3/0.5 m {:.0}/{:.0}%, at most {ceiling} mergeable",
            within(0.3),
            within(0.5)
        )
    };
    println!(
        "drift correction from object merges only ({ms:.0} ms), later robots' places to earlier robots':"
    );
    println!("  before: {}", summary(&|id| before[&id]));
    println!("  after:  {}", summary(&|id| after[&id]));
    let all = station.corrected_positions(&[station::OBJECTS, station::PLACES, station::ROOMS]);
    println!(
        "  with every merge as a loop closure: {}",
        summary(&|id| all[&id])
    );
}
