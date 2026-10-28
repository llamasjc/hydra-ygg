//! The live station: robots stream their deltas over TCP while they map.
//!
//! Robot r connects to port `base + r` (its Hydra runs with
//! HYDRA_YGG_RECORD=tcp://<station host>:<base + r>). Deltas are applied as they arrive, in any
//! order between robots. With --align each robot's frame is found from its objects, as in
//! replay --align. Every --every seconds the station graph is written to --out, and with
//! --dsg-out also converted for Hydra's static visualizer (station_to_dsg from PATH, or through
//! `ros2 run`, so source the ROS workspace). --deform writes the graph with drift between robots
//! corrected (see station::deform). --rooms detects rooms again on the merged places in the
//! --dsg-out file (Hydra's RoomFinder, through station_to_dsg).
//!
//! usage: station_server --robots N [--port 7447] [--align] [--deform] [--rooms] [--max-matching]
//!                       [--out station.json] [--dsg-out station_dsg.json] [--every 5]

use std::collections::BTreeMap;
use std::io::{BufRead, BufReader};
use std::net::TcpListener;
use std::sync::mpsc;
use std::time::{Duration, Instant};

use serde_json::Value;
use station::{Config, Station, robot_of};

enum Event {
    Delta(u8, Value),
    Connected(u8, String),
    Closed(u8),
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let value = |name: &str| {
        args.iter()
            .position(|a| a == name)
            .map(|i| args[i + 1].clone())
    };
    let flag = |name: &str| args.iter().any(|a| a == name);
    let Some(robots) = value("--robots").map(|v| v.parse::<u8>().unwrap()) else {
        eprintln!(
            "usage: station_server --robots N [--port 7447] [--align] [--deform] [--rooms] [--max-matching] [--out station.json] [--dsg-out station_dsg.json] [--every 5]"
        );
        std::process::exit(2);
    };
    let port: u16 = value("--port").map_or(7447, |v| v.parse().unwrap());
    let every = Duration::from_secs_f64(value("--every").map_or(5.0, |v| v.parse().unwrap()));
    let out = value("--out");
    let dsg_out = value("--dsg-out");
    let align = flag("--align");
    let deform = flag("--deform");
    let rooms = flag("--rooms");

    let config = Config {
        max_matching: flag("--max-matching"),
        ..Default::default()
    };
    let rules = config.rules.clone();
    let mut station = Station::new(config);

    let (tx, rx) = mpsc::channel::<Event>();
    for r in 0..robots {
        station.add_robot(r, Default::default());
        let listener = TcpListener::bind(("0.0.0.0", port + r as u16))
            .unwrap_or_else(|e| panic!("port {}: {e}", port + r as u16));
        println!("robot {r}: waiting on port {}", port + r as u16);
        let tx = tx.clone();
        std::thread::spawn(move || {
            // one connection per robot run; a reconnect continues the same robot
            for stream in listener.incoming().flatten() {
                let peer = stream
                    .peer_addr()
                    .map(|a| a.to_string())
                    .unwrap_or_default();
                tx.send(Event::Connected(r, peer)).ok();
                for line in BufReader::new(stream).lines() {
                    let Ok(line) = line else { break };
                    match serde_json::from_str(&line) {
                        Ok(v) => {
                            tx.send(Event::Delta(r, v)).ok();
                        }
                        Err(e) => eprintln!("robot {r}: bad delta: {e}"),
                    }
                }
                tx.send(Event::Closed(r)).ok();
            }
        });
    }
    drop(tx);

    let mut deltas: BTreeMap<u8, usize> = BTreeMap::new();
    let mut last_write = Instant::now();
    let (mut total_ms, mut count) = (0.0f64, 0usize);
    loop {
        match rx.recv_timeout(Duration::from_millis(200)) {
            Ok(Event::Connected(r, peer)) => println!("robot {r}: connected from {peer}"),
            Ok(Event::Closed(r)) => println!("robot {r}: disconnected"),
            Ok(Event::Delta(r, delta)) => {
                let t = Instant::now();
                if let Err(e) = station.receive(r, &delta) {
                    eprintln!("robot {r}: {e}");
                    continue;
                }
                total_ms += t.elapsed().as_secs_f64() * 1e3;
                count += 1;
                let n = deltas.entry(r).or_default();
                *n += 1;
                if align && r > 0 && (*n).is_multiple_of(50) {
                    try_align(&mut station, r);
                }
            }
            Err(mpsc::RecvTimeoutError::Timeout) => {}
            Err(mpsc::RecvTimeoutError::Disconnected) => break,
        }
        if last_write.elapsed() >= every && count > 0 {
            last_write = Instant::now();
            report(&station, &deltas, &rules, total_ms / count as f64);
            if deform {
                station.correct_drift(&[station::OBJECTS, station::PLACES, station::ROOMS]);
            }
            if let Some(path) = &out {
                write(&station, path, dsg_out.as_deref(), deform, rooms);
            }
        }
    }
}

fn report(station: &Station, deltas: &BTreeMap<u8, usize>, rules: &[station::MergeRule], ms: f64) {
    let merged: Vec<String> = rules
        .iter()
        .map(|rule| {
            let n = station
                .merges
                .keys()
                .filter(|id| station.layer_of_robot_node(**id) == Some(rule.layer))
                .count();
            format!("{:?} {n}", rule.layer)
        })
        .collect();
    let per_robot: BTreeMap<u8, usize> =
        station.merges.keys().fold(BTreeMap::new(), |mut m, id| {
            *m.entry(robot_of(*id)).or_default() += 1;
            m
        });
    println!(
        "deltas {deltas:?}: station nodes={} edges={}, merged {} (per robot {per_robot:?}), {ms:.1} ms per delta",
        station.num_nodes(),
        station.num_edges(),
        merged.join(", ")
    );
}

/// Write the station graph (through a temporary file, so readers never see half of it), and
/// convert it for the visualizer.
fn write(station: &Station, path: &str, dsg: Option<&str>, deform: bool, rooms: bool) {
    let tmp = format!("{path}.tmp");
    let mut graph = if deform {
        station.export_corrected(&[station::OBJECTS, station::PLACES, station::ROOMS])
    } else {
        station.export()
    };
    station::merge_rooms_by_places(&mut graph);
    if let Err(e) =
        std::fs::write(&tmp, graph.to_string()).and_then(|_| std::fs::rename(&tmp, path))
    {
        eprintln!("{path}: {e}");
        return;
    }
    if let Some(dsg) = dsg {
        let tmp = format!("{dsg}.tmp.json");
        // station_to_dsg on PATH, else through ros2 run (an installed ROS package is not on PATH)
        let run = |cmd: &str, pre: &[&str]| {
            std::process::Command::new(cmd)
                .args(pre)
                .args([path, tmp.as_str()])
                .args(rooms.then_some("--rooms"))
                .stdout(std::process::Stdio::null())
                .status()
        };
        let ok = match run("station_to_dsg", &[]) {
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
                run("ros2", &["run", "hydra_ygg_sink", "station_to_dsg"])
            }
            r => r,
        }
        .is_ok_and(|s| s.success());
        if !ok || std::fs::rename(&tmp, dsg).is_err() {
            eprintln!("station_to_dsg failed (is the ROS workspace sourced?)");
        }
    }
}

fn try_align(station: &mut Station, r: u8) {
    const RADIUS: f64 = 0.3;
    const MIN_INLIERS: usize = 6;
    let (mine, theirs) = station.alignment_points(r);
    let Some((t, n)) = station::align::estimate(&mine, &theirs, RADIUS) else {
        return;
    };
    let current = station::align::score(&mine, &theirs, &station.transform_of(r).unwrap(), RADIUS);
    if n >= MIN_INLIERS && n > current + current / 10 {
        station.set_transform(r, t);
        println!(
            "align robot {r}: yaw {:.4} rad, translation [{:.3}, {:.3}, {:.3}] m, {n} of {} objects and described places fit (was {current})",
            t.yaw,
            t.translation[0],
            t.translation[1],
            t.translation[2],
            mine.len()
        );
    }
}
