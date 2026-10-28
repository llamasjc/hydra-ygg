//! Joining rooms that different robots found for the same space.

use std::collections::{BTreeMap, BTreeSet};

use serde_json::{Value, json};

use crate::config::{PLACES, ROOMS};
use crate::frame::robot_of;
use crate::node::{edge_key, layer_key, position};

/// Rooms of different robots that share places (after the place merge) are one room: a room
/// joins another when at least half of the smaller room's places are shared. The joined room
/// keeps the lowest id, takes the other rooms' edges, and sits at the mean of its places.
/// Returns how many rooms were joined away.
pub fn merge_rooms_by_places(graph: &mut Value) -> usize {
    let layer: BTreeMap<u64, _> = graph["nodes"]
        .as_array()
        .unwrap()
        .iter()
        .map(|n| (n["id"].as_u64().unwrap(), layer_key(n)))
        .collect();
    let mut places_of: BTreeMap<u64, BTreeSet<u64>> = BTreeMap::new();
    for e in graph["edges"].as_array().unwrap() {
        let (s, t) = (e["source"].as_u64().unwrap(), e["target"].as_u64().unwrap());
        for (room, place) in [(s, t), (t, s)] {
            if layer.get(&room) == Some(&ROOMS) && layer.get(&place) == Some(&PLACES) {
                places_of.entry(room).or_default().insert(place);
            }
        }
    }

    // union-find over rooms; a set's root is its lowest id
    let rooms: Vec<u64> = places_of.keys().copied().collect();
    let mut parent: BTreeMap<u64, u64> = rooms.iter().map(|r| (*r, *r)).collect();
    fn find(parent: &mut BTreeMap<u64, u64>, x: u64) -> u64 {
        let p = parent[&x];
        if p == x {
            return x;
        }
        let root = find(parent, p);
        parent.insert(x, root);
        root
    }
    for (i, a) in rooms.iter().enumerate() {
        for b in &rooms[i + 1..] {
            if robot_of(*a) == robot_of(*b) {
                continue;
            }
            let (pa, pb) = (&places_of[a], &places_of[b]);
            let shared = pa.intersection(pb).count();
            if shared > 0 && 2 * shared >= pa.len().min(pb.len()) {
                let (ra, rb) = (find(&mut parent, *a), find(&mut parent, *b));
                parent.insert(ra.max(rb), ra.min(rb));
            }
        }
    }
    let into: BTreeMap<u64, u64> = rooms
        .iter()
        .map(|r| (*r, find(&mut parent, *r)))
        .filter(|(r, root)| r != root)
        .collect();
    if into.is_empty() {
        return 0;
    }

    // each joined room moves to the mean of all the places it now holds
    let mut members: BTreeMap<u64, BTreeSet<u64>> = BTreeMap::new();
    for r in &rooms {
        let root = into.get(r).copied().unwrap_or(*r);
        members.entry(root).or_default().extend(&places_of[r]);
    }
    let pos: BTreeMap<u64, [f64; 3]> = graph["nodes"]
        .as_array()
        .unwrap()
        .iter()
        .filter_map(|n| Some((n["id"].as_u64()?, position(n)?)))
        .collect();
    let roots: BTreeSet<u64> = into.values().copied().collect();
    let nodes = graph["nodes"].as_array_mut().unwrap();
    nodes.retain(|n| !into.contains_key(&n["id"].as_u64().unwrap()));
    for n in nodes.iter_mut() {
        let id = n["id"].as_u64().unwrap();
        if !roots.contains(&id) {
            continue;
        }
        let ps: Vec<[f64; 3]> = members[&id]
            .iter()
            .filter_map(|p| pos.get(p))
            .copied()
            .collect();
        if !ps.is_empty() {
            let k = ps.len() as f64;
            let mean: [f64; 3] = std::array::from_fn(|i| ps.iter().map(|p| p[i]).sum::<f64>() / k);
            n["attributes"]["position"] = json!(mean);
        }
    }

    // edges follow their rooms; drop self-loops and duplicates
    let mut seen = BTreeSet::new();
    let edges = graph["edges"].as_array_mut().unwrap();
    edges.retain_mut(|e| {
        for end in ["source", "target"] {
            if let Some(r) = into.get(&e[end].as_u64().unwrap()) {
                e[end] = json!(r);
            }
        }
        let (s, t) = (e["source"].as_u64().unwrap(), e["target"].as_u64().unwrap());
        s != t && seen.insert(edge_key(s, t))
    });
    into.len()
}
