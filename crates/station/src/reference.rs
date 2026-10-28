//! The station graph computed from scratch, the reference the incremental station is checked
//! against. Slow, and deliberately simple.

use std::collections::BTreeMap;

use serde_json::{Value, json};

use crate::Station;
use crate::frame::{global_id, robot_of};
use crate::matching::assign;
use crate::node::{EdgeKey, add, distance, edge_info, edge_key, position};

impl Station {
    /// The station graph recomputed from every robot's whole graph, and the merges it implies.
    /// [`Station::export`] and [`Station::merges`] must always equal these.
    pub fn merged_graph(&self) -> (Value, BTreeMap<u64, u64>) {
        // every robot's graph in the station frame, under global ids
        let mut nodes: BTreeMap<u64, Value> = BTreeMap::new();
        let mut edges: BTreeMap<EdgeKey, Value> = BTreeMap::new();
        for (&r, robot) in &self.robots {
            let g = robot.decoder.graph();
            for n in g["nodes"].as_array().into_iter().flatten() {
                let mut n = n.clone();
                let id = global_id(r, n["id"].as_u64().unwrap());
                n["id"] = json!(id);
                if let Some(p) = position(&n) {
                    n["attributes"]["position"] = json!(robot.transform.apply(p));
                }
                nodes.insert(id, n);
            }
            for e in g["edges"].as_array().into_iter().flatten() {
                let s = global_id(r, e["source"].as_u64().unwrap());
                let t = global_id(r, e["target"].as_u64().unwrap());
                edges.insert(edge_key(s, t), edge_info(e));
            }
        }

        // robot by robot, match against the nodes kept so far
        let mut merges = BTreeMap::new();
        let mut kept = Vec::new();
        for &robot in self.robots.keys() {
            let mut pairs = Vec::new();
            let mut mine = Vec::new();
            for (&id, n) in nodes.iter().filter(|(id, _)| robot_of(**id) == robot) {
                let Some(group) = self.group_of(n) else {
                    continue;
                };
                let radius = self.config.rules[group.rule].radius;
                let p = add(position(n).unwrap(), self.offset(id));
                for (other, other_group, q) in &kept {
                    let d = distance(p, *q);
                    if *other_group == group && d <= radius {
                        pairs.push((d, id, *other));
                    }
                }
                mine.push((id, group, p));
            }
            // targets of different groups never overlap, so one assignment covers them all
            let matched = assign(pairs, self.config.one_to_one, self.config.max_matching);
            for (id, group, p) in mine {
                match matched.get(&id) {
                    Some(&to) => {
                        merges.insert(id, to);
                    }
                    None => kept.push((id, group, p)),
                }
            }
        }

        // edges follow merged nodes; a station edge keeps its smallest robot edge's info
        let resolve = |id: u64| merges.get(&id).copied().unwrap_or(id);
        let mut out_edges: BTreeMap<EdgeKey, Value> = BTreeMap::new();
        for (key, info) in &edges {
            let (s, t) = (resolve(key.0), resolve(key.1));
            if s != t && nodes.contains_key(&s) && nodes.contains_key(&t) {
                out_edges
                    .entry(edge_key(s, t))
                    .or_insert_with(|| info.clone());
            }
        }
        for from in merges.keys() {
            nodes.remove(from);
        }
        let mut g = self.header();
        g["nodes"] = json!(nodes.into_values().collect::<Vec<_>>());
        g["edges"] = json!(
            out_edges
                .into_iter()
                .map(|(k, info)| json!({"source": k.0, "target": k.1, "info": info}))
                .collect::<Vec<_>>()
        );
        (g, merges)
    }
}
