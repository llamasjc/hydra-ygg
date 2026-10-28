//! Drift correction across robots: bend each later robot's map onto the maps it overlaps.
//!
//! One rigid transform per robot cannot undo odometry drift inside a robot's map. Every merge is
//! a loop closure between two robots: the merged node should sit where the node it merged into
//! sits. Each node of a later robot gets an offset; offsets are pulled towards those loop
//! closures and kept smooth along the robot's own edges (places to places, places to objects,
//! ...), as in a deformation graph. Robots are solved in order, so robot 2 bends onto the
//! already corrected maps of robots 0 and 1. Robot 0 is the reference and does not move.

use std::collections::{BTreeMap, HashMap};

/// Weight of a loop closure against one edge of smoothness.
pub const CLOSURE_WEIGHT: f64 = 10.0;
/// Gauss-Seidel sweeps; offsets spread one edge per sweep, so this bounds the reach.
pub const SWEEPS: usize = 2000;

/// Offsets for the nodes of one robot.
///
/// `nodes` are the robot's nodes, `edges` its own edges between them, `closures` node ->
/// where it should be moved by (target minus current position). Nodes with no path to a
/// closure keep a zero offset.
pub fn offsets(
    nodes: &[u64],
    edges: &[(u64, u64)],
    closures: &HashMap<u64, [f64; 3]>,
    sweeps: usize,
) -> HashMap<u64, [f64; 3]> {
    let index: HashMap<u64, usize> = nodes.iter().enumerate().map(|(i, n)| (*n, i)).collect();
    let mut adj: Vec<Vec<usize>> = vec![Vec::new(); nodes.len()];
    for (a, b) in edges {
        if let (Some(&i), Some(&j)) = (index.get(a), index.get(b)) {
            if i != j {
                adj[i].push(j);
                adj[j].push(i);
            }
        }
    }
    let target: Vec<Option<[f64; 3]>> = nodes.iter().map(|n| closures.get(n).copied()).collect();
    // only nodes connected to a closure move
    let mut reached = vec![false; nodes.len()];
    let mut stack: Vec<usize> = (0..nodes.len()).filter(|i| target[*i].is_some()).collect();
    for i in &stack {
        reached[*i] = true;
    }
    while let Some(i) = stack.pop() {
        for &j in &adj[i] {
            if !reached[j] {
                reached[j] = true;
                stack.push(j);
            }
        }
    }
    let mut x = vec![[0.0; 3]; nodes.len()];
    for _ in 0..sweeps {
        let mut moved = 0.0f64;
        for i in 0..nodes.len() {
            if !reached[i] {
                continue;
            }
            let mut sum = [0.0; 3];
            let mut w = adj[i].len() as f64;
            for &j in &adj[i] {
                for k in 0..3 {
                    sum[k] += x[j][k];
                }
            }
            if let Some(t) = target[i] {
                for k in 0..3 {
                    sum[k] += CLOSURE_WEIGHT * t[k];
                }
                w += CLOSURE_WEIGHT;
            }
            if w == 0.0 {
                continue;
            }
            for k in 0..3 {
                let v = sum[k] / w;
                moved = moved.max((v - x[i][k]).abs());
                x[i][k] = v;
            }
        }
        if moved < 1e-5 {
            break;
        }
    }
    nodes
        .iter()
        .zip(x)
        .zip(reached)
        .filter(|(_, r)| *r)
        .map(|((n, o), _)| (*n, o))
        .collect()
}

/// Corrected positions for every node, robot by robot (see the module docs).
///
/// `positions` are every robot node in the station frame, `edges` every robot edge,
/// `merges` merged node -> node it merged into, `robot_of` the robot of a node and
/// `use_closure` which merges count as loop closures.
pub fn correct(
    positions: &HashMap<u64, [f64; 3]>,
    edges: &[(u64, u64)],
    merges: &BTreeMap<u64, u64>,
    robot_of: impl Fn(u64) -> u8,
    use_closure: impl Fn(u64) -> bool,
    sweeps: usize,
) -> HashMap<u64, [f64; 3]> {
    let mut out = positions.clone();
    let mut by_robot: BTreeMap<u8, Vec<u64>> = BTreeMap::new();
    for id in positions.keys() {
        by_robot.entry(robot_of(*id)).or_default().push(*id);
    }
    for (r, mut nodes) in by_robot {
        if r == 0 {
            continue;
        }
        nodes.sort();
        let mine: Vec<(u64, u64)> = edges
            .iter()
            .filter(|(a, b)| robot_of(*a) == r && robot_of(*b) == r)
            .copied()
            .collect();
        let mut closures = HashMap::new();
        for (from, to) in merges {
            if robot_of(*from) != r || !use_closure(*from) {
                continue;
            }
            let (Some(p), Some(q)) = (positions.get(from), out.get(to)) else {
                continue;
            };
            closures.insert(*from, [q[0] - p[0], q[1] - p[1], q[2] - p[2]]);
        }
        for (id, o) in offsets(&nodes, &mine, &closures, sweeps) {
            let p = positions[&id];
            out.insert(id, [p[0] + o[0], p[1] + o[1], p[2] + o[2]]);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bends_a_drifting_corridor_back() {
        // robot 0: a straight corridor of places at y=0. Robot 1 walks it too, but its map
        // drifts sideways by 5 cm per metre. Its first and last places are loop closures; the
        // ones in between must follow.
        let r1 = 1u64 << 48;
        let mut positions = HashMap::new();
        let mut merges = BTreeMap::new();
        let mut edges = Vec::new();
        for i in 0..21u64 {
            positions.insert(i, [i as f64, 0.0, 0.0]);
            positions.insert(r1 | i, [i as f64, 0.05 * i as f64, 0.0]);
            if i > 0 {
                edges.push((r1 | (i - 1), r1 | i));
            }
        }
        merges.insert(r1, 0);
        merges.insert(r1 | 20, 20);
        let out = correct(&positions, &edges, &merges, |id| (id >> 48) as u8, |_| true, SWEEPS);
        let worst = (0..21u64).map(|i| out[&(r1 | i)][1].abs()).fold(0.0, f64::max);
        assert!(worst < 0.01, "worst sideways error {worst} m (was 1.0 m)");
        assert_eq!(out[&5], [5.0, 0.0, 0.0]);
    }
}
