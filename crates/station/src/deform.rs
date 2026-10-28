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
/// A loop closure that disagrees with its neighbours by more than this (m) is an outlier.
pub const INLIER_THRESHOLD: f64 = 0.5;
/// Graduated non-convexity rounds (Yang et al. 2020, truncated least squares).
pub const GNC_ROUNDS: usize = 8;

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
        if let (Some(&i), Some(&j)) = (index.get(a), index.get(b))
            && i != j
        {
            adj[i].push(j);
            adj[j].push(i);
        }
    }
    let target: Vec<Option<[f64; 3]>> = nodes.iter().map(|n| closures.get(n).copied()).collect();
    let mut weight: Vec<f64> = vec![1.0; nodes.len()];
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
    // Robust: closures are weighted by graduated non-convexity with a truncated least squares
    // cost. Each round solves, measures every closure against what its neighbours say the offset
    // there is, and lowers the weight of those that disagree; wrong merges end with weight 0.
    let mut mu = f64::INFINITY;
    for round in 0..=GNC_ROUNDS {
        solve(&adj, &target, &weight, &reached, &mut x, sweeps);
        if round == GNC_ROUNDS {
            break;
        }
        let residual: Vec<f64> = (0..nodes.len())
            .map(|i| match target[i] {
                Some(t) if !adj[i].is_empty() => {
                    let n = adj[i].len() as f64;
                    let mut m = [0.0; 3];
                    for &j in &adj[i] {
                        for k in 0..3 {
                            m[k] += x[j][k] / n;
                        }
                    }
                    ((t[0] - m[0]).powi(2) + (t[1] - m[1]).powi(2) + (t[2] - m[2]).powi(2)).sqrt()
                }
                _ => 0.0,
            })
            .collect();
        let c2 = INLIER_THRESHOLD * INLIER_THRESHOLD;
        if mu.is_infinite() {
            let r_max = residual.iter().copied().fold(0.0, f64::max);
            if r_max * r_max <= c2 {
                break; // every closure agrees
            }
            mu = c2 / (2.0 * r_max * r_max - c2).max(1e-9);
        } else {
            mu *= 1.4;
        }
        for i in 0..nodes.len() {
            if target[i].is_none() {
                continue;
            }
            let r2 = residual[i] * residual[i];
            weight[i] = if r2 >= (mu + 1.0) / mu * c2 {
                0.0
            } else if r2 <= mu / (mu + 1.0) * c2 {
                1.0
            } else {
                (INLIER_THRESHOLD / residual[i] * (mu * (mu + 1.0)).sqrt() - mu).clamp(0.0, 1.0)
            };
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

/// Gauss-Seidel on the offsets: each reached node takes the weighted mean of its neighbours and
/// its loop closure.
fn solve(
    adj: &[Vec<usize>],
    target: &[Option<[f64; 3]>],
    weight: &[f64],
    reached: &[bool],
    x: &mut [[f64; 3]],
    sweeps: usize,
) {
    for _ in 0..sweeps {
        let mut moved = 0.0f64;
        for i in 0..x.len() {
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
                let cw = CLOSURE_WEIGHT * weight[i];
                for k in 0..3 {
                    sum[k] += cw * t[k];
                }
                w += cw;
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
        let out = correct(
            &positions,
            &edges,
            &merges,
            |id| (id >> 48) as u8,
            |_| true,
            SWEEPS,
        );
        let worst = (0..21u64)
            .map(|i| out[&(r1 | i)][1].abs())
            .fold(0.0, f64::max);
        assert!(worst < 0.01, "worst sideways error {worst} m (was 1.0 m)");
        assert_eq!(out[&5], [5.0, 0.0, 0.0]);
    }

    #[test]
    fn ignores_a_wrong_loop_closure() {
        // robot 1 agrees with robot 0 along a corridor, but one place merged 3 m off
        let r1 = 1u64 << 48;
        let mut positions = HashMap::new();
        let mut merges = BTreeMap::new();
        let mut edges = Vec::new();
        for i in 0..21u64 {
            positions.insert(i, [i as f64, 0.0, 0.0]);
            positions.insert(r1 | i, [i as f64, 0.0, 0.0]);
            if i > 0 {
                edges.push((r1 | (i - 1), r1 | i));
            }
            if i % 2 == 0 && i != 10 {
                merges.insert(r1 | i, i);
            }
        }
        positions.insert(100, [10.0, 3.0, 0.0]);
        merges.insert(r1 | 10, 100);
        let out = correct(
            &positions,
            &edges,
            &merges,
            |id| (id >> 48) as u8,
            |_| true,
            SWEEPS,
        );
        let worst = (0..21u64)
            .map(|i| out[&(r1 | i)][1].abs())
            .fold(0.0, f64::max);
        assert!(worst < 0.05, "worst sideways error {worst} m");
    }
}
