//! Frame alignment from objects: where a robot's map sits in the station frame.
//!
//! Objects seen by two robots have the same label, and the distances between them agree in
//! both maps whatever the frames are. Every pair of same-label matches whose distances agree
//! gives a candidate transform (rotation about z, then translation); the one under which the
//! most objects land on a same-label object of the other map wins, and is refined by least
//! squares over those objects.

use std::collections::HashMap;

use crate::Transform;

/// A labelled point.
pub type Labelled = (String, [f64; 3]);

/// Distances between two matches must agree to this (m) for the pair to propose a transform.
const PAIR_TOLERANCE: f64 = 0.15;
/// Matches closer together than this (m, horizontally) give a poor rotation.
const MIN_BASELINE: f64 = 1.0;
/// Points a hypothesis is first scored on.
const SAMPLE: usize = 24;
/// At most this many candidate transforms are scored.
const MAX_HYPOTHESES: usize = 200_000;

/// The transform taking `from` (a robot's own frame) onto `to` (the station frame), and how
/// many objects of `from` it puts within `radius` of a same-label object of `to`.
pub fn estimate(from: &[Labelled], to: &[Labelled], radius: f64) -> Option<(Transform, usize)> {
    let index = Grid::new(to, radius);
    let mut matches: Vec<(usize, usize)> = Vec::new();
    for (i, (la, _)) in from.iter().enumerate() {
        for (j, (lb, _)) in to.iter().enumerate() {
            if la == lb {
                matches.push((i, j));
            }
        }
    }

    // Most hypotheses are wrong; score each on an evenly spread sample first and only count all
    // points for those that could beat the best so far.
    let step = (from.len() / SAMPLE).max(1);
    let sample: Vec<Labelled> = from.iter().step_by(step).cloned().collect();
    let (from_ids, sample_ids) = (index.label_ids(from), index.label_ids(&sample));
    let mut best: Option<(Transform, usize)> = None;
    let mut best_sample = 0usize;
    let mut tried = 0;
    'outer: for (k, &(i1, j1)) in matches.iter().enumerate() {
        for &(i2, j2) in &matches[k + 1..] {
            if i1 == i2 || j1 == j2 {
                continue;
            }
            let (a1, a2, b1, b2) = (from[i1].1, from[i2].1, to[j1].1, to[j2].1);
            let da = horizontal(a1, a2);
            let db = horizontal(b1, b2);
            if da < MIN_BASELINE
                || (da - db).abs() > PAIR_TOLERANCE
                || ((a2[2] - a1[2]) - (b2[2] - b1[2])).abs() > PAIR_TOLERANCE
            {
                continue;
            }
            let yaw = (b2[1] - b1[1]).atan2(b2[0] - b1[0]) - (a2[1] - a1[1]).atan2(a2[0] - a1[0]);
            let t = from_rotation(yaw, a1, b1);
            let quick = index.count(&sample, &sample_ids, &t);
            if quick * 4 >= best_sample * 3 {
                let score = index.count(from, &from_ids, &t);
                if best.as_ref().is_none_or(|b| score > b.1) {
                    best = Some((t, score));
                }
                best_sample = best_sample.max(quick);
            }
            tried += 1;
            if tried >= MAX_HYPOTHESES {
                break 'outer;
            }
        }
    }

    // refine on the inliers, twice: the inlier set can grow once the fit improves
    let (mut t, _) = best?;
    for _ in 0..2 {
        let pairs = index.inliers(from, &t);
        if pairs.len() < 2 {
            break;
        }
        t = least_squares(
            &pairs
                .iter()
                .map(|(i, j)| (from[*i].1, to[*j].1))
                .collect::<Vec<_>>(),
        );
    }
    let score = index.inliers(from, &t).len();
    Some((t, score))
}

/// How many objects of `from` land on a same-label object of `to` under `t`.
pub fn score(from: &[Labelled], to: &[Labelled], t: &Transform, radius: f64) -> usize {
    Grid::new(to, radius).inliers(from, t).len()
}

fn horizontal(a: [f64; 3], b: [f64; 3]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)).sqrt()
}

/// The transform with this yaw that takes `a` onto `b`.
fn from_rotation(yaw: f64, a: [f64; 3], b: [f64; 3]) -> Transform {
    let r = Transform {
        yaw,
        translation: [0.0; 3],
    }
    .apply(a);
    Transform {
        yaw,
        translation: [b[0] - r[0], b[1] - r[1], b[2] - r[2]],
    }
}

/// The yaw and translation that best take each `.0` onto its `.1`.
fn least_squares(pairs: &[([f64; 3], [f64; 3])]) -> Transform {
    let n = pairs.len() as f64;
    type Pair = ([f64; 3], [f64; 3]);
    let mean = |f: &dyn Fn(&Pair) -> [f64; 3]| {
        let mut m = [0.0; 3];
        for p in pairs {
            let v = f(p);
            for k in 0..3 {
                m[k] += v[k] / n;
            }
        }
        m
    };
    let ma = mean(&|p| p.0);
    let mb = mean(&|p| p.1);
    let (mut sin, mut cos) = (0.0, 0.0);
    for (a, b) in pairs {
        let (ax, ay) = (a[0] - ma[0], a[1] - ma[1]);
        let (bx, by) = (b[0] - mb[0], b[1] - mb[1]);
        sin += ax * by - ay * bx;
        cos += ax * bx + ay * by;
    }
    from_rotation(sin.atan2(cos), ma, mb)
}

/// Labelled points bucketed in a dense 2D grid of cells one radius wide (compressed rows), for
/// "is there a same-label point within radius". Labels are interned to integers so the hot loop
/// compares numbers, not strings.
struct Grid<'a> {
    points: &'a [Labelled],
    labels: HashMap<&'a str, u32>,
    point_label: Vec<u32>,
    radius: f64,
    origin: [f64; 2],
    size: [i64; 2],
    /// cell c holds points[order[start[c]..start[c + 1]]]
    start: Vec<u32>,
    order: Vec<u32>,
}

impl<'a> Grid<'a> {
    fn new(points: &'a [Labelled], radius: f64) -> Self {
        let radius = radius.max(1e-3);
        let mut labels: HashMap<&str, u32> = HashMap::new();
        let point_label: Vec<u32> = points
            .iter()
            .map(|(l, _)| {
                let n = labels.len() as u32;
                *labels.entry(l.as_str()).or_insert(n)
            })
            .collect();
        let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
        for (_, p) in points {
            for k in 0..2 {
                lo[k] = lo[k].min(p[k]);
                hi[k] = hi[k].max(p[k]);
            }
        }
        if points.is_empty() {
            (lo, hi) = ([0.0; 2], [0.0; 2]);
        }
        let size = [0, 1].map(|k| ((hi[k] - lo[k]) / radius).floor() as i64 + 1);
        let cell_of = |p: &[f64; 3]| {
            let x = ((p[0] - lo[0]) / radius).floor() as i64;
            let y = ((p[1] - lo[1]) / radius).floor() as i64;
            (y * size[0] + x) as usize
        };
        let cells = (size[0] * size[1]) as usize;
        let mut start = vec![0u32; cells + 1];
        for (_, p) in points {
            start[cell_of(p) + 1] += 1;
        }
        for c in 0..cells {
            start[c + 1] += start[c];
        }
        let mut fill = start.clone();
        let mut order = vec![0u32; points.len()];
        for (j, (_, p)) in points.iter().enumerate() {
            let c = cell_of(p);
            order[fill[c] as usize] = j as u32;
            fill[c] += 1;
        }
        Grid {
            points,
            labels,
            point_label,
            radius,
            origin: lo,
            size,
            start,
            order,
        }
    }

    fn label_ids(&self, from: &[Labelled]) -> Vec<Option<u32>> {
        from.iter()
            .map(|(l, _)| self.labels.get(l.as_str()).copied())
            .collect()
    }

    /// The nearest same-label point within the radius of `q`, with its squared distance.
    fn nearest(&self, label: u32, q: [f64; 3]) -> Option<(f64, usize)> {
        let r2 = self.radius * self.radius;
        let cx = ((q[0] - self.origin[0]) / self.radius).floor() as i64;
        let cy = ((q[1] - self.origin[1]) / self.radius).floor() as i64;
        let mut best: Option<(f64, usize)> = None;
        for y in (cy - 1).max(0)..=(cy + 1).min(self.size[1] - 1) {
            for x in (cx - 1).max(0)..=(cx + 1).min(self.size[0] - 1) {
                let c = (y * self.size[0] + x) as usize;
                for &j in &self.order[self.start[c] as usize..self.start[c + 1] as usize] {
                    let j = j as usize;
                    if self.point_label[j] != label {
                        continue;
                    }
                    let r = self.points[j].1;
                    let d = (q[0] - r[0]).powi(2) + (q[1] - r[1]).powi(2) + (q[2] - r[2]).powi(2);
                    if d <= r2 && best.is_none_or(|b| d < b.0) {
                        best = Some((d, j));
                    }
                }
            }
        }
        best
    }

    /// How many points of `from` (labels from [`Grid::label_ids`]) land within the radius of a
    /// same-label point under `t`.
    fn count(&self, from: &[Labelled], ids: &[Option<u32>], t: &Transform) -> usize {
        from.iter()
            .zip(ids)
            .filter(|((_, p), id)| id.is_some_and(|l| self.nearest(l, t.apply(*p)).is_some()))
            .count()
    }

    /// Each point of `from` under `t` with its nearest same-label point within the radius.
    fn inliers(&self, from: &[Labelled], t: &Transform) -> Vec<(usize, usize)> {
        let ids = self.label_ids(from);
        from.iter()
            .zip(&ids)
            .enumerate()
            .filter_map(|(i, ((_, p), id))| Some((i, self.nearest((*id)?, t.apply(*p))?.1)))
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn recovers_a_known_transform() {
        // a scattered set of labelled objects, and the same seen from a rotated, shifted frame
        let mut to = Vec::new();
        let mut seed = 7u64;
        let mut rnd = || {
            seed = seed
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            (seed >> 33) as f64 / (1u64 << 31) as f64
        };
        for k in 0..60 {
            to.push((
                format!("{}", k % 7),
                [rnd() * 30.0, rnd() * 20.0, rnd() * 2.0],
            ));
        }
        let truth = Transform {
            yaw: 0.9,
            translation: [12.0, -4.0, 0.3],
        };
        // from = truth^-1(to), with 2 cm noise, and a few objects only one robot saw
        let (s, c) = truth.yaw.sin_cos();
        let mut from: Vec<Labelled> = to
            .iter()
            .take(45)
            .map(|(l, p)| {
                let (x, y) = (p[0] - truth.translation[0], p[1] - truth.translation[1]);
                (
                    l.clone(),
                    [
                        c * x + s * y + 0.02 * rnd(),
                        -s * x + c * y - 0.02 * rnd(),
                        p[2] - truth.translation[2],
                    ],
                )
            })
            .collect();
        from.push(("3".into(), [100.0, 100.0, 0.0]));
        let (t, n) = estimate(&from, &to, 0.3).unwrap();
        assert!(n >= 44, "{n} inliers");
        assert!((t.yaw - truth.yaw).abs() < 0.01, "{t:?}");
        for k in 0..3 {
            assert!(
                (t.translation[k] - truth.translation[k]).abs() < 0.05,
                "{t:?}"
            );
        }
    }
}
