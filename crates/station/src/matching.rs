//! Finding which nodes of a later robot are the same as nodes of earlier robots.

use std::collections::{BTreeMap, HashMap, HashSet};

use scene_graph::{Coordinate, Feature, NodeId, PosQueryCount, SceneGraph};

/// Nodes that may merge with each other: same merge rule, and same label if the rule asks.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Group {
    pub rule: usize,
    pub label: String,
}

/// Every mergeable node at its matching position, one Yggdrasil indexed layer per group, so a
/// radius search only sees candidates of the right group.
#[derive(Default)]
pub struct SpatialIndex {
    sg: SceneGraph,
    layers: HashMap<Group, u32>,
    ygg: HashMap<u64, NodeId>,
    id_of: HashMap<NodeId, u64>,
    pub group: HashMap<u64, Group>,
    pub pos: HashMap<u64, [f64; 3]>,
}

impl SpatialIndex {
    pub fn insert(&mut self, id: u64, group: Group, p: [f64; 3]) {
        let layer = *self
            .layers
            .entry(group.clone())
            .or_insert_with(|| self.sg.new_indexed_layer().id);
        let anon = self.sg.new_coordinates(
            p[0] as f32,
            p[1] as f32,
            p[2] as f32,
            vec![Feature::new("id", id)],
        );
        let nid = self.sg.layer_mut(layer).unwrap().push_node(anon).unwrap();
        self.ygg.insert(id, nid);
        self.id_of.insert(nid, id);
        self.group.insert(id, group);
        self.pos.insert(id, p);
    }

    pub fn remove(&mut self, id: u64) {
        if let Some(nid) = self.ygg.remove(&id) {
            self.id_of.remove(&nid);
            self.sg.del_node(nid).unwrap();
            self.group.remove(&id);
            self.pos.remove(&id);
        }
    }

    /// Nodes of `group` within about `radius` of `p`. Yggdrasil stores f32, so this searches a
    /// hair wider; callers check exact distances.
    pub fn near(&self, group: &Group, p: [f64; 3], radius: f64) -> Vec<u64> {
        let Some(layer) = self.layers.get(group) else {
            return Vec::new();
        };
        let centre = Coordinate::new(p[0] as f32, p[1] as f32, p[2] as f32);
        self.sg
            .layer(*layer)
            .unwrap()
            .nodes_within(&centre, radius as f32 + 1e-3, PosQueryCount::All)
            .into_iter()
            .map(|(nid, _)| self.id_of[&nid])
            .collect()
    }
}

/// A candidate merge: (distance, node, node it would merge into).
pub type Candidate = (f64, u64, u64);

/// Decide merges from candidate pairs: nearest pairs first. One-to-one, each target is taken
/// once; with `max_matching`, nodes left over then steal targets along augmenting paths so the
/// number of merges is the largest possible.
pub fn assign(
    mut pairs: Vec<Candidate>,
    one_to_one: bool,
    max_matching: bool,
) -> BTreeMap<u64, u64> {
    pairs.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)).then(a.2.cmp(&b.2)));
    let mut out = BTreeMap::new();
    let mut taken: HashMap<u64, u64> = HashMap::new();
    for &(_, id, to) in &pairs {
        if out.contains_key(&id) || (one_to_one && taken.contains_key(&to)) {
            continue;
        }
        out.insert(id, to);
        taken.insert(to, id);
    }
    if one_to_one && max_matching {
        augment(&pairs, &mut out, &mut taken);
    }
    out
}

/// Grow a one-to-one matching along augmenting paths (depth first, nearest candidates first).
fn augment(pairs: &[Candidate], out: &mut BTreeMap<u64, u64>, taken: &mut HashMap<u64, u64>) {
    let mut options: BTreeMap<u64, Vec<u64>> = BTreeMap::new();
    for &(_, id, to) in pairs {
        options.entry(id).or_default().push(to);
    }
    let free: Vec<u64> = options
        .keys()
        .copied()
        .filter(|id| !out.contains_key(id))
        .collect();
    for start in free {
        // path: stack[i].0 would take via[i]; stack[i].1 is the next option to try
        let mut seen: HashSet<u64> = HashSet::new();
        let mut stack: Vec<(u64, usize)> = vec![(start, 0)];
        let mut via: Vec<u64> = Vec::new();
        let mut found = false;
        while let Some(top) = stack.last_mut() {
            let (u, k) = *top;
            let Some(&t) = options[&u].get(k) else {
                stack.pop();
                via.pop();
                continue;
            };
            top.1 += 1;
            if !seen.insert(t) {
                continue;
            }
            via.push(t);
            match taken.get(&t) {
                None => {
                    found = true;
                    break;
                }
                Some(&holder) => stack.push((holder, 0)),
            }
        }
        if found {
            for ((u, _), t) in stack.iter().zip(&via) {
                out.insert(*u, *t);
                taken.insert(*t, *u);
            }
        }
    }
}
