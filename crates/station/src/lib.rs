//! Hydra-Multi station on Yggdrasil: one global graph built from several robots' deltas.
//!
//! * Each robot's graph is rebuilt from its deltas ([`delta::Decoder`]).
//! * Robot node ids are made globally unique: the node's category character stays in the top
//!   byte, the robot number goes in the next byte (Hydra's own prefixes do not separate
//!   robots for every layer).
//! * Robot positions are moved into the station frame with each robot's transform.
//! * Nodes seen by several robots are merged by [`MergeRule`]s: objects with the same semantic
//!   label, places and rooms, each when their centres are close enough, found with
//!   Yggdrasil's spatial index. The merged-away node's edges move to the node it was merged
//!   into.
//! * The result lives in a Yggdrasil graph ([`Topology`]) and is updated incrementally: a
//!   delta only touches the nodes it names, the objects near them, and their edges.
//!   [`Station::merged_graph`] recomputes everything from scratch and is kept as the
//!   reference the incremental result is checked against.

pub mod align;
pub mod deform;
pub mod delta;

use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};

use dsg_convert::LayerKey;
use dsg_convert::incremental::HYDRA_LAYERS;
use dsg_convert::topology::Topology;
use scene_graph::{Coordinate, Feature, NodeId, PosQueryCount, SceneGraph};
use serde_json::{Value, json};

/// Rigid transform into the station frame: rotation about z (yaw) then translation.
#[derive(Clone, Copy, Debug)]
pub struct Transform {
    pub yaw: f64,
    pub translation: [f64; 3],
}

impl Default for Transform {
    fn default() -> Self {
        Transform {
            yaw: 0.0,
            translation: [0.0; 3],
        }
    }
}

impl Transform {
    pub fn apply(&self, p: [f64; 3]) -> [f64; 3] {
        let (s, c) = self.yaw.sin_cos();
        [
            c * p[0] - s * p[1] + self.translation[0],
            s * p[0] + c * p[1] + self.translation[1],
            p[2] + self.translation[2],
        ]
    }
}

/// When two robots' nodes of one Spark (layer, partition) count as the same node: centres
/// within `radius`, and the same semantic label if `same_label`.
#[derive(Clone, Copy, Debug)]
pub struct MergeRule {
    pub layer: LayerKey,
    pub radius: f64,
    pub same_label: bool,
}

pub struct Config {
    pub rules: Vec<MergeRule>,
    /// Each kept node takes at most one node of each later robot: when several of one robot's
    /// nodes have the same nearest target, only the closest merges and the rest stay separate.
    pub one_to_one: bool,
    /// After the nearest-first pairing, re-pair along augmenting paths so the most nodes merge
    /// (every pair still within the radius). One-to-one only.
    pub max_matching: bool,
}

pub const OBJECTS: LayerKey = (2, 0);
pub const PLACES: LayerKey = (3, 0);
pub const ROOMS: LayerKey = (4, 0);

impl Default for Config {
    fn default() -> Self {
        let rule = |layer, radius, same_label| MergeRule {
            layer,
            radius,
            same_label,
        };
        Config {
            rules: vec![
                rule(OBJECTS, 0.5, true),
                rule(PLACES, 1.0, false),
                rule(ROOMS, 1.0, false),
            ],
            one_to_one: true,
            max_matching: false,
        }
    }
}

impl Config {
    /// Objects only, as before places and rooms were merged.
    pub fn objects_only() -> Self {
        let mut c = Config::default();
        c.rules.retain(|r| r.layer == OBJECTS);
        c
    }
}

/// What one delta changed in the station graph.
#[derive(Debug, Default, Clone, PartialEq)]
pub struct Change {
    pub nodes_added: usize,
    pub nodes_updated: usize,
    pub nodes_removed: usize,
    pub edges_added: usize,
    pub edges_removed: usize,
    pub merges_changed: usize,
}

struct Robot {
    decoder: delta::Decoder,
    transform: Transform,
}

type EdgeKey = (u64, u64);

fn edge_key(s: u64, t: u64) -> EdgeKey {
    (s.min(t), s.max(t))
}

/// Global id of robot `robot`'s node `id`: category byte kept, robot number in the next byte.
pub fn global_id(robot: u8, id: u64) -> u64 {
    let category = id >> 56;
    let index = id & ((1 << 48) - 1);
    (category << 56) | ((robot as u64) << 48) | index
}

pub fn robot_of(global: u64) -> u8 {
    ((global >> 48) & 0xff) as u8
}

fn layer_key(n: &Value) -> LayerKey {
    (
        n["layer"].as_i64().unwrap_or(0),
        n["partition"].as_i64().unwrap_or(0),
    )
}

fn position(n: &Value) -> Option<[f64; 3]> {
    let p = n["attributes"]["position"].as_array()?;
    Some([0, 1, 2].map(|i| p[i].as_f64().unwrap()))
}

/// A robot edge's info, with its spark_dsg record (weight and other attributes) when it has one.
fn edge_info(e: &Value) -> Value {
    let mut info = e["info"].clone();
    if let Some(r) = e.get("record") {
        if !info.is_object() {
            info = json!({});
        }
        info["record"] = r.clone();
    }
    info
}

fn distance(p: [f64; 3], q: [f64; 3]) -> f64 {
    ((p[0] - q[0]).powi(2) + (p[1] - q[1]).powi(2) + (p[2] - q[2]).powi(2)).sqrt()
}

/// Node -> kept node from candidate (distance, node, kept node) pairs, nearest pairs first.
/// With `max_matching`, unmatched nodes then take a kept node along augmenting paths, nearest
/// candidates first, so the number of merges is the largest possible.
fn assign(
    mut pairs: Vec<(f64, u64, u64)>,
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
    if !(one_to_one && max_matching) {
        return out;
    }
    // candidates per node, nearest first (pairs are sorted by distance)
    let mut adj: BTreeMap<u64, Vec<u64>> = BTreeMap::new();
    for &(_, id, to) in &pairs {
        adj.entry(id).or_default().push(to);
    }
    let free: Vec<u64> = adj
        .keys()
        .copied()
        .filter(|id| !out.contains_key(id))
        .collect();
    for start in free {
        let mut seen: HashSet<u64> = HashSet::new();
        let mut stack: Vec<(u64, usize)> = vec![(start, 0)];
        let mut via: Vec<u64> = Vec::new();
        let mut found = false;
        while let Some(top) = stack.last_mut() {
            let (u, k) = (top.0, top.1);
            if k >= adj[&u].len() {
                stack.pop();
                via.pop();
                continue;
            }
            top.1 += 1;
            let t = adj[&u][k];
            if !seen.insert(t) {
                continue;
            }
            via.push(t);
            match taken.get(&t) {
                None => {
                    found = true;
                    break;
                }
                Some(&v) => stack.push((v, 0)),
            }
        }
        if found {
            for (i, (u, _)) in stack.iter().enumerate() {
                out.insert(*u, via[i]);
                taken.insert(via[i], *u);
            }
        }
    }
    out
}

/// Every mergeable node in the station frame, one Yggdrasil indexed layer per merge group
/// (rule, and label when the rule needs one).
#[derive(Default)]
struct ObjectIndex {
    sg: SceneGraph,
    layers: HashMap<String, u32>,
    ygg: HashMap<u64, NodeId>,
    global: HashMap<NodeId, u64>,
    label: HashMap<u64, String>,
    pos: HashMap<u64, [f64; 3]>,
}

impl ObjectIndex {
    fn insert(&mut self, id: u64, label: String, p: [f64; 3]) {
        let lid = match self.layers.get(&label) {
            Some(l) => *l,
            None => {
                let l = self.sg.new_indexed_layer().id;
                self.layers.insert(label.clone(), l);
                l
            }
        };
        let anon = self.sg.new_coordinates(
            p[0] as f32,
            p[1] as f32,
            p[2] as f32,
            vec![Feature::new("id", id)],
        );
        let nid = self.sg.layer_mut(lid).unwrap().push_node(anon).unwrap();
        self.ygg.insert(id, nid);
        self.global.insert(nid, id);
        self.label.insert(id, label);
        self.pos.insert(id, p);
    }

    fn remove(&mut self, id: u64) {
        if let Some(nid) = self.ygg.remove(&id) {
            self.global.remove(&nid);
            self.sg.del_node(nid).unwrap();
            self.label.remove(&id);
            self.pos.remove(&id);
        }
    }

    /// Objects with this label within `radius` of `p`, nearest first.
    fn near(&self, label: &str, p: [f64; 3], radius: f64) -> Vec<(u64, f64)> {
        let Some(lid) = self.layers.get(label) else {
            return Vec::new();
        };
        self.sg
            .layer(*lid)
            .unwrap()
            // Yggdrasil stores f32: search a hair wider, callers measure exact f64 distances
            .nodes_within(
                &Coordinate::new(p[0] as f32, p[1] as f32, p[2] as f32),
                radius as f32 + 1e-3,
                PosQueryCount::All,
            )
            .into_iter()
            .map(|(nid, d)| (self.global[&nid], d as f64))
            .collect()
    }
}

pub struct Station {
    pub config: Config,
    robots: BTreeMap<u8, Robot>,
    /// Every robot node, in the station frame, under its global id.
    nodes: HashMap<u64, Value>,
    /// Every robot edge under global ids, and the edges at each node.
    raw_edges: HashMap<EdgeKey, Value>,
    incident: HashMap<u64, HashSet<EdgeKey>>,
    objects: ObjectIndex,
    /// Global id merged away -> global id it now lives on.
    pub merges: BTreeMap<u64, u64>,
    /// Mergeable nodes by robot and merge group.
    members: HashMap<(u8, String), BTreeSet<u64>>,
    /// Station graph: the robot edges behind each station edge, and where each robot edge went.
    resolved: HashMap<EdgeKey, BTreeSet<EdgeKey>>,
    resolved_of: HashMap<EdgeKey, EdgeKey>,
    /// The station graph's structure and positions in Yggdrasil.
    pub topology: Topology,
    /// Drift correction per node (see [`Station::correct_drift`]), used when matching.
    offsets: HashMap<u64, [f64; 3]>,
}

impl Station {
    pub fn new(config: Config) -> Self {
        Station {
            config,
            robots: BTreeMap::new(),
            nodes: HashMap::new(),
            raw_edges: HashMap::new(),
            incident: HashMap::new(),
            objects: ObjectIndex::default(),
            merges: BTreeMap::new(),
            members: HashMap::new(),
            resolved: HashMap::new(),
            resolved_of: HashMap::new(),
            topology: Topology::new(HYDRA_LAYERS),
            offsets: HashMap::new(),
        }
    }

    pub fn add_robot(&mut self, robot: u8, transform: Transform) {
        self.robots.insert(
            robot,
            Robot {
                decoder: delta::Decoder::default(),
                transform,
            },
        );
    }

    pub fn num_nodes(&self) -> usize {
        self.topology.num_nodes()
    }

    pub fn num_edges(&self) -> usize {
        self.resolved.len()
    }

    /// Every robot node of one (layer, partition) with a position, merged away or not.
    pub fn robot_nodes(&self, layer: LayerKey) -> Vec<(u64, [f64; 3])> {
        self.nodes
            .iter()
            .filter(|(_, n)| layer_key(n) == layer)
            .filter_map(|(id, n)| Some((*id, position(n)?)))
            .collect()
    }

    /// The objects of `robot` in its own frame, and the objects of every lower robot in the
    /// station frame, labelled, for frame alignment.
    pub fn alignment_objects(&self, robot: u8) -> (Vec<align::Labelled>, Vec<align::Labelled>) {
        let label = |n: &Value| n["attributes"]["semantic_label"].to_string();
        let mine = self.robots[&robot]
            .decoder
            .nodes
            .values()
            .filter(|n| layer_key(n) == OBJECTS)
            .filter_map(|n| Some((label(n), position(n)?)))
            .collect();
        let theirs = self
            .nodes
            .iter()
            .filter(|(id, n)| robot_of(**id) < robot && layer_key(n) == OBJECTS)
            .filter_map(|(_, n)| Some((label(n), position(n)?)))
            .collect();
        (mine, theirs)
    }

    /// [`Station::alignment_objects`] plus places with a descriptor: a place that has objects
    /// attached is labelled by the sorted labels of those objects, so places count in frame
    /// alignment too and robots that share few objects can still be aligned.
    pub fn alignment_points(&self, robot: u8) -> (Vec<align::Labelled>, Vec<align::Labelled>) {
        let (mut mine, mut theirs) = self.alignment_objects(robot);
        let r = &self.robots[&robot];
        mine.extend(described_places(
            |id| r.decoder.nodes.get(&id),
            r.decoder.edges.keys(),
        ));
        let lower = |id: u64| self.nodes.get(&id).filter(|_| robot_of(id) < robot);
        theirs.extend(described_places(lower, self.raw_edges.keys()));
        (mine, theirs)
    }

    pub fn transform_of(&self, robot: u8) -> Option<Transform> {
        self.robots.get(&robot).map(|r| r.transform)
    }

    /// The (layer, partition) of any robot node by global id, merged away or not.
    pub fn layer_of_robot_node(&self, id: u64) -> Option<LayerKey> {
        self.nodes.get(&id).map(layer_key)
    }

    /// The merge group and radius of a node, if any rule covers it.
    fn group(&self, n: &Value) -> Option<(String, f64)> {
        position(n)?;
        let (i, rule) = self
            .config
            .rules
            .iter()
            .enumerate()
            .find(|(_, r)| r.layer == layer_key(n))?;
        let label = if rule.same_label {
            n["attributes"]["semantic_label"].to_string()
        } else {
            String::new()
        };
        Some((format!("{i}/{label}"), rule.radius))
    }

    fn radius_of(&self, group: &str) -> f64 {
        let i: usize = group.split('/').next().unwrap().parse().unwrap();
        self.config.rules[i].radius
    }

    fn header(&self) -> Value {
        self.robots
            .values()
            .map(|r| &r.decoder.header)
            .find(|h| h.is_object())
            .cloned()
            .unwrap_or_else(|| json!({}))
    }

    /// Take one delta from `robot` and bring the station graph up to date.
    pub fn receive(&mut self, robot: u8, delta: &Value) -> Result<Change, String> {
        let r = self
            .robots
            .get_mut(&robot)
            .ok_or(format!("unknown robot {robot}"))?;
        r.decoder.apply(delta)?;
        let transform = r.transform;

        let mut node_ids: Vec<u64> = Vec::new();
        for id in delta["removed_nodes"].as_array().into_iter().flatten() {
            node_ids.push(id.as_u64().unwrap());
        }
        for n in delta["nodes"].as_array().into_iter().flatten() {
            node_ids.push(n["id"].as_u64().unwrap());
        }
        let mut edge_keys: Vec<EdgeKey> = Vec::new();
        for pair in delta["removed_edges"].as_array().into_iter().flatten() {
            edge_keys.push((pair[0].as_u64().unwrap(), pair[1].as_u64().unwrap()));
        }
        for e in delta["edges"].as_array().into_iter().flatten() {
            edge_keys.push((e["source"].as_u64().unwrap(), e["target"].as_u64().unwrap()));
        }
        Ok(self.refresh(robot, transform, &node_ids, &edge_keys, BTreeSet::new()))
    }

    /// Move robot `robot` to a new transform, e.g. after frame alignment improved.
    pub fn set_transform(&mut self, robot: u8, transform: Transform) -> Change {
        let Some(r) = self.robots.get_mut(&robot) else {
            return Change::default();
        };
        r.transform = transform;
        let ids: Vec<u64> = r.decoder.nodes.keys().copied().collect();
        self.refresh(robot, transform, &ids, &[], BTreeSet::new())
    }

    /// Correct drift between robots ([`deform`], with the merges of `closure_layers` as loop
    /// closures) and match again on the corrected positions, so nodes that drift kept apart
    /// can now merge. Corrections stay until the next call; repeat to refine.
    pub fn correct_drift(&mut self, closure_layers: &[LayerKey]) -> Change {
        let corrected = self.corrected_positions(closure_layers);
        let mut dirty: BTreeSet<(u8, String)> = BTreeSet::new();
        let ids: Vec<u64> = self.objects.pos.keys().copied().collect();
        for id in ids {
            let (Some(raw), Some(c)) = (self.nodes.get(&id).and_then(position), corrected.get(&id))
            else {
                continue;
            };
            let offset = [c[0] - raw[0], c[1] - raw[1], c[2] - raw[2]];
            let old = self.offsets.get(&id).copied().unwrap_or([0.0; 3]);
            if distance(offset, old) < 1e-6 {
                continue;
            }
            self.offsets.insert(id, offset);
            let group = self.objects.label[&id].clone();
            self.objects.remove(id);
            self.objects.insert(id, group.clone(), *c);
            for r in self.robots.keys().filter(|r| **r >= robot_of(id)) {
                dirty.insert((*r, group.clone()));
            }
        }
        let robot = *self.robots.keys().next().unwrap_or(&0);
        let transform = self.transform_of(robot).unwrap_or_default();
        self.refresh(robot, transform, &[], &[], dirty)
    }

    /// A node's position for matching: station frame plus its drift correction.
    fn matching_position(&self, id: u64, p: [f64; 3]) -> [f64; 3] {
        let o = self.offsets.get(&id).copied().unwrap_or([0.0; 3]);
        [p[0] + o[0], p[1] + o[1], p[2] + o[2]]
    }

    /// Re-read these nodes and edges of `robot` from its decoder and update everything that
    /// depends on them.
    fn refresh(
        &mut self,
        robot: u8,
        transform: Transform,
        node_ids: &[u64],
        edge_keys: &[EdgeKey],
        mut dirty: BTreeSet<(u8, String)>,
    ) -> Change {
        let mut change = Change::default();
        // Nodes whose station-graph presence or attributes may have changed, and robot edges
        // whose station edge may have changed.
        let mut touched_nodes: BTreeSet<u64> = BTreeSet::new();
        let mut touched_edges: BTreeSet<EdgeKey> = BTreeSet::new();
        // Merge groups whose matching must be worked out again, in robot order (`dirty`).
        let later: Vec<u8> = self
            .robots
            .keys()
            .copied()
            .filter(|r| *r >= robot)
            .collect();

        // 1. robot nodes, in the station frame
        for &local in node_ids {
            let id = global_id(robot, local);
            let new = self.robots[&robot].decoder.nodes.get(&local).map(|n| {
                let mut n = n.clone();
                n["id"] = json!(id);
                if let Some(p) = position(&n) {
                    n["attributes"]["position"] = json!(transform.apply(p));
                }
                n
            });
            if new.as_ref() == self.nodes.get(&id) {
                continue;
            }
            // its group's matching, for this robot and every later one, may change now
            if let Some(group) = self.objects.label.get(&id).cloned() {
                for r in &later {
                    dirty.insert((*r, group.clone()));
                }
                self.objects.remove(id);
                if let Some(set) = self.members.get_mut(&(robot, group)) {
                    set.remove(&id);
                }
                if self.merges.remove(&id).is_some() {
                    change.merges_changed += 1;
                }
            }
            match new {
                Some(n) => {
                    if let Some((group, _)) = self.group(&n) {
                        for r in &later {
                            dirty.insert((*r, group.clone()));
                        }
                        let p = self.matching_position(id, position(&n).unwrap());
                        self.objects.insert(id, group.clone(), p);
                        self.members.entry((robot, group)).or_default().insert(id);
                    }
                    self.nodes.insert(id, n);
                }
                None => {
                    self.nodes.remove(&id);
                }
            }
            touched_nodes.insert(id);
        }

        // 2. robot edges
        for &(s, t) in edge_keys {
            let key = edge_key(global_id(robot, s), global_id(robot, t));
            let info = self.robots[&robot]
                .decoder
                .edges
                .get(&edge_key(s, t))
                .map(edge_info);
            match info {
                Some(info) => {
                    self.raw_edges.insert(key, info);
                    self.incident.entry(key.0).or_default().insert(key);
                    self.incident.entry(key.1).or_default().insert(key);
                }
                None => {
                    self.raw_edges.remove(&key);
                    for end in [key.0, key.1] {
                        if let Some(set) = self.incident.get_mut(&end) {
                            set.remove(&key);
                            if set.is_empty() {
                                self.incident.remove(&end);
                            }
                        }
                    }
                }
            }
            touched_edges.insert(key);
        }

        // 3. merges, lower robots first: a robot's matching depends only on lower robots
        while let Some((r, group)) = dirty.pop_first() {
            let ids: Vec<u64> = self
                .members
                .get(&(r, group.clone()))
                .map(|s| s.iter().copied().collect())
                .unwrap_or_default();
            let wanted = self.matching(r, &group, &ids);
            let mut changed = false;
            for id in ids {
                let target = wanted.get(&id).copied();
                if target == self.merges.get(&id).copied() {
                    continue;
                }
                match target {
                    Some(to) => self.merges.insert(id, to),
                    None => self.merges.remove(&id),
                };
                change.merges_changed += 1;
                touched_nodes.insert(id);
                changed = true;
            }
            // which nodes are kept changed: later robots match against them
            if changed {
                for later in self.robots.keys().filter(|x| **x > r) {
                    dirty.insert((*later, group.clone()));
                }
            }
        }

        // 4. the station edges of every touched node's robot edges
        for id in &touched_nodes {
            if let Some(set) = self.incident.get(id) {
                touched_edges.extend(set.iter().copied());
            }
        }
        let mut pairs: BTreeSet<EdgeKey> = BTreeSet::new();
        for key in touched_edges {
            if let Some(old) = self.resolved_of.remove(&key) {
                let set = self.resolved.get_mut(&old).unwrap();
                set.remove(&key);
                pairs.insert(old);
            }
            if !self.raw_edges.contains_key(&key) {
                continue;
            }
            let (s, t) = (self.resolve(key.0), self.resolve(key.1));
            if s == t || !self.nodes.contains_key(&s) || !self.nodes.contains_key(&t) {
                continue;
            }
            let pair = edge_key(s, t);
            self.resolved.entry(pair).or_default().insert(key);
            self.resolved_of.insert(key, pair);
            pairs.insert(pair);
        }

        // 5. Yggdrasil: edges out, nodes out, nodes in or moved, edges in
        for pair in &pairs {
            if self.resolved.get(pair).is_some_and(|s| s.is_empty()) {
                self.resolved.remove(pair);
                if self.topology.remove_edge(pair.0, pair.1) {
                    change.edges_removed += 1;
                }
            }
        }
        for id in &touched_nodes {
            let wanted = if self.merges.contains_key(id) {
                None
            } else {
                self.nodes.get(id)
            };
            match (wanted, self.topology.layer_of(*id)) {
                (None, Some(_)) => {
                    self.topology.remove_node(*id);
                    change.nodes_removed += 1;
                }
                (Some(n), Some(key)) if key != layer_key(n) => {
                    // changed layer: comes back with its edges below
                    for other in self.topology.neighbors(*id) {
                        pairs.insert(edge_key(*id, other));
                    }
                    self.topology.remove_node(*id);
                    self.topology.add_node(*id, layer_key(n), position(n));
                    change.nodes_updated += 1;
                }
                (Some(n), Some(_)) => {
                    self.topology.set_position(*id, position(n));
                    change.nodes_updated += 1;
                }
                (Some(n), None) => {
                    self.topology.add_node(*id, layer_key(n), position(n));
                    change.nodes_added += 1;
                }
                (None, None) => {}
            }
        }
        for pair in &pairs {
            if self.resolved.contains_key(pair) && self.topology.add_edge(pair.0, pair.1) {
                change.edges_added += 1;
            }
        }
        change
    }

    /// The node a node should merge into: the nearest one of its merge group, of a lower
    /// robot, within the rule's radius, and not merged away itself.
    /// Where each of robot `robot`'s nodes of one merge group merges: candidates are kept
    /// nodes (not merged away) of lower robots within the rule's radius. Many-to-one, each node
    /// takes its nearest. One-to-one, pairs are taken nearest first, each node and each kept
    /// node at most once, so a node whose nearest was taken falls back to its next nearest.
    fn matching(&self, robot: u8, group: &str, ids: &[u64]) -> BTreeMap<u64, u64> {
        let radius = self.radius_of(group);
        let mut pairs: Vec<(f64, u64, u64)> = Vec::new();
        for id in ids {
            let p = self.objects.pos[id];
            for (o, _) in self.objects.near(group, p, radius) {
                if robot_of(o) >= robot || self.merges.contains_key(&o) {
                    continue;
                }
                let d = distance(p, self.objects.pos[&o]);
                if d <= radius {
                    pairs.push((d, *id, o));
                }
            }
        }
        assign(pairs, self.config.one_to_one, self.config.max_matching)
    }

    fn resolve(&self, id: u64) -> u64 {
        self.merges.get(&id).copied().unwrap_or(id)
    }

    /// Every robot node's position after drift correction ([`deform`]), with the merges of
    /// `closure_layers` as loop closures.
    pub fn corrected_positions(&self, closure_layers: &[LayerKey]) -> HashMap<u64, [f64; 3]> {
        let positions: HashMap<u64, [f64; 3]> = self
            .nodes
            .iter()
            .filter_map(|(id, n)| Some((*id, position(n)?)))
            .collect();
        let edges: Vec<EdgeKey> = self.raw_edges.keys().copied().collect();
        deform::correct(
            &positions,
            &edges,
            &self.merges,
            robot_of,
            |id| {
                self.layer_of_robot_node(id)
                    .is_some_and(|l| closure_layers.contains(&l))
            },
            deform::SWEEPS,
        )
    }

    /// [`Station::export`] with drift-corrected positions.
    pub fn export_corrected(&self, closure_layers: &[LayerKey]) -> Value {
        let pos = self.corrected_positions(closure_layers);
        let mut g = self.export();
        for n in g["nodes"].as_array_mut().into_iter().flatten() {
            if let Some(p) = n["id"].as_u64().and_then(|id| pos.get(&id))
                && n["attributes"]["position"].is_array()
            {
                n["attributes"]["position"] = json!(p);
            }
        }
        g
    }

    /// The station graph as Spark-DSG JSON.
    pub fn export(&self) -> Value {
        let mut ids: Vec<u64> = self.topology.node_ids().collect();
        ids.sort();
        let mut pairs: Vec<&EdgeKey> = self.resolved.keys().collect();
        pairs.sort();
        let mut g = self.header();
        g["nodes"] = json!(ids.iter().map(|id| &self.nodes[id]).collect::<Vec<_>>());
        g["edges"] = json!(
            pairs
                .iter()
                .map(|p| {
                    let first = self.resolved[*p].first().unwrap();
                    // the robots whose edges this station edge stands for
                    let robots: BTreeSet<u8> = self.resolved[*p].iter().map(|k| robot_of(k.0)).collect();
                    json!({"source": p.0, "target": p.1, "info": self.raw_edges[first], "robots": robots})
                })
                .collect::<Vec<_>>()
        );
        g
    }

    /// The station graph recomputed from every robot's whole graph, and the merges it implies.
    /// Slow; the reference [`Station::export`] and [`Station::merges`] must always equal.
    pub fn merged_graph(&self) -> (Value, BTreeMap<u64, u64>) {
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

        // Robot by robot; each node merges into the nearest kept node of its group of a lower
        // robot.
        let mut merges = BTreeMap::new();
        let mut kept: Vec<(u64, String, [f64; 3])> = Vec::new();
        for robot in self.robots.keys() {
            let mut mine = Vec::new();
            let mut pairs: Vec<(f64, u64, u64)> = Vec::new();
            let mut group_of: BTreeMap<u64, (String, [f64; 3])> = BTreeMap::new();
            for (id, n) in nodes.iter().filter(|(id, _)| robot_of(**id) == *robot) {
                let Some((label, radius)) = self.group(n) else {
                    continue;
                };
                let p = self.matching_position(*id, position(n).unwrap());
                for (o, l, q) in &kept {
                    let d = distance(p, *q);
                    if *l == label && d <= radius {
                        pairs.push((d, *id, *o));
                    }
                }
                group_of.insert(*id, (label, p));
            }
            // kept nodes are told apart by group, so one assignment covers every group
            let matched = assign(pairs, self.config.one_to_one, self.config.max_matching);
            for (id, (label, p)) in group_of {
                match matched.get(&id) {
                    Some(to) => {
                        merges.insert(id, *to);
                    }
                    None => mine.push((id, label, p)),
                }
            }
            kept.extend(mine);
        }

        let resolve = |id: u64| merges.get(&id).copied().unwrap_or(id);
        let mut out_edges: BTreeMap<EdgeKey, Value> = BTreeMap::new();
        for (key, info) in &edges {
            let (s, t) = (resolve(key.0), resolve(key.1));
            if s == t || merges.contains_key(&s) || !nodes.contains_key(&s) {
                continue;
            }
            if merges.contains_key(&t) || !nodes.contains_key(&t) {
                continue;
            }
            // the station edge keeps the info of its smallest robot edge
            out_edges
                .entry(edge_key(s, t))
                .or_insert_with(|| info.clone());
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

/// Rooms of different robots that share places after the place merge are one room. A room joins
/// another when at least half of the smaller room's places are shared. The joined room keeps the
/// Places with at least one object attached, labelled by the sorted labels of those objects.
fn described_places<'a, 'n>(
    node: impl Fn(u64) -> Option<&'n Value>,
    edges: impl Iterator<Item = &'a EdgeKey>,
) -> Vec<align::Labelled> {
    let mut labels: BTreeMap<u64, Vec<String>> = BTreeMap::new();
    for (a, b) in edges {
        let (Some(na), Some(nb)) = (node(*a), node(*b)) else {
            continue;
        };
        for (p, o) in [(na, nb), (nb, na)] {
            if layer_key(p) == PLACES && layer_key(o) == OBJECTS {
                labels
                    .entry(p["id"].as_u64().unwrap_or(0))
                    .or_default()
                    .push(o["attributes"]["semantic_label"].to_string());
            }
        }
    }
    labels
        .into_iter()
        .filter_map(|(pid, mut l)| {
            l.sort();
            let n = node(pid)?;
            Some((format!("place:{}", l.join(",")), position(n)?))
        })
        .collect()
}

/// lowest id, its edges move over, and its position is the mean of its places.
pub fn merge_rooms_by_places(graph: &mut Value) -> usize {
    const ROOMS: (i64, i64) = (4, 0);
    const PLACES: (i64, i64) = (3, 0);
    let key = |n: &Value| {
        (
            n["layer"].as_i64().unwrap_or(-1),
            n["partition"].as_i64().unwrap_or(0),
        )
    };
    let mut kind: BTreeMap<u64, (i64, i64)> = BTreeMap::new();
    for n in graph["nodes"].as_array().unwrap() {
        kind.insert(n["id"].as_u64().unwrap(), key(n));
    }
    let mut places_of: BTreeMap<u64, std::collections::BTreeSet<u64>> = BTreeMap::new();
    for e in graph["edges"].as_array().unwrap() {
        let (s, t) = (e["source"].as_u64().unwrap(), e["target"].as_u64().unwrap());
        for (room, place) in [(s, t), (t, s)] {
            if kind.get(&room) == Some(&ROOMS) && kind.get(&place) == Some(&PLACES) {
                places_of.entry(room).or_default().insert(place);
            }
        }
    }

    // union-find over rooms
    let mut parent: BTreeMap<u64, u64> = places_of.keys().map(|r| (*r, *r)).collect();
    fn find(parent: &mut BTreeMap<u64, u64>, x: u64) -> u64 {
        let p = parent[&x];
        if p == x {
            return x;
        }
        let root = find(parent, p);
        parent.insert(x, root);
        root
    }
    let rooms: Vec<u64> = places_of.keys().copied().collect();
    for (i, a) in rooms.iter().enumerate() {
        for b in &rooms[i + 1..] {
            if robot_of(*a) == robot_of(*b) {
                continue;
            }
            let (pa, pb) = (&places_of[a], &places_of[b]);
            let shared = pa.intersection(pb).count();
            if shared > 0 && 2 * shared >= pa.len().min(pb.len()) {
                let (ra, rb) = (find(&mut parent, *a), find(&mut parent, *b));
                if ra != rb {
                    parent.insert(ra.max(rb), ra.min(rb));
                }
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

    // new room positions: the mean of every place the joined rooms hold
    let mut members: BTreeMap<u64, std::collections::BTreeSet<u64>> = BTreeMap::new();
    for r in &rooms {
        let root = into.get(r).copied().unwrap_or(*r);
        members
            .entry(root)
            .or_default()
            .extend(places_of[r].iter().copied());
    }
    let pos: BTreeMap<u64, [f64; 3]> = graph["nodes"]
        .as_array()
        .unwrap()
        .iter()
        .filter_map(|n| {
            let p = n["attributes"]["position"].as_array()?;
            Some((
                n["id"].as_u64()?,
                [p[0].as_f64()?, p[1].as_f64()?, p[2].as_f64()?],
            ))
        })
        .collect();

    let nodes = graph["nodes"].as_array_mut().unwrap();
    nodes.retain(|n| !into.contains_key(&n["id"].as_u64().unwrap()));
    for n in nodes.iter_mut() {
        let id = n["id"].as_u64().unwrap();
        if !into.values().any(|r| *r == id) {
            continue;
        }
        let ps: Vec<[f64; 3]> = members[&id]
            .iter()
            .filter_map(|p| pos.get(p).copied())
            .collect();
        if !ps.is_empty() {
            let k = ps.len() as f64;
            let mean: Vec<f64> = (0..3)
                .map(|i| ps.iter().map(|p| p[i]).sum::<f64>() / k)
                .collect();
            n["attributes"]["position"] = serde_json::json!(mean);
        }
    }

    let mut seen = std::collections::BTreeSet::new();
    let edges = graph["edges"].as_array_mut().unwrap();
    let mut kept = Vec::new();
    for mut e in edges.drain(..) {
        for end in ["source", "target"] {
            let id = e[end].as_u64().unwrap();
            if let Some(r) = into.get(&id) {
                e[end] = serde_json::json!(r);
            }
        }
        let (s, t) = (e["source"].as_u64().unwrap(), e["target"].as_u64().unwrap());
        if s != t && seen.insert((s.min(t), s.max(t))) {
            kept.push(e);
        }
    }
    *edges = kept;
    into.len()
}

#[cfg(test)]
mod tests {
    use super::*;
    use delta::Encoder;
    use dsg_convert::compare::compare;

    fn object(id: u64, label: u64, x: f64) -> Value {
        json!({"id": (b'O' as u64) << 56 | id, "layer": 2, "partition": 0,
               "attributes": {"position": [x, 0.0, 0.0], "semantic_label": label}})
    }

    fn place(id: u64, x: f64) -> Value {
        json!({"id": (b'p' as u64) << 56 | id, "layer": 3, "partition": 0,
               "attributes": {"position": [x, 0.0, 0.0]}})
    }

    fn check(station: &Station) {
        let (want, merges) = station.merged_graph();
        assert_eq!(station.merges, merges);
        let d = compare(&want, &station.export(), 0.0);
        assert!(d.is_equal(), "{d:?}");
        assert_eq!(station.topology.num_edges(), station.num_edges());
    }

    #[test]
    fn merges_the_same_object_seen_by_two_robots() {
        let p = |i: u64| (b'p' as u64) << 56 | i;
        let o = |i: u64| (b'O' as u64) << 56 | i;
        // robot 0: chair at x=1 under place 1; robot 1 sees it at x=1.2 in its own frame,
        // which is shifted by +5 in x, plus a table only it sees.
        let a = json!({"nodes": [object(1, 7, 1.0), place(1, 1.0)],
                       "edges": [{"source": p(1), "target": o(1), "info": {}}]});
        let b = json!({"nodes": [object(1, 7, -3.8), object(2, 9, -3.0), place(1, -3.8)],
                       "edges": [{"source": p(1), "target": o(1), "info": {}},
                                 {"source": p(1), "target": o(2), "info": {}}]});
        let mut station = Station::new(Config::objects_only());
        station.add_robot(0, Transform::default());
        station.add_robot(
            1,
            Transform {
                yaw: 0.0,
                translation: [5.0, 0.0, 0.0],
            },
        );
        let (mut ea, mut eb) = (Encoder::default(), Encoder::default());
        station.receive(0, &ea.encode(&a)).unwrap();
        station.receive(1, &eb.encode(&b)).unwrap();
        check(&station);

        // robot 1's chair merged into robot 0's; its place now links to robot 0's chair
        let chair0 = global_id(0, o(1));
        assert_eq!(
            station.merges,
            BTreeMap::from([(global_id(1, o(1)), chair0)])
        );
        let g = station.export();
        assert_eq!(g["nodes"].as_array().unwrap().len(), 4);
        assert!(station.topology.has_edge(global_id(1, p(1)), chair0));
        assert_eq!(station.num_edges(), 3);

        // robot 0 loses its chair: robot 1's comes back, with its own edge
        let a2 = json!({"nodes": [place(1, 1.0)], "edges": []});
        station.receive(0, &ea.encode(&a2)).unwrap();
        check(&station);
        assert!(station.merges.is_empty());
        assert!(
            station
                .topology
                .has_edge(global_id(1, p(1)), global_id(1, o(1)))
        );

        // drift correction keeps the incremental graph equal to the reference
        station.correct_drift(&[OBJECTS, PLACES, ROOMS]);
        check(&station);

        // robot 1 drifts away: the chair seen again by robot 0 stays separate
        station.receive(0, &ea.encode(&a)).unwrap();
        station.set_transform(
            1,
            Transform {
                yaw: 0.0,
                translation: [9.0, 0.0, 0.0],
            },
        );
        check(&station);
        assert!(station.merges.is_empty());
    }
}
