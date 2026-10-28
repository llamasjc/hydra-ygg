//! The multi-robot station: several robots' scene graphs merged into one, kept in Yggdrasil.
//!
//! Each robot streams deltas of its Hydra scene graph ([`delta`]). For every delta the station:
//!
//! 1. rebuilds that robot's graph and moves its nodes into the station frame
//!    ([`Transform`]), under ids that are unique across robots ([`global_id`]);
//! 2. merges nodes that several robots saw ([`MergeRule`]): an object, place or room of a
//!    later robot merges into the nearest matching node of an earlier robot, found with a
//!    Yggdrasil radius search ([`matching`]);
//! 3. reroutes the edges of merged-away nodes onto the nodes they merged into;
//! 4. applies only what changed to the station graph in Yggdrasil ([`Topology`]).
//!
//! Optional steps: [`align`] finds a robot's frame when it is unknown, [`deform`] corrects
//! drift using the merges as loop closures, and [`rooms`] joins rooms that robots share.
//!
//! [`Station::merged_graph`] recomputes everything from scratch; the incremental result must
//! always equal it.

pub mod align;
pub mod config;
pub mod deform;
pub mod delta;
pub mod frame;
pub mod matching;
mod node;
mod reference;
pub mod rooms;

use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};

use dsg_convert::LayerKey;
use dsg_convert::incremental::HYDRA_LAYERS;
use dsg_convert::topology::Topology;
use serde_json::{Value, json};

pub use config::{Config, MergeRule, OBJECTS, PLACES, ROOMS};
pub use frame::{Transform, global_id, robot_of};
use matching::{Group, SpatialIndex, assign};
use node::{EdgeKey, add, distance, edge_info, edge_key, label, layer_key, position, sub};
pub use rooms::merge_rooms_by_places;

/// What one update changed in the station graph.
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

/// Bookkeeping for one [`Station::refresh`].
#[derive(Default)]
struct Pending {
    change: Change,
    /// Nodes whose presence or attributes in the station graph may have changed.
    nodes: BTreeSet<u64>,
    /// Robot edges whose station edge may have changed.
    edges: BTreeSet<EdgeKey>,
    /// (robot, group) matchings to redo, lowest robot first.
    groups: BTreeSet<(u8, Group)>,
}

pub struct Station {
    pub config: Config,
    robots: BTreeMap<u8, Robot>,
    /// Every robot node in the station frame, by global id, merged away or not.
    nodes: HashMap<u64, Value>,
    /// Every robot edge by global ids, and the edges at each node.
    raw_edges: HashMap<EdgeKey, Value>,
    incident: HashMap<u64, HashSet<EdgeKey>>,
    /// Mergeable nodes, searchable by position.
    index: SpatialIndex,
    /// Mergeable nodes by robot and group.
    members: HashMap<(u8, Group), BTreeSet<u64>>,
    /// Merged-away node -> the node it merged into.
    pub merges: BTreeMap<u64, u64>,
    /// Station edge -> the robot edges it stands for, and the reverse.
    station_edges: HashMap<EdgeKey, BTreeSet<EdgeKey>>,
    station_edge_of: HashMap<EdgeKey, EdgeKey>,
    /// Drift correction per node (see [`Station::correct_drift`]), added before matching.
    offsets: HashMap<u64, [f64; 3]>,
    /// The station graph in Yggdrasil.
    pub topology: Topology,
}

impl Station {
    pub fn new(config: Config) -> Self {
        Station {
            config,
            robots: BTreeMap::new(),
            nodes: HashMap::new(),
            raw_edges: HashMap::new(),
            incident: HashMap::new(),
            index: SpatialIndex::default(),
            members: HashMap::new(),
            merges: BTreeMap::new(),
            station_edges: HashMap::new(),
            station_edge_of: HashMap::new(),
            offsets: HashMap::new(),
            topology: Topology::new(HYDRA_LAYERS),
        }
    }

    pub fn add_robot(&mut self, robot: u8, transform: Transform) {
        let decoder = delta::Decoder::default();
        self.robots.insert(robot, Robot { decoder, transform });
    }

    pub fn num_nodes(&self) -> usize {
        self.topology.num_nodes()
    }

    pub fn num_edges(&self) -> usize {
        self.station_edges.len()
    }

    pub fn transform_of(&self, robot: u8) -> Option<Transform> {
        self.robots.get(&robot).map(|r| r.transform)
    }

    /// Every robot node of one layer with a position, merged away or not.
    pub fn robot_nodes(&self, layer: LayerKey) -> Vec<(u64, [f64; 3])> {
        self.nodes
            .iter()
            .filter(|(_, n)| layer_key(n) == layer)
            .filter_map(|(id, n)| Some((*id, position(n)?)))
            .collect()
    }

    /// The layer of any robot node, merged away or not.
    pub fn layer_of_robot_node(&self, id: u64) -> Option<LayerKey> {
        self.nodes.get(&id).map(layer_key)
    }

    // ---- updates ----

    /// Take one delta from `robot` and bring the station graph up to date.
    pub fn receive(&mut self, robot: u8, delta: &Value) -> Result<Change, String> {
        let r = self
            .robots
            .get_mut(&robot)
            .ok_or(format!("unknown robot {robot}"))?;
        r.decoder.apply(delta)?;

        let ids = |key: &str| delta[key].as_array().into_iter().flatten();
        let id = |v: &Value| v.as_u64().unwrap();
        let nodes: Vec<u64> = ids("removed_nodes")
            .map(id)
            .chain(ids("nodes").map(|n| id(&n["id"])))
            .collect();
        let edges: Vec<EdgeKey> = ids("removed_edges")
            .map(|p| (id(&p[0]), id(&p[1])))
            .chain(ids("edges").map(|e| (id(&e["source"]), id(&e["target"]))))
            .collect();
        Ok(self.refresh(robot, &nodes, &edges, Pending::default()))
    }

    /// Move `robot` to a new frame, e.g. after alignment found a better one.
    pub fn set_transform(&mut self, robot: u8, transform: Transform) -> Change {
        let Some(r) = self.robots.get_mut(&robot) else {
            return Change::default();
        };
        r.transform = transform;
        let ids: Vec<u64> = r.decoder.nodes.keys().copied().collect();
        self.refresh(robot, &ids, &[], Pending::default())
    }

    /// Correct drift between robots ([`deform`], with the merges in `closure_layers` as loop
    /// closures) and match again on the corrected positions, so nodes that drift kept apart can
    /// now merge. Positions in the exported graph are not changed; see
    /// [`Station::export_corrected`]. Repeat to refine.
    pub fn correct_drift(&mut self, closure_layers: &[LayerKey]) -> Change {
        let corrected = self.corrected_positions(closure_layers);
        let mut pending = Pending::default();
        let ids: Vec<u64> = self.index.pos.keys().copied().collect();
        for id in ids {
            let (Some(raw), Some(&c)) =
                (self.nodes.get(&id).and_then(position), corrected.get(&id))
            else {
                continue;
            };
            let offset = sub(c, raw);
            if distance(offset, self.offset(id)) < 1e-6 {
                continue;
            }
            self.offsets.insert(id, offset);
            let group = self.index.group[&id].clone();
            self.index.remove(id);
            self.index.insert(id, group.clone(), c);
            self.redo_from(robot_of(id), &group, &mut pending);
        }
        let first = self.robots.keys().next().copied().unwrap_or(0);
        self.refresh(first, &[], &[], pending)
    }

    /// Re-read these nodes and edges of `robot` and update everything that depends on them.
    fn refresh(
        &mut self,
        robot: u8,
        nodes: &[u64],
        edges: &[EdgeKey],
        mut pending: Pending,
    ) -> Change {
        self.update_nodes(robot, nodes, &mut pending);
        self.update_edges(robot, edges, &mut pending);
        self.rematch(&mut pending);
        let pairs = self.reroute_edges(&mut pending);
        self.write_topology(pairs, &mut pending);
        pending.change
    }

    /// Step 1: copy robot nodes into the station frame and the spatial index.
    fn update_nodes(&mut self, robot: u8, ids: &[u64], pending: &mut Pending) {
        let transform = self.robots[&robot].transform;
        for &local in ids {
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
            // out with the old version...
            if let Some(group) = self.index.group.get(&id).cloned() {
                self.redo_from(robot, &group, pending);
                self.index.remove(id);
                if let Some(set) = self.members.get_mut(&(robot, group)) {
                    set.remove(&id);
                }
                if self.merges.remove(&id).is_some() {
                    pending.change.merges_changed += 1;
                }
            }
            // ...in with the new
            match new {
                Some(n) => {
                    if let Some(group) = self.group_of(&n) {
                        self.redo_from(robot, &group, pending);
                        let p = add(position(&n).unwrap(), self.offset(id));
                        self.index.insert(id, group.clone(), p);
                        self.members.entry((robot, group)).or_default().insert(id);
                    }
                    self.nodes.insert(id, n);
                }
                None => {
                    self.nodes.remove(&id);
                }
            }
            pending.nodes.insert(id);
        }
    }

    /// Step 2: copy robot edges.
    fn update_edges(&mut self, robot: u8, keys: &[EdgeKey], pending: &mut Pending) {
        for &(s, t) in keys {
            let key = edge_key(global_id(robot, s), global_id(robot, t));
            let edge = self.robots[&robot].decoder.edges.get(&edge_key(s, t));
            match edge.map(edge_info) {
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
            pending.edges.insert(key);
        }
    }

    /// Step 3: redo the matchings that may have changed, lowest robot first (a robot only
    /// matches against lower robots, so their result must be final first).
    fn rematch(&mut self, pending: &mut Pending) {
        while let Some((robot, group)) = pending.groups.pop_first() {
            let ids: Vec<u64> = self
                .members
                .get(&(robot, group.clone()))
                .map(|s| s.iter().copied().collect())
                .unwrap_or_default();
            let wanted = self.matching(robot, &group, &ids);
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
                pending.change.merges_changed += 1;
                pending.nodes.insert(id);
                changed = true;
            }
            // the kept nodes changed, and later robots match against them
            if changed {
                self.redo_from(robot.saturating_add(1), &group, pending);
            }
        }
    }

    /// Where each of `robot`'s nodes in `ids` merges: into a kept node (not merged away) of a
    /// lower robot, in the same group, within the rule's radius.
    fn matching(&self, robot: u8, group: &Group, ids: &[u64]) -> BTreeMap<u64, u64> {
        let radius = self.config.rules[group.rule].radius;
        let mut pairs = Vec::new();
        for id in ids {
            let p = self.index.pos[id];
            for other in self.index.near(group, p, radius) {
                if robot_of(other) >= robot || self.merges.contains_key(&other) {
                    continue;
                }
                let d = distance(p, self.index.pos[&other]);
                if d <= radius {
                    pairs.push((d, *id, other));
                }
            }
        }
        assign(pairs, self.config.one_to_one, self.config.max_matching)
    }

    /// Step 4: point each affected robot edge at the station edge between the nodes its ends
    /// now live on. Returns the station edges that may have changed.
    fn reroute_edges(&mut self, pending: &mut Pending) -> BTreeSet<EdgeKey> {
        for id in &pending.nodes {
            if let Some(set) = self.incident.get(id) {
                pending.edges.extend(set);
            }
        }
        let mut pairs = BTreeSet::new();
        for &key in &pending.edges {
            if let Some(old) = self.station_edge_of.remove(&key) {
                self.station_edges.get_mut(&old).unwrap().remove(&key);
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
            self.station_edges.entry(pair).or_default().insert(key);
            self.station_edge_of.insert(key, pair);
            pairs.insert(pair);
        }
        pairs
    }

    /// Step 5: apply the changes to Yggdrasil: edges out, nodes out, nodes in or moved, edges in.
    fn write_topology(&mut self, mut pairs: BTreeSet<EdgeKey>, pending: &mut Pending) {
        let change = &mut pending.change;
        for pair in &pairs {
            if self.station_edges.get(pair).is_some_and(|s| s.is_empty()) {
                self.station_edges.remove(pair);
                if self.topology.remove_edge(pair.0, pair.1) {
                    change.edges_removed += 1;
                }
            }
        }
        for &id in &pending.nodes {
            let wanted = if self.merges.contains_key(&id) {
                None
            } else {
                self.nodes.get(&id)
            };
            match (wanted, self.topology.layer_of(id)) {
                (None, Some(_)) => {
                    self.topology.remove_node(id);
                    change.nodes_removed += 1;
                }
                (Some(n), Some(layer)) if layer != layer_key(n) => {
                    // moved layer: re-add it; its edges come back below
                    pairs.extend(
                        self.topology
                            .neighbors(id)
                            .into_iter()
                            .map(|o| edge_key(id, o)),
                    );
                    self.topology.remove_node(id);
                    self.topology.add_node(id, layer_key(n), position(n));
                    change.nodes_updated += 1;
                }
                (Some(n), Some(_)) => {
                    self.topology.set_position(id, position(n));
                    change.nodes_updated += 1;
                }
                (Some(n), None) => {
                    self.topology.add_node(id, layer_key(n), position(n));
                    change.nodes_added += 1;
                }
                (None, None) => {}
            }
        }
        for pair in &pairs {
            if self.station_edges.contains_key(pair) && self.topology.add_edge(pair.0, pair.1) {
                change.edges_added += 1;
            }
        }
    }

    // ---- helpers ----

    /// The merge group of a node, if a rule covers it and it has a position.
    fn group_of(&self, n: &Value) -> Option<Group> {
        position(n)?;
        let rule = self
            .config
            .rules
            .iter()
            .position(|r| r.layer == layer_key(n))?;
        let label = if self.config.rules[rule].same_label {
            label(n)
        } else {
            String::new()
        };
        Some(Group { rule, label })
    }

    /// Mark `group`'s matching to be redone for robot `from` and every later robot.
    fn redo_from(&self, from: u8, group: &Group, pending: &mut Pending) {
        for r in self.robots.keys().filter(|r| **r >= from) {
            pending.groups.insert((*r, group.clone()));
        }
    }

    fn offset(&self, id: u64) -> [f64; 3] {
        self.offsets.get(&id).copied().unwrap_or_default()
    }

    /// The node a robot node lives on in the station graph.
    fn resolve(&self, id: u64) -> u64 {
        self.merges.get(&id).copied().unwrap_or(id)
    }

    fn header(&self) -> Value {
        self.robots
            .values()
            .map(|r| &r.decoder.header)
            .find(|h| h.is_object())
            .cloned()
            .unwrap_or_else(|| json!({}))
    }

    // ---- output ----

    /// The station graph as Spark-DSG JSON. Each edge also lists the robots it came from.
    pub fn export(&self) -> Value {
        let mut ids: Vec<u64> = self.topology.node_ids().collect();
        ids.sort();
        let mut pairs: Vec<&EdgeKey> = self.station_edges.keys().collect();
        pairs.sort();
        let edge = |p: &EdgeKey| {
            let sources = &self.station_edges[p];
            let robots: BTreeSet<u8> = sources.iter().map(|k| robot_of(k.0)).collect();
            let info = &self.raw_edges[sources.first().unwrap()];
            json!({"source": p.0, "target": p.1, "info": info, "robots": robots})
        };
        let mut g = self.header();
        g["nodes"] = json!(ids.iter().map(|id| &self.nodes[id]).collect::<Vec<_>>());
        g["edges"] = json!(pairs.into_iter().map(edge).collect::<Vec<_>>());
        g
    }

    /// Every robot node's position after drift correction ([`deform`]), with the merges in
    /// `closure_layers` as loop closures.
    pub fn corrected_positions(&self, closure_layers: &[LayerKey]) -> HashMap<u64, [f64; 3]> {
        let positions: HashMap<u64, [f64; 3]> = self
            .nodes
            .iter()
            .filter_map(|(id, n)| Some((*id, position(n)?)))
            .collect();
        let edges: Vec<EdgeKey> = self.raw_edges.keys().copied().collect();
        let is_closure = |id| {
            self.layer_of_robot_node(id)
                .is_some_and(|l| closure_layers.contains(&l))
        };
        deform::correct(
            &positions,
            &edges,
            &self.merges,
            robot_of,
            is_closure,
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

    // ---- frame alignment input ----

    /// Labelled points for aligning `robot`: its own in its own frame, and every lower robot's
    /// in the station frame. Objects are labelled by class; a place with objects attached is
    /// labelled by the sorted classes of those objects, so places help when few objects are
    /// shared.
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

    /// [`Station::alignment_points`] with objects only.
    pub fn alignment_objects(&self, robot: u8) -> (Vec<align::Labelled>, Vec<align::Labelled>) {
        let object = |n: &Value| {
            (layer_key(n) == OBJECTS)
                .then(|| Some((label(n), position(n)?)))
                .flatten()
        };
        let mine = self.robots[&robot]
            .decoder
            .nodes
            .values()
            .filter_map(object)
            .collect();
        let theirs = self
            .nodes
            .iter()
            .filter(|(id, _)| robot_of(**id) < robot)
            .filter_map(|(_, n)| object(n))
            .collect();
        (mine, theirs)
    }
}

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
                let id = p["id"].as_u64().unwrap_or(0);
                labels.entry(id).or_default().push(label(o));
            }
        }
    }
    labels
        .into_iter()
        .filter_map(|(id, mut l)| {
            l.sort();
            Some((format!("place:{}", l.join(",")), position(node(id)?)?))
        })
        .collect()
}

#[cfg(test)]
mod tests;
