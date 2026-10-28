//! A Spark-DSG graph's structure (which nodes exist, in which layer, and how they connect)
//! held in a Yggdrasil `SceneGraph` and edited one operation at a time.
//!
//! spark_dsg (with the hydra-yggdrasil backend patch) forwards every node and edge it adds or
//! removes here, and answers `hasNode`, `hasEdge`, `numNodes` and `numEdges` from it.
//!
//! Mapping:
//! * each Spark (layer, partition) is a Yggdrasil layer, ordered so higher Spark layers get
//!   higher ids; any layer may nest any layer with a lower Spark layer number
//! * an edge inside one (layer, partition) is a pair of Yggdrasil edges, one on each node
//! * an edge between layers nests the lower node under the higher one when the lower node
//!   has no parent yet
//! * any other edge (a second parent, or between partitions of one layer number) is kept in
//!   the `dsg_links` feature of both nodes

use std::collections::{BTreeMap, BTreeSet, HashMap};

use scene_graph::{Bounds, Coordinate, Feature, NodeId, SceneGraph};

use crate::{F_DSG_ID, LayerKey};

pub const F_LINKS: &str = "dsg_links";

#[derive(Default)]
pub struct Topology {
    sg: SceneGraph,
    layers: BTreeMap<LayerKey, u32>,
    key_of_layer: HashMap<u32, LayerKey>,
    /// Spark-DSG id -> Yggdrasil id. Yggdrasil holds the nodes; this only finds them.
    ids: HashMap<u64, NodeId>,
    dsg_of: HashMap<NodeId, u64>,
    num_edges: usize,
    /// How many times a new (layer, partition) forced the graph to be laid out again.
    pub rebuilds: usize,
}

/// Everything needed to replay a graph into a new layout.
struct Snapshot {
    nodes: Vec<(u64, LayerKey, Option<[f64; 3]>, Vec<Feature>)>,
    edges: Vec<(u64, u64)>,
}

impl Topology {
    pub fn new(keys: impl IntoIterator<Item = LayerKey>) -> Self {
        let mut t = Topology::default();
        t.layout(keys.into_iter().collect());
        t
    }

    pub fn graph(&self) -> &SceneGraph {
        &self.sg
    }

    fn layout(&mut self, keys: BTreeSet<LayerKey>) {
        self.sg = SceneGraph::default();
        self.layers.clear();
        self.key_of_layer.clear();
        self.ids.clear();
        self.dsg_of.clear();
        self.num_edges = 0;
        for key in &keys {
            let lid = self.sg.new_indexed_layer().id;
            self.layers.insert(*key, lid);
            self.key_of_layer.insert(lid, *key);
        }
        for (hi, hi_lid) in &self.layers {
            for (lo, lo_lid) in &self.layers {
                if lo.0 < hi.0 {
                    self.sg
                        .layer_mut(*hi_lid)
                        .unwrap()
                        .add_child(*lo_lid)
                        .unwrap();
                }
            }
        }
    }

    fn snapshot(&self) -> Snapshot {
        let mut nodes = Vec::new();
        let mut edges = BTreeSet::new();
        for layer in self.sg.layers() {
            let key = self.key_of_layer[&layer.id];
            for node in layer.all_nodes() {
                let id = self.dsg_of[&node.id];
                let pos = node.coordinates.first().map(|c| [c.x as f64, c.y as f64, c.z as f64]);
                let kept = node
                    .features
                    .iter()
                    .filter(|f| f.key() != F_DSG_ID && f.key() != F_LINKS)
                    .cloned()
                    .collect();
                nodes.push((id, key, pos, kept));
                for other in self.neighbors(id) {
                    edges.insert((id.min(other), id.max(other)));
                }
            }
        }
        Snapshot {
            nodes,
            edges: edges.into_iter().collect(),
        }
    }

    /// Lay the graph out again with `extra` added to its layers.
    fn rebuild(&mut self, extra: LayerKey) {
        let snap = self.snapshot();
        let mut keys: BTreeSet<LayerKey> = self.layers.keys().copied().collect();
        keys.insert(extra);
        self.layout(keys);
        for (id, key, pos, features) in snap.nodes {
            self.add_node(id, key, pos);
            let nid = self.ids[&id];
            self.sg.node_mut(nid).unwrap().features.extend(features);
        }
        for (s, t) in snap.edges {
            self.add_edge(s, t);
        }
        self.rebuilds += 1;
    }

    pub fn clear(&mut self) {
        let keys = self.layers.keys().copied().collect();
        self.layout(keys);
    }

    pub fn add_node(&mut self, id: u64, key: LayerKey, pos: Option<[f64; 3]>) -> bool {
        if self.ids.contains_key(&id) {
            return false;
        }
        if !self.layers.contains_key(&key) {
            self.rebuild(key);
        }
        let features = vec![Feature::new(F_DSG_ID, id)];
        let anon = match pos {
            Some([x, y, z]) => self.sg.new_coordinates(x as f32, y as f32, z as f32, features),
            None => self.sg.new_node(features),
        };
        let nid = self
            .sg
            .layer_mut(self.layers[&key])
            .unwrap()
            .push_node(anon)
            .unwrap();
        self.ids.insert(id, nid);
        self.dsg_of.insert(nid, id);
        true
    }

    pub fn remove_node(&mut self, id: u64) -> bool {
        if !self.ids.contains_key(&id) {
            return false;
        }
        for other in self.neighbors(id) {
            self.remove_edge(id, other);
        }
        let nid = self.ids.remove(&id).unwrap();
        self.dsg_of.remove(&nid);
        self.sg.del_node(nid).unwrap();
        true
    }

    /// Set (or with `None`, remove) one feature of node `id`.
    pub fn set_feature(&mut self, id: u64, key: &str, value: Option<String>) -> bool {
        let Some(&nid) = self.ids.get(&id) else {
            return false;
        };
        let mut node = self.sg.node_mut(nid).unwrap();
        match value {
            Some(v) => node.set_feature(Feature::new(key, v)),
            None => node.features.retain(|f| f.key() != key),
        }
        true
    }

    /// One feature of node `id`.
    pub fn feature(&self, id: u64, key: &str) -> Option<&str> {
        self.sg.node(*self.ids.get(&id)?).ok()?.feature(key).ok()
    }

    /// Move node `id` to `pos`, keeping its parent, children and edges.
    pub fn set_position(&mut self, id: u64, pos: Option<[f64; 3]>) -> bool {
        let Some(&nid) = self.ids.get(&id) else {
            return false;
        };
        // Yggdrasil keeps a parent's spatial index and bounds in sync only through nest and
        // detach, so the node leaves its parent while it moves.
        let parent = self.sg.node(nid).unwrap().pid;
        if let Some(p) = parent {
            self.sg.detach(nid).from(p).unwrap();
        }
        let coord = pos.map(|[x, y, z]| Coordinate::new(x as f32, y as f32, z as f32));
        let node = self.sg.node(nid).unwrap();
        let bounds = node
            .children()
            .iter()
            .filter_map(|c| self.sg.node(*c).ok()?.bounds)
            .fold(coord.map(Bounds::point), |acc, b| {
                Some(*acc.unwrap_or(Bounds::Unbounded).merge(&b))
            });
        let mut node = self.sg.node_mut(nid).unwrap();
        node.coordinates = coord.into_iter().collect();
        node.bounds = bounds;
        // NodeMut re-indexes the layer's position index when it is dropped.
        drop(node);
        if let Some(p) = parent {
            self.sg.nest(nid).under(p).unwrap();
        }
        true
    }

    /// Spark-DSG ids of every node.
    pub fn node_ids(&self) -> impl Iterator<Item = u64> + '_ {
        self.ids.keys().copied()
    }

    pub fn has_node(&self, id: u64) -> bool {
        self.ids.contains_key(&id)
    }

    pub fn layer_of(&self, id: u64) -> Option<LayerKey> {
        self.ids
            .get(&id)
            .map(|nid| self.key_of_layer[&nid.layer_id()])
    }

    pub fn num_nodes(&self) -> usize {
        self.ids.len()
    }

    pub fn num_edges(&self) -> usize {
        self.num_edges
    }

    /// The node `id` is nested under, if any.
    pub fn parent_of(&self, id: u64) -> Option<u64> {
        let node = self.sg.node(*self.ids.get(&id)?).ok()?;
        node.pid.map(|p| self.dsg_of[&p])
    }

    fn links(&self, nid: NodeId) -> Vec<u64> {
        self.sg
            .node(nid)
            .ok()
            .and_then(|n| n.feature(F_LINKS).ok().map(str::to_owned))
            .map(|v| serde_json::from_str(&v).unwrap())
            .unwrap_or_default()
    }

    fn set_links(&mut self, nid: NodeId, links: &[u64]) {
        let mut node = self.sg.node_mut(nid).unwrap();
        node.features.retain(|f| f.key() != F_LINKS);
        if !links.is_empty() {
            node.features
                .push(Feature::new(F_LINKS, serde_json::to_string(links).unwrap()));
        }
    }

    /// Every node `id` shares an edge with.
    pub fn neighbors(&self, id: u64) -> Vec<u64> {
        let Some(&nid) = self.ids.get(&id) else {
            return Vec::new();
        };
        let node = self.sg.node(nid).unwrap();
        let mut out: Vec<u64> = node.edges.iter().map(|e| self.dsg_of[&e.dst]).collect();
        out.extend(node.pid.map(|p| self.dsg_of[&p]));
        out.extend(node.children().iter().map(|c| self.dsg_of[c]));
        out.extend(self.links(nid));
        out
    }

    pub fn has_edge(&self, s: u64, t: u64) -> bool {
        let (Some(&a), Some(&b)) = (self.ids.get(&s), self.ids.get(&t)) else {
            return false;
        };
        let (na, nb) = (self.sg.node(a).unwrap(), self.sg.node(b).unwrap());
        na.edges.iter().any(|e| e.dst == b)
            || na.pid == Some(b)
            || nb.pid == Some(a)
            || self.links(a).contains(&t)
    }

    pub fn add_edge(&mut self, s: u64, t: u64) -> bool {
        if s == t || self.has_edge(s, t) {
            return false;
        }
        let (Some(&a), Some(&b)) = (self.ids.get(&s), self.ids.get(&t)) else {
            return false;
        };
        let (ka, kb) = (
            self.key_of_layer[&a.layer_id()],
            self.key_of_layer[&b.layer_id()],
        );
        if ka == kb {
            let layer = self.sg.layer_mut(a.layer_id()).unwrap();
            layer.add_edge(a, b, "").unwrap();
            layer.add_edge(b, a, "").unwrap();
        } else {
            let (child, parent) = if ka.0 < kb.0 { (a, b) } else { (b, a) };
            let nestable = ka.0 != kb.0 && self.sg.node(child).unwrap().pid.is_none();
            if nestable {
                self.sg.nest(child).under(parent).unwrap();
            } else {
                for (x, other) in [(a, t), (b, s)] {
                    let mut links = self.links(x);
                    links.push(other);
                    self.set_links(x, &links);
                }
            }
        }
        self.num_edges += 1;
        true
    }

    pub fn remove_edge(&mut self, s: u64, t: u64) -> bool {
        if !self.has_edge(s, t) {
            return false;
        }
        let (a, b) = (self.ids[&s], self.ids[&t]);
        let (na, nb) = (self.sg.node(a).unwrap(), self.sg.node(b).unwrap());
        if na.edges.iter().any(|e| e.dst == b) {
            let layer = self.sg.layer_mut(a.layer_id()).unwrap();
            layer.del_edge(a, b).unwrap();
            layer.del_edge(b, a).unwrap();
        } else if na.pid == Some(b) {
            self.sg.detach(a).from(b).unwrap();
        } else if nb.pid == Some(a) {
            self.sg.detach(b).from(a).unwrap();
        } else {
            for (x, other) in [(a, t), (b, s)] {
                let links: Vec<u64> = self.links(x).into_iter().filter(|l| *l != other).collect();
                self.set_links(x, &links);
            }
        }
        self.num_edges -= 1;
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn follows_nodes_edges_and_parents() {
        let mut t = Topology::new([(2, 0), (3, 0), (4, 0)]);
        assert!(t.add_node(1, (2, 0), Some([0.0, 0.0, 0.0])));
        assert!(!t.add_node(1, (2, 0), None));
        t.add_node(2, (2, 0), Some([1.0, 0.0, 0.0]));
        t.add_node(10, (3, 0), Some([0.5, 0.0, 0.0]));
        t.add_node(11, (3, 0), Some([2.0, 0.0, 0.0]));
        assert!(t.add_edge(1, 2)); // same layer
        assert!(t.add_edge(10, 1)); // nests 1 under 10
        assert!(t.add_edge(11, 1)); // second parent: a link
        assert!(!t.add_edge(1, 10));
        assert_eq!((t.num_nodes(), t.num_edges()), (4, 3));
        assert_eq!(t.parent_of(1), Some(10));
        assert!(t.has_edge(2, 1) && t.has_edge(1, 10) && t.has_edge(1, 11));

        // a new partition lays everything out again, keeping all of it
        t.add_node(50, (2, 97), None);
        assert!(t.add_edge(50, 1));
        assert_eq!(t.rebuilds, 1);
        assert_eq!((t.num_nodes(), t.num_edges()), (5, 4));
        assert_eq!(t.layer_of(50), Some((2, 97)));
        assert!(t.has_edge(2, 1) && t.has_edge(1, 10) && t.has_edge(1, 11) && t.has_edge(1, 50));

        // removing the parent keeps the child and its other edges
        assert!(t.remove_node(10));
        assert!(t.has_node(1) && !t.has_edge(1, 10));
        assert_eq!(t.parent_of(1), None);
        assert_eq!((t.num_nodes(), t.num_edges()), (4, 3));
        assert!(t.remove_edge(1, 11) && t.remove_edge(1, 2));
        assert_eq!(t.num_edges(), 1);
        t.clear();
        assert_eq!((t.num_nodes(), t.num_edges()), (0, 0));
    }
}
