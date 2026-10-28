//! A copy of a Spark-DSG graph in Yggdrasil, updated from typed records instead of JSON.
//!
//! Each update brings every node as (id, layer key, position, semantic label, record) and
//! every edge as (source, target, record). A record is spark_dsg's own binary encoding of the
//! node or edge, opaque here: the mirror stores it and hands it back so spark_dsg can rebuild
//! the graph exactly. Only nodes and edges whose record changed are touched.
//!
//! Where things live:
//! * structure (nodes, layers, edges, nesting) and positions: Yggdrasil, via [`Topology`]
//! * semantic label: Yggdrasil feature `semantic_label`
//! * node record: Yggdrasil feature `dsg_record` (hex)
//! * edge record: a map next to the Yggdrasil graph (Yggdrasil has nowhere to put data on a
//!   nesting edge)

use std::collections::hash_map::DefaultHasher;
use std::collections::{HashMap, HashSet};
use std::hash::{Hash, Hasher};

use crate::LayerKey;
use crate::topology::Topology;

pub const F_RECORD: &str = "dsg_record";
pub const F_LABEL: &str = crate::F_LABEL;

/// One node of an update.
pub struct NodeIn<'a> {
    pub id: u64,
    pub key: LayerKey,
    pub pos: Option<[f64; 3]>,
    pub label: Option<u64>,
    pub record: &'a [u8],
}

/// One edge of an update.
pub struct EdgeIn<'a> {
    pub source: u64,
    pub target: u64,
    pub record: &'a [u8],
}

#[derive(Debug, Default, Clone, PartialEq)]
pub struct Stats {
    pub nodes_added: usize,
    pub nodes_updated: usize,
    pub nodes_removed: usize,
    pub edges_added: usize,
    pub edges_updated: usize,
    pub edges_removed: usize,
    /// Which nodes and edges this update added or changed, and which it removed (a node that
    /// changed layer is in both).
    pub changed_nodes: Vec<u64>,
    pub removed_nodes: Vec<u64>,
    pub changed_edges: Vec<(u64, u64)>,
    pub removed_edges: Vec<(u64, u64)>,
}

struct NodeState {
    key: LayerKey,
    pos: Option<[f64; 3]>,
    label: Option<u64>,
    hash: u64,
}

pub struct TypedMirror {
    pub topology: Topology,
    header: Vec<u8>,
    nodes: HashMap<u64, NodeState>,
    edges: HashMap<(u64, u64), (u64, u64, Vec<u8>)>,
}

fn hash(bytes: &[u8]) -> u64 {
    let mut h = DefaultHasher::new();
    bytes.hash(&mut h);
    h.finish()
}

fn edge_key(s: u64, t: u64) -> (u64, u64) {
    (s.min(t), s.max(t))
}

const HEX: &[u8; 16] = b"0123456789abcdef";

pub fn to_hex(bytes: &[u8]) -> String {
    let mut out = Vec::with_capacity(bytes.len() * 2);
    for b in bytes {
        out.push(HEX[(b >> 4) as usize]);
        out.push(HEX[(b & 15) as usize]);
    }
    String::from_utf8(out).unwrap()
}

fn from_hex(text: &str, out: &mut Vec<u8>) {
    let digit = |c: u8| match c {
        b'0'..=b'9' => c - b'0',
        _ => c - b'a' + 10,
    };
    out.extend(
        text.as_bytes()
            .chunks_exact(2)
            .map(|p| (digit(p[0]) << 4) | digit(p[1])),
    );
}

impl Default for TypedMirror {
    fn default() -> Self {
        TypedMirror {
            topology: Topology::new(crate::incremental::HYDRA_LAYERS),
            header: Vec::new(),
            nodes: HashMap::new(),
            edges: HashMap::new(),
        }
    }
}

impl TypedMirror {
    pub fn num_nodes(&self) -> usize {
        self.nodes.len()
    }

    pub fn num_edges(&self) -> usize {
        self.edges.len()
    }

    /// Bring the mirror to this update: `header` is the graph's own encoded header (layers,
    /// names, metadata), then every node and edge the graph has now.
    pub fn apply(&mut self, header: &[u8], nodes: &[NodeIn], edges: &[EdgeIn]) -> Stats {
        let mut stats = Stats::default();
        self.header = header.to_vec();

        // Edges that are gone, or whose ends moved layer (re-added below).
        let new_edges: HashMap<(u64, u64), &EdgeIn> = edges
            .iter()
            .map(|e| (edge_key(e.source, e.target), e))
            .collect();
        let new_nodes: HashMap<u64, &NodeIn> = nodes.iter().map(|n| (n.id, n)).collect();
        let relayered: HashSet<u64> = nodes
            .iter()
            .filter(|n| self.nodes.get(&n.id).is_some_and(|o| o.key != n.key))
            .map(|n| n.id)
            .collect();
        let gone: Vec<(u64, u64)> = self
            .edges
            .keys()
            .filter(|k| {
                !new_edges.contains_key(k) || relayered.contains(&k.0) || relayered.contains(&k.1)
            })
            .copied()
            .collect();
        for k in gone {
            self.topology.remove_edge(k.0, k.1);
            self.edges.remove(&k);
            stats.edges_removed += 1;
            stats.removed_edges.push(k);
        }

        // Nodes that are gone or changed layer.
        let stale: Vec<u64> = self
            .nodes
            .keys()
            .filter(|id| !new_nodes.contains_key(id) || relayered.contains(id))
            .copied()
            .collect();
        for id in stale {
            self.topology.remove_node(id);
            self.nodes.remove(&id);
            stats.nodes_removed += 1;
            stats.removed_nodes.push(id);
        }

        // New and changed nodes.
        for n in nodes {
            let h = hash(n.record);
            match self.nodes.get(&n.id) {
                Some(old) if old.hash == h => continue,
                Some(old) => {
                    if old.pos != n.pos {
                        self.topology.set_position(n.id, n.pos);
                    }
                    if old.label != n.label {
                        self.topology
                            .set_feature(n.id, F_LABEL, n.label.map(|l| l.to_string()));
                    }
                    stats.nodes_updated += 1;
                }
                None => {
                    self.topology.add_node(n.id, n.key, n.pos);
                    if let Some(l) = n.label {
                        self.topology
                            .set_feature(n.id, F_LABEL, Some(l.to_string()));
                    }
                    stats.nodes_added += 1;
                }
            }
            self.topology
                .set_feature(n.id, F_RECORD, Some(to_hex(n.record)));
            stats.changed_nodes.push(n.id);
            self.nodes.insert(
                n.id,
                NodeState {
                    key: n.key,
                    pos: n.pos,
                    label: n.label,
                    hash: h,
                },
            );
        }

        // New and changed edges, in a fixed order so nesting does not depend on hashing.
        let mut keys: Vec<&(u64, u64)> = new_edges.keys().collect();
        keys.sort();
        for k in keys {
            let e = new_edges[k];
            match self.edges.get_mut(k) {
                Some(old) => {
                    if old.2 != e.record {
                        *old = (e.source, e.target, e.record.to_vec());
                        stats.edges_updated += 1;
                        stats.changed_edges.push((e.source, e.target));
                    }
                }
                None => {
                    self.topology.add_edge(e.source, e.target);
                    self.edges
                        .insert(*k, (e.source, e.target, e.record.to_vec()));
                    stats.edges_added += 1;
                    stats.changed_edges.push((e.source, e.target));
                }
            }
        }
        stats
    }

    pub fn header(&self) -> &[u8] {
        &self.header
    }

    /// Every node record, read back out of Yggdrasil, concatenated.
    pub fn node_records(&self) -> Vec<u8> {
        let mut out = Vec::new();
        let mut ids: Vec<u64> = self.topology.node_ids().collect();
        ids.sort();
        for id in ids {
            if let Some(hex) = self.topology.feature(id, F_RECORD) {
                from_hex(hex, &mut out);
            }
        }
        out
    }

    /// Every edge record, concatenated. Only edges Yggdrasil also holds are returned.
    pub fn edge_records(&self) -> Vec<u8> {
        let mut keys: Vec<&(u64, u64)> = self.edges.keys().collect();
        keys.sort();
        let mut out = Vec::new();
        for k in keys {
            if self.topology.has_edge(k.0, k.1) {
                out.extend_from_slice(&self.edges[k].2);
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn node(id: u64, layer: i64, x: f64, tag: u8) -> (u64, LayerKey, [f64; 3], Vec<u8>) {
        (id, (layer, 0), [x, 0.0, 0.0], vec![tag, id as u8])
    }

    #[test]
    fn follows_changes_and_reads_back() {
        let mut m = TypedMirror::default();
        let apply = |m: &mut TypedMirror,
                     nodes: &[(u64, LayerKey, [f64; 3], Vec<u8>)],
                     edges: &[(u64, u64)]| {
            let n: Vec<NodeIn> = nodes
                .iter()
                .map(|(id, key, p, r)| NodeIn {
                    id: *id,
                    key: *key,
                    pos: Some(*p),
                    label: Some(7),
                    record: r,
                })
                .collect();
            let recs: Vec<Vec<u8>> = edges
                .iter()
                .map(|(s, t)| vec![9, *s as u8, *t as u8])
                .collect();
            let e: Vec<EdgeIn> = edges
                .iter()
                .zip(&recs)
                .map(|((s, t), r)| EdgeIn {
                    source: *s,
                    target: *t,
                    record: r,
                })
                .collect();
            m.apply(b"hdr", &n, &e)
        };

        let a = [
            node(1, 4, 0.0, 0),
            node(10, 3, 1.0, 0),
            node(11, 3, 2.0, 0),
            node(20, 2, 1.5, 0),
        ];
        let s = apply(&mut m, &a, &[(1, 10), (1, 11), (10, 11), (10, 20)]);
        assert_eq!((s.nodes_added, s.edges_added), (4, 4));
        assert_eq!(s.changed_nodes.len(), 4);
        assert_eq!(m.topology.parent_of(20), Some(10));

        // unchanged: nothing touched
        let s = apply(&mut m, &a, &[(1, 10), (1, 11), (10, 11), (10, 20)]);
        assert_eq!(s, Stats::default());

        // place 10 gone, place 11 moved and changed, object 20 now under 11
        let b = [node(1, 4, 0.0, 0), node(11, 3, 2.5, 1), node(20, 2, 1.5, 0)];
        let s = apply(&mut m, &b, &[(1, 11), (11, 20)]);
        assert_eq!(
            (
                s.nodes_removed,
                s.nodes_updated,
                s.edges_removed,
                s.edges_added
            ),
            (1, 1, 3, 1)
        );
        assert_eq!(
            (s.removed_nodes.clone(), s.changed_nodes.clone()),
            (vec![10], vec![11])
        );
        assert_eq!(s.changed_edges, vec![(11, 20)]);
        assert_eq!(m.topology.parent_of(20), Some(11));
        assert!(m.topology.has_edge(1, 11) && !m.topology.has_node(10));

        // records come back exactly, in id order
        assert_eq!(
            m.node_records(),
            [vec![0, 1], vec![1, 11], vec![0, 20]].concat()
        );
        assert_eq!(m.edge_records(), [vec![9, 1, 11], vec![9, 11, 20]].concat());
        assert_eq!(m.header(), b"hdr");
    }
}
