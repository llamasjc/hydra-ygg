//! C++ entry point for mirroring Hydra's scene graph into Yggdrasil.
//!
//! Two ways in: `apply_dsg_json` diffs whole snapshots, and `apply_ops` replays the node and edge
//! operations Hydra performed (reported by the spark_dsg observer patch), taking attributes from
//! the snapshot because Hydra edits those in place.

use std::collections::{BTreeMap, HashMap, HashSet};
use std::time::Instant;

use dsg_convert::compare::compare;
use serde_json::Value;

#[cxx::bridge(namespace = "hydra_ygg")]
mod ffi {
    struct UpdateStats {
        nodes: usize,
        nodes_added: usize,
        nodes_updated: usize,
        nodes_removed: usize,
        edges_added: usize,
        edges_removed: usize,
        rebuilt: bool,
        apply_us: u64,
    }

    /// One structural change to the spark_dsg graph. `kind`: 0 node added, 1 node removed,
    /// 2 edge added, 3 edge removed, 4 graph cleared. `target` is unused for node ops.
    #[derive(Clone, Copy, Debug)]
    struct GraphOp {
        kind: u8,
        source: u64,
        target: u64,
    }

    struct OpStats {
        ops: usize,
        nodes_added: usize,
        nodes_removed: usize,
        edges_added: usize,
        edges_removed: usize,
        clears: usize,
        /// Nodes or edges the ops say exist but the snapshot lacks (a reporting bug).
        unknown: usize,
        update: UpdateStats,
    }

    /// A search result: the id given to `NodeIndex::add` and its Euclidean distance.
    struct Neighbor {
        id: u64,
        distance: f64,
    }

    /// One node of a typed update. Its record is `bytes[offset..offset + len]` of the update's
    /// node buffer: spark_dsg's binary encoding of the node, stored as is.
    struct NodeRecord {
        id: u64,
        layer_key: i64,
        has_pos: bool,
        x: f64,
        y: f64,
        z: f64,
        has_label: bool,
        label: u64,
        offset: usize,
        len: usize,
    }

    /// One edge of a typed update; record as for `NodeRecord`.
    struct EdgeRecord {
        source: u64,
        target: u64,
        offset: usize,
        len: usize,
    }

    struct TypedStats {
        nodes: usize,
        edges: usize,
        nodes_added: usize,
        nodes_updated: usize,
        nodes_removed: usize,
        edges_added: usize,
        edges_updated: usize,
        edges_removed: usize,
        apply_us: u64,
    }

    struct CompareResult {
        equal: bool,
        missing_nodes: usize,
        extra_nodes: usize,
        missing_edges: usize,
        extra_edges: usize,
        mismatches: usize,
        max_position_error: f64,
        summary: String,
    }

    extern "Rust" {
        /// A spark_dsg graph copied into Yggdrasil from typed records (see dsg_convert::typed).
        type TypedStore;
        fn new_typed_store() -> Box<TypedStore>;
        fn apply(
            self: &mut TypedStore,
            header: &[u8],
            nodes: &[NodeRecord],
            node_bytes: &[u8],
            edges: &[EdgeRecord],
            edge_bytes: &[u8],
        ) -> Result<TypedStats>;
        /// From now on append every update's delta (what changed, as a station delta) to the
        /// file at `path`, one JSON line per update, stamped with the `set_time` time
        /// (gzip-compressed when `path` ends in .gz).
        fn record_to(self: &mut TypedStore, path: &str) -> Result<()>;
        /// The time of the next `apply`, in nanoseconds.
        fn set_time(self: &mut TypedStore, timestamp_ns: u64);
        /// The header given to the last `apply`.
        fn header(self: &TypedStore) -> Vec<u8>;
        /// Every node record, read back out of Yggdrasil, concatenated.
        fn node_records(self: &TypedStore) -> Vec<u8>;
        /// Every edge record, concatenated.
        fn edge_records(self: &TypedStore) -> Vec<u8>;

        type Mirror;
        fn new_mirror() -> Box<Mirror>;
        fn apply_dsg_json(self: &mut Mirror, json: &str) -> Result<UpdateStats>;
        fn apply_ops(self: &mut Mirror, snapshot_json: &str, ops: &[GraphOp]) -> Result<OpStats>;
        fn edge_audit(self: &Mirror) -> String;
        fn export_dsg_json(self: &Mirror) -> String;
        fn compare_with(self: &Mirror, json: &str, tol: f64) -> Result<CompareResult>;
        fn save(self: &Mirror, path: &str) -> Result<()>;

        /// A spark_dsg graph's structure kept in Yggdrasil, edited one operation at a time
        /// (see dsg_convert::topology). Layer keys are packed as layer * 1000 + partition.
        type Topology;
        fn new_topology(layer_keys: &[i64]) -> Box<Topology>;
        fn add_node(
            self: &mut Topology,
            id: u64,
            layer_key: i64,
            has_pos: bool,
            x: f64,
            y: f64,
            z: f64,
        ) -> bool;
        fn remove_node(self: &mut Topology, id: u64) -> bool;
        fn add_edge(self: &mut Topology, source: u64, target: u64) -> bool;
        fn remove_edge(self: &mut Topology, source: u64, target: u64) -> bool;
        fn clear(self: &mut Topology);
        fn has_node(self: &Topology, id: u64) -> bool;
        fn has_edge(self: &Topology, source: u64, target: u64) -> bool;
        fn num_nodes(self: &Topology) -> usize;
        fn num_edges(self: &Topology) -> usize;
        /// Layer key (packed) of a node, or -1 if missing.
        fn layer_of(self: &Topology, id: u64) -> i64;
        /// Yggdrasil nesting parent of a node, or u64::MAX if none.
        fn parent_of(self: &Topology, id: u64) -> u64;

        /// Yggdrasil-backed nearest-node search over groups of points (one indexed layer per
        /// group, e.g. per semantic label); replaces Hydra's nanoflann NearestNodeFinder.
        type NodeIndex;
        fn new_node_index() -> Box<NodeIndex>;
        fn add(self: &mut NodeIndex, group: i64, id: u64, x: f64, y: f64, z: f64) -> Result<()>;
        fn has_group(self: &NodeIndex, group: i64) -> bool;
        /// Ids of the `count` nearest nodes of `group`, nearest first; `skip_first` drops the
        /// nearest one (the query node itself), as NearestNodeFinder::find does.
        fn nearest(
            self: &NodeIndex,
            group: i64,
            x: f64,
            y: f64,
            z: f64,
            count: usize,
            skip_first: bool,
        ) -> Vec<u64>;
        /// The `count` nearest nodes of `group`, nearest first.
        fn nearest_n(
            self: &NodeIndex,
            group: i64,
            x: f64,
            y: f64,
            z: f64,
            count: usize,
        ) -> Vec<Neighbor>;
        /// All nodes of `group` within `radius` (Euclidean), nearest first.
        fn within(
            self: &NodeIndex,
            group: i64,
            x: f64,
            y: f64,
            z: f64,
            radius: f64,
        ) -> Vec<Neighbor>;
    }
}

pub struct TypedStore {
    inner: dsg_convert::typed::TypedMirror,
    recorder: Option<Recorder>,
    time_ns: u64,
}

/// Writes each update's delta in the station's delta format, with each node's record (hex)
/// so the station can rebuild real spark_dsg nodes.
struct Recorder {
    out: Box<dyn std::io::Write + Send>,
    seq: u64,
    header: Vec<u8>,
}

/// Connect to a station, waiting up to a minute for it to come up.
fn connect(addr: &str) -> Result<std::net::TcpStream, String> {
    let mut last = String::new();
    for _ in 0..120 {
        match std::net::TcpStream::connect(addr) {
            Ok(s) => {
                s.set_nodelay(true).ok();
                return Ok(s);
            }
            Err(e) => last = e.to_string(),
        }
        std::thread::sleep(std::time::Duration::from_millis(500));
    }
    Err(format!("station at {addr}: {last}"))
}

pub fn new_typed_store() -> Box<TypedStore> {
    Box::new(TypedStore {
        inner: Default::default(),
        recorder: None,
        time_ns: 0,
    })
}

fn slice(bytes: &[u8], offset: usize, len: usize) -> Result<&[u8], String> {
    bytes
        .get(offset..offset.saturating_add(len))
        .ok_or_else(|| format!("record {offset}+{len} outside buffer of {}", bytes.len()))
}

impl TypedStore {
    pub fn apply(
        &mut self,
        header: &[u8],
        nodes: &[ffi::NodeRecord],
        node_bytes: &[u8],
        edges: &[ffi::EdgeRecord],
        edge_bytes: &[u8],
    ) -> Result<ffi::TypedStats, String> {
        use dsg_convert::typed::{EdgeIn, NodeIn};
        let t = Instant::now();
        let nodes = nodes
            .iter()
            .map(|n| {
                Ok(NodeIn {
                    id: n.id,
                    key: unpack(n.layer_key),
                    pos: n.has_pos.then_some([n.x, n.y, n.z]),
                    label: n.has_label.then_some(n.label),
                    record: slice(node_bytes, n.offset, n.len)?,
                })
            })
            .collect::<Result<Vec<_>, String>>()?;
        let edges = edges
            .iter()
            .map(|e| {
                Ok(EdgeIn {
                    source: e.source,
                    target: e.target,
                    record: slice(edge_bytes, e.offset, e.len)?,
                })
            })
            .collect::<Result<Vec<_>, String>>()?;
        let s = self.inner.apply(header, &nodes, &edges);
        // A failed write (disk full, station gone) stops recording, never the mapping.
        if let Some(r) = &mut self.recorder {
            if let Err(e) = r.write(self.time_ns, header, &nodes, &edges, &s) {
                eprintln!("[YggdrasilStore] recording stopped: {e}");
                self.recorder = None;
            }
        }
        Ok(ffi::TypedStats {
            nodes: self.inner.num_nodes(),
            edges: self.inner.num_edges(),
            nodes_added: s.nodes_added,
            nodes_updated: s.nodes_updated,
            nodes_removed: s.nodes_removed,
            edges_added: s.edges_added,
            edges_updated: s.edges_updated,
            edges_removed: s.edges_removed,
            apply_us: t.elapsed().as_micros() as u64,
        })
    }
    pub fn record_to(&mut self, path: &str) -> Result<(), String> {
        // tcp://host:port streams the deltas to a running station (station_server) instead.
        if let Some(addr) = path.strip_prefix("tcp://") {
            let stream = connect(addr)?;
            self.recorder = Some(Recorder {
                out: Box::new(std::io::BufWriter::new(stream)),
                seq: 0,
                header: Vec::new(),
            });
            return Ok(());
        }
        let file = std::fs::File::create(path).map_err(|e| format!("{path}: {e}"))?;
        let file = std::io::BufWriter::new(file);
        // A path ending in .gz is gzip-compressed (about 6x smaller); each update is flushed
        // so a run stopped with Ctrl-C still leaves every finished update readable.
        let out: Box<dyn std::io::Write + Send> = if path.ends_with(".gz") {
            Box::new(flate2::write::GzEncoder::new(file, flate2::Compression::fast()))
        } else {
            Box::new(file)
        };
        self.recorder = Some(Recorder {
            out,
            seq: 0,
            header: Vec::new(),
        });
        Ok(())
    }
    pub fn set_time(&mut self, timestamp_ns: u64) {
        self.time_ns = timestamp_ns;
    }
    pub fn header(&self) -> Vec<u8> {
        self.inner.header().to_vec()
    }
    pub fn node_records(&self) -> Vec<u8> {
        self.inner.node_records()
    }
    pub fn edge_records(&self) -> Vec<u8> {
        self.inner.edge_records()
    }
}

impl Recorder {
    fn write(
        &mut self,
        time_ns: u64,
        header: &[u8],
        nodes: &[dsg_convert::typed::NodeIn],
        edges: &[dsg_convert::typed::EdgeIn],
        s: &dsg_convert::typed::Stats,
    ) -> std::io::Result<()> {
        use dsg_convert::typed::to_hex;
        use serde_json::json;
        use std::collections::{HashMap, HashSet};
        use std::io::Write;

        let changed: HashSet<u64> = s.changed_nodes.iter().copied().collect();
        let node_json: Vec<_> = nodes
            .iter()
            .filter(|n| changed.contains(&n.id))
            .map(|n| {
                let mut attrs = json!({});
                if let Some(p) = n.pos {
                    attrs["position"] = json!(p);
                }
                if let Some(l) = n.label {
                    attrs["semantic_label"] = json!(l);
                }
                json!({"id": n.id, "layer": n.key.0, "partition": n.key.1,
                       "attributes": attrs, "record": to_hex(n.record)})
            })
            .collect();
        let changed_edges: HashSet<(u64, u64)> = s.changed_edges.iter().copied().collect();
        let by_key: HashMap<(u64, u64), &dsg_convert::typed::EdgeIn> =
            edges.iter().map(|e| ((e.source, e.target), e)).collect();
        let mut edge_keys: Vec<&(u64, u64)> = changed_edges.iter().collect();
        edge_keys.sort();
        let edge_json: Vec<_> = edge_keys
            .into_iter()
            .map(|k| {
                let e = by_key[k];
                json!({"source": e.source, "target": e.target, "info": {},
                       "record": to_hex(e.record)})
            })
            .collect();
        self.seq += 1;
        let mut delta = json!({
            "seq": self.seq,
            "t": time_ns,
            "nodes": node_json,
            "removed_nodes": s.removed_nodes,
            "edges": edge_json,
            "removed_edges": s.removed_edges.iter().map(|(a, b)| [a, b]).collect::<Vec<_>>(),
        });
        if header != self.header.as_slice() {
            delta["header"] = json!({"record": to_hex(header)});
            self.header = header.to_vec();
        }
        writeln!(self.out, "{delta}")?;
        self.out.flush()
    }
}

pub struct Topology {
    inner: dsg_convert::topology::Topology,
}

fn unpack(key: i64) -> dsg_convert::LayerKey {
    (key.div_euclid(1000), key.rem_euclid(1000))
}

pub fn new_topology(layer_keys: &[i64]) -> Box<Topology> {
    Box::new(Topology {
        inner: dsg_convert::topology::Topology::new(layer_keys.iter().map(|k| unpack(*k))),
    })
}

impl Topology {
    #[allow(clippy::too_many_arguments)]
    pub fn add_node(&mut self, id: u64, key: i64, has_pos: bool, x: f64, y: f64, z: f64) -> bool {
        let pos = has_pos.then_some([x, y, z]);
        self.inner.add_node(id, unpack(key), pos)
    }
    pub fn remove_node(&mut self, id: u64) -> bool {
        self.inner.remove_node(id)
    }
    pub fn add_edge(&mut self, s: u64, t: u64) -> bool {
        self.inner.add_edge(s, t)
    }
    pub fn remove_edge(&mut self, s: u64, t: u64) -> bool {
        self.inner.remove_edge(s, t)
    }
    pub fn clear(&mut self) {
        self.inner.clear()
    }
    pub fn has_node(&self, id: u64) -> bool {
        self.inner.has_node(id)
    }
    pub fn has_edge(&self, s: u64, t: u64) -> bool {
        self.inner.has_edge(s, t)
    }
    pub fn num_nodes(&self) -> usize {
        self.inner.num_nodes()
    }
    pub fn num_edges(&self) -> usize {
        self.inner.num_edges()
    }
    pub fn layer_of(&self, id: u64) -> i64 {
        self.inner.layer_of(id).map_or(-1, |(l, p)| l * 1000 + p)
    }
    pub fn parent_of(&self, id: u64) -> u64 {
        self.inner.parent_of(id).unwrap_or(u64::MAX)
    }
}

pub struct NodeIndex {
    sg: scene_graph::SceneGraph,
    /// group -> (Yggdrasil layer id, Spark-DSG id of each Yggdrasil node)
    groups: HashMap<i64, (u32, HashMap<scene_graph::NodeId, u64>)>,
}

pub fn new_node_index() -> Box<NodeIndex> {
    Box::new(NodeIndex {
        sg: Default::default(),
        groups: HashMap::new(),
    })
}

impl NodeIndex {
    pub fn add(&mut self, group: i64, id: u64, x: f64, y: f64, z: f64) -> Result<(), String> {
        let sg = &mut self.sg;
        let (lid, ids) = self
            .groups
            .entry(group)
            .or_insert_with(|| (sg.new_indexed_layer().id, HashMap::new()));
        let anon = sg.new_coordinates(x as f32, y as f32, z as f32, Vec::new());
        let nid = sg
            .layer_mut(*lid)
            .map_err(|e| e.to_string())?
            .push_node(anon)
            .map_err(|e| e.to_string())?;
        ids.insert(nid, id);
        Ok(())
    }

    pub fn has_group(&self, group: i64) -> bool {
        self.groups.contains_key(&group)
    }

    #[allow(clippy::too_many_arguments)]
    pub fn nearest(
        &self,
        group: i64,
        x: f64,
        y: f64,
        z: f64,
        count: usize,
        skip_first: bool,
    ) -> Vec<u64> {
        self.nearest_n(group, x, y, z, count + usize::from(skip_first))
            .into_iter()
            .skip(usize::from(skip_first))
            .map(|n| n.id)
            .collect()
    }

    pub fn nearest_n(
        &self,
        group: i64,
        x: f64,
        y: f64,
        z: f64,
        count: usize,
    ) -> Vec<ffi::Neighbor> {
        self.query(group, [x, y, z], |layer, pos| {
            layer.nearest_nodes(pos, scene_graph::PosQueryCount::Count(count))
        })
    }

    pub fn within(&self, group: i64, x: f64, y: f64, z: f64, radius: f64) -> Vec<ffi::Neighbor> {
        let mut found = self.query(group, [x, y, z], |layer, pos| {
            layer.nodes_within(pos, radius as f32, scene_graph::PosQueryCount::All)
        });
        found.sort_by(|a, b| a.distance.total_cmp(&b.distance));
        found
    }

    fn query(
        &self,
        group: i64,
        [x, y, z]: [f64; 3],
        search: impl Fn(
            &scene_graph::Layer,
            &scene_graph::Coordinate,
        ) -> Vec<(scene_graph::NodeId, f32)>,
    ) -> Vec<ffi::Neighbor> {
        let Some((lid, ids)) = self.groups.get(&group) else {
            return Vec::new();
        };
        let Ok(layer) = self.sg.layer(*lid) else {
            return Vec::new();
        };
        search(
            layer,
            &scene_graph::Coordinate::new(x as f32, y as f32, z as f32),
        )
        .into_iter()
        .map(|(nid, d)| ffi::Neighbor {
            id: ids[&nid],
            distance: d as f64,
        })
        .collect()
    }
}

pub struct Mirror {
    inner: dsg_convert::incremental::Mirror,
    /// Edge additions seen by `apply_ops`, per "layer.partition -> layer.partition".
    audit: BTreeMap<String, usize>,
}

pub fn new_mirror() -> Box<Mirror> {
    Box::new(Mirror {
        inner: Default::default(),
        audit: BTreeMap::new(),
    })
}

fn pair(s: u64, t: u64) -> (u64, u64) {
    (s.min(t), s.max(t))
}

fn layer_name(node: &Value) -> String {
    format!(
        "{}.{}",
        node["layer"].as_i64().unwrap_or(-1),
        node["partition"].as_i64().unwrap_or(0)
    )
}

impl Mirror {
    /// Bring the mirror to Hydra's latest graph, changing only what differs from the last one.
    pub fn apply_dsg_json(&mut self, json: &str) -> Result<ffi::UpdateStats, String> {
        let input: Value = serde_json::from_str(json).map_err(|e| e.to_string())?;
        let t = Instant::now();
        let d = self.inner.apply(&input);
        Ok(ffi::UpdateStats {
            nodes: input["nodes"].as_array().map_or(0, Vec::len),
            nodes_added: d.nodes_added,
            nodes_updated: d.nodes_updated,
            nodes_removed: d.nodes_removed,
            edges_added: d.edges_added,
            edges_removed: d.edges_removed,
            rebuilt: d.rebuilt,
            apply_us: t.elapsed().as_micros() as u64,
        })
    }

    /// Replay Hydra's node and edge operations since the last call, then bring the mirror to
    /// the result. Node and edge contents come from `snapshot_json`; which nodes and edges exist
    /// comes only from the ops, so a missed op shows up when the mirror is compared.
    pub fn apply_ops(
        &mut self,
        snapshot_json: &str,
        ops: &[ffi::GraphOp],
    ) -> Result<ffi::OpStats, String> {
        let snapshot: Value = serde_json::from_str(snapshot_json).map_err(|e| e.to_string())?;
        let empty = Vec::new();
        let snap_nodes: HashMap<u64, &Value> = snapshot["nodes"]
            .as_array()
            .unwrap_or(&empty)
            .iter()
            .map(|n| (n["id"].as_u64().unwrap(), n))
            .collect();
        let snap_edges: HashMap<(u64, u64), &Value> = snapshot["edges"]
            .as_array()
            .unwrap_or(&empty)
            .iter()
            .map(|e| {
                let (s, t) = (e["source"].as_u64().unwrap(), e["target"].as_u64().unwrap());
                (pair(s, t), e)
            })
            .collect();

        let mut stats = ffi::OpStats {
            ops: ops.len(),
            nodes_added: 0,
            nodes_removed: 0,
            edges_added: 0,
            edges_removed: 0,
            clears: 0,
            unknown: 0,
            update: ffi::UpdateStats {
                nodes: 0,
                nodes_added: 0,
                nodes_updated: 0,
                nodes_removed: 0,
                edges_added: 0,
                edges_removed: 0,
                rebuilt: false,
                apply_us: 0,
            },
        };
        let mut nodes: HashSet<u64> = self.inner.node_ids().collect();
        let mut edges: HashSet<(u64, u64)> = self.inner.edge_pairs().collect();
        for op in ops {
            match op.kind {
                0 => {
                    nodes.insert(op.source);
                    stats.nodes_added += 1;
                }
                1 => {
                    nodes.remove(&op.source);
                    stats.nodes_removed += 1;
                }
                2 => {
                    edges.insert(pair(op.source, op.target));
                    stats.edges_added += 1;
                    if let (Some(a), Some(b)) =
                        (snap_nodes.get(&op.source), snap_nodes.get(&op.target))
                    {
                        let (a, b) = (layer_name(a), layer_name(b));
                        let (lo, hi) = if a <= b { (a, b) } else { (b, a) };
                        *self.audit.entry(format!("{lo} - {hi}")).or_default() += 1;
                    }
                }
                3 => {
                    edges.remove(&pair(op.source, op.target));
                    stats.edges_removed += 1;
                }
                4 => {
                    nodes.clear();
                    edges.clear();
                    stats.clears += 1;
                }
                k => return Err(format!("unknown graph op kind {k}")),
            }
        }

        // The graph the ops describe, with the snapshot's contents.
        let mut desired = snapshot.clone();
        let mut out_nodes = Vec::with_capacity(nodes.len());
        for id in &nodes {
            match snap_nodes.get(id) {
                Some(n) => out_nodes.push((*n).clone()),
                None => stats.unknown += 1,
            }
        }
        let mut out_edges = Vec::with_capacity(edges.len());
        for key in &edges {
            match snap_edges.get(key) {
                Some(e) => out_edges.push((*e).clone()),
                None => stats.unknown += 1,
            }
        }
        out_nodes.sort_by_key(|n| n["id"].as_u64());
        desired["nodes"] = Value::Array(out_nodes);
        desired["edges"] = Value::Array(out_edges);

        let t = Instant::now();
        let d = self.inner.apply(&desired);
        stats.update = ffi::UpdateStats {
            nodes: nodes.len(),
            nodes_added: d.nodes_added,
            nodes_updated: d.nodes_updated,
            nodes_removed: d.nodes_removed,
            edges_added: d.edges_added,
            edges_removed: d.edges_removed,
            rebuilt: d.rebuilt,
            apply_us: t.elapsed().as_micros() as u64,
        };
        Ok(stats)
    }

    /// Count of every kind of edge Hydra added, by the layers it joins.
    pub fn edge_audit(&self) -> String {
        self.audit
            .iter()
            .map(|(k, n)| format!("{k}: {n}\n"))
            .collect()
    }

    pub fn export_value(&self) -> Value {
        self.inner.export()
    }

    pub fn export_dsg_json(&self) -> String {
        self.export_value().to_string()
    }

    pub fn compare_with(&self, json: &str, tol: f64) -> Result<ffi::CompareResult, String> {
        let other: Value = serde_json::from_str(json).map_err(|e| e.to_string())?;
        let d = compare(&other, &self.export_value(), tol);
        Ok(ffi::CompareResult {
            equal: d.is_equal(),
            missing_nodes: d.missing_nodes,
            extra_nodes: d.extra_nodes,
            missing_edges: d.missing_edges,
            extra_edges: d.extra_edges,
            mismatches: d.mismatch_count,
            max_position_error: d.max_position_error,
            summary: d.mismatches.join("\n"),
        })
    }

    pub fn save(&self, path: &str) -> Result<(), String> {
        let bytes = serde_json::to_vec(&self.inner.sg).map_err(|e| e.to_string())?;
        std::fs::write(path, bytes).map_err(|e| e.to_string())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mirror_matches_its_input() {
        let json = r#"{"directed":false,"nodes":[
            {"id":1,"layer":3,"partition":0,"attributes":{"position":[1.0,2.0,3.0]}},
            {"id":2,"layer":3,"partition":0,"attributes":{"position":[0.0,0.0,0.0]}},
            {"id":3,"layer":4,"partition":0,"attributes":{"position":[0.5,0.5,0.5]}}],
            "edges":[{"source":1,"target":2,"info":{"weight":1.0}},
                     {"source":3,"target":1,"info":{"weight":1.0}}]}"#;
        let mut m = new_mirror();
        let stats = m.apply_dsg_json(json).unwrap();
        assert_eq!(
            (stats.nodes, stats.nodes_added, stats.edges_added),
            (3, 3, 2)
        );
        let r = m.compare_with(json, 1e-5).unwrap();
        assert!(r.equal, "{}", r.summary);
    }

    #[test]
    fn node_index_matches_brute_force() {
        let mut index = new_node_index();
        let mut pts = Vec::new();
        let mut seed = 12345u64;
        let mut rand = || {
            seed = seed
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            (seed >> 33) as f64 / (1u64 << 31) as f64 * 20.0
        };
        for id in 0..500u64 {
            let p = [rand(), rand(), rand()];
            index.add((id % 3) as i64, id, p[0], p[1], p[2]).unwrap();
            pts.push((id, p));
        }
        for q in 0..50u64 {
            let (qid, p) = pts[q as usize * 7];
            let group = (qid % 3) as i64;
            let mut brute: Vec<(f64, u64)> = pts
                .iter()
                .filter(|(id, _)| (id % 3) as i64 == group)
                .map(|(id, o)| {
                    let d: f64 = (0..3).map(|i| (o[i] - p[i]).powi(2)).sum();
                    (d, *id)
                })
                .collect();
            brute.sort_by(|a, b| a.partial_cmp(b).unwrap());
            let want: Vec<u64> = brute.iter().skip(1).take(3).map(|b| b.1).collect();
            assert_eq!(index.nearest(group, p[0], p[1], p[2], 3, true), want);
            let r = brute[3].0.sqrt() + 1e-3;
            let within: Vec<u64> = index
                .within(group, p[0], p[1], p[2], r)
                .iter()
                .map(|n| n.id)
                .collect();
            let want: Vec<u64> = brute
                .iter()
                .take_while(|b| b.0.sqrt() <= r)
                .map(|b| b.1)
                .collect();
            assert_eq!(within, want);
        }
        assert!(index.nearest(9, 0.0, 0.0, 0.0, 3, false).is_empty());
    }

    #[test]
    fn ops_follow_the_graph_and_catch_a_missed_op() {
        let snap = r#"{"directed":false,"nodes":[
            {"id":1,"layer":3,"partition":0,"attributes":{"position":[1.0,2.0,3.0]}},
            {"id":3,"layer":4,"partition":0,"attributes":{"position":[0.5,0.5,0.5]}}],
            "edges":[{"source":3,"target":1,"info":{"weight":1.0}}]}"#;
        let op = |kind, source, target| ffi::GraphOp {
            kind,
            source,
            target,
        };
        let mut m = new_mirror();
        // node 2 and its edge come and go between snapshots
        let ops = [
            op(0, 1, 0),
            op(0, 2, 0),
            op(0, 3, 0),
            op(2, 1, 2),
            op(2, 3, 1),
        ];
        let ops = [&ops[..], &[op(3, 1, 2), op(1, 2, 0)]].concat();
        let s = m.apply_ops(snap, &ops).unwrap();
        assert_eq!((s.nodes_added, s.edges_added, s.unknown), (3, 2, 0));
        assert!(m.compare_with(snap, 1e-5).unwrap().equal);
        assert_eq!(m.edge_audit(), "3.0 - 4.0: 1\n");

        // a node the snapshot has but no op added is reported as a difference
        let snap2 = snap.replace(
            r#"{"id":3,"#,
            r#"{"id":4,"layer":3,"partition":0,"attributes":{"position":[0.0,0.0,0.0]}},{"id":3,"#,
        );
        m.apply_ops(&snap2, &[]).unwrap();
        assert_eq!(m.compare_with(&snap2, 1e-5).unwrap().missing_nodes, 1);
    }
}
