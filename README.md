# hydra-ygg

Run [Hydra](https://github.com/MIT-SPARK/Hydra) and Hydra-Multi on top of the Yggdrasil scene graph.
Hydra and Yggdrasil are both used as published; nothing here patches either of them.

## Layout

| path | what |
|---|---|
| `crates/dsg-convert` | Spark-DSG JSON <-> Yggdrasil `SceneGraph`, lossless |
| `crates/tools` | `roundtrip` binary and `compare.py` |
| planned: `crates/ygg-delta` | change log, deltas and snapshots between robot and station |
| planned: `crates/ygg-bridge` | C++-facing API (cxx) that Hydra's sink calls |
| planned: `crates/station` | global multi-robot graph and backend |
| planned: `ros/hydra_ygg_sink` | colcon package with Hydra's `YggdrasilSink` |

## Setup

`scene-graph` is a path dependency on a checkout of the public Yggdrasil repo next to this one
(`../Yggdrasil-master`). Switch it to a pinned git dependency in `Cargo.toml` once the URL is set.

## Round-trip check

```sh
cargo run --release --bin roundtrip -- path/to/dsg.json /tmp/rt.json
python3 crates/tools/compare.py path/to/dsg.json /tmp/rt.json   # expect ROUND TRIP OK
```

On the stock-Hydra uH2 office baseline (5,156 nodes, 15,566 edges) this gives 0 differences and
a maximum position change of 1.9 µm (f32 coordinates).

## Known mapping gaps

* Yggdrasil nodes have one parent. In the office baseline, 9 objects link to two places; the
  first link becomes nesting and the second is kept as the `dsg_extra_edges` feature.
* Hydra's export does not link the building to the rooms, so the building has no children.
* All attributes except position are one JSON-string feature (`dsg_attrs`); `semantic_label`
  is also kept as its own feature for label queries.
