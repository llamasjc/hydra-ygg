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
| `crates/ygg-bridge` | cxx bridge: Rust mirror API callable from C++ |
| `ros/hydra_ygg_sink` | colcon package with Hydra's `YggdrasilSink` |

## Setup

`scene-graph` comes from the public Yggdrasil repo (github.com/brdsdsu/Yggdrasil), pinned to a
commit in `Cargo.toml`. `cargo test --workspace` fetches it.

C++ smoke test (what CI runs):

```bash
cargo build --release -p ygg-bridge
g++ -std=c++17 -O2 crates/ygg-bridge/tests/smoke.cpp -Itarget/cxxbridge \
    target/release/libygg_bridge.a -lpthread -ldl -o smoke
./smoke              # built-in graph
./smoke dsg.json     # any Hydra export
```

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
