# hydra-yggdrasil

Run [Hydra](https://github.com/MIT-SPARK/Hydra) and Hydra-Multi on top of the Yggdrasil scene graph.
Yggdrasil is used unmodified, straight from [upstream](https://github.com/brdsdsu/Yggdrasil)
(rev 3b17a94). Hydra and spark_dsg get small patches (`patches/`) that let
Hydra's backend keep its scene graph in Yggdrasil.

## Layout

| path | what |
|---|---|
| `crates/dsg-convert` | Spark-DSG JSON <-> Yggdrasil `SceneGraph`, lossless |
| `crates/tools` | `roundtrip` binary and `compare.py` |
| planned: `crates/ygg-bridge` | C++-facing API (cxx) that Hydra's sink calls |
| `crates/station` | robot deltas and the multi-robot station graph |
| `crates/ygg-bridge` | cxx bridge: Rust mirror API callable from C++ |
| `ros/hydra_ygg_sink` | colcon package: `YggdrasilStore` for the patched backend, and `ygg_mirror_node` |
| `patches/hydra` | Hydra patches adding the `HYDRA_GRAPH_BACKEND` switch |
| `patches/spark_dsg` | spark_dsg patch reporting every node and edge add/remove to an observer |

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
no position change.

## Known mapping gaps

* Yggdrasil nodes have one parent. In the office baseline, 9 objects link to two places; the
  first link becomes nesting and the second is kept as the `dsg_extra_edges` feature.
* Hydra's export does not link the building to the rooms, so the building has no children.
* All attributes except position are one JSON-string feature (`dsg_attrs`); `semantic_label`
  is also kept as its own feature for label queries.

## Hydra backend on Yggdrasil

`patches/hydra` adds a graph store hook to Hydra's backend. After every backend update,
`YggdrasilStore` copies the graph into Yggdrasil as typed records, with no JSON
(`dsg_convert::typed`): structure, positions and semantic labels go into Yggdrasil directly, and
each node also carries spark_dsg's own binary encoding of itself, so the graph read back out of
Yggdrasil is exactly the one Hydra built. Hydra edits attributes in place, so every node is
encoded each update, and Yggdrasil only changes where an encoding changed. Encoding is split across threads
(`HYDRA_YGG_THREADS`, default up to 8), and in `both` mode the Yggdrasil apply runs in the
background, so Hydra only waits for the encoding: about 10 ms per update on the 5,156-node office
graph with 4 threads, against about 280 ms for the old JSON path (`typed_check`). With `libhydra_ygg_sink.so` preloaded, `HYDRA_GRAPH_BACKEND` picks the backend
graph:

| value | published / saved graph |
|---|---|
| `spark` (default) | stock Hydra, spark_dsg only |
| `yggdrasil` | each update is stored in Yggdrasil and the graph sent downstream (ROS topic, visualizer, sinks) is read back out of Yggdrasil |
| `both` | spark_dsg is published, Yggdrasil is kept alongside and compared every `HYDRA_YGG_COMPARE_EVERY` updates |

The backend's own update functions (place and room updates, merging, loop-closure deformation)
still compute on spark_dsg; porting them to Yggdrasil is the next step.

```bash
scripts/apply_patches.sh /root/hydra_ws/src
colcon build --packages-above spark_dsg   # spark_dsg changed, so rebuild everything using it
HYDRA_GRAPH_BACKEND=yggdrasil \
LD_PRELOAD=/root/hydra_ws/install/hydra_ygg_sink/lib/libhydra_ygg_sink.so \
  ros2 launch hydra_ros uhumans2.launch.yaml
```

On shutdown the backend output folder also gets `dsg_yggdrasil.json` (native Yggdrasil),
`dsg_from_yggdrasil.json` (exported back to Spark-DSG JSON), `yggdrasil_compare.txt` and
`edge_audit.txt` (every edge Hydra added, counted by the layers it joins).

`ros/hydra_ygg_sink/test/observer_check.cpp` checks the spark_dsg patch on any `dsg.json`: it
edits a graph the ways the backend does and requires Yggdrasil, fed only the reported
operations, to equal spark_dsg after every step.

## Algorithms on Yggdrasil

Ported one at a time; each can be switched back to Hydra's own and, with `compare: true`,
runs Hydra's version alongside and reports any disagreement.

| Hydra piece | Yggdrasil version | switch |
|---|---|---|
| graph structure under every spark_dsg graph (which nodes exist, layers, edges, parents) | `YggdrasilGraphBackend` on `dsg_convert::topology` (patches/spark_dsg/0002) | `SPARK_DSG_BACKEND=spark\|yggdrasil\|both` |
| every nearest-node search (`NearestNodeFinder`: objects to places, place connections, frontiers, merges) | `YggdrasilNnBackend` (patches/hydra/0003) | `HYDRA_NN_BACKEND=nanoflann\|yggdrasil\|both` |
| merge-candidate search (`NearestNode`, `SemanticNearestNode`, nanoflann) | `YggdrasilNearestNode`, `YggdrasilSemanticNearestNode` (Yggdrasil indexed layers) | `scripts/set_merge_strategy.sh yggdrasil\|hydra\|off` |

The uhumans2 config ships with object merging off; `set_merge_strategy.sh` turns it on with
the chosen search.

### spark_dsg on Yggdrasil

With `patches/spark_dsg/0002` every `DynamicSceneGraph` (frontend, backend, clones) forwards each
node and edge it adds or removes to a Yggdrasil graph as it happens, and with
`SPARK_DSG_BACKEND=yggdrasil` answers `hasNode`, `hasEdge`, `numNodes` and `numEdges` from
Yggdrasil. `both` answers from spark_dsg and checks Yggdrasil on every query. Attributes stay in
spark_dsg's C++ objects (Hydra edits them through references), as do the per-node parent and
sibling sets the API hands out by reference.

Checks: spark_dsg's own test suite passes in all three modes (210 tests), and
`backend_check <dsg.json>` edits the office graph the ways Hydra does and asks every structure
question after each step (0 disagreements in 78,672 queries).

## Hydra-Multi station

`crates/station`: robots send deltas (only nodes and edges added, changed or removed since the
last update) instead of whole graphs. The station rebuilds each robot's graph, moves it into
the station frame, gives every node a robot-unique id, and merges nodes that several robots
saw: objects with the same label within 0.5 m, places within 1 m, rooms within 1 m (found
with Yggdrasil's spatial index; rules in `station::Config`). Matching is one-to-one: a kept
node takes at most one node of each later robot. Pairs are taken nearest first, so a node
whose nearest match was taken falls back to its next nearest. The merged graph is kept in
Yggdrasil and updated incrementally: a delta only touches the nodes it names, mergeable nodes
near them, and their edges. `Station::merged_graph` rebuilds everything from scratch and is
the reference the incremental result is checked against.

Frame alignment (`station::align`): when robots do not share a frame, each later robot's
frame is found from its objects. Pairs of same-label matches whose distances agree propose a
yaw and translation; the one that puts the most objects on a same-label object of the lower
robots wins and is refined by least squares. `replay --align` re-estimates every 50 deltas and
moves the robot (`Station::set_transform`, re-merging incrementally) when the fit is clearly
better.

Drift correction (`station::deform`): one rigid transform per robot cannot undo drift inside a
robot's map. Every merge is a loop closure between two robots; each node of a later robot gets
an offset, pulled towards its loop closures and kept smooth along the robot's own edges (a
deformation graph), robots solved in order. `replay --deform` / `station_server --deform` write
the corrected graph. Checked on held-out data: with a 2nd run bent by simulated drift, correcting
from object merges only brought the places' median distance to the other robot's places from
0.41 m to 0.20 m (within 0.3 m: 29% to 68%).

```sh
cargo run --release -p station --bin multi_check -- path/to/dsg.json 20 [--objects-only]
```

On the office graph with a second simulated robot (rotated, shifted, 3 cm noise, 2% of nodes
dropping in and out each step): incremental station equal to the full rebuild at every step,
11 ms per delta at 9,000-10,000 nodes (the full rebuild took about 1 s); objects 113 of 113
merged onto the right partner, places 1,187 of 1,188 (1 onto a neighbouring place), rooms
8 of 8; deltas 88% smaller than full graphs.

### Recording robots and replaying them through the station

Run Hydra with `HYDRA_YGG_RECORD=<file>` (plus the usual `LD_PRELOAD` and `HYDRA_GRAPH_BACKEND`)
and every backend update appends its delta to that file: one JSON line with the update time,
the nodes and edges that changed (id, layer, position, label and spark_dsg's binary record)
and the ones removed. A path ending in `.gz` is gzip-compressed (about 6x smaller; a run
stopped with Ctrl-C still replays up to its last finished update). Then replay one file per robot through the station, in time order:

```sh
cargo run --release -p station --bin replay -- robot0.jsonl robot1.jsonl robot2.jsonl \
    [--transform 1,yaw,x,y,z] [--objects-only] [--out station.json]
```

Robots without `--transform` stay in their own frame (identity), which is right when all runs
share one world frame.

Live: `station_server --robots N [--align] [--deform] [--out station.json] [--dsg-out dsg.json]`
listens on port 7447 + robot; each Hydra runs with `HYDRA_YGG_RECORD=tcp://<host>:<port>`.

After correcting, the station matches again on the corrected positions
(`Station::correct_drift`; `replay --deform` runs three rounds, `station_server --deform` one per
report), so nodes drift had kept apart merge too. On the bent run: objects 28 to 53 merged, places
1,044 to 1,097; the incremental result still equals the full rebuild.

Frame alignment also uses places: a place with objects attached is described by the sorted
labels of those objects and matched like an object. On the office runs started from a wrong
frame this fits 116 points instead of 14 objects and recovers the frame exactly.

Rooms of different robots are joined when at least half of the smaller room's places are shared,
in replay and in the live station's output.

Loop closures are robust: drift correction weighs every merge by graduated non-convexity with
a truncated least squares cost (as Hydra-Multi's robust optimisation does), so a wrong merge ends
up with weight 0 instead of bending the map. The robots' trajectories (agent layer) are corrected
with the rest of the graph. On the bent run this raised the re-merged objects from 53 to 63 and
places from 1,097 to 1,147.

Rooms are detected again on the merged places with Hydra's own `RoomFinder` (uHumans2 settings),
as Hydra-Multi does on its merged graph: `station_to_dsg station.json out.json --rooms`, or
`station_server --dsg-out out.json --rooms`.
