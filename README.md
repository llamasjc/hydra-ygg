# hydra-yggdrasil

[Hydra](https://github.com/MIT-SPARK/Hydra) builds a 3D scene graph of a building while a robot
moves through it. This repo does two things:

1. **Runs Hydra on [Yggdrasil](https://github.com/brdsdsu/Yggdrasil)**, a Rust scene graph
   with a spatial index, so Hydra's graph lookups and nearest-node searches are answered by
   Yggdrasil.
2. **Merges several robots' scene graphs into one** (multi-robot, like MIT's Hydra-Multi),
   live or from recorded runs.

Yggdrasil itself is not modified.

## Background in one minute

A **3D scene graph** describes a building in layers:

| layer | nodes | example |
|---|---|---|
| 2 objects | things Hydra recognised | a chair at (3.1, 0.4, 0.5) |
| 3 places | free-space points the robot could stand at, linked when you can walk between them | |
| 4 rooms | groups of places | "room 3" |
| 5 building | | |

Edges link an object to the place it is near, places to each other, and a place to its room.
Hydra builds this graph online and keeps correcting it. Its **backend** holds the graph and
publishes it.

## How it works

### Part 1: Hydra on Yggdrasil (one robot)

```
Hydra backend --(every update: only what changed)--> Yggdrasil copy of the graph
      ^                                                   |
      +------- lookups and nearest-node searches ---------+
```

- After each backend update, the nodes and edges that changed are written into a Yggdrasil
  copy of the graph. It is never rebuilt from scratch.
- Questions Hydra asks of the graph ("what are this node's neighbours?", "which places are
  within 1 m?") are answered by Yggdrasil, and the graph Hydra publishes is read from it.
- Hydra still keeps its own graph (spark_dsg) in memory, because its algorithms write to it.
  Yggdrasil is a live copy that answers the reads, not a replacement.
- Correctness check: by default, every answer from Yggdrasil is compared with spark_dsg's
  answer. Set `HYDRA_YGG_COMPARE_EVERY=0` to turn this off once you trust it.

This is done with small patches to Hydra and spark_dsg (`patches/`) and a library loaded into
Hydra with `LD_PRELOAD` (`ros/hydra_ygg_sink`).

### Part 2: merging robots (the station)

Each robot runs Part 1 and also sends its graph changes over TCP (or to a file) to one
**station**. For each change the station:

1. **Puts the robot's nodes in a shared frame.** Each robot has its own start point, so its
   coordinates are rotated and shifted into the station's frame. Node ids get the robot number
   added so they stay unique.
2. **Finds the frame if it is unknown** (`--align`). It looks for the same objects (same class,
   same layout) in both robots' maps and solves for the rotation and shift that lines them up
   (RANSAC). Places count too, described by the objects attached to them.
3. **Merges nodes seen by several robots.** A later robot's object merges into an earlier
   robot's object with the same class within 0.5 m. Places and rooms merge within 1 m. Each
   node merges at most once (one-to-one), and `--max-matching` pairs them so the most nodes
   merge. Candidates are found with a Yggdrasil radius search.
4. **Corrects drift** (`--deform`). Robots' maps bend slowly as odometry drifts. Every merge
   says "these two nodes are the same spot", so the station bends each later robot's map to
   agree, while keeping neighbouring nodes moving together. A robust weighting (GNC) ignores
   merges that disagree with the rest, so one wrong merge cannot wreck the map. It then
   merges again on the corrected positions.
5. **Re-detects rooms** (`--rooms`). It runs Hydra's own room finder on the merged places.
6. **Writes the merged graph** as Spark-DSG JSON that Hydra's visualiser opens in RViz.

Only the nodes a change touches are recomputed. A slow from-scratch version
(`Station::merged_graph`) is kept, and checks confirm the two always agree.

Compared with Hydra-Multi, ours follows the same steps. The differences are that Hydra-Multi
optimises full 6-DoF robot poses (ours moves node positions only) and merges meshes (ours
doesn't).

## Results (uHumans2 office, 3 runs used as 3 robots)

| | |
|---|---|
| Yggdrasil vs Hydra's own graph | 0 differences in 78,672 checked queries; 210 spark_dsg tests pass |
| objects merged | 224 of 225 |
| places merged | 1,993 (at most 2,066 possible) |
| rooms on the merged graph | 7 (Hydra on one robot alone: 7) |
| frame found from a wrong start | within 0.0005 rad and 6 cm |
| station time per update | 5-17 ms at ~12,000 nodes |
| one wrong merge 3 m off | map moves < 5 cm (2.7 m without GNC) |

## Setup

You need:

- ROS 2 Jazzy with a colcon workspace containing Hydra, hydra_ros and spark_dsg (Hydra's Docker
  image works)
- Rust: `curl https://sh.rustup.rs -sSf | sh`
- the uHumans2 bags, or your own Hydra setup

```bash
cd /root/hydra_ws/src
git clone https://github.com/llamasjc/hydra-yggdrasil
hydra-yggdrasil/scripts/apply_patches.sh /root/hydra_ws/src
cd /root/hydra_ws && colcon build --packages-above spark_dsg && source install/setup.bash
```

## Running

**One robot, Hydra on Yggdrasil:**

```bash
HYDRA_GRAPH_BACKEND=yggdrasil \
LD_PRELOAD=/root/hydra_ws/install/hydra_ygg_sink/lib/libhydra_ygg_sink.so \
  ros2 launch hydra_ros uhumans2.launch.yaml
```

**Three robots live:** `scripts/live3.sh` starts the station, three Hydras and three uHumans2
bags, each robot in its own ROS domain. To do it by hand:

```bash
# the station
cargo run --release --bin station_server -- --robots 3 --align --deform --max-matching \
    --rooms --out station.json --dsg-out station_dsg.json
# each robot N (0, 1, 2)
HYDRA_YGG_RECORD=tcp://<station host>:$((7447+N)) HYDRA_GRAPH_BACKEND=yggdrasil \
LD_PRELOAD=.../libhydra_ygg_sink.so ros2 launch hydra_ros uhumans2.launch.yaml
```

**Record first, merge later:** run each robot with `HYDRA_YGG_RECORD=robotN.jsonl.gz`, then

```bash
cargo run --release --bin replay -- robot0.jsonl.gz robot1.jsonl.gz robot2.jsonl.gz \
    --align --deform --max-matching --out station.json
ros2 run hydra_ygg_sink station_to_dsg station.json station_dsg.json --rooms
```

**View the result:**

```bash
ros2 launch hydra_visualizer static_visualizer.launch.yaml scene_graph:=$PWD/station_dsg.json
```

## Station options

| flag | what it does |
|---|---|
| `--align` | find each robot's frame from shared objects and places |
| `--deform` | correct drift from the merges (robust), then merge again |
| `--max-matching` | pair nodes so the most merge, instead of nearest first |
| `--rooms` | re-detect rooms on the merged graph with Hydra's room finder |
| `--many-to-one` | allow several nodes of one robot to merge into the same node |
| `--radius L,P=R` | merge radius for layer L, partition P (e.g. `3,0=0.6` for places) |
| `--transform R,yaw,x,y,z` | a known frame for robot R |
| `--every N` (station_server) | write the merged graph every N updates |

## Code map

| path | what |
|---|---|
| `crates/station` | the station. Start with `src/lib.rs`, which walks through one update step by step |
| `crates/station/src/{matching,align,deform,rooms}.rs` | merging, frame finding, drift correction, room joining |
| `crates/station/src/bin` | `station_server` (live), `replay` (recordings), `multi_check` (simulated test) |
| `crates/dsg-convert` | Spark-DSG JSON to and from Yggdrasil |
| `crates/ygg-bridge` | the C++ interface Hydra's side calls into |
| `ros/hydra_ygg_sink` | ROS package: the Yggdrasil store, graph and search backends, `station_to_dsg` |
| `patches/` | the Hydra and spark_dsg patches |
| `docs/details.md` | full design notes and measurements |

## Scripts

| script | use |
|---|---|
| `scripts/apply_patches.sh` | apply the patches to a colcon workspace (safe to rerun) |
| `scripts/live3.sh` | the live 3-robot demo |
| `scripts/set_merge_strategy.sh` | turn Hydra's object merging on, searched with Yggdrasil or Hydra's own search |
| `scripts/r3d_to_bag.py` | turn an iPhone Record3D `.r3d` recording into a bag Hydra can map |
| `scripts/record3d_wifi/` | stream Record3D live over Wi-Fi into ROS |
| `scripts/uh2_labels.py` | add the label image Hydra expects to the original uHumans2 bags |

## License

Apache-2.0 (see `LICENSE`). Hydra and spark_dsg are BSD-licensed by MIT; the patches in
`patches/` apply to them. `scripts/record3d_wifi/worker.js` is adapted from Record3D's
MIT-licensed WebRTC demo.
