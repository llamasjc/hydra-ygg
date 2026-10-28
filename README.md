# hydra-yggdrasil

Multi-robot 3D scene graphs: [Hydra](https://github.com/MIT-SPARK/Hydra) running on the
[Yggdrasil](https://github.com/brdsdsu/Yggdrasil) scene graph, plus a Hydra-Multi style station
that merges several robots' scene graphs into one, live or from recordings.

- **Hydra on Yggdrasil**: small patches let Hydra's backend keep its graph in Yggdrasil and run
  its nearest-node searches on Yggdrasil's spatial index. Yggdrasil is used unmodified.
- **Multi-robot station** (`crates/station`): robots stream graph deltas; the station aligns
  their frames, merges objects, places and rooms seen by several robots, corrects drift with
  robust (GNC) loop closures, and re-detects rooms on the merged graph with Hydra's room finder.
- **Phone input**: map with an iPhone/iPad running Record3D, from a recording or live over Wi-Fi.

## Results (uHumans2 office, 3 robots)

| | |
|---|---|
| objects merged | 224 of 225 later-robot objects |
| places merged | 1,993 (one-to-one, at most 2,066 possible) |
| rooms on the merged graph | 7 (Hydra on one robot alone: 7) |
| frame alignment from a wrong start | exact (116 matching objects and places), 0.15 s |
| station update | 5-20 ms per delta at ~12,000 nodes |
| wrong merge injected 3 m off | map bent < 5 cm (2.7 m without GNC) |

Every incremental station result is checked against a full rebuild.

## Requirements

- ROS 2 Jazzy with a colcon workspace containing Hydra, hydra_ros and spark_dsg
  (e.g. Hydra's Docker image)
- Rust (stable, edition 2024): `curl https://sh.rustup.rs -sSf | sh`
- uHumans2 bags to try it, or your own Hydra setup

## Quick start

```bash
# in the workspace's src folder
git clone https://github.com/llamasjc/hydra-yggdrasil
hydra-yggdrasil/scripts/apply_patches.sh /root/hydra_ws/src
cd /root/hydra_ws && colcon build --packages-above spark_dsg && source install/setup.bash
```

**One robot, Hydra on Yggdrasil:**

```bash
HYDRA_GRAPH_BACKEND=yggdrasil \
LD_PRELOAD=/root/hydra_ws/install/hydra_ygg_sink/lib/libhydra_ygg_sink.so \
  ros2 launch hydra_ros uhumans2.launch.yaml
```

**Several robots, recorded then merged:** run each robot's Hydra with
`HYDRA_YGG_RECORD=robotN.jsonl.gz` added, then

```bash
cd hydra-yggdrasil
cargo run --release --bin replay -- robot0.jsonl.gz robot1.jsonl.gz robot2.jsonl.gz \
    --align --deform --max-matching --out station.json
ros2 run hydra_ygg_sink station_to_dsg station.json station_dsg.json --rooms
ros2 launch hydra_visualizer static_visualizer.launch.yaml scene_graph:=$PWD/station_dsg.json
```

**Several robots, live:** `scripts/live3.sh` starts the station, three Hydras and three
uHumans2 bags (each robot in its own ROS domain). By hand:

```bash
cargo run --release --bin station_server -- --robots 3 --align --deform --max-matching \
    --out station.json --dsg-out station_dsg.json --rooms
# each robot N:
HYDRA_YGG_RECORD=tcp://<station host>:$((7447+N)) HYDRA_GRAPH_BACKEND=yggdrasil \
LD_PRELOAD=.../libhydra_ygg_sink.so ros2 launch hydra_ros uhumans2.launch.yaml
```

## Station options

| flag | what it does |
|---|---|
| `--align` | find each robot's frame from shared objects and places (no common frame needed) |
| `--deform` | robust drift correction from the merges (GNC), then merge again on corrected positions |
| `--max-matching` | pair places for the largest one-to-one matching instead of greedy nearest |
| `--many-to-one` | let several nodes of one robot merge into the same node |
| `--radius L,P=R` | merge radius for a layer (default: objects 0.5 m, places 1 m, rooms 1 m) |
| `--transform R,yaw,x,y,z` | a known starting frame for robot R |
| `--rooms` (station_server, station_to_dsg) | re-detect rooms on the merged places with Hydra's RoomFinder |

## Scripts

| script | use |
|---|---|
| `scripts/apply_patches.sh` | apply the Hydra and spark_dsg patches to a colcon workspace (idempotent) |
| `scripts/live3.sh` | live 3-robot demo on the uHumans2 office bags |
| `scripts/set_merge_strategy.sh` | turn Hydra's object merging on, searched with Yggdrasil or Hydra's own search |
| `scripts/r3d_to_bag.py` | Record3D `.r3d` recording to a ROS 2 bag Hydra can map |
| `scripts/record3d_wifi/` | live Record3D Wi-Fi stream to ROS topics (`python3 bridge.py`, open the page it prints) |
| `scripts/uh2_labels.py` | give the original uHumans2 bags the label image Hydra expects |

## How it compares to Hydra-Multi

Same pipeline: inter-robot loop closures from objects and places, robust (GNC) outlier rejection,
graph correction, node merging, and room detection rerun on the merged graph. Differences:
drift is corrected with a deformation graph over node positions (Hydra-Multi optimises 6-DoF
robot poses with GTSAM, so orientations here are not corrected), the mesh is not merged, and
merging is an exact one-to-one matching.

## Layout

| path | what |
|---|---|
| `crates/station` | the multi-robot station: merging, alignment, drift correction, `replay`, `station_server` |
| `crates/dsg-convert` | Spark-DSG JSON <-> Yggdrasil, lossless; typed records; incremental topology |
| `crates/ygg-bridge` | C++ API (cxx) that the ROS package calls; delta recording |
| `crates/tools` | `roundtrip` and `compare.py` |
| `ros/hydra_ygg_sink` | colcon package: Yggdrasil store, graph and nearest-node backends, `station_to_dsg` |
| `patches/` | Hydra and spark_dsg patches |
| `docs/details.md` | design notes, checks and measurements |

## License

Apache-2.0 (see `LICENSE`). Hydra and spark_dsg are BSD-licensed by MIT; the patches in
`patches/` apply to them. `scripts/record3d_wifi/worker.js` is adapted from Record3D's MIT-licensed
WebRTC demo.
