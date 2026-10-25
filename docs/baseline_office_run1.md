# Stock Hydra baseline: uH2 office, run 1 (2026-10-25)

Run: Hydra (public main) in Docker on desktop-arch, bag hydra_office_mcap at --rate 0.5, GT poses (base_link_gt), LCD off.
Saved to: desktop-arch:/mnt/storage/hydra_output/office_run1 (264 MB). Stopped with `pkill -INT -f hydra_ros_node` (not the launch) so the backend drained its queue and saved.

## Backend graph (backend/dsg.json)
| layer | this run | Yggdrasil paper's office graph (Spark-DSG) |
|---|---|---|
| buildings (5) | 1 | 1 |
| rooms (4) | 8 | 9 |
| places (3/0) | 1,188 | 1,130 |
| mesh places (3/1) | 3,250 | 1,472 |
| agent poses (2/'a') | 596 | 705 |
| objects (2/0) | 113 | 215 |
| total nodes / edges | 5,156 / 15,566 | 3,532 / 11,012 |

Objects by label: computer 36, chair 25, plant 15, trashcan 15, couch 12, painting 10. Layer-2 stats: 167 removed, 51 merged.

The two graphs come from different Hydra versions and label spaces (this run: 21-class uH2 office labels, 7 object classes; paper's graph: ADE20K), so the node counts are not expected to match.

## Not yet available
- No ground-truth TSDF saved, so evaluate_places / evaluate_rooms can't run yet.
- No room bbox file; no object-metric tool in public Hydra.
- Yggdrasil Table I evaluation harness is not in the repo.
