#!/usr/bin/env bash
# Choose how Hydra's backend finds object merge candidates, by editing the installed
# uhumans2 config (the original is kept as uhumans2.yaml.orig).
#   yggdrasil  Yggdrasil spatial index, with Hydra's own search run alongside for comparison
#   hydra      Hydra's own SemanticNearestNode
#   off        no strategy (stock uhumans2: object merging is off)
# Usage: set_merge_strategy.sh yggdrasil|hydra|off [hydra install share dir]
set -euo pipefail
mode="${1:?usage: $0 yggdrasil|hydra|off [share dir]}"
cfg="${2:-/root/hydra_ws/install/hydra/share/hydra}/config/datasets/uhumans2.yaml"
[ -f "$cfg.orig" ] || cp -L "$cfg" "$cfg.orig"
rm -f "$cfg"  # may be a symlink into the source tree; never edit the source
python3 - "$cfg.orig" "$cfg" "$mode" <<'PY'
import sys, yaml
src, dst, mode = sys.argv[1:]
cfg = yaml.safe_load(open(src))
objects = cfg["backend"]["update_functors"]["objects"]
strategy = {
    "yggdrasil": {"type": "YggdrasilSemanticNearestNode", "num_merges_to_consider": 1, "compare": True},
    "hydra": {"type": "SemanticNearestNode", "num_merges_to_consider": 1},
}.get(mode)
if mode not in ("yggdrasil", "hydra", "off"):
    sys.exit(f"unknown mode {mode}")
if strategy:
    objects["merge_proposer"] = {"strategy": strategy}
yaml.safe_dump(cfg, open(dst, "w"), sort_keys=False)
print(f"object merge strategy: {mode} ({dst})")
PY
