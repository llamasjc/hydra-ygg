#!/usr/bin/env bash
# Apply the hydra-ygg patches to Hydra and spark_dsg in a colcon workspace; patches already
# applied are skipped. Usage: apply_patches.sh <workspace src dir, e.g. /root/hydra_ws/src>
set -euo pipefail
here="$(cd "$(dirname "$0")/.." && pwd)"
src="${1:?usage: $0 <workspace src dir>}"

find_repo() {  # $1: a file that only that repo has
  local f
  f="$(find "$src" -path "*/$1" -not -path "*/hydra-ygg/*" | head -1)"
  [ -n "$f" ] || { echo "cannot find $1 under $src" >&2; exit 1; }
  git -C "$(dirname "$f")" rev-parse --show-toplevel
}

apply() {  # $1: repo dir, $2: patch dir. Applied patches are listed in .git/hydra-ygg-patches.
  local stamp
  stamp="$(git -C "$1" rev-parse --absolute-git-dir)/hydra-ygg-patches"
  touch "$stamp"
  for p in "$2"/*.patch; do
    local name
    name="$(basename "$p")"
    if grep -qxF "$name" "$stamp"; then
      echo "already applied: $name"
      continue
    fi
    if git -C "$1" apply --reverse --check "$p" 2>/dev/null; then
      echo "already applied: $name"
    else
      git -C "$1" apply "$p"
      echo "applied: $name -> $1"
    fi
    echo "$name" >> "$stamp"
  done
}

apply "$(find_repo src/backend/backend_module.cpp)" "$here/patches/hydra"
apply "$(find_repo src/edge_container.cpp)" "$here/patches/spark_dsg"
