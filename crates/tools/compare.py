"""Diff two Spark-DSG JSON files node by node and edge by edge.

usage: python3 compare.py original.json roundtrip.json [--tol 1e-5]

Edges are compared as undirected pairs (Spark-DSG graphs are undirected). Floats are compared
with a relative tolerance; the largest position error is reported separately because Yggdrasil
stores coordinates as f32 while Spark-DSG exports f64.
"""

import argparse
import json
import math
from collections import Counter


def close(a, b, tol, path, diffs):
    if isinstance(a, float) or isinstance(b, float):
        if isinstance(a, (int, float)) and isinstance(b, (int, float)):
            if not math.isclose(a, b, rel_tol=tol, abs_tol=tol):
                diffs.append((path, a, b))
            return
    if type(a) is not type(b):
        diffs.append((path, a, b))
    elif isinstance(a, dict):
        for k in a.keys() | b.keys():
            if k not in a or k not in b:
                diffs.append((f"{path}.{k}", a.get(k, "<missing>"), b.get(k, "<missing>")))
            else:
                close(a[k], b[k], tol, f"{path}.{k}", diffs)
    elif isinstance(a, list):
        if len(a) != len(b):
            diffs.append((f"{path}[len]", len(a), len(b)))
        else:
            for i, (x, y) in enumerate(zip(a, b)):
                close(x, y, tol, f"{path}[{i}]", diffs)
    elif a != b:
        diffs.append((path, a, b))


def edge_key(e):
    return (min(e["source"], e["target"]), max(e["source"], e["target"]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("original")
    ap.add_argument("roundtrip")
    ap.add_argument("--tol", type=float, default=1e-5)
    args = ap.parse_args()
    a = json.load(open(args.original))
    b = json.load(open(args.roundtrip))

    na = {n["id"]: n for n in a["nodes"]}
    nb = {n["id"]: n for n in b["nodes"]}
    print(f"nodes: {len(na)} vs {len(nb)}; missing {len(na.keys() - nb.keys())}, extra {len(nb.keys() - na.keys())}")

    diffs = []
    max_pos_err = 0.0
    for nid in na.keys() & nb.keys():
        x, y = na[nid], nb[nid]
        for k in ("layer", "partition"):
            if x.get(k, 0) != y.get(k, 0):
                diffs.append((f"{nid}.{k}", x.get(k), y.get(k)))
        px, py = x["attributes"].get("position"), y["attributes"].get("position")
        if px and py:
            max_pos_err = max(max_pos_err, max(abs(p - q) for p, q in zip(px, py)))
        close(x["attributes"], y["attributes"], args.tol, f"{nid}", diffs)
    print(f"node attribute diffs: {len(diffs)} (rel/abs tol {args.tol})")
    print(f"max position error: {max_pos_err:.3g} m")
    for d in diffs[:20]:
        print("  ", d)

    ea = Counter(edge_key(e) for e in a["edges"])
    eb = Counter(edge_key(e) for e in b["edges"])
    print(f"edges: {sum(ea.values())} vs {sum(eb.values())}; missing {sum((ea - eb).values())}, extra {sum((eb - ea).values())}")
    ia = {edge_key(e): e["info"] for e in a["edges"]}
    ib = {edge_key(e): e["info"] for e in b["edges"]}
    ediffs = []
    for k in ia.keys() & ib.keys():
        close(ia[k], ib[k], args.tol, f"edge{k}", ediffs)
    print(f"edge info diffs: {len(ediffs)}")
    for d in ediffs[:10]:
        print("  ", d)

    ok = not diffs and not ediffs and ea == eb and na.keys() == nb.keys()
    print("ROUND TRIP OK" if ok else "ROUND TRIP DIFFERS")


if __name__ == "__main__":
    main()
