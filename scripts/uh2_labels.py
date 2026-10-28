#!/usr/bin/env python3
"""Give original uHumans2 bags the label image Hydra reads.

The original uHumans2 release has a color segmentation image (/tesse/seg_cam/rgb/image_raw).
Hydra's office bag has /tesse/seg_cam/converted/image_raw instead. Both exist for the office
s1_00h run, so the color -> label table is learned from that pair and applied to the others.

  learn  <original_mcap> <hydra_mcap> <table.json> [--every N]
         compares frames with the same timestamp and writes the color -> label pixel table
  apply  <table.json> <original_mcap> <out_mcap> [--map R,G,B=LABEL ...]
         copies every topic, replacing the color segmentation with the converted image
         (and dropping /tesse/seg_cam/camera_info, as Hydra's bag does); colors not in the
         table are labelled 0 and listed at the end, --map gives them a label instead
         The output is a sqlite3 rosbag2 (ros2 bag play reads it directly; rosbags' mcap
         output has no message index, which ros2 bag play cannot read)

Convert a ROS 1 .bag to mcap first, the same way office_mcap was made (rosbags-convert).
Needs: pip install rosbags numpy
"""
import json
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
from rosbags.rosbag2 import Reader, StoragePlugin, Writer
from rosbags.typesys import Stores, get_typestore

SEG = "/tesse/seg_cam/rgb/image_raw"
SEG_INFO = "/tesse/seg_cam/camera_info"
CONVERTED = "/tesse/seg_cam/converted/image_raw"

TS = get_typestore(Stores.ROS2_JAZZY)


def pixels(msg):
    """Image as (h, w, bytes per pixel) uint8."""
    data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    bpp = msg.step // msg.width
    return data.reshape(msg.height, msg.step)[:, : msg.width * bpp].reshape(
        msg.height, msg.width, bpp
    )


def keys(px):
    """One integer per pixel from its bytes."""
    k = np.zeros(px.shape[:2], dtype=np.int64)
    for c in range(px.shape[2]):
        k = (k << 8) | px[:, :, c]
    return k


def stamp(msg):
    return (msg.header.stamp.sec, msg.header.stamp.nanosec)


def frames(path, topic, every):
    with Reader(path) as r:
        conns = [c for c in r.connections if c.topic == topic]
        if not conns:
            sys.exit(f"{path}: no topic {topic}")
        for i, (conn, _, raw) in enumerate(r.messages(connections=conns)):
            if i % every == 0:
                yield TS.deserialize_cdr(raw, conn.msgtype)


def learn(original, hydra, table_path, every=20):
    wanted = {stamp(m): m for m in frames(hydra, CONVERTED, every)}
    print(f"{len(wanted)} converted frames sampled")
    counts = {}
    out_format = None
    matched = 0
    for m in frames(original, SEG, 1):
        conv = wanted.get(stamp(m))
        if conv is None:
            continue
        matched += 1
        if out_format is None:
            out_format = {"encoding": conv.encoding, "bpp": conv.step // conv.width,
                          "is_bigendian": int(conv.is_bigendian)}
            print(f"input {m.encoding}, output {conv.encoding}")
        pairs = np.stack([keys(pixels(m)).ravel(), keys(pixels(conv)).ravel()], axis=1)
        uniq, n = np.unique(pairs, axis=0, return_counts=True)
        for (a, b), c in zip(uniq.tolist(), n.tolist()):
            counts.setdefault(a, {}).setdefault(b, 0)
            counts[a][b] += c
    if not matched:
        sys.exit("no frames with matching timestamps")
    table, total, agree = {}, 0, 0
    for color, outs in counts.items():
        best = max(outs, key=outs.get)
        table[str(color)] = best
        total += sum(outs.values())
        agree += outs[best]
    Path(table_path).write_text(json.dumps({"format": out_format, "table": table}))
    print(f"{matched} frames matched, {len(table)} colors, "
          f"{100.0 * agree / total:.3f}% of pixels agree with the table")
    unknown = sorted(table.items(), key=lambda kv: -sum(counts[int(kv[0])].values()))[:15]
    for color, out in unknown:
        c = int(color)
        print(f"  color {c >> 16 & 255},{c >> 8 & 255},{c & 255} -> {out} "
              f"({sum(counts[c].values())} px)")


def apply(table_path, original, out, extra=()):
    spec = json.loads(Path(table_path).read_text())
    fmt = spec["format"]
    bpp = fmt["bpp"]
    lut = {int(k): v for k, v in spec["table"].items()}
    for m in extra:
        rgb, label = m.split("=")
        r, g, b = (int(x) for x in rgb.split(","))
        lut[(r << 16) | (g << 8) | b] = int(label)
    missing = {}
    with Reader(original) as r, Writer(out, version=9, storage_plugin=StoragePlugin.SQLITE3) as w:
        out_conn = {}
        for c in r.connections:
            if c.topic == SEG_INFO:
                continue
            topic = CONVERTED if c.topic == SEG else c.topic
            out_conn[c.id] = w.add_connection(
                topic, c.msgtype, typestore=TS,
                offered_qos_profiles=c.ext.offered_qos_profiles)
        n = 0
        for conn, t, raw in r.messages():
            if conn.id not in out_conn:
                continue
            if conn.topic == SEG:
                msg = TS.deserialize_cdr(raw, conn.msgtype)
                k = keys(pixels(msg))
                uniq, inv = np.unique(k, return_inverse=True)
                vals = np.array([lut.get(u, 0) for u in uniq.tolist()], dtype=np.int64)
                counts = np.bincount(inv.ravel(), minlength=len(uniq))
                for u, c in zip(uniq.tolist(), counts.tolist()):
                    if u not in lut:
                        missing[u] = missing.get(u, 0) + c
                v = vals[inv].reshape(k.shape)
                px = np.stack([(v >> (8 * (bpp - 1 - i))) & 255 for i in range(bpp)], axis=-1)
                msg = replace(msg, encoding=fmt["encoding"], is_bigendian=fmt["is_bigendian"],
                              step=msg.width * bpp,
                              data=px.astype(np.uint8).ravel())
                raw = TS.serialize_cdr(msg, conn.msgtype)
                n += 1
                if n % 500 == 0:
                    print(f"{n} label images")
            w.write(out_conn[conn.id], t, raw)
    print(f"done: {n} label images written to {out}")
    if missing:
        print(f"warning: {len(missing)} colors not in the table were labelled 0:")
        for c, px in sorted(missing.items(), key=lambda kv: -kv[1]):
            print(f"  color {c >> 16 & 255},{c >> 8 & 255},{c & 255} ({px} px); "
                  "give it a label with --map R,G,B=LABEL")


if __name__ == "__main__":
    a = sys.argv[1:]
    if len(a) >= 4 and a[0] == "learn":
        every = int(a[a.index("--every") + 1]) if "--every" in a else 20
        learn(a[1], a[2], a[3], every)
    elif len(a) >= 4 and a[0] == "apply":
        maps = [a[i + 1] for i, x in enumerate(a) if x == "--map"]
        apply(a[1], a[2], a[3], maps)
    else:
        sys.exit(__doc__)
