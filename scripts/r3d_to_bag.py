#!/usr/bin/env python3
"""Turn a Record3D recording (.r3d, iPhone/iPad with LiDAR) into a ROS 2 bag Hydra can map.

  r3d_to_bag.py <recording.r3d> <out_bag> [--label N]

Writes, per frame:
  /phone/rgb/image_raw        color, resized to the depth resolution (rgb8)
  /phone/depth/image_raw      LiDAR depth in metres (32FC1), invalid = 0
  /phone/camera_info          intrinsics scaled to the depth resolution
  /phone/semantic/image_raw   one label everywhere (8UC1, --label, default 0): the phone has no
                              semantics, so Hydra builds places and rooms but no objects
  /tf                         world -> phone_camera (ROS optical frame, z up world)
and /clock, as a sqlite3 rosbag2 (ros2 bag play reads it directly).

Record3D stores ARKit poses: world y up, camera looking down -z with y up. They are turned
into a z-up world and the ROS optical convention (x right, y down, z forward).

Needs: pip install rosbags numpy pillow lzfse
"""
import io
import json
import sys
import zipfile

import lzfse
import numpy as np
from PIL import Image
from rosbags.rosbag2 import StoragePlugin, Writer
from rosbags.typesys import Stores, get_typestore

TS = get_typestore(Stores.ROS2_JAZZY)
Img = TS.types["sensor_msgs/msg/Image"]
Info = TS.types["sensor_msgs/msg/CameraInfo"]
RoI = TS.types["sensor_msgs/msg/RegionOfInterest"]
Header = TS.types["std_msgs/msg/Header"]
Time = TS.types["builtin_interfaces/msg/Time"]
TFMsg = TS.types["tf2_msgs/msg/TFMessage"]
TFS = TS.types["geometry_msgs/msg/TransformStamped"]
Tf = TS.types["geometry_msgs/msg/Transform"]
Vec = TS.types["geometry_msgs/msg/Vector3"]
Quat = TS.types["geometry_msgs/msg/Quaternion"]
Clock = TS.types["rosgraph_msgs/msg/Clock"]

FRAME = "phone_camera"
WORLD = "world"

# ARKit world (y up) -> z-up world, and ARKit camera (y up, looking -z) -> ROS optical
WORLD_FIX = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=float)
CAMERA_FIX = np.diag([1.0, -1.0, -1.0])


def quat_to_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quat(m):
    t = np.trace(m)
    if t > 0:
        s = 0.5 / np.sqrt(t + 1.0)
        return ((m[2, 1] - m[1, 2]) * s, (m[0, 2] - m[2, 0]) * s, (m[1, 0] - m[0, 1]) * s, 0.25 / s)
    i = int(np.argmax(np.diag(m)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2.0 * np.sqrt(1.0 + m[i, i] - m[j, j] - m[k, k])
    q = [0.0, 0.0, 0.0, 0.0]
    q[i] = 0.25 * s
    q[j] = (m[j, i] + m[i, j]) / s
    q[k] = (m[k, i] + m[i, k]) / s
    q[3] = (m[k, j] - m[j, k]) / s
    return tuple(q)


def header(stamp_ns, frame):
    return Header(stamp=Time(sec=stamp_ns // 10**9, nanosec=stamp_ns % 10**9), frame_id=frame)


def image(stamp_ns, arr, encoding):
    arr = np.ascontiguousarray(arr)
    h, w = arr.shape[:2]
    return Img(header=header(stamp_ns, FRAME), height=h, width=w, encoding=encoding,
               is_bigendian=0, step=arr.strides[0], data=arr.view(np.uint8).ravel())


def frame_files(names, ext):
    """Frame index -> file name, for rgbd/<i>.<ext>."""
    out = {}
    for n in names:
        if n.startswith("rgbd/") and n.endswith("." + ext):
            stem = n[len("rgbd/"):-len(ext) - 1]
            if stem.isdigit():
                out[int(stem)] = n
    return out


def convert(src, out, label=0):
    z = zipfile.ZipFile(src)
    meta = json.loads(z.read("metadata"))
    w, h = meta["w"], meta["h"]
    dw, dh = meta.get("dw", w), meta.get("dh", h)
    fps = meta.get("fps", 30)
    K = np.array(meta["K"], dtype=float).reshape(3, 3).T  # stored column-major
    poses = meta["poses"]  # per frame: qx, qy, qz, qw, px, py, pz
    stamps = meta.get("frameTimestamps")
    sx, sy = dw / w, dh / h
    fx, fy, cx, cy = K[0, 0] * sx, K[1, 1] * sy, K[0, 2] * sx, K[1, 2] * sy

    names = z.namelist()
    rgb_files = frame_files(names, "jpg")
    depth_files = frame_files(names, "depth")
    frames = sorted(set(rgb_files) & set(depth_files))
    print(f"{len(frames)} frames, color {w}x{h}, depth {dw}x{dh}, {fps} fps")

    with Writer(out, version=9, storage_plugin=StoragePlugin.SQLITE3) as wr:
        conn = {
            "rgb": wr.add_connection("/phone/rgb/image_raw", Img.__msgtype__, typestore=TS),
            "depth": wr.add_connection("/phone/depth/image_raw", Img.__msgtype__, typestore=TS),
            "sem": wr.add_connection("/phone/semantic/image_raw", Img.__msgtype__, typestore=TS),
            "info": wr.add_connection("/phone/camera_info", Info.__msgtype__, typestore=TS),
            "tf": wr.add_connection("/tf", TFMsg.__msgtype__, typestore=TS),
            "clock": wr.add_connection("/clock", Clock.__msgtype__, typestore=TS),
        }

        def write(key, t, msg):
            wr.write(conn[key], t, TS.serialize_cdr(msg, msg.__msgtype__))

        t0 = 10**9  # start at 1 s; Hydra dislikes time 0
        for n, i in enumerate(frames):
            if i >= len(poses):
                break
            if stamps:
                t = t0 + int((stamps[i] - stamps[frames[0]]) * 1e9)
            else:
                t = t0 + int(n * 1e9 / fps)

            rgb = Image.open(io.BytesIO(z.read(rgb_files[i]))).convert("RGB").resize((dw, dh), Image.BILINEAR)
            depth = np.frombuffer(lzfse.decompress(z.read(depth_files[i])), dtype=np.float32)
            depth = depth.reshape(dh, dw).copy()
            depth[~np.isfinite(depth)] = 0.0

            qx, qy, qz, qw, px, py, pz = poses[i]
            R = WORLD_FIX @ quat_to_matrix(qx, qy, qz, qw) @ CAMERA_FIX
            p = WORLD_FIX @ np.array([px, py, pz])
            q = matrix_to_quat(R)

            write("clock", t, Clock(clock=Time(sec=t // 10**9, nanosec=t % 10**9)))
            tf = TFS(header=header(t, WORLD), child_frame_id=FRAME,
                     transform=Tf(translation=Vec(x=p[0], y=p[1], z=p[2]),
                                  rotation=Quat(x=q[0], y=q[1], z=q[2], w=q[3])))
            write("tf", t, TFMsg(transforms=[tf]))
            write("info", t, Info(
                header=header(t, FRAME), height=dh, width=dw, distortion_model="plumb_bob",
                d=np.zeros(5), k=np.array([fx, 0, cx, 0, fy, cy, 0, 0, 1], dtype=float),
                r=np.eye(3).ravel(), p=np.array([fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0], dtype=float),
                binning_x=0, binning_y=0,
                roi=RoI(x_offset=0, y_offset=0, height=0, width=0, do_rectify=False)))
            write("rgb", t, image(t, np.asarray(rgb, dtype=np.uint8), "rgb8"))
            write("depth", t, image(t, depth, "32FC1"))
            write("sem", t, image(t, np.full((dh, dw), label, dtype=np.uint8), "mono8"))
            if (n + 1) % 200 == 0:
                print(f"{n + 1} frames")
    print(f"done: {out}")


if __name__ == "__main__":
    a = sys.argv[1:]
    if len(a) < 2:
        sys.exit(__doc__)
    label = int(a[a.index("--label") + 1]) if "--label" in a else 0
    convert(a[0], a[1], label)
