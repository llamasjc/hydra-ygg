#!/usr/bin/env python3
"""Record3D Wi-Fi stream -> ROS 2 topics for Hydra and RViz.

  python3 bridge.py [--port 8765] [--label 0]

then open http://<this machine>:<port>/ in Chrome or Safari, enter the phone's address and
press Start. The page receives the phone's WebRTC stream and posts frames here; this node
publishes, per frame, under the topic and frame names of the uHumans2 bags, so Hydra runs on
the phone with its usual uhumans2 launch:
  /tesse/left_cam/rgb/image_raw        colour (rgb8)
  /tesse/depth_cam/mono/image_raw      depth in metres (32FC1, 0 = invalid; the Wi-Fi stream
                                       only carries 0-3 m, compressed)
  /tesse/seg_cam/converted/image_raw   one label everywhere (mono8, --label)
  /tesse/left_cam/camera_info
  /tf                                  world -> base_link_gt, the phone camera (z-up world,
                                       ROS optical frame); base_link_gt -> left_cam is identity

The first frame's metadata is printed so the pose format can be checked.
"""
import json
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

HERE = Path(__file__).resolve().parent
FRAME = "left_cam"
BODY = "base_link_gt"
WORLD = "world"
MAX_DEPTH = 3.0

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


def find(obj, test):
    """First value in nested JSON for which test(key, value) holds."""
    stack = [(None, obj)]
    while stack:
        key, v = stack.pop(0)
        if test(key, v):
            return v
        if isinstance(v, dict):
            stack.extend(v.items())
        elif isinstance(v, list) and v and not all(isinstance(x, (int, float)) for x in v):
            stack.extend((None, x) for x in v)
    return None


def numbers(v, n):
    return isinstance(v, list) and len(v) == n and all(isinstance(x, (int, float)) for x in v)


def parse_pose(meta):
    """ARKit camera pose (rotation, position) from per-frame metadata, in whichever of the
    usual layouts it comes: [qx,qy,qz,qw,tx,ty,tz], {qx..qw, tx..tz / px..pz}, or a 4x4 matrix."""
    v = find(meta, lambda k, v: k and "pose" in k.lower() and numbers(v, 7))
    if v is not None:
        return quat_to_matrix(*v[:4]), np.array(v[4:7])
    v = find(meta, lambda k, v: isinstance(v, dict) and {"qx", "qy", "qz", "qw"} <= v.keys())
    if v is not None:
        t = [v.get(a, v.get(b, 0.0)) for a, b in (("tx", "px"), ("ty", "py"), ("tz", "pz"))]
        return quat_to_matrix(v["qx"], v["qy"], v["qz"], v["qw"]), np.array(t, dtype=float)
    v = find(meta, lambda k, v: numbers(v, 16))
    if v is not None:
        m = np.array(v, dtype=float).reshape(4, 4)
        if abs(m[3, 3] - 1) > 1e-6 or np.abs(m[3, :3]).max() > 1e-6:
            m = m.T  # column-major
        return m[:3, :3], m[:3, 3]
    return None


def parse_k(meta):
    v = find(meta, lambda k, v: (k == "K" or str(k).startswith("intrinsic")) and numbers(v, 9))
    if v is None:
        return None
    k = np.array(v, dtype=float).reshape(3, 3)
    if k[2, 0] != 0 or k[2, 1] != 0:  # stored column-major
        k = k.T
    return k


def decode_jpeg(data):
    """(h, w, 3) RGB from JPEG bytes, with OpenCV (part of ROS) or Pillow."""
    try:
        import cv2
        bgr = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        return np.ascontiguousarray(bgr[..., ::-1])
    except ImportError:
        import io
        from PIL import Image as PILImage
        return np.asarray(PILImage.open(io.BytesIO(data)).convert("RGB"))


def hue(rgb):
    """HSV hue in [0, 1) of an (h, w, 3) uint8 image."""
    c = rgb.astype(np.float32) / 255.0
    r, g, b = c[..., 0], c[..., 1], c[..., 2]
    mx, mn = c.max(-1), c.min(-1)
    d = np.where(mx - mn > 1e-6, mx - mn, 1.0)
    h = np.where(mx == r, ((g - b) / d) % 6.0, np.where(mx == g, (b - r) / d + 2.0, (r - g) / d + 4.0))
    return np.where(mx - mn > 1e-6, h / 6.0, 0.0)


class Bridge(Node):
    def __init__(self, label):
        super().__init__("record3d_bridge")
        self.label = label
        self.rgb = self.create_publisher(Image, "/tesse/left_cam/rgb/image_raw", 10)
        self.depth = self.create_publisher(Image, "/tesse/depth_cam/mono/image_raw", 10)
        self.sem = self.create_publisher(Image, "/tesse/seg_cam/converted/image_raw", 10)
        self.info = self.create_publisher(CameraInfo, "/tesse/left_cam/camera_info", 10)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        body = TransformStamped()
        body.header.stamp = self.get_clock().now().to_msg()
        body.header.frame_id = BODY
        body.child_frame_id = FRAME
        body.transform.rotation.w = 1.0
        self.static_tf.sendTransform(body)
        self.stream_meta = {}
        self.count = 0
        self.lock = threading.Lock()
        # the web server only hands frames over; they are decoded and published on the ROS
        # thread, newest first, so a slow frame never holds up the browser
        self.pending = None
        self.received = 0
        self.create_timer(0.01, self.publish_pending)

    def offer(self, meta, width, height, pixels):
        with self.lock:
            self.pending = (meta, width, height, pixels)
            self.received += 1

    def publish_pending(self):
        with self.lock:
            item, self.pending = self.pending, None
        if item is None:
            return
        try:
            self.frame(*item)
        except Exception as e:  # keep going; report what broke
            print("frame error:", e, flush=True)

    def image(self, stamp, arr, encoding):
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = FRAME
        msg.height, msg.width = arr.shape[:2]
        msg.encoding = encoding
        msg.step = arr.strides[0]
        msg.data = arr.tobytes()
        return msg

    def frame(self, meta, width, height, pixels):
        if self.count == 0:
            print("first frame metadata:", json.dumps(meta)[:2000], flush=True)
        pose = parse_pose(meta)
        k = parse_k(meta)
        if k is None:
            k = parse_k(self.stream_meta)
        if pose is None or k is None:
            if self.count % 100 == 0:
                print(f"frame {self.count}: no {'pose' if pose is None else 'intrinsics'} in metadata, skipped", flush=True)
            self.count += 1
            return
        self.count += 1

        jpeg, depth_mm = pixels
        half = width
        rgb = decode_jpeg(jpeg)
        if rgb.shape[:2] != (height, width):
            raise ValueError(f"colour is {rgb.shape[1]}x{rgb.shape[0]}, expected {width}x{height}")
        depth = np.frombuffer(depth_mm, dtype="<u2").reshape(height, width).astype(np.float32) / 1000.0
        depth[(depth < 0.05) | (depth > MAX_DEPTH - 0.05)] = 0.0

        # intrinsics are for the phone's full colour image; its principal point is near the
        # centre, which gives the size they refer to
        sx, sy = half / (2.0 * k[0, 2]), height / (2.0 * k[1, 2])
        fx, fy, cx, cy = k[0, 0] * sx, k[1, 1] * sy, k[0, 2] * sx, k[1, 2] * sy

        R, p = pose
        R = WORLD_FIX @ R @ CAMERA_FIX
        p = WORLD_FIX @ p
        q = matrix_to_quat(R)

        stamp = self.get_clock().now().to_msg()
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = WORLD
        tf.child_frame_id = BODY
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, p)
        tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = map(float, q)

        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = FRAME
        info.height, info.width = height, half
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        if True:
            self.tf.sendTransform(tf)
            self.info.publish(info)
            self.rgb.publish(self.image(stamp, rgb, "rgb8"))
            self.depth.publish(self.image(stamp, depth, "32FC1"))
            self.sem.publish(self.image(stamp, np.full((height, half), self.label, np.uint8), "mono8"))
        if self.count % 20 == 0:
            print(f"{self.received} frames received, {self.count} published, camera at [{p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f}]", flush=True)


def serve(bridge, port):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def reply(self, code, body=b"", ctype="text/plain"):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            files = {"/": ("page.html", "text/html"), "/worker.js": ("worker.js", "text/javascript")}
            if self.path in files:
                name, ctype = files[self.path]
                self.reply(200, (HERE / name).read_bytes(), ctype)
            else:
                self.reply(404)

        def do_POST(self):
            n = int(self.headers.get("Content-Length", 0))
            body = b""
            while len(body) < n:
                chunk = self.rfile.read(n - len(body))
                if not chunk:
                    break
                body += chunk
            if self.path == "/stream_metadata":
                bridge.stream_meta = json.loads(body or b"{}")
                print("stream metadata:", json.dumps(bridge.stream_meta)[:2000], flush=True)
                self.reply(200)
            elif self.path == "/frame":
                w, h = (int(x) for x in self.headers["X-Size"].split("x"))
                nj = int(self.headers["X-Jpeg-Bytes"])
                if len(body) != nj + w * h * 2:
                    print(f"frame of {len(body)} bytes, expected {nj + w * h * 2}; skipped", flush=True)
                    self.reply(200)
                    return
                bridge.offer(json.loads(self.headers["X-Meta"]), w, h, (body[:nj], body[nj:]))
                self.reply(200)
            else:
                self.reply(404)

    server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
    print(f"open http://<this machine's address>:{port}/ in Chrome or Safari", flush=True)
    server.serve_forever()


def main():
    a = sys.argv[1:]
    port = int(a[a.index("--port") + 1]) if "--port" in a else 8765
    label = int(a[a.index("--label") + 1]) if "--label" in a else 0
    rclpy.init()
    bridge = Bridge(label)
    threading.Thread(target=serve, args=(bridge, port), daemon=True).start()
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
