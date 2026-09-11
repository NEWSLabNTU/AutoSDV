#!/usr/bin/env python3
"""Measure how well the live scan actually sits on the map.

    python3 scripts/testing/localization/ndt_alignment_report.py --seconds 60

Takes each incoming scan, transforms it into the map frame using the pose the
stack is currently publishing, and measures the distance from every scan point to
the nearest map point. That residual is the thing "is it aligned" actually means,
and it is independent of NDT's own opinion of itself.

**Why not just read NVTL.** Nearest-voxel transformation likelihood is computed
by the same estimator whose pose is in question, against the same voxel grid, and
it rises with `ndt.resolution` regardless of accuracy. It cannot distinguish "the
scan is on the map" from "the scan is confidently on the wrong part of the map".
A nearest-neighbour residual can: a converged pose puts most scan points within a
map voxel of a real surface, and a diverged one does not, whatever NVTL says.

Reported separately for stationary and moving frames, because they answer
different questions -- the stationary ones say whether initial convergence
succeeded, the moving ones say whether tracking is holding.
"""

from __future__ import annotations

import argparse
import math
import os
import re
import sys
import time

# Re-exec without the user site-packages, BEFORE numpy is imported.
#
# ~/.local has numpy 2.2.6, which shadows the apt numpy 1.21.5 that the apt
# scipy 1.8.0 was built against. Importing scipy.spatial under the newer numpy
# fails with "numpy.dtype size changed, may indicate binary incompatibility",
# which reads like a scipy bug rather than a shadowing problem. The ROS stack is
# apt-installed too, so the apt pair is the consistent one here.
if os.environ.get("PYTHONNOUSERSITE") != "1":
    os.environ["PYTHONNOUSERSITE"] = "1"
    os.execv(sys.executable, [sys.executable] + sys.argv)

import numpy as np


# scripts/testing/localization/<this file> -> repo root is four levels up.
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
DEFAULT_MAP = os.path.join(REPO_ROOT, "data", "COSS-map-planning",
                           "pointcloud_map.pcd")


def load_pcd(path: str) -> np.ndarray:
    with open(path, "rb") as fh:
        header = b""
        while not header.strip().endswith(b"binary"):
            line = fh.readline()
            if not line:
                raise ValueError(f"{path}: no binary DATA line")
            header += line
        count = int(re.search(rb"POINTS (\d+)", header).group(1))
        raw = np.frombuffer(fh.read(count * 16), dtype=np.float32, count=count * 4)
    return raw.reshape(count, 4)[:, :3].astype(np.float64)


def quat_to_matrix(x, y, z, w) -> np.ndarray:
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def pct(v, q):
    return float(np.percentile(v, q)) if len(v) else float("nan")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--seconds", type=float, default=60.0)
    parser.add_argument("--map", default=DEFAULT_MAP,
                        help="pointcloud map PCD (default: the COSS map)")
    parser.add_argument("--topic", default="/localization/util/downsample/pointcloud")
    parser.add_argument("--label", default="")
    parser.add_argument("--max-radius", type=float, default=3.0,
                        help="residuals are capped here; beyond it the point is counted as an outlier")
    args = parser.parse_args()

    from scipy.spatial import cKDTree

    print(f"  loading map {args.map}", file=sys.stderr)
    map_pts = load_pcd(args.map)
    tree = cKDTree(map_pts)
    print(f"  {len(map_pts):,} map points indexed", file=sys.stderr)

    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    import sensor_msgs_py.point_cloud2 as pc2
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import Odometry
    import tf2_ros
    from rclpy.duration import Duration
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

    frames = []      # (moving: bool, residuals: np.ndarray, n_points, outlier_frac)

    class Report(Node):
        def __init__(self):
            super().__init__(
                "ndt_alignment_report",
                parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
            )
            self.buf = tf2_ros.Buffer()
            self.listener = tf2_ros.TransformListener(self.buf, self)
            self.create_subscription(Odometry, "/localization/kinematic_state", self.on_odom, 20)
            # Sensor clouds are published BEST_EFFORT. A default (RELIABLE)
            # subscriber never connects, and rclpy reports it only as a one-line
            # "incompatible QoS" warning while the callback simply never fires.
            sensor_qos = QoSProfile(depth=5, history=QoSHistoryPolicy.KEEP_LAST,
                                    reliability=QoSReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(PointCloud2, args.topic, self.on_scan, sensor_qos)
            self.speed = 0.0

        def on_odom(self, msg):
            t = msg.twist.twist.linear
            self.speed = math.sqrt(t.x * t.x + t.y * t.y + t.z * t.z)

        def on_scan(self, msg):
            # Use the pose the stack publishes for THIS scan's stamp; asking for
            # the latest transform instead would credit the estimator with motion
            # it had not yet observed.
            try:
                tf = self.buf.lookup_transform(
                    "map", msg.header.frame_id, msg.header.stamp,
                    timeout=Duration(seconds=0.2))
            except Exception:
                return
            pts = np.array(
                [[p[0], p[1], p[2]] for p in
                 pc2.read_points(msg, field_names=["x", "y", "z"], skip_nans=True)],
                dtype=np.float64)
            if len(pts) < 100:
                return
            q = tf.transform.rotation
            t = tf.transform.translation
            world = pts @ quat_to_matrix(q.x, q.y, q.z, q.w).T + np.array([t.x, t.y, t.z])
            d, _ = tree.query(world, k=1, distance_upper_bound=args.max_radius)
            outliers = np.isinf(d)
            frames.append((self.speed > 0.5, d[~outliers], len(world), float(outliers.mean())))

    rclpy.init()
    node = Report()
    deadline = time.time() + args.seconds
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

    if not frames:
        print("no scans with a usable map transform.", file=sys.stderr)
        print("  Is the replay running and NDT activated?", file=sys.stderr)
        print("  Check: python3 scripts/testing/localization/check_ndt_activated.py", file=sys.stderr)
        return 1

    label = f" [{args.label}]" if args.label else ""
    print(f"\nScan-to-map alignment{label} — {len(frames)} scans")
    for moving, name in ((False, "stationary  (initial convergence)"),
                         (True, "moving      (tracking)")):
        sel = [f for f in frames if f[0] == moving]
        if not sel:
            print(f"  {name}: no frames")
            continue
        res = np.concatenate([f[1] for f in sel])
        out = float(np.mean([f[3] for f in sel]))
        print(f"  {name}: {len(sel)} scans")
        print(f"      residual   p50 {pct(res, 50):.3f}   p95 {pct(res, 95):.3f}   "
              f"mean {res.mean():.3f} m")
        print(f"      beyond {args.max_radius:.0f} m: {out * 100:.1f}% of points")
    print("\n  A converged pose keeps p50 within roughly a map voxel (ndt.resolution)."
          "\n  A p50 that grows while moving is tracking drift, not a tuning constant.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
