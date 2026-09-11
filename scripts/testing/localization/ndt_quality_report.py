#!/usr/bin/env python3
"""Score an NDT replay on pose quality, deliberately not on NVTL.

    python3 scripts/testing/localization/ndt_quality_report.py --seconds 120

Run it while a replay is going. It records for a while and prints metrics that
NDT does not gate on.

**Why not NVTL.** The repo's own tuning study
(docs/research/localization/ndt_parameter_tuning_coss_map.md) was re-measured and
corrected: nearest-voxel transformation likelihood is a mean per-point fit that
rises with `ndt.resolution` and rises again when imperfect far returns are
cropped away. Maximising it therefore selects for coarse voxels and narrow crop
boxes whether or not the pose gets better — and when the re-measurement used
pose-quality metrics instead, both of that study's headline conclusions
reversed. Resolution 2.0 beat 4.0 on accuracy while scoring *lower* NVTL.

So this reports:

  scatter        per-frame deviation from a locally smoothed path. A pose that
                 jitters against a static map is wrong even when it scores well.
  yaw step       frame-to-frame heading change. Large steps are the flip and
                 slip failures a mean score averages away.
  init->result   how far NDT moves the prior each frame. Small and steady means
                 the prior is good; large or growing means the prior is stale,
                 which is a fusion problem and not a parameter to tune.
  exe time       so a configuration that is accurate but too slow for the Orin
                 is visible as such.

`initial_to_result_distance` and `exe_time_ms` come from NDT's own diagnostics.
"""

from __future__ import annotations

import argparse
import math
import statistics
import sys
import time


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def percentile(values, fraction):
    if not values:
        return float("nan")
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(len(ordered) * fraction))]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--seconds", type=float, default=120.0)
    parser.add_argument("--label", default="", help="printed with the results")
    args = parser.parse_args()

    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from diagnostic_msgs.msg import DiagnosticArray

    poses = []          # (t, x, y, yaw)
    init_to_result = []
    exe_ms = []
    nvtl = []

    class Listen(Node):
        def __init__(self):
            super().__init__("ndt_quality_report")
            self.create_subscription(Odometry, "/localization/kinematic_state",
                                     self.on_pose, 50)
            self.create_subscription(DiagnosticArray, "/diagnostics", self.on_diag, 20)

        def on_pose(self, msg):
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            p = msg.pose.pose
            poses.append((t, p.position.x, p.position.y, yaw_of(p.orientation)))

        def on_diag(self, msg):
            for status in msg.status:
                if "ndt_scan_matcher" not in status.name:
                    continue
                for kv in status.values:
                    try:
                        value = float(kv.value)
                    except (TypeError, ValueError):
                        continue
                    if kv.key == "initial_to_result_distance":
                        init_to_result.append(value)
                    elif kv.key == "exe_time_ms":
                        exe_ms.append(value)
                    elif "nearest_voxel_transformation_likelihood" in kv.key:
                        nvtl.append(value)

    rclpy.init()
    node = Listen()
    deadline = time.time() + args.seconds
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

    if len(poses) < 20:
        print(f"only {len(poses)} poses on /localization/kinematic_state.", file=sys.stderr)
        print("  NDT is not localizing. Set an initial pose first:", file=sys.stderr)
        print("  python3 scripts/testing/localization/set_initial_pose.py <SET>", file=sys.stderr)
        return 1

    # Scatter against a 5-frame moving average: a real trajectory is smooth at
    # this timescale, so what the smoother rejects is estimator noise rather
    # than motion.
    scatter = []
    yaw_steps = []
    window = 5
    for i in range(window, len(poses) - window):
        xs = [p[1] for p in poses[i - window:i + window + 1]]
        ys = [p[2] for p in poses[i - window:i + window + 1]]
        scatter.append(math.hypot(poses[i][1] - statistics.fmean(xs),
                                  poses[i][2] - statistics.fmean(ys)))
    for a, b in zip(poses, poses[1:]):
        d = b[3] - a[3]
        yaw_steps.append(abs(math.degrees(math.atan2(math.sin(d), math.cos(d)))))

    path_length = sum(math.dist(a[1:3], b[1:3]) for a, b in zip(poses, poses[1:]))

    label = f" [{args.label}]" if args.label else ""
    print(f"\nNDT pose quality{label} — {len(poses)} poses, {path_length:.1f} m of path")
    print(f"  scatter vs smoothed path   p50 {percentile(scatter, .5):.3f}  "
          f"p95 {percentile(scatter, .95):.3f} m")
    print(f"  frame-to-frame yaw step    p50 {percentile(yaw_steps, .5):.3f}  "
          f"p95 {percentile(yaw_steps, .95):.3f} deg")
    if init_to_result:
        print(f"  initial_to_result_distance p50 {percentile(init_to_result, .5):.3f}  "
              f"p95 {percentile(init_to_result, .95):.3f} m")
    if exe_ms:
        print(f"  exe_time_ms                p50 {percentile(exe_ms, .5):.1f}  "
              f"p95 {percentile(exe_ms, .95):.1f} ms")
    if nvtl:
        print(f"  NVTL (reported, NOT ranked on) p50 {percentile(nvtl, .5):.2f}")
    print("\n  Rank configurations on scatter and yaw step. NVTL rises with coarser"
          "\n  voxels and tighter crops regardless of accuracy, which is how the"
          "\n  earlier study reached two conclusions that later reversed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
