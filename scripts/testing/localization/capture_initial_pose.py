#!/usr/bin/env python3
"""Capture a localization pose so a replay can be re-initialised without a human.

    python3 scripts/testing/localization/capture_initial_pose.py COSS

Place the pose once in RViz, wait for NDT to settle, run this, and every
subsequent replay of that set can start unattended.

The written file records WHICH of the two sources it came from, because they are
worth very different things and look identical once saved. Under the planning
simulator this topic is simple_planning_simulator reporting its own ground truth,
so the pose is the click unchanged; under a replay it is NDT agreeing with the
map. Both are usable as a seed, only the second is a measurement.

This exists because tuning needs repetition. Re-deriving NDT parameters means
running the same bag many times and comparing, and a hand-placed pose makes
every run start somewhere slightly different — which shows up in the results as
parameter differences that are really placement differences.

The pose is taken from `/localization/kinematic_state`, i.e. after NDT has
converged and the EKF has settled, not from the raw click. The click is only a
seed; what is worth saving is where the map says the vehicle actually was.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time


# scripts/testing/localization/<this file> -> repo root is four levels up.
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
DEFAULT_DIR = os.path.join(REPO_ROOT, "data", "initial_poses")


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("name", help="set name, e.g. COSS")
    parser.add_argument("--out-dir", default=DEFAULT_DIR)
    parser.add_argument("--settle", type=float, default=3.0,
                        help="seconds of poses to check for stability (default 3)")
    parser.add_argument("--max-drift", type=float, default=0.5,
                        help="metres the pose may move while settling (default 0.5)")
    args = parser.parse_args()

    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry

    samples = []

    class Capture(Node):
        def __init__(self):
            super().__init__("capture_initial_pose")
            self.create_subscription(
                Odometry, "/localization/kinematic_state", self.on_pose, 10)

        def on_pose(self, msg):
            p = msg.pose.pose
            samples.append((p.position.x, p.position.y, p.position.z,
                            p.orientation.x, p.orientation.y,
                            p.orientation.z, p.orientation.w))

    rclpy.init()
    node = Capture()
    deadline = time.time() + args.settle
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node_names = node.get_node_names_and_namespaces()
    node.destroy_node()
    rclpy.shutdown()

    if not samples:
        print("no pose on /localization/kinematic_state.", file=sys.stderr)
        print("  Is the stack up, is the bag playing, and has NDT converged?",
              file=sys.stderr)
        return 1

    # A pose that is still moving is not a starting pose. Saving one mid-slide
    # would seed later runs somewhere the vehicle never was, and the resulting
    # bad convergence would look like a parameter problem.
    xs = [s[0] for s in samples]
    ys = [s[1] for s in samples]
    drift = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
    if drift > args.max_drift:
        print(f"pose moved {drift:.2f} m while sampling, above the {args.max_drift} m "
              f"limit.", file=sys.stderr)
        print("  Either the vehicle is moving in the replay, or NDT has not settled. "
              "Pause playback at the start and try again.", file=sys.stderr)
        return 1

    x, y, z, qx, qy, qz, qw = samples[-1]

    # Where the pose came from decides what it is worth, and the two cases are
    # indistinguishable once written to a file. In the planning simulator
    # /localization/kinematic_state is simple_planning_simulator echoing its own
    # ground truth, so it returns the click unchanged and never touches the point
    # cloud map -- a seed. Only a replay run has NDT actually agreeing with the
    # map. Recording "converged" for both is how a click ends up quoted as a
    # measurement, so detect it and say which one this is.
    simulated = any("simple_planning_simulator" in n for n, _ in node_names)

    os.makedirs(args.out_dir, exist_ok=True)
    path = os.path.join(args.out_dir, f"{args.name}.yaml")
    with open(path, "w", encoding="utf-8") as f:
        f.write(f"# Initial pose for the {args.name} replay.\n")
        if simulated:
            f.write("#\n# SOURCE: planning simulator -- simple_planning_simulator was\n")
            f.write("# running, so this is the clicked pose echoed back as ground truth,\n")
            f.write("# not a pose NDT matched against the map. Treat it as a seed for\n")
            f.write("# method AUTO. A zero drift and a round yaw are expected here.\n#\n")
        else:
            f.write("#\n# SOURCE: replay -- captured from /localization/kinematic_state\n")
            f.write("# with no simulator running, i.e. after NDT converged against the\n")
            f.write("# point cloud map.\n#\n")
        f.write(f"# {len(samples)} samples, drift {drift:.3f} m while sampling.\n")
        f.write("#\n")
        f.write("# Frame: map (MGRS 51RUH, matching the merged NTU map).\n")
        f.write("initial_pose:\n")
        f.write(f"  x: {x:.4f}\n  y: {y:.4f}\n  z: {z:.4f}\n")
        f.write(f"  qx: {qx:.6f}\n  qy: {qy:.6f}\n  qz: {qz:.6f}\n  qw: {qw:.6f}\n")
        f.write(f"  yaw_deg: {math.degrees(yaw_of(type('Q', (), dict(x=qx, y=qy, z=qz, w=qw)))):.2f}\n")

    print(f"captured -> {path}")
    print(f"  x={x:.2f} y={y:.2f} z={z:.2f}  drift while sampling {drift:.3f} m")
    print(f"  replay unattended with: just ntu-test init {args.name}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
