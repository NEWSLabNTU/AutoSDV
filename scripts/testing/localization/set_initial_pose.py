#!/usr/bin/env python3
"""Re-apply a captured initial pose, so a replay starts without a human.

    python3 scripts/testing/localization/set_initial_pose.py COSS

Calls `/localization/initialize`, the same service RViz's 2D Pose Estimate
reaches through the ADAPI adaptor. Publishing `/initialpose3d` directly does NOT
initialise Autoware — pose_initializer publishes that topic rather than
listening to it, and it is what triggers the EKF out of its dormant state.

Method is AUTO, not DIRECT: AUTO hands the pose to the configured estimator as a
starting guess, so NDT refines it against the map. The captured pose came from
one converged run and the vehicle will not be in exactly that spot on the next
one; letting NDT settle it is the difference between a seed and an assertion.
"""

from __future__ import annotations

import argparse
import os
import sys
import time

# scripts/testing/localization/<this file> -> repo root is four levels up.
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
DEFAULT_DIR = os.path.join(REPO_ROOT, "data", "initial_poses")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("name", help="set name, e.g. COSS")
    parser.add_argument("--dir", default=DEFAULT_DIR)
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args()

    path = os.path.join(args.dir, f"{args.name}.yaml")
    if not os.path.exists(path):
        print(f"no captured pose at {path}", file=sys.stderr)
        print("  Place one in RViz, then: "
              f"python3 scripts/testing/localization/capture_initial_pose.py {args.name}",
              file=sys.stderr)
        return 1

    import yaml
    pose = yaml.safe_load(open(path, encoding="utf-8"))["initial_pose"]

    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from autoware_localization_msgs.srv import InitializeLocalization

    rclpy.init()
    # use_sim_time is not optional here, and getting it wrong fails silently.
    #
    # A replay stack runs on bag time. These recordings are days older than now,
    # so a node stamping with the wall clock hands NDT an initial pose whose
    # timestamp is days away from every scan it holds. NDT validates the pose
    # against the sensor timestamp, rejects it, and then reports exactly what a
    # healthy-but-idle matcher reports: iteration_num 0, NVTL 0.0, sub-millisecond
    # exe_time, and no pose output at all -- while the EKF happily dead-reckons on
    # IMU and velocity, so the vehicle still moves and nothing looks broken.
    node = Node(
        "set_initial_pose",
        parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
    )

    # The clock has to actually arrive before the stamp is read; a fresh sim-time
    # node reports 0 until the first /clock message lands.
    deadline = time.time() + args.timeout
    while node.get_clock().now().nanoseconds == 0 and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.get_clock().now().nanoseconds == 0:
        print("no /clock after waiting — is the bag playing with --clock?", file=sys.stderr)
        print("  Start playback first: just bag play-ntu <SET>", file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    client = node.create_client(InitializeLocalization, "/localization/initialize")
    if not client.wait_for_service(timeout_sec=args.timeout):
        print("/localization/initialize never appeared — is the stack up?", file=sys.stderr)
        node.destroy_node(); rclpy.shutdown()
        return 1

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = "map"
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.pose.position.x = float(pose["x"])
    msg.pose.pose.position.y = float(pose["y"])
    msg.pose.pose.position.z = float(pose["z"])
    msg.pose.pose.orientation.x = float(pose["qx"])
    msg.pose.pose.orientation.y = float(pose["qy"])
    msg.pose.pose.orientation.z = float(pose["qz"])
    msg.pose.pose.orientation.w = float(pose["qw"])
    # Coarse on purpose: this is a seed for NDT, and a tight covariance would
    # claim a precision a re-used pose does not have.
    msg.pose.covariance[0] = 1.0
    msg.pose.covariance[7] = 1.0
    msg.pose.covariance[14] = 0.25
    msg.pose.covariance[21] = 0.05
    msg.pose.covariance[28] = 0.05
    msg.pose.covariance[35] = 0.2

    request = InitializeLocalization.Request()
    request.method = InitializeLocalization.Request.AUTO
    request.pose_with_covariance.append(msg)

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=args.timeout)
    status = future.result().status if future.result() else None
    node.destroy_node()
    rclpy.shutdown()

    if status is None:
        print("no response from /localization/initialize", file=sys.stderr)
        return 1
    if not status.success:
        print(f"initialization rejected: {status.message}", file=sys.stderr)
        return 1
    print(f"initialized from {path}: x={pose['x']:.2f} y={pose['y']:.2f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
