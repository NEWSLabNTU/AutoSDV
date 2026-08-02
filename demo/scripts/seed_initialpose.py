#!/usr/bin/env python3
"""Publish an initial pose on /initialpose, the way RViz's "2D Pose Estimate" does.

The COSS bag's GNSS is single-point with about 20 m of scatter and disagrees
with the direction of travel, so letting it seed localization puts the vehicle
somewhere different on every run. This publishes a fixed pose instead, so a
replay is reproducible and needs no human at the RViz window.

The default is the pose the operator set by hand on 2026-07-28 and confirmed
against the map; NDT's align refined it to (0.117, -8.350, 9.312).

The bag must already be playing: the pose initializer runs an NDT align, which
needs a live scan and a running /clock.

    python3 seed_initialpose.py                       # the recorded COSS pose
    python3 seed_initialpose.py --x 3 --y -8 --yaw 3.06
"""
import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node

# Operator's click, COSS map, 2026-07-28. Yaw is about 175.4 deg.
DEFAULT_X, DEFAULT_Y = -1.839, -8.280
DEFAULT_YAW = 2 * math.atan2(0.9992, 0.0397)
COV_XY, COV_YAW = 0.25, 0.06853892326654787


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--x", type=float, default=DEFAULT_X)
    p.add_argument("--y", type=float, default=DEFAULT_Y)
    p.add_argument("--yaw", type=float, default=DEFAULT_YAW, help="[rad] in the map frame")
    p.add_argument("--repeat", type=int, default=3, help="publications, ~1 s apart")
    p.add_argument("--clock-timeout", type=float, default=60.0,
                   help="[s] to wait for /clock before giving up")
    args = p.parse_args()

    rclpy.init()
    node = Node("initialpose_seeder")
    node.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
    pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 1)

    # Wait for the bag to start driving the clock, so the stamp is sim time.
    deadline = time.time() + args.clock_timeout
    while node.get_clock().now().nanoseconds == 0 and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.get_clock().now().nanoseconds == 0:
        node.get_logger().error(
            "no /clock after %.0f s -- is the bag playing with --clock?" % args.clock_timeout)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = "map"
    msg.pose.pose.position.x = args.x
    msg.pose.pose.position.y = args.y
    msg.pose.pose.orientation.z = math.sin(args.yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(args.yaw / 2.0)
    msg.pose.covariance[0] = COV_XY
    msg.pose.covariance[7] = COV_XY
    msg.pose.covariance[35] = COV_YAW

    for _ in range(args.repeat):
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        node.get_logger().info(
            "published /initialpose at (%.3f, %.3f) yaw %.1f deg"
            % (args.x, args.y, math.degrees(args.yaw)))
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(1.0)

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
