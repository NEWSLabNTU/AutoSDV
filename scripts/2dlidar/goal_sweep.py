"""Try many goal poses against a live stack and report which routing accepts.

mission_planner rejects hand-picked goals with "Goal's footprint exceeds lane!"
even though the lanes (3.08-3.58 m) comfortably fit sample_vehicle (1.896 m), so
the cause is not lane width. Rather than burn a six-minute run per candidate,
sweep poses sampled along the ground-truth track in one run and report the
verdict for each.
"""
import bisect
import math
import os
import sys
import time
from pathlib import Path

import rclpy
import rosbag2_py
from autoware_adapi_v1_msgs.msg import LocalizationInitializationState
from autoware_adapi_v1_msgs.srv import (ClearRoute, InitializeLocalization,
                                        SetRoutePoints)
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

GT_BAG = os.environ.get("GT_BAG", "data/rosbags/phase3/sample_ndt_gt")
TRANSIENT_LOCAL = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


def gt_poses():
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=GT_BAG, storage_id="sqlite3"),
           rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    topic = "/localization/kinematic_state"
    out = []
    while r.has_next():
        t, d, _ = r.read_next()
        if t == topic:
            m = deserialize_message(d, get_message(types[topic]))
            p, o = m.pose.pose.position, m.pose.pose.orientation
            out.append((p.x, p.y, o.z, o.w))
    return out


def make_pose(x, y, qz, qw):
    p = Pose()
    p.position.x, p.position.y = x, y
    p.orientation.z, p.orientation.w = qz, qw
    return p


def main():
    poses = gt_poses()
    rclpy.init()
    node = Node("goal_sweep")
    state = {"init": None}
    node.create_subscription(
        LocalizationInitializationState, "/api/localization/initialization_state",
        lambda m: state.__setitem__("init", m.state), TRANSIENT_LOCAL)
    cli_init = node.create_client(InitializeLocalization, "/api/localization/initialize")
    cli_route = node.create_client(SetRoutePoints, "/api/routing/set_route_points")
    # Routing refuses a second goal with "The route is already set" until the
    # previous one is cleared -- which is also what made that message appear in
    # probe runs that reused a stack.
    cli_clear = node.create_client(ClearRoute, "/api/routing/clear_route")

    def call(cli, req, timeout=25.0):
        if not cli.wait_for_service(timeout_sec=timeout):
            return None, "service unavailable"
        fut = cli.call_async(req)
        end = time.time() + timeout
        while time.time() < end and rclpy.ok() and not fut.done():
            rclpy.spin_once(node, timeout_sec=0.05)
        return (fut.result(), None) if fut.done() else (None, "timeout")

    # initialize at the track start
    x, y, qz, qw = poses[0]
    pwc = PoseWithCovarianceStamped()
    pwc.header.frame_id = "map"
    pwc.pose.pose = make_pose(x, y, qz, qw)
    pwc.pose.covariance = [0.0] * 36
    for i, v in ((0, 0.25), (7, 0.25), (14, 0.25), (21, 0.068), (28, 0.068), (35, 0.068)):
        pwc.pose.covariance[i] = v
    req = InitializeLocalization.Request()
    req.pose = [pwc]
    for attempt in range(6):
        res, err = call(cli_init, req)
        if res is not None and res.status.success:
            break
        for _ in range(100):
            rclpy.spin_once(node, timeout_sec=0.1)
    print(f"initialize: {'ok' if res and res.status.success else 'FAILED'}", flush=True)

    end = time.time() + 30
    while time.time() < end and state["init"] != LocalizationInitializationState.INITIALIZED:
        rclpy.spin_once(node, timeout_sec=0.1)
    print(f"init state: {state['init']}", flush=True)

    for frac in (0.15, 0.25, 0.35, 0.45, 0.55, 0.65, 0.75, 0.85, 0.95, 1.0):
        i = min(int(frac * (len(poses) - 1)), len(poses) - 1)
        gx, gy, gqz, gqw = poses[i]
        for allow_mod in (False, True):
            call(cli_clear, ClearRoute.Request(), 10.0)
            rreq = SetRoutePoints.Request()
            rreq.header.frame_id = "map"
            rreq.header.stamp = node.get_clock().now().to_msg()
            rreq.option.allow_goal_modification = allow_mod
            rreq.goal = make_pose(gx, gy, gqz, gqw)
            res, err = call(cli_route, rreq, 20.0)
            ok = res is not None and res.status.success
            msg = err or (res.status.message if res else "")
            print(f"  frac={frac:<5} idx={i:<5} allow_mod={allow_mod!s:<5} "
                  f"{'OK' if ok else 'fail'}: {msg}", flush=True)
            if ok:
                # Report and keep going: knowing WHICH goals are accepted maps how
                # far the lanelet coverage extends, which one early return hides.
                print(f"ACCEPTED goal frac={frac} idx={i} "
                      f"pose=({gx:.4f},{gy:.4f},{gqz:.4f},{gqw:.4f}) "
                      f"allow_mod={allow_mod}", flush=True)
                break
    print("sweep complete", flush=True)
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
