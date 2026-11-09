#!/usr/bin/env python3
"""Drive car A's twin along a route: initialize localization at the route's start, set the
goal through Autoware's routing API, engage, and wait until it arrives.

    twin_go.py [--route coss_loop] [--routes config/routes.yaml] [--timeout 600]
    twin_go.py --spawn [--route coss_loop]      print the runner's pose:= for the route's start

The twin has no GNSS (car A runs use_gnss: false), so the initial pose is given, at the
route's start, where the runner spawned the car. After the golf cart twin's twin_go.py
(NEWSLabNTU/2026-golf-cart, src/simulation/golfcart_twin/scripts).
Prints the time, the top speed seen and the distance driven; exits 0 on arrival.
"""
import argparse
import math
import sys
import time
from collections import namedtuple
from pathlib import Path

import yaml

Pose = namedtuple("Pose", "x y z qx qy qz qw yaw")
ROUTES = Path(__file__).resolve().parents[1] / "config" / "routes.yaml"


def load_route(path, name):
    """(start, goal) Poses of route `name` in the routes file."""
    routes = yaml.safe_load(Path(path).read_text())
    if name not in routes:
        raise SystemExit(f"no route {name!r}: known routes are {', '.join(sorted(routes))}")

    def pose(p):
        y = float(p["yaw"])
        return Pose(float(p["x"]), float(p["y"]), float(p["z"]), 0.0, 0.0,
                    math.sin(y / 2), math.cos(y / 2), y)

    r = routes[name]
    return pose(r["start"]), pose(r["goal"])


def spawn_arg(p):
    """The runner's pose:= (x,y,z,yaw), 5 cm above the road, as a spawn."""
    return f"{p.x:g},{p.y:g},{p.z + 0.05:g},{p.yaw:g}"


def drive(start, goal, timeout):
    import rclpy
    from autoware_adapi_v1_msgs.msg import (LocalizationInitializationState, OperationModeState,
                                            RouteState)
    from autoware_adapi_v1_msgs.srv import (ChangeOperationMode, ClearRoute,
                                            InitializeLocalization, SetRoutePoints)
    from geometry_msgs.msg import Pose as RosPose
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile

    rclpy.init()
    node = rclpy.create_node("twin_go", parameter_overrides=[
        Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    seen = {"top": 0.0, "dist": 0.0, "last": None}
    node.create_subscription(LocalizationInitializationState, "/api/localization/initialization_state",
                             lambda m: seen.__setitem__("loc", m.state), latched)
    node.create_subscription(OperationModeState, "/api/operation_mode/state",
                             lambda m: seen.__setitem__("mode", m), latched)
    node.create_subscription(RouteState, "/api/routing/state",
                             lambda m: seen.__setitem__("route", m.state), latched)

    def on_odom(m):
        p, v = m.pose.pose.position, m.twist.twist.linear.x
        seen["top"] = max(seen["top"], abs(v))
        if seen["last"] is not None:
            seen["dist"] += math.hypot(p.x - seen["last"][0], p.y - seen["last"][1])
        seen["last"] = (p.x, p.y)
    node.create_subscription(Odometry, "/localization/kinematic_state", on_odom, 10)

    def wait(cond, what, limit):
        t_end = time.monotonic() + limit
        while not cond():
            if time.monotonic() > t_end:
                sys.exit(f"[twin_go] timed out waiting for {what}")
            rclpy.spin_once(node, timeout_sec=0.2)
        print(f"[twin_go] {what}", flush=True)

    def call(srv_type, name, req, limit=30):
        # limit: initialization runs NDT's Monte Carlo alignment, which takes far longer
        cli = node.create_client(srv_type, name)
        wait(lambda: cli.service_is_ready(), f"{name} up", 300)
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=limit)
        res = fut.result()
        if res is None:
            sys.exit(f"[twin_go] {name}: no answer in {limit:g} s")
        if not res.status.success:
            sys.exit(f"[twin_go] {name} failed: {res.status.message}")
        return res

    def ros_pose(p):
        r = RosPose()
        r.position.x, r.position.y, r.position.z = p.x, p.y, p.z
        r.orientation.z, r.orientation.w = p.qz, p.qw
        return r

    t0 = time.monotonic()
    init = InitializeLocalization.Request()
    pw = PoseWithCovarianceStamped()
    pw.header.frame_id = "map"
    pw.pose.pose = ros_pose(start)
    for i, c in ((0, 0.25), (7, 0.25), (35, 0.01)):   # x, y, yaw variances
        pw.pose.covariance[i] = c
    init.pose = [pw]
    call(InitializeLocalization, "/api/localization/initialize", init, limit=180)
    wait(lambda: seen.get("loc") == LocalizationInitializationState.INITIALIZED, "localized", 120)
    call(ClearRoute, "/api/routing/clear_route", ClearRoute.Request())
    # the routing state goes UNSET after the call returns; setting a route before then is refused
    wait(lambda: seen.get("route") == RouteState.UNSET, "route cleared", 30)
    req = SetRoutePoints.Request()
    req.header.frame_id = "map"
    req.goal = ros_pose(goal)
    call(SetRoutePoints, "/api/routing/set_route_points", req)
    wait(lambda: seen.get("route") == RouteState.SET, "route set", 60)
    wait(lambda: getattr(seen.get("mode"), "is_autonomous_mode_available", False),
         "autonomous mode available", 120)
    call(ChangeOperationMode, "/api/operation_mode/change_to_autonomous", ChangeOperationMode.Request())
    t_drive = time.monotonic()
    wait(lambda: seen.get("route") == RouteState.ARRIVED, "arrived", timeout)
    print(f"[twin_go] arrived: {time.monotonic() - t_drive:.1f} s driving ({time.monotonic() - t0:.1f} s "
          f"in all), top speed {seen['top']:.2f} m/s, {seen['dist']:.1f} m", flush=True)
    rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--route", default="coss_loop")
    ap.add_argument("--routes", default=str(ROUTES))
    ap.add_argument("--timeout", type=float, default=600.0)
    ap.add_argument("--spawn", action="store_true", help="print the route start's pose:= and exit")
    a = ap.parse_args()
    start, goal = load_route(a.routes, a.route)
    if a.spawn:
        print(spawn_arg(start))
        return
    drive(start, goal, a.timeout)


if __name__ == "__main__":
    main()
