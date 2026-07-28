"""Drive planning and control on top of a live pose source, and report.

Answers the question the accuracy gate does not: with the pose coming from
`pose_source`, does the rest of the stack actually work? Sequence is the one a
human performs through RViz -- initialize, set a route, engage -- but asserted
rather than watched, so it can run for both pose sources and be compared.

Reports, per run: whether localization reached INITIALIZED, whether routing
reached SET, whether the operation mode reached AUTONOMOUS, the rate and count
of published trajectories and control commands, and the lateral offset between
the ego pose and the planned trajectory.
"""
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from autoware_adapi_v1_msgs.msg import (LocalizationInitializationState,
                                        OperationModeState, RouteState)
from autoware_adapi_v1_msgs.srv import (ChangeOperationMode, ClearRoute,
                                        InitializeLocalization, SetRoutePoints)
from autoware_control_msgs.msg import Control
from autoware_planning_msgs.msg import Trajectory
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy)

# Sample site only. START is the ground-truth bag's first pose; GOAL was chosen
# by sweeping candidates against a live stack (goal_sweep.py). Both are specific
# to data/sample-rosbag-replay/sample-map-rosbag and must be re-derived for any
# other site.
START = (89571.1406, 42301.1719, 0.2874, 0.9578)
# Verified by sweeping poses along the ground-truth track against a live stack
# (scripts/2dlidar/goal_sweep.py): acceptance is patchy rather than monotonic -- 0.15, 0.25,
# 0.35, 0.45 and 0.85 of the track are accepted while 0.55-0.75, 0.95 and the
# final pose are rejected with "The planned route is empty" / "Goal's footprint
# exceeds lane!". Lane width is not the cause (3.08-3.58 m against a 1.896 m
# sample_vehicle); those poses simply do not sit cleanly inside a mapped
# lanelet. This is the furthest accepted goal, 79 m along the route.
GOAL = (89556.7339, 42368.0278, 0.8808, 0.4733)

# Every topic that might carry the planned trajectory in this Autoware version.
TRAJECTORY_TOPICS = (
    "/planning/trajectory",
    "/planning/scenario_planning/trajectory",
    "/planning/scenario_planning/scenario_selector/trajectory",
)

TRANSIENT_LOCAL = QoSProfile(
    depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


def make_pose(x, y, qz, qw):
    p = Pose()
    p.position.x, p.position.y = x, y
    p.orientation.z, p.orientation.w = qz, qw
    return p


def trajectory_geometry(pose_xy, traj):
    """Cross-track distance plus the diagnostics needed to interpret it.

    Autoware plans from the ego pose, so a healthy trajectory starts at the
    vehicle and `d_first` is near zero. A large `d_min` with a large `d_first`
    means the plan and the pose refer to different places -- a stale or
    mis-framed trajectory -- whereas a large `d_min` with a small `d_first`
    would mean the vehicle genuinely sits off its own planned path.
    Distinguishing those is the whole point: reporting `d_min` alone says
    nothing about which happened.

    Returns (d_min, d_first, length) in metres, or None for a degenerate
    trajectory.
    """
    pts = [(p.pose.position.x, p.pose.position.y) for p in traj.points]
    if len(pts) < 2:
        return None
    px, py = pose_xy
    best = None
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        dx, dy = x1 - x0, y1 - y0
        seg = dx * dx + dy * dy
        t = 0.0 if seg == 0 else max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / seg))
        d = math.hypot(px - (x0 + t * dx), py - (y0 + t * dy))
        best = d if best is None else min(best, d)
    d_first = math.hypot(px - pts[0][0], py - pts[0][1])
    length = sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))
    return best, d_first, length


class Probe(Node):
    def __init__(self):
        super().__init__("downstream_probe")
        self.init_state = None
        self.route_state = None
        self.mode_state = None
        self.mode_available = None
        self.odom = None
        self.traj_stamps = []
        self.ctrl_stamps = []
        self.lateral = []
        self.pairs = []
        self.d_first = []
        self.traj_len = []
        self.ego_speed = []
        self.samples = []
        self.ctrl_samples = []
        # Count the pose-estimator contract topic. Without this a run where the
        # estimator published NOTHING still passes every step: ekf_localizer
        # dead-reckons from the seed on wheel/IMU odometry alone, kinematic_state
        # flows, planning plans from it, and the probe sees a healthy stack that
        # is not localising at all.
        self.estimator_poses = 0
        # Counting estimator poses is NOT enough: a particle filter with no
        # observations still emits poses from its motion model, and a run that
        # way passed every step on an entirely empty scan. Gate on the scan
        # carrying finite returns too.
        self.scans_seen = 0
        self.scans_with_returns = 0
        self.finite_beams_max = 0

        self.create_subscription(
            LocalizationInitializationState,
            "/api/localization/initialization_state",
            lambda m: setattr(self, "init_state", m.state), TRANSIENT_LOCAL)
        self.create_subscription(
            RouteState, "/api/routing/state",
            lambda m: setattr(self, "route_state", m.state), TRANSIENT_LOCAL)
        self.create_subscription(
            OperationModeState, "/api/operation_mode/state",
            self._on_mode, TRANSIENT_LOCAL)
        self.create_subscription(Odometry, "/localization/kinematic_state",
                                 self._on_odom, 10)
        # Autoware 1.5.0 carries more than one trajectory topic name, and which
        # one the assembled stack actually publishes is not obvious from the
        # launch tree: tier4_planning_launch sets output_trajectory to
        # /planning/trajectory (what the controller subscribes to), while
        # autoware.launch.xml and awapi reference
        # /planning/scenario_planning/trajectory. Watching one name and
        # reporting "no trajectory" measured the probe, not the stack. Watch
        # every candidate and report each separately.
        self.traj_counts = {t: 0 for t in TRAJECTORY_TOPICS}
        for topic in TRAJECTORY_TOPICS:
            self.create_subscription(
                Trajectory, topic,
                lambda m, t=topic: self._on_traj(m, t), 1)
        self.create_subscription(Control, "/control/command/control_cmd",
                                 self._on_ctrl, 1)

        self.cli_init = self.create_client(InitializeLocalization,
                                           "/api/localization/initialize")
        self.cli_route = self.create_client(SetRoutePoints,
                                            "/api/routing/set_route_points")
        # Routing refuses a new goal with "The route is already set" until the
        # previous one is cleared. That message appeared in several earlier runs
        # purely because this probe never cleared.
        self.cli_clear = self.create_client(ClearRoute, "/api/routing/clear_route")
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/localization/pose_estimator/pose_with_covariance",
            lambda m: setattr(self, "estimator_poses", self.estimator_poses + 1), 10)
        self.create_subscription(LaserScan, "/scan", self._on_scan_check, 10)
        self.cli_auto = self.create_client(
            ChangeOperationMode, "/api/operation_mode/change_to_autonomous")

    def _on_mode(self, m):
        self.mode_state = m.mode
        self.mode_available = bool(m.is_autonomous_mode_available)

    def _on_scan_check(self, m):
        self.scans_seen += 1
        finite = sum(1 for r in m.ranges if math.isfinite(r) and r > 0.0)
        if finite:
            self.scans_with_returns += 1
        self.finite_beams_max = max(self.finite_beams_max, finite)

    def _on_odom(self, m):
        self.odom = m

    def _on_traj(self, m, topic):
        self.traj_counts[topic] = self.traj_counts.get(topic, 0) + 1
        self.traj_stamps.append(time.time())
        if self.odom is None:
            return
        g = trajectory_geometry((self.odom.pose.pose.position.x,
                                 self.odom.pose.pose.position.y), m)
        if g is None:
            return
        d_min, d_first, length = g
        # Pair every cross-track sample with d_first so the statistic can be
        # conditioned later: once the ego drives past the goal, the plan no
        # longer describes where the vehicle is and both blow up together,
        # which would otherwise average a tracking metric with a
        # has-overshot-the-goal metric and report neither.
        self.pairs.append((d_min, d_first))
        self.lateral.append(d_min)
        self.d_first.append(d_first)
        self.traj_len.append(length)
        self.ego_speed.append(abs(self.odom.twist.twist.linear.x))
        if len(self.samples) < 6:
            self.samples.append({
                "topic": topic,
                "ego": [round(self.odom.pose.pose.position.x, 2),
                        round(self.odom.pose.pose.position.y, 2)],
                "traj_first": [round(m.points[0].pose.position.x, 2),
                               round(m.points[0].pose.position.y, 2)],
                "d_min": round(d_min, 3), "d_first": round(d_first, 3),
                "len": round(length, 1), "n_points": len(m.points),
            })

    def _on_ctrl(self, m):
        self.ctrl_stamps.append(time.time())
        self.ctrl_samples.append((m.longitudinal.acceleration,
                                  m.lateral.steering_tire_angle))

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for(self, predicate, seconds):
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def call(self, client, request, seconds=20.0):
        if not client.wait_for_service(timeout_sec=seconds):
            return None, "service unavailable"
        future = client.call_async(request)
        end = time.time() + seconds
        while time.time() < end and rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            return None, "timeout"
        return future.result(), None


def main():
    out_path = Path(sys.argv[1])
    rclpy.init()
    node = Probe()
    report = {"steps": []}

    def step(name, ok, detail=""):
        report["steps"].append({"step": name, "ok": bool(ok), "detail": detail})
        print(f"[{'ok ' if ok else 'FAIL'}] {name}: {detail}", flush=True)

    # 1. initialize localization at the ground-truth start pose
    req = InitializeLocalization.Request()
    pwc = PoseWithCovarianceStamped()
    pwc.header.frame_id = "map"
    pwc.pose.pose = make_pose(*START)
    pwc.pose.covariance = [0.0] * 36
    for i, v in ((0, 0.25), (7, 0.25), (14, 0.25), (21, 0.068), (28, 0.068), (35, 0.068)):
        pwc.pose.covariance[i] = v
    # The ADAPI request carries only the pose; unlike
    # autoware_localization_msgs/InitializeLocalization it has no method field.
    req.pose = [pwc]
    # NDT-based sources run pose_initializer's align step, which needs the PCD
    # map loaded and a scan in hand; called too early it returns "align server
    # failed". Retry rather than treating a warm-up race as a verdict. mcl does
    # not use the align path at all (ndt_enabled follows use_ndt_pose).
    res, err, attempts = None, None, 0
    for attempts in range(1, 7):
        res, err = node.call(node.cli_init, req)
        if res is not None and res.status.success:
            break
        node.spin(10.0)
    step("localization/initialize", res is not None and res.status.success,
         f"attempts={attempts}; " + (err or (res.status.message if res else "")))

    # particle_filter is outside the pose_initializer contract, so seed it too.
    pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 1)
    for _ in range(3):
        pwc.header.stamp = node.get_clock().now().to_msg()
        pub.publish(pwc)
        node.spin(0.2)

    ok = node.wait_for(
        lambda: node.init_state == LocalizationInitializationState.INITIALIZED, 60)
    step("localization INITIALIZED", ok, f"state={node.init_state}")

    ok = node.wait_for(lambda: node.odom is not None, 30)
    step("kinematic_state flowing", ok)

    # 2. set a route to the ground-truth end pose
    node.call(node.cli_clear, ClearRoute.Request(), 10.0)
    rreq = SetRoutePoints.Request()
    rreq.header.frame_id = "map"
    rreq.header.stamp = node.get_clock().now().to_msg()
    # Let Autoware adjust the goal to fit the lane. mission_planner rejects a
    # hand-picked pose with "Goal's footprint exceeds lane!" even at poses the
    # vehicle demonstrably drove through, once the full-size sample_vehicle
    # footprint is used; this is the same allowance RViz offers, and the point
    # here is to exercise planning and control, not to hand-tune goal geometry.
    rreq.option.allow_goal_modification = True
    rreq.goal = make_pose(*GOAL)
    res, err = node.call(node.cli_route, rreq, 30.0)
    route_ok = res is not None and res.status.success
    step("routing/set_route_points", route_ok,
         err or (res.status.message if res else ""))
    ok = node.wait_for(lambda: node.route_state == RouteState.SET, 30)
    step("route SET", ok, f"state={node.route_state}")

    # 3. a trajectory must appear before autonomous mode is even meaningful
    ok = node.wait_for(lambda: len(node.traj_stamps) > 0, 45)
    step("trajectory published", ok,
         "counts=" + ", ".join(f"{t}={c}" for t, c in node.traj_counts.items()))

    node.spin(3.0)
    step("autonomous mode available", bool(node.mode_available),
         f"is_autonomous_mode_available={node.mode_available}")

    # 4. engage
    res, err = node.call(node.cli_auto, ChangeOperationMode.Request(), 20.0)
    step("change_to_autonomous", res is not None and res.status.success,
         err or (res.status.message if res else ""))
    ok = node.wait_for(lambda: node.mode_state == OperationModeState.AUTONOMOUS, 20)
    step("mode AUTONOMOUS", ok, f"mode={node.mode_state}")

    # 5. observe
    node.traj_stamps.clear()
    node.traj_counts = {t: 0 for t in TRAJECTORY_TOPICS}
    node.ctrl_stamps.clear()
    node.lateral.clear()
    node.pairs.clear()
    node.d_first.clear()
    node.traj_len.clear()
    node.ego_speed.clear()
    node.samples.clear()
    node.spin(30.0)

    def rate(stamps):
        if len(stamps) < 2:
            return 0.0
        return (len(stamps) - 1) / (stamps[-1] - stamps[0])

    lat = sorted(node.lateral)
    report["observation"] = {
        "trajectory_count": len(node.traj_stamps),
        "trajectory_counts_by_topic": dict(node.traj_counts),
        "trajectory_hz": round(rate(node.traj_stamps), 2),
        "control_count": len(node.ctrl_stamps),
        "control_hz": round(rate(node.ctrl_stamps), 2),
        "lateral_mean_m": round(sum(lat) / len(lat), 3) if lat else None,
        "d_first_mean_m": round(sum(node.d_first) / len(node.d_first), 3) if node.d_first else None,
        "d_first_max_m": round(max(node.d_first), 3) if node.d_first else None,
        "traj_len_mean_m": round(sum(node.traj_len) / len(node.traj_len), 1) if node.traj_len else None,
        "ego_speed_mean_mps": round(sum(node.ego_speed) / len(node.ego_speed), 3) if node.ego_speed else None,
        "samples": node.samples,
        # Cross-track while the plan is current (its first point within 5 m of
        # the ego, i.e. Autoware is still planning from the vehicle). This is
        # the tracking-quality number; the unconditioned one above is not.
        "tracking": (lambda cur: {
            "n": len(cur),
            "n_total": len(node.pairs),
            "cross_track_mean_m": round(sum(cur) / len(cur), 3) if cur else None,
            "cross_track_p95_m": round(sorted(cur)[int(0.95 * (len(cur) - 1))], 3) if cur else None,
            "cross_track_max_m": round(max(cur), 3) if cur else None,
        })([d for d, f in node.pairs if f <= 5.0]),
        "lateral_p95_m": round(lat[int(0.95 * (len(lat) - 1))], 3) if lat else None,
        "lateral_max_m": round(lat[-1], 3) if lat else None,
        "final_mode": node.mode_state,
        "final_route_state": node.route_state,
    }
    # The estimator must have actually published; see estimator_poses above.
    step("scan carries returns",
         node.scans_with_returns >= 20 and node.finite_beams_max >= 50,
         f"{node.scans_with_returns}/{node.scans_seen} scans with returns, "
         f"max {node.finite_beams_max} finite beams")
    report["observation"]["scans_seen"] = node.scans_seen
    report["observation"]["scans_with_returns"] = node.scans_with_returns
    report["observation"]["finite_beams_max"] = node.finite_beams_max
    step("pose estimator published", node.estimator_poses >= 20,
         f"{node.estimator_poses} poses on "
         "/localization/pose_estimator/pose_with_covariance")
    report["observation"]["estimator_poses"] = node.estimator_poses
    print(json.dumps(report["observation"], indent=2), flush=True)
    out_path.write_text(json.dumps(report, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
