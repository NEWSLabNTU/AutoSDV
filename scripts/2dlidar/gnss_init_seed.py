#!/usr/bin/env python3
# scripts/2dlidar/gnss_init_seed.py — Phase 4 Task 1: derive a real
# GNSS-seeded /initialpose from raw NavSatFix, using Autoware's own
# gnss_poser node (not a reimplementation of its MGRS/geography_utils
# projection math).
#
# Design: gnss_poser needs (a) /map/map_projector_info (transient_local,
# normally from autoware_map_projection_loader) before it will touch any
# NavSatFix message at all, (b) at least two NavSatFix messages to compute
# a course-over-ground heading when use_gnss_ins_orientation is false (no
# dedicated GNSS-INS orientation topic exists in this bag), and (c) a real
# gnss_link -> base_link static transform, or it silently falls back to
# identity (gnss_poser_node.cpp's get_static_transform() catches the tf2
# lookup failure, logs "Please publish TF gnss_link to base_link", and
# proceeds with an identity transform). GNSS_BAG (the raw Autoware sample
# bag) carries no /tf_static at all -- confirmed via `ros2 bag info`; only
# the enriched sample_ndt_gt GT bag has one, and reading GT_BAG's /tf_static
# here to feed the seed would smuggle in exactly the ground-truth
# dependency this task exists to remove. Instead this script starts a real
# robot_state_publisher over the sample vehicle+sensor_kit xacro (the
# actual mounting-transform source on a real vehicle, independent of any
# recorded bag) so gnss_poser's lookup resolves honestly.
#
# This script starts three real Autoware nodes as subprocesses
# (robot_state_publisher, autoware_map_projection_loader, gnss_poser),
# publishes the first N raw NavSatFix messages read out of a source bag
# onto gnss_poser's `fix` input, waits for the resulting
# geometry_msgs/PoseWithCovarianceStamped on `gnss_pose_cov`, prints it
# (space-separated, one line) to stdout, and exits. Nothing here
# reimplements gnss_poser's coordinate transform or the vehicle's mounting
# geometry -- only the "start real nodes, feed them messages, read the
# result" plumbing is new.
#
# Output line: x y z qx qy qz qw cov_xx cov_yy cov_yaw src_t0 src_t1
#              cov_full[0..35] (Phase 4 Task 2: full row-major 6x6
#              covariance, space-separated, appended after src_t1)
# (src_t0/src_t1: elapsed seconds of the two source NavSatFix messages used,
# relative to the source bag's first message -- for correlating the seed
# against a ground-truth bag that shares the same replay start time.)
import argparse
import os
import subprocess
import sys
import time


def read_navsatfix_messages(bag_path, topic, count):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in type_map:
        raise SystemExit(f"FAIL: topic {topic} not found in {bag_path}")
    msg_type = get_message(type_map[topic])

    out = []
    first_t = None
    while reader.has_next() and len(out) < count:
        rtopic, data, t = reader.read_next()
        if rtopic != topic:
            continue
        if first_t is None:
            first_t = t
        out.append((deserialize_message(data, msg_type), (t - first_t) / 1e9))
    if len(out) < count:
        raise SystemExit(
            f"FAIL: only found {len(out)}/{count} messages on {topic} in {bag_path}"
        )
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gnss-bag", required=True)
    ap.add_argument("--gnss-topic", default="/sensing/gnss/ublox/nav_sat_fix")
    ap.add_argument("--map-projector-info", required=True)
    ap.add_argument("--lanelet2-map", required=True)
    ap.add_argument("--n-msgs", type=int, default=2)
    ap.add_argument("--namespace", default="/gnss_seed")
    ap.add_argument("--timeout-s", type=float, default=20.0)
    ap.add_argument("--vehicle-model", default="sample_vehicle")
    ap.add_argument("--sensor-model", default="sample_sensor_kit")
    ap.add_argument(
        "--vehicle-xacro",
        default="/opt/autoware/1.5.0/share/tier4_vehicle_launch/urdf/vehicle.xacro",
        help="Same xacro tier4_vehicle_launch/vehicle.launch.xml uses to start "
        "robot_state_publisher -- this is the real mounting-transform source "
        "(base_link -> ... -> gnss_link) a live vehicle bring-up would use, "
        "not a value recovered from any recorded bag.",
    )
    args = ap.parse_args()

    fixes = read_navsatfix_messages(args.gnss_bag, args.gnss_topic, args.n_msgs)

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from sensor_msgs.msg import NavSatFix, NavSatStatus

    ns = args.namespace
    procs = []
    logs_dir = os.path.join(os.path.dirname(args.gnss_bag), "..", "tmp")

    def start(cmd, log_path):
        log_f = open(log_path, "w")
        p = subprocess.Popen(
            ["setsid"] + cmd, stdout=log_f, stderr=subprocess.STDOUT
        )
        procs.append(p)
        return p

    tmp_dir = os.path.join(os.getcwd(), "tmp")
    os.makedirs(tmp_dir, exist_ok=True)

    try:
        # --- robot_state_publisher: real static-TF source for gnss_link ->
        # base_link (see module docstring). Same xacro tier4_vehicle_launch/
        # vehicle.launch.xml uses; config_dir mirrors that launch file's
        # default ($(sensor_model)_description/config).
        config_dir = os.path.join(
            "/opt/autoware/1.5.0/share", f"{args.sensor_model}_description", "config"
        )
        urdf = subprocess.run(
            [
                "xacro", args.vehicle_xacro,
                f"vehicle_model:={args.vehicle_model}",
                f"sensor_model:={args.sensor_model}",
                f"config_dir:={config_dir}",
            ],
            check=True, capture_output=True, text=True,
        ).stdout
        rsp_params_path = os.path.join(tmp_dir, "gnss_seed_robot_description.yaml")
        with open(rsp_params_path, "w") as f:
            f.write("robot_state_publisher:\n  ros__parameters:\n    robot_description: |\n")
            for line in urdf.splitlines():
                f.write("      " + line + "\n")
        # NOTE: deliberately NOT remapped -- publishes on the standard /tf,
        # /tf_static so gnss_poser's (also unremapped) tf2 listener sees it,
        # exactly as on a real vehicle bring-up.
        start(
            [
                "ros2", "run", "robot_state_publisher", "robot_state_publisher",
                "--ros-args", "--params-file", rsp_params_path,
            ],
            os.path.join(tmp_dir, "gnss_seed_rsp.log"),
        )

        start(
            [
                "ros2", "run", "autoware_map_projection_loader",
                "autoware_map_projection_loader_node", "--ros-args",
                "-p", f"map_projector_info_path:={args.map_projector_info}",
                "-p", f"lanelet2_map_path:={args.lanelet2_map}",
                "-r", f"/map/map_projector_info:={ns}/map/map_projector_info",
            ],
            os.path.join(tmp_dir, "gnss_seed_maploader.log"),
        )
        start(
            [
                "ros2", "run", "autoware_gnss_poser", "gnss_poser", "--ros-args",
                "-r", f"fix:={ns}/fix",
                "-r", f"autoware_orientation:={ns}/autoware_orientation",
                "-r", f"gnss_pose:={ns}/gnss_pose",
                "-r", f"gnss_pose_cov:={ns}/gnss_pose_cov",
                "-r", f"gnss_fixed:={ns}/gnss_fixed",
                "-r", f"/map/map_projector_info:={ns}/map/map_projector_info",
                "-p", "use_gnss_ins_orientation:=false",
                "-p", "base_frame:=base_link",
                "-p", "gnss_base_frame:=gnss_base_link",
                "-p", "map_frame:=map",
                "-p", "buff_epoch:=1",
                "-p", "gnss_pose_pub_method:=0",
            ],
            os.path.join(tmp_dir, "gnss_seed_poser.log"),
        )

        rclpy.init()
        node = Node("gnss_init_seed_driver")

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        fix_pub = node.create_publisher(NavSatFix, f"{ns}/fix", sensor_qos)

        received = []

        def on_pose_cov(msg):
            received.append(msg)

        node.create_subscription(
            PoseWithCovarianceStamped, f"{ns}/gnss_pose_cov", on_pose_cov, 10
        )

        # Give the two nodes time to come up and subscribe/advertise before
        # publishing (map_projection_loader's transient_local publisher must
        # exist before gnss_poser's transient_local subscription connects).
        deadline = time.time() + args.timeout_s
        while fix_pub.get_subscription_count() == 0 and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        if fix_pub.get_subscription_count() == 0:
            raise SystemExit("FAIL: gnss_poser never subscribed to fix topic")

        for src_msg, _elapsed in fixes:
            msg = NavSatFix()
            msg.header = src_msg.header
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.status = src_msg.status
            msg.latitude = src_msg.latitude
            msg.longitude = src_msg.longitude
            msg.altitude = src_msg.altitude
            msg.position_covariance = src_msg.position_covariance
            msg.position_covariance_type = src_msg.position_covariance_type
            fix_pub.publish(msg)
            # spin briefly between publishes so gnss_poser processes them in
            # order and its internal prev_position state updates
            spin_until = time.time() + 1.0
            while time.time() < spin_until:
                rclpy.spin_once(node, timeout_sec=0.1)

        deadline = time.time() + args.timeout_s
        while len(received) < len(fixes) and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)

        if not received:
            raise SystemExit("FAIL: gnss_poser produced no gnss_pose_cov output")

        result = received[-1]
        p = result.pose.pose.position
        o = result.pose.pose.orientation
        cov = result.pose.covariance
        # Phase 4 Task 2: emit the full 36-element row-major 6x6 covariance
        # (not just the xx/yy/yaw-yaw diagonal already printed above for
        # backward compatibility / the derived-scalar fallback) so the
        # caller can populate /initialpose's covariance field directly and
        # let the particle_filter fork's multivariate-normal sampler
        # (build_pose_covariance_marginal/sample_pose_particles) consume
        # gnss_poser's real (x, y, yaw) marginal -- including any
        # off-diagonal correlation gnss_poser itself reports -- instead of
        # only the diagonal-derived scalar spreads.
        cov_full = " ".join(str(v) for v in cov)
        print(
            f"{p.x} {p.y} {p.z} {o.x} {o.y} {o.z} {o.w} "
            f"{cov[0]} {cov[7]} {cov[35]} "
            f"{fixes[0][1]} {fixes[-1][1]} "
            f"{cov_full}"
        )

        node.destroy_node()
        rclpy.shutdown()
    finally:
        for p in procs:
            try:
                os.killpg(os.getpgid(p.pid), 15)
            except ProcessLookupError:
                pass
        time.sleep(1)
        for p in procs:
            try:
                os.killpg(os.getpgid(p.pid), 9)
            except ProcessLookupError:
                pass


if __name__ == "__main__":
    main()
