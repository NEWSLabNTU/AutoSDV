#!/usr/bin/env python3
"""Compare NDT ground-truth poses against particle_filter (PF) poses.

Phase 3 Task 4 gate: reads two rosbag2 recordings — an NDT ground-truth
track (`nav_msgs/Odometry` on `/localization/kinematic_state`) and a PF
track (`geometry_msgs/PoseStamped` on `/pf/viz/inferred_pose`) — aligns
them by nearest timestamp, computes translational/yaw error statistics,
and writes a markdown report. Exits 0 iff the Phase 3 thresholds are met.

Bag reading uses `rosbag2_py` (rclpy/ROS environment required); the
alignment/statistics functions below are pure and import nothing ROS,
so they are unit-testable in a plain shell (see test_compare_poses.py).
"""
import argparse
import bisect
import math
import sys
from pathlib import Path

# Phase 3 gate thresholds (full-overlap stats). Loose by design — PF
# parameter tuning is a Phase-3 follow-up, not part of this gate.
TRANS_MEAN_THRESHOLD = 1.0  # m
TRANS_P95_THRESHOLD = 2.5  # m
YAW_MEAN_ABS_THRESHOLD = 0.2  # rad

# Vehicle is stationary until ~116s into the source bag, then moves for
# ~40s. Both bags were replayed from the same source bag at -r 1.0 with
# --clock, so these are sim-time seconds and directly comparable across
# the two bags without any additional offset.
MOTION_WINDOW_START_S = 116.0
MOTION_WINDOW_END_S = 156.0


def quat_to_yaw(qx, qy, qz, qw):
    """Yaw (rotation about Z) from a quaternion, via atan2 — no tf dependency."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_error(a, b):
    """Shortest signed angular difference b - a, normalized to [-pi, pi]."""
    d = b - a
    return math.atan2(math.sin(d), math.cos(d))


def align_tracks(gt, pf, max_dt=0.1):
    """Pair each PF sample with its nearest-timestamp GT sample.

    gt, pf: list of (t, x, y, yaw) tuples, sorted by t ascending.
    Pairs whose |dt| > max_dt are rejected. Returns list of
    (trans_err, yaw_err) tuples. Raises ValueError if either track is
    empty, or if no pair survives the max_dt filter (empty overlap).
    """
    if not gt or not pf:
        raise ValueError("align_tracks: gt and pf tracks must both be non-empty")

    gt_times = [p[0] for p in gt]
    errors = []
    for t_pf, x_pf, y_pf, yaw_pf in pf:
        idx = bisect.bisect_left(gt_times, t_pf)
        candidates = []
        if idx < len(gt_times):
            candidates.append(idx)
        if idx > 0:
            candidates.append(idx - 1)
        if not candidates:
            continue
        best_idx = min(candidates, key=lambda i: abs(gt_times[i] - t_pf))
        dt = abs(gt_times[best_idx] - t_pf)
        if dt > max_dt:
            continue
        t_gt, x_gt, y_gt, yaw_gt = gt[best_idx]
        trans_err = math.hypot(x_pf - x_gt, y_pf - y_gt)
        yerr = yaw_error(yaw_gt, yaw_pf)
        errors.append((trans_err, yerr))

    if not errors:
        raise ValueError(
            "align_tracks: no overlapping pairs within max_dt=%s "
            "(empty overlap between gt and pf tracks)" % max_dt
        )
    return errors


def _percentile(sorted_vals, pct):
    """Linear-interpolation percentile, matching numpy's default method."""
    n = len(sorted_vals)
    if n == 1:
        return sorted_vals[0]
    rank = (pct / 100.0) * (n - 1)
    lo = int(math.floor(rank))
    hi = int(math.ceil(rank))
    if lo == hi:
        return sorted_vals[lo]
    frac = rank - lo
    return sorted_vals[lo] + frac * (sorted_vals[hi] - sorted_vals[lo])


def stats(errors):
    """Compute error statistics from a list of (trans_err, yaw_err) tuples.

    Returns dict with trans_mean/trans_rms/trans_max/trans_p95 (m) and
    yaw_mean_abs/yaw_max_abs (rad). Raises ValueError on empty input.
    """
    if not errors:
        raise ValueError("stats: errors list must be non-empty")

    trans = sorted(e[0] for e in errors)
    yaw_abs = [abs(e[1]) for e in errors]
    n = len(trans)

    return {
        "n": n,
        "trans_mean": sum(trans) / n,
        "trans_rms": math.sqrt(sum(v * v for v in trans) / n),
        "trans_max": max(trans),
        "trans_p95": _percentile(trans, 95),
        "yaw_mean_abs": sum(yaw_abs) / n,
        "yaw_max_abs": max(yaw_abs),
    }


def _read_bag(bag_path, topic, msg_field):
    """Read a rosbag2 topic into a list of (t, x, y, yaw) tuples.

    msg_field selects the pose accessor: 'odometry' for
    nav_msgs/Odometry (pose.pose), 'pose_stamped' for
    geometry_msgs/PoseStamped (pose). Imported lazily so plain-shell
    unit tests never need rosbag2_py/rclpy installed.
    """
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}
    if topic not in type_map:
        raise ValueError(
            "topic %r not found in bag %s (available: %s)"
            % (topic, bag_path, sorted(type_map))
        )
    msg_type = get_message(type_map[topic])

    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))

    track = []
    while reader.has_next():
        (name, data, _bag_t_ns) = reader.read_next()
        if name != topic:
            continue
        msg = deserialize_message(data, msg_type)
        if msg_field == "odometry":
            pose = msg.pose.pose
        else:
            pose = msg.pose
        stamp = msg.header.stamp
        t = stamp.sec + stamp.nanosec * 1e-9
        yaw = quat_to_yaw(
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        )
        track.append((t, pose.position.x, pose.position.y, yaw))

    track.sort(key=lambda p: p[0])
    return track


def read_gt_bag(bag_path):
    return _read_bag(bag_path, "/localization/kinematic_state", "odometry")


def read_pf_bag(bag_path):
    return _read_bag(bag_path, "/pf/viz/inferred_pose", "pose_stamped")


def _fmt_stats_table(s):
    lines = [
        "| Metric | Value |",
        "|---|---|",
        "| n (pairs) | %d |" % s["n"],
        "| Trans. mean (m) | %.4f |" % s["trans_mean"],
        "| Trans. RMS (m) | %.4f |" % s["trans_rms"],
        "| Trans. max (m) | %.4f |" % s["trans_max"],
        "| Trans. p95 (m) | %.4f |" % s["trans_p95"],
        "| Yaw mean \\|err\\| (rad) | %.4f |" % s["yaw_mean_abs"],
        "| Yaw max \\|err\\| (rad) | %.4f |" % s["yaw_max_abs"],
    ]
    return "\n".join(lines)


def _threshold_table(s):
    checks = [
        ("Mean translational error < %.1f m" % TRANS_MEAN_THRESHOLD,
         s["trans_mean"], TRANS_MEAN_THRESHOLD, s["trans_mean"] < TRANS_MEAN_THRESHOLD),
        ("p95 translational error < %.1f m" % TRANS_P95_THRESHOLD,
         s["trans_p95"], TRANS_P95_THRESHOLD, s["trans_p95"] < TRANS_P95_THRESHOLD),
        ("Mean \\|yaw\\| error < %.1f rad" % YAW_MEAN_ABS_THRESHOLD,
         s["yaw_mean_abs"], YAW_MEAN_ABS_THRESHOLD, s["yaw_mean_abs"] < YAW_MEAN_ABS_THRESHOLD),
    ]
    lines = ["| Threshold | Value | Limit | Result |", "|---|---|---|---|"]
    all_pass = True
    for name, value, limit, ok in checks:
        all_pass = all_pass and ok
        lines.append("| %s | %.4f | %.4f | %s |" % (name, value, limit, "PASS" if ok else "FAIL"))
    return "\n".join(lines), all_pass


def generate_report(gt_bag, pf_bag, full_stats, motion_stats, motion_window_actual, all_pass, notes):
    overall = "PASS" if all_pass else "FAIL"
    threshold_table, _ = _threshold_table(full_stats)
    if motion_window_actual:
        w_start, w_end = motion_window_actual
        window_heading = "## Motion-Window Statistics (%.1fs–%.1fs, %.1f s of motion data, source-bag sim time)" % (
            w_start, w_end, w_end - w_start,
        )
    else:
        window_heading = "## Motion-Window Statistics (%.0fs–%.0fs configured, source-bag sim time)" % (
            MOTION_WINDOW_START_S, MOTION_WINDOW_END_S,
        )
    parts = [
        "# Phase 3: MCL (particle_filter) vs NDT Ground Truth Comparison",
        "",
        "Auto-generated by `scripts/2dlidar/compare_poses.py`. Do not hand-edit; re-run the tool to refresh.",
        "",
        "- GT bag: `%s`" % gt_bag,
        "- PF bag: `%s`" % pf_bag,
        "",
        "## Overall Result: **%s**" % overall,
        "",
        "## Full-Overlap Statistics",
        "",
        _fmt_stats_table(full_stats),
        "",
        window_heading,
        "",
        _fmt_stats_table(motion_stats) if motion_stats else "_No pairs fell in the motion window._",
        "",
        "## Thresholds (applied to full-overlap stats)",
        "",
        threshold_table,
        "",
        "## Notes",
        "",
        notes,
        "",
    ]
    return "\n".join(parts)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("gt_bag", help="Path to NDT ground-truth rosbag2 (nav_msgs/Odometry on /localization/kinematic_state)")
    parser.add_argument("pf_bag", help="Path to PF rosbag2 (geometry_msgs/PoseStamped on /pf/viz/inferred_pose)")
    parser.add_argument("--out", default=None, help="Path to write the markdown report")
    parser.add_argument("--max-dt", type=float, default=0.1, help="Max |dt| (s) for nearest-timestamp pairing")
    args = parser.parse_args(argv)

    gt = read_gt_bag(args.gt_bag)
    pf = read_pf_bag(args.pf_bag)

    errors = align_tracks(gt, pf, max_dt=args.max_dt)
    full_stats = stats(errors)

    # Motion-window subset: re-align restricted to PF samples whose GT
    # partner timestamp falls in the motion window. Re-derive from raw
    # tracks so the window is applied in source-bag sim time consistently.
    gt_times = [p[0] for p in gt]
    t0 = gt_times[0]
    gt_window = [p for p in gt if MOTION_WINDOW_START_S <= (p[0] - t0) <= MOTION_WINDOW_END_S]
    pf_window = [p for p in pf if MOTION_WINDOW_START_S <= (p[0] - t0) <= MOTION_WINDOW_END_S]
    motion_stats = None
    motion_window_actual = None
    if gt_window and pf_window:
        try:
            motion_errors = align_tracks(gt_window, pf_window, max_dt=args.max_dt)
            motion_stats = stats(motion_errors)
            # Report the actual data extent inside the configured window,
            # not the configured bounds themselves — both bags may end
            # (or the window may start) before/after the nominal bounds.
            gt_rel = [p[0] - t0 for p in gt_window]
            pf_rel = [p[0] - t0 for p in pf_window]
            motion_window_actual = (max(min(gt_rel), min(pf_rel)), min(max(gt_rel), max(pf_rel)))
        except ValueError:
            motion_stats = None

    _, all_pass = _threshold_table(full_stats)

    if motion_window_actual:
        motion_duration_text = "%.1f s of overlapping motion data, %.1fs-%.1fs" % (
            motion_window_actual[1] - motion_window_actual[0],
            motion_window_actual[0],
            motion_window_actual[1],
        )
    else:
        motion_duration_text = "~40s configured, but no overlapping pairs were found"

    notes = "\n".join([
        "- Replay rate: 1.0x for both GT and PF recordings, sourced from the same "
        "outdoor bag with `--clock` (sim time), so cross-bag timestamps are directly comparable.",
        "- PF params: `range_method=cddt`, particle count per vendored `config/localize.yaml` "
        "default (4000), `max_range=30.0` (outdoor VLP-32C), `scan_topic=/scan`, "
        "`odometry_topic=/odom`.",
        "- Odometry input to PF was synthesized wheel+IMU planar unicycle integration "
        "(`scripts/2dlidar/wheel_imu_odom.py`), not a native wheel encoder feed — "
        "odometry quality directly affects PF motion-model accuracy between scans.",
        "- GT is Autoware NDT `/localization/kinematic_state`; PF publishes pose in the "
        "occupancy-grid frame, which is sliced from the same PCD map as NDT's map frame — "
        "poses are directly comparable without a transform.",
        "- Z-band caveat: comparison is 2D (x, y, yaw) only; the GT track's z is Autoware's "
        "3D NDT estimate while PF is inherently 2D, so z is not compared.",
        "- `/scan` was bridged from `/sensing/lidar/velodyne_points` via `pointcloud_to_laserscan` "
        "with a z-band filter (see Task 3); QoS was bridged from `pointcloud_to_laserscan`'s "
        "best-effort publisher to `particle_filter`'s reliable-QoS subscription.",
        "- Vehicle stationary until ~116s into the source bag, then in motion for the "
        "remainder of the recording (%s); thresholds gate on full-overlap stats per the "
        "plan, motion-window stats are reported separately for diagnostic context."
        % motion_duration_text,
        "- **Likely cause of FAIL (diagnostic, not a fix):** translational error is small "
        "(~1 m) at the start of the GT/PF overlap and grows roughly monotonically to "
        "10-25+ m by the end of the run (motion-window mean error exceeds the full-overlap "
        "mean), i.e. the error pattern looks like unbounded drift/divergence rather than a "
        "fixed frame offset. Plausible contributors: (1) the synthesized wheel+IMU odometry "
        "feeding PF's motion model has no absolute correction and can accumulate heading "
        "error, which a particle filter with too few effective particles or a poor "
        "sensor model may fail to correct via scan matching; (2) `range_method=cddt` / "
        "particle count / sensor-model noise parameters were carried over from the vendored "
        "indoor defaults and are not tuned for this outdoor VLP-32C + COSS map scenario; "
        "(3) possible scan-to-map mismatches (z-band filter choices, `max_range=30.0`) "
        "reducing effective localization likelihood signal. PF parameter tuning is "
        "explicitly out of scope for this gate per the plan and is deferred to a "
        "Phase-3 follow-up.",
    ])

    report = generate_report(
        args.gt_bag, args.pf_bag, full_stats, motion_stats, motion_window_actual, all_pass, notes
    )

    if args.out:
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.out).write_text(report)
    else:
        print(report)

    return 0 if all_pass else 1


if __name__ == "__main__":
    sys.exit(main())
