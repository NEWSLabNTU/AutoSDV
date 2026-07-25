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


def _read_bag(bag_path, topic, msg_field, time_source="stamp"):
    """Read a rosbag2 topic into a list of (t, x, y, yaw) tuples.

    msg_field selects the pose accessor: 'odometry' for
    nav_msgs/Odometry (pose.pose), 'pose_stamped' for
    geometry_msgs/PoseStamped (pose). Imported lazily so plain-shell
    unit tests never need rosbag2_py/rclpy installed.

    time_source selects the alignment clock: 'stamp' (default) uses each
    message's own `header.stamp` (its publisher's clock at the moment it
    was generated). 'bag' instead uses the rosbag2 storage receive-time
    recorded for that message (the same clock `ros2 bag play --clock`
    uses to pace playback). These normally agree; they can diverge when a
    bag is itself a *replay-then-record* of an earlier bag (a two-hop
    "GT bag used as the PF replay source" chain, as in the Phase 3b
    sample-site run): the recorded message content still carries the
    *original* source bag's sim-time stamp, while the enclosing bag's own
    storage clock reflects the wall-clock time it was captured at. A
    consumer that replays that enclosing bag (and stamps its own output
    via sim time, e.g. the particle filter) ends up on the *storage*
    clock basis, not the original content-stamp basis, so alignment
    against a `stamp`-sourced GT track produces zero overlapping pairs.
    See docs/reports/2dlidar-phase3b-sample-site.md for the concrete case.
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

    if time_source not in ("stamp", "bag"):
        raise ValueError("time_source must be 'stamp' or 'bag', got %r" % time_source)

    track = []
    while reader.has_next():
        (name, data, bag_t_ns) = reader.read_next()
        if name != topic:
            continue
        msg = deserialize_message(data, msg_type)
        if msg_field == "odometry":
            pose = msg.pose.pose
        else:
            pose = msg.pose
        if time_source == "bag":
            t = bag_t_ns * 1e-9
        else:
            stamp = msg.header.stamp
            t = stamp.sec + stamp.nanosec * 1e-9
        yaw = quat_to_yaw(
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        )
        track.append((t, pose.position.x, pose.position.y, yaw))

    track.sort(key=lambda p: p[0])
    return track


def read_gt_bag(bag_path, time_source="stamp"):
    return _read_bag(bag_path, "/localization/kinematic_state", "odometry", time_source=time_source)


def read_pf_bag(bag_path, time_source="stamp"):
    return _read_bag(bag_path, "/pf/viz/inferred_pose", "pose_stamped", time_source=time_source)


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


def generate_report(gt_bag, pf_bag, full_stats, motion_stats, motion_window_actual,
                     all_pass, notes, motion_window_cfg=(MOTION_WINDOW_START_S, MOTION_WINDOW_END_S)):
    overall = "PASS" if all_pass else "FAIL"
    threshold_table, _ = _threshold_table(full_stats)
    window_section = []
    if motion_window_cfg is None:
        window_section = [
            "## Motion-Window Statistics",
            "",
            "_Motion-window analysis disabled (`--no-motion-window`); this bag moves "
            "throughout the recording, so a fixed stationary/motion split is not "
            "meaningful here. Only full-overlap statistics are reported._",
        ]
    else:
        cfg_start, cfg_end = motion_window_cfg
        if motion_window_actual:
            w_start, w_end = motion_window_actual
            window_heading = "## Motion-Window Statistics (%.1fs–%.1fs, %.1f s of motion data, source-bag sim time)" % (
                w_start, w_end, w_end - w_start,
            )
        else:
            window_heading = "## Motion-Window Statistics (%.0fs–%.0fs configured, source-bag sim time)" % (
                cfg_start, cfg_end,
            )
        window_section = [
            window_heading,
            "",
            _fmt_stats_table(motion_stats) if motion_stats else "_No pairs fell in the motion window._",
        ]
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
    ] + window_section + [
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
    parser.add_argument(
        "--gt-time-source", choices=["stamp", "bag"], default="stamp",
        help="Clock basis for the GT track: 'stamp' (default, header.stamp -- "
             "COSS behavior unchanged) or 'bag' (rosbag2 storage receive-time). "
             "Use 'bag' when the GT bag is itself the bag that was replayed to "
             "produce the PF run (a replay-then-record chain), which puts the "
             "PF track's timestamps on the GT bag's storage clock rather than "
             "its message content clock -- see _read_bag() docstring.",
    )
    window_group = parser.add_mutually_exclusive_group()
    window_group.add_argument(
        "--no-motion-window", action="store_true",
        help="Disable the motion-window section entirely (for bags that move "
             "throughout, where a fixed stationary/motion split is meaningless). "
             "Default behavior (COSS motion window) is unchanged when omitted.",
    )
    window_group.add_argument(
        "--motion-window", type=float, nargs=2, metavar=("START", "END"), default=None,
        help="Override the motion-window bounds (source-bag sim-time seconds). "
             "Default: %.0f %.0f (COSS bag stationary/motion split)."
             % (MOTION_WINDOW_START_S, MOTION_WINDOW_END_S),
    )
    args = parser.parse_args(argv)

    if args.no_motion_window:
        motion_window_cfg = None
    elif args.motion_window:
        motion_window_cfg = (args.motion_window[0], args.motion_window[1])
    else:
        motion_window_cfg = (MOTION_WINDOW_START_S, MOTION_WINDOW_END_S)

    gt = read_gt_bag(args.gt_bag, time_source=args.gt_time_source)
    pf = read_pf_bag(args.pf_bag)

    errors = align_tracks(gt, pf, max_dt=args.max_dt)
    full_stats = stats(errors)

    # Motion-window subset: re-align restricted to PF samples whose GT
    # partner timestamp falls in the motion window. Re-derive from raw
    # tracks so the window is applied in source-bag sim time consistently.
    motion_stats = None
    motion_window_actual = None
    if motion_window_cfg is not None:
        w_start, w_end = motion_window_cfg
        gt_times = [p[0] for p in gt]
        t0 = gt_times[0]
        gt_window = [p for p in gt if w_start <= (p[0] - t0) <= w_end]
        pf_window = [p for p in pf if w_start <= (p[0] - t0) <= w_end]
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

    if motion_window_cfg is None:
        motion_duration_text = None
    elif motion_window_actual:
        motion_duration_text = "%.1f s of overlapping motion data, %.1fs-%.1fs" % (
            motion_window_actual[1] - motion_window_actual[0],
            motion_window_actual[0],
            motion_window_actual[1],
        )
    else:
        motion_duration_text = "~40s configured, but no overlapping pairs were found"

    notes_lines = [
        "- Replay rate: 1.0x for both GT and PF recordings, replayed with `--clock` "
        "(sim time).",
        "- GT time source: `--gt-time-source=%s` (%s)." % (
            args.gt_time_source,
            "message `header.stamp` -- default, unchanged from the COSS run"
            if args.gt_time_source == "stamp"
            else "rosbag2 storage receive-time, used because the GT bag was itself "
                 "replayed to produce the PF run; see _read_bag() docstring in "
                 "compare_poses.py for why header.stamp would give zero overlap here",
        ),
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
    ]
    if motion_window_cfg is not None:
        notes_lines.append(
            "- Vehicle stationary until ~%.0fs into the source bag, then in motion for the "
            "remainder of the recording (%s); thresholds gate on full-overlap stats per the "
            "plan, motion-window stats are reported separately for diagnostic context."
            % (motion_window_cfg[0], motion_duration_text)
        )
    if not all_pass:
        notes_lines.append(
            "- **Diagnostic note (not a fix):** the run above FAILs the Phase 3 gate. "
            "Full-overlap trans. mean=%.2f m, p95=%.2f m, yaw mean|err|=%.3f rad "
            "(thresholds: mean<%.1f m, p95<%.1f m, yaw<%.1f rad). Plausible contributors: "
            "(1) the synthesized wheel+IMU odometry feeding PF's motion model has no "
            "absolute correction and can accumulate heading error, which a particle filter "
            "with too few effective particles or a poor sensor model may fail to correct via "
            "scan matching; (2) `range_method=cddt` / particle count / sensor-model noise "
            "parameters were carried over from vendored defaults and may not be tuned for "
            "this scenario; (3) possible scan-to-map mismatches (z-band filter choice, "
            "`max_range`) reducing effective localization likelihood signal; (4) ground-truth "
            "quality itself (see report notes on the GT source) may be a contributing factor. "
            "PF parameter tuning is explicitly out of scope for this gate and is deferred to "
            "a Phase-3 follow-up."
            % (full_stats["trans_mean"], full_stats["trans_p95"], full_stats["yaw_mean_abs"],
               TRANS_MEAN_THRESHOLD, TRANS_P95_THRESHOLD, YAW_MEAN_ABS_THRESHOLD)
        )
    notes = "\n".join(notes_lines)

    report = generate_report(
        args.gt_bag, args.pf_bag, full_stats, motion_stats, motion_window_actual, all_pass, notes,
        motion_window_cfg=motion_window_cfg,
    )

    if args.out:
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.out).write_text(report)
    else:
        print(report)

    return 0 if all_pass else 1


if __name__ == "__main__":
    sys.exit(main())
