#!/usr/bin/env python3
"""Summarise an NDT replay run: diagnostics split into init and tracking phase.

Usage: summarize_ndt_run.py <run_dir>  (run_dir from run-ndt-replay.sh)
"""
import sys
import math
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

SCALARS = {
    "/localization/pose_estimator/transform_probability": "tp",
    "/localization/pose_estimator/nearest_voxel_transformation_likelihood": "nvtl",
    "/localization/pose_estimator/iteration_num": "iter",
    "/localization/pose_estimator/exe_time_ms": "exe_ms",
    "/localization/pose_estimator/initial_to_result_distance": "i2r",
    "/localization/pose_estimator/local_optimal_solution_oscillation_num": "osc",
}
POSES = {
    "/localization/pose_estimator/pose": "ndt_pose",
    "/sensing/gnss/pose": "gnss_pose",
    "/localization/kinematic_state": "ekf",
}
SPEED = "/vehicle/status/velocity_status"


def read(bag_dir):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = set(SCALARS) | set(POSES) | {SPEED}
    out = {k: [] for k in wanted}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in wanted:
            continue
        msg = deserialize_message(data, get_message(types[topic]))
        stamp = getattr(msg, "stamp", None) or getattr(getattr(msg, "header", None), "stamp", None)
        t = stamp.sec + stamp.nanosec * 1e-9 if stamp else 0.0
        if topic in SCALARS:
            out[topic].append((t, msg.data))
        elif topic == SPEED:
            out[topic].append((t, msg.longitudinal_velocity))
        else:
            p = msg.pose.pose.position if hasattr(msg.pose, "pose") else msg.pose.position
            out[topic].append((t, p.x, p.y, p.z))
    return out


def stats(vals):
    if not vals:
        return "n=0"
    v = sorted(vals)
    n = len(v)
    mean = sum(v) / n
    return (f"n={n} mean={mean:.3f} min={v[0]:.3f} "
            f"p50={v[n // 2]:.3f} p95={v[int(n * 0.95)]:.3f} max={v[-1]:.3f}")


def phase_split(speed, thresh=0.2):
    """Return sim time at which the vehicle first exceeds `thresh` m/s."""
    for t, v in speed:
        if abs(v) > thresh:
            return t
    return None


def nearest(series, t):
    best, bd = None, 1e9
    for row in series:
        d = abs(row[0] - t)
        if d < bd:
            best, bd = row, d
    return best if bd < 0.5 else None


def main():
    run = Path(sys.argv[1])
    d = read(run / "diagnostics_bag")
    speed = d[SPEED]
    t_move = phase_split(speed)
    t0 = min((v[0][0] for v in d.values() if v), default=0.0)
    print(f"run: {run.name}")
    print(f"bag sim-time span: {t0:.1f} .. {max(s[0] for s in speed):.1f}")
    print(f"first motion (>0.2 m/s) at t={t_move:.1f} (+{t_move - t0:.1f}s)" if t_move else "no motion")

    for topic, name in SCALARS.items():
        series = d[topic]
        print(f"\n{name}:")
        print(f"  all     {stats([v for _, v in series])}")
        if t_move:
            print(f"  init    {stats([v for t, v in series if t < t_move])}")
            print(f"  track   {stats([v for t, v in series if t >= t_move])}")

    # NDT vs GNSS horizontal distance
    ndt = d["/localization/pose_estimator/pose"]
    gnss = d["/sensing/gnss/pose"]
    ekf = d["/localization/kinematic_state"]
    for label, series in (("ndt", ndt), ("ekf", ekf)):
        errs = []
        for row in series:
            g = nearest(gnss, row[0])
            if g:
                errs.append(math.hypot(row[1] - g[1], row[2] - g[2]))
        print(f"\n{label} vs gnss horizontal [m]: {stats(errs)}")

    print(f"\npublished ndt poses: {len(ndt)}   gnss fixes: {len(gnss)}   ekf: {len(ekf)}")
    if ndt:
        print(f"ndt pose sim-time span: {ndt[0][0]:.1f} .. {ndt[-1][0]:.1f}")
        gaps = [(ndt[i + 1][0] - ndt[i][0]) for i in range(len(ndt) - 1)]
        gaps.sort()
        print(f"ndt publish gaps [s]: {stats(gaps)}")


if __name__ == "__main__":
    main()
