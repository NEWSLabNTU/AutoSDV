#!/usr/bin/env python3
"""Heading-minus-course-over-ground bias, measured on straight segments only.

A vehicle driving straight has heading == course. A constant difference is a
yaw offset between base_link and the sensor NDT actually localises, i.e. a
mounting calibration error. Turns are excluded because a finite chord
systematically lags the instantaneous heading through a curve.

Usage: yaw_bias.py label=<run_bag_dir> [...]
"""
import math
import sys
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# Resolved from this file so the script works from any working directory.
REPO = Path(__file__).resolve().parents[3]
RAW = str(REPO / "data/rosbags/outdoor_20251226_153115")
N = "/localization/pose_estimator/pose"
V = "/vehicle/status/velocity_status"
IMU = "/sensing/camera/zedxm/imu/data"
RATE_LIMIT_DEG = 2.0
BASELINE_S = 1.5
MIN_MOVE_M = 0.6


def yaw_of(o):
    return math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z))


def read(uri, topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=uri, storage_id="sqlite3"),
           rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    out = {t: [] for t in topics}
    while r.has_next():
        t, data, _ = r.read_next()
        if t in out:
            out[t].append(deserialize_message(data, get_message(types[t])))
    return out


def main():
    imu = read(RAW, [IMU])[IMU]
    gt = np.array([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in imu])
    gw = np.array([m.angular_velocity.z for m in imu])

    def rate_at(t):
        i = min(max(np.searchsorted(gt, t), 5), len(gt) - 6)
        return math.degrees(abs(np.mean(gw[i - 5:i + 5])))

    for arg in sys.argv[1:]:
        label, path = arg.split("=", 1)
        d = read(path, [N, V])
        p = [(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
              m.pose.position.x, m.pose.position.y, yaw_of(m.pose.orientation))
             for m in d[N]]
        v = [(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
              m.longitudinal_velocity) for m in d[V]]
        tm = next(t for t, s in v if abs(s) > 0.2)
        te = max(t for t, s in v if abs(s) > 0.2)
        p = [x for x in p if tm <= x[0] <= te]

        diffs = []
        for i in range(len(p)):
            j = i
            while j < len(p) and p[j][0] - p[i][0] < BASELINE_S:
                j += 1
            if j >= len(p):
                break
            if rate_at((p[i][0] + p[j][0]) / 2) > RATE_LIMIT_DEG:
                continue
            dx, dy = p[j][1] - p[i][1], p[j][2] - p[i][2]
            if math.hypot(dx, dy) < MIN_MOVE_M:
                continue
            dh = (p[i][3] - math.atan2(dy, dx) + math.pi) % (2 * math.pi) - math.pi
            diffs.append(math.degrees(dh))
        a = np.array(diffs)
        if len(a) == 0:
            print(f"{label:>16}: no straight segments")
            continue
        print(f"{label:>16}: heading-course mean {a.mean():+7.2f} deg  "
              f"median {np.median(a):+7.2f}  sd {a.std():5.2f}  n={len(a)}")


if __name__ == "__main__":
    main()
