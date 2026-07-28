"""How faithfully does a ring GROUP emulate a 2-D LiDAR?

A LaserScan is one plane: one range per bearing, all from a single elevation.
A group of adjacent 3-D rings flattened into a scan is not that -- it is a
vertical wedge, and pointcloud_to_laserscan keeps the nearest return per bearing.
The question is how wide that wedge is in metres at operating range, which
decides whether the approximation is acceptable.
"""
import math
import statistics
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2

import os

BAG = os.environ.get("BAG", "data/rosbags/phase3/sample_ndt_gt")
TOPIC = os.environ.get("TOPIC", "/sensing/lidar/top/pointcloud_raw_ex")


def main():
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=BAG, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    per = {}
    frames = 0
    while reader.has_next() and frames < 3:
        name, data, _ = reader.read_next()
        if name != TOPIC:
            continue
        frames += 1
        msg = deserialize_message(data, get_message(types[TOPIC]))
        for ch, el in point_cloud2.read_points(
                msg, field_names=("channel", "elevation"), skip_nans=True):
            per.setdefault(int(ch), []).append(float(el))

    med = {c: statistics.median(v) for c, v in per.items()}
    near = sorted(med, key=lambda c: abs(med[c]))[:7]
    print("VLS128 channels nearest horizontal (median elevation):")
    for c in near:
        print(f"  ch {c:3d}  {math.degrees(med[c]):+7.3f} deg")

    print("\nvertical extent of a ring group, as a wedge:")
    print(f"{'group':<18}{'span deg':>9}{'z@10m':>8}{'z@30m':>8}{'z@60m':>8}")
    groups = (("1 ring  71", [71]),
              ("3 rings 70-72", [70, 71, 72]),
              ("5 rings 69-73", [69, 70, 71, 72, 73]))
    for label, chs in groups:
        els = [med[c] for c in chs if c in med]
        span = max(els) - min(els)
        cells = "".join(f"{r * math.tan(span):>8.2f}" for r in (10, 30, 60))
        print(f"{label:<18}{math.degrees(span):>9.3f}{cells}")

    print("\nreference points:")
    print("  slab            fixed 0.30 m window in base_link at every range")
    print("  real 2-D LiDAR  a single plane: 0.000 deg, 0.00 m at every range")
    return 0


if __name__ == "__main__":
    sys.exit(main())
