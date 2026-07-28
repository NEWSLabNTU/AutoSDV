#!/usr/bin/env python3
"""Report each LiDAR ring's elevation, so a 2-D scan ring can be chosen from data.

Extracting one ring from a 3-D LiDAR gives MCL a 2-D scan whose geometry is
independent of the vehicle height (see
docs/design/mcl-user-setup-ux.md). Which ring to use is a property of the sensor
and its mounting, and the datasheet index order is not a reliable guide: on the
VLS128 in the Autoware sample bag the horizontal channel is 71, nowhere near the
middle of 0..127.

The clouds in this repo carry a per-point `elevation` field alongside `channel`,
so the horizontal ring is measurable rather than inferred.

Also printed is the ground-intercept range for a given mounting height:
a ring at elevation e mounted h above the ground stops seeing ground beyond
h / tan(-e) when it points down, and never sees ground at all when it points up.
A low vehicle with a slightly-up ring is looking at the sky, which is worth
knowing before a drive rather than after.

Usage:
    inspect_rings.py BAG --topic /sensing/lidar/top/pointcloud_raw_ex
    inspect_rings.py BAG --topic ... --height 0.3 --frames 5
"""
import argparse
import math
import statistics
import sys
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2


def collect(bag: Path, topic: str, frames: int, ring_field: str):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in types:
        raise SystemExit(
            f"{topic} is not in {bag.name}. Available cloud topics:\n  "
            + "\n  ".join(t for t, ty in types.items() if "PointCloud2" in ty))

    per_ring = {}
    seen = 0
    while reader.has_next() and seen < frames:
        name, data, _ = reader.read_next()
        if name != topic:
            continue
        seen += 1
        msg = deserialize_message(data, get_message(types[topic]))
        fields = {f.name for f in msg.fields}
        missing = {ring_field, "elevation"} - fields
        if missing:
            raise SystemExit(
                f"{topic} has no {'/'.join(sorted(missing))} field; present: "
                + ", ".join(sorted(fields))
                + f"\nTry --ring-field with one of them.")
        for ring, elev, dist in point_cloud2.read_points(
                msg, field_names=(ring_field, "elevation", "distance"),
                skip_nans=True):
            per_ring.setdefault(int(ring), []).append((float(elev), float(dist)))
    if not seen:
        raise SystemExit(f"no messages on {topic} in {bag.name}")
    return per_ring, seen


def ground_intercept(elev_rad: float, height_m: float):
    """Range at which a ring meets the ground, or None when it never does."""
    if elev_rad >= -1e-6 or height_m <= 0.0:
        return None
    return height_m / math.tan(-elev_rad)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", type=Path)
    ap.add_argument("--topic", required=True)
    ap.add_argument("--ring-field", default="channel")
    ap.add_argument("--frames", type=int, default=3)
    ap.add_argument("--height", type=float, default=None,
                    help="sensor height above ground, to report ground intercept")
    ap.add_argument("--top", type=int, default=8,
                    help="how many near-horizontal rings to list")
    args = ap.parse_args()

    per_ring, frames = collect(args.bag, args.topic, args.frames, args.ring_field)
    rows = []
    for ring, vals in per_ring.items():
        elev = statistics.median(v[0] for v in vals)
        rng = statistics.median(v[1] for v in vals)
        rows.append((abs(elev), ring, elev, len(vals), rng))
    rows.sort()

    print(f"{args.bag.name}: {frames} frame(s), {len(per_ring)} rings on {args.topic}")
    header = f"{'ring':>5}  {'elev (deg)':>11}  {'points':>8}  {'median range':>13}"
    if args.height is not None:
        header += f"  {'ground at':>10}"
    print(header)
    for _, ring, elev, n, rng in rows[: args.top]:
        line = f"{ring:5d}  {math.degrees(elev):+11.3f}  {n:8d}  {rng:10.2f} m"
        if args.height is not None:
            gi = ground_intercept(elev, args.height)
            line += f"  {'never' if gi is None else f'{gi:7.1f} m'}"
        print(line)

    best = rows[0]
    print(f"\nnearest horizontal: ring {best[1]} at {math.degrees(best[2]):+.3f} deg")
    print(f"  scan_ring:={best[1]}")
    if args.height is not None:
        gi = ground_intercept(best[2], args.height)
        if gi is None:
            print(f"  note: this ring points up or level; at {args.height} m it never "
                  "meets the ground, so it will not see kerbs or slopes")
        else:
            print(f"  note: at {args.height} m mounting height it meets the ground "
                  f"at {gi:.1f} m; beyond that it sees only upright obstacles")
    return 0


if __name__ == "__main__":
    sys.exit(main())
