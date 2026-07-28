#!/usr/bin/env python3
"""Align a rosbag's storage timestamps with its message header stamps.

`ros2 bag play --clock` derives /clock from a bag's *storage* timestamps, while
nodes look transforms up at a message's *header* stamp. When the two disagree,
every such lookup fails with "Lookup would require extrapolation into the past"
and anything TF-dependent silently produces nothing.

The Autoware sample rosbag disagrees by 328.9 days: storage times start at
1614315746 (2021-02-26) while header stamps start at 1585897255 (2020-04-03).
Measured consequence in AutoSDV's logging simulation: ekf_localizer publishes
map->base_link on a timer stamped at clock time, perception asks for it at the
LiDAR stamp, the lookup lands 329 days before the buffer's earliest entry, no
occupancy grid is produced, and behavior_path_planner waits forever -- so no
path, no trajectory, no control command. Localization itself survives, because
NDT matches on the cloud and the EKF publishes a pose regardless, which is why
the failure looks like a planning problem rather than a clock problem.

This shifts every storage timestamp by one constant offset rather than
re-stamping each message individually: a constant preserves inter-message
spacing exactly (so replay timing, rates and ordering are unchanged) and it
also covers messages that carry no header at all, which a per-message rewrite
could not.

Usage:
    restamp_bag.py INPUT_BAG OUTPUT_BAG [--dry-run]
"""
import argparse
import statistics
import sys
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def open_reader(uri: Path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(uri), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    return reader


def header_stamp_ns(msg):
    """Header stamp in nanoseconds, or None when the message has no header."""
    header = getattr(msg, "header", None)
    if header is None:
        return None
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def measure_offset(uri: Path):
    """Median (storage_time - header_stamp) in ns, plus per-topic detail.

    The median rather than the first sample: a bag can contain a few messages
    whose header stamp is zero or otherwise unset, and one of those as the
    first message would poison a single-sample estimate.
    """
    reader = open_reader(uri)
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    diffs = []
    per_topic = {}
    total = 0
    headerless = 0
    while reader.has_next():
        topic, data, bag_ns = reader.read_next()
        total += 1
        try:
            msg = deserialize_message(data, get_message(types[topic]))
        except Exception:
            continue
        stamp = header_stamp_ns(msg)
        if stamp is None or stamp == 0:
            headerless += 1
            continue
        d = bag_ns - stamp
        diffs.append(d)
        per_topic.setdefault(topic, []).append(d)
    if not diffs:
        raise SystemExit("no message carried a usable header stamp; cannot measure an offset")
    return int(statistics.median(diffs)), per_topic, total, headerless


def restamp(src: Path, dst: Path, offset_ns: int):
    reader = open_reader(src)
    topics = reader.get_all_topics_and_types()

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(dst), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    for t in topics:
        writer.create_topic(t)

    written = 0
    while reader.has_next():
        topic, data, bag_ns = reader.read_next()
        writer.write(topic, data, bag_ns - offset_ns)
        written += 1
    del writer
    return written


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input", type=Path)
    ap.add_argument("output", type=Path, nargs="?")
    ap.add_argument("--dry-run", action="store_true",
                    help="report the measured offset and exit without writing")
    args = ap.parse_args()

    offset, per_topic, total, headerless = measure_offset(args.input)
    print(f"{total} messages, {headerless} without a usable header stamp")
    print(f"median offset (storage - header): {offset} ns = {offset/1e9:.1f} s "
          f"= {offset/1e9/86400:.1f} days")
    spread = {t: (min(v), max(v)) for t, v in per_topic.items()}
    inconsistent = {t: v for t, v in spread.items() if v[1] - v[0] > 1_000_000_000}
    for t, (lo, hi) in sorted(spread.items()):
        flag = "  <-- varies by >1s" if t in inconsistent else ""
        print(f"  {t}: {lo/1e9:.3f}..{hi/1e9:.3f} s{flag}")
    if inconsistent:
        print("\nwarning: the offset is not constant on the topics above; a single "
              "shift will not align them exactly.", file=sys.stderr)

    if abs(offset) < 1_000_000_000:
        print("\nstorage and header times already agree within 1 s; nothing to do")
        return 0
    if args.dry_run or args.output is None:
        return 0

    if args.output.exists():
        raise SystemExit(f"{args.output} exists; refusing to overwrite")
    written = restamp(args.input, args.output, offset)
    print(f"\nwrote {written} messages to {args.output}")

    check, _, _, _ = measure_offset(args.output)
    print(f"verification: residual offset {check} ns = {check/1e9:.3f} s")
    if abs(check) > 1_000_000_000:
        raise SystemExit("re-stamped bag still disagrees by more than 1 s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
