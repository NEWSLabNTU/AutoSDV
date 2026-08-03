#!/usr/bin/env python3
"""Export the exact inputs NDT saw during a run, for offline replay.

A live replay cannot compare two matchers against each other. The slower one
misses the scan budget, drops scans, and every dropped scan leaves a staler
prior, which scores worse, which fails the convergence gate -- so what gets
measured is a collapse rather than a speed ratio
(`docs/superpowers/plans/2026-08-03-ndt-gpu-vs-cpu-profiling-orin.md`).

This writes the (scan, initial guess) pairs the stack actually used, together
with the map, into one flat file. Feeding that identical sequence to each arm
offline makes "the same work" structural instead of something to verify
afterwards.

    export_ndt_frames.py --run tmp/demo-runs/<run> --out tmp/ndt-frames.bin

Format, little-endian, no dependencies on either side:

    magic  "NDTB"                     4 bytes
    version u32                       = 1
    num_map_points u32, then xyz f32 * 3 * n
    num_frames u32, then per frame:
        stamp f64
        pose  f64 * 7                 x y z qx qy qz qw
        num_points u32, then xyz f32 * 3 * n
"""
import argparse
import struct
import sys
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

REPO = Path(__file__).resolve().parents[3]
# The cloud NDT actually aligns: after the crop, voxel grid and random
# downsample. Aligning the raw scan would measure a different problem.
SCAN_TOPIC = "/localization/util/downsample/pointcloud"
# The prior handed to NDT for that scan, as the matcher itself published it.
POSE_TOPIC = "/localization/pose_estimator/initial_pose_with_covariance"
MAGIC, VERSION = b"NDTB", 1


def read_cloud(msg):
    off = {f.name: f.offset for f in msg.fields}
    n, step = msg.width * msg.height, msg.point_step
    arr = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, step)

    def col(o):
        return arr[:, o:o + 4].copy().view(np.float32).ravel()

    pts = np.stack([col(off["x"]), col(off["y"]), col(off["z"])], axis=1)
    return pts[np.isfinite(pts).all(axis=1)]


def read_pcd(path):
    with open(path, "rb") as f:
        hdr = {}
        while True:
            parts = f.readline().decode("ascii", "replace").split()
            if not parts:
                continue
            hdr[parts[0]] = parts[1:]
            if parts[0] == "DATA":
                break
        if hdr["DATA"][0] != "binary":
            raise SystemExit(f"{path}: only binary PCD supported")
        count = int(hdr["POINTS"][0])
        sizes = [int(s) for s in hdr["SIZE"]]
        step = sum(sizes)
        raw = f.read(count * step)
    arr = np.frombuffer(raw, dtype=np.uint8).reshape(count, step)
    offs, o = {}, 0
    for name, s in zip(hdr["FIELDS"], sizes):
        offs[name] = o
        o += s

    def col(x):
        return arr[:, x:x + 4].copy().view(np.float32).ravel()

    return np.stack([col(offs["x"]), col(offs["y"]), col(offs["z"])], axis=1)


def find_bag(run):
    run = Path(run)
    for name in ("bag", "diagnostics_bag"):
        if (run / name / "metadata.yaml").exists():
            return run / name
    if (run / "metadata.yaml").exists():
        return run
    raise SystemExit(f"no rosbag under {run}")


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--run", required=True, help="a completed demo run directory")
    p.add_argument("--map", default=str(REPO / "data/COSS-map-planning/pointcloud_map.pcd"))
    p.add_argument("--out", default=str(REPO / "tmp/ndt-frames.bin"))
    p.add_argument("--max-frames", type=int, default=0, help="0 = all")
    p.add_argument("--pair-tolerance", type=float, default=0.05,
                   help="[s] how close a prior must be to a scan to pair them")
    args = p.parse_args()

    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(find_bag(args.run)), storage_id="sqlite3"),
           rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    for topic in (SCAN_TOPIC, POSE_TOPIC):
        if topic not in types:
            raise SystemExit(f"{args.run} has no {topic}; record it or pick another run")

    scans, poses = [], []
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic == SCAN_TOPIC:
            m = deserialize_message(data, get_message(topic and types[topic]))
            ts = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            scans.append((ts, read_cloud(m)))
        elif topic == POSE_TOPIC:
            m = deserialize_message(data, get_message(types[topic]))
            ts = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            q, t_ = m.pose.pose.orientation, m.pose.pose.position
            poses.append((ts, (t_.x, t_.y, t_.z, q.x, q.y, q.z, q.w)))
    print(f"read {len(scans)} scans and {len(poses)} priors", file=sys.stderr)
    if not scans or not poses:
        raise SystemExit("nothing to pair")

    pose_ts = np.array([p_[0] for p_ in poses])
    frames = []
    for ts, pts in scans:
        i = int(np.argmin(np.abs(pose_ts - ts)))
        if abs(pose_ts[i] - ts) > args.pair_tolerance or len(pts) == 0:
            continue
        frames.append((ts, poses[i][1], pts))
        if args.max_frames and len(frames) >= args.max_frames:
            break
    if not frames:
        raise SystemExit("no scan paired with a prior; widen --pair-tolerance")

    mp = read_pcd(args.map)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, "wb") as f:
        f.write(MAGIC)
        f.write(struct.pack("<I", VERSION))
        f.write(struct.pack("<I", len(mp)))
        f.write(mp.astype(np.float32).tobytes())
        f.write(struct.pack("<I", len(frames)))
        for ts, pose, pts in frames:
            f.write(struct.pack("<d", ts))
            f.write(struct.pack("<7d", *pose))
            f.write(struct.pack("<I", len(pts)))
            f.write(pts.astype(np.float32).tobytes())

    size_mb = out.stat().st_size / 1e6
    pt_counts = [len(p_[2]) for p_ in frames]
    print(f"wrote {out} ({size_mb:.0f} MB): {len(mp)} map points, {len(frames)} frames, "
          f"{min(pt_counts)}-{max(pt_counts)} points per scan")


if __name__ == "__main__":
    main()
