#!/usr/bin/env python3
"""Shared readers for the map-quality tools.

Both tools place a live scan into the map using the pose NDT produced for it,
then ask a question about the result. Everything they need in common lives here:
binary PCD reading, PointCloud2 unpacking, pose lookup, and the 2 m XY grid the
map footprint is expressed on.
"""
import struct
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

REPO = Path(__file__).resolve().parents[4]
DEFAULT_BAG = REPO / "data/rosbags/outdoor_20251226_153115"
DEFAULT_MAP = REPO / "data/COSS-map-planning/pointcloud_map.pcd"
RAW_CLOUD_TOPIC = "/sensing/lidar/velodyne_points"
POSE_TOPIC = "/localization/pose_estimator/pose"
CELL = 2.0


def open_reader(uri):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(uri), storage_id="sqlite3"),
           rosbag2_py.ConverterOptions("", ""))
    return r, {t.name: t.type for t in r.get_all_topics_and_types()}


def find_bag(run_dir):
    """A demo run directory, a diagnostics run directory, or a bag itself."""
    run = Path(run_dir)
    if (run / "metadata.yaml").exists():
        return run
    for name in ("bag", "diagnostics_bag"):
        if (run / name / "metadata.yaml").exists():
            return run / name
    raise SystemExit(f"no rosbag under {run} (looked for bag/, diagnostics_bag/)")


def read_cloud(msg):
    """PointCloud2 -> (n, 3) float32, without a point_cloud2 dependency."""
    off = {f.name: f.offset for f in msg.fields}
    n, step = msg.width * msg.height, msg.point_step
    arr = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, step)

    def col(o):
        return arr[:, o:o + 4].copy().view(np.float32).ravel()

    return np.stack([col(off["x"]), col(off["y"]), col(off["z"])], axis=1)


def read_pcd(path):
    """Binary PCD -> (n, 3) float32. Ascii and compressed PCDs are not handled."""
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
            raise SystemExit(f"{path}: only binary PCD is supported, got {hdr['DATA'][0]}")
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


def quat_to_R(qx, qy, qz, qw):
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])


def read_poses(run_dir):
    """[(t, x, y, z, qx, qy, qz, qw)] from a run's recorded NDT poses."""
    r, types = open_reader(find_bag(run_dir))
    out = []
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic != POSE_TOPIC:
            continue
        m = deserialize_message(data, get_message(types[topic]))
        p, o = m.pose.position, m.pose.orientation
        out.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                    p.x, p.y, p.z, o.x, o.y, o.z, o.w))
    if not out:
        raise SystemExit(f"{run_dir} has no {POSE_TOPIC}; did NDT publish?")
    return out


def read_scans_near(bag, times):
    """The raw scan closest to each requested time, as {index: (n,3) array}."""
    r, types = open_reader(bag)
    best = {i: (float("inf"), None) for i in range(len(times))}
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic != RAW_CLOUD_TOPIC:
            continue
        m = deserialize_message(data, get_message(types[topic]))
        ts = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        cloud = None
        for i, t in enumerate(times):
            dt = abs(ts - t)
            if dt < best[i][0]:
                cloud = read_cloud(m) if cloud is None else cloud
                best[i] = (dt, cloud)
    missing = [i for i, (_, c) in best.items() if c is None]
    if missing:
        raise SystemExit(f"no {RAW_CLOUD_TOPIC} near requested times in {bag}")
    return {i: c for i, (_, c) in best.items()}


def place(scan, pose, lidar_z_offset=0.31):
    """Scan in the sensor frame -> map frame, using an NDT pose for base_link."""
    _, px, py, pz, qx, qy, qz, qw = pose
    world = scan @ quat_to_R(qx, qy, qz, qw).T + np.array([px, py, pz + lidar_z_offset])
    rng = np.linalg.norm(scan[:, :2], axis=1)
    ok = np.isfinite(rng) & np.isfinite(world).all(axis=1)
    return world[ok], rng[ok]


def cell_key(x, y, cell=CELL):
    return (np.floor(x / cell).astype(np.int64) << 20) + np.floor(y / cell).astype(np.int64)


def motion_window(run_dir, speed_topic="/vehicle/status/velocity_status", thresh=0.2):
    """(t_start, t_end) of the driving segment, by wheel speed."""
    r, types = open_reader(find_bag(run_dir))
    ts = []
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic != speed_topic:
            continue
        m = deserialize_message(data, get_message(types[topic]))
        if abs(m.longitudinal_velocity) > thresh:
            ts.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
    return (min(ts), max(ts)) if ts else (None, None)
