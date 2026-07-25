#!/usr/bin/env python3
"""Accumulate a 2D occupancy grid directly from LiDAR scans at NDT-GT poses.

Phase 3b scan-accumulation experiment. Hypothesis under test: the PCD-sliced
occupancy grid used by the particle filter (MCL) lacks the structure the 2D
scan actually observes, so a grid *accumulated from the scans themselves*
(each transformed into map frame using the corresponding NDT ground-truth
pose) should better match what the scan-matcher sees at run time, and might
fix PF lock-on.

Encoding (map_server convention, negate: 0) -- NO unknown cells: every cell
is either occupied (0, >= min_hits scan points landed in it) or free (254,
everything else). This differs from `pcd_to_pgm.py`'s 3-value encoding
(which also has 205=unknown for columns with zero PCD points at all): here
we deliberately never emit unknown, because range_libc's raycaster only
consults "occupied vs not" -- marking swept-but-empty space as free (instead
of leaving it unknown) maximizes the region where ray casts are considered
valid.

Coordinate pipeline per scan:
  1. sensor frame (e.g. `velodyne_top`): z-band filter, keep (x, y).
  2. sensor -> base_link: static, from `/tf_static` in the bag. Composed by
     walking the static TF tree from the scan's frame_id up to `base_link`,
     multiplying/rotating the intermediate (translation, quaternion) hops
     together (see `compose_static_chain`). The composed 3D transform is
     then collapsed to a **planar** transform (tx, ty, yaw) --
     `quat_to_yaw` extracts only the rotation-about-Z component of the
     composed quaternion, and the composed translation's Z component is
     dropped. This is an approximation whenever the chain has non-trivial
     roll/pitch (it does here: `base_link -> sensor_kit_base_link` has a
     small ~1-2 deg tilt); it is applied to every point via a single 2D
     rotate+translate (`transform_xy`), not a true 3D transform of each
     point. Given the tilt magnitude here (~1-2 deg) the planar
     approximation's error is small relative to the grid resolution.
  3. base_link -> map: per-scan, using the time-nearest
     `/localization/kinematic_state` sample (planar pose x, y, yaw only;
     roll/pitch/z of that pose are likewise dropped for the same reason).
     Scans with no kinematic_state sample within `MAX_DT_S` are skipped.

Pure functions (`accumulate`, `transform_xy`, `quat_to_yaw`,
`compose_static_chain`) import nothing ROS and are unit-tested in
`test_scan_accumulate_grid.py` under a plain interpreter. The CLI
(`main`/bag-reading helpers) imports `rosbag2_py` etc. lazily, inside
functions, and requires a sourced ROS/Autoware environment to run.
"""
import argparse
import bisect
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent / "map"))
from pcd_to_pgm import write_map  # noqa: E402

OCCUPIED, FREE = 0, 254
MARGIN_M = 5.0
MAX_DT_S = 0.1


# --------------------------------------------------------------------------
# Pure functions (no ROS imports) -- unit tested directly.
# --------------------------------------------------------------------------

def quat_to_yaw(qx, qy, qz, qw):
    """Yaw (rotation about Z) from a quaternion, via atan2 -- no tf dependency.

    Ignores roll/pitch; exact only for quaternions that are pure yaw
    rotations, otherwise an approximation (see module docstring).
    """
    import math
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def _qmul(q1, q2):
    """Hamilton product q1 * q2, both (x, y, z, w)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return (x, y, z, w)


def _qrot(q, v):
    """Rotate 3-vector v by quaternion q = (x, y, z, w)."""
    qx, qy, qz, qw = q
    # v' = q * (v, 0) * q^-1, expanded via the standard formula.
    ux, uy, uz = qx, qy, qz
    vx, vy, vz = v
    # cross(u, v)
    cx = uy * vz - uz * vy
    cy = uz * vx - ux * vz
    cz = ux * vy - uy * vx
    # cross(u, cross(u,v))
    ccx = uy * cz - uz * cy
    ccy = uz * cx - ux * cz
    ccz = ux * cy - uy * cx
    rx = vx + 2.0 * qw * cx + 2.0 * ccx
    ry = vy + 2.0 * qw * cy + 2.0 * ccy
    rz = vz + 2.0 * qw * cz + 2.0 * ccz
    return (rx, ry, rz)


def compose_static_chain(chain):
    """Compose a list of (translation, quaternion) hops into one planar transform.

    `chain` is ordered innermost-first: chain[0] is the hop closest to the
    scan's sensor frame (child), chain[-1] is the hop closest to the root
    (e.g. base_link). Each hop (t_i, q_i) maps a point from its child frame
    into its parent frame: p_parent = R(q_i) @ p_child + t_i.

    Returns (tx, ty, yaw): the composed transform collapsed to 2D (see
    module docstring for why this is an approximation when hops have
    roll/pitch).
    """
    if not chain:
        raise ValueError("compose_static_chain: chain must be non-empty")
    t_acc = (0.0, 0.0, 0.0)
    q_acc = (0.0, 0.0, 0.0, 1.0)
    for t_i, q_i in chain:
        rotated = _qrot(q_i, t_acc)
        t_acc = (rotated[0] + t_i[0], rotated[1] + t_i[1], rotated[2] + t_i[2])
        q_acc = _qmul(q_i, q_acc)
    yaw = quat_to_yaw(*q_acc)
    return t_acc[0], t_acc[1], yaw


def transform_xy(points_xy, tx, ty, yaw):
    """Rotate+translate an (N, 2) array of points by a planar transform."""
    import math
    points_xy = np.asarray(points_xy, dtype=np.float64)
    c, s = math.cos(yaw), math.sin(yaw)
    rot = np.array([[c, -s], [s, c]])
    return points_xy @ rot.T + np.array([tx, ty])


def accumulate(points_map_xy, resolution, min_hits):
    """Bin scan points (already in map frame) into an occupancy grid.

    points_map_xy: list of (N_i, 2) arrays, one per scan, already
      transformed into map frame.
    resolution: grid cell size in meters.
    min_hits: minimum number of accumulated points in a cell for it to be
      marked occupied.

    Returns (grid uint8 [rows, cols], origin (x_min, y_min)). Extent is
    auto-computed from the data plus a MARGIN_M margin on every side. No
    unknown cells: everything not occupied is free (see module docstring).

    Raises ValueError if there are no points at all (empty list, or every
    scan is an empty array).
    """
    non_empty = [np.asarray(p) for p in points_map_xy if np.asarray(p).size > 0]
    if not non_empty:
        raise ValueError("accumulate: no points provided (empty scan list)")

    all_pts = np.concatenate(non_empty, axis=0)
    x, y = all_pts[:, 0], all_pts[:, 1]

    x_min = float(x.min()) - MARGIN_M
    x_max = float(x.max()) + MARGIN_M
    y_min = float(y.min()) - MARGIN_M
    y_max = float(y.max()) + MARGIN_M

    cols = int(np.ceil((x_max - x_min) / resolution)) + 1
    rows = int(np.ceil((y_max - y_min) / resolution)) + 1

    ci = ((x - x_min) / resolution).astype(np.int64).clip(0, cols - 1)
    ri = ((y - y_min) / resolution).astype(np.int64).clip(0, rows - 1)
    flat = ri * cols + ci

    counts = np.bincount(flat, minlength=rows * cols)

    grid = np.full(rows * cols, FREE, np.uint8)
    grid[counts >= min_hits] = OCCUPIED
    return grid.reshape(rows, cols), (x_min, y_min)


# --------------------------------------------------------------------------
# CLI / bag-reading (ROS-dependent; imports deferred into functions).
# --------------------------------------------------------------------------

def _stamp_to_float(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def _read_tf_static(reader, type_map):
    """Return dict: child_frame_id -> (parent_frame_id, (tx,ty,tz), (qx,qy,qz,qw))."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    tf_type = get_message(type_map["/tf_static"])
    edges = {}
    r2 = rosbag2_py.SequentialReader()
    r2.open(reader[0], reader[1])
    while r2.has_next():
        topic, data, _t = r2.read_next()
        if topic != "/tf_static":
            continue
        msg = deserialize_message(data, tf_type)
        for tr in msg.transforms:
            t = tr.transform.translation
            q = tr.transform.rotation
            edges[tr.child_frame_id] = (
                tr.header.frame_id,
                (t.x, t.y, t.z),
                (q.x, q.y, q.z, q.w),
            )
    return edges


def _static_chain_to_root(edges, leaf, root="base_link", max_hops=20):
    """Walk `edges` (child -> (parent, t, q)) from `leaf` up to `root`.

    Returns a chain ordered innermost-first, suitable for
    compose_static_chain(). If leaf == root, returns an identity chain.
    """
    if leaf == root:
        return [((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))]
    chain = []
    frame = leaf
    for _ in range(max_hops):
        if frame not in edges:
            raise ValueError(
                f"_static_chain_to_root: no static transform chain from "
                f"'{leaf}' to '{root}' (stuck at '{frame}')"
            )
        parent, t, q = edges[frame]
        chain.append((t, q))
        frame = parent
        if frame == root:
            return chain
    raise ValueError(
        f"_static_chain_to_root: chain from '{leaf}' to '{root}' exceeded "
        f"{max_hops} hops (possible cycle)"
    )


def _parse_xyz(msg):
    """Parse a sensor_msgs/PointCloud2 into an (N, 3) float64 array of x,y,z."""
    offsets = {f.name: f.offset for f in msg.fields}
    for required in ("x", "y", "z"):
        if required not in offsets:
            raise ValueError(f"_parse_xyz: pointcloud missing field '{required}'")
    dtype = np.dtype({
        "names": ["x", "y", "z"],
        "formats": ["<f4", "<f4", "<f4"],
        "offsets": [offsets["x"], offsets["y"], offsets["z"]],
        "itemsize": msg.point_step,
    })
    n = msg.width * msg.height
    arr = np.frombuffer(msg.data, dtype=dtype, count=n)
    return np.column_stack(
        [arr["x"].astype(np.float64), arr["y"].astype(np.float64), arr["z"].astype(np.float64)]
    )


def _read_kinematic_state(reader, type_map):
    """Return list of (t, x, y, yaw) sorted by t, from /localization/kinematic_state."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    ks_type = get_message(type_map["/localization/kinematic_state"])
    out = []
    r2 = rosbag2_py.SequentialReader()
    r2.open(reader[0], reader[1])
    while r2.has_next():
        topic, data, _t = r2.read_next()
        if topic != "/localization/kinematic_state":
            continue
        msg = deserialize_message(data, ks_type)
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        out.append((_stamp_to_float(msg.header.stamp), p.x, p.y, quat_to_yaw(o.x, o.y, o.z, o.w)))
    out.sort(key=lambda e: e[0])
    return out


def _nearest_pose(ks_sorted, ks_times, t, max_dt):
    idx = bisect.bisect_left(ks_times, t)
    candidates = []
    if idx < len(ks_times):
        candidates.append(idx)
    if idx > 0:
        candidates.append(idx - 1)
    if not candidates:
        return None
    best = min(candidates, key=lambda i: abs(ks_times[i] - t))
    if abs(ks_times[best] - t) > max_dt:
        return None
    return ks_sorted[best]


def _run(bag, out_prefix, z_min, z_max, resolution, min_hits):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    storage_options = rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader_args = (storage_options, converter_options)

    probe = rosbag2_py.SequentialReader()
    probe.open(storage_options, converter_options)
    type_map = {t.name: t.type for t in probe.get_all_topics_and_types()}
    del probe

    pc_topic = "/sensing/lidar/top/pointcloud_raw_ex"
    if pc_topic not in type_map:
        raise ValueError(f"_run: topic '{pc_topic}' not found in bag {bag}")

    print("Reading /tf_static ...")
    edges = _read_tf_static(reader_args, type_map)

    print("Reading /localization/kinematic_state ...")
    ks_sorted = _read_kinematic_state(reader_args, type_map)
    ks_times = [e[0] for e in ks_sorted]
    if not ks_sorted:
        raise ValueError("_run: no /localization/kinematic_state messages in bag")

    print(f"Reading {pc_topic} and accumulating scans ...")
    pc_type = get_message(type_map[pc_topic])
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    scans_map_xy = []
    n_scans_total = 0
    n_scans_used = 0
    n_scans_skipped_dt = 0
    sensor_to_base = {}  # cache per frame_id

    while reader.has_next():
        topic, data, _t = reader.read_next()
        if topic != pc_topic:
            continue
        n_scans_total += 1
        msg = deserialize_message(data, pc_type)
        frame_id = msg.header.frame_id

        if frame_id not in sensor_to_base:
            chain = _static_chain_to_root(edges, frame_id, root="base_link")
            sensor_to_base[frame_id] = compose_static_chain(chain)
        s_tx, s_ty, s_yaw = sensor_to_base[frame_id]

        xyz = _parse_xyz(msg)
        band = (xyz[:, 2] >= z_min) & (xyz[:, 2] <= z_max)
        if not np.any(band):
            continue
        sensor_xy = xyz[band, :2]

        t_scan = _stamp_to_float(msg.header.stamp)
        pose = _nearest_pose(ks_sorted, ks_times, t_scan, MAX_DT_S)
        if pose is None:
            n_scans_skipped_dt += 1
            continue
        _t_pose, map_x, map_y, map_yaw = pose

        base_xy = transform_xy(sensor_xy, s_tx, s_ty, s_yaw)
        map_xy = transform_xy(base_xy, map_x, map_y, map_yaw)
        scans_map_xy.append(map_xy)
        n_scans_used += 1

    print(
        f"Scans: {n_scans_total} total, {n_scans_used} used, "
        f"{n_scans_skipped_dt} skipped (no kinematic_state within {MAX_DT_S}s)"
    )
    if not scans_map_xy:
        raise ValueError("_run: no scans survived filtering -- nothing to accumulate")

    total_pts = sum(a.shape[0] for a in scans_map_xy)
    print(f"Accumulating {total_pts} band-filtered points from {n_scans_used} scans ...")
    grid, origin = accumulate(scans_map_xy, resolution, min_hits)

    occ = int((grid == OCCUPIED).sum())
    free = int((grid == FREE).sum())
    print(f"grid {grid.shape[1]}x{grid.shape[0]} cells, {occ} occupied, {free} free")

    pgm, yml = write_map(grid, origin, resolution, Path(out_prefix))
    print(f"wrote {pgm} and {yml}")
    return grid, origin


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("bag", type=Path)
    ap.add_argument("out_prefix", type=Path)
    ap.add_argument("--z-min", type=float, default=-0.15)
    ap.add_argument("--z-max", type=float, default=0.15)
    ap.add_argument("--resolution", type=float, default=0.1)
    ap.add_argument("--min-hits", type=int, default=3)
    args = ap.parse_args()

    _run(args.bag, args.out_prefix, args.z_min, args.z_max, args.resolution, args.min_hits)


if __name__ == "__main__":
    main()
