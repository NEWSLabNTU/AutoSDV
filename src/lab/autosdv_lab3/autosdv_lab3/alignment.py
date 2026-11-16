"""Lab 3 grading, item A2: how well the live LiDAR scans lie on the team's map.

Everything comes from the run bag:

- the map: /map/pointcloud_map (latched; the grader records it);
- the scans: the raw LiDAR topic, in the sensor's frame;
- the sensor's mount: /tf_static, base_link -> the scan's frame;
- the pose: /localization/kinematic_state, Autoware's own estimate of base_link in map.

One scan every SCAN_PERIOD s inside the lap is placed in the map at the estimated pose,
each point at its own time within the sweep (the per-point time_stamp), and each point's
distance to the nearest map point is taken. Points farther than OUTLIER from every map
point are people, doors and things the map does not hold; they are left out of the
distance and reported as a share. A scan's score is the median distance of its kept
points; the item's is the median over the scans.

This judges the pose against the team's own map, the map NDT matched against: a sharp
map and a consistent pose score well. It does not see a map that is bent as a whole.
"""

import math
from typing import Dict, Optional, Tuple

import numpy as np

MAP_TOPIC = '/map/pointcloud_map'
SCAN_PERIOD = 0.5          # s between scored scans
OUTLIER = 1.0              # m: farther than this from the map, a point is not scored
MAX_RANGE = 15.0           # m: farther points are dropped (the room is smaller)
MIN_RANGE = 0.3            # m: the car itself


def _matrix(tr) -> np.ndarray:
    q, p = tr.rotation, tr.translation
    return pose_matrix((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))


def pose_matrix(xyz, q) -> np.ndarray:
    x, y, z, w = q
    m = np.eye(4)
    m[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    m[:3, 3] = xyz
    return m


def chain(edges, root: str, leaf: str) -> Optional[np.ndarray]:
    """T(root <- leaf) through the static tree."""
    children: Dict[str, list] = {}
    for parent, child, m in edges:
        children.setdefault(parent.lstrip('/'), []).append((child.lstrip('/'), m))
    stack, seen = [(root, np.eye(4))], set()
    while stack:
        frame, m = stack.pop()
        if frame == leaf.lstrip('/'):
            return m
        if frame in seen:
            continue
        seen.add(frame)
        stack.extend((c, m @ cm) for c, cm in children.get(frame, []))
    return None


class Poses:
    """base_link in map against the stack's clock (header stamps), for interpolation."""

    def __init__(self, rows: np.ndarray):
        # rows: stamp x y z yaw (yaw unwrapped); roll and pitch are taken as zero: the
        # floor is the map's XY plane and the car is flat on it
        self.r = rows[np.argsort(rows[:, 0])] if len(rows) else rows

    def at(self, t: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        r = self.r
        return (np.interp(t, r[:, 0], r[:, 1]), np.interp(t, r[:, 0], r[:, 2]),
                np.interp(t, r[:, 0], r[:, 3]), np.interp(t, r[:, 0], r[:, 4]))

    def covers(self, t0: float, t1: float) -> bool:
        return len(self.r) > 1 and self.r[0, 0] <= t0 and t1 <= self.r[-1, 0]


def place(points: np.ndarray, offsets: np.ndarray, stamp: float, T_base_sensor: np.ndarray,
          poses: Poses) -> np.ndarray:
    """Sensor-frame points to the map, each at its own time stamp + offset."""
    p = points @ T_base_sensor[:3, :3].T + T_base_sensor[:3, 3]
    x, y, z, yaw = poses.at(stamp + offsets)
    c, s = np.cos(yaw), np.sin(yaw)
    return np.stack([x + c * p[:, 0] - s * p[:, 1], y + s * p[:, 0] + c * p[:, 1], z + p[:, 2]], 1)


def score_scans(map_xyz: np.ndarray, scans, poses: Poses, T_base_sensor: np.ndarray) -> dict:
    """scans: iterable of (header stamp, Nx3 sensor-frame points, per-point offsets [s])."""
    from scipy.spatial import cKDTree
    tree = cKDTree(map_xyz)
    per_scan, kept_share = [], []
    for stamp, pts, off in scans:
        if not len(pts):
            continue
        rng = np.linalg.norm(pts, axis=1)
        sel = (rng > MIN_RANGE) & (rng < MAX_RANGE) & np.isfinite(rng)
        pts, off = pts[sel], off[sel]
        if not len(pts) or not poses.covers(stamp + float(off.min(initial=0.0)),
                                             stamp + float(off.max(initial=0.0))):
            continue
        d, _ = tree.query(place(pts, off, stamp, T_base_sensor, poses),
                          distance_upper_bound=OUTLIER)
        inside = np.isfinite(d)
        kept_share.append(float(inside.mean()))
        if inside.any():
            per_scan.append(float(np.median(d[inside])))
    if not per_scan:
        return {'scans': 0}
    return {'scans': len(per_scan),
            'median_m': round(float(np.median(per_scan)), 4),
            'p90_m': round(float(np.percentile(per_scan, 90)), 4),
            'points_on_map': round(float(np.median(kept_share)), 3)}


# ------------------------------------------------------------------ from the bag

def cloud_xyz(msg) -> np.ndarray:
    from sensor_msgs_py import point_cloud2
    pts = point_cloud2.read_points(msg, field_names=['x', 'y', 'z'], skip_nans=True)
    return np.stack([pts['x'], pts['y'], pts['z']], axis=1).astype(float)


def cloud_xyz_t(msg) -> Tuple[np.ndarray, np.ndarray]:
    from sensor_msgs_py import point_cloud2
    names = [f.name for f in msg.fields]
    tf = 'time_stamp' if 'time_stamp' in names else ('time' if 'time' in names else None)
    pts = point_cloud2.read_points(msg, field_names=['x', 'y', 'z'] + ([tf] if tf else []),
                                   skip_nans=True)
    xyz = np.stack([pts['x'], pts['y'], pts['z']], axis=1).astype(float)
    if tf == 'time_stamp':        # Seyond / Nebula: uint32 ns after the header stamp
        off = np.asarray(pts[tf], float) * 1e-9
    elif tf == 'time':
        off = np.asarray(pts[tf], float)
    else:
        off = np.zeros(len(xyz))
    return xyz, off


def read_pcd_xyz(path: str) -> np.ndarray:
    """x y z of a PCD file (ascii or binary; float fields), for a bag without the map."""
    with open(path, 'rb') as f:
        header = {}
        while True:
            line = f.readline().decode('ascii', 'replace').strip()
            if not line or line.startswith('#'):
                continue
            k, _, v = line.partition(' ')
            header[k.upper()] = v.split()
            if k.upper() == 'DATA':
                break
        fields, sizes = header['FIELDS'], [int(x) for x in header['SIZE']]
        types, counts = header['TYPE'], [int(x) for x in header.get('COUNT', ['1'] * len(fields))]
        n = int(header['POINTS'][0])
        kind = header['DATA'][0]
        if kind == 'ascii':
            a = np.loadtxt(f, max_rows=n, ndmin=2)
            idx = np.cumsum([0] + counts[:-1])
            return a[:, [idx[fields.index(c)] for c in 'xyz']].astype(float)
        if kind != 'binary':
            raise ValueError(f'{path}: DATA {kind} is not supported (convert it to binary)')
        code = {('F', 4): 'f4', ('F', 8): 'f8', ('U', 1): 'u1', ('U', 2): 'u2', ('U', 4): 'u4',
                ('I', 1): 'i1', ('I', 2): 'i2', ('I', 4): 'i4'}
        dt = np.dtype([(name, '<' + code[(t, sz)], (c,)) if c > 1 else (name, '<' + code[(t, sz)])
                       for name, t, sz, c in zip(fields, types, sizes, counts)])
        a = np.frombuffer(f.read(dt.itemsize * n), dtype=dt, count=n)
        return np.stack([a['x'], a['y'], a['z']], axis=1).astype(float)


def read_alignment(path: str, lidar_topic: str, t0: float, t1: float, odom_topic: str,
                   map_pcd: Optional[str] = None) -> dict:
    """A2 from the bag at `path`, over the bag-time window [t0, t1]. The map comes from the
    bag's MAP_TOPIC, or from `map_pcd` when the bag was recorded without it."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
    need = [lidar_topic, '/tf_static', odom_topic] + ([] if map_pcd else [MAP_TOPIC])
    missing = [t for t in need if t not in topics]
    if missing:
        return {'scans': 0, 'why': f'the bag lacks {missing}'}
    reader.set_filter(rosbag2_py.StorageFilter(topics=need))
    types = {t: get_message(topics[t]) for t in need}
    map_xyz = read_pcd_xyz(map_pcd) if map_pcd else None
    statics, rows, scans, frame = [], [], [], None
    last = -1e18
    while reader.has_next():
        topic, raw, stamp = reader.read_next()
        t = stamp * 1e-9
        if topic == lidar_topic:
            if t < t0 or t > t1 or t - last < SCAN_PERIOD:
                continue
            last = t
        m = deserialize_message(raw, types[topic])
        if topic == MAP_TOPIC:
            map_xyz = cloud_xyz(m)
        elif topic == '/tf_static':
            statics += [(tr.header.frame_id, tr.child_frame_id, _matrix(tr.transform))
                        for tr in m.transforms]
        elif topic == odom_topic:
            p, q = m.pose.pose.position, m.pose.pose.orientation
            hs = m.header.stamp
            rows.append((hs.sec + hs.nanosec * 1e-9, p.x, p.y, p.z,
                         math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))))
        else:
            xyz, off = cloud_xyz_t(m)
            hs = m.header.stamp
            scans.append((hs.sec + hs.nanosec * 1e-9, xyz, off))
            frame = m.header.frame_id
    if map_xyz is None or not len(map_xyz):
        return {'scans': 0, 'why': f'no {MAP_TOPIC} message in the bag (or pass the map: --map PCD)'}
    T = chain(statics, 'base_link', frame) if frame else None
    if T is None:
        return {'scans': 0, 'why': f'no static transform base_link -> {frame}'}
    r = np.array(rows, float).reshape(-1, 5)
    if len(r):
        r = r[np.argsort(r[:, 0])]
        r[:, 4] = np.unwrap(r[:, 4])
    return score_scans(map_xyz, scans, Poses(r), T)
