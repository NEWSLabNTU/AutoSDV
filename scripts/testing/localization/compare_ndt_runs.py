#!/usr/bin/env python3
"""Compare replay runs: position scatter, yaw jitter, prediction error, score.

    compare_ndt_runs.py a=<run_dir> b=<run_dir> ...     # table
    compare_ndt_runs.py --row a=<run_dir>               # one TSV row

Reading a run means scanning a multi-gigabyte rosbag, so several runs are worth
extracting concurrently. --row emits a single machine-readable line, which is
what `just demo compare` fans out over GNU parallel and then formats.
"""
import math
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

N = "/localization/pose_estimator/pose"
V = "/vehicle/status/velocity_status"
I = "/localization/pose_estimator/initial_to_result_distance"
NV = "/localization/pose_estimator/nearest_voxel_transformation_likelihood"
IT = "/localization/pose_estimator/iteration_num"
EX = "/localization/pose_estimator/exe_time_ms"
CLOUD = "/localization/util/downsample/pointcloud"


def yaw_of(o):
    return math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z))


def load(path):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3"),
           rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    out = {k: [] for k in (N, V, I, NV, IT, EX, CLOUD)}
    while r.has_next():
        t, data, _ = r.read_next()
        if t not in out:
            continue
        m = deserialize_message(data, get_message(types[t]))
        if t == N:
            ts = m.header.stamp
            out[t].append((ts.sec + ts.nanosec * 1e-9, m.pose.position.x,
                           m.pose.position.y, yaw_of(m.pose.orientation)))
        elif t == V:
            ts = m.header.stamp
            out[t].append((ts.sec + ts.nanosec * 1e-9, m.longitudinal_velocity))
        elif t == CLOUD:
            out[t].append(m.width * m.height)
        else:
            out[t].append((m.stamp.sec + m.stamp.nanosec * 1e-9, m.data))
    return out


def stats(label, path):
    d = load(path)
    v = d[V]
    tm = next(t for t, s in v if abs(s) > 0.2)
    te = max(t for t, s in v if abs(s) > 0.2)
    p = [x for x in d[N] if tm <= x[0] <= te]

    steps, yaws = [], []
    for i in range(1, len(p)):
        dt = p[i][0] - p[i - 1][0]
        if not 0 < dt < 0.5:
            continue
        steps.append(math.hypot(p[i][1] - p[i - 1][1], p[i][2] - p[i - 1][2]))
        a = (p[i][3] - p[i - 1][3] + math.pi) % (2 * math.pi) - math.pi
        yaws.append(abs(math.degrees(a)))
    sm = p[::10]
    smooth = sum(math.hypot(sm[i + 1][1] - sm[i][1], sm[i + 1][2] - sm[i][2])
                 for i in range(len(sm) - 1))
    raw = sum(steps)
    mv = [x for t, x in d[I] if t >= tm]
    nvm = [x for t, x in d[NV] if t >= tm]
    itm = [x for t, x in d[IT] if t >= tm]
    exm = [x for t, x in d[EX] if t >= tm]
    pts = d[CLOUD]
    return {
        "label": label,
        "poses": len(p),
        "pts": int(np.median(pts)) if pts else 0,
        "scatter": (raw - smooth) / max(1, len(p)),
        "yaw_med": np.median(yaws) if yaws else float("nan"),
        "yaw_p95": np.percentile(yaws, 95) if yaws else float("nan"),
        "i2r": float(np.mean(mv)) if mv else float("nan"),
        "i2r95": float(np.percentile(mv, 95)) if mv else float("nan"),
        "nvtl": float(np.mean(nvm)) if nvm else float("nan"),
        "iters": float(np.mean(itm)) if itm else float("nan"),
        "exe": float(np.mean(exm)) if exm else float("nan"),
        "dist": smooth,
    }


FIELDS = ("label", "pts", "poses", "scatter", "yaw_med", "yaw_p95",
          "i2r", "i2r95", "nvtl", "iters", "exe", "dist")


def format_row(r):
    return (f"{r['label']:>10} {r['pts']:6d} {r['poses']:6d} {r['scatter']:8.3f} "
            f"{r['yaw_med']:7.2f}° {r['yaw_p95']:7.2f}° {r['i2r']:9.3f} "
            f"{r['i2r95']:8.3f} {r['nvtl']:6.3f} {r['iters']:6.1f} "
            f"{r['exe']:7.1f} {r['dist']:7.1f}")


def header():
    return (f"{'run':>10} {'pts':>6} {'poses':>6} {'scatter':>8} {'yaw med':>8} "
            f"{'yaw p95':>8} {'i2r mean':>9} {'i2r p95':>8} {'NVTL':>6} "
            f"{'iters':>6} {'exe ms':>7} {'dist m':>7}")


def main():
    args = sys.argv[1:]
    if args and args[0] == "--row":
        # One run, one line: the unit of work for a parallel fan-out.
        r = stats(*args[1].split("=", 1))
        print("\t".join(str(r[f]) for f in FIELDS))
        return
    if args and args[0] == "--format":
        # Reassemble rows produced by --row into the usual table.
        print(header())
        print("-" * len(header()))
        for line in sorted(l for l in sys.stdin.read().splitlines() if l.strip()):
            v = line.split("\t")
            r = dict(zip(FIELDS, v))
            for f in FIELDS[1:]:
                r[f] = float(r[f])
            r["pts"], r["poses"] = int(r["pts"]), int(r["poses"])
            print(format_row(r))
        return
    rows = [stats(*a.split("=", 1)) for a in args]
    print(header())
    print("-" * len(header()))
    for r in rows:
        print(format_row(r))


if __name__ == "__main__":
    main()
