#!/usr/bin/env python3
"""Compare NDT matchers on speed, cost and whether they did the same work.

Reads the run directories a benchmark produced and reports three things:

  speed        exe_time per scan, and whether the matcher kept up with the
               10 Hz scan rate at all -- a matcher that drops scans looks fast
               per scan while localising worse
  cost         CPU and GPU utilisation, from the samples play_launch already
               takes per node
  equivalence  iterations and score, because a speed comparison is meaningless
               if the configurations did not converge to the same place

    ndt_benchmark_report.py <runs.tsv>        # "config<TAB>run_dir" per line
    ndt_benchmark_report.py gpu=<dir> cpu=<dir> ...
"""
import csv
import math
import statistics as st
import sys
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

EXE = "/localization/pose_estimator/exe_time_ms"
ITER = "/localization/pose_estimator/iteration_num"
NVTL = "/localization/pose_estimator/nearest_voxel_transformation_likelihood"
POSE = "/localization/pose_estimator/pose"
SCAN_HZ = 10.0        # the VLP32C runs at 10 Hz, so this is the time budget
SCAN_BUDGET_MS = 1000.0 / SCAN_HZ


def find_bag(run):
    run = Path(run)
    for name in ("bag", "diagnostics_bag"):
        if (run / name / "metadata.yaml").exists():
            return run / name
    if (run / "metadata.yaml").exists():
        return run
    raise SystemExit(f"no rosbag under {run}")


def read_run(run_dir):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(find_bag(run_dir)), storage_id="sqlite3"),
           rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    out = {EXE: [], ITER: [], NVTL: [], POSE: 0}
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic == POSE:
            out[POSE] += 1
        elif topic in (EXE, ITER, NVTL):
            out[topic].append(deserialize_message(data, get_message(types[topic])).data)
    return out


def read_metrics(run_dir):
    """cpu_percent / gpu_utilization sampled by play_launch for the matcher."""
    path = Path(run_dir) / "matcher_metrics.csv"
    cpu, gpu, rss = [], [], []
    if not path.exists():
        return None
    with open(path) as f:
        for row in csv.DictReader(f):
            try:
                c = float(row.get("cpu_percent") or "nan")
                g = float(row.get("gpu_utilization_percent") or "nan")
                m = float(row.get("rss_bytes") or "nan")
            except ValueError:
                continue
            if not math.isnan(c):
                cpu.append(c)
            if not math.isnan(g):
                gpu.append(g)
            if not math.isnan(m):
                rss.append(m)
    # Samples before the matcher starts working are not interesting; keep the
    # busiest half, which is the replay itself rather than idle startup.
    def busy(v):
        if not v:
            return []
        v = sorted(v)
        return v[len(v) // 2:]
    return {"cpu": busy(cpu), "gpu": busy(gpu), "rss": rss}


def pct(v, p):
    if not v:
        return float("nan")
    v = sorted(v)
    return v[min(len(v) - 1, int(len(v) * p))]


def main():
    args = sys.argv[1:]
    if not args:
        raise SystemExit(__doc__)
    runs = []
    if len(args) == 1 and args[0].endswith(".tsv"):
        for line in Path(args[0]).read_text().splitlines():
            if line.strip():
                cfg, d = line.split("\t")
                runs.append((cfg, d))
    else:
        for a in args:
            cfg, d = a.split("=", 1)
            runs.append((cfg, d))

    rows = []
    for cfg, d in runs:
        data = read_run(d)
        exe, iters, nvtl = data[EXE], data[ITER], data[NVTL]
        if not exe:
            print(f"{cfg}: no exe_time in {d}, skipping", file=sys.stderr)
            continue
        m = read_metrics(d)
        rows.append({
            "cfg": cfg, "dir": d, "n": len(exe),
            "mean": st.mean(exe), "p50": pct(exe, 0.5), "p95": pct(exe, 0.95),
            "max": max(exe),
            "over_budget": 100.0 * sum(1 for x in exe if x > SCAN_BUDGET_MS) / len(exe),
            "iters": st.mean(iters) if iters else float("nan"),
            "nvtl": st.mean(nvtl) if nvtl else float("nan"),
            "poses": data[POSE],
            "cpu": st.mean(m["cpu"]) if m and m["cpu"] else float("nan"),
            "gpu": st.mean(m["gpu"]) if m and m["gpu"] else float("nan"),
            "rss": max(m["rss"]) / 1e6 if m and m["rss"] else float("nan"),
        })

    if not rows:
        raise SystemExit("nothing to report")

    print(f"NDT matcher benchmark -- {rows[0]['n']} alignments or so per run, "
          f"scan budget {SCAN_BUDGET_MS:.0f} ms at {SCAN_HZ:.0f} Hz\n")
    hdr = (f"{'config':>9} {'aligns':>7} {'mean ms':>8} {'p50':>7} {'p95':>7} "
           f"{'max':>7} {'>budget':>8} {'cpu %':>7} {'gpu %':>6} {'rss MB':>7} "
           f"{'iters':>6} {'NVTL':>6} {'poses':>6}")
    print(hdr)
    print("-" * len(hdr))
    for r in rows:
        print(f"{r['cfg']:>9} {r['n']:7d} {r['mean']:8.2f} {r['p50']:7.2f} "
              f"{r['p95']:7.2f} {r['max']:7.1f} {r['over_budget']:7.1f}% "
              f"{r['cpu']:7.1f} {r['gpu']:6.1f} {r['rss']:7.0f} "
              f"{r['iters']:6.2f} {r['nvtl']:6.3f} {r['poses']:6d}")

    by = {r["cfg"]: r for r in rows}
    print()
    if "gpu" in by and "cpu" in by:
        g, c = by["gpu"], by["cpu"]
        print(f"GPU vs the same algorithm on CPU: {c['mean'] / g['mean']:.2f}x on mean "
              f"exe_time ({c['mean']:.2f} -> {g['mean']:.2f} ms), "
              f"{c['p95'] / g['p95']:.2f}x on p95")
        di, dn = abs(g["iters"] - c["iters"]), abs(g["nvtl"] - c["nvtl"])
        # Score decides equivalence, not iteration count. The two arms use
        # different convergence tests on purpose -- the GPU compares the applied
        # step as Autoware does, the CPU the raw Newton step before its line
        # search -- so they reach the same answer in different numbers of
        # iterations. That is a documented difference, not a reason to distrust
        # the timings; a score or NVTL gap is.
        if dn < 0.05:
            verdict = "same result"
        else:
            verdict = "DIFFERENT result -- the speed numbers are not comparable"
        print(f"  equivalence: NVTL differs by {dn:.3f} -> {verdict}")
        print(f"  (iterations differ by {di:.2f}; the arms use different "
              f"convergence tests, so this is expected)")
    if "cpu" in by and "autoware" in by:
        c, a = by["cpu"], by["autoware"]
        print(f"cuda_ndt on CPU vs Autoware OpenMP: {a['mean'] / c['mean']:.2f}x "
              f"({a['mean']:.2f} -> {c['mean']:.2f} ms)")
    if "gpu" in by and "autoware" in by:
        g, a = by["gpu"], by["autoware"]
        print(f"GPU vs Autoware OpenMP: {a['mean'] / g['mean']:.2f}x "
              f"({a['mean']:.2f} -> {g['mean']:.2f} ms)")
    print("\nNote: exe_time counts one alignment. A matcher that misses the "
          f"{SCAN_BUDGET_MS:.0f} ms budget\ndrops scans, so compare the alignment "
          "count and pose count too, not just the mean.")


if __name__ == "__main__":
    main()
