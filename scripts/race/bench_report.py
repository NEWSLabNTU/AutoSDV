#!/usr/bin/env python3
"""Markdown table from tmp/race-bench/<label>/ directories (written by bench.sh).

One row per label: host, power mode, clocks pinned (min == max in sysfs: cpu, gpu, both, or
"no"), 1-min load at start/end, L1 p50/p95/max and overruns,
L2 p95/max, acados p95/max, GPU utilisation p95 (Jetson runs). Times in ms.
"CasADi record wall p95" is acados_tracker's `record` step (under RP_DET, unpinned): context for
the acados column, not the live L2 number (that is "L2 p95").

    scripts/race/bench_report.py tmp/race-bench/x86-idle tmp/race-bench/orin-idle -o report.md
"""
import argparse
import csv
import json
import os
import sys

DASH = "—"
HEADER = ["Label", "Host", "nvpmodel", "Clocks pinned", "Load 1m start", "Load 1m end",
          "L1 p50 ms", "L1 p95 ms", "L1 max ms", "L1 overruns",
          "L2 p95 ms", "L2 max ms", "acados p95 ms", "acados max ms",
          "CasADi record wall p95 ms", "GPU util p95 %"]


def _load(path):
    try:
        with open(path) as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return None


def _f(x, digits=1, scale=1.0):
    return DASH if x is None else f"{x * scale:.{digits}f}"


def _percentile(xs, q):
    xs = sorted(xs)
    if not xs:
        return None
    k = (len(xs) - 1) * q
    lo = int(k)
    hi = min(lo + 1, len(xs) - 1)
    return xs[lo] + (xs[hi] - xs[lo]) * (k - lo)


def _gpu_p95(path):
    try:
        with open(path, newline="") as fh:
            vals = [float(r["gpu_load_percent"]) for r in csv.DictReader(fh)
                    if r.get("gpu_load_percent") not in (None, "")]
    except (OSError, ValueError, KeyError):
        return None
    return _percentile(vals, 0.95)


def _power_mode(nv):
    if not nv:
        return DASH
    for line in nv.splitlines():
        if "Mode:" in line:
            return line.split("Mode:", 1)[1].strip() or DASH
    return nv.splitlines()[0]


def _pinned(clocks):
    if not clocks:
        return DASH
    cpu, gpu = clocks.get("cpu_pinned"), clocks.get("gpu_pinned")
    if cpu is None and gpu is None:
        return DASH
    got = [n for n, v in (("cpu", cpu), ("gpu", gpu)) if v]
    return "+".join(got) if got else "no"


def row(d):
    label = os.path.basename(os.path.normpath(d))
    live = _load(os.path.join(d, label + ".json")) or {}
    host = live.get("host") or {}
    allv = live.get("all") or {}
    ac = _load(os.path.join(d, f"acados_{label}.json")) or {}
    load = _load(os.path.join(d, "load.json")) or {}
    t_ac, t_cs = ac.get("time_tot") or {}, ac.get("casadi_solve_wall") or {}
    ms = 1000.0
    gpu = _gpu_p95(os.path.join(d, "gpu.csv"))
    return [label, host.get("machine") or DASH, _power_mode(host.get("nvpmodel")),
            _pinned(host.get("clocks")),
            _f(load.get("start"), 2), _f(load.get("end"), 2),
            _f(allv.get("l1_p50"), 1, ms), _f(allv.get("l1_p95"), 1, ms),
            _f(allv.get("l1_max"), 1, ms),
            DASH if allv.get("l1_overruns") is None else str(allv["l1_overruns"]),
            _f(allv.get("l2_p95"), 2, ms), _f(allv.get("l2_max"), 2, ms),
            _f(t_ac.get("p95"), 3, ms), _f(t_ac.get("max"), 3, ms),
            _f(t_cs.get("p95"), 2, ms), _f(gpu, 1)]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("dirs", nargs="+")
    ap.add_argument("-o", "--output", required=True)
    a = ap.parse_args(argv)
    lines = ["| " + " | ".join(HEADER) + " |", "|" + "|".join(["---"] * len(HEADER)) + "|"]
    lines += ["| " + " | ".join(row(d)) + " |" for d in a.dirs]
    with open(a.output, "w") as fh:
        fh.write("\n".join(lines) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
