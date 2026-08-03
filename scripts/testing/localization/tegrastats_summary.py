#!/usr/bin/env python3
"""Summarise a tegrastats log: GPU busy, rail power, CPU load.

On Jetson, play_launch's per-node gpu_utilization_percent and
gpu_power_milliwatts are nan -- they come from NVML, which Tegra does not
implement. tegrastats is the only source, and on Orin power is a first-class
number: the case for moving NDT to the GPU is that it frees CPU inside a fixed
power envelope, which cannot be checked without the rails.

Reports the mean over a window that starts after the stack has settled, so map
loading and TensorRT warm-up do not colour the steady-state figure.

    tegrastats_summary.py <tegrastats.log> [--skip-seconds N]
    tegrastats_summary.py <run_dir>/tegrastats.log --tsv
"""

import argparse
import re
import statistics
import sys
from pathlib import Path

# GR3D_FREQ is the GPU's busy percentage. On some L4T releases it carries a
# trailing frequency, e.g. "GR3D_FREQ 45%@1300", so stop at the percent sign.
GR3D = re.compile(r"GR3D_FREQ\s+(\d+)%")
CPU_BLOCK = re.compile(r"CPU\s+\[([^\]]+)\]")
CPU_CORE = re.compile(r"(\d+)%@")
# Rails appear as "NAME instant/average", both in mW. Take the instantaneous
# reading; tegrastats' own running average is over the whole process lifetime,
# which spans the warm-up this script exists to exclude.
RAIL = re.compile(r"(VDD_[A-Z0-9_]+|VIN_[A-Z0-9_]+)\s+(\d+)mW/(\d+)mW")


def parse(path, skip_seconds):
    samples = []
    with open(path, errors="replace") as fh:
        for line in fh:
            gr3d = GR3D.search(line)
            if not gr3d:
                continue  # not a sample line
            row = {"gpu_pct": float(gr3d.group(1))}

            cpu_block = CPU_BLOCK.search(line)
            if cpu_block:
                cores = [float(c) for c in CPU_CORE.findall(cpu_block.group(1))]
                if cores:
                    # Sum, not mean: "138%" is the familiar top(1) reading for a
                    # process using 1.4 cores, and keeps this comparable with
                    # the cpu_percent column play_launch already reports.
                    row["cpu_sum_pct"] = sum(cores)
                    row["cpu_cores"] = len(cores)

            for name, instant, _avg in RAIL.findall(line):
                row[name] = float(instant)
            samples.append(row)

    # tegrastats samples at a fixed interval, so dropping the first N samples
    # drops the first N seconds at the default 1000 ms.
    return samples[skip_seconds:] if skip_seconds < len(samples) else []


def mean_of(samples, key):
    vals = [s[key] for s in samples if key in s]
    return statistics.fmean(vals) if vals else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log", type=Path)
    ap.add_argument(
        "--skip-seconds",
        type=int,
        default=30,
        help="discard this many leading samples, so map load and warm-up do "
        "not enter the steady-state mean (default 30)",
    )
    ap.add_argument("--tsv", action="store_true", help="one row, for tables")
    args = ap.parse_args()

    if not args.log.is_file():
        print(f"no tegrastats log at {args.log}", file=sys.stderr)
        return 1

    samples = parse(args.log, args.skip_seconds)
    if not samples:
        print(
            f"{args.log}: no usable samples after skipping {args.skip_seconds}",
            file=sys.stderr,
        )
        return 1

    rails = sorted(
        {k for s in samples for k in s if k.startswith(("VDD_", "VIN_"))}
    )
    gpu = mean_of(samples, "gpu_pct")
    cpu = mean_of(samples, "cpu_sum_pct")
    cores = next((s["cpu_cores"] for s in samples if "cpu_cores" in s), None)

    if args.tsv:
        cells = [f"{len(samples)}", f"{gpu:.1f}", f"{cpu:.0f}" if cpu else "-"]
        cells += [f"{mean_of(samples, r):.0f}" for r in rails]
        print("\t".join(cells))
        return 0

    print(f"{args.log}  ({len(samples)} samples, first {args.skip_seconds} dropped)")
    print(f"  GPU busy      {gpu:.1f} %")
    if cpu is not None:
        print(f"  CPU total     {cpu:.0f} % of {cores * 100} % ({cores} cores)")
    for rail in rails:
        print(f"  {rail:<13} {mean_of(samples, rail):.0f} mW")
    # Deliberately not summing the rails: VIN_SYS_5V0 is a system input and
    # overlaps the VDD_* loads, so a total would double-count. Compare rails
    # like for like across configurations instead.
    return 0


if __name__ == "__main__":
    sys.exit(main())
