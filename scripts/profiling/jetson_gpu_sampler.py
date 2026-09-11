#!/usr/bin/env python3
"""Sample Jetson GPU load, clock, power and temperature to CSV.

Usage:
    scripts/profiling/jetson_gpu_sampler.py [-i SECONDS] [-o OUT.csv] [-d SECONDS]

Why this exists: `play_launch`'s resource monitor reads GPU state through NVML,
and **NVML does not exist on Jetson** -- there is no libnvidia-ml on this board.
Every `gpu_*` column in every capture this project has taken is empty as a
result, which is why the CUDA A/B numbers had to be gathered by hand from
`tegrastats`.

Everything needed is in sysfs, readable without root, and it is the same set of
rails `tegrastats` prints:

    /sys/devices/platform/gpu.0/load        GPU busy, per-mille (0-1000)
    /sys/class/devfreq/17000000.gpu/cur_freq        GPU clock, Hz
    /sys/class/hwmon/hwmon*/  (name == ina3221)     VDD_GPU_SOC, VDD_CPU_CV, VIN_SYS_5V0
    /sys/class/thermal/thermal_zone*/               per-zone temperature

Verified on an AGX Orin (R36.4.4, sm_87): a CUDA kernel moved `load` from 0 to
999/1000 and VDD_GPU_SOC from 4.4 W to 25.9 W, so these are live readings and
not a stub. The paths are Orin-specific; on another Tegra the devfreq node and
the ina3221 rail names differ.

The timestamp column is ISO-8601 UTC with milliseconds, matching play_launch's
`system_stats.csv`, so a run of this beside a capture can be joined on time.
"""

import argparse
import csv
import glob
import os
import sys
import time
from datetime import datetime, timezone

GPU_LOAD = "/sys/devices/platform/gpu.0/load"
GPU_FREQ = "/sys/class/devfreq/17000000.gpu/cur_freq"


def read_int(path):
    value = read_str(path)
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def read_str(path):
    # Some sysfs nodes exist but fail on read -- several thermal zones on this
    # board return nothing at all, and the decoder then raises TypeError rather
    # than OSError. Catch broadly: a sampler must never take the run down.
    try:
        with open(path) as handle:
            return handle.read().strip()
    except Exception:
        return None


def find_rails():
    """Return [(label, volt_path, curr_path)] for every INA3221 channel.

    The board exposes more than one ina3221; the labelled channels on each are
    the rails worth recording. Channel numbering is not stable across boards,
    so the label is read rather than assumed.
    """
    rails = []
    for hwmon in sorted(glob.glob("/sys/class/hwmon/hwmon*")):
        if read_str(os.path.join(hwmon, "name")) != "ina3221":
            continue
        for label_path in sorted(glob.glob(os.path.join(hwmon, "in*_label"))):
            label = read_str(label_path)
            if not label or "shunt" in label:
                continue
            index = os.path.basename(label_path)[2:-6]  # in<N>_label -> N
            volt = os.path.join(hwmon, f"in{index}_input")
            curr = os.path.join(hwmon, f"curr{index}_input")
            if os.path.exists(volt) and os.path.exists(curr):
                rails.append((label, volt, curr))
    return rails


def find_zones():
    zones = []
    for zone in sorted(glob.glob("/sys/class/thermal/thermal_zone*")):
        kind = read_str(os.path.join(zone, "type"))
        temp = os.path.join(zone, "temp")
        # Some zones exist but never read; skip them rather than emit a column
        # of blanks.
        if kind and read_int(temp) is not None:
            zones.append((kind, temp))
    return zones


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("-i", "--interval", type=float, default=2.0,
                        help="seconds between samples (default: 2.0, matching play_launch)")
    parser.add_argument("-o", "--output", default="-",
                        help="CSV output path, or - for stdout (default: -)")
    parser.add_argument("-d", "--duration", type=float, default=0.0,
                        help="stop after this many seconds (default: run until interrupted)")
    args = parser.parse_args()

    if read_int(GPU_LOAD) is None:
        sys.exit(f"{GPU_LOAD}: not readable — this is not a Jetson, or the nvgpu "
                 f"driver is not loaded")

    rails = find_rails()
    zones = find_zones()

    columns = ["timestamp", "gpu_load_percent", "gpu_freq_mhz"]
    columns += [f"{label}_mw" for label, _, _ in rails]
    columns += [f"temp_{kind}_c" for kind, _ in zones]

    handle = sys.stdout if args.output == "-" else open(args.output, "w", newline="")
    writer = csv.writer(handle)
    writer.writerow(columns)

    started = time.monotonic()
    try:
        while True:
            row = [datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z"]

            load = read_int(GPU_LOAD)
            row.append(f"{load / 10.0:.1f}" if load is not None else "")

            freq = read_int(GPU_FREQ)
            row.append(f"{freq // 1000000}" if freq is not None else "")

            for _, volt_path, curr_path in rails:
                volt_mv = read_int(volt_path)
                curr_ma = read_int(curr_path)
                # mV * mA / 1000 = mW
                row.append(f"{volt_mv * curr_ma // 1000}"
                           if volt_mv is not None and curr_ma is not None else "")

            for _, temp_path in zones:
                milli_c = read_int(temp_path)
                row.append(f"{milli_c / 1000.0:.1f}" if milli_c is not None else "")

            writer.writerow(row)
            handle.flush()

            if args.duration and time.monotonic() - started >= args.duration:
                break
            time.sleep(args.interval)
    except KeyboardInterrupt:
        pass
    finally:
        if handle is not sys.stdout:
            handle.close()


if __name__ == "__main__":
    main()
