#!/usr/bin/env python3
"""Judge a built world from swb's world_report.py JSON: exit 1 when a 5 cm wheel cannot drive
its lanes.

    world_gate.py WORLD_REPORT.json [--max-roughness 0.01] [--pit-depth 0.04]

Fails on an unmeshed lane cell (the car falls through) or lane roughness p99 above
--max-roughness. A pit deeper than the site's fill depth (--pit-depth) only warns: the
pipeline keeps real steps, such as COSS's 9 cm kerb, and routes avoid them.
"""
import argparse
import json
import sys


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("report")
    ap.add_argument("--max-roughness", type=float, default=0.01)
    ap.add_argument("--pit-depth", type=float, default=0.04)
    a = ap.parse_args()
    r = json.load(open(a.report))
    bad = []
    if r["lane_cells_meshed"] < 1.0:
        bad.append(f"lanes {100 * r['lane_cells_meshed']:.2f} % meshed: a hole under a lanelet")
    if r["roughness_p99"] > a.max_roughness:
        bad.append(f"lanes rough: p99 {1000 * r['roughness_p99']:.1f} mm > {1000 * a.max_roughness:.0f} mm")
    if r["pit_depth_max"] > a.pit_depth:
        print(f"[world gate] warning: a pit {100 * r['pit_depth_max']:.0f} cm deep under a lane "
              f"(the site fills {100 * a.pit_depth:.0f} cm); keep routes off it")
    for b in bad:
        print(f"[world gate] FAIL: {b}")
    if not bad:
        print(f"[world gate] ok: lanes meshed, p99 roughness {1000 * r['roughness_p99']:.1f} mm")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
