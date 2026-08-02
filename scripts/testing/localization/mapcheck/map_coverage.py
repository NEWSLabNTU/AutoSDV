#!/usr/bin/env python3
"""How much of the LiDAR scan has any map to match against, by range.

Answers the first question to ask before widening the measurement-range crop:
does the map even extend that far? A scan point beyond the mapped area cannot
constrain the pose, and because the downsample filter keeps a fixed number of
points regardless of range, unmapped far returns dilute the ones that carry the
constraint.

Cheap: no nearest-neighbour search, just a footprint test on a 2 m grid.
Run map_agreement.py afterwards to ask whether the mapped returns actually fit.

    scripts/testing/localization/mapcheck/map_coverage.py --run tmp/demo-runs/<run>
"""
import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import (CELL, DEFAULT_BAG, DEFAULT_MAP, cell_key, place, read_pcd,  # noqa: E402
                    read_poses, read_scans_near)

SHELLS = [(0, 10), (10, 20), (20, 30), (30, 40), (40, 60), (60, 80), (80, 120)]
CROPS = [20, 30, 40, 60, 100]


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--run", required=True, help="demo run directory (for NDT poses)")
    p.add_argument("--map", default=str(DEFAULT_MAP))
    p.add_argument("--bag", default=str(DEFAULT_BAG), help="raw bag with the full-range cloud")
    p.add_argument("--at", type=float, default=0.5,
                   help="where along the run to sample, 0..1 (default: halfway)")
    args = p.parse_args()

    mp = read_pcd(args.map)
    keys, counts = np.unique(cell_key(mp[:, 0], mp[:, 1]), return_counts=True)
    density = dict(zip(keys.tolist(), counts.tolist()))
    print(f"map: {len(mp)} points  "
          f"x[{mp[:,0].min():.0f},{mp[:,0].max():.0f}] "
          f"y[{mp[:,1].min():.0f},{mp[:,1].max():.0f}] "
          f"z[{mp[:,2].min():.1f},{mp[:,2].max():.1f}]  "
          f"{len(keys)} occupied {CELL:.0f} m cells")

    poses = read_poses(args.run)
    pose = poses[int(len(poses) * args.at)]
    scan = read_scans_near(args.bag, [pose[0]])[0]
    world, rng = place(scan, pose)
    hits = np.array([density.get(int(k), 0) for k in cell_key(world[:, 0], world[:, 1])])

    print(f"\nscan of {len(rng)} points at NDT pose ({pose[1]:.1f}, {pose[2]:.1f})")
    print(f"{'shell':>12} {'points':>8} {'in map':>8} {'in-map%':>8} {'map pts/cell':>13}")
    for lo, hi in SHELLS:
        sel = (rng >= lo) & (rng < hi)
        if not sel.any():
            continue
        h = hits[sel]
        n_in = int((h > 0).sum())
        dens = h[h > 0].mean() if n_in else 0.0
        print(f"{lo:4.0f}-{hi:<7.0f} {int(sel.sum()):8d} {n_in:8d} "
              f"{100 * n_in / sel.sum():7.1f}% {dens:13.0f}")
    print(f"{'total':>12} {len(rng):8d} {int((hits > 0).sum()):8d} "
          f"{100 * (hits > 0).sum() / len(rng):7.1f}%")

    print("\nwhat a crop box would keep:")
    total_mapped = max(1, int((hits > 0).sum()))
    for cap in CROPS:
        sel = rng <= cap
        if not sel.any():
            continue
        h = hits[sel]
        print(f"  +/-{cap:3d} m: {int(sel.sum()):6d} points, "
              f"{100 * (h > 0).sum() / sel.sum():5.1f}% of them mapped, "
              f"{100 * (h > 0).sum() / total_mapped:5.1f}% of all mapped points kept")


if __name__ == "__main__":
    main()
