#!/usr/bin/env python3
"""Where the map covers the scan, does it agree with it -- and if not, why?

Two questions, from the same nearest-neighbour pass over several observer poses:

1. **Ground or vegetation?** Ground cannot move between mapping and recording,
   so a displaced *ground* return means the map is wrong there. Foliage moves
   with the wind and the season, and in a park it disagrees at every range.
   Height is measured against the lowest map point in each 2 m cell, so a
   sloping site needs no global plane assumption.

2. **Warped map or moving foliage?** If several vehicle poses see the same map
   cell displaced the same way, the displacement belongs to the map (SLAM drift
   while mapping). If each observer sees something different, it is noise or
   vegetation. Reported as a coherence: 1.0 means every observer agrees.

A near field that fits while the far field does not is also evidence that the
pose is right -- a wrong pose spoils the near field first.

    scripts/testing/localization/mapcheck/map_agreement.py --run tmp/demo-runs/<run>
"""
import argparse
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import (CELL, DEFAULT_BAG, DEFAULT_MAP, cell_key, place, read_pcd,  # noqa: E402
                    read_poses, read_scans_near)

SHELLS = [(0, 10), (10, 20), (20, 30), (30, 40), (40, 60), (60, 80)]
CLASSES = (("ground <0.5m", 0.0, 0.5), ("low veg 0.5-2m", 0.5, 2.0), ("high >2m", 2.0, 1e9))


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--run", required=True, help="demo run directory (for NDT poses)")
    p.add_argument("--map", default=str(DEFAULT_MAP))
    p.add_argument("--bag", default=str(DEFAULT_BAG))
    p.add_argument("--poses", type=int, default=8, help="observer poses to sample")
    p.add_argument("--coherence-range", type=float, default=25.0,
                   help="[m] only cells beyond this take part in the warp test")
    args = p.parse_args()

    mp = read_pcd(args.map)
    mk = cell_key(mp[:, 0], mp[:, 1])
    order = np.argsort(mk)
    mk_s, mz_s = mk[order], mp[order, 2]
    uniq, starts = np.unique(mk_s, return_index=True)
    ends = np.append(starts[1:], len(mk_s))
    ground = {int(k): float(mz_s[s:e].min()) for k, s, e in zip(uniq, starts, ends)}
    print(f"map: {len(mp)} points, {len(ground)} cells; building KD-tree ...", flush=True)
    tree = cKDTree(mp)

    poses = read_poses(args.run)
    picks = [poses[int(len(poses) * f)] for f in np.linspace(0.15, 0.99, args.poses)]
    scans = read_scans_near(args.bag, [p_[0] for p_ in picks])

    acc = defaultdict(list)
    cellres = defaultdict(list)
    for i, pose in enumerate(picks):
        world, rng = place(scans[i], pose)
        # workers=-1 spreads the query over every core; this is the expensive step
        dist, idx = tree.query(world, k=1, workers=-1)
        resid = mp[idx] - world
        wk = cell_key(world[:, 0], world[:, 1])
        gz = np.array([ground.get(int(k), np.nan) for k in wk])
        height = world[:, 2] - gz

        for lo, hi in SHELLS:
            in_shell = (rng >= lo) & (rng < hi) & np.isfinite(height)
            for label, hlo, hhi in CLASSES:
                sel = in_shell & (height >= hlo) & (height < hhi)
                if sel.sum() >= 20:
                    acc[(lo, hi, label)].append(dist[sel])

        far_ground = (rng >= args.coherence_range) & np.isfinite(height) & (height < 0.5)
        for k, rv in zip(wk[far_ground], resid[far_ground]):
            cellres[(int(k), i)].append(rv)

    print("\nscan-to-map distance [m], median, by what the beam hit")
    print(f"{'shell':>10} {'ground <0.5m':>14} {'low veg 0.5-2m':>16} {'high >2m':>10}")
    for lo, hi in SHELLS:
        row = []
        for label, _, _ in CLASSES:
            v = acc.get((lo, hi, label))
            row.append(f"{np.median(np.concatenate(v)):.3f}" if v else "-")
        print(f"{lo:3.0f}-{hi:<6.0f} {row[0]:>14} {row[1]:>16} {row[2]:>10}")

    percell = defaultdict(dict)
    for (k, i), vs in cellres.items():
        if len(vs) >= 5:
            percell[k][i] = np.mean(np.stack(vs), axis=0)
    multi = {k: v for k, v in percell.items() if len(v) >= 3}
    print(f"\nwarp test: ground cells beyond {args.coherence_range:.0f} m "
          f"seen by >=3 of {args.poses} poses: {len(multi)}")
    if not multi:
        print("  too few shared cells; try --poses or a longer run")
        return
    mags, cohs, spreads = [], [], []
    for obs in multi.values():
        vecs = np.stack(list(obs.values()))
        mean = vecs.mean(axis=0)
        mags.append(np.linalg.norm(mean))
        cohs.append(np.linalg.norm(mean) / max(1e-9, np.linalg.norm(vecs, axis=1).mean()))
        spreads.append(np.linalg.norm(vecs - mean, axis=1).mean())
    mags, cohs, spreads = np.array(mags), np.array(cohs), np.array(spreads)
    print(f"  per-cell displacement   : median {np.median(mags):.3f} m, "
          f"p90 {np.percentile(mags, 90):.3f} m")
    print(f"  disagreement over poses : median {np.median(spreads):.3f} m")
    print(f"  coherence               : median {np.median(cohs):.2f} "
          f"(1 = every observer sees the same shift, 0 = random)")
    print(f"  coherent shifts > 0.5 m : {int(((mags > 0.5) & (cohs > 0.7)).sum())} "
          f"of {len(mags)} cells")
    print("\n  high coherence with a real magnitude means the map is displaced there;\n"
          "  low coherence means vegetation or noise, not a map defect.")


if __name__ == "__main__":
    main()
