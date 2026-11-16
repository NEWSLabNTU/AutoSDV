"""`ros2 run autosdv_lab3 lab3_localization_report BAG`: how well the stack localized.

For a recording of a replay (Part 1: the team's own mapping bag played back against the
team's map) or of any run. BAG must hold the localization output and what the scan-to-map
measure needs, which config/record/localization_topics.txt lists: the pose estimate,
NDT's NVTL, the LiDAR, /tf_static and the map.

Over the time NDT was publishing (first to last NVTL message) it reports:

- localized share: the share of 0.5 s bins in which NDT scored at or above the Lab 3
  gate (NVTL 1.3); a bin with no NDT output counts as not localized;
- NVTL median and minimum;
- scan-to-map: one LiDAR scan every 0.5 s placed in the map at the estimated pose, each
  point's distance to the nearest map point; the median over the scans, and the share of
  points that lie on the map at all (alignment.py).

The same measures as the week-2 grader's, so a team sees in Part 1 what Part 2 will
measure on its lap. Written to localization.yaml in the bag folder.
"""

import argparse
import os
import sys

import numpy as np
import yaml

from autosdv_lab3 import alignment, grade

REPORT = 'localization.yaml'


def report(bag: str, map_pcd=None) -> dict:
    r = grade.read_run(bag)
    rep = {'bag': os.path.abspath(bag)}
    if not len(r.nvtl):
        rep['error'] = f'the bag has no {grade.NVTL}: was the stack running and localized?'
        return rep
    t0, t1 = float(r.nvtl[0, 0]), float(r.nvtl[-1, 0])
    rep['duration_s'] = round(t1 - t0, 1)
    rep['localized_share'] = grade.localized_share(r.nvtl, t0, t1)
    rep['nvtl_median'] = round(float(np.median(r.nvtl[:, 1])), 3)
    rep['nvtl_min'] = round(float(r.nvtl[:, 1].min()), 3)
    rep['nvtl_gate'] = grade.NVTL_GATE
    if r.lidar_topic:
        rep['scan_to_map'] = alignment.read_alignment(bag, r.lidar_topic, t0, t1, grade.ODOM, map_pcd)
    else:
        rep['scan_to_map'] = {'scans': 0, 'why': 'the bag holds no LiDAR topic'}
    return rep


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog='lab3_localization_report',
                                 description='Localization measures of a recorded run or replay.')
    ap.add_argument('bag')
    ap.add_argument('--map', help='the map PCD, when the bag was recorded without /map/pointcloud_map')
    ap.add_argument('-o', '--out', help=f'report path (default BAG/{REPORT})')
    args = ap.parse_args(argv)
    if not os.path.isdir(args.bag):
        print(f'lab3_localization_report: {args.bag} is not a bag folder', file=sys.stderr)
        return 2
    rep = report(args.bag, args.map)
    print(f'\nLab 3 localization: {args.bag}')
    for k, v in rep.items():
        if k != 'bag':
            print(f'  {k:18s} {v}')
    out = args.out or os.path.join(args.bag, REPORT)
    with open(out, 'w') as f:
        yaml.safe_dump(rep, f, sort_keys=False)
    print(f'  report: {out}')
    return 0 if 'error' not in rep else 1


if __name__ == '__main__':
    sys.exit(main())
