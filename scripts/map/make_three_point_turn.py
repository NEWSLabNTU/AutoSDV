#!/usr/bin/env python3
"""A three-point turn for Autoware 2.1.0's direction change module, at RC-car scale.

    python3 scripts/map/make_three_point_turn.py [--out DIR] [--lane-width 0.8] [--radius 1.2]

Writes src/sim/autosdv_rc_sim/tracks/three_point_turn by default. (Not in the autosdv_rc_sim
Python package: modules there are on the lockstep path, which may not call libm.)

The car drives east in one lane, turns left into a bay and stops facing north (forward),
backs out swinging round to face west (reverse), and drives west into the other lane
(forward). Seen from above (x east, y north, metres, defaults):

     2.6 +-----------------------------+
         |                bay    .     |   T: one lanelet, direction_change=yes; its
         |                     .  .    |      custom centerline is the whole manoeuvre:
     1.2 +---- L4 -----+    .      .   |      (1.0, 0) left arc R=1.2 and north to the
     0.4 |   <- west   0 <- - - - -    |      cusp (2.2, 2.0); REVERSE on an arc R to the
         +---- L0 -----------+ .       |      cusp (3.4, 0.8); west to (0, 0.8)
    -0.4 |   -> east        1.0   T    |
         +-------------------+---------+
        -3.5          0     1.0       4.0

    L0  eastbound approach (prefix, untagged), ends at x = 1.0; the start pose is on it
    T   the turn, tagged direction_change=yes; entered at x = 1.0 (eastbound lane),
        left at x = 0 (westbound lane)
    L4  westbound exit (suffix, untagged), starts at x = 0; the goal is on it

How the module reads it. It concatenates the centerlines of the tagged route lanelets
(here T's alone) and calls a cusp every yaw jump above cusp_detection_angle_threshold_deg
(90) between consecutive points; the path's yaw is the direction from each point to the
next, so a centerline that turns back on itself has a 180 deg jump at the turning point.
T's centerline turns back twice. On the leg after an odd number of cusps the module flips
the yaw by pi and makes the velocity negative: the car faces against its travel.

Why one lanelet, with the cusps inside its centerline rather than between lanelets.
Lanelet2 makes B a successor of A only when A's left and right bounds end on the points
where B's left and right bounds start. At a cusp the travel direction reverses, so a
lanelet per leg needs its left and right swapped relative to its own direction (a
"mirrored" lanelet), and Lanelet2's OSM loader re-orients those (geometry::align, below):
the reverse leg came out reversed and the route broke. The alternative upstream offers,
lanelet-area-lanelet routes (allow_area), does not work in the 2.1.0 binaries: the route
handler builds its routing graph with Germany's vehicle rules, which refuse every area,
and a route given segment by segment with area primitives kills the mission planner
(NoSuchPrimitiveError on the area id). So L0 -> T -> L4 is an ordinary three-lanelet
route, the routing graph's own shortest path, and a goal pose on L4 is all the mission
planner needs (no allow_area anywhere).

Why T is left west of where it is entered: the module trims the tagged centerline at the
point nearest the route goal (trimPathToGoal). A U-turn ends next to where it began; with
T's exit east of its entry, a goal on L4 is nearer T's first point, the trimmed centerline
is one point long, and the module finds no cusp (tried).

T's shape: its left bound is the short stretch of the centre line between the lanes,
from (1.0, 0.4) back to (0, 0.4); its right bound runs round the outside of the bay.
Lanelet2's OSM loader reverses a bound whose middle point (the point at index size/2)
lies on the wrong side of the other bound (geometry::align), so the right bound is
sampled densely along the bay's far edge to keep its middle point there. T's default
centerline (used only for closest-lanelet angles) is meaningless; the custom one matters.

Centerlines: every lanelet carries a custom centerline sampled every 0.1 m. Autoware's
map loader otherwise regenerates centerlines at center_line_resolution (5 m); with
use_waypoints (true in the loader's config) a custom centerline becomes the lanelet's
waypoints, and the route handler builds every path from them.

Radius: the car turns at 0.88 m (wheelbase 0.319 m, 0.349 rad); both arcs default to
1.2 m. The reverse leg (1.9 m) is longer than cusp_detection_distance_threshold (1.2 m):
a leg shorter than that starts inside the next cusp's stop zone, and the module would
switch again without moving.

Also written: a flat point cloud for the map loader and its metadata (one 20 m cell),
map_projector_info.yaml (Local), and track.yaml with the start and goal poses (start on
L0 facing east, goal on L4 facing west), which the RC plant and the smoke tests read.
"""
import argparse
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import yaml

DEFAULT_OUT = str(Path(__file__).resolve().parents[2] / 'src' / 'sim' / 'autosdv_rc_sim'
                  / 'tracks' / 'three_point_turn')
CENTERLINE_STEP = 0.1     # [m] custom centerline spacing
SPEED_LIMIT_KMH = 3.6     # 1 m/s on the approach and the exit
# The tagged lanelet holds both reverse legs. The actuator caps reverse at reverse_max_speed
# (0.5 m/s, autosdv_vehicle_interface), so the manoeuvre is planned at that speed rather than
# commanded faster and silently capped.
TAGGED_SPEED_LIMIT_KMH = 1.8  # 0.5 m/s


def _line(p0, p1, step=CENTERLINE_STEP):
    n = max(1, int(math.ceil(math.dist(p0, p1) / step)))
    return [(p0[0] + (p1[0] - p0[0]) * i / n, p0[1] + (p1[1] - p0[1]) * i / n) for i in range(n + 1)]


def _arc(center, radius, a0, a1, step=CENTERLINE_STEP):
    n = max(1, int(math.ceil(abs(a1 - a0) * radius / step)))
    return [(center[0] + radius * math.cos(a0 + (a1 - a0) * i / n),
             center[1] + radius * math.sin(a0 + (a1 - a0) * i / n)) for i in range(n + 1)]


def _join(*pieces):
    out = list(pieces[0])
    for p in pieces[1:]:
        out += p[1:]
    return out


def geometry(w=0.8, r=1.2, entry=1.0, approach=3.0, exit_x=-3.5, margin=0.6):
    """Lanelets [(name, left, right, centerline, tagged)] and the two cusps, in metres."""
    h = w / 2.0
    y_west = w                       # the westbound lane's centre
    q = math.pi / 2.0
    # T is entered at x = entry in the eastbound lane and left at x = 0 in the westbound
    # lane, west of where it was entered: the module trims the tagged centerline at the
    # point nearest the goal (trimPathToGoal), so for a goal on L4 that point must be T's
    # last, not its first.
    c1 = (entry, r)                                  # leg 1: left arc to heading north
    top = y_west + r                                 # where leg 2's arc must start
    leg1 = _join(_arc(c1, r, -q, 0.0), _line((entry + r, r), (entry + r, top)))
    c2 = (entry + 2 * r, top)                        # leg 2: travel south, swinging east
    leg2 = _arc(c2, r, math.pi, 1.5 * math.pi)       # ends at (entry + 2r, y_west), car facing west
    leg3 = _line((entry + 2 * r, y_west), (0.0, y_west))
    centre = _join(leg1, leg2, leg3)
    x_max = entry + 2 * r + margin
    y_max = top + margin
    t_left = [(entry, h), (0.0, h)]
    # Lanelet2's loader (geometry::align) reverses a bound whose middle point (the point at
    # index size/2) lies on the wrong side of the other bound. Sampling the bay's far edge
    # densely puts the right bound's middle point there, right of the (westward) left bound.
    t_right = ([(entry, -h), (x_max, -h)] + _line((x_max, y_max), (0.0, y_max), 0.5)
               + [(0.0, y_west + h)])
    lls = [
        ('L0', [(-approach, h), (entry, h)], [(-approach, -h), (entry, -h)],
         _line((-approach, 0.0), (entry, 0.0)), False),
        ('T', t_left, t_right, centre, True),
        ('L4', [(0.0, y_west - h), (exit_x, y_west - h)], [(0.0, y_west + h), (exit_x, y_west + h)],
         _line((0.0, y_west), (exit_x, y_west)), False),
    ]
    cusps = [leg1[-1], leg2[-1]]
    return lls, cusps


def _osm(lls) -> str:
    root = ET.Element('osm', {'version': '0.6', 'generator': 'scripts/map/make_three_point_turn.py'})
    ET.SubElement(root, 'MetaInfo', {'format_version': '1', 'map_version': '1'})
    next_id = [1]
    shared = {}   # points shared between consecutive lanelets (Lanelet2 succession)

    def new_id():
        next_id[0] += 1
        return next_id[0] - 1

    def node(p, share):
        # bounds share points, which is what makes the lanelets successors; a centerline
        # keeps its own points (T's turns back across itself)
        key = (round(p[0], 4), round(p[1], 4))
        if share and key in shared:
            return shared[key]
        nid = new_id()
        el = ET.SubElement(root, 'node', {'id': str(nid), 'lat': '0', 'lon': '0'})
        for k, v in (('local_x', f'{p[0]:.4f}'), ('local_y', f'{p[1]:.4f}'), ('ele', '0.0000')):
            ET.SubElement(el, 'tag', {'k': k, 'v': v})
        if share:
            shared[key] = nid
        return nid

    def way(points, tags, share):
        ids = [node(p, share) for p in points]
        wid = new_id()
        el = ET.SubElement(root, 'way', {'id': str(wid)})
        for n in ids:
            ET.SubElement(el, 'nd', {'ref': str(n)})
        for k, v in tags:
            ET.SubElement(el, 'tag', {'k': k, 'v': v})
        return wid

    line = (('type', 'line_thin'), ('subtype', 'solid'))
    for name, left, right, center, tagged in lls:
        lw = way(left, line, True)
        rw = way(right, line, True)
        cw = way(center, (), False)
        rel = ET.SubElement(root, 'relation', {'id': str(new_id())})
        for role, ref in (('left', lw), ('right', rw), ('centerline', cw)):
            ET.SubElement(rel, 'member', {'type': 'way', 'role': role, 'ref': str(ref)})
        tags = [('type', 'lanelet'), ('subtype', 'road'), ('location', 'urban'),
                ('one_way', 'yes'), ('participant:vehicle', 'yes'),
                ('speed_limit', f'{TAGGED_SPEED_LIMIT_KMH if tagged else SPEED_LIMIT_KMH:.1f}'),
                ('name', name)]
        if tagged:
            tags.append(('direction_change', 'yes'))
        for k, v in tags:
            ET.SubElement(rel, 'tag', {'k': k, 'v': v})
    ET.indent(root)
    return '<?xml version="1.0" encoding="UTF-8"?>\n' + ET.tostring(root, encoding='unicode') + '\n'


def _pcd(lls, step=0.25) -> str:
    xs = [p[0] for _, l, r, _, _ in lls for p in l + r]
    ys = [p[1] for _, l, r, _, _ in lls for p in l + r]
    x0, x1 = math.floor(min(xs)) - 1.0, math.ceil(max(xs)) + 1.0
    y0, y1 = math.floor(min(ys)) - 1.0, math.ceil(max(ys)) + 1.0
    points = []
    for i in range(int(round((x1 - x0) / step)) + 1):
        for j in range(int(round((y1 - y0) / step)) + 1):
            points.append(f'{x0 + i * step:.3f} {y0 + j * step:.3f} 0.000')
    header = ['# .PCD v0.7 - Point Cloud Data file format', 'VERSION 0.7', 'FIELDS x y z',
              'SIZE 4 4 4', 'TYPE F F F', 'COUNT 1 1 1', f'WIDTH {len(points)}', 'HEIGHT 1',
              'VIEWPOINT 0 0 0 1 0 0 0', f'POINTS {len(points)}', 'DATA ascii']
    return '\n'.join(header + points) + '\n'


def write(out_dir, w=0.8, r=1.2) -> dict:
    lls, cusps = geometry(w=w, r=r)
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    (out / 'lanelet2_map.osm').write_text(_osm(lls))
    (out / 'pointcloud_map.pcd').write_text(_pcd(lls))
    (out / 'map_projector_info.yaml').write_text('projector_type: Local\n')
    # one cell holding the whole cloud (the map loader's differential loading; just map check)
    (out / 'pointcloud_map_metadata.yaml').write_text(
        'x_resolution: 20.0\ny_resolution: 20.0\npointcloud_map.pcd: [-10, -10]\n')
    data = {
        'kind': 'three_point_turn', 'lane_width': w, 'radius': r,
        'tagged': [n for n, *_, t in lls if t],
        'cusps': [{'x': round(x, 4), 'y': round(y, 4)} for x, y in cusps],
        'start_pose': {'x': -2.0, 'y': 0.0, 'yaw': 0.0},
        'goal_pose': {'x': -1.5, 'y': round(w, 4), 'yaw': round(math.pi, 6)},
    }
    (out / 'track.yaml').write_text(
        '# Generated by scripts/map/make_three_point_turn.py; regenerate, do not edit.\n'
        + yaml.safe_dump(data, sort_keys=False, default_flow_style=None))
    return data


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='Write the three-point-turn map (direction change module).')
    ap.add_argument('--out', default=DEFAULT_OUT, help=f'output directory (default {DEFAULT_OUT})')
    ap.add_argument('--lane-width', type=float, default=0.8)
    ap.add_argument('--radius', type=float, default=1.2, help='both arcs [m] (the car turns at 0.88)')
    args = ap.parse_args(argv)
    if args.radius < 0.88:
        print(f'error: radius {args.radius} m is tighter than the car turns (0.88 m)', file=sys.stderr)
        return 2
    data = write(args.out, args.lane_width, args.radius)
    print(f"wrote {args.out}: lanelet2_map.osm, pointcloud_map.pcd, pointcloud_map_metadata.yaml, "
          f"map_projector_info.yaml, track.yaml (tagged {', '.join(data['tagged'])}; cusps "
          + ', '.join(f"({c['x']:.2f}, {c['y']:.2f})" for c in data['cusps']) + ')')
    return 0


if __name__ == '__main__':
    sys.exit(main())
