"""`ros2 run autosdv_lab3 lab3_check_lanelet MAP_DIR`: is the lanelet2 loop drivable?

    lab3_check_lanelet MAP_DIR [--no-sim] [--goal-behind 1.0] [--timeout S] [--keep-log]

Four requirements, each PASS or FAIL:

  loop          the road lanelets form one closed chain by successor relations
                (the next lanelet starts on the same two nodes the previous ends on)
  clockwise     the loop's centreline, seen from above (z up), has negative signed area
  speed limits  every lanelet has speed_limit set, above 0 and at most 10 km/h
  start boundary  a lanelet boundary BOUNDARY_MIN..BOUNDARY_MAX m behind the map origin
                (the start mark: the mapping recording began there), along the loop

The run's single goal, about 1.0 m behind the start, must fall in the lanelet BEFORE the
start's: with goal and start in one lanelet Autoware routes the 1 m behind the car, and
with the boundary too close behind the car the planner's look-behind finds the goal and
cuts the path there (the TA simulation: 0.14 and 0.25 m past did not move, 0.62 m did).
So the boundary goes 0.4-0.6 m behind the start mark and the goal about 1.0 m behind
it, 0.4-0.6 m into the previous lanelet. A hint below the table says where the
boundary and the goal fall, and which way the loop leaves the start mark (the direction
to drag for 2D Pose Estimate).

Unless --no-sim, a lap in the planning simulator follows (lab3_sim_lap): the car
starts at the origin projected onto the loop and drives to one goal --goal-behind m
behind it, the procedure of the field run. Exit 0 only when every requirement and the
lap pass. The result is recorded in the map folder's lab3_map.yaml (lanelet_check).
"""

import argparse
import math
import os
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import List, Optional

from autosdv_lab3 import lanelet_speed, mapdir
from autosdv_lab3.route import Loop, MapError, find_chain, read_lanelets

GOAL_BEHIND = 1.0     # m: the run's 2D Goal Pose behind the start (0.7 m was unreliable)
# m: the goal this far before the boundary, at least (measured: 1.0 m behind a start
# 0.5 m past it drove; 0.2 m before it did not move)
GOAL_CLEAR = 0.5
ORIGIN_OFF_LOOP = 1.0  # m from the centreline: the recording did not start on the loop
# m behind the origin: the start boundary. Below 0.4 the TA simulation did not move (0.14,
# 0.25 m); above 0.6 the 1.0 m goal falls within 0.4 m of the boundary, the failing class
# (0.2 m before it did not move).
BOUNDARY_MIN, BOUNDARY_MAX = 0.4, 0.6


@dataclass
class Result:
    requirement: str
    ok: Optional[bool]    # None: not run (an earlier failure leaves nothing to measure)
    value: str = ''
    reason: str = ''


def repo_root() -> Optional[Path]:
    """The AutoSDV checkout: $AUTOSDV_ROOT, else up from this file, else up from cwd."""
    if os.environ.get('AUTOSDV_ROOT'):
        return Path(os.environ['AUTOSDV_ROOT'])
    for start in (Path(__file__).resolve(), Path.cwd().resolve()):
        for d in [start] + list(start.parents):
            if (d / 'versions.yaml').is_file() and (d / 'scripts/map/check_map.py').is_file():
                return d
    return None


# ------------------------------------------------------------------ the checks

def checks(map_dir: str):
    """([Result], Loop or None)."""
    try:
        lanelets = read_lanelets(os.path.join(map_dir, mapdir.LANELET))
    except MapError as e:
        return [Result('loop', False, '', str(e)),
                Result('clockwise', None, '', 'needs a readable map'),
                Result('speed limits', None, '', 'needs a readable map')], None
    if not lanelets:
        return [Result('loop', False, '', 'no road lanelets in the file'),
                Result('clockwise', None, '', 'needs the closed loop'),
                Result('speed limits', None, '', 'needs lanelets')], None
    res = []
    order, problem = find_chain(lanelets)
    loop = None
    if problem:
        res.append(Result('loop', False, f'{len(order)} of {len(lanelets)} chained', problem))
        res.append(Result('clockwise', None, '', 'needs the closed loop'))
    else:
        loop = Loop.from_chain(lanelets, order)
        res.append(Result('loop', True, f'{len(order)} lanelets, {loop.length:.1f} m'))
        area = loop.signed_area()
        res.append(Result('clockwise', area < 0, f'signed area {area:+.1f} m^2',
                          '' if area < 0 else 'the loop runs counter-clockwise seen from above: '
                          'redraw it the other way round (lanelets point the way the car drives)'))
    ok, value, reason = lanelet_speed.check(lanelets)
    res.append(Result('speed limits', ok, value,
                      reason + ('; set each lanelet\'s speed limit in Vector Map Builder'
                                if reason else '')))
    res.append(boundary_check(loop))
    return res, loop


def boundary_check(loop: Optional[Loop], origin=(0.0, 0.0)) -> Result:
    """The start boundary: a lanelet boundary BOUNDARY_MIN..BOUNDARY_MAX m behind the origin."""
    if loop is None:
        return Result('start boundary', None, '', 'needs the closed loop')
    h = start_hint(loop, origin)
    if h.off_loop > ORIGIN_OFF_LOOP:
        return Result('start boundary', False, f'origin {h.off_loop:.1f} m off the loop',
                      'the map origin is the start mark: the mapping recording must start with '
                      'the car on the start mark, on the loop')
    ok = BOUNDARY_MIN <= h.behind <= BOUNDARY_MAX
    return Result('start boundary', ok, f'{h.behind:.2f} m behind the origin',
                  '' if ok else f'split the lanelet so that one begins about 0.5 m behind the '
                  f'start mark (the map origin), {BOUNDARY_MIN:g}-{BOUNDARY_MAX:g} m')


def static_checks(map_dir: str) -> List[Result]:
    return checks(map_dir)[0]


@dataclass
class StartHint:
    s_origin: float          # the origin projected onto the loop, arc length
    off_loop: float          # m from the origin to the centreline
    lanelet: int             # the lanelet the origin projects into
    behind: float            # m from the nearest boundary behind the origin, along the loop
    previous: int            # the lanelet before it
    near: bool               # BOUNDARY_MIN <= behind <= BOUNDARY_MAX
    goal_lanelet: int        # where a goal goal_behind m behind the start falls
    goal_behind: float
    heading_deg: float = 0.0  # the loop's direction at the origin, from +x, counter-clockwise
    axis: str = '+x'         # the map axis nearest that direction

    @property
    def goal_in_previous(self) -> bool:
        return self.goal_lanelet != self.lanelet

    def text(self) -> str:
        lines = [f'the map origin (the start mark) projects onto lanelet {self.lanelet}, '
                 f'{self.behind:.2f} m past its start (lanelet {self.previous} ends there)']
        if self.off_loop > ORIGIN_OFF_LOOP:
            lines.append(f'the origin is {self.off_loop:.1f} m off the loop: did the mapping '
                         'recording start on the start mark?')
        lines.append(f'the loop leaves the start mark towards the map\'s {self.axis} '
                     f'(heading {self.heading_deg:.0f} deg): drag that way for 2D Pose Estimate')
        lines.append(('good, the start lanelet begins ' if self.near else
                      'NOT good: split the lanelet so that one begins ')
                     + f'{BOUNDARY_MIN:g}-{BOUNDARY_MAX:g} m behind the start mark')
        least = self.behind + GOAL_CLEAR
        if self.goal_in_previous and self.goal_behind >= least:
            lines.append(f'a goal {self.goal_behind:g} m behind the start falls in lanelet '
                         f'{self.goal_lanelet}, the previous one: good')
        elif self.goal_in_previous:
            lines.append(f'a goal {self.goal_behind:g} m behind the start falls in lanelet '
                         f'{self.goal_lanelet}, only {self.goal_behind - self.behind:.2f} m '
                         f'before the boundary: put it at least {least:.1f} m behind the start')
        else:
            lines.append(f'a goal {self.goal_behind:g} m behind the start falls in lanelet '
                         f'{self.goal_lanelet}, the start\'s own: put it at least {least:.1f} m '
                         'behind the start, or split the lanelet')
        return '\n'.join('  hint: ' + ln for ln in lines)

    def as_dict(self) -> dict:
        d = asdict(self)
        return {k: (round(v, 3) if isinstance(v, float) else v) for k, v in d.items()}


def start_hint(loop: Loop, origin=(0.0, 0.0), goal_behind: float = GOAL_BEHIND) -> StartHint:
    s0 = loop.project(*origin)
    x, y, _ = loop.pose_at(s0)
    lid = loop.lane_at(s0)
    behind = loop.wrap(s0 - loop.starts[lid])
    previous = loop.order[loop.order.index(lid) - 1]
    x1, y1, _ = loop.pose_at(s0 + 0.5)
    dx, dy = x1 - x, y1 - y
    heading = math.degrees(math.atan2(dy, dx))
    axis = ('+x' if dx > 0 else '-x') if abs(dx) >= abs(dy) else ('+y' if dy > 0 else '-y')
    return StartHint(s_origin=s0, off_loop=float(((x - origin[0]) ** 2 + (y - origin[1]) ** 2) ** 0.5),
                     lanelet=lid, behind=behind, previous=previous, near=BOUNDARY_MIN <= behind <= BOUNDARY_MAX,
                     goal_lanelet=loop.lane_at(s0 - goal_behind), goal_behind=goal_behind,
                     heading_deg=heading, axis=axis)


def table(results: List[Result]) -> str:
    rows = [('requirement', 'result', 'measured', 'reason')]
    for r in results:
        rows.append((r.requirement, {True: 'PASS', False: 'FAIL', None: '-'}[r.ok], r.value, r.reason))
    w = [max(len(row[k]) for row in rows) for k in range(3)]
    return '\n'.join(f'  {a:<{w[0]}}  {b:<{w[1]}}  {c:<{w[2]}}  {d}'.rstrip() for a, b, c, d in rows)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='Check a Lab 3 lanelet2 loop, then drive it in the simulator.')
    ap.add_argument('map_dir')
    ap.add_argument('--no-sim', action='store_true', help='the requirements only, no simulator lap')
    ap.add_argument('--goal-behind', type=float, default=GOAL_BEHIND,
                    help=f'm from the start back to the one goal (default {GOAL_BEHIND:g})')
    ap.add_argument('--timeout', type=float, default=None, help='simulator lap timeout, s')
    ap.add_argument('--keep-log', action='store_true', help="keep the simulator's run directory")
    args = ap.parse_args(argv)
    if not os.path.isdir(args.map_dir):
        print(f'error: {args.map_dir} is not a directory', file=sys.stderr)
        return 2

    results, loop = checks(args.map_dir)
    hint = start_hint(loop, goal_behind=args.goal_behind) if loop is not None else None
    static_ok = all(r.ok for r in results)
    sim = None
    if args.no_sim:
        results.append(Result('sim lap', None, '', 'skipped (--no-sim)'))
    elif not static_ok:
        results.append(Result('sim lap', None, '', 'not run: fix the rows above first'))
    elif not hint.goal_in_previous:
        results.append(Result('sim lap', False, '', f'not run: a goal {args.goal_behind:g} m behind '
                              f'the origin falls in lanelet {hint.lanelet}, the start\'s own '
                              f'(Autoware then routes the {args.goal_behind:g} m behind the car)'))
    else:
        print(table(results))
        print(hint.text())
        print('\nrequirements pass; driving a lap in the planning simulator ...', flush=True)
        from autosdv_lab3 import sim_lap
        sim = sim_lap.run_lap(args.map_dir, timeout=args.timeout, keep_log=args.keep_log,
                              goal_behind=args.goal_behind)
        results.append(Result('sim lap', sim.ok, sim.value if sim.ok else sim.reason,
                              '' if sim.ok else sim.value))
    print(table(results))
    if hint is not None:
        print(hint.text())
    passed = all(r.ok for r in results if not (args.no_sim and r.requirement == 'sim lap'))
    data = {
        'passed': bool(passed),
        'sim': not args.no_sim,
        'goal_behind': args.goal_behind,
        'lanelet_sha256': mapdir.sha256(os.path.join(args.map_dir, mapdir.LANELET))
        if os.path.isfile(os.path.join(args.map_dir, mapdir.LANELET)) else None,
        'results': [asdict(r) for r in results],
        'start_hint': hint.as_dict() if hint is not None else None,
    }
    if sim is not None:
        data['sim_lap'] = sim.as_dict()
    mapdir.record_step(args.map_dir, 'lanelet_check', data)
    print('\nPASS' if passed else '\nFAIL')
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
