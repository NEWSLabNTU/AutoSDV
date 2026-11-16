"""Lab 3 week 2: grade one autonomous lap from its bag alone.

    lab3_grader --team N [--out-dir ~/lab3/bags]   record a dated bag; Ctrl-C grades it
    lab3_grade BAG [-o REPORT]                     grade a recorded bag again

The team launches the stack and drives the lap the standard Autoware way (initial
pose, goal, Auto in RViz). The grader runs beside it: started after the launch, it
records the run topics into ~/lab3/bags/team<N>_run_<date>_<time>; Ctrl-C after the
lap stops the recording and grades the bag. Grading reads nothing but the bag, so
`lab3_grade BAG` on another machine gives the same result.

What the bag supplies:

- the loop: the lanelet2 file the stack loaded, carried in /lab3/run_meta
  (lab3_run_meta, latched and repeated);
- the car's path: /localization/kinematic_state, Autoware's own pose estimate;
- who drove: /api/operation_mode/state.

The lap. The TA places the car on the start mark before the run. The map's origin is
wherever the car stood when the team's mapping bag began, so the mark is not at the
origin and no coordinate of it is assumed: the start is where the car stands when it
first moves under autonomous control, and the lap ends where it last comes to rest.

| Measure | How |
|---|---|
| travel time | first autonomous motion to the final stop |
| travel distance | the path's length over that window |
| every segment | each lanelet of the loop holds at least one pose of the path |
| end gap | the straight-line distance from the start to where the car came to rest; an |
|  | overshoot past the start is fine. Added to the travel distance for scoring |
| one lap | progress round the loop (unwrapped arc length) reaches at least END_BEHIND short |
|  | of a full lap: the car came round to the start (or past it) |
| autonomous | the distance driven in any other operation mode (a transmitter takeover |
|  | leaves no trace in the bag: the TA notes it) |

The marks from these are the TA's to set; the report gives the measurements and
which rules the lap met.
"""

import argparse
import math
import json
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np
import yaml

from autosdv_lab3 import route

ODOM = '/localization/kinematic_state'
OP_MODE = '/api/operation_mode/state'
RUN_META = '/lab3/run_meta'
NVTL = '/localization/pose_estimator/nearest_voxel_transformation_likelihood'

AUTONOMOUS = 2           # autoware_adapi_v1_msgs/OperationModeState.AUTONOMOUS
MOVING = 0.05            # m/s: faster than this the car is moving
REST_S = 1.0             # s below MOVING that count as having stopped
END_BEHIND = 2.0         # m: a lap reaches at least this close behind the start; an overshoot is fine
OFF_LANE_TOL = 0.15      # m: a pose this close outside a lanelet still counts as on it
REPORT = 'grade.yaml'
VELOCITY = '/vehicle/status/velocity_status'
NDT_INPUT = '/localization/util/downsample/pointcloud'

# ---- localization and speed measures (the rubric's thresholds are set after the TA dry run) ----
NVTL_GATE = 1.3          # the Lab 3 indoor NDT gate (ndt_indoor_robin_w.param.yaml)
NVTL_BIN = 0.5           # s: the lap in bins; a bin is localized when NDT scored >= the gate
SPEED_MAX_KMH = 10.0     # above this for SPEEDING_HOLD_S, the lap does not count as complete
SPEEDING_HOLD_S = 0.3    # one bad hall-sensor edge must not cancel a lap

RUN_TOPICS_FILE = 'run_topics.txt'


# ------------------------------------------------------------------ the bag

@dataclass
class Run:
    odom: np.ndarray = field(default_factory=lambda: np.zeros((0, 4)))   # t x y v
    op: List[Tuple[float, int]] = field(default_factory=list)
    run_meta: dict = field(default_factory=dict)
    nvtl: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    speed: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))   # t, measured m/s
    lidar_topic: Optional[str] = None
    start: float = 0.0
    end: float = 0.0


def read_run(path: str) -> Run:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
    types = {t: get_message(topics[t]) for t in (ODOM, OP_MODE, RUN_META, NVTL, VELOCITY)
             if t in topics}
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(types)))
    r = Run()
    # NDT's own input (deskewed, cropped, downsampled, in base_link) when recorded: the scan
    # NDT matched, and small; else the raw LiDAR
    r.lidar_topic = next((t for t in (NDT_INPUT, '/sensing/lidar/iv_points',
                                      '/sensing/lidar/velodyne_points') if t in topics), None)
    odom, nvtl, speed, first = [], [], [], None
    while reader.has_next():
        topic, raw, stamp = reader.read_next()
        t = stamp * 1e-9
        first = t if first is None else first
        r.end = t
        if topic not in types:
            continue
        m = deserialize_message(raw, types[topic])
        if topic == ODOM:
            p = m.pose.pose.position
            odom.append((t, p.x, p.y, m.twist.twist.linear.x))
        elif topic == OP_MODE:
            r.op.append((t, m.mode))
        elif topic == RUN_META:
            r.run_meta = json.loads(m.data)
        elif topic == NVTL:
            nvtl.append((t, m.data))
        elif topic == VELOCITY:
            speed.append((t, m.longitudinal_velocity))
    r.start = first or 0.0
    r.odom = np.array(odom, float).reshape(-1, 4)
    r.nvtl = np.array(nvtl, float).reshape(-1, 2)
    r.speed = np.array(speed, float).reshape(-1, 2)
    return r


# ------------------------------------------------------------------ the measures

def mode_at(op: List[Tuple[float, int]], t: float) -> Optional[int]:
    mode = None
    for ts, m in op:
        if ts > t:
            break
        mode = m
    return mode


def lap_window(odom: np.ndarray, op) -> Tuple[Optional[int], Optional[int], str]:
    """(first index, last index) of the lap in odom, or (None, None, why)."""
    moving = np.abs(odom[:, 3]) > MOVING if len(odom) else np.zeros(0, bool)
    auto = np.array([mode_at(op, t) == AUTONOMOUS for t in odom[:, 0]], bool)
    go = np.flatnonzero(moving & auto)
    if not len(go):
        return None, None, ('the car never moved under autonomous control'
                            if moving.any() else 'the car never moved')
    i0 = int(go[0])
    last = int(np.flatnonzero(moving)[-1])
    # the stop after the last motion: the first sample that has been still for REST_S
    t_last = odom[last, 0]
    rest = np.flatnonzero((odom[:, 0] >= t_last + REST_S) & ~moving)
    if not len(rest):
        return i0, None, 'the car had not come to rest when the recording stopped'
    i1 = last + 1 if last + 1 < len(odom) else last
    return i0, i1, ''


def path_length(xy: np.ndarray) -> float:
    return float(np.sum(np.hypot(*np.diff(xy, axis=0).T))) if len(xy) > 1 else 0.0


def progress(loop: route.Loop, xy: np.ndarray) -> np.ndarray:
    """Arc length round the loop of each pose, unwrapped: a lap adds loop.length."""
    s = np.array([loop.project(x, y) for x, y in xy])
    d = np.diff(s)
    half = loop.length / 2
    d[d > half] -= loop.length
    d[d < -half] += loop.length
    return np.concatenate([[0.0], np.cumsum(d)])


def lanelet_of(loop: route.Loop, x: float, y: float) -> Optional[int]:
    for lid, ll in loop.lanelets.items():
        if route.point_in_polygon(x, y, ll.polygon):
            return lid
    best, lid_best = None, None
    for lid, ll in loop.lanelets.items():
        d = route.point_polyline_distance(np.array([[x, y]]),
                                          np.vstack([ll.polygon, ll.polygon[:1]]))[0]
        if best is None or d < best:
            best, lid_best = d, lid
    return lid_best if best is not None and best <= OFF_LANE_TOL else None


def grade_run(r: Run) -> dict:
    rep = {'ok': False, 'reasons': [], 'map_hash': r.run_meta.get('map_hash'),
           'vehicle_id': r.run_meta.get('vehicle_id'), 'autosdv': r.run_meta.get('autosdv')}
    text = r.run_meta.get('lanelet2_map')
    if not text:
        rep['reasons'].append('the bag has no /lab3/run_meta with the lanelet2 map: was the '
                              'stack launched with lab3_run.launch.xml before the grader?')
        return rep
    try:
        loop = route.loop_from_lanelets(route.lanelets_from_text(text))
    except route.MapError as e:
        rep['reasons'].append(f'the lanelet2 map in the bag is not a loop: {e}')
        return rep
    rep['loop_length_m'] = round(loop.length, 2)
    rep['lanelets'] = len(loop.order)
    if not len(r.odom):
        rep['reasons'].append(f'the bag has no {ODOM}')
        return rep
    i0, i1, why = lap_window(r.odom, r.op)
    if i0 is None or i1 is None:
        rep['reasons'].append(why)
        return rep
    o = r.odom[i0:i1 + 1]
    xy = o[:, 1:3]
    t0, t1 = float(o[0, 0]), float(o[-1, 0])
    rep['travel_time_s'] = round(t1 - t0, 2)
    rep['travel_distance_m'] = round(path_length(xy), 2)
    rep['start'] = [round(float(v), 3) for v in xy[0]]
    rep['end'] = [round(float(v), 3) for v in xy[-1]]

    prog = progress(loop, xy)
    behind = loop.length - prog[-1]          # > 0: short of a full lap, i.e. behind
    rep['end_behind_start_m'] = round(float(behind), 2)   # < 0: past the start
    gap = float(np.hypot(*(xy[-1] - xy[0])))
    rep['end_gap_m'] = round(gap, 2)
    rep['distance_plus_gap_m'] = round(rep['travel_distance_m'] + gap, 2)
    rep['laps'] = round(float(prog[-1] / loop.length), 3)

    seen = {lanelet_of(loop, x, y) for x, y in xy}
    missed = [lid for lid in loop.order if lid not in seen]
    rep['segments_visited'] = f'{len(loop.order) - len(missed)} of {len(loop.order)}'
    rep['segments_missed'] = missed

    steps = np.hypot(*np.diff(xy, axis=0).T)
    manual = [d for d, t in zip(steps, o[1:, 0]) if mode_at(r.op, t) != AUTONOMOUS]
    rep['distance_not_autonomous_m'] = round(float(sum(manual)), 2)
    rep['window'] = [t0, t1]
    rep['localized_share'] = localized_share(r.nvtl, t0, t1)
    if len(r.nvtl):
        sel = (r.nvtl[:, 0] >= t0) & (r.nvtl[:, 0] <= t1)
        if sel.any():
            rep['nvtl_min'] = round(float(r.nvtl[sel, 1].min()), 3)
            rep['nvtl_median'] = round(float(np.median(r.nvtl[sel, 1])), 3)
    sp = speeding(r.speed, t0, t1)
    rep['max_speed_kmh'] = sp['max_kmh']
    rep['speeding'] = sp['speeding']

    checks = {
        'every segment': (not missed, f'lanelets {missed} not driven'),
        'one lap': (behind <= END_BEHIND,
                    f'the car stopped {behind:.2f} m behind the start: the lap is not complete '
                    f'(it must come within {END_BEHIND:g} m of the start, or pass it)'),
        'speed limit': (not rep.get('speeding'),
                        f'the measured speed exceeded {SPEED_MAX_KMH:g} km/h for '
                        f'{SPEEDING_HOLD_S:g} s or longer (max {rep.get("max_speed_kmh")} km/h)'),
        'autonomous': (rep['distance_not_autonomous_m'] < 0.1,
                       f'{rep["distance_not_autonomous_m"]:.2f} m driven outside autonomous mode'),
    }
    rep['checks'] = {k: bool(ok) for k, (ok, _) in checks.items()}
    rep['reasons'] = [msg for ok, msg in checks.values() if not ok]
    rep['ok'] = all(rep['checks'].values())
    return rep


def localized_share(nvtl: np.ndarray, t0: float, t1: float) -> float:
    """Share of NVTL_BIN bins of [t0, t1] in which NDT scored at or above the gate. A bin
    with no NDT output at all counts as not localized."""
    n = max(1, int(math.ceil((t1 - t0) / NVTL_BIN)))
    ok = np.zeros(n, bool)
    if len(nvtl):
        sel = (nvtl[:, 0] >= t0) & (nvtl[:, 0] < t0 + n * NVTL_BIN) & (nvtl[:, 1] >= NVTL_GATE)
        ok[((nvtl[sel, 0] - t0) // NVTL_BIN).astype(int)] = True
    return round(float(ok.mean()), 4)


def speeding(speed: np.ndarray, t0: float, t1: float) -> dict:
    """The measured speed's maximum in [t0, t1] and whether it stayed above SPEED_MAX_KMH
    for SPEEDING_HOLD_S or longer."""
    sel = (speed[:, 0] >= t0) & (speed[:, 0] <= t1) if len(speed) else np.zeros(0, bool)
    s = speed[sel] if len(speed) else speed
    if not len(s):
        return {'max_kmh': None, 'speeding': False}
    over = np.abs(s[:, 1]) * 3.6 > SPEED_MAX_KMH
    run_start, longest = None, 0.0
    for (t, _), o in zip(s, over):
        if o and run_start is None:
            run_start = t
        if not o:
            run_start = None
        if run_start is not None:
            longest = max(longest, t - run_start)
    return {'max_kmh': round(float(np.abs(s[:, 1]).max() * 3.6), 2),
            'speeding': bool(over.any() and longest >= SPEEDING_HOLD_S)}


def print_report(rep: dict, bag: str):
    print(f'\nLab 3 lap: {bag}')
    for key in ('loop_length_m', 'travel_time_s', 'travel_distance_m', 'end_gap_m',
                'distance_plus_gap_m', 'end_behind_start_m',
                'laps', 'segments_visited', 'distance_not_autonomous_m', 'localized_share',
                'nvtl_median', 'nvtl_min', 'max_speed_kmh', 'speeding'):
        if key in rep:
            print(f'  {key:28s} {rep[key]}')
    for k, ok in rep.get('checks', {}).items():
        print(f'  {"PASS" if ok else "FAIL"}  {k}')
    for why in rep['reasons']:
        print(f'  - {why}')
    al = rep.get('alignment')
    if al:
        print(f'  {"scan_to_map":28s} {al}')
    print('  LAP ' + ('ACCEPTED' if rep['ok'] else 'NOT ACCEPTED'))



def grade_bag(bag: str, out: Optional[str] = None) -> int:
    r = read_run(bag)
    rep = grade_run(r)
    rep['bag'] = os.path.abspath(bag)
    if 'window' in rep and r.lidar_topic:
        from autosdv_lab3 import alignment
        rep['alignment'] = alignment.read_alignment(bag, r.lidar_topic, *rep['window'], ODOM)
    print_report(rep, bag)
    out = out or os.path.join(bag, REPORT)
    rep.pop('lanelet2_map', None)
    with open(out, 'w') as f:
        yaml.safe_dump(rep, f, sort_keys=False)
    print(f'  report: {out}')
    return 0 if rep['ok'] else 1


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog='lab3_grade', description='Grade a Lab 3 run bag.')
    ap.add_argument('bag')
    ap.add_argument('-o', '--out', help=f'report path (default BAG/{REPORT})')
    args = ap.parse_args(argv)
    if not os.path.isdir(args.bag):
        print(f'lab3_grade: {args.bag} is not a bag folder', file=sys.stderr)
        return 2
    return grade_bag(args.bag, args.out)


# ------------------------------------------------------------------ the recorder

def run_topics() -> List[str]:
    from ament_index_python.packages import get_package_share_directory
    path = os.path.join(get_package_share_directory('autosdv_lab3'), 'config', 'record',
                        RUN_TOPICS_FILE)
    with open(path) as f:
        return [line.strip() for line in f if line.strip()]


def _strip_ros_args(argv: List[str]) -> List[str]:
    return argv[:argv.index('--ros-args')] if '--ros-args' in argv else argv


def grader_main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog='lab3_grader',
                                 description='Record a Lab 3 graded run; Ctrl-C grades it.')
    ap.add_argument('--team', required=True, help='team number')
    ap.add_argument('--out-dir', default=os.path.expanduser('~/lab3/bags'))
    # launched as a node: ignore the --ros-args launch appends
    args, _ = ap.parse_known_args(_strip_ros_args(sys.argv[1:] if argv is None else argv))
    out = os.path.join(args.out_dir, f'team{args.team}_run_{time.strftime("%Y%m%d_%H%M%S")}')
    os.makedirs(args.out_dir, exist_ok=True)
    cmd = ['ros2', 'bag', 'record', '-o', out, *run_topics()]
    print(f'lab3_grader: recording -> {out}\n'
          'Drive the lap in RViz (2D Goal Pose, Auto). Press Ctrl-C once the car has stopped.')
    # its own session: Ctrl-C reaches this process only, which then stops the recorder
    # and grades
    proc = subprocess.Popen(cmd, start_new_session=True)
    stop = {'now': False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(now=True))
    signal.signal(signal.SIGTERM, lambda *_: stop.update(now=True))
    while proc.poll() is None and not stop['now']:
        time.sleep(0.2)
    if proc.poll() is None:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(30)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
    if not os.path.isdir(out):
        print('lab3_grader: nothing was recorded', file=sys.stderr)
        return 1
    grade_bag(out)
    return 0   # the verdict is printed; a non-zero exit only makes launch report a crash


if __name__ == '__main__':
    sys.exit(main())
