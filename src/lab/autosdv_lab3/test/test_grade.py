"""The week-2 lap grader on synthetic runs: the loop from run_meta, the path from odometry."""

import numpy as np
import pytest

from autosdv_lab3 import fixtures, grade, route
from autosdv_lab3.grade import AUTONOMOUS, Run

STOP = 1


def loop():
    return route.loop_from_lanelets(route.lanelets_from_text(fixtures.lanelet_osm()))


def drive(s_from: float, s_to: float, lp, v=0.8, dt=0.1, rest=3.0, mode=AUTONOMOUS):
    """A run that stands, drives the centreline from arc length s_from to s_to (unwrapped),
    then stands for `rest` s."""
    rows, t = [], 0.0
    x, y, _ = lp.pose_at(s_from)
    for _ in range(20):
        rows.append((t, x, y, 0.0))
        t += dt
    s = s_from
    while s < s_to:
        s = min(s + v * dt, s_to)
        x, y, _ = lp.pose_at(s)
        rows.append((t, x, y, v))
        t += dt
    for _ in range(int(rest / dt)):
        rows.append((t, x, y, 0.0))
        t += dt
    r = Run(odom=np.array(rows, float), op=[(0.0, STOP), (1.0, mode)],
            run_meta={'lanelet2_map': fixtures.lanelet_osm(), 'map_hash': 'h'})
    r.start, r.end = 0.0, t
    return r


def test_full_lap_ending_behind_the_start_is_accepted():
    lp = loop()
    rep = grade.grade_run(drive(2.0, 2.0 + lp.length - 0.6, lp))
    assert rep['ok'], rep['reasons']
    assert rep['end_behind_start_m'] == pytest.approx(0.6, abs=0.1)
    assert rep['travel_distance_m'] == pytest.approx(lp.length - 0.6, abs=0.15)
    assert rep['travel_time_s'] == pytest.approx((lp.length - 0.6) / 0.8, abs=0.5)
    assert rep['segments_missed'] == []


def test_stopping_too_far_behind_fails():
    lp = loop()
    rep = grade.grade_run(drive(2.0, 2.0 + lp.length - 2.6, lp))
    assert not rep['ok'] and not rep['checks']['one lap']
    assert 'm behind the start' in ' '.join(rep['reasons'])


def test_stopping_past_the_start_fails():
    lp = loop()
    rep = grade.grade_run(drive(2.0, 2.0 + lp.length + 0.4, lp))
    assert not rep['checks']['one lap']
    assert 'past the start' in ' '.join(rep['reasons'])


def test_a_short_drive_misses_segments():
    lp = loop()
    rep = grade.grade_run(drive(2.0, 12.0, lp))
    assert not rep['checks']['every segment'] and rep['segments_missed']


def test_manual_driving_is_flagged():
    lp = loop()
    r = drive(2.0, 2.0 + lp.length - 0.5, lp)
    r.op.append((20.0, 3))           # LOCAL from 20 s
    rep = grade.grade_run(r)
    assert not rep['checks']['autonomous']


def test_no_run_meta_is_reported():
    lp = loop()
    r = drive(2.0, 20.0, lp)
    r.run_meta = {}
    rep = grade.grade_run(r)
    assert not rep['ok'] and 'run_meta' in rep['reasons'][0]


def test_never_engaged():
    lp = loop()
    rep = grade.grade_run(drive(2.0, 20.0, lp, mode=STOP))
    assert not rep['ok'] and 'autonomous' in rep['reasons'][0]


def test_still_moving_when_the_recording_stopped():
    lp = loop()
    rep = grade.grade_run(drive(2.0, 20.0, lp, rest=0.0))
    assert not rep['ok'] and 'come to rest' in rep['reasons'][0]
