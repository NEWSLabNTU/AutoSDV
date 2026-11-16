"""The lanelet loop check, its start hint, the routes, and (opt-in) the simulator lap.

The simulator lap launches Autoware: run it with LAB3_SIM=1 and a private ROS_DOMAIN_ID,
    LAB3_SIM=1 ROS_DOMAIN_ID=87 python3 -m pytest test/test_lanelet_check.py -k sim
"""

import os
import re

import pytest

from autosdv_lab3 import fixtures, lanelet_check, mapdir
from autosdv_lab3.route import (BACKWARD_CLEAR, MapError, find_chain, load_loop,
                                loop_route, read_lanelets, single_goal_route)
from fixtures import make_map


def rows(map_dir):
    return {r.requirement: r for r in lanelet_check.static_checks(map_dir)}


@pytest.fixture(scope='module')
def maps(tmp_path_factory):
    base = tmp_path_factory.mktemp('lab3_maps')
    return {v: make_map(base, v) for v in fixtures.VARIANTS}


def test_exactly_four_requirements(maps):
    assert list(rows(maps['good'])) == ['loop', 'clockwise', 'speed limits', 'start boundary']


def test_good_passes_every_check(maps):
    r = rows(maps['good'])
    assert all(x.ok for x in r.values()), lanelet_check.table(list(r.values()))


@pytest.mark.parametrize('variant, failing', [('ccw', 'clockwise'), ('gap', 'loop')])
def test_each_variant_fails_its_requirement_only(maps, variant, failing):
    r = rows(maps[variant])
    assert r[failing].ok is False, lanelet_check.table(list(r.values()))
    assert r[failing].reason
    # reversing the loop moves the origin relative to the boundaries, so the start
    # boundary of the counter-clockwise variant is not this test's concern
    others = [x for k, x in r.items() if k != failing and k != 'start boundary']
    # after a broken chain the direction cannot be measured (None), but nothing else fails
    assert all(x.ok in (True, None) for x in others), lanelet_check.table(list(r.values()))


@pytest.mark.parametrize('bad', ['12', '0', None])
def test_speed_limit_row(tmp_path, bad):
    d = make_map(tmp_path, 'good')
    p = os.path.join(d, mapdir.LANELET)
    text = open(p).read()
    tag = re.search(r'\s*<tag k="speed_limit" v="[^"]*"/>', text)
    new = '' if bad is None else f'\n    <tag k="speed_limit" v="{bad}"/>'
    open(p, 'w').write(text[:tag.start()] + new + text[tag.end():])
    r = rows(d)
    assert r['speed limits'].ok is False and r['loop'].ok and r['clockwise'].ok


def test_width_corners_and_clearance_are_not_checked(tmp_path):
    # a 0.4 m lane with 0.4 m corners used to fail; it is the team's choice now
    d = os.path.join(str(tmp_path), 'narrow')
    fixtures.write_map(d)
    with open(os.path.join(d, mapdir.LANELET), 'w') as f:
        f.write(fixtures.lanelet_osm(width=0.4, radius=0.4))
    assert all(x.ok for x in rows(d).values())


def test_main_records_and_exits(maps):
    assert lanelet_check.main([maps['good'], '--no-sim']) == 0
    meta = mapdir.load_meta(maps['good'])['lanelet_check']
    assert meta['passed'] is True and meta['sim'] is False
    assert meta['start_hint']['near'] is True
    assert lanelet_check.main([maps['ccw'], '--no-sim']) == 1
    assert mapdir.load_meta(maps['ccw'])['lanelet_check']['passed'] is False


def test_start_boundary_too_far_fails(maps, capsys):
    r = rows(maps['far'])
    assert r['start boundary'].ok is False and '2.00 m behind' in r['start boundary'].value
    assert all(x.ok for k, x in r.items() if k != 'start boundary')
    assert lanelet_check.main([maps['far'], '--no-sim']) == 1
    assert "the start's own" in capsys.readouterr().out


@pytest.mark.parametrize('into, ok', [(0.2, False), (0.5, True), (0.75, True), (1.0, False)])
def test_start_boundary_window(into, ok):
    from autosdv_lab3.route import lanelets_from_text, loop_from_lanelets
    loop = loop_from_lanelets(lanelets_from_text(fixtures.lanelet_osm()))
    first = loop.order[0]
    x, y, _ = loop.pose_at(loop.starts[first] + into)
    assert lanelet_check.boundary_check(loop, (x, y)).ok is ok


def test_start_hint_numbers(maps):
    near = lanelet_check.start_hint(load_loop(maps['good']))
    assert near.behind == pytest.approx(fixtures.ORIGIN_INTO, abs=0.03)
    assert near.near and near.goal_in_previous and near.off_loop < 0.02
    assert near.goal_lanelet == near.previous
    far = lanelet_check.start_hint(load_loop(maps['far']))
    assert far.behind == pytest.approx(2.0, abs=0.03)
    assert not far.near and not far.goal_in_previous
    # a goal far enough back reaches the previous lanelet
    assert lanelet_check.start_hint(load_loop(maps['far']), goal_behind=2.5).goal_in_previous


def test_sim_lap_refused_when_goal_and_start_share_a_lanelet(maps, capsys, monkeypatch):
    from autosdv_lab3 import sim_lap

    def no_launch(*a, **k):
        raise AssertionError('the lap must be refused before any launch')
    monkeypatch.setattr(sim_lap, 'run_lap', no_launch)
    assert lanelet_check.main([maps['far']]) == 1
    assert 'not run' in capsys.readouterr().out


def test_chain_order_and_gap(maps):
    lanelets = read_lanelets(os.path.join(maps['good'], mapdir.LANELET))
    order, problem = find_chain(lanelets)
    assert problem == '' and len(order) == 4 and order[0] == min(lanelets)
    with pytest.raises(MapError, match='no successor'):
        load_loop(maps['gap'])


def _forward(loop, a, b):
    return (b - a) % loop.length


def test_single_goal_route(maps):
    loop = load_loop(maps['good'])
    s0 = loop.project(0.0, 0.0)
    route = single_goal_route(loop, s0, 0.7)
    assert len(route.legs) == 1 and route.waypoints == [] and route.legs[0].waypoints == []
    assert _forward(loop, s0, route.legs[0].s_goal) == pytest.approx(loop.length - 0.7, abs=1e-6)
    assert abs(route.start[0]) < 0.02 and abs(route.start[1]) < 0.02
    with pytest.raises(MapError, match="start's own lanelet"):
        single_goal_route(loop, s0 + 1.0, 0.7)


@pytest.mark.parametrize('variant', ['good', 'ccw'])
def test_scripted_route_goes_all_the_way_round(maps, variant):
    loop = load_loop(maps[variant])
    route = loop_route(loop)
    s0 = route.s_start
    lap = _forward(loop, s0, route.legs[-1].s_goal)
    assert loop.length - 3.0 < lap < loop.length - 2.0
    marks = []
    for leg in route.legs:
        marks += leg.s_waypoints + [leg.s_goal]
    ahead = [_forward(loop, s0, s) for s in marks]
    assert ahead == sorted(ahead) and len(set(ahead)) == len(ahead)
    starts = [s0] + [leg.s_goal for leg in route.legs[:-1]]
    for a, leg in zip(starts, route.legs):
        assert loop.lane_at(a) != loop.lane_at(leg.s_goal)
        assert loop.length - _forward(loop, a, leg.s_goal) > BACKWARD_CLEAR


# ------------------------------------------------------------------ the simulator lap

sim = pytest.mark.skipif(os.environ.get('LAB3_SIM') != '1' or os.environ.get('ROS_DOMAIN_ID', '0') == '0',
                         reason='launches Autoware: set LAB3_SIM=1 and a private ROS_DOMAIN_ID')


@sim
@pytest.mark.xfail(strict=False, reason='the 21 m fixture loop: the one-goal lap did not start '
                   'for any of 4 start/goal pairs (2026-11-16); it drives on the 37 m and 98 m loops')
def test_sim_lap_good(maps):
    from autosdv_lab3 import sim_lap
    res = sim_lap.run_lap(maps['good'])
    print(res.as_dict())
    assert res.ok, res.as_dict()
