"""Per-lanelet speed limits: the check, the in-place edit, and the speed-blind map hash."""

import os
import re

import pytest

from autosdv_lab3 import fixtures, lanelet_check, lanelet_speed, mapdir
from autosdv_lab3.route import read_lanelets


@pytest.fixture
def osm(tmp_path):
    p = tmp_path / 'lanelet2_map.osm'
    p.write_text(fixtures.lanelet_osm())
    return p


def speeds(path):
    return {lid: ll.tags.get('speed_limit') for lid, ll in read_lanelets(str(path)).items()}


def test_fixture_loop_passes_the_check(osm):
    ok, value, reason = lanelet_speed.check(read_lanelets(str(osm)))
    assert ok, reason
    assert value == '10-10 km/h'


@pytest.mark.parametrize('bad, why', [('12', 'above the 10 km/h maximum'), ('0', 'not above 0'),
                                      ('fast', "'fast'")])
def test_check_refuses_out_of_range(osm, bad, why):
    text = osm.read_text()
    first = re.search(r'<tag k="speed_limit" v="[^"]*"/>', text)
    osm.write_text(text[:first.start()] + f'<tag k="speed_limit" v="{bad}"/>' + text[first.end():])
    ok, value, reason = lanelet_speed.check(read_lanelets(str(osm)))
    assert not ok and value.startswith('1 of')
    assert why in reason


def test_check_refuses_a_missing_tag(osm):
    text = osm.read_text()
    first = re.search(r'\s*<tag k="speed_limit" v="[^"]*"/>', text)
    osm.write_text(text[:first.start()] + text[first.end():])
    ok, _, reason = lanelet_speed.check(read_lanelets(str(osm)))
    assert not ok and 'no speed_limit' in reason


def test_set_speeds_changes_only_the_tags(osm):
    text = osm.read_text()
    lids = sorted(speeds(osm))
    new = lanelet_speed.set_speeds(text, {lids[0]: 4.0, lids[1]: 6.5})
    assert mapdir.normalized_osm(new) == mapdir.normalized_osm(text)
    osm.write_text(new)
    s = speeds(osm)
    assert s[lids[0]] == '4' and s[lids[1]] == '6.5'
    assert all(v == '10' for k, v in s.items() if k not in lids[:2])


def test_set_speeds_adds_a_missing_tag(osm):
    text = osm.read_text()
    lid = sorted(speeds(osm))[0]
    start, end = lanelet_speed._relation_span(text, lid)
    block = re.sub(r'\s*<tag k="speed_limit" v="[^"]*"/>', '', text[start:end])
    stripped = text[:start] + block + text[end:]
    assert speeds_of_text(stripped, osm)[lid] is None
    new = lanelet_speed.set_speeds(stripped, {lid: 3})
    assert speeds_of_text(new, osm)[lid] == '3'
    assert mapdir.normalized_osm(new) == mapdir.normalized_osm(stripped)


def speeds_of_text(text, path):
    path.write_text(text)
    return speeds(path)


def test_map_hash_ignores_speeds_but_not_geometry(tmp_path):
    d = fixtures.write_map(str(tmp_path / 'map'))
    before = mapdir.folder_hash(d)
    p = os.path.join(d, mapdir.LANELET)
    text = open(p).read()
    lid = sorted(speeds(p))[0]
    open(p, 'w').write(lanelet_speed.set_speeds(text, {lid: 3}))
    assert mapdir.folder_hash(d) == before
    open(p, 'w').write(text.replace('local_x" v="', 'local_x" v="1', 1))
    assert mapdir.folder_hash(d) != before


def test_cli_sets_corners_and_lists(tmp_path, capsys):
    d = fixtures.write_map(str(tmp_path / 'map'))
    assert lanelet_speed.main([d]) == 0
    assert 'speed_limit' in capsys.readouterr().out
    assert lanelet_speed.main([d, '--all', '5']) == 0
    assert set(speeds(os.path.join(d, mapdir.LANELET)).values()) == {'5'}
    assert lanelet_speed.main([d, '--all', '11']) == 2
    assert set(speeds(os.path.join(d, mapdir.LANELET)).values()) == {'5'}


def test_plan_changes_corners_vs_straights():
    class L:
        def __init__(self, turn):
            self.tags = {'turn_direction': turn}
    ls = {1: L('straight'), 2: L('right'), 3: L('straight'), 4: L('left')}
    assert lanelet_speed.plan_changes(ls, corners=3, straights=8) == {1: 8, 2: 3, 3: 8, 4: 3}
    assert lanelet_speed.plan_changes(ls, every=5, single=[(2, 2)]) == {1: 5, 2: 2, 3: 5, 4: 5}
    with pytest.raises(KeyError):
        lanelet_speed.plan_changes(ls, single=[(9, 2)])


def test_check_lanelet_reports_the_speed_row(tmp_path):
    d = fixtures.write_map(str(tmp_path / 'map'))
    rows = {r.requirement: r for r in lanelet_check.static_checks(d)}
    assert rows['speed limits'].ok


def test_run_meta_records_the_segment_limits(tmp_path):
    from autosdv_lab3 import run_meta
    d = fixtures.write_map(str(tmp_path / 'map'))
    meta = run_meta.build(d, 'car_b', 0.0)
    assert meta['speed_limits'] and set(meta['speed_limits'].values()) == {'10'}
