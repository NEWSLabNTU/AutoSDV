"""The Lab 3 overall speed limit: 3 km/h from the shipped config file, at most 10."""

from pathlib import Path

import pytest

from autosdv_lab3 import fixtures, speed_limit

SHIPPED = Path(__file__).resolve().parents[1] / 'config' / 'speed_limit.yaml'


def test_shipped_config_is_3_kmh():
    assert speed_limit.read_config(str(SHIPPED)) == (3.0, None)
    assert speed_limit.DEFAULT_KMH == 3.0


def test_default_is_below_the_lanelet_limit():
    assert speed_limit.MAX_KMH == 10.0
    assert speed_limit.DEFAULT_KMH < fixtures.SPEED_LIMIT_KMH == speed_limit.MAX_KMH


@pytest.mark.parametrize('text, kmh', [('max_speed_kmh: 5\n', 5.0), ('max_speed_kmh: 10.0\n', 10.0),
                                       ('max_speed_kmh: 0.5\n', 0.5)])
def test_reads_a_valid_limit(tmp_path, text, kmh):
    p = tmp_path / 'speed_limit.yaml'
    p.write_text(text)
    assert speed_limit.read_config(str(p)) == (kmh, None)


@pytest.mark.parametrize('text, why', [('max_speed_kmh: 12\n', 'above the 10 km/h maximum'),
                                       ('max_speed_kmh: 0\n', 'not above 0'),
                                       ('max_speed_kmh: fast\n', 'not a number'),
                                       ('speed: 5\n', 'no max_speed_kmh'),
                                       ('max_speed_kmh: [5\n', 'cannot read')])
def test_refuses_an_unusable_file(tmp_path, text, why):
    p = tmp_path / 'speed_limit.yaml'
    p.write_text(text)
    kmh, reason = speed_limit.read_config(str(p))
    assert kmh is None and why in reason


def test_missing_file(tmp_path):
    kmh, reason = speed_limit.read_config(str(tmp_path / 'none.yaml'))
    assert kmh is None and 'cannot read' in reason


def test_kmh_to_mps():
    assert speed_limit.kmh_to_mps(3.6) == pytest.approx(1.0)
    assert speed_limit.kmh_to_mps(3.0) == pytest.approx(0.8333, abs=1e-4)


def test_message_carries_the_limit_in_mps():
    pytest.importorskip('autoware_internal_planning_msgs')
    msg = speed_limit.velocity_limit(3.0)
    assert msg.max_velocity == pytest.approx(3.0 / 3.6)
    assert msg.use_constraints is False
    assert msg.sender == 'lab3_speed_limit'


@pytest.mark.parametrize('mps, kmh', [(5 / 3.6, 5.0), (1.0, 3.6), (20.0, 10.0), (0.0, None),
                                      (-1.0, None)])
def test_adopts_another_senders_limit_clamped(mps, kmh):
    assert speed_limit.adopted_kmh(mps) == kmh
