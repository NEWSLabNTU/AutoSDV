"""The Lab 3 speed limit: 3 km/h by default, at most 10, raised only on request."""

import pytest

from autosdv_lab3 import fixtures, speed_limit


def test_default_is_3_kmh_below_the_lanelet_limit():
    assert speed_limit.DEFAULT_KMH == 3.0
    assert speed_limit.MAX_KMH == 10.0
    assert speed_limit.DEFAULT_KMH < fixtures.SPEED_LIMIT_KMH == speed_limit.MAX_KMH


@pytest.mark.parametrize('kmh', [0.5, 3, 3.0, 5, 10.0])
def test_accepts_limits_up_to_10(kmh):
    assert speed_limit.refusal(kmh) is None


@pytest.mark.parametrize('kmh, why', [(0, 'not above 0'), (-1.0, 'not above 0'),
                                      (10.5, 'above the 10 km/h maximum'),
                                      (36, 'above the 10 km/h maximum'),
                                      ('5', 'not a number'), (True, 'not a number')])
def test_refuses_out_of_range(kmh, why):
    assert why in speed_limit.refusal(kmh)


def test_kmh_to_mps():
    assert speed_limit.kmh_to_mps(3.6) == pytest.approx(1.0)
    assert speed_limit.kmh_to_mps(3.0) == pytest.approx(0.8333, abs=1e-4)


def test_message_carries_the_limit_in_mps():
    pytest.importorskip('autoware_internal_planning_msgs')
    msg = speed_limit.velocity_limit(3.0)
    assert msg.max_velocity == pytest.approx(3.0 / 3.6)
    assert msg.use_constraints is False
    assert msg.sender == 'lab3_speed_limit'


def test_cli_refuses_before_touching_ros(capsys):
    assert speed_limit.cli(['12']) == 2
    assert 'above the 10 km/h maximum' in capsys.readouterr().err


@pytest.mark.parametrize('mps, kmh', [(5 / 3.6, 5.0), (1.0, 3.6), (20.0, 10.0), (0.0, None),
                                      (-1.0, None)])
def test_adopts_another_senders_limit_clamped(mps, kmh):
    assert speed_limit.adopted_kmh(mps) == kmh


def test_first_lap_holds_the_default():
    assert speed_limit.lap_refusal(5, lap1_done=False) == speed_limit.LAP1_RULE
    assert speed_limit.lap_refusal(3, lap1_done=False) is None      # the default itself
    assert speed_limit.lap_refusal(2, lap1_done=False) is None      # lowering is always allowed
    assert speed_limit.lap_refusal(10, lap1_done=True) is None


@pytest.mark.parametrize('event, done', [
    ({'event': 'arrived', 'lap': 1, 'leg': 2, 'legs': 2}, True),
    ({'event': 'arrived', 'lap': 1, 'leg': 1, 'legs': 2}, False),   # half a lap
    ({'event': 'leg_set', 'lap': 1, 'leg': 2, 'legs': 2}, False),
    ({'event': 'arrived', 'lap': 2, 'leg': 2, 'legs': 2}, False),
    ({}, False)])
def test_lap1_completion(event, done):
    assert speed_limit.completes_lap1(event) is done


def test_events_topic_matches_the_route_script():
    from autosdv_lab3 import route
    assert speed_limit.EVENTS_TOPIC == route.EVENTS_TOPIC
