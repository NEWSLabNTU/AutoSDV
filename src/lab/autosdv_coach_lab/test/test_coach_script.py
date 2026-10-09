import math

import pytest

from autosdv_coach_lab.coach_script import CoachScript, DetectorModel, world_to_base_link


def test_piecewise_distance_and_hold():
    s = CoachScript(initial_distance=6.0, durations=[10.0, 25.0], speeds=[0.0, 0.8])
    assert s.distance(0.0) == 6.0
    assert s.distance(10.0) == 6.0
    assert s.distance(20.0) == pytest.approx(14.0)
    assert s.distance(35.0) == pytest.approx(26.0)
    assert s.distance(100.0) == pytest.approx(26.0)
    assert s.speed(15.0) == 0.8 and s.speed(40.0) == 0.0


def test_board_world_follows_anchor_heading_and_offset():
    s = CoachScript(initial_distance=5.0, lateral_offset=1.0, durations=[], speeds=[])
    x, y = s.board_world(0.0, (10.0, 20.0, math.pi / 2))
    assert (x, y) == pytest.approx((9.0, 25.0))


def test_world_to_base_link():
    assert world_to_base_link(1.0, 3.0, 1.0, 1.0, math.pi / 2) == pytest.approx((2.0, 0.0))


def test_mismatched_script_rejected():
    with pytest.raises(ValueError):
        CoachScript(durations=[1.0], speeds=[])


def test_detector_windows_fov_range_noise():
    d = DetectorModel(noise_std=0.0, dropout_windows=[2.0, 3.0], max_range=10.0,
                      fov=math.radians(90.0), seed=1)
    assert d.sample(1.0, 5.0, 0.0) == (5.0, 0.0)
    assert d.sample(2.5, 5.0, 0.0) is None
    assert d.sample(3.0, 5.0, 0.0) is not None
    assert d.sample(1.0, 11.0, 0.0) is None
    assert d.sample(1.0, 1.0, 2.0) is None          # outside +-45 deg
    assert DetectorModel(dropout_windows=[-1.0, -1.0]).in_window(0.0) is False


def test_dropout_probability_is_seeded():
    a = DetectorModel(dropout_prob=0.5, seed=3)
    b = DetectorModel(dropout_prob=0.5, seed=3)
    ra = [a.sample(0.0, 5.0, 0.0) is None for _ in range(100)]
    rb = [b.sample(0.0, 5.0, 0.0) is None for _ in range(100)]
    assert ra == rb and 20 < sum(ra) < 80


def test_rate_limited_board_starts_and_stops_smoothly():
    s = CoachScript(initial_distance=6.0, durations=[2.0, 10.0], speeds=[0.0, 1.0], accel=1.0)
    assert s.speed(2.5) == pytest.approx(0.5, abs=0.02)
    assert s.speed(5.0) == pytest.approx(1.0)
    assert s.speed(12.5) == pytest.approx(0.5, abs=0.02)   # decelerating after the segment
    assert s.speed(20.0) == 0.0
    # 1 s ramp up, 9 s at speed, 1 s ramp down: 0.5 + 9 + 0.5 m
    assert s.distance(30.0) == pytest.approx(16.0, abs=0.02)
    assert s.distance(1.0) == pytest.approx(6.0)
