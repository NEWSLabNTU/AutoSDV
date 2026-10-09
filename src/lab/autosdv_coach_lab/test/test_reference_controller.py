import pytest

from autosdv_coach_lab.reference.cruise_controller import CruiseController


class S:
    def __init__(self, valid=True, range=2.0, range_rate=0.0):
        self.valid, self.range, self.range_rate = valid, range, range_rate
        self.bearing, self.age = 0.0, 0.0


def test_at_headway_with_board_matching_speed_holds_speed():
    c = CruiseController({'headway': 2.0, 'kp': 0.6, 'ff_tau': 0.0})
    assert c.update(S(range=2.0, range_rate=0.0), 0.8, 0.1) == pytest.approx(0.8)


def test_static_board_is_pure_p():
    c = CruiseController({'headway': 2.0, 'kp': 0.6, 'ff_tau': 0.0})
    # ego at 1.0 closing on a static board: range_rate = -1.0, board speed 0
    assert c.update(S(range=5.0, range_rate=-1.0), 1.0, 0.1) == pytest.approx(1.8)


def test_invalid_returns_zero_and_resets():
    c = CruiseController({})
    c.update(S(range=5.0, range_rate=0.5), 0.5, 0.1)
    assert c.update(S(valid=False), 0.5, 0.1) == 0.0
    assert c.v_board is None


def test_feed_forward_filter_converges():
    c = CruiseController({'headway': 2.0, 'kp': 0.0, 'ff_tau': 0.3})
    v = 0.0
    for _ in range(50):
        v = c.update(S(range=2.0, range_rate=0.8), 0.0, 0.1)
    assert v == pytest.approx(0.8, abs=1e-3)
