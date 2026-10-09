import math
import sys
import types

import pytest

from autosdv_coach_lab import pursuit
from autosdv_coach_lab.pursuit import PlannerParams, build_trajectory, clamp_speed

P = PlannerParams(v_max=1.5, accel_max=0.5, decel_max=2.0, standoff_min=1.0,
                  path_length=10.0, path_step=0.2, brake_decel=0.8, envelope_decel=0.6,
                  reaction_time=0.5)


def arc(points):
    return [math.hypot(p[0] - points[0][0], p[1] - points[0][1]) for p in points]


# --- trajectory ------------------------------------------------------------

@pytest.mark.parametrize('s_stop', [0.0, 0.1, 2.5, 9.9, 10.0, 50.0])
def test_trajectory_fixed_length_whatever_the_stop(s_stop):
    pts = build_trajectory(1.0, 2.0, 0.3, 1.0, s_stop, P)
    assert len(pts) == 51
    s = arc(pts)
    assert s[-1] == pytest.approx(10.0)


def test_trajectory_arc_length_strictly_increasing_uniform():
    pts = build_trajectory(-3.0, 7.0, 2.0, 1.0, 4.0, P)
    s = arc(pts)
    steps = [b - a for a, b in zip(s, s[1:])]
    assert all(d == pytest.approx(0.2) for d in steps)


def test_trajectory_heading_and_origin():
    pts = build_trajectory(1.0, 1.0, math.pi / 2, 1.0, 5.0, P)
    assert pts[0][:2] == pytest.approx((1.0, 1.0))
    assert pts[-1][:2] == pytest.approx((1.0, 11.0))
    assert all(p[2] == pytest.approx(math.pi / 2) for p in pts)


def test_stop_encoded_as_zero_velocity_from_s_stop_on():
    pts = build_trajectory(0.0, 0.0, 0.0, 1.2, 3.0, P)
    for (x, _, _, v, _) in pts:
        if x < 3.0 - 1e-6:
            assert v == pytest.approx(min(1.2, math.sqrt(2 * 0.8 * (3.0 - x))))
            assert v > 0.0
        else:
            assert v == 0.0
    # far from the stop the commanded speed is held
    assert pts[0][3] == pytest.approx(1.2)


@pytest.mark.parametrize('v,v_ego,s_stop', [
    (1.5, 0.0, 0.0), (1.5, 1.5, 0.3), (0.4, 1.4, 2.0), (1.5, 0.2, 6.0),
    (1.0, 1.0, 20.0), (0.0, 1.2, 0.9), (0.0, 0.0, 0.0)])
def test_profile_monotone_non_increasing(v, v_ego, s_stop):
    vs = [p[3] for p in build_trajectory(0, 0, 0, v, s_stop, P, v_ego=v_ego)]
    assert all(b <= a + 1e-12 for a, b in zip(vs, vs[1:]))
    assert all(x == 0.0 for x, d in zip(vs, range(len(vs))) if d * 0.2 >= s_stop - 1e-9)


def test_slowing_down_ramps_from_current_speed():
    vs, accs = pursuit.velocity_profile(0.4, 1.4, 8.0, P)
    assert vs[0] == pytest.approx(1.4)
    assert accs[0] == pytest.approx(-0.8, abs=0.05)
    assert min(vs[:30]) == pytest.approx(0.4)        # settles on the command
    assert vs[-1] == 0.0


def test_stop_ramp_starts_at_ego_speed_with_feedforward_decel():
    s_stop = pursuit.stop_distance(0.0, 1.2, 10.0, True, P)
    assert s_stop == pytest.approx(1.2 ** 2 / 1.6)
    vs, accs = pursuit.velocity_profile(0.0, 1.2, s_stop, P)
    assert vs[0] == pytest.approx(1.2)
    assert all(a <= 0.0 for a in accs)


def test_stop_ramp_steepens_when_floor_is_closer_than_braking_distance():
    vs, _ = pursuit.velocity_profile(0.0, 1.2, 0.4, P)
    assert vs[0] == pytest.approx(1.2)
    assert vs[2] == 0.0


def test_zero_stop_distance_is_all_zero():
    assert all(p[3] == 0.0 for p in build_trajectory(0, 0, 0, 1.0, 0.0, P))


def test_stop_distance():
    # driving: the floor
    assert pursuit.stop_distance(1.0, 1.0, 5.0, True, P) == pytest.approx(4.0)
    assert pursuit.stop_distance(1.0, 1.0, 30.0, True, P) == pytest.approx(10.0)
    assert pursuit.stop_distance(1.0, 1.0, 0.5, True, P) == 0.0
    # stopping: braking distance, never past the floor
    assert pursuit.stop_distance(0.0, 0.8, 5.0, True, P) == pytest.approx(0.4)
    assert pursuit.stop_distance(0.0, 1.5, 1.5, True, P) == pytest.approx(0.5)
    assert pursuit.stop_distance(0.0, 0.0, 5.0, True, P) == 0.0
    # board lost: braking distance
    assert pursuit.stop_distance(1.0, 0.8, 5.0, False, P) == pytest.approx(0.4)
    assert pursuit.stop_distance(1.0, 0.0, 5.0, False, P) == 0.0


# --- clamps ----------------------------------------------------------------

def test_invalid_state_stops_immediately():
    assert clamp_speed(1.0, 1.5, 0.1, 5.0, False, P) == (0.0, 'invalid')


def test_nan_and_none_stop():
    assert clamp_speed(float('nan'), 1.0, 0.1, 5.0, True, P) == (0.0, 'error')
    assert clamp_speed(None, 1.0, 0.1, 5.0, True, P) == (0.0, 'error')


def test_negative_request_is_zero_after_rate_limit():
    v, _ = clamp_speed(-5.0, 0.0, 0.1, 5.0, True, P)
    assert v == 0.0


def test_accel_limit():
    v, lim = clamp_speed(1.5, 0.0, 0.1, 8.0, True, P)
    assert v == pytest.approx(0.05) and lim == 'accel'


def test_decel_limit():
    v, lim = clamp_speed(0.0, 1.0, 0.1, 8.0, True, P)
    assert v == pytest.approx(0.8) and lim == 'decel'


def test_v_max_cap():
    v, lim = clamp_speed(10.0, 1.5, 0.1, 50.0, True, P)
    assert v == pytest.approx(1.5) and lim == 'v_max'


def test_standoff_floor_wins_over_everything():
    assert clamp_speed(1.5, 1.5, 0.1, 1.0, True, P) == (0.0, 'standoff')
    assert clamp_speed(1.5, 1.5, 0.1, 0.4, True, P) == (0.0, 'standoff')


def test_braking_envelope_near_floor():
    v, lim = clamp_speed(1.5, 1.5, 0.1, 1.1, True, P)
    assert v == pytest.approx(pursuit.braking_speed(0.1, P)) and lim == 'standoff'


@pytest.mark.parametrize('gap', [0.05, 0.5, 1.0, 2.2, 5.0])
def test_braking_speed_stops_within_gap(gap):
    v = pursuit.braking_speed(gap, P)
    assert v * P.reaction_time + v * v / (2 * P.envelope_decel) == pytest.approx(gap)


def test_braking_speed_at_headway():
    # the documented numbers in board_pursuit_planner.param.yaml
    q = PlannerParams(envelope_decel=0.6, reaction_time=0.5)
    assert pursuit.braking_speed(1.0, q) == pytest.approx(0.84, abs=0.01)
    assert pursuit.braking_speed(2.6, q) == pytest.approx(1.5, abs=0.01)
    assert pursuit.braking_speed(0.0, q) == 0.0


def test_inside_limits_passes_through():
    assert clamp_speed(0.8, 0.8, 0.1, 6.0, True, P) == (pytest.approx(0.8), '')


# --- controller loading ------------------------------------------------------

class _State:
    def __init__(self, valid=True, range=4.0, range_rate=0.0, bearing=0.0, age=0.0):
        self.valid, self.range, self.range_rate = valid, range, range_rate
        self.bearing, self.age = bearing, age


def test_load_reference_controller_and_params():
    c = pursuit.load_controller(
        'autosdv_coach_lab.reference.cruise_controller:CruiseController',
        {'headway': 3.0, 'kp': 1.0, 'k_ff': 0.0})
    assert c.update(_State(range=5.0), 0.0, 0.1) == pytest.approx(2.0)


def test_load_stub_returns_zero():
    c = pursuit.load_controller(
        'autosdv_coach_lab.stub.cruise_controller:CruiseController', {})
    c.reset()
    assert c.update(_State(), 0.0, 0.1) == 0.0


@pytest.mark.parametrize('spec', ['', 'no_colon', ':Cls', 'mod:'])
def test_malformed_spec(spec):
    with pytest.raises(ValueError):
        pursuit.load_controller(spec, {})


def test_missing_module_and_class():
    with pytest.raises(ImportError):
        pursuit.load_controller('no_such_module_xyz:C', {})
    with pytest.raises(AttributeError):
        pursuit.load_controller('autosdv_coach_lab.pursuit:NoSuchClass', {})


def test_class_without_update_rejected(monkeypatch):
    mod = types.ModuleType('fake_student_mod')

    class Bad:
        def __init__(self, params):
            pass

        def reset(self):
            pass

    mod.Bad = Bad
    monkeypatch.setitem(sys.modules, 'fake_student_mod', mod)
    with pytest.raises(ValueError):
        pursuit.load_controller('fake_student_mod:Bad', {})


def test_params_dict_is_a_copy():
    mod = types.ModuleType('fake_student_mod2')

    class Good:
        def __init__(self, params):
            params['mutated'] = True

        def reset(self):
            pass

        def update(self, s, v, dt):
            return 0.0

    mod.Good = Good
    sys.modules['fake_student_mod2'] = mod
    try:
        p = {'a': 1}
        pursuit.load_controller('fake_student_mod2:Good', p)
        assert p == {'a': 1}
    finally:
        del sys.modules['fake_student_mod2']


def test_yaw_from_quaternion():
    yaw = 0.7
    assert pursuit.yaw_from_quaternion(0, 0, math.sin(yaw / 2), math.cos(yaw / 2)) == \
        pytest.approx(yaw)
