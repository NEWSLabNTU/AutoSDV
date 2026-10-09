import math

import pytest

from autosdv_coach_lab.tracker import AlphaBetaTracker, PoseHistory, TrackerParams, base_link_to_world


def feed(tr, xs, ys, t0=0.0, dt=0.1):
    t = t0
    for x, y in zip(xs, ys):
        tr.update(t, x, y)
        t += dt
    return t - dt


def test_uninitialised_is_invalid():
    s = AlphaBetaTracker().predict(0.0)
    assert not s.valid


def test_static_board_range_bearing():
    tr = AlphaBetaTracker()
    t = feed(tr, [4.0] * 20, [1.0] * 20)
    s = tr.predict(t)
    assert s.valid
    assert s.range == pytest.approx(math.hypot(4.0, 1.0), abs=1e-6)
    assert s.bearing == pytest.approx(math.atan2(1.0, 4.0), abs=1e-6)
    assert s.range_rate == pytest.approx(0.0, abs=1e-6)
    assert s.age == pytest.approx(0.0)


def test_receding_board_converges_to_positive_range_rate():
    tr = AlphaBetaTracker()
    xs = [3.0 + 0.8 * 0.1 * i for i in range(60)]
    t = feed(tr, xs, [0.0] * 60)
    s = tr.predict(t)
    assert s.range_rate == pytest.approx(0.8, abs=0.02)
    assert s.range == pytest.approx(xs[-1], abs=0.05)


def test_approaching_board_negative_range_rate():
    tr = AlphaBetaTracker()
    xs = [8.0 - 1.0 * 0.1 * i for i in range(50)]
    t = feed(tr, xs, [0.0] * 50)
    assert tr.predict(t).range_rate == pytest.approx(-1.0, abs=0.03)


def test_prediction_extrapolates_to_now_and_reports_age():
    tr = AlphaBetaTracker()
    xs = [3.0 + 0.1 * i for i in range(60)]   # 1 m/s
    t = feed(tr, xs, [0.0] * 60)
    s = tr.predict(t + 0.2)
    assert s.age == pytest.approx(0.2)
    assert s.range == pytest.approx(xs[-1] + 0.2, abs=0.05)


def test_timeout_invalidates_and_next_detection_reinitialises():
    p = TrackerParams(timeout=0.5)
    tr = AlphaBetaTracker(p)
    t = feed(tr, [5.0] * 10, [0.0] * 10)
    assert tr.predict(t + 0.4).valid
    assert not tr.predict(t + 0.6).valid
    assert tr.update(t + 2.0, 2.0, 0.0)          # far from the old track: restart
    s = tr.predict(t + 2.0)
    assert s.valid and s.range == pytest.approx(2.0)
    assert s.range_rate == 0.0


def test_outlier_rejected_then_restart_after_consecutive():
    tr = AlphaBetaTracker(TrackerParams(gate=1.0, max_consecutive_rejects=3))
    t = feed(tr, [5.0] * 10, [0.0] * 10)
    assert not tr.update(t + 0.1, 9.0, 0.0)
    assert tr.predict(t + 0.1).range == pytest.approx(5.0, abs=1e-6)
    assert not tr.update(t + 0.2, 9.0, 0.0)
    assert tr.update(t + 0.3, 9.0, 0.0)          # third in a row: accepted as a restart
    assert tr.predict(t + 0.3).range == pytest.approx(9.0)


def test_rejects_out_of_range_and_nonfinite_and_out_of_order():
    tr = AlphaBetaTracker(TrackerParams(min_range=0.5, max_range=20.0))
    assert not tr.update(0.0, 0.1, 0.0)
    assert not tr.update(0.0, 25.0, 0.0)
    assert not tr.update(0.0, float('nan'), 0.0)
    assert tr.update(1.0, 5.0, 0.0)
    assert not tr.update(0.9, 5.0, 0.0)


def test_board_speed_is_bounded():
    tr = AlphaBetaTracker(TrackerParams(max_board_speed=2.0, gate=100.0))
    t = feed(tr, [3.0 + 1.0 * i for i in range(10)], [0.0] * 10)   # 10 m/s
    assert abs(tr.predict(t).range_rate) <= 2.0 + 1e-9


# --- odometry-frame tracking ---------------------------------------------------

def test_base_link_to_world():
    assert base_link_to_world(1.0, 0.0, (2.0, 3.0, math.pi / 2)) == pytest.approx((2.0, 4.0))


def test_pose_history_interpolates_and_clamps():
    h = PoseHistory(horizon=10.0)
    h.add(0.0, 0.0, 0.0, 0.0)
    h.add(1.0, 2.0, 0.0, 0.2)
    assert h.at(0.5) == pytest.approx((1.0, 0.0, 0.1))
    assert h.at(-1.0) == (0.0, 0.0, 0.0)
    assert h.at(5.0) == (2.0, 0.0, 0.2)
    assert PoseHistory().at(0.0) is None


def test_pose_history_drops_old_and_out_of_order():
    h = PoseHistory(horizon=1.0)
    for i in range(30):
        h.add(i * 0.1, i, 0, 0)
    h.add(1.0, 99, 0, 0)            # out of order: ignored
    assert len(h) <= 11
    assert h.at(2.9)[0] == pytest.approx(29)


def run_pursuit(ego_speeds, board_speed, d0=4.0, dt=0.1, latency=0.0):
    """Ego driving along x with a speed profile; board ahead at board_speed.
    Detections are sampled `latency` before they are filed."""
    tr = AlphaBetaTracker(TrackerParams(gate=5.0))
    h = PoseHistory(horizon=5.0)
    ex, bx, t = 0.0, d0, 0.0
    hist = []
    for v in ego_speeds:
        h.add(t, ex, 0.0, 0.0)
        hist.append((t, ex, bx))
        ts = t - latency
        past = [r for r in hist if r[0] <= ts + 1e-9]
        if past:
            t_s, ex_s, bx_s = past[-1]
            tr.update(t_s, bx_s - ex_s, 0.0, h.at(t_s))
        last = tr.predict(t, (ex, 0.0, 0.0), v)
        t += dt
        ex += v * dt
        bx += board_speed * dt
    return last, ex, bx


def test_range_rate_follows_ego_speed_change_without_lag():
    # Ego steps from 0 to 1.5 m/s at the last sample; board static. In the
    # odometry frame the board is static, so range_rate is -v_ego at once.
    speeds = [0.0] * 30 + [1.5]
    s, _, _ = run_pursuit(speeds, 0.0, d0=8.0)
    assert s.range_rate == pytest.approx(-1.5, abs=0.02)


def test_board_speed_recovered_while_ego_accelerates():
    speeds = [min(0.05 * i, 1.2) for i in range(60)]
    s, ex, bx = run_pursuit(speeds, 0.8, d0=5.0, latency=0.1)
    v_ego = speeds[-1]
    assert v_ego + s.range_rate == pytest.approx(0.8, abs=0.05)
    assert s.range == pytest.approx(bx - ex, abs=0.1)


def test_prediction_relative_to_current_ego_pose():
    tr = AlphaBetaTracker()
    for i in range(10):
        tr.update(i * 0.1, 5.0, 0.0, (0.0, 0.0, 0.0))   # board at world (5, 0)
    s = tr.predict(0.9, (1.0, 0.0, math.pi / 2), 0.0)   # ego moved, turned left
    assert s.range == pytest.approx(math.hypot(4.0, 0.0))
    assert s.bearing == pytest.approx(-math.pi / 2)
