import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent))
from wheel_imu_odom import integrate_step


def test_straight_line():
    x, y, th = 0.0, 0.0, 0.0
    for _ in range(100):
        x, y, th = integrate_step(x, y, th, v=2.0, omega=0.0, dt=0.05)
    assert x == pytest.approx(10.0, abs=1e-6)
    assert y == pytest.approx(0.0, abs=1e-9)
    assert th == pytest.approx(0.0)


def test_quarter_circle_left():
    # v=1 m/s, omega=0.1 rad/s → radius 10 m; after pi/2/0.1 s heading = 90°
    x, y, th = 0.0, 0.0, 0.0
    steps = 10000
    total_t = (math.pi / 2) / 0.1
    dt = total_t / steps
    for _ in range(steps):
        x, y, th = integrate_step(x, y, th, v=1.0, omega=0.1, dt=dt)
    assert th == pytest.approx(math.pi / 2, abs=1e-3)
    assert x == pytest.approx(10.0, abs=0.05)   # r*sin(θ)
    assert y == pytest.approx(10.0 * (1 - math.cos(math.pi / 2)), abs=0.05)


def test_zero_dt_no_motion():
    assert integrate_step(1.0, 2.0, 0.5, v=3.0, omega=1.0, dt=0.0) == (1.0, 2.0, 0.5)


def test_reverse():
    x, y, th = 0.0, 0.0, 0.0
    x, y, th = integrate_step(x, y, th, v=-1.0, omega=0.0, dt=1.0)
    assert x == pytest.approx(-1.0)
