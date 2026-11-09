"""twin_go.py's route file reading: poses in the map frame, the quaternion from the yaw."""
import math
import sys
from pathlib import Path

import pytest

TWIN = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TWIN / "scripts"))
import twin_go  # noqa: E402


def test_a_route_reads_its_start_and_goal():
    start, goal = twin_go.load_route(TWIN / "config/routes.yaml", "coss_loop")
    assert (start.x, start.y, start.z) == (28.30, -2.32, 8.93)
    assert (goal.x, goal.y) == (33.05, 3.41)


def test_the_yaw_is_a_quaternion_about_z():
    start, _ = twin_go.load_route(TWIN / "config/routes.yaml", "coss_loop")
    assert start.qz == pytest.approx(math.sin(0.476 / 2))
    assert start.qw == pytest.approx(math.cos(0.476 / 2))
    assert start.qx == 0.0 and start.qy == 0.0


def test_an_unknown_route_names_the_known_ones():
    with pytest.raises(SystemExit, match="coss_loop"):
        twin_go.load_route(TWIN / "config/routes.yaml", "nowhere")


def test_the_spawn_argument_is_x_y_z_yaw():
    start, _ = twin_go.load_route(TWIN / "config/routes.yaml", "coss_loop")
    assert twin_go.spawn_arg(start) == "28.3,-2.32,8.98,0.476"   # 5 cm above the road


def test_autoware_is_capped_at_car_as_speed_not_only_the_motor():
    # without a latched limit Autoware plans to max_vel 4.17 m/s and only the plant's
    # motor holds the car at 2.04: the cap has to be Autoware's, as the drive tool sets it
    topic, msg = twin_go.velocity_limit(2.0)
    assert topic == "/planning/scenario_planning/max_velocity_default"
    assert msg.max_velocity == 2.0
    assert msg.use_constraints is False
    assert twin_go.MAX_SPEED == 2.0
