"""A proof run compares a twin bag with a car bag on the same route: both record one topic
list, scripts/rosbag/proof_run_topics.txt, so neither side lacks what the other has."""
from pathlib import Path

REPO = Path(__file__).resolve().parents[4]
TOPICS = REPO / "scripts/rosbag/proof_run_topics.txt"

NEEDED = [
    "/sensing/lidar/velodyne_points", "/sensing/camera/zedxm/imu/data",
    "/vehicle/status/velocity_status", "/vehicle/status/steering_status",
    "/control/command/control_cmd", "/localization/kinematic_state",
    "/localization/pose_estimator/nearest_voxel_transformation_likelihood",
    "/planning/scenario_planning/trajectory", "/perception/object_recognition/objects",
    "/api/operation_mode/state", "/api/routing/state", "/tf", "/tf_static",
]


def _topics():
    return [l.strip() for l in TOPICS.read_text().splitlines() if l.strip() and not l.lstrip().startswith("#")]


def test_the_list_holds_what_a_comparison_reads():
    have = set(_topics())
    assert not [t for t in NEEDED if t not in have]


def test_the_car_and_the_twin_record_the_same_list():
    bag = (REPO / "just/bag.just").read_text()
    twin = (REPO / "just/twin.just").read_text()
    assert "proof_run_topics.txt" in bag and "proof_run_topics.txt" in twin
