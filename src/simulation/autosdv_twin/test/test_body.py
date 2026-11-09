"""The generated body's catalogue entry against vehicle_info and the photos."""

import json
from pathlib import Path

import pytest
import yaml

TWIN = Path(__file__).resolve().parents[1]
ENTRY = TWIN / "isaac/body/vehicle.newslab.autosdv_car_a/entry.json"
SPEC = TWIN / "vehicle/autosdv_car_a.yaml"


@pytest.fixture(scope="module")
def entry():
    return json.loads(ENTRY.read_text())


@pytest.fixture(scope="module")
def geometry():
    return yaml.safe_load(SPEC.read_text())["geometry"]


def test_footprint_is_vehicle_info(entry, geometry):
    ext = entry["carla_bbox"]["extent"]
    length = geometry["wheel_base"] + geometry["front_overhang"] + geometry["rear_overhang"]
    width = geometry["wheel_tread"] + geometry["left_overhang"] + geometry["right_overhang"]
    assert 2 * ext[0] == pytest.approx(length, abs=0.01)   # 0.421
    assert 2 * ext[1] == pytest.approx(width, abs=0.01)    # 0.333


def test_rear_axle_offset(entry, geometry):
    length = 2 * entry["carla_bbox"]["extent"][0]
    assert entry["rear_axle_x"] == pytest.approx(-length / 2 + geometry["rear_overhang"], abs=1e-4)


def test_wheels_at_the_axles(entry, geometry):
    xs = sorted({round(w["pos"][0] - entry["rear_axle_x"], 4) for w in entry["wheels"]})
    assert xs == pytest.approx([0.0, geometry["wheel_base"]], abs=1e-4)
    assert all(w["radius"] == geometry["wheel_radius"] for w in entry["wheels"])


def test_lidar_top_height(entry):
    # the photos: the VLP-16's top about 0.39 m above the ground
    assert entry["render_max"][2] == pytest.approx(0.39, abs=0.01)
