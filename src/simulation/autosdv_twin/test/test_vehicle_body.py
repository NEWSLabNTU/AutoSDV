"""autosdv_vehicle_description draws car A's mesh for car_a and twin, the box for the rest."""

import os
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import xacro

REPO = Path(__file__).resolve().parents[4]
XACRO = Path(os.environ.get(
    "VEHICLE_XACRO",
    REPO / "src/vehicle/autosdv_vehicle_launch/autosdv_vehicle_description/urdf/vehicle.xacro"))
PARAMS = REPO / "src/param/autoware_individual_params/individual_params/config"


def _car_geometry(vehicle_id):
    mappings = {"config_dir": str(PARAMS / vehicle_id / "autosdv_sensor_kit")} if vehicle_id else {}
    root = ET.fromstring(xacro.process_file(str(XACRO), mappings=mappings).toxml())
    link = next(l for l in root.iter("link") if l.get("name") == "car_link")
    return link.find("visual/geometry")[0]


@pytest.mark.parametrize("vehicle_id", ["car_a", "twin"])
def test_car_a_mesh(vehicle_id):
    g = _car_geometry(vehicle_id)
    assert g.tag == "mesh" and g.get("filename").endswith("/mesh/car_a.dae")


@pytest.mark.parametrize("vehicle_id", ["car_b", "car_c", "sim", "default", None])
def test_others_keep_the_box(vehicle_id):
    g = _car_geometry(vehicle_id)
    assert g.tag == "box" and g.get("size") == "0.421 0.306 0.262"
