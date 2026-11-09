"""The twin's two descriptions of car A must agree.

The Isaac side reads the vehicle spec (vehicle/autosdv_car_a.yaml); Autoware reads the
`twin` instance in individual_params through the sensor kit's URDF. A mount edited in one
and not the other would put a sensor in two places, so every spec sensor's pose in
base_link is compared with the pose the URDF gives its link.

Needs a sourced workspace: the URDF's xacro includes resolve through the ament index.
"""

import math
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import xacro
import yaml

REPO = Path(__file__).resolve().parents[4]
TWIN_PKG = REPO / "src/simulation/autosdv_twin"
SPEC = TWIN_PKG / "vehicle/autosdv_car_a.yaml"
PARAMS = REPO / "src/param/autoware_individual_params/individual_params/config"
TWIN_CFG = PARAMS / "twin/autosdv_sensor_kit"
SENSORS_XACRO = (
    REPO
    / "src/sensor_kit/autosdv_sensor_kit_launch/autosdv_sensor_kit_description/urdf/sensors.xacro"
)
VEHICLE_INFO = (
    REPO
    / "src/vehicle/autosdv_vehicle_launch/autosdv_vehicle_description/config/vehicle_info.param.yaml"
)
TOL = 1e-6
# Frames a launch file publishes as a static transform rather than the URDF: child ->
# (parent, the launch's default offset). zed_imu_tf.launch.xml's offsets default to zero.
STATIC_TF = {"zedxm_imu_link": "zedxm_left_camera_frame"}


def _tf(x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    t = np.eye(4)
    t[:3, :3] = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
    t[:3, 3] = [x, y, z]
    return t


def _mount(m):
    return _tf(**{k: float(m[k]) for k in ("x", "y", "z", "roll", "pitch", "yaw")})


def _urdf_poses():
    """base_link -> every link, from the twin's sensor kit URDF."""
    doc = xacro.process_file(str(SENSORS_XACRO), mappings={"config_dir": str(TWIN_CFG)})
    root = ET.fromstring(doc.toxml())
    parent = {}
    for j in root.iter("joint"):
        o = j.find("origin")
        xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
        rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
        parent[j.find("child").get("link")] = (j.find("parent").get("link"), _tf(*xyz, *rpy))
    for child, par in STATIC_TF.items():
        parent.setdefault(child, (par, np.eye(4)))

    def pose(link):
        t = np.eye(4)
        while link != "base_link":
            link, step = parent[link]
            t = step @ t
        return t

    return pose, parent


@pytest.fixture(scope="module")
def spec():
    return yaml.safe_load(SPEC.read_text())


def test_mounts_match(spec):
    pose, links = _urdf_poses()
    kit = _mount(spec["sensor_kit"]["mount"])
    for name, s in spec["sensors"].items():
        assert s["link"] in links, f"{name}: link {s['link']} not in the twin URDF"
        want = kit @ _mount(s["mount"])
        got = pose(s["link"])
        assert np.allclose(want, got, atol=TOL), (
            f"{name}: spec puts {s['link']} at {want[:3, 3]}, the twin URDF at {got[:3, 3]}"
        )


def test_geometry_matches_vehicle_info(spec):
    info = yaml.safe_load(VEHICLE_INFO.read_text())["/**"]["ros__parameters"]
    for key, value in spec["geometry"].items():
        assert info[key] == pytest.approx(value, abs=TOL), key


def test_car_a_untouched():
    sub = REPO / "src/param/autoware_individual_params"
    r = subprocess.run(
        ["git", "diff", "--quiet", "HEAD", "--", "individual_params/config/car_a"], cwd=sub
    )
    assert r.returncode == 0, "the twin work must not change car_a"
