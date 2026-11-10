"""The twin's Autoware launch: AutoSDV's logging simulation as car A runs, with the twin's own
settings, and the logging simulation forwarding them (a launch argument nobody declares is
dropped without a word)."""

import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[4]
TWIN = REPO / "src/simulation/autosdv_twin/launch/twin_autoware.launch.xml"
LOGSIM = REPO / "src/launcher/autosdv_launch/launch/logging_simulation.launch.yaml"

WANT = {
    "vehicle_id": "twin",
    "planning_profile": "rc",
    "sensor_suite": "vlp16_zed",
    "camera_model": "none",
    "imu_source": "zed",
    "use_gnss": "false",
    "use_sim_time": "true",
    "launch_sensing_driver": "false",
    "launch_vehicle_interface": "false",
}


def _include_args():
    root = ET.parse(TWIN).getroot()
    inc = next(i for i in root.iter("include") if "logging_simulation.launch.yaml" in i.get("file"))
    return {a.get("name"): a.get("value") for a in inc.findall("arg")}


def test_the_twin_runs_the_logging_simulation_as_car_a():
    args = _include_args()
    for k, v in WANT.items():
        assert args.get(k) == v, f"{k}: {args.get(k)!r}, want {v!r}"


def test_the_logging_simulation_declares_and_forwards_planning_profile():
    doc = yaml.safe_load(LOGSIM.read_text())["launch"]
    declared = {e["arg"]["name"] for e in doc if "arg" in e}
    assert "planning_profile" in declared
    inc = next(e["include"] for e in doc if "include" in e
               and e["include"]["file"].endswith("autosdv_autoware.launch.xml"))
    passed = {a["name"]: a["value"] for a in inc["arg"]}
    assert passed.get("planning_profile") == "$(var planning_profile)"


def test_the_twin_localizes_with_car_as_matcher():
    # cuda_ndt, AutoSDV's default, as car A runs; it once kept autonomous mode unavailable
    # by naming its status without the node prefix (known-config-defects §9, fixed upstream)
    root = ET.parse(TWIN).getroot()
    arg = next(a for a in root.findall("arg") if a.get("name") == "twin_pose_source")
    assert arg.get("default") == "cuda_ndt"
