"""`just twin env`: the twin runs on the DDS profile every AutoSDV process uses, and on the
twin's unicast loopback profile only when asked (a host without the cyclonedds-sysctl
setup step: rmem_max below 10 MB, or no multicast on lo)."""
import os
import subprocess
from pathlib import Path

REPO = Path(__file__).resolve().parents[4]


def _env(**over):
    env = {k: v for k, v in os.environ.items() if k != "TWIN_CYCLONEDDS_URI"}
    env.update(over)
    out = subprocess.run(["just", "twin", "env"], cwd=REPO, env=env, capture_output=True,
                         text=True, check=True).stdout
    return dict(line[len("export "):].split("=", 1) for line in out.splitlines()
                if line.startswith("export CYCLONEDDS_URI="))


def test_the_twin_uses_the_workspace_profile():
    uri = "file:///opt/autoware/1.5.0/config/cyclonedds.xml"
    assert _env(CYCLONEDDS_URI=uri)["CYCLONEDDS_URI"] == uri


def test_the_unicast_profile_is_opt_in():
    uni = "file:///x/cyclonedds-unicast-lo.xml"
    assert _env(TWIN_CYCLONEDDS_URI=uni)["CYCLONEDDS_URI"] == uni
