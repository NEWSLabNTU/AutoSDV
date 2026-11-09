"""`just twin world` fails a world a 5 cm wheel cannot drive: a hole under a lanelet, or a
lane rougher than 1 cm at p99 (swb's world_report.py measures; world_gate.py judges)."""
import json
import subprocess
import sys
from pathlib import Path

GATE = Path(__file__).resolve().parents[1] / "scripts/world_gate.py"
GOOD = {"lane_cells": 100, "lane_cells_meshed": 1.0, "roughness_p99": 0.0026,
        "pit_depth_max": 0.02, "cell": 0.25}


def _gate(tmp_path, **over):
    f = tmp_path / "world_report.json"
    f.write_text(json.dumps({**GOOD, **over}))
    return subprocess.run([sys.executable, str(GATE), str(f), "--pit-depth", "0.04"],
                          capture_output=True, text=True)


def test_the_built_coss_world_passes(tmp_path):
    assert _gate(tmp_path).returncode == 0


def test_a_hole_under_a_lanelet_fails(tmp_path):
    r = _gate(tmp_path, lane_cells_meshed=0.999)
    assert r.returncode == 1 and "meshed" in r.stdout


def test_a_rough_lane_fails(tmp_path):
    r = _gate(tmp_path, roughness_p99=0.012)
    assert r.returncode == 1 and "rough" in r.stdout


def test_a_pit_deeper_than_the_site_fills_warns_only(tmp_path):
    # COSS has one real 9 cm kerb step under a path: faithful, so a warning, not a failure
    r = _gate(tmp_path, pit_depth_max=0.09)
    assert r.returncode == 0 and "pit" in r.stdout
