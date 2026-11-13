"""bench.sh: SWB preflight, load.json on failure, acados pinning, GPU sampler check.

The race venv is faked: its bin/python logs each call and its CPU affinity, so nothing is solved.
"""
import json
import os
import subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
BENCH = os.environ.get("BENCH_SH", os.path.join(HERE, "bench.sh"))

STUB = """#!/bin/bash
echo "$* | $(grep Cpus_allowed_list /proc/self/status | cut -f2)" >> "{log}"
case "$1" in *bench_live.py) exit "${{STUB_LIVE_RC:-0}}" ;; esac
exit 0
"""


def _swb(tmp_path):
    swb = tmp_path / "swb"
    for f in ("src/sim_vehicle_spec/vehicles/drift_mustang.yaml",
              "src/swb_isaac_world/config/surface_classes.yaml"):
        (swb / f).parent.mkdir(parents=True, exist_ok=True)
        (swb / f).write_text("x: 1\n")
    return swb


def _bench(tmp_path, *args, swb=True, env_extra=None):
    venv = tmp_path / "venv"
    (venv / "bin").mkdir(parents=True, exist_ok=True)
    log = tmp_path / "calls.log"
    py = venv / "bin" / "python"
    py.write_text(STUB.format(log=log))
    py.chmod(0o755)
    lines = [f'export AUTOSDV_RACE_VENV="{venv}"', f'export ACADOS_SOURCE_DIR="{tmp_path}/acados"']
    if swb:
        lines.append(f'export SWB="${{SWB:-{_swb(tmp_path)}}}"')
    (venv / "race-env.sh").write_text("\n".join(lines) + "\n")
    env = {k: v for k, v in os.environ.items() if k not in ("SWB", "RP_DET", "RACE_GPU_SAMPLER")}
    env.update({"AUTOSDV_RACE_VENV": str(venv), "RACE_BENCH_OUT": str(tmp_path / "out")})
    env.update(env_extra or {})
    r = subprocess.run(["bash", BENCH, "lbl", *args], env=env, capture_output=True, text=True,
                       timeout=60)
    calls = log.read_text().splitlines() if log.exists() else []
    return r, calls, tmp_path / "out" / "lbl"


def test_unset_swb_stops_naming_it_and_how_to_get_it(tmp_path):
    r, calls, _ = _bench(tmp_path, swb=False)
    assert r.returncode != 0
    assert "SWB" in r.stderr and "scenario-world-bridge" in r.stderr
    assert "sim_vehicle_ecu" in r.stderr and "sim_vehicle_spec" in r.stderr
    assert calls == []


def test_missing_swb_dir_stops(tmp_path):
    r, calls, _ = _bench(tmp_path, env_extra={"SWB": str(tmp_path / "nope")})
    assert r.returncode != 0
    assert str(tmp_path / "nope") in r.stderr and "scenario-world-bridge" in r.stderr
    assert calls == []


def test_a_failed_run_still_records_both_loads(tmp_path):
    r, calls, out = _bench(tmp_path, env_extra={"STUB_LIVE_RC": "3"})
    assert r.returncode != 0
    load = json.loads((out / "load.json").read_text())
    assert isinstance(load["start"], float) and isinstance(load["end"], float)


def test_acados_runs_on_the_first_cpu_of_cpus(tmp_path):
    allowed = sorted(os.sched_getaffinity(0))
    first, second = allowed[0], allowed[-1]
    r, calls, _ = _bench(tmp_path, "--cpus", f"{first},{second}")
    assert r.returncode == 0, r.stderr
    acados = [c for c in calls if "acados_tracker.py" in c]
    assert len(acados) == 2
    assert all(c.split(" | ")[1] == str(first) for c in acados), acados
    live = [c for c in calls if "bench_live.py" in c]
    assert len(live) == 1 and f"--cpus {first},{second}" in live[0]


def test_without_cpus_nothing_is_pinned(tmp_path):
    r, calls, _ = _bench(tmp_path)
    assert r.returncode == 0, r.stderr
    own = subprocess.run(["bash", "-c", "grep Cpus_allowed_list /proc/self/status | cut -f2"],
                         capture_output=True, text=True).stdout.strip()
    assert all(c.split(" | ")[1] == own for c in calls), calls


def test_a_sampler_that_dies_at_startup_is_warned_about(tmp_path):
    dead = tmp_path / "dead_sampler"
    dead.write_text("#!/bin/bash\nexit 1\n")
    dead.chmod(0o755)
    r, _, _ = _bench(tmp_path, env_extra={"RACE_GPU_SAMPLER": str(dead)})
    assert r.returncode == 0, r.stderr
    assert "GPU sampler" in r.stderr and "died" in r.stderr


def test_a_running_sampler_is_not_warned_about(tmp_path):
    alive = tmp_path / "alive_sampler"
    alive.write_text("#!/bin/bash\nexec sleep 30\n")
    alive.chmod(0o755)
    r, _, _ = _bench(tmp_path, env_extra={"RACE_GPU_SAMPLER": str(alive)})
    assert r.returncode == 0, r.stderr
    assert "died" not in r.stderr
