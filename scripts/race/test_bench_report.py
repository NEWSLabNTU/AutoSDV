"""bench_report.py: one table row per label directory (system python, no ROS)."""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bench_report  # noqa: E402


def _layer(prefix, base):
    return {f"{prefix}_p50": base, f"{prefix}_p95": base * 2, f"{prefix}_p99": base * 3,
            f"{prefix}_max": base * 4, f"{prefix}_overruns": 1, f"{prefix}_n": 10}


def _label_dir(root, label, nvpmodel, load_start, load_end, gpu=False):
    d = root / label
    d.mkdir()
    allv = {**_layer("l1", 0.1), **_layer("l2", 0.001)}
    (d / f"{label}.json").write_text(json.dumps({
        "host": {"machine": "x86_64", "nvpmodel": nvpmodel, "loadavg": [0, 0, 0]},
        "label": label, "per_course": {}, "all": allv}))
    (d / f"acados_{label}.json").write_text(json.dumps({
        "time_tot": {"p50": 0.0002, "p95": 0.0004, "p99": 0.0005, "max": 0.0008},
        "casadi_solve_wall": {"p50": 0.003, "p95": 0.004, "p99": 0.005, "max": 0.006}}))
    (d / "load.json").write_text(json.dumps({"start": load_start, "end": load_end}))
    if gpu:
        rows = ["timestamp,gpu_load_percent,gpu_freq_mhz"] + \
               [f"t{i},{i},1000" for i in range(1, 101)]
        (d / "gpu.csv").write_text("\n".join(rows) + "\n")
    return d


def test_report_has_one_row_per_label_with_load_and_power_mode(tmp_path):
    a = _label_dir(tmp_path, "orin-idle", "NV Power Mode: MAXN", 0.11, 0.22, gpu=True)
    b = _label_dir(tmp_path, "x86-idle", None, 1.5, 2.5)
    out = tmp_path / "r.md"
    bench_report.main([str(a), str(b), "-o", str(out)])
    rows = [ln for ln in out.read_text().splitlines()
            if ln.startswith("|") and not ln.startswith("| Label") and "---" not in ln]
    assert len(rows) == 2
    orin = next(r for r in rows if "orin-idle" in r)
    x86 = next(r for r in rows if "x86-idle" in r)
    assert "MAXN" in orin and "0.11" in orin and "0.22" in orin
    assert "—" in x86 and "MAXN" not in x86 and "1.50" in x86 and "2.50" in x86
    assert "95" in orin          # gpu util p95 sampled
    assert "100 ms" not in orin and "400.0" in orin   # L1 max = 0.4 s in ms


def test_clocks_pinned_cell_from_host_clocks(tmp_path):
    a = _label_dir(tmp_path, "orin-maxn", "NV Power Mode: MAXN", 0.1, 0.2)
    b = _label_dir(tmp_path, "orin-float", "NV Power Mode: MAXN", 0.1, 0.2)
    c = _label_dir(tmp_path, "x86-none", None, 0.1, 0.2)
    for d, cpu, gpu in ((a, True, True), (b, True, False), (c, None, None)):
        f = d / f"{d.name}.json"
        j = json.loads(f.read_text())
        j["host"]["clocks"] = {"cpu": None, "gpu": None, "cpu_pinned": cpu, "gpu_pinned": gpu}
        f.write_text(json.dumps(j))
    out = tmp_path / "r.md"
    bench_report.main([str(a), str(b), str(c), "-o", str(out)])
    lines = out.read_text().splitlines()
    col = lines[0].split(" | ").index("Clocks pinned")
    cell = {ln.split(" | ")[0].strip("| "): ln.split(" | ")[col] for ln in lines[2:]}
    assert cell["orin-maxn"] == "cpu+gpu"
    assert cell["orin-float"] == "cpu"
    assert cell["x86-none"] == "—"
