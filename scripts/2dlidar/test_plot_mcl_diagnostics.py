#!/usr/bin/env python3
"""Tests for plot_mcl_diagnostics.py's pure parts -- plain-shell, no ROS deps.

Run: PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest scripts/2dlidar/test_plot_mcl_diagnostics.py -v
"""
import json
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent))
from plot_mcl_diagnostics import (  # noqa: E402
    load_records,
    eigenvalue_ratio,
    update_rate,
    divergence_onset,
    join_nearest_stamp,
)


# ---------------------------------------------------------------------------
# load_records
# ---------------------------------------------------------------------------

def _write_jsonl(tmp_path, lines):
    p = tmp_path / "diag.jsonl"
    p.write_text("\n".join(lines) + "\n")
    return p


def test_load_records_parses_valid_lines(tmp_path):
    lines = [
        json.dumps({"iter": 1, "n_eff": 100.0}),
        json.dumps({"iter": 2, "n_eff": 200.0}),
    ]
    path = _write_jsonl(tmp_path, lines)
    records = load_records(path)
    assert len(records) == 2
    assert records[0]["iter"] == 1
    assert records[1]["n_eff"] == 200.0


def test_load_records_skips_malformed_lines(tmp_path):
    lines = [
        json.dumps({"iter": 1, "n_eff": 100.0}),
        "{not valid json",
        "",
        json.dumps({"iter": 3, "n_eff": 300.0}),
    ]
    path = _write_jsonl(tmp_path, lines)
    records = load_records(path)
    assert len(records) == 2
    assert [r["iter"] for r in records] == [1, 3]


def test_load_records_no_valid_records_raises(tmp_path):
    lines = ["{not valid json", "also not json", ""]
    path = _write_jsonl(tmp_path, lines)
    with pytest.raises(ValueError):
        load_records(path)


def test_load_records_missing_file_raises():
    with pytest.raises(FileNotFoundError):
        load_records(Path("/nonexistent/path/diag.jsonl"))


# ---------------------------------------------------------------------------
# eigenvalue_ratio -- the ridge signature
# ---------------------------------------------------------------------------

def test_eigenvalue_ratio_isotropic_is_one():
    # Circular (isotropic) covariance: equal variances, no correlation.
    assert eigenvalue_ratio(4.0, 4.0, 0.0) == pytest.approx(1.0)


def test_eigenvalue_ratio_elongated_ridge():
    # Highly elongated along x: xx >> yy, no correlation -> eigenvalues
    # are exactly xx and yy, ratio = xx/yy.
    assert eigenvalue_ratio(100.0, 1.0, 0.0) == pytest.approx(100.0)


def test_eigenvalue_ratio_matches_numpy_eig():
    np = pytest.importorskip("numpy")
    cov_xx, cov_yy, cov_xy = 10.0, 3.0, 4.0
    mat = np.array([[cov_xx, cov_xy], [cov_xy, cov_yy]])
    eigvals = np.linalg.eigvalsh(mat)
    expected = max(eigvals) / min(eigvals)
    assert eigenvalue_ratio(cov_xx, cov_yy, cov_xy) == pytest.approx(expected)


def test_eigenvalue_ratio_degenerate_zero_covariance():
    # Zero covariance (e.g. a single particle / no spread): both
    # eigenvalues are 0. Ratio is undefined -- must not raise or produce
    # a bogus finite number.
    result = eigenvalue_ratio(0.0, 0.0, 0.0)
    assert math.isnan(result)


# ---------------------------------------------------------------------------
# update_rate
# ---------------------------------------------------------------------------

def test_update_rate_basic():
    assert update_rate(0.05) == pytest.approx(20.0)


def test_update_rate_zero_dt_is_nan():
    assert math.isnan(update_rate(0.0))


def test_update_rate_negative_dt_is_nan():
    # dt_update is negative only for the first record (no previous
    # update) -- treat as "no rate", not a crash.
    assert math.isnan(update_rate(-1.0))


# ---------------------------------------------------------------------------
# divergence_onset
# ---------------------------------------------------------------------------

def test_divergence_onset_detects_sustained_crossing():
    times = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
    errors = [0.1, 0.2, 0.15, 1.5, 1.6, 1.8]
    onset = divergence_onset(times, errors, threshold=1.0)
    assert onset == pytest.approx(3.0)


def test_divergence_onset_ignores_transient_spike():
    times = [0.0, 1.0, 2.0, 3.0, 4.0]
    errors = [0.1, 5.0, 0.1, 0.1, 0.1]  # single spike, not sustained
    onset = divergence_onset(times, errors, threshold=1.0)
    assert onset is None


def test_divergence_onset_never_crosses_returns_none():
    times = [0.0, 1.0, 2.0]
    errors = [0.1, 0.2, 0.3]
    assert divergence_onset(times, errors, threshold=1.0) is None


# ---------------------------------------------------------------------------
# join_nearest_stamp -- time-preserving nearest-timestamp join (used for
# the per-sample error-vs-time panel; compare_poses.align_tracks discards
# the pairing timestamps, which the plot needs to place points on the
# shared time axis).
# ---------------------------------------------------------------------------

def test_join_nearest_stamp_pairs_within_max_dt():
    gt = [(0.0, 0.0, 0.0, 0.0), (1.0, 1.0, 0.0, 0.0), (2.0, 2.0, 0.0, 0.0)]
    pf = [(0.02, 0.1, 0.0, 0.0), (1.05, 1.2, 0.0, 0.0)]
    pairs = join_nearest_stamp(pf, gt, max_dt=0.1)
    assert len(pairs) == 2
    t0, trans0, yaw0 = pairs[0]
    assert t0 == pytest.approx(0.02)
    assert trans0 == pytest.approx(0.1)


def test_join_nearest_stamp_empty_when_no_overlap():
    gt = [(0.0, 0.0, 0.0, 0.0)]
    pf = [(100.0, 0.0, 0.0, 0.0)]
    pairs = join_nearest_stamp(pf, gt, max_dt=0.1)
    assert pairs == []


def test_join_nearest_stamp_empty_inputs():
    assert join_nearest_stamp([], [(0.0, 0.0, 0.0, 0.0)], max_dt=0.1) == []
    assert join_nearest_stamp([(0.0, 0.0, 0.0, 0.0)], [], max_dt=0.1) == []
