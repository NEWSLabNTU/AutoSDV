#!/usr/bin/env python3
"""Tests for score_sensor_model.py's pure functions.

Plain-shell, no ROS deps.
Run: PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest scripts/2dlidar/test_score_sensor_model.py -v
"""
import json
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent))
from score_sensor_model import (  # noqa: E402
    field_stats,
    percentile_rank,
    is_local_max,
    apply_skip_nonfinite_mask,
    build_table,
    build_upstream_table,
    build_normalized_short_table,
    VARIANT_BUILDERS,
    summarize,
    NumpyEncoder,
)


# ---------------------------------------------------------------------------
# 1. field_stats -- argmax/gap/dist on a synthetic field with a KNOWN
#    argmax.
# ---------------------------------------------------------------------------

def _make_grid_field(n, gt_row, gt_col, gt_ll, hot_row, hot_col, hot_ll, fill=-1000.0):
    """Build a flat (n*n,) log_likelihood array matching evaluate_pose_grid's
    meshgrid row-major layout: index = row*n + col, row indexes ys, col
    indexes xs."""
    grid = np.full((n, n), fill, dtype=np.float64)
    grid[gt_row, gt_col] = gt_ll
    grid[hot_row, hot_col] = hot_ll
    return grid.reshape(-1)


def test_field_stats_known_argmax_off_center():
    # 5x5 grid, GT at the centre (2,2), a known hotter cell adjacent to it
    # (2,3) -- adjacent so this also exercises local_max=False (a neighbour
    # beats GT), while still being a non-trivial off-centre argmax for the
    # gap/dist computation.
    n = 5
    xs = np.arange(n, dtype=np.float64)  # 0..4, spacing 1.0
    ys = np.arange(n, dtype=np.float64)
    gt_x, gt_y = xs[2], ys[2]
    ll = _make_grid_field(n, gt_row=2, gt_col=2, gt_ll=-10.0,
                           hot_row=2, hot_col=3, hot_ll=-3.0)

    stats = field_stats(ll, xs, ys, gt_x, gt_y)

    assert stats["gap_nats"] == pytest.approx(7.0)          # -3 - (-10)
    assert stats["argmax_x"] == pytest.approx(3.0)
    assert stats["argmax_y"] == pytest.approx(2.0)
    expected_dist = float(np.hypot(3.0 - 2.0, 2.0 - 2.0))
    assert stats["dist_m"] == pytest.approx(expected_dist)
    assert stats["local_max"] is False   # the (2,3) neighbour outscores GT
    assert stats["ll_at_gt"] == pytest.approx(-10.0)
    assert stats["ll_at_argmax"] == pytest.approx(-3.0)


def test_field_stats_gt_is_the_argmax_zero_gap():
    n = 3
    xs = np.array([10.0, 11.0, 12.0])
    ys = np.array([20.0, 21.0, 22.0])
    ll = np.full((n, n), -50.0, dtype=np.float64)
    ll[1, 1] = -1.0   # GT (centre) is the unique maximum
    stats = field_stats(ll.reshape(-1), xs, ys, gt_x=11.0, gt_y=21.0)

    assert stats["gap_nats"] == pytest.approx(0.0)
    assert stats["dist_m"] == pytest.approx(0.0)
    assert stats["local_max"] is True
    assert stats["gt_rank_pct"] == pytest.approx(1.0)


def test_field_stats_shape_mismatch_raises():
    with pytest.raises(ValueError):
        field_stats(np.zeros(10), np.zeros(3), np.zeros(3), 0.0, 0.0)


# ---------------------------------------------------------------------------
# 2. percentile_rank -- known distributions.
# ---------------------------------------------------------------------------

def test_percentile_rank_maximum_is_one():
    values = np.array([1.0, 2.0, 3.0, 4.0, 5.0])
    assert percentile_rank(values, index=4) == pytest.approx(1.0)  # value=5, the max


def test_percentile_rank_minimum():
    values = np.array([1.0, 2.0, 3.0, 4.0, 5.0])
    # value=1 (index 0): only itself is <= 1 -> 1/5
    assert percentile_rank(values, index=0) == pytest.approx(0.2)


def test_percentile_rank_middle_value():
    values = np.array([10.0, 20.0, 30.0, 40.0, 50.0])
    # value=30 (index 2): 30<=30,40<=? no wait <=: values<=30 are 10,20,30 -> 3/5
    assert percentile_rank(values, index=2) == pytest.approx(0.6)


def test_percentile_rank_ties_all_count():
    values = np.array([5.0, 5.0, 5.0, 1.0])
    # value=5.0 (index 0): everything is <=5.0 (three 5's plus the 1.0) -> 4/4
    assert percentile_rank(values, index=0) == pytest.approx(1.0)
    # the 1.0: only itself <= 1.0 -> 1/4
    assert percentile_rank(values, index=3) == pytest.approx(0.25)


# ---------------------------------------------------------------------------
# 3. is_local_max -- true/false cases including ties and edges.
# ---------------------------------------------------------------------------

def test_is_local_max_true_strict_peak():
    grid = np.array([
        [1.0, 1.0, 1.0],
        [1.0, 9.0, 1.0],
        [1.0, 1.0, 1.0],
    ])
    assert is_local_max(grid, 1, 1) is True


def test_is_local_max_false_when_neighbour_higher():
    grid = np.array([
        [1.0, 1.0, 1.0],
        [1.0, 5.0, 10.0],
        [1.0, 1.0, 1.0],
    ])
    assert is_local_max(grid, 1, 1) is False


def test_is_local_max_tie_counts_as_local_max():
    # Equal-valued neighbour: >= means a tie still qualifies.
    grid = np.array([
        [1.0, 1.0, 1.0],
        [1.0, 5.0, 5.0],
        [1.0, 1.0, 1.0],
    ])
    assert is_local_max(grid, 1, 1) is True


def test_is_local_max_corner_cell_uses_only_in_bounds_neighbours():
    grid = np.array([
        [9.0, 1.0],
        [1.0, 1.0],
    ])
    assert is_local_max(grid, 0, 0) is True
    assert is_local_max(grid, 1, 1) is False


def test_is_local_max_single_cell_grid_trivially_true():
    grid = np.array([[3.0]])
    assert is_local_max(grid, 0, 0) is True


# ---------------------------------------------------------------------------
# 4. apply_skip_nonfinite_mask -- semantics required by the brief: when
#    enabled, drop non-finite OBSERVED beams and their paired angles
#    entirely; when disabled, a no-op passthrough.
# ---------------------------------------------------------------------------

def test_skip_nonfinite_false_is_passthrough():
    obs = np.array([1.0, np.inf, 3.0, np.nan])
    ang = np.array([0.1, 0.2, 0.3, 0.4])
    out_obs, out_ang = apply_skip_nonfinite_mask(obs, ang, skip_nonfinite=False)
    np.testing.assert_array_equal(out_obs, obs)
    np.testing.assert_array_equal(out_ang, ang)


def test_skip_nonfinite_true_drops_inf_and_nan_beams_and_angles():
    obs = np.array([1.0, np.inf, 3.0, np.nan, 5.0])
    ang = np.array([0.1, 0.2, 0.3, 0.4, 0.5])
    out_obs, out_ang = apply_skip_nonfinite_mask(obs, ang, skip_nonfinite=True)
    np.testing.assert_allclose(out_obs, [1.0, 3.0, 5.0])
    np.testing.assert_allclose(out_ang, [0.1, 0.3, 0.5])


def test_skip_nonfinite_true_all_finite_is_unchanged():
    obs = np.array([1.0, 2.0, 3.0])
    ang = np.array([0.1, 0.2, 0.3])
    out_obs, out_ang = apply_skip_nonfinite_mask(obs, ang, skip_nonfinite=True)
    np.testing.assert_allclose(out_obs, obs)
    np.testing.assert_allclose(out_ang, ang)


def test_skip_nonfinite_true_all_nonfinite_returns_empty():
    obs = np.array([np.inf, np.nan])
    ang = np.array([0.1, 0.2])
    out_obs, out_ang = apply_skip_nonfinite_mask(obs, ang, skip_nonfinite=True)
    assert out_obs.shape == (0,)
    assert out_ang.shape == (0,)


# ---------------------------------------------------------------------------
# 5. Variant registry / dispatch.
# ---------------------------------------------------------------------------

def test_build_table_upstream_matches_direct_call():
    import plot_sensor_model as psm
    table = build_table("upstream", max_range_px=20, z_hit=0.75, z_short=0.01,
                         z_max=0.07, z_rand=0.12, sigma_px=3.0)
    expected = psm.sensor_model_table(20, 0.75, 0.01, 0.07, 0.12, 3.0)
    np.testing.assert_allclose(table, expected)


def test_build_table_unknown_variant_raises_clear_error():
    with pytest.raises(ValueError, match="unknown --variant"):
        build_table("bogus", 20, 0.75, 0.01, 0.07, 0.12, 3.0)


def test_variant_registry_has_upstream_and_normalized_short():
    assert set(VARIANT_BUILDERS) == {"upstream", "normalized_short"}
    assert VARIANT_BUILDERS["upstream"] is build_upstream_table
    assert VARIANT_BUILDERS["normalized_short"] is build_normalized_short_table


def test_build_table_normalized_short_matches_fork_direct_call():
    fork_path = (
        Path(__file__).resolve().parents[2]
        / "src" / "localization" / "external" / "particle_filter"
    )
    if str(fork_path) not in sys.path:
        sys.path.insert(0, str(fork_path))
    from particle_filter.sensor_model import build_table as fork_build_table

    table = build_table("normalized_short", max_range_px=20, z_hit=0.75, z_short=0.01,
                         z_max=0.07, z_rand=0.12, sigma_px=3.0, lambda_short=0.5)
    expected = fork_build_table(20, 0.75, 0.01, 0.07, 0.12, 3.0,
                                 variant="normalized_short", lambda_short=0.5)
    np.testing.assert_array_equal(table, expected)


def test_build_table_normalized_short_default_lambda_short_is_one():
    table_default = build_table("normalized_short", max_range_px=20, z_hit=0.75,
                                 z_short=0.01, z_max=0.07, z_rand=0.12, sigma_px=3.0)
    table_explicit = build_table("normalized_short", max_range_px=20, z_hit=0.75,
                                  z_short=0.01, z_max=0.07, z_rand=0.12, sigma_px=3.0,
                                  lambda_short=1.0)
    np.testing.assert_array_equal(table_default, table_explicit)


# ---------------------------------------------------------------------------
# 6. summarize -- aggregate median/worst-case direction per field.
# ---------------------------------------------------------------------------

def _fake_result(gap, dist, rank, local_max):
    return {"gap_nats": gap, "dist_m": dist, "gt_rank_pct": rank, "local_max": local_max}


def test_summarize_median_and_worst_case_directions():
    per_timestamp = [
        _fake_result(gap=10.0, dist=1.0, rank=0.9, local_max=True),
        _fake_result(gap=20.0, dist=5.0, rank=0.5, local_max=False),
        _fake_result(gap=30.0, dist=3.0, rank=0.1, local_max=False),
    ]
    summary = summarize(per_timestamp)

    assert summary["n_timestamps"] == 3
    assert summary["gap_nats_median"] == pytest.approx(20.0)
    assert summary["gap_nats_worst"] == pytest.approx(30.0)   # higher gap = worse
    assert summary["dist_m_median"] == pytest.approx(3.0)
    assert summary["dist_m_worst"] == pytest.approx(5.0)      # higher dist = worse
    assert summary["gt_rank_pct_median"] == pytest.approx(0.5)
    assert summary["gt_rank_pct_worst"] == pytest.approx(0.1)  # lower rank = worse
    assert summary["local_max_count"] == 1


# ---------------------------------------------------------------------------
# 7. JSON round-trip -- numpy float64/bool/ndarray survive via NumpyEncoder.
# ---------------------------------------------------------------------------

def test_json_roundtrip_numpy_types_survive(tmp_path):
    result = {
        "variant": "upstream",
        "skip_nonfinite": np.bool_(False),
        "params": {"z_hit": np.float64(0.75), "theta_discretization": np.int64(112)},
        "per_timestamp": [
            {
                "gap_nats": np.float64(47.53),
                "dist_m": np.float64(29.39),
                "gt_rank_pct": np.float64(0.9999),
                "local_max": np.bool_(False),
            }
        ],
        "summary": {"gap_nats_median": np.float64(47.53), "local_max_count": np.int64(0)},
        "some_array": np.array([1.0, 2.0, 3.0]),
    }

    path = tmp_path / "result.json"
    path.write_text(json.dumps(result, cls=NumpyEncoder))
    loaded = json.loads(path.read_text())

    assert loaded["variant"] == "upstream"
    assert loaded["skip_nonfinite"] is False
    assert loaded["params"]["z_hit"] == pytest.approx(0.75)
    assert loaded["params"]["theta_discretization"] == 112
    assert loaded["per_timestamp"][0]["gap_nats"] == pytest.approx(47.53)
    assert loaded["per_timestamp"][0]["local_max"] is False
    assert loaded["summary"]["local_max_count"] == 0
    assert loaded["some_array"] == [1.0, 2.0, 3.0]

    # Confirm these are genuinely JSON-native types now, not numpy.
    assert isinstance(loaded["params"]["z_hit"], float)
    assert isinstance(loaded["summary"]["local_max_count"], int)


def test_json_dump_without_encoder_would_fail_on_numpy_bool():
    # Sanity check that the encoder is actually doing work (i.e. this isn't
    # a vacuous test) -- plain json.dumps chokes on np.bool_.
    with pytest.raises(TypeError):
        json.dumps({"x": np.bool_(True)})
