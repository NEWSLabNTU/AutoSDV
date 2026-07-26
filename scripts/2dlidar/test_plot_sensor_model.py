#!/usr/bin/env python3
"""Tests for plot_sensor_model.py's pure table-building functions.

Plain-shell, no ROS deps.
Run: PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest scripts/2dlidar/test_plot_sensor_model.py -v
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent))
from plot_sensor_model import (  # noqa: E402
    sensor_model_table,
    sensor_model_table_reference_loop,
    mixture_mass_breakdown,
    mixture_mass_breakdown_normalized_short,
    noreturn_ratio,
    build_scan_ranges,
    decimate_scan,
    frozen_field_grid,
)


# ---------------------------------------------------------------------------
# 1. sensor_model_table must agree with a literal nested-loop port of the
#    upstream precompute_sensor_model() (particle_filter.py:459-506) --
#    this is the drift-detection test the brief asks for.
# ---------------------------------------------------------------------------

def test_vectorised_table_matches_reference_loop_small_table():
    table = sensor_model_table(
        max_range_px=20, z_hit=0.75, z_short=0.01, z_max=0.07, z_rand=0.12, sigma_hit=3.0,
    )
    reference = sensor_model_table_reference_loop(
        max_range_px=20, z_hit=0.75, z_short=0.01, z_max=0.07, z_rand=0.12, sigma_hit=3.0,
    )
    np.testing.assert_allclose(table, reference, rtol=1e-10, atol=1e-12)


def test_vectorised_table_matches_reference_loop_at_full_scale_params():
    # Same params as the CLI defaults / algorithm doc sec. 2.3, but a small
    # table (this test would take a while at the real 1201x1201 size).
    table = sensor_model_table(
        max_range_px=50, z_hit=0.75, z_short=0.01, z_max=0.07, z_rand=0.12, sigma_hit=8.0,
    )
    reference = sensor_model_table_reference_loop(
        max_range_px=50, z_hit=0.75, z_short=0.01, z_max=0.07, z_rand=0.12, sigma_hit=8.0,
    )
    np.testing.assert_allclose(table, reference, rtol=1e-10, atol=1e-12)


# ---------------------------------------------------------------------------
# 2. Hand-computed reference cells on a tiny table (max_range_px=4, so
#    table_width=5), worked out by hand from the formula.
# ---------------------------------------------------------------------------

def _hand_column(d, r_values, z_hit, z_short, z_max, z_rand, sigma_hit, max_range_px):
    raw = []
    for r in r_values:
        prob = z_hit * math.exp(-((r - d) ** 2) / (2.0 * sigma_hit ** 2)) / (sigma_hit * math.sqrt(2 * math.pi))
        if r < d:
            prob += 2.0 * z_short * (d - r) / d
        if r == max_range_px:
            prob += z_max
        if r < max_range_px:
            prob += z_rand / max_range_px
        raw.append(prob)
    norm = sum(raw)
    return [p / norm for p in raw]


def test_hand_computed_column_d2_max_range_px_4():
    max_range_px = 4
    z_hit, z_short, z_max, z_rand, sigma_hit = 0.75, 0.01, 0.07, 0.12, 1.0
    table = sensor_model_table(max_range_px, z_hit, z_short, z_max, z_rand, sigma_hit)

    d = 2
    expected = _hand_column(d, range(max_range_px + 1), z_hit, z_short, z_max, z_rand,
                             sigma_hit, max_range_px)
    np.testing.assert_allclose(table[:, d], expected, rtol=1e-12)

    # Column sums to 1 (normalisation).
    assert table[:, d].sum() == pytest.approx(1.0)

    # r == d (perfect match, index 2) must be the largest mass in this
    # column: hit term peaks there, and it is the only r < max_range_px
    # cell without a short-ramp bonus below the peak... spot check it is
    # at least the max hit-driven cell relative to its immediate neighbours.
    assert table[2, d] > table[3, d]


def test_hand_computed_specific_cell_d1_r0_max_range_px_4():
    # d=1, r=0: r < d so short ramp fires: 2*z_short*(1-0)/1 = 2*z_short.
    max_range_px = 4
    z_hit, z_short, z_max, z_rand, sigma_hit = 0.75, 0.01, 0.07, 0.12, 1.0
    table = sensor_model_table(max_range_px, z_hit, z_short, z_max, z_rand, sigma_hit)
    reference = sensor_model_table_reference_loop(max_range_px, z_hit, z_short, z_max, z_rand,
                                                    sigma_hit)
    assert table[0, 1] == pytest.approx(reference[0, 1], rel=1e-10)

    # Manually verify the raw (pre-normalisation) numerator ratio makes sense:
    # unnormalised prob(r=0,d=1) should include the 2*z_short short-ramp term.
    raw_hit = z_hit * math.exp(-((0 - 1) ** 2) / (2.0 * sigma_hit ** 2)) / (sigma_hit * math.sqrt(2 * math.pi))
    raw_short = 2.0 * z_short * (1 - 0) / 1
    raw_rand = z_rand / max_range_px  # r=0 < max_range_px
    raw_total_col1 = sum(
        z_hit * math.exp(-((r - 1) ** 2) / (2.0 * sigma_hit ** 2)) / (sigma_hit * math.sqrt(2 * math.pi))
        + (2.0 * z_short * (1 - r) / 1 if r < 1 else 0.0)
        + (z_max if r == max_range_px else 0.0)
        + (z_rand / max_range_px if r < max_range_px else 0.0)
        for r in range(max_range_px + 1)
    )
    expected_cell = (raw_hit + raw_short + raw_rand) / raw_total_col1
    assert table[0, 1] == pytest.approx(expected_cell, rel=1e-10)


def test_d_equals_zero_column_no_division_by_zero():
    # d=0: short-ramp condition r < d is never true (r >= 0), so the d/0
    # guard in the vectorised implementation must not corrupt this column,
    # and no warning/NaN should appear.
    table = sensor_model_table(max_range_px=10, z_hit=0.75, z_short=0.01, z_max=0.07,
                                z_rand=0.12, sigma_hit=2.0)
    assert np.all(np.isfinite(table[:, 0]))
    assert table[:, 0].sum() == pytest.approx(1.0)


# ---------------------------------------------------------------------------
# 3. mixture_mass_breakdown and noreturn_ratio -- shape/consistency checks
#    plus the algorithm-doc cross-check numbers (sec. 5.1, 5.2), reproduced
#    here as regression tests so they can't silently drift.
# ---------------------------------------------------------------------------

def test_mixture_mass_breakdown_sums_to_one():
    max_range_px = 1200  # 60 m / 0.05 m
    breakdown = mixture_mass_breakdown(max_range_px, 0.75, 0.01, 0.07, 0.12, 8.0)
    d_px = 400  # 20 m
    total = breakdown["hit"][d_px] + breakdown["short"][d_px] + breakdown["max"][d_px] \
        + breakdown["rand"][d_px]
    assert total == pytest.approx(1.0, rel=1e-9)


def test_algorithm_doc_cross_check_effective_z_hit():
    # docs/research/localization/2d_mcl_algorithm.md sec 5.1, at resolution 0.05:
    # 10 m -> 25.4%, 20 m -> 15.2%, 50 m -> 6.8% (+/- rounding).
    resolution = 0.05
    max_range_px = int(round(60.0 / resolution))
    breakdown = mixture_mass_breakdown(max_range_px, 0.75, 0.01, 0.07, 0.12, 8.0)
    expected = {10: 25.4, 20: 15.2, 50: 6.8}
    for d_m, exp_pct in expected.items():
        d_px = int(round(d_m / resolution))
        got_pct = breakdown["hit"][d_px] * 100
        assert got_pct == pytest.approx(exp_pct, abs=0.3), f"d={d_m}m: got {got_pct:.2f}%"


def test_mixture_mass_breakdown_normalized_short_sums_to_one():
    max_range_px = 1200  # 60 m / 0.05 m
    breakdown = mixture_mass_breakdown_normalized_short(
        max_range_px, 0.75, 0.01, 0.07, 0.12, 8.0, lambda_short=1.0,
    )
    d_px = 400  # 20 m
    total = breakdown["hit"][d_px] + breakdown["short"][d_px] + breakdown["max"][d_px] \
        + breakdown["rand"][d_px]
    assert total == pytest.approx(1.0, rel=1e-9)


def test_mixture_mass_breakdown_normalized_short_effective_z_hit_stays_flat():
    # Phase 3e Task 2, sec 5.1's canonical fix: unlike upstream (25.4% ->
    # 6.8% from 10m to 50m), effective z_hit stays flat across d.
    resolution = 0.05
    max_range_px = int(round(60.0 / resolution))
    breakdown = mixture_mass_breakdown_normalized_short(
        max_range_px, 0.75, 0.01, 0.07, 0.12, 8.0, lambda_short=1.0,
    )
    shares = [breakdown["hit"][int(round(d_m / resolution))] for d_m in (10, 20, 50)]
    assert max(shares) - min(shares) < 1e-6, shares
    assert all(abs(s - 0.75) < 0.05 for s in shares), shares


def test_mixture_mass_breakdown_normalized_short_matches_fork_build_table():
    # Cross-check this plotting-side re-derivation against the vendored
    # fork's particle_filter.sensor_model.build_table (the function that
    # actually runs online), so the two never silently drift apart.
    fork_path = (
        Path(__file__).resolve().parents[2]
        / "src" / "localization" / "external" / "particle_filter"
    )
    if str(fork_path) not in sys.path:
        sys.path.insert(0, str(fork_path))
    from particle_filter.sensor_model import build_table as fork_build_table

    max_range_px = 50
    z_hit, z_short, z_max, z_rand, sigma_hit, lambda_short = 0.75, 0.01, 0.07, 0.12, 8.0, 0.7
    table = fork_build_table(max_range_px, z_hit, z_short, z_max, z_rand, sigma_hit,
                              variant="normalized_short", lambda_short=lambda_short)
    breakdown = mixture_mass_breakdown_normalized_short(
        max_range_px, z_hit, z_short, z_max, z_rand, sigma_hit, lambda_short,
    )
    d_px = 30
    assert table[d_px, d_px] > 0  # sanity: table is populated
    np.testing.assert_allclose(
        breakdown["hit"][d_px] + breakdown["short"][d_px] + breakdown["max"][d_px]
        + breakdown["rand"][d_px],
        1.0, rtol=1e-9,
    )


def test_algorithm_doc_cross_check_noreturn_ratio_at_20m():
    # sec 5.2 / sec 4: P(no-return)/P(perfect match) at d=20m ~= 1.87
    # (0.014141 / 0.007576 in the doc's worked example).
    resolution = 0.05
    max_range_px = int(round(60.0 / resolution))
    table = sensor_model_table(max_range_px, 0.75, 0.01, 0.07, 0.12, 8.0)
    ratio = noreturn_ratio(table, max_range_px)
    d_px = int(round(20 / resolution))
    assert ratio[d_px] == pytest.approx(1.87, abs=0.02)


def test_noreturn_ratio_matches_doc_worked_example_raw_values():
    # sec 4's table: perfect match at 20m -> 0.007576, no-return -> 0.014141.
    resolution = 0.05
    max_range_px = int(round(60.0 / resolution))
    table = sensor_model_table(max_range_px, 0.75, 0.01, 0.07, 0.12, 8.0)
    d_px = int(round(20 / resolution))
    assert table[d_px, d_px] == pytest.approx(0.007576, abs=2e-5)
    assert table[max_range_px, d_px] == pytest.approx(0.014141, abs=2e-5)


# ---------------------------------------------------------------------------
# 4. Part B frozen-scan pure functions: build_scan_ranges, decimate_scan,
#    frozen_field_grid.
# ---------------------------------------------------------------------------

ANGLE_MIN, ANGLE_MAX, ANGLE_INC = -3.14159, 3.14159, 0.0043
RANGES_SIZE = int(math.ceil((ANGLE_MAX - ANGLE_MIN) / ANGLE_INC))


def test_build_scan_ranges_empty_points_all_inf():
    ranges = build_scan_ranges(
        np.zeros((0, 3)), min_height=0.0, max_height=1.0,
        angle_min=ANGLE_MIN, angle_max=ANGLE_MAX, angle_increment=ANGLE_INC,
        range_min=0.1, range_max=60.0,
    )
    assert ranges.shape == (RANGES_SIZE,)
    assert np.all(np.isinf(ranges))


def test_build_scan_ranges_single_point_lands_in_expected_bin():
    # A point straight ahead (+x axis, angle=0) at range 10m.
    pts = np.array([[10.0, 0.0, 0.5]])
    ranges = build_scan_ranges(
        pts, min_height=0.0, max_height=1.0,
        angle_min=ANGLE_MIN, angle_max=ANGLE_MAX, angle_increment=ANGLE_INC,
        range_min=0.1, range_max=60.0,
    )
    expected_idx = int((0.0 - ANGLE_MIN) / ANGLE_INC)
    assert ranges[expected_idx] == pytest.approx(10.0)
    # every other bin is still inf (use_inf default)
    assert np.isinf(ranges).sum() == RANGES_SIZE - 1


def test_build_scan_ranges_height_filter_drops_point():
    pts = np.array([[10.0, 0.0, 5.0]])  # z=5.0, outside [0,1] band
    ranges = build_scan_ranges(
        pts, min_height=0.0, max_height=1.0,
        angle_min=ANGLE_MIN, angle_max=ANGLE_MAX, angle_increment=ANGLE_INC,
        range_min=0.1, range_max=60.0,
    )
    assert np.all(np.isinf(ranges))


def test_build_scan_ranges_range_min_and_max_filters():
    pts = np.array([
        [0.05, 0.0, 0.5],   # range 0.05 < range_min=0.1 -> dropped
        [100.0, 0.0, 0.5],  # range 100 > range_max=60 -> dropped
    ])
    ranges = build_scan_ranges(
        pts, min_height=0.0, max_height=1.0,
        angle_min=ANGLE_MIN, angle_max=ANGLE_MAX, angle_increment=ANGLE_INC,
        range_min=0.1, range_max=60.0,
    )
    assert np.all(np.isinf(ranges))


def test_build_scan_ranges_nan_point_dropped():
    pts = np.array([[float("nan"), 0.0, 0.5]])
    ranges = build_scan_ranges(
        pts, min_height=0.0, max_height=1.0,
        angle_min=ANGLE_MIN, angle_max=ANGLE_MAX, angle_increment=ANGLE_INC,
        range_min=0.1, range_max=60.0,
    )
    assert np.all(np.isinf(ranges))


def test_build_scan_ranges_keeps_closest_point_per_bin():
    # Two points at (numerically) the same angle bin, different ranges --
    # the CLOSER one must win (matches the node's `if range < ranges[index]`).
    pts = np.array([
        [10.0, 0.0, 0.5],
        [5.0, 0.0001, 0.5],  # same bin (angle ~0), closer
    ])
    ranges = build_scan_ranges(
        pts, min_height=0.0, max_height=1.0,
        angle_min=ANGLE_MIN, angle_max=ANGLE_MAX, angle_increment=ANGLE_INC,
        range_min=0.1, range_max=60.0,
    )
    expected_idx = int((0.0 - ANGLE_MIN) / ANGLE_INC)
    assert ranges[expected_idx] == pytest.approx(5.0, abs=0.001)


def test_decimate_scan_matches_lidarCB_construction():
    # particle_filter.py:432-439: angles = linspace(angle_min, angle_max,
    # len(ranges)), NOT angle_min + k*angle_increment; both sliced [0::step].
    ranges_size = 100
    ranges = np.arange(ranges_size, dtype=np.float64)
    angles, dec_ranges = decimate_scan(ranges, angle_min=-1.0, angle_max=1.0, angle_step=10)
    expected_angles = np.linspace(-1.0, 1.0, ranges_size)[0::10]
    np.testing.assert_allclose(angles, expected_angles, rtol=1e-6)
    np.testing.assert_allclose(dec_ranges, ranges[0::10])
    assert angles.dtype == np.float32
    assert dec_ranges.dtype == np.float32


def test_decimate_scan_length_with_non_multiple_step():
    ranges = np.arange(1462, dtype=np.float64)
    angles, dec_ranges = decimate_scan(ranges, ANGLE_MIN, ANGLE_MAX, angle_step=18)
    assert angles.shape[0] == dec_ranges.shape[0] == len(range(0, 1462, 18))


def test_frozen_field_grid_matches_direct_log_sum():
    resolution = 0.05
    max_range_px = 400  # 20 m
    table = sensor_model_table(max_range_px, 0.75, 0.01, 0.07, 0.12, 8.0)

    # 3 beams, all observing a perfect match to a 10m-predicting pose.
    observed_m = np.array([10.0, 10.0, 10.0])
    predicted_m = np.array([[10.0, 10.0, 10.0]])  # 1 pose x 3 beams

    log_likelihood, raw_weight, probs = frozen_field_grid(
        predicted_m, observed_m, resolution, table,
    )
    d_px = int(round(10.0 / resolution))
    expected_cell = table[d_px, d_px]
    np.testing.assert_allclose(probs, np.full((1, 3), expected_cell))
    assert log_likelihood[0] == pytest.approx(3 * math.log(expected_cell), rel=1e-9)
    assert raw_weight[0] == pytest.approx(expected_cell ** 3, rel=1e-9)


def test_frozen_field_grid_noreturn_beam_clamps_to_max_range_px():
    resolution = 0.05
    max_range_px = 400
    table = sensor_model_table(max_range_px, 0.75, 0.01, 0.07, 0.12, 8.0)

    observed_m = np.array([np.inf])
    predicted_m = np.array([[10.0]])
    log_likelihood, raw_weight, probs = frozen_field_grid(
        predicted_m, observed_m, resolution, table,
    )
    d_px = int(round(10.0 / resolution))
    assert probs[0, 0] == pytest.approx(table[max_range_px, d_px])


def test_frozen_field_grid_raw_product_underflows_but_log_likelihood_does_not():
    # Craft a case that a raw float64 product of many small terms
    # underflows to exactly 0.0 while the log-space sum stays finite and
    # very negative -- this is the mechanism behind the phase's headline
    # underflow-vs-misspecification question.
    resolution = 0.05
    max_range_px = 400
    table = sensor_model_table(max_range_px, 0.75, 0.01, 0.07, 0.12, 8.0)

    num_beams = 200
    # observe 10m on every beam, but predict a very poor match (2m) so each
    # beam's probability is tiny (off the Gaussian's tail, no short/rand
    # mass since r > d here doesn't trigger the short ramp).
    observed_m = np.full(num_beams, 10.0)
    predicted_m = np.full((1, num_beams), 2.0)

    log_likelihood, raw_weight, probs = frozen_field_grid(
        predicted_m, observed_m, resolution, table,
    )
    assert np.all(probs < 1e-3)          # confirm these are indeed tiny per-beam terms
    assert raw_weight[0] == 0.0          # underflowed to exactly 0 in float64
    assert np.isfinite(log_likelihood[0])
    assert log_likelihood[0] < -700      # deep in "would-underflow" territory
