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
    noreturn_ratio,
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
