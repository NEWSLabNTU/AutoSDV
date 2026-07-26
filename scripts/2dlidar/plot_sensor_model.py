#!/usr/bin/env python3
"""Render diagnostic charts for the vendored particle_filter's beam sensor model.

Phase 3d Task 5 (Part A). This script rebuilds the sensor-model lookup table
with the *same* four-component mixture formula as `precompute_sensor_model`
in `src/localization/external/particle_filter/particle_filter/particle_filter.py`
(lines 459-506) and renders four PNG charts that make the structural defects
documented in `docs/research/localization/2d_mcl_algorithm.md` sections 3.1,
4, 5.1 and 5.2 visible:

  1. sensor-model-columns          -- normalised P(r|d) for d = 10/20/50 m
  2. sensor-model-mixture-mass     -- effective mixture-weight breakdown (the
                                       section 5.1 table, as a chart)
  3. sensor-model-noreturn-ratio   -- P(no-return) / P(perfect match) (5.2)
  4. sensor-model-resolution-coupling -- (2) and (3) at two resolutions

Pure/table-building functions (`sensor_model_table`, `mixture_mass_breakdown`,
`noreturn_ratio`) import nothing beyond numpy and are unit tested in
`test_plot_sensor_model.py` under a plain interpreter, no ROS required. The
CLI additionally needs matplotlib (Agg backend, set below) to render PNGs.

Run:
    python3 scripts/2dlidar/plot_sensor_model.py \
        --resolution 0.05 --max-range 60.0 --out-dir docs/reports/assets

Phase 3d Task 5 (Part B) additionally implements `--frozen-scan`: reconstruct
one LaserScan exactly as `run-particle-filter.sh` does (pointcloud_to_laserscan's
z-band + azimuth-binning loop, replicated in `build_scan_ranges`, then
particle_filter's `angle_step` decimation in `decimate_scan`), evaluate a
fine pose grid around the recorded ground-truth pose through `range_libc`
directly, and render the LOG-SPACE (sum of log per-beam probabilities, never
a product) likelihood field. This is deliberately NOT what the live
`/pf/debug/likelihood_field` grid computes (`eval_sensor_model` in
`range_libc/includes/RangeLib.h:533` accumulates a raw float64 *product*
over ~82 beams, which underflows to exactly 0.0 for any pose whose fit is
merely mediocre -- see `frozen_field_grid`'s `raw_weight` output, used only
to quantify that underflow, never to render). `build_scan_ranges`,
`decimate_scan` and `frozen_field_grid` import nothing beyond numpy and are
unit tested alongside the Part A functions. The bag/map/range_libc-dependent
glue (`reconstruct_frozen_scan`, `load_map_occupancy_grid`,
`evaluate_frozen_field`) requires a sourced ROS/Autoware environment; run it
via:
    bash -c 'source /opt/autoware/1.5.0/setup.bash && python3 \
        scripts/2dlidar/plot_sensor_model.py --frozen-scan <gt-bag> \
        --map <yaml> --time <rel_s> --label <name> --out-dir docs/reports/assets'
"""
import argparse
import math
import sys
from pathlib import Path

import numpy as np

# ---------------------------------------------------------------------------
# Fixed categorical palette (seaborn "deep"-derived, reasonably CVD-distinct).
# Assigned by entity (hit/short/max/rand, or resolution), never by rank.
# ---------------------------------------------------------------------------
COLOR_HIT = "#4C72B0"     # blue
COLOR_SHORT = "#DD8452"   # orange
COLOR_MAX = "#55A868"     # green
COLOR_RAND = "#C44E52"    # red
COLOR_RES_A = "#4C72B0"   # blue -- finer resolution (0.05)
COLOR_RES_B = "#DD8452"   # orange -- coarser resolution (0.10)
COLOR_D10 = "#4C72B0"
COLOR_D20 = "#DD8452"
COLOR_D50 = "#55A868"
GRID_KW = dict(color="0.85", linewidth=0.6, zorder=0)


# ---------------------------------------------------------------------------
# Sensor-model table -- duplicated formula.
#
# Source of truth: precompute_sensor_model(), particle_filter.py:459-506
# (src/localization/external/particle_filter/particle_filter/particle_filter.py).
# That method loops `for d in range(table_width): for r in range(table_width):`
# in pure Python (table_width = MAX_RANGE_PX + 1) and computes, per cell:
#
#   prob  = z_hit * exp(-(r-d)^2 / (2*sigma^2)) / (sigma*sqrt(2*pi))
#   if r < d:                 prob += 2*z_short*(d-r)/d
#   if r == MAX_RANGE_PX:     prob += z_max
#   if r <  MAX_RANGE_PX:     prob += z_rand / MAX_RANGE_PX
#   table[r, d] = prob
#   (then each column d is divided by its own sum)
#
# `sensor_model_table` below is a vectorised re-expression of the exact same
# arithmetic (same terms, same conditions, same per-column normalisation);
# `test_plot_sensor_model.py` checks it against a literal nested-loop port of
# the upstream method on a small table so any future drift between the two
# is caught.
# ---------------------------------------------------------------------------

def sensor_model_table(max_range_px, z_hit, z_short, z_max, z_rand, sigma_hit):
    """Build the normalised (r, d) sensor-model table in PIXEL units.

    Returns a float64 array of shape (table_width, table_width) where
    table_width = max_range_px + 1, table[r, d] = P(observe r | predicted d),
    each column normalised to sum to 1 (matches particle_filter.py:459-506).
    """
    if max_range_px < 1:
        raise ValueError("sensor_model_table: max_range_px must be >= 1")
    table_width = int(max_range_px) + 1
    r_idx = np.arange(table_width, dtype=np.float64)[:, None]   # rows: observed r
    d_idx = np.arange(table_width, dtype=np.float64)[None, :]   # cols: predicted d

    diff = r_idx - d_idx
    prob = z_hit * np.exp(-(diff * diff) / (2.0 * sigma_hit * sigma_hit)) \
        / (sigma_hit * np.sqrt(2.0 * np.pi))

    short_mask = r_idx < d_idx
    with np.errstate(divide="ignore", invalid="ignore"):
        short_term = np.where(
            short_mask,
            2.0 * z_short * (d_idx - r_idx) / np.where(d_idx == 0, 1.0, d_idx),
            0.0,
        )
    prob = prob + short_term

    max_mask = (r_idx == float(max_range_px))
    prob = prob + np.where(max_mask, z_max, 0.0)

    rand_mask = r_idx < float(max_range_px)
    prob = prob + np.where(rand_mask, z_rand / float(max_range_px), 0.0)

    norm = prob.sum(axis=0, keepdims=True)
    return prob / norm


def sensor_model_table_reference_loop(max_range_px, z_hit, z_short, z_max, z_rand, sigma_hit):
    """Literal nested-loop port of precompute_sensor_model's inner math.

    Used only by tests, as an independent (slow, obviously-correct) oracle
    for `sensor_model_table` on small tables.
    """
    table_width = int(max_range_px) + 1
    table = np.zeros((table_width, table_width), dtype=np.float64)
    for d in range(table_width):
        norm = 0.0
        for r in range(table_width):
            prob = 0.0
            z = float(r - d)
            prob += z_hit * np.exp(-(z * z) / (2.0 * sigma_hit * sigma_hit)) \
                / (sigma_hit * np.sqrt(2.0 * np.pi))
            if r < d:
                prob += 2.0 * z_short * (d - r) / float(d)
            if int(r) == int(max_range_px):
                prob += z_max
            if r < int(max_range_px):
                prob += z_rand * 1.0 / float(max_range_px)
            norm += prob
            table[int(r), int(d)] = prob
        table[:, int(d)] /= norm
    return table


def mixture_mass_breakdown(max_range_px, z_hit, z_short, z_max, z_rand, sigma_hit):
    """Effective (post-normalisation) mass of each of the 4 components, per column d.

    Returns a dict of 4 arrays (each shape (table_width,)), keys
    "hit"/"short"/"max"/"rand", each holding that component's share of the
    normalised column -- i.e. the *effective* mixture weight, as opposed to
    the configured z_hit/z_short/z_max/z_rand. This is the section 5.1 table.
    """
    table_width = int(max_range_px) + 1
    r_idx = np.arange(table_width, dtype=np.float64)[:, None]
    d_idx = np.arange(table_width, dtype=np.float64)[None, :]

    diff = r_idx - d_idx
    hit_term = z_hit * np.exp(-(diff * diff) / (2.0 * sigma_hit * sigma_hit)) \
        / (sigma_hit * np.sqrt(2.0 * np.pi))

    short_mask = r_idx < d_idx
    with np.errstate(divide="ignore", invalid="ignore"):
        short_term = np.where(
            short_mask,
            2.0 * z_short * (d_idx - r_idx) / np.where(d_idx == 0, 1.0, d_idx),
            0.0,
        )

    max_mask = (r_idx == float(max_range_px))
    max_term = np.where(max_mask, z_max, 0.0)

    rand_mask = r_idx < float(max_range_px)
    rand_term = np.where(rand_mask, z_rand / float(max_range_px), 0.0)

    total = (hit_term + short_term + max_term + rand_term).sum(axis=0)
    return {
        "hit": hit_term.sum(axis=0) / total,
        "short": short_term.sum(axis=0) / total,
        "max": max_term.sum(axis=0) / total,
        "rand": rand_term.sum(axis=0) / total,
    }


def mixture_mass_breakdown_normalized_short(max_range_px, z_hit, z_short, z_max, z_rand,
                                             sigma_hit, lambda_short=1.0):
    """Same as `mixture_mass_breakdown`, but for the Phase 3e Task 2
    "normalized_short" variant (docs/research/localization/
    2d_mcl_algorithm.md sec 5.1's canonical fix): the short-reading term is
    a per-column-normalised truncated exponential,
    `eta * lambda_short * exp(-lambda_short * r)` for `0 <= r <= d`, whose
    mass no longer scales with `d` the way the upstream ramp's does. This
    is a standalone re-derivation for plotting (matches
    `particle_filter.sensor_model.build_table(..., variant="normalized_short")`
    in the vendored fork; see `test_plot_sensor_model.py` for the
    cross-check against the fork's implementation).
    """
    table_width = int(max_range_px) + 1
    r_idx = np.arange(table_width, dtype=np.float64)[:, None]
    d_idx = np.arange(table_width, dtype=np.float64)[None, :]

    diff = r_idx - d_idx
    hit_term = z_hit * np.exp(-(diff * diff) / (2.0 * sigma_hit * sigma_hit)) \
        / (sigma_hit * np.sqrt(2.0 * np.pi))

    short_mask = r_idx < d_idx
    with np.errstate(divide="ignore", invalid="ignore"):
        denom = 1.0 - np.exp(-lambda_short * d_idx)
        eta = np.where(d_idx == 0, 0.0, 1.0 / np.where(d_idx == 0, 1.0, denom))
        short_term = np.where(
            short_mask,
            z_short * eta * lambda_short * np.exp(-lambda_short * r_idx),
            0.0,
        )

    max_mask = (r_idx == float(max_range_px))
    max_term = np.where(max_mask, z_max, 0.0)

    rand_mask = r_idx < float(max_range_px)
    rand_term = np.where(rand_mask, z_rand / float(max_range_px), 0.0)

    total = (hit_term + short_term + max_term + rand_term).sum(axis=0)
    return {
        "hit": hit_term.sum(axis=0) / total,
        "short": short_term.sum(axis=0) / total,
        "max": max_term.sum(axis=0) / total,
        "rand": rand_term.sum(axis=0) / total,
    }


def noreturn_ratio(table, max_range_px):
    """P(no-return) / P(perfect match), per column d.

    `inf / world_scale` clamps to MAX_RANGE_PX (RangeLib.h:533 in the algorithm
    doc's section 3.2), so "no return" reads as r = max_range_px; "perfect
    match" reads as r = d. Column d = 0 is excluded (P(perfect match) there
    equals P(no-return) trivially only at d=0, not diagnostic).
    """
    table_width = table.shape[0]
    d = np.arange(table_width)
    no_return = table[max_range_px, :]
    perfect = table[d, d]
    with np.errstate(divide="ignore", invalid="ignore"):
        ratio = np.where(perfect > 0, no_return / perfect, np.nan)
    return ratio


# ---------------------------------------------------------------------------
# Part B: frozen-scan log-space likelihood field -- pure functions.
#
# These three functions import nothing beyond numpy and are unit-tested in
# test_plot_sensor_model.py under a plain interpreter. The ROS/bag/
# range_libc-dependent glue that calls them lives further below, inside
# functions only (never imported at module load time).
# ---------------------------------------------------------------------------

def build_scan_ranges(points_frame_xyz, min_height, max_height,
                       angle_min, angle_max, angle_increment,
                       range_min, range_max):
    """Re-implement pointcloud_to_laserscan's cloudCallback binning loop
    (external/pointcloud_to_laserscan's pointcloud_to_laserscan_node.cpp,
    ~lines 256-311) in vectorised numpy, given points ALREADY transformed
    into the scan's target frame (base_link, for run-particle-filter.sh's
    `target_frame:=base_link`).

    Per-point filter order, matching the C++ source exactly (order matters
    only in that every check is a `continue`, i.e. an AND across all of
    them -- reproduced here as a single combined mask):
      1. drop non-finite x/y/z
      2. drop z outside [min_height, max_height]
      3. drop range = hypot(x,y) outside [range_min, range_max]
      4. drop azimuth = atan2(y,x) outside [angle_min, angle_max]
      5. bin index = int((azimuth - angle_min) / angle_increment) (truncation
         toward zero == floor since azimuth >= angle_min here); keep the
         MINIMUM range per bin (closest obstacle wins).

    Returns a float64 array of length `ranges_size =
    ceil((angle_max - angle_min) / angle_increment)`, `inf` in every bin no
    surviving point landed in (matches the node's `use_inf=True` default).
    """
    pts = np.asarray(points_frame_xyz, dtype=np.float64)
    ranges_size = int(np.ceil((angle_max - angle_min) / angle_increment))
    ranges = np.full(ranges_size, np.inf, dtype=np.float64)
    if pts.shape[0] == 0:
        return ranges

    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    in_height = (z <= max_height) & (z >= min_height)
    r = np.hypot(x, y)
    in_range = (r >= range_min) & (r <= range_max)
    angle = np.arctan2(y, x)
    in_angle = (angle >= angle_min) & (angle <= angle_max)
    keep = finite & in_height & in_range & in_angle
    if not np.any(keep):
        return ranges

    idx = ((angle[keep] - angle_min) / angle_increment).astype(np.int64)
    rr = r[keep]
    valid = idx < ranges_size
    idx, rr = idx[valid], rr[valid]
    np.minimum.at(ranges, idx, rr)
    return ranges


def decimate_scan(ranges, angle_min, angle_max, angle_step):
    """Reproduce particle_filter's `lidarCB` decimation exactly
    (particle_filter.py:432-439): angles are `linspace(angle_min, angle_max,
    len(ranges))` -- NOT `angle_min + k*angle_increment` -- then both angles
    and ranges are sliced `[0::angle_step]`.

    Returns (angles, ranges), both float32, length
    ceil(len(ranges) / angle_step).
    """
    ranges = np.asarray(ranges)
    ranges_size = ranges.shape[0]
    angles = np.linspace(angle_min, angle_max, ranges_size)
    return (
        angles[0::angle_step].astype(np.float32),
        ranges[0::angle_step].astype(np.float32),
    )


def frozen_field_grid(predicted_ranges_m, observed_ranges_m, resolution, table):
    """Evaluate a pose grid's per-beam sensor-model lookups and accumulate
    them TWO ways, to directly settle underflow (float64 raw product,
    RangeLib.h:533's `eval_sensor_model`) vs. mis-specification (the
    likelihood surface's actual shape):

      log_likelihood -- sum of log(P(r|d)) per pose, in nats. Underflow-free
                         by construction (a sum can't underflow to 0 the way
                         a product of ~82 small terms can); this is what
                         gets rendered.
      raw_weight      -- product of P(r|d) per pose, replicating
                         `eval_sensor_model`'s exact float64 accumulation
                         (`weight *= sensor_model[r][d]`). Computed only so
                         its underflow-to-exactly-0.0 rate can be measured
                         against the live renders, never rendered itself.

    `predicted_ranges_m`: (num_poses, num_beams) float array, metres
      (range_libc's calc_range_repeat_angles output for the pose grid).
    `observed_ranges_m`: (num_beams,) float array, metres, `inf` allowed.
    `resolution`: map resolution, m/px -- the SAME metres->pixels conversion
      RangeLib.h:533 applies (`r = obs/world_scale`, clamped to
      [0, table_width-1], then truncated to int) before indexing the table;
      reproduced here identically so the two accumulations are evaluated on
      literally the same per-beam lookups.
    `table`: the (table_width, table_width) normalised sensor-model table
      from `sensor_model_table` (rows=observed r index, cols=predicted d
      index), at the SAME resolution/max_range as this call's clamp.

    Returns (log_likelihood (num_poses,), raw_weight (num_poses,),
    probs (num_poses, num_beams) -- the per-beam table lookups themselves,
    for further inspection/testing).
    """
    predicted = np.asarray(predicted_ranges_m, dtype=np.float64)
    observed = np.asarray(observed_ranges_m, dtype=np.float64)
    table_width = table.shape[0]

    with np.errstate(invalid="ignore"):
        obs_idx = np.clip(observed / resolution, 0, table_width - 1).astype(np.int64)
        pred_idx = np.clip(predicted / resolution, 0, table_width - 1).astype(np.int64)

    probs = table[obs_idx[None, :], pred_idx]          # (num_poses, num_beams)
    log_likelihood = np.log(probs).sum(axis=1)          # nats -- underflow-free
    raw_weight = np.prod(probs, axis=1)                 # replicates eval_sensor_model
    return log_likelihood, raw_weight, probs


# ---------------------------------------------------------------------------
# Plotting.
# ---------------------------------------------------------------------------

def _mpl():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    return plt


def _param_str(resolution, max_range, z_hit, z_short, z_max, z_rand, sigma_px):
    return (
        f"resolution={resolution:g} m/px, max_range={max_range:g} m, "
        f"z_hit={z_hit:g}, z_short={z_short:g}, z_max={z_max:g}, z_rand={z_rand:g}, "
        f"sigma={sigma_px:g} px ({sigma_px * resolution:g} m)"
    )


def plot_columns(ax, resolution, max_range, z_hit, z_short, z_max, z_rand, sigma_px):
    max_range_px = int(round(max_range / resolution))
    table = sensor_model_table(max_range_px, z_hit, z_short, z_max, z_rand, sigma_px)
    r_m = np.arange(table.shape[0]) * resolution

    for d_m, color in ((10, COLOR_D10), (20, COLOR_D20), (50, COLOR_D50)):
        d_px = int(round(d_m / resolution))
        ax.plot(r_m, table[:, d_px], color=color, linewidth=2, label=f"d = {d_m} m", zorder=3)

    ax.set_yscale("log")
    ax.set_xlabel("observed range r (m)")
    ax.set_ylabel("P(r | d)  (normalised column, log scale)")
    ax.set_title("Sensor-model columns: P(observed | predicted)", pad=34)
    ax.text(
        0.5, 1.06,
        "Peak at r=d is the Gaussian hit term; the shoulder for r<d is the\n"
        "unnormalised short-reading ramp, which widens and flattens the peak as d grows.",
        transform=ax.transAxes, ha="center", va="bottom", fontsize=8, color="0.35",
    )
    ax.grid(True, **GRID_KW)
    ax.legend(frameon=False)


def plot_mixture_mass(ax, resolution, max_range, z_hit, z_short, z_max, z_rand, sigma_px,
                       d_range_m=(5, 55)):
    max_range_px = int(round(max_range / resolution))
    breakdown = mixture_mass_breakdown(max_range_px, z_hit, z_short, z_max, z_rand, sigma_px)

    d_lo_px = int(round(d_range_m[0] / resolution))
    d_hi_px = int(round(d_range_m[1] / resolution))
    d_px = np.arange(d_lo_px, d_hi_px + 1)
    d_m = d_px * resolution

    ax.stackplot(
        d_m,
        breakdown["hit"][d_px] * 100,
        breakdown["short"][d_px] * 100,
        breakdown["max"][d_px] * 100,
        breakdown["rand"][d_px] * 100,
        colors=[COLOR_HIT, COLOR_SHORT, COLOR_MAX, COLOR_RAND],
        labels=["hit (Gaussian)", "short ramp", "max-range spike", "uniform random"],
        zorder=3,
    )
    ax.set_xlabel("predicted range d (m)")
    ax.set_ylabel("effective mixture-weight share of column (%)")
    ax.set_ylim(0, 100)
    ax.set_title("Effective mixture-weight breakdown vs predicted range", pad=34)
    ax.text(
        0.5, 1.06,
        "Configured weights are z_hit=%.2f/z_short=%.2f/z_max=%.2f/z_rand=%.2f; the\n"
        "unnormalised short ramp swallows most of the column mass as d grows (sec. 5.1)."
        % (z_hit, z_short, z_max, z_rand),
        transform=ax.transAxes, ha="center", va="bottom", fontsize=8, color="0.35",
    )
    ax.grid(True, axis="y", **GRID_KW)
    ax.legend(loc="center right", frameon=False, fontsize=8)


def plot_mixture_mass_comparison(axes, resolution, max_range, z_hit, z_short, z_max, z_rand,
                                  sigma_px, lambda_short=1.0, d_range_m=(5, 55)):
    """Phase 3e Task 2: upstream vs normalized_short mixture-mass
    breakdown, side by side, so the effective-weight repair (sec 5.1) is
    directly visible -- configured z_hit=0.75 degrades to 6.8% by 50m
    under "upstream" but stays close to 0.75 across the whole range under
    "normalized_short". `axes` is a length-2 sequence of matplotlib Axes.
    """
    max_range_px = int(round(max_range / resolution))
    upstream = mixture_mass_breakdown(max_range_px, z_hit, z_short, z_max, z_rand, sigma_px)
    normalized = mixture_mass_breakdown_normalized_short(
        max_range_px, z_hit, z_short, z_max, z_rand, sigma_px, lambda_short,
    )

    d_lo_px = int(round(d_range_m[0] / resolution))
    d_hi_px = int(round(d_range_m[1] / resolution))
    d_px = np.arange(d_lo_px, d_hi_px + 1)
    d_m = d_px * resolution

    for ax, breakdown, title in (
        (axes[0], upstream, "upstream (unnormalised p_short)"),
        (axes[1], normalized, f"normalized_short (lambda_short={lambda_short:g} /px)"),
    ):
        ax.stackplot(
            d_m,
            breakdown["hit"][d_px] * 100,
            breakdown["short"][d_px] * 100,
            breakdown["max"][d_px] * 100,
            breakdown["rand"][d_px] * 100,
            colors=[COLOR_HIT, COLOR_SHORT, COLOR_MAX, COLOR_RAND],
            labels=["hit (Gaussian)", "short", "max-range", "rand"],
            zorder=3,
        )
        ax.axhline(z_hit * 100, color="0.2", linewidth=1.2, linestyle="--", zorder=4,
                   label=f"configured z_hit={z_hit:g}")
        ax.set_xlabel("predicted range d (m)")
        ax.set_ylim(0, 100)
        ax.set_title(title, fontsize=10)
        ax.grid(True, axis="y", **GRID_KW)

    axes[0].set_ylabel("effective mixture-weight share of column (%)")
    axes[1].legend(loc="center right", frameon=False, fontsize=8)


def plot_noreturn_ratio(ax, resolution, max_range, z_hit, z_short, z_max, z_rand, sigma_px,
                         d_range_m=(5, 55)):
    max_range_px = int(round(max_range / resolution))
    table = sensor_model_table(max_range_px, z_hit, z_short, z_max, z_rand, sigma_px)
    ratio = noreturn_ratio(table, max_range_px)

    d_lo_px = max(1, int(round(d_range_m[0] / resolution)))
    d_hi_px = int(round(d_range_m[1] / resolution))
    d_px = np.arange(d_lo_px, d_hi_px + 1)
    d_m = d_px * resolution

    ax.plot(d_m, ratio[d_px], color=COLOR_HIT, linewidth=2, zorder=3)
    ax.axhline(1.0, color=COLOR_RAND, linewidth=1.5, linestyle="--", zorder=2,
               label="break-even (no-return == perfect match)")
    ax.set_xlabel("predicted range d (m)")
    ax.set_ylabel("P(no-return) / P(perfect match)")
    ax.set_title("A missing beam vs. a matching beam", pad=34)
    ax.text(
        0.5, 1.06,
        "Above the dashed line, a beam that returns nothing outscores one that hits\n"
        "exactly where predicted -- the filter is rewarded for predicting empty scans (sec. 5.2).",
        transform=ax.transAxes, ha="center", va="bottom", fontsize=8, color="0.35",
    )
    ax.grid(True, **GRID_KW)
    ax.legend(frameon=False, fontsize=8)


def render_all_figures(resolution, max_range, z_hit, z_short, z_max, z_rand, sigma_px,
                        out_dir, prefix, lambda_short=1.0):
    plt = _mpl()
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    params = _param_str(resolution, max_range, z_hit, z_short, z_max, z_rand, sigma_px)
    written = []

    # 1. columns
    fig, ax = plt.subplots(figsize=(7, 5))
    plot_columns(ax, resolution, max_range, z_hit, z_short, z_max, z_rand, sigma_px)
    fig.suptitle(params, fontsize=8, color="0.4", y=0.995)
    fig.tight_layout(rect=(0, 0, 1, 0.88))
    path = out_dir / f"{prefix}-sensor-model-columns.png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    written.append(path)

    # 2. mixture mass
    fig, ax = plt.subplots(figsize=(7, 5))
    plot_mixture_mass(ax, resolution, max_range, z_hit, z_short, z_max, z_rand, sigma_px)
    fig.suptitle(params, fontsize=8, color="0.4", y=0.995)
    fig.tight_layout(rect=(0, 0, 1, 0.88))
    path = out_dir / f"{prefix}-sensor-model-mixture-mass.png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    written.append(path)

    # 2b. mixture mass -- upstream vs normalized_short comparison
    # (Phase 3e Task 2, sec 5.1's canonical fix).
    fig, axes = plt.subplots(1, 2, figsize=(13, 5), sharey=True)
    plot_mixture_mass_comparison(axes, resolution, max_range, z_hit, z_short, z_max, z_rand,
                                  sigma_px, lambda_short=lambda_short)
    fig.suptitle(
        params + f", lambda_short={lambda_short:g} /px\n"
        "Effective mixture-weight breakdown: upstream vs. normalized_short (sec 5.1)",
        fontsize=9, y=1.0,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.90))
    path = out_dir / f"{prefix}-sensor-model-mixture-mass-comparison.png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    written.append(path)

    # 3. no-return ratio
    fig, ax = plt.subplots(figsize=(7, 5))
    plot_noreturn_ratio(ax, resolution, max_range, z_hit, z_short, z_max, z_rand, sigma_px)
    fig.suptitle(params, fontsize=8, color="0.4", y=0.995)
    fig.tight_layout(rect=(0, 0, 1, 0.88))
    path = out_dir / f"{prefix}-sensor-model-noreturn-ratio.png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    written.append(path)

    # 4. resolution coupling: figures 2 & 3 at res 0.05 vs 0.10, side by side.
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    for res, color, label in ((0.05, COLOR_RES_A, "0.05 m/px"), (0.10, COLOR_RES_B, "0.10 m/px")):
        max_range_px = int(round(max_range / res))
        breakdown = mixture_mass_breakdown(max_range_px, z_hit, z_short, z_max, z_rand, sigma_px)
        d_lo_px = int(round(5 / res))
        d_hi_px = int(round(55 / res))
        d_px = np.arange(d_lo_px, d_hi_px + 1)
        d_m = d_px * res
        axes[0].plot(d_m, breakdown["hit"][d_px] * 100, color=color, linewidth=2,
                     label=f"effective z_hit @ {label}", zorder=3)

        table = sensor_model_table(max_range_px, z_hit, z_short, z_max, z_rand, sigma_px)
        ratio = noreturn_ratio(table, max_range_px)
        d_lo_px2 = max(1, d_lo_px)
        d_px2 = np.arange(d_lo_px2, d_hi_px + 1)
        d_m2 = d_px2 * res
        axes[1].plot(d_m2, ratio[d_px2], color=color, linewidth=2, label=label, zorder=3)

    axes[0].set_xlabel("predicted range d (m)")
    axes[0].set_ylabel("effective z_hit share (%)")
    axes[0].set_title("Effective z_hit vs. map resolution")
    axes[0].grid(True, **GRID_KW)
    axes[0].legend(frameon=False, fontsize=8)

    axes[1].axhline(1.0, color=COLOR_RAND, linewidth=1.5, linestyle="--", zorder=2,
                     label="break-even")
    axes[1].set_xlabel("predicted range d (m)")
    axes[1].set_ylabel("P(no-return) / P(perfect match)")
    axes[1].set_title("No-return ratio vs. map resolution")
    axes[1].grid(True, **GRID_KW)
    axes[1].legend(frameon=False, fontsize=8)

    fig.suptitle(
        "Resolution coupling: sigma_hit is fixed in PIXELS (%.1f px), so halving\n"
        "the cell size halves the metric matching width and reshapes both charts above."
        % sigma_px,
        fontsize=9,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.90))
    path = out_dir / f"{prefix}-sensor-model-resolution-coupling.png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    written.append(path)

    return written


# ---------------------------------------------------------------------------
# Part B: frozen-scan glue -- bag reading, map loading, range_libc.
#
# ROS/rosbag2_py/range_libc/PIL/yaml imports happen INSIDE these functions,
# never at module scope, so `import plot_sensor_model` and the pure-function
# unit tests above still work under a plain (non-ROS) interpreter.
# ---------------------------------------------------------------------------

# Fixed to match run-particle-filter.sh's pointcloud_to_laserscan invocation
# (Step 2) and particle_filter.py's ANGLE_STEP default -- not CLI-overridable
# because they are what make this reconstruction "exactly as
# run-particle-filter.sh does" (the brief's phrasing), not a free parameter.
SCAN_ANGLE_MIN = -3.14159
SCAN_ANGLE_MAX = 3.14159
SCAN_ANGLE_INCREMENT = 0.0043
SCAN_RANGE_MIN = 0.1
SCAN_ANGLE_STEP = 18


def _quat_to_rotmat(q):
    """3x3 rotation matrix from a quaternion (x, y, z, w)."""
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _qmul(q1, q2):
    """Hamilton product q1 * q2, both (x, y, z, w) -- same convention as
    scripts/2dlidar/scan_accumulate_grid.py's `_qmul` (duplicated here, not
    imported, to keep this script's ROS-free pure functions importable
    without pulling in scan_accumulate_grid's `pcd_to_pgm` sys.path hack)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return (x, y, z, w)


def _quat_to_yaw(q):
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _compose_static_chain_3d(chain):
    """Compose a list of (translation, quaternion) hops -- innermost-first,
    same convention as scan_accumulate_grid.compose_static_chain -- into one
    FULL 3D (translation, quaternion) transform, WITHOUT collapsing to a
    planar (tx, ty, yaw). The z-band filter this feeds (1.91611-2.21611 m,
    base_link) needs the sensor mount's true z offset and any roll/pitch in
    the static chain, which a planar collapse (as used for the Part A
    scan-accumulation grid, where only x/y ever mattered) would discard.
    """
    if not chain:
        raise ValueError("_compose_static_chain_3d: chain must be non-empty")
    t_acc = np.zeros(3)
    q_acc = (0.0, 0.0, 0.0, 1.0)
    for t_i, q_i in chain:
        r_i = _quat_to_rotmat(q_i)
        t_acc = r_i @ t_acc + np.asarray(t_i, dtype=np.float64)
        q_acc = _qmul(q_i, q_acc)
    return t_acc, q_acc


def load_map_occupancy_grid(yaml_path):
    """Load a nav2 map_server-style `.yaml` + `.pgm` pair and build a
    `nav_msgs/OccupancyGrid` message, replicating map_server's pixel ->
    cell thresholding (negate / occupied_thresh / free_thresh) so
    `range_libc.PyOMap(msg)` sees exactly what it would from the real
    `GetMap` service in `get_omap()` (particle_filter.py:292-324) -- same
    `>10` binary-occupancy convention documented in
    docs/research/localization/2d_mcl_algorithm.md sec 1.2.
    """
    import yaml as _yaml
    from PIL import Image
    from nav_msgs.msg import OccupancyGrid

    yaml_path = Path(yaml_path)
    meta = _yaml.safe_load(yaml_path.read_text())
    pgm_path = yaml_path.parent / meta["image"]
    resolution = float(meta["resolution"])
    origin = meta["origin"]
    negate = int(meta.get("negate", 0))
    occupied_thresh = float(meta.get("occupied_thresh", 0.65))
    free_thresh = float(meta.get("free_thresh", 0.196))

    img = np.array(Image.open(pgm_path).convert("L"), dtype=np.float64)
    height, width = img.shape
    p_occ = (img / 255.0) if negate else ((255.0 - img) / 255.0)
    cell = np.full((height, width), -1, dtype=np.int8)
    cell[p_occ > occupied_thresh] = 100
    cell[p_occ < free_thresh] = 0
    # image row 0 = top = y_max (map_server/pcd_to_pgm.write_map convention);
    # OccupancyGrid.data row 0 must be y = origin_y (the grid's bottom row).
    data = np.flipud(cell)

    msg = OccupancyGrid()
    msg.info.resolution = resolution
    msg.info.width = int(width)
    msg.info.height = int(height)
    msg.info.origin.position.x = float(origin[0])
    msg.info.origin.position.y = float(origin[1])
    msg.info.origin.position.z = 0.0
    yaw = float(origin[2]) if len(origin) > 2 else 0.0
    msg.info.origin.orientation.z = math.sin(yaw / 2.0)
    msg.info.origin.orientation.w = math.cos(yaw / 2.0)
    msg.data = data.reshape(-1).astype(np.int8).tolist()
    return msg


def reconstruct_frozen_scan(bag_path, time_rel_s, min_height, max_height,
                             range_max, pc_topic="/sensing/lidar/top/pointcloud_raw_ex",
                             ks_topic="/localization/kinematic_state"):
    """Reconstruct one 82-beam decimated scan + the paired GT pose, exactly
    as run-particle-filter.sh's pipeline would have produced them from this
    bag at wall-clock `time_rel_s` seconds after the bag's own recording
    start (matching how the Task 4 live renders' "t=9.8s into replay"
    captions line up 1:1 against `sample_ndt_gt`'s own `ros2 bag info`
    start time, since `BAG=GT_BAG` for that run).

    Returns a dict: observed_ranges_m, downsampled_angles (float32, len 82
    at the vendored angle_step=18), gt_x, gt_y, gt_theta, scan_stamp_rel_s
    (the actual pointcloud's stamp, for the figure caption), plus a few
    counts for the report (n_points_total, n_points_in_band).
    """
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    bag_path = str(bag_path)
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions("", "")

    probe = rosbag2_py.SequentialReader()
    probe.open(storage_options, converter_options)
    type_map = {t.name: t.type for t in probe.get_all_topics_and_types()}
    del probe
    for required in (pc_topic, ks_topic, "/tf_static"):
        if required not in type_map:
            raise ValueError(f"reconstruct_frozen_scan: topic '{required}' not in {bag_path}")

    tf_type = get_message(type_map["/tf_static"])
    ks_type = get_message(type_map[ks_topic])
    pc_type = get_message(type_map[pc_topic])

    # Pass 1: bag start time, /tf_static edges, all GT poses, and the
    # RECORDED timestamp of every pointcloud message (not its payload --
    # 265 full clouds would be a large amount of memory to hold at once).
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    bag_start_ns = None
    edges = {}   # child_frame_id -> (parent_frame_id, (tx,ty,tz), (qx,qy,qz,qw))
    ks_list = []  # (t_s, x, y, yaw)
    pc_times_ns = []
    pc_frame_id = None
    while reader.has_next():
        topic, data, t = reader.read_next()
        if bag_start_ns is None:
            bag_start_ns = t
        if topic == "/tf_static":
            msg = deserialize_message(data, tf_type)
            for tr in msg.transforms:
                tt, qq = tr.transform.translation, tr.transform.rotation
                edges[tr.child_frame_id] = (
                    tr.header.frame_id, (tt.x, tt.y, tt.z), (qq.x, qq.y, qq.z, qq.w),
                )
        elif topic == ks_topic:
            msg = deserialize_message(data, ks_type)
            p, o = msg.pose.pose.position, msg.pose.pose.orientation
            t_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            ks_list.append((t_s, p.x, p.y, _quat_to_yaw((o.x, o.y, o.z, o.w))))
        elif topic == pc_topic:
            pc_times_ns.append(t)
            if pc_frame_id is None:
                msg = deserialize_message(data, pc_type)
                pc_frame_id = msg.header.frame_id

    if bag_start_ns is None:
        raise ValueError(f"reconstruct_frozen_scan: empty bag {bag_path}")
    if not pc_times_ns:
        raise ValueError(f"reconstruct_frozen_scan: no '{pc_topic}' messages in {bag_path}")
    if not ks_list:
        raise ValueError(f"reconstruct_frozen_scan: no '{ks_topic}' messages in {bag_path}")
    ks_list.sort(key=lambda e: e[0])

    target_ns = bag_start_ns + int(round(time_rel_s * 1e9))
    best_i = min(range(len(pc_times_ns)), key=lambda i: abs(pc_times_ns[i] - target_ns))
    target_pc_ns = pc_times_ns[best_i]

    # Pass 2: fetch just that one pointcloud message's payload.
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    pc_msg = None
    seen = 0
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic != pc_topic:
            continue
        if seen == best_i:
            pc_msg = deserialize_message(data, pc_type)
            break
        seen += 1
    if pc_msg is None:
        raise ValueError("reconstruct_frozen_scan: failed to re-locate target pointcloud")

    # sensor frame -> base_link, full 3D (translation, quaternion).
    chain = []
    frame = pc_frame_id
    for _ in range(20):
        if frame == "base_link":
            break
        if frame not in edges:
            raise ValueError(
                f"reconstruct_frozen_scan: no static chain from '{pc_frame_id}' "
                f"to 'base_link' (stuck at '{frame}')"
            )
        parent, t_i, q_i = edges[frame]
        chain.append((t_i, q_i))
        frame = parent
    else:
        raise ValueError("reconstruct_frozen_scan: static chain exceeded 20 hops")
    if not chain:
        t_acc, q_acc = np.zeros(3), (0.0, 0.0, 0.0, 1.0)
    else:
        t_acc, q_acc = _compose_static_chain_3d(chain)
    r_acc = _quat_to_rotmat(q_acc)

    offsets = {f.name: f.offset for f in pc_msg.fields}
    dtype = np.dtype({
        "names": ["x", "y", "z"],
        "formats": ["<f4", "<f4", "<f4"],
        "offsets": [offsets["x"], offsets["y"], offsets["z"]],
        "itemsize": pc_msg.point_step,
    })
    n = pc_msg.width * pc_msg.height
    arr = np.frombuffer(pc_msg.data, dtype=dtype, count=n)
    xyz_sensor = np.column_stack(
        [arr["x"].astype(np.float64), arr["y"].astype(np.float64), arr["z"].astype(np.float64)]
    )
    xyz_base = xyz_sensor @ r_acc.T + t_acc

    full_ranges = build_scan_ranges(
        xyz_base, min_height, max_height,
        SCAN_ANGLE_MIN, SCAN_ANGLE_MAX, SCAN_ANGLE_INCREMENT,
        SCAN_RANGE_MIN, range_max,
    )
    angles82, ranges82 = decimate_scan(full_ranges, SCAN_ANGLE_MIN, SCAN_ANGLE_MAX,
                                        SCAN_ANGLE_STEP)

    scan_stamp_s = pc_msg.header.stamp.sec + pc_msg.header.stamp.nanosec * 1e-9
    scan_t_ns = target_pc_ns
    ks_times = [e[0] for e in ks_list]
    best_k = min(range(len(ks_times)), key=lambda i: abs(ks_times[i] - scan_stamp_s))
    _t_pose, gt_x, gt_y, gt_theta = ks_list[best_k]

    in_band = (xyz_base[:, 2] >= min_height) & (xyz_base[:, 2] <= max_height) \
        & np.isfinite(xyz_base[:, 2])

    return {
        "observed_ranges_m": ranges82,
        "downsampled_angles": angles82,
        "gt_x": gt_x,
        "gt_y": gt_y,
        "gt_theta": gt_theta,
        "gt_pose_dt_s": _t_pose - scan_stamp_s,
        "scan_stamp_rel_s": (scan_t_ns - bag_start_ns) / 1e9,
        "scan_stamp_s": scan_stamp_s,
        "n_points_total": int(n),
        "n_points_in_band": int(in_band.sum()),
        "n_bins_finite": int(np.isfinite(ranges82).sum()),
        "n_bins_total": int(ranges82.shape[0]),
    }


def load_range_method(map_yaml, max_range_m, theta_discretization=112):
    """Load the map and build the range_libc raycaster ONCE, so callers that
    need to evaluate multiple poses/scans against the SAME map (e.g.
    `score_sensor_model.py`'s multi-timestamp sweep) can reuse it instead of
    paying `PyOMap`'s per-cell Cython load loop again for every timestamp
    (the dominant cost noted in the Phase 3d report).

    Returns (range_method, resolution) -- the same `range_libc.PyCDDTCast`
    instance and map resolution `evaluate_frozen_field` would have built
    internally.
    """
    import range_libc

    map_msg = load_map_occupancy_grid(map_yaml)
    oMap = range_libc.PyOMap(map_msg)
    resolution = float(map_msg.info.resolution)
    max_range_px = int(round(max_range_m / resolution))
    range_method = range_libc.PyCDDTCast(oMap, max_range_px, theta_discretization)
    return range_method, resolution


def evaluate_pose_grid(range_method, resolution, gt_x, gt_y, gt_theta, downsampled_angles,
                        window_m, field_res_m):
    """Raycast a fine pose grid (heading fixed at `gt_theta`) centred on
    (gt_x, gt_y) against an ALREADY-BUILT `range_method` (see
    `load_range_method`) -- the SAME PyCDDTCast + calc_range_repeat_angles
    path get_omap()/MCL() use (particle_filter.py:292-324, 645), just
    evaluated over a pose grid instead of the particle set.

    Returns (predicted_ranges_m (num_poses, num_beams), xs, ys), where xs/ys
    are the grid's metric coordinates (map frame) and predicted_ranges_m[i]
    pairs with (xs.ravel()[i], ys.ravel()[i]) under `numpy.meshgrid`
    row-major flattening (matches build_likelihood_field's layout in
    particle_filter.py:757-759).
    """
    n = int(round(window_m / field_res_m)) + 1
    num_rays = downsampled_angles.shape[0]
    num_poses = n * n
    offsets = (np.arange(n) - (n - 1) / 2.0) * field_res_m
    xs = gt_x + offsets
    ys = gt_y + offsets
    grid_x, grid_y = np.meshgrid(xs, ys)

    queries = np.zeros((num_poses, 3), dtype=np.float32)
    queries[:, 0] = grid_x.ravel().astype(np.float32)
    queries[:, 1] = grid_y.ravel().astype(np.float32)
    queries[:, 2] = np.float32(gt_theta)
    ranges = np.zeros(num_poses * num_rays, dtype=np.float32)
    range_method.calc_range_repeat_angles(
        queries, np.ascontiguousarray(downsampled_angles, dtype=np.float32), ranges,
    )
    predicted = ranges.reshape(num_poses, num_rays).astype(np.float64)
    return predicted, xs, ys


def evaluate_frozen_field(map_yaml, gt_x, gt_y, gt_theta, downsampled_angles,
                           window_m, field_res_m, max_range_m, theta_discretization=112):
    """Raycast a fine pose grid (heading fixed at `gt_theta`) centred on
    (gt_x, gt_y) against the map via range_libc directly -- the SAME
    PyCDDTCast + calc_range_repeat_angles path get_omap()/MCL() use
    (particle_filter.py:292-324, 645), just evaluated over a pose grid
    instead of the particle set.

    Returns (predicted_ranges_m (num_poses, num_beams), xs, ys, resolution),
    where xs/ys are the grid's metric coordinates (map frame) and
    predicted_ranges_m[i] pairs with (xs.ravel()[i], ys.ravel()[i]) under
    `numpy.meshgrid` row-major flattening (matches build_likelihood_field's
    layout in particle_filter.py:757-759).

    Kept for backward compatibility (single-call convenience); internally
    just chains `load_range_method` + `evaluate_pose_grid` so existing
    callers/tests see identical behaviour.
    """
    range_method, resolution = load_range_method(map_yaml, max_range_m, theta_discretization)
    predicted, xs, ys = evaluate_pose_grid(
        range_method, resolution, gt_x, gt_y, gt_theta, downsampled_angles,
        window_m, field_res_m,
    )
    return predicted, xs, ys, resolution


def plot_frozen_field(ax, log_likelihood_grid, xs, ys, gt_x, gt_y, argmax_x, argmax_y,
                       log_floor_nats, title):
    """Render the log-space frozen field: nats-below-max, floor-clipped,
    with the GT pose and the field's own argmax both marked."""
    res = xs[1] - xs[0] if len(xs) > 1 else 1.0
    extent = (xs[0] - res / 2.0, xs[-1] + res / 2.0, ys[0] - res / 2.0, ys[-1] + res / 2.0)
    below_max = np.clip(log_likelihood_grid - log_likelihood_grid.max(), -log_floor_nats, 0.0)
    im = ax.imshow(below_max, origin="lower", extent=extent, cmap="inferno",
                    vmin=-log_floor_nats, vmax=0.0, aspect="equal")
    ax.plot(gt_x, gt_y, marker="P", markersize=16, markeredgecolor="black",
            markerfacecolor="#39FF6A", linestyle="none", label="GT pose", zorder=5)
    ax.plot(argmax_x, argmax_y, marker="*", markersize=18, markeredgecolor="black",
            markerfacecolor="#39C4FF", linestyle="none", label="argmax (log-space)", zorder=5)
    ax.set_xlabel("map x (m)")
    ax.set_ylabel("map y (m)")
    ax.set_title(title, pad=10)
    ax.legend(loc="upper right", framealpha=0.85, fontsize=8)
    cbar = ax.figure.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
    cbar.set_label(f"log-likelihood, nats below max (floor at -{log_floor_nats:g})")


def run_frozen_scan(args):
    """CLI entry point for --frozen-scan. Prints the underflow-vs-
    misspecification numbers (the phase's headline question) and writes one
    PNG."""
    scan = reconstruct_frozen_scan(
        args.frozen_scan, args.time, args.min_height, args.max_height, args.max_range,
    )
    predicted, xs, ys, resolution = evaluate_frozen_field(
        args.map, scan["gt_x"], scan["gt_y"], scan["gt_theta"], scan["downsampled_angles"],
        args.window_m, args.field_res_m, args.max_range, args.theta_discretization,
    )
    max_range_px = int(round(args.max_range / resolution))
    table = sensor_model_table(max_range_px, args.z_hit, args.z_short, args.z_max,
                                args.z_rand, args.sigma_px)
    log_likelihood, raw_weight, probs = frozen_field_grid(
        predicted, scan["observed_ranges_m"], resolution, table,
    )

    n = int(round(args.window_m / args.field_res_m)) + 1
    ll_grid = log_likelihood.reshape(n, n)
    center_idx = (n - 1) // 2 * n + (n - 1) // 2   # GT pose is the grid centre by construction
    argmax_idx = int(np.argmax(log_likelihood))
    argmax_x = xs[argmax_idx % n]
    argmax_y = ys[argmax_idx // n]
    ll_at_gt = float(log_likelihood[center_idx])
    ll_at_argmax = float(log_likelihood[argmax_idx])
    n_underflow = int(np.sum(raw_weight == 0.0))
    frac_underflow = n_underflow / raw_weight.shape[0]
    raw_at_gt = float(raw_weight[center_idx])
    raw_at_argmax = float(raw_weight[argmax_idx])
    dist_argmax_m = float(np.hypot(argmax_x - scan["gt_x"], argmax_y - scan["gt_y"]))
    ll_min = float(log_likelihood.min())
    ll_min_idx = int(np.argmin(log_likelihood))

    print(f"Frozen scan reconstructed: t={args.time:g}s rel (scan stamp rel "
          f"{scan['scan_stamp_rel_s']:.3f}s, GT pose dt={scan['gt_pose_dt_s']:+.3f}s)")
    print(f"  points: {scan['n_points_total']} total, {scan['n_points_in_band']} in z-band "
          f"[{args.min_height:g}, {args.max_height:g}] m")
    print(f"  decimated beams: {scan['n_bins_total']}, {scan['n_bins_finite']} finite "
          f"({scan['n_bins_finite'] / scan['n_bins_total'] * 100:.1f}%)")
    print(f"  GT pose: x={scan['gt_x']:.3f} y={scan['gt_y']:.3f} theta={scan['gt_theta']:.4f}")
    print(f"  argmax pose: x={argmax_x:.3f} y={argmax_y:.3f} "
          f"({dist_argmax_m:.2f} m from GT)")
    print(f"  log-likelihood (nats): at GT = {ll_at_gt:.2f}, at argmax = {ll_at_argmax:.2f}, "
          f"gap = {ll_at_argmax - ll_at_gt:.2f} nats")
    print(f"  raw float64 product: at GT = {raw_at_gt:.6g}, at argmax = {raw_at_argmax:.6g}")
    print(f"  worst pose in grid: log-likelihood = {ll_min:.2f} nats "
          f"({ll_at_gt - ll_min:.2f} nats below GT), raw product = {raw_weight[ll_min_idx]:.6g}")
    print(f"  grid poses whose raw float64 product underflows to exactly 0.0: "
          f"{n_underflow}/{raw_weight.shape[0]} ({frac_underflow * 100:.4f}%)")

    plt = _mpl()
    fig, ax = plt.subplots(figsize=(8, 7))
    title = (
        f"Frozen-scan log-likelihood field ({args.label}) -- t={args.time:g}s into replay\n"
        f"window={n}x{n} @ {args.field_res_m:g}m, GT-argmax gap "
        f"{ll_at_argmax - ll_at_gt:.1f} nats"
    )
    plot_frozen_field(ax, ll_grid, xs, ys, scan["gt_x"], scan["gt_y"], argmax_x, argmax_y,
                       args.log_floor_nats, title)
    fig.tight_layout()
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / f"{args.prefix}-frozen-field-{args.label}.png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"Wrote: {path}")
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--resolution", type=float, default=0.05, help="map resolution, m/cell")
    ap.add_argument("--max-range", type=float, default=60.0, help="max sensor range, m")
    ap.add_argument("--z-hit", type=float, default=0.75)
    ap.add_argument("--z-short", type=float, default=0.01)
    ap.add_argument("--z-max", type=float, default=0.07)
    ap.add_argument("--z-rand", type=float, default=0.12)
    ap.add_argument("--sigma-px", type=float, default=8.0, help="sigma_hit, in map pixels")
    ap.add_argument("--lambda-short", type=float, default=1.0,
                     help="1/pixel decay rate for the normalized_short mixture-mass "
                          "comparison figure (Phase 3e Task 2)")
    ap.add_argument("--out-dir", type=Path, default=Path("docs/reports/assets"))
    ap.add_argument("--prefix", default="2dlidar-phase3d")

    frozen = ap.add_argument_group("frozen-scan (Part B)")
    frozen.add_argument("--frozen-scan", type=Path, default=None,
                         help="GT rosbag2 dir to reconstruct one scan from")
    frozen.add_argument("--map", type=Path, default=None,
                         help="map .yaml (nav2 map_server convention)")
    frozen.add_argument("--time", type=float, default=None,
                         help="timestamp, seconds relative to the bag's own recording start")
    frozen.add_argument("--window-m", type=float, default=60.0,
                         help="frozen-field pose-grid extent, metres")
    frozen.add_argument("--field-res-m", type=float, default=0.1,
                         help="frozen-field pose-grid spacing, metres")
    frozen.add_argument("--min-height", type=float, default=1.91611,
                         help="z-band min, base_link frame (sample sensor kit default)")
    frozen.add_argument("--max-height", type=float, default=2.21611,
                         help="z-band max, base_link frame (sample sensor kit default)")
    frozen.add_argument("--theta-discretization", type=int, default=112)
    frozen.add_argument("--log-floor-nats", type=float, default=120.0,
                         help="colourbar floor for the frozen-field render, nats below max")
    frozen.add_argument("--label", default=None,
                         help="output filename suffix, e.g. tracking/divergence")

    args = ap.parse_args(argv)

    if args.frozen_scan is not None:
        if args.map is None or args.time is None or args.label is None:
            ap.error("--frozen-scan requires --map, --time and --label")
        return run_frozen_scan(args)

    written = render_all_figures(
        args.resolution, args.max_range, args.z_hit, args.z_short, args.z_max,
        args.z_rand, args.sigma_px, args.out_dir, args.prefix,
        lambda_short=args.lambda_short,
    )

    max_range_px = int(round(args.max_range / args.resolution))
    breakdown = mixture_mass_breakdown(max_range_px, args.z_hit, args.z_short, args.z_max,
                                        args.z_rand, args.sigma_px)
    table = sensor_model_table(max_range_px, args.z_hit, args.z_short, args.z_max,
                                args.z_rand, args.sigma_px)
    ratio = noreturn_ratio(table, max_range_px)
    print("Cross-check against docs/research/localization/2d_mcl_algorithm.md sec. 5.1/5.2:")
    for d_m in (10, 20, 50):
        d_px = int(round(d_m / args.resolution))
        print(f"  effective z_hit at d={d_m:>3} m: {breakdown['hit'][d_px] * 100:.1f} %")
    d20_px = int(round(20 / args.resolution))
    print(f"  P(no-return)/P(perfect match) at d=20 m: {ratio[d20_px]:.2f}")

    print("Wrote:")
    for p in written:
        print(f"  {p}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
