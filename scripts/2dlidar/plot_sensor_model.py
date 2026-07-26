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
"""
import argparse
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
                        out_dir, prefix):
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
    ap.add_argument("--out-dir", type=Path, default=Path("docs/reports/assets"))
    ap.add_argument("--prefix", default="2dlidar-phase3d")
    args = ap.parse_args(argv)

    written = render_all_figures(
        args.resolution, args.max_range, args.z_hit, args.z_short, args.z_max,
        args.z_rand, args.sigma_px, args.out_dir, args.prefix,
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
