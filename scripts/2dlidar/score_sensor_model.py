#!/usr/bin/env python3
"""Offline sensor-model scoring harness (Phase 3e Task 1 -- the gate).

For each requested timestamp, this script reconstructs the frozen scan
exactly as `run-particle-filter.sh`'s pipeline would (reusing
`plot_sensor_model.reconstruct_frozen_scan`, which itself replicates
pointcloud_to_laserscan's z-band + azimuth-binning loop and
particle_filter's angle_step=18 decimation), evaluates a log-space
likelihood field over a pose grid centred on the ground-truth pose
(reusing `plot_sensor_model.evaluate_pose_grid` / `sensor_model_table` /
`frozen_field_grid`), and reports four per-timestamp statistics plus an
aggregate summary:

  gap_nats     -- log-likelihood at the grid's argmax MINUS log-likelihood
                  at the GT pose (0 = perfect; positive = some other pose
                  scores higher than GT).
  dist_m       -- Euclidean distance from the GT pose to the argmax pose.
  gt_rank_pct  -- GT pose's percentile rank among all grid poses' log
                  likelihoods (fraction of grid poses GT beats).
  local_max    -- whether GT's grid cell log-likelihood is >= all 8 of its
                  immediate neighbours (GT is the grid centre by
                  construction, matching `evaluate_pose_grid`'s convention).

This is the offline "gate" Phase 3e's fixes (Tasks 2-4) are measured
against: `--variant upstream` (this task) reproduces the Phase 3d frozen-
scan baseline; later tasks add `--variant normalized_short` and
`--skip-nonfinite` without changing this harness's control flow.

Pure functions (`field_stats`, `percentile_rank`, `is_local_max`,
`apply_skip_nonfinite_mask`, `build_upstream_table`, `build_table`) import
nothing beyond numpy and are unit tested in `test_score_sensor_model.py`
under a plain interpreter, no ROS required. The bag/map/range_libc-
dependent glue (`score_timestamp`, `run`) imports rosbag2_py/range_libc
INSIDE functions only, matching `plot_sensor_model.py`'s existing pattern.

Run (baseline reproduction):
    bash -c 'source /opt/autoware/1.5.0/setup.bash && python3 \
        scripts/2dlidar/score_sensor_model.py \
        --gt-bag data/rosbags/phase3/sample_ndt_gt \
        --map data/sample-rosbag-replay/sample-map-rosbag/occupancy_grid_scanaccum_mh1r05.yaml \
        --times 9.82,23.35 --variant upstream \
        --out-json tmp/phase3e-baseline-check.json'
"""
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import plot_sensor_model as psm  # noqa: E402


# ---------------------------------------------------------------------------
# Variant registry -- table builders.
#
# `upstream` reuses plot_sensor_model.sensor_model_table with the same
# defaults Phase 3d used (z_hit=0.75, z_short=0.01, z_max=0.07, z_rand=0.12,
# sigma_px=8.0). `normalized_short` (Phase 3e Task 2) reuses the vendored
# fork's particle_filter.sensor_model.build_table -- the SAME function
# wired into precompute_sensor_model() -- so this harness scores the exact
# arithmetic that would run online, not a re-derivation of it. Neither
# addition touches score_timestamp/run's control flow.
# ---------------------------------------------------------------------------

def build_upstream_table(max_range_px, z_hit, z_short, z_max, z_rand, sigma_px,
                          lambda_short=1.0):
    """upstream variant: identical arithmetic to Phase 3d's
    `precompute_sensor_model` port (plot_sensor_model.sensor_model_table)."""
    return psm.sensor_model_table(max_range_px, z_hit, z_short, z_max, z_rand, sigma_px)


def build_normalized_short_table(max_range_px, z_hit, z_short, z_max, z_rand, sigma_px,
                                  lambda_short=1.0):
    """normalized_short variant (Phase 3e Task 2): per-column-normalised
    truncated-exponential short-reading component, via the vendored fork's
    `particle_filter.sensor_model.build_table` -- the exact function
    `precompute_sensor_model()` calls online (see
    src/localization/external/particle_filter/particle_filter/sensor_model.py).
    """
    fork_path = (
        Path(__file__).resolve().parents[2]
        / "src" / "localization" / "external" / "particle_filter"
    )
    if str(fork_path) not in sys.path:
        sys.path.insert(0, str(fork_path))
    from particle_filter.sensor_model import build_table as fork_build_table
    return fork_build_table(
        max_range_px, z_hit, z_short, z_max, z_rand, sigma_px,
        variant="normalized_short", lambda_short=lambda_short,
    )


VARIANT_BUILDERS = {
    "upstream": build_upstream_table,
    "normalized_short": build_normalized_short_table,
}


def build_table(variant, max_range_px, z_hit, z_short, z_max, z_rand, sigma_px,
                 lambda_short=1.0):
    """Dispatch to the variant's table builder. Raises ValueError on an
    unknown variant name (clear error, per the brief)."""
    try:
        builder = VARIANT_BUILDERS[variant]
    except KeyError:
        raise ValueError(
            f"unknown --variant '{variant}'; available: {sorted(VARIANT_BUILDERS)}"
        )
    return builder(max_range_px, z_hit, z_short, z_max, z_rand, sigma_px, lambda_short)


# ---------------------------------------------------------------------------
# skip-nonfinite beam mask.
#
# Semantics (must match a later task's reuse of this exact function): when
# enabled, drop non-finite (inf/nan) OBSERVED beams and their corresponding
# angles from evaluation entirely -- filtering both observed_ranges_m and
# downsampled_angles for those beam indices BEFORE the pose grid is
# evaluated (predicted ranges are computed only for the surviving angles).
# When disabled, nothing is filtered: all beams (including inf) participate,
# and frozen_field_grid's existing clamp-to-max-range-index behaviour
# handles them -- this is the Phase 3d baseline path, unchanged.
# ---------------------------------------------------------------------------

def apply_skip_nonfinite_mask(observed_ranges_m, downsampled_angles, skip_nonfinite):
    """Return (observed_ranges_m, downsampled_angles), filtered to drop
    non-finite observed beams (and their paired angles) iff `skip_nonfinite`
    is True. A no-op (returns the inputs unchanged) when False."""
    observed_ranges_m = np.asarray(observed_ranges_m)
    downsampled_angles = np.asarray(downsampled_angles)
    if not skip_nonfinite:
        return observed_ranges_m, downsampled_angles
    finite = np.isfinite(observed_ranges_m)
    return observed_ranges_m[finite], downsampled_angles[finite]


# ---------------------------------------------------------------------------
# Pure field statistics -- the part TDD'd against a synthetic field with a
# known argmax before any bag/map glue is written.
# ---------------------------------------------------------------------------

def percentile_rank(values, index):
    """Fraction of `values` that `values[index]` is >= to (itself included),
    i.e. what fraction of the population GT "beats or ties". 1.0 means GT is
    the (or a) maximum; 0.0 would mean nothing else in the population is
    ever <= it (impossible unless there's only one element, since the
    element always beats itself)."""
    values = np.asarray(values, dtype=np.float64)
    target = values[index]
    return float(np.mean(values <= target))


def is_local_max(grid, row, col):
    """True iff grid[row, col] >= all of its (up to 8) immediate
    neighbours within the 2D grid bounds. Ties count as local-max (>=, not
    >), matching the brief's ">= all 8 neighbours" wording."""
    grid = np.asarray(grid)
    n_rows, n_cols = grid.shape
    center = grid[row, col]
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            if dr == 0 and dc == 0:
                continue
            r, c = row + dr, col + dc
            if 0 <= r < n_rows and 0 <= c < n_cols:
                if grid[r, c] > center:
                    return False
    return True


def field_stats(log_likelihood, xs, ys, gt_x, gt_y):
    """Compute the four per-timestamp statistics from a flat
    `log_likelihood` array (length n*n, `numpy.meshgrid`-row-major layout
    matching `evaluate_pose_grid`'s convention) and the grid's metric
    coordinate axes `xs`/`ys` (each length n).

    GT is the grid centre by construction (same convention as
    `plot_sensor_model.run_frozen_scan`'s `center_idx`): center row/col is
    `(n-1)//2`.

    Returns a dict: gap_nats, dist_m, gt_rank_pct, local_max, plus the
    argmax pose (argmax_x, argmax_y) and both raw log-likelihoods
    (ll_at_gt, ll_at_argmax) for downstream reporting/debugging.
    """
    log_likelihood = np.asarray(log_likelihood, dtype=np.float64)
    xs = np.asarray(xs)
    ys = np.asarray(ys)
    n = xs.shape[0]
    if ys.shape[0] != n or log_likelihood.shape[0] != n * n:
        raise ValueError(
            f"field_stats: shape mismatch (xs={xs.shape}, ys={ys.shape}, "
            f"log_likelihood={log_likelihood.shape})"
        )

    ll_grid = log_likelihood.reshape(n, n)
    center_row = center_col = (n - 1) // 2
    center_idx = center_row * n + center_col

    argmax_idx = int(np.argmax(log_likelihood))
    argmax_row, argmax_col = divmod(argmax_idx, n)
    argmax_x = float(xs[argmax_col])
    argmax_y = float(ys[argmax_row])

    ll_at_gt = float(log_likelihood[center_idx])
    ll_at_argmax = float(log_likelihood[argmax_idx])

    return {
        "gap_nats": ll_at_argmax - ll_at_gt,
        "dist_m": float(np.hypot(argmax_x - gt_x, argmax_y - gt_y)),
        "gt_rank_pct": percentile_rank(log_likelihood, center_idx),
        "local_max": bool(is_local_max(ll_grid, center_row, center_col)),
        "argmax_x": argmax_x,
        "argmax_y": argmax_y,
        "ll_at_gt": ll_at_gt,
        "ll_at_argmax": ll_at_argmax,
    }


# ---------------------------------------------------------------------------
# JSON serialisation helper -- numpy float64/bool64/int64 are not
# JSON-serialisable by default.
# ---------------------------------------------------------------------------

class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.floating):
            return float(obj)
        if isinstance(obj, np.integer):
            return int(obj)
        if isinstance(obj, np.bool_):
            return bool(obj)
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return super().default(obj)


# ---------------------------------------------------------------------------
# Bag/map/range_libc glue -- ROS-dependent, imports only inside functions
# via plot_sensor_model (which itself defers those imports).
# ---------------------------------------------------------------------------

def score_timestamp(gt_bag, time_rel_s, range_method, resolution, table,
                     min_height, max_height, max_range_m, window_m, field_res_m,
                     skip_nonfinite, theta_discretization=112):
    """Reconstruct one frozen scan at `time_rel_s`, evaluate its log-space
    field against the pre-built `range_method`/`table`, and return a dict
    with the requested timestamp, snapped scan stamp, per-timestamp
    statistics, and beam-mask bookkeeping."""
    scan = psm.reconstruct_frozen_scan(
        gt_bag, time_rel_s, min_height, max_height, max_range_m,
    )
    observed_ranges_m, downsampled_angles = apply_skip_nonfinite_mask(
        scan["observed_ranges_m"], scan["downsampled_angles"], skip_nonfinite,
    )
    n_beams_used = int(observed_ranges_m.shape[0])
    n_beams_total = int(scan["observed_ranges_m"].shape[0])

    predicted, xs, ys = psm.evaluate_pose_grid(
        range_method, resolution, scan["gt_x"], scan["gt_y"], scan["gt_theta"],
        downsampled_angles, window_m, field_res_m,
    )
    log_likelihood, _raw_weight, _probs = psm.frozen_field_grid(
        predicted, observed_ranges_m, resolution, table,
    )
    stats = field_stats(log_likelihood, xs, ys, scan["gt_x"], scan["gt_y"])

    return {
        "requested_time_s": time_rel_s,
        "scan_stamp_rel_s": scan["scan_stamp_rel_s"],
        "gt_x": float(scan["gt_x"]),
        "gt_y": float(scan["gt_y"]),
        "gt_theta": float(scan["gt_theta"]),
        "gt_pose_dt_s": float(scan["gt_pose_dt_s"]),
        "n_beams_total": n_beams_total,
        "n_beams_used": n_beams_used,
        "gap_nats": stats["gap_nats"],
        "dist_m": stats["dist_m"],
        "gt_rank_pct": stats["gt_rank_pct"],
        "local_max": stats["local_max"],
        "argmax_x": stats["argmax_x"],
        "argmax_y": stats["argmax_y"],
        "ll_at_gt": stats["ll_at_gt"],
        "ll_at_argmax": stats["ll_at_argmax"],
    }


def summarize(per_timestamp):
    """Aggregate summary across all timestamps: median and worst-case
    (direction picked per field -- higher gap/dist is worse, lower rank is
    worse) of gap_nats/dist_m/gt_rank_pct, plus count of local_max=True."""
    gaps = np.array([r["gap_nats"] for r in per_timestamp], dtype=np.float64)
    dists = np.array([r["dist_m"] for r in per_timestamp], dtype=np.float64)
    ranks = np.array([r["gt_rank_pct"] for r in per_timestamp], dtype=np.float64)
    local_maxes = [bool(r["local_max"]) for r in per_timestamp]

    return {
        "n_timestamps": len(per_timestamp),
        "gap_nats_median": float(np.median(gaps)),
        "gap_nats_worst": float(np.max(gaps)),
        "dist_m_median": float(np.median(dists)),
        "dist_m_worst": float(np.max(dists)),
        "gt_rank_pct_median": float(np.median(ranks)),
        "gt_rank_pct_worst": float(np.min(ranks)),
        "local_max_count": int(sum(local_maxes)),
    }


def _parse_times(times_str):
    return [float(t.strip()) for t in times_str.split(",") if t.strip()]


def run(args):
    range_method, resolution = psm.load_range_method(
        args.map, args.max_range, args.theta_discretization,
    )
    max_range_px = int(round(args.max_range / resolution))
    table = build_table(
        args.variant, max_range_px, args.z_hit, args.z_short, args.z_max,
        args.z_rand, args.sigma_px, args.lambda_short,
    )

    times = _parse_times(args.times)
    per_timestamp = []
    timings_s = []
    for t in times:
        t0 = time.monotonic()
        result = score_timestamp(
            args.gt_bag, t, range_method, resolution, table,
            args.min_height, args.max_height, args.max_range,
            args.window_m, args.field_res_m, args.skip_nonfinite,
            args.theta_discretization,
        )
        dt = time.monotonic() - t0
        timings_s.append(dt)
        per_timestamp.append(result)
        print(
            f"t={t:g}s (scan stamp rel {result['scan_stamp_rel_s']:.3f}s): "
            f"gap={result['gap_nats']:.2f} nats, dist={result['dist_m']:.2f} m, "
            f"gt_rank_pct={result['gt_rank_pct']:.4f}, local_max={result['local_max']} "
            f"[{dt:.1f}s]"
        )

    summary = summarize(per_timestamp)
    summary["runtime_s_per_timestamp"] = timings_s
    summary["runtime_s_total"] = float(sum(timings_s))

    out = {
        "variant": args.variant,
        "skip_nonfinite": bool(args.skip_nonfinite),
        "params": {
            "gt_bag": str(args.gt_bag),
            "map": str(args.map),
            "z_hit": args.z_hit,
            "z_short": args.z_short,
            "z_max": args.z_max,
            "z_rand": args.z_rand,
            "sigma_px": args.sigma_px,
            "lambda_short": args.lambda_short,
            "min_height": args.min_height,
            "max_height": args.max_height,
            "max_range_m": args.max_range,
            "window_m": args.window_m,
            "field_res_m": args.field_res_m,
            "theta_discretization": args.theta_discretization,
            "map_resolution_m": resolution,
            "requested_times_s": times,
        },
        "per_timestamp": per_timestamp,
        "summary": summary,
    }

    out_json = Path(args.out_json)
    out_json.parent.mkdir(parents=True, exist_ok=True)
    out_json.write_text(json.dumps(out, indent=2, cls=NumpyEncoder))
    print(f"Wrote: {out_json}")

    print(
        f"Summary: gap_nats median={summary['gap_nats_median']:.2f} "
        f"worst={summary['gap_nats_worst']:.2f}, dist_m median="
        f"{summary['dist_m_median']:.2f} worst={summary['dist_m_worst']:.2f}, "
        f"gt_rank_pct median={summary['gt_rank_pct_median']:.4f} "
        f"worst={summary['gt_rank_pct_worst']:.4f}, local_max="
        f"{summary['local_max_count']}/{summary['n_timestamps']}"
    )

    if args.out_fig is not None:
        render_diagnostic_fig(out, args.out_fig)
        print(f"Wrote: {args.out_fig}")

    return 0


def render_diagnostic_fig(result, out_fig):
    """Cheap diagnostic figure: gap_nats and dist_m across the requested
    timestamps, on already-computed numbers (no re-evaluation)."""
    plt = psm._mpl()
    per_timestamp = result["per_timestamp"]
    ts = [r["requested_time_s"] for r in per_timestamp]
    gaps = [r["gap_nats"] for r in per_timestamp]
    dists = [r["dist_m"] for r in per_timestamp]

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    ax1.plot(ts, gaps, marker="o", color="#4C72B0")
    ax1.set_ylabel("gap_nats (GT->argmax)")
    ax1.grid(True, color="0.85", linewidth=0.6)
    ax1.set_title(
        f"score_sensor_model.py -- variant={result['variant']}, "
        f"skip_nonfinite={result['skip_nonfinite']}"
    )

    ax2.plot(ts, dists, marker="o", color="#DD8452")
    ax2.set_ylabel("dist_m (GT->argmax)")
    ax2.set_xlabel("requested time (s, relative to bag start)")
    ax2.grid(True, color="0.85", linewidth=0.6)

    fig.tight_layout()
    out_fig = Path(out_fig)
    out_fig.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_fig, dpi=150)
    plt.close(fig)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--gt-bag", type=Path, required=True, help="GT rosbag2 dir")
    ap.add_argument("--map", type=Path, required=True,
                     help="map .yaml (nav2 map_server convention)")
    ap.add_argument("--times", type=str, default="5,10,15,20,23",
                     help="comma-separated requested timestamps, seconds relative to "
                          "the bag's own recording start (snapped to nearest scan)")
    ap.add_argument("--variant", type=str, default="upstream",
                     choices=sorted(VARIANT_BUILDERS),
                     help="sensor-model table variant")
    ap.add_argument("--skip-nonfinite", action="store_true",
                     help="drop non-finite observed beams (and their angles) from "
                          "evaluation entirely, instead of clamping them to max range")
    ap.add_argument("--window-m", type=float, default=60.0,
                     help="frozen-field pose-grid extent, metres")
    ap.add_argument("--field-res-m", type=float, default=0.1,
                     help="frozen-field pose-grid spacing, metres (Phase 3d's default; "
                          "matches the baseline this task reproduces)")
    ap.add_argument("--min-height", type=float, default=1.91611,
                     help="z-band min, base_link frame (sample sensor kit default)")
    ap.add_argument("--max-height", type=float, default=2.21611,
                     help="z-band max, base_link frame (sample sensor kit default)")
    ap.add_argument("--max-range", type=float, default=60.0, help="max sensor range, m")
    ap.add_argument("--z-hit", type=float, default=0.75)
    ap.add_argument("--z-short", type=float, default=0.01)
    ap.add_argument("--z-max", type=float, default=0.07)
    ap.add_argument("--z-rand", type=float, default=0.12)
    ap.add_argument("--sigma-px", type=float, default=8.0, help="sigma_hit, in map pixels")
    ap.add_argument("--lambda-short", type=float, default=1.0,
                     help="1/pixel decay rate for --variant normalized_short's "
                          "short-reading component (ignored for --variant upstream)")
    ap.add_argument("--theta-discretization", type=int, default=112)
    ap.add_argument("--out-json", type=Path, required=True)
    ap.add_argument("--out-fig", type=Path, default=None)

    args = ap.parse_args(argv)
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
