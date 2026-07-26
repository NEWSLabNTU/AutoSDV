#!/usr/bin/env python3
"""Offline chart suite for MCL (particle_filter) per-update diagnostics.

Phase 3d Task 3. Reads the JSONL emitted by the fork's
`DiagnosticsRecorder` (Phase 3d Task 1, `diag_enable:=true`; schema
documented in `particle_filter/diagnostics.py` and the task brief) and
renders one PNG with six panels sharing a time axis, each aimed at a
specific structural defect from
`docs/research/localization/2d_mcl_algorithm.md` sec. 5:

  1. translational error vs GT (if --gt-bag given), divergence onset marked
  2. N_eff, the ess_threshold_ratio*N line, and a resample-event rug
  3. weight entropy and weight_max
  4. pose covariance trace and its xy eigenvalue ratio (the ridge signature
     for sec. 5.4 -- beam non-independence sharpens the likelihood into a
     ridge along the corridor, which shows up here as a large eigenvalue
     ratio, not as an elevated trace)
  5. beam category fractions (hit/short/long/clamped/nonfinite), stacked
  6. per-stage timing (t_propose/t_motion/t_sensor/t_norm) and update rate

Pure/table-building functions (`load_records`, `eigenvalue_ratio`,
`update_rate`, `divergence_onset`, `join_nearest_stamp`) import nothing
beyond numpy/compare_poses and are unit tested in
`test_plot_mcl_diagnostics.py` under a plain interpreter, no ROS
required. `--gt-bag` additionally needs rosbag2_py/rclpy (sourced ROS
env; imported lazily, same pattern as compare_poses.py), and rendering
needs matplotlib (Agg backend, set below).

--- Time-basis choice for the GT join (see brief) ---
particle_filter.py stamps `/pf/viz/inferred_pose` (and every diagnostics
record) via `self.get_clock().now()` / `time.time()` -- i.e. whatever
wall-clock the process was running on, NOT the incoming scan's sim-time
content stamp propagated through to the pose. The diagnostics record
carries both: `stamp_scan` (the last LaserScan header.stamp -- sim time,
if the source bag carries `use_sim_time`/`/clock`) and `stamp_wall`
(`time.time()` at record time -- always real wall-clock). Since the pose
that would need aligning is on the wall-clock basis, `stamp_wall` is the
default join key (`--time-basis stamp_wall`), paired against
`--gt-time-source bag` (rosbag2 storage receive-time, the GT bag's own
wall-clock -- see compare_poses._read_bag's docstring for why `stamp`
vs `bag` matters). `--time-basis stamp_scan` + `--gt-time-source stamp`
is offered as a fallback for runs recorded with synchronized sim time.
If neither basis produces any pair within `--max-dt`, the panel renders
"GT comparison unavailable" with the reason instead of a misleading
curve -- see `compute_error_series()`.
"""
import argparse
import bisect
import json
import math
import sys
import textwrap
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from compare_poses import align_tracks, yaw_error  # noqa: E402

# ---------------------------------------------------------------------------
# Fixed categorical palette (seaborn "deep"-derived, CVD-reasonable),
# assigned by entity, never by rank.
# ---------------------------------------------------------------------------
COLOR_ERROR = "#C44E52"       # red -- error is bad
COLOR_NEFF = "#4C72B0"        # blue
COLOR_NEFF_THRESH = "#8C8C8C"  # grey -- reference line
COLOR_ENTROPY = "#4C72B0"     # blue
COLOR_WMAX = "#DD8452"        # orange
COLOR_COV_TRACE = "#4C72B0"   # blue
COLOR_EIG_RATIO = "#C44E52"   # red -- the ridge signature, make it pop
COLOR_HIT = "#4C72B0"
COLOR_SHORT = "#DD8452"
COLOR_LONG = "#55A868"
COLOR_CLAMPED = "#8172B2"
COLOR_NONFINITE = "#937860"
COLOR_T_PROPOSE = "#4C72B0"
COLOR_T_MOTION = "#DD8452"
COLOR_T_SENSOR = "#55A868"
COLOR_T_NORM = "#8172B2"
COLOR_UPDATE_HZ = "#C44E52"
GRID_KW = dict(color="0.85", linewidth=0.6, zorder=0)


# ---------------------------------------------------------------------------
# JSONL loading.
# ---------------------------------------------------------------------------

def load_records(path):
    """Load one JSON object per line from `path`.

    Malformed lines (invalid JSON, blank lines) are skipped silently
    (this is diagnostics data -- a truncated/corrupted last line from a
    killed process is expected, not exceptional). Raises `ValueError` if
    the file contains zero valid records, so a caller never silently
    plots an empty figure. Raises `FileNotFoundError` if `path` doesn't
    exist (standard open() behavior, not caught).
    """
    path = Path(path)
    records = []
    skipped = 0
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                records.append(json.loads(line))
            except json.JSONDecodeError:
                skipped += 1
                continue
    if not records:
        raise ValueError(
            "load_records: no valid JSON records found in %s (%d malformed/blank "
            "lines skipped)" % (path, skipped)
        )
    if skipped:
        print(
            "load_records: skipped %d malformed/blank line(s) in %s" % (skipped, path),
            file=sys.stderr,
        )
    return records


# ---------------------------------------------------------------------------
# Derived-series maths.
# ---------------------------------------------------------------------------

def eigenvalue_ratio(cov_xx, cov_yy, cov_xy):
    """Ratio of the larger to smaller eigenvalue of the 2x2 xy covariance
    [[cov_xx, cov_xy], [cov_xy, cov_yy]] -- the ridge/elongation signature
    (sec. 5.4: correlated beam errors sharpen the likelihood into a ridge
    along the corridor, which is far more visible as an eigenvalue ratio
    than as the trace, since an elongated-but-thin ridge can have the same
    trace as a compact isotropic blob of different size).

    Closed-form via the trace/determinant of a symmetric 2x2 matrix
    (eig = trace/2 +/- sqrt((trace/2)^2 - det)) -- no numpy.linalg needed,
    avoids an eigendecomposition call per row of a long series. Returns
    NaN when both eigenvalues are ~0 (degenerate/zero-spread cloud), since
    a ratio of two numbers indistinguishable from zero is meaningless, not
    "1.0" or "0.0".
    """
    trace = cov_xx + cov_yy
    det = cov_xx * cov_yy - cov_xy * cov_xy
    half_trace = trace / 2.0
    disc = half_trace * half_trace - det
    disc = max(disc, 0.0)  # guard tiny negative from float error
    sqrt_disc = math.sqrt(disc)
    eig_max = half_trace + sqrt_disc
    eig_min = half_trace - sqrt_disc
    if eig_max <= 1e-15:
        return float("nan")
    if eig_min <= 1e-15:
        return float("inf")
    return eig_max / eig_min


def update_rate(dt_update):
    """1/dt_update in Hz, or NaN when dt_update is non-positive (the
    first record has no previous update, dt_update <= 0 by convention)."""
    if dt_update is None or dt_update <= 0.0:
        return float("nan")
    return 1.0 / dt_update


def divergence_onset(times, errors, threshold):
    """First time at which translational error crosses `threshold` and
    stays elevated on average for the remainder of the series (guards
    against marking a single transient spike as "the" divergence point).

    Returns None if no such sustained crossing exists. Pure function --
    scans from the end for the earliest index `i` such that (a) `errors[i]`
    itself exceeds `threshold` and (b) at least 80% of the remaining
    samples `errors[i:]` also exceed it. The 80% fraction (not a suffix
    mean) is what rejects a single transient spike: one huge outlier can
    drag a suffix *mean* above threshold even while almost every sample
    in that suffix is fine, which a mean-based test would misreport as
    "sustained".
    """
    n = len(times)
    if n == 0:
        return None
    above_count = 0
    best = None
    for i in range(n - 1, -1, -1):
        if errors[i] > threshold:
            above_count += 1
        suffix_len = n - i
        fraction_above = above_count / suffix_len
        if errors[i] > threshold and fraction_above >= 0.8:
            best = i
    return times[best] if best is not None else None


def join_nearest_stamp(pf, gt, max_dt):
    """Nearest-timestamp join of `pf` onto `gt`, keeping the PF sample's
    own timestamp (unlike `compare_poses.align_tracks`, which discards it
    -- needed here to place each error sample on the shared time axis).

    Same nearest-neighbor/max_dt algorithm as `align_tracks`; kept as a
    separate small function (rather than changing `align_tracks`'s
    return type, which `compare_poses.py`'s report generator depends on)
    that reuses `yaw_error` from compare_poses.

    `pf`, `gt`: lists of (t, x, y, yaw), sorted by t ascending. Returns a
    list of (t_pf, trans_err, yaw_err) tuples, or [] if either input is
    empty or no pair survives the max_dt filter (never raises -- the
    caller decides what "no overlap" means for its panel).
    """
    if not gt or not pf:
        return []
    gt_times = [p[0] for p in gt]
    pairs = []
    for t_pf, x_pf, y_pf, yaw_pf in pf:
        idx = bisect.bisect_left(gt_times, t_pf)
        candidates = []
        if idx < len(gt_times):
            candidates.append(idx)
        if idx > 0:
            candidates.append(idx - 1)
        if not candidates:
            continue
        best_idx = min(candidates, key=lambda i: abs(gt_times[i] - t_pf))
        if abs(gt_times[best_idx] - t_pf) > max_dt:
            continue
        t_gt, x_gt, y_gt, yaw_gt = gt[best_idx]
        trans_err = math.hypot(x_pf - x_gt, y_pf - y_gt)
        yerr = yaw_error(yaw_gt, yaw_pf)
        pairs.append((t_pf, trans_err, yerr))
    return pairs


# ---------------------------------------------------------------------------
# Series extraction from records.
# ---------------------------------------------------------------------------

def _series(records, key):
    return [r.get(key) for r in records]


def _time_axis(records):
    """Elapsed seconds since the first record, on the stamp_wall basis
    (monotonic time.time() calls -- reliable even though stamp_scan can
    repeat/jump, see module docstring)."""
    t0 = records[0]["stamp_wall"]
    return [r["stamp_wall"] - t0 for r in records]


def compute_error_series(records, times, gt_bag_path, time_basis, gt_time_source, max_dt):
    """Build the GT-alignment error series for panel 1.

    Returns (t_err, trans_err, reason) where `t_err`/`trans_err` are
    parallel lists (possibly empty) and `reason` is None on success or a
    human-readable string explaining why the panel is unavailable.
    Imports rosbag2_py lazily (only when gt_bag_path is given), matching
    compare_poses.py's pattern so plain-shell tests never need ROS.
    """
    if gt_bag_path is None:
        return [], [], "no --gt-bag given"

    from compare_poses import read_gt_bag  # lazy: needs rosbag2_py/rclpy

    stamp_key = "stamp_wall" if time_basis == "stamp_wall" else "stamp_scan"
    pf_track = []
    for r, t in zip(records, times):
        stamp = r.get(stamp_key)
        if stamp is None:
            continue
        pf_track.append((stamp, r["pose_x"], r["pose_y"], r["pose_theta"]))

    try:
        gt_track = read_gt_bag(gt_bag_path, time_source=gt_time_source)
    except Exception as exc:  # pragma: no cover -- needs a real bag
        return [], [], "failed to read GT bag %s: %s" % (gt_bag_path, exc)

    pairs = join_nearest_stamp(pf_track, gt_track, max_dt=max_dt)
    if not pairs:
        return [], [], (
            "no overlapping timestamps within max_dt=%.3fs between diag "
            "'%s' and GT bag '%s' (--gt-time-source=%s) -- the two "
            "recordings are likely from different clocks/sessions; see "
            "module docstring for the stamp_wall/stamp_scan basis choice"
            % (max_dt, stamp_key, gt_bag_path, gt_time_source)
        )

    # Re-key pairs onto the shared plot time axis (elapsed seconds from
    # the diag file's first stamp_wall), not the raw stamp used for
    # joining.
    t0 = records[0]["stamp_wall"]
    stamp_to_t = {r.get(stamp_key): t for r, t in zip(records, times)}
    t_err, trans_err = [], []
    for t_pf, trans, _yaw in pairs:
        t_plot = stamp_to_t.get(t_pf)
        if t_plot is None:
            continue
        t_err.append(t_plot)
        trans_err.append(trans)
    order = np.argsort(t_err)
    t_err = list(np.asarray(t_err)[order])
    trans_err = list(np.asarray(trans_err)[order])
    return t_err, trans_err, None


# ---------------------------------------------------------------------------
# Plotting.
# ---------------------------------------------------------------------------

def _mpl():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    return plt


def plot_error_panel(ax, t_err, trans_err, reason, onset_threshold=1.0):
    if reason is not None:
        wrapped = "\n".join(textwrap.wrap(reason, width=68))
        ax.text(
            0.5, 0.5, "GT comparison unavailable:\n%s" % wrapped,
            transform=ax.transAxes, ha="center", va="center", fontsize=8,
            color="0.4",
        )
        ax.set_ylabel("trans. error (m)")
        ax.set_xticks([])
        ax.set_yticks([])
        return
    ax.plot(t_err, trans_err, color=COLOR_ERROR, linewidth=1.2, zorder=3)
    onset = divergence_onset(t_err, trans_err, threshold=onset_threshold)
    if onset is not None:
        ax.axvline(onset, color="0.2", linewidth=1.2, linestyle="--", zorder=2)
        ax.text(onset, ax.get_ylim()[1] if ax.get_ylim()[1] > 0 else 1.0,
                " divergence onset", fontsize=7, color="0.3", va="top")
    ax.set_ylabel("trans. error (m)")
    ax.grid(True, **GRID_KW)


def plot_neff_panel(ax, times, n_eff, resampled, max_particles, ess_threshold_ratio):
    ax.plot(times, n_eff, color=COLOR_NEFF, linewidth=1.2, zorder=3, label="N_eff")
    thresh = ess_threshold_ratio * max_particles
    ax.axhline(thresh, color=COLOR_NEFF_THRESH, linewidth=1.2, linestyle="--", zorder=2,
               label="ess_threshold_ratio * N = %.0f" % thresh)
    # Resample-event rug: ticks along the top of the axis.
    resample_times = [t for t, r in zip(times, resampled) if r]
    if resample_times:
        ymax = max(n_eff) if n_eff else max_particles
        rug_y = ymax * 1.02
        ax.plot(resample_times, [rug_y] * len(resample_times), "|", color="0.2",
                markersize=6, zorder=4, label="resampled")
    ax.set_ylabel("N_eff")
    ax.grid(True, **GRID_KW)
    ax.legend(frameon=False, fontsize=7, loc="upper right")


def plot_entropy_panel(ax, times, entropy, weight_max):
    ax.plot(times, entropy, color=COLOR_ENTROPY, linewidth=1.2, zorder=3, label="weight entropy (nats)")
    ax.set_ylabel("weight entropy (nats)", color=COLOR_ENTROPY)
    ax.tick_params(axis="y", labelcolor=COLOR_ENTROPY)
    ax2 = ax.twinx()
    ax2.plot(times, weight_max, color=COLOR_WMAX, linewidth=1.0, zorder=3, label="weight_max")
    ax2.set_ylabel("weight_max", color=COLOR_WMAX)
    ax2.tick_params(axis="y", labelcolor=COLOR_WMAX)
    ax.grid(True, **GRID_KW)


def plot_covariance_panel(ax, times, cov_trace, eig_ratio):
    ax.plot(times, cov_trace, color=COLOR_COV_TRACE, linewidth=1.2, zorder=3)
    ax.set_ylabel("cov trace (m^2)", color=COLOR_COV_TRACE)
    ax.tick_params(axis="y", labelcolor=COLOR_COV_TRACE)
    ax2 = ax.twinx()
    ax2.plot(times, eig_ratio, color=COLOR_EIG_RATIO, linewidth=1.4, zorder=4)
    ax2.set_yscale("log")
    ax2.set_ylabel("xy eigenvalue ratio (log)", color=COLOR_EIG_RATIO)
    ax2.tick_params(axis="y", labelcolor=COLOR_EIG_RATIO)
    ax.text(
        0.5, 1.10,
        "High eigenvalue ratio = elongated (ridge-shaped) uncertainty, not just large --\n"
        "the sec. 5.4 signature of beams that fail to disambiguate along a corridor.",
        transform=ax.transAxes, ha="center", va="bottom", fontsize=7.5, color="0.35",
    )
    ax.grid(True, **GRID_KW)


def plot_beam_categories_panel(ax, times, frac_hit, frac_short, frac_long, frac_clamped, frac_nonfinite):
    ax.stackplot(
        times, frac_hit, frac_short, frac_long, frac_clamped, frac_nonfinite,
        colors=[COLOR_HIT, COLOR_SHORT, COLOR_LONG, COLOR_CLAMPED, COLOR_NONFINITE],
        labels=["hit", "short", "long", "clamped", "nonfinite"],
        zorder=3,
    )
    ax.set_ylim(0, 1)
    ax.set_ylabel("beam category fraction")
    ax.grid(True, axis="y", **GRID_KW)
    ax.legend(loc="upper right", frameon=True, facecolor="white", framealpha=0.85,
              fontsize=7, ncol=3)


def plot_timing_panel(ax, times, t_propose, t_motion, t_sensor, t_norm, update_hz):
    ax.stackplot(
        times,
        [v * 1000 for v in t_propose], [v * 1000 for v in t_motion],
        [v * 1000 for v in t_sensor], [v * 1000 for v in t_norm],
        colors=[COLOR_T_PROPOSE, COLOR_T_MOTION, COLOR_T_SENSOR, COLOR_T_NORM],
        labels=["t_propose", "t_motion", "t_sensor", "t_norm"],
        zorder=3,
    )
    ax.set_ylabel("per-stage time (ms)")
    ax.set_xlabel("elapsed time (s), stamp_wall basis")
    ax.grid(True, axis="y", **GRID_KW)
    ax.legend(loc="upper left", frameon=False, fontsize=7, ncol=4)
    ax2 = ax.twinx()
    ax2.plot(times, update_hz, color=COLOR_UPDATE_HZ, linewidth=1.0, linestyle=":", zorder=4,
              label="update rate (Hz)")
    ax2.set_ylabel("update rate (Hz)", color=COLOR_UPDATE_HZ)
    ax2.tick_params(axis="y", labelcolor=COLOR_UPDATE_HZ)


def render_figure(records, out_dir, prefix, gt_bag_path=None, time_basis="stamp_wall",
                   gt_time_source="bag", max_dt=0.1, max_particles=4000,
                   ess_threshold_ratio=0.5, onset_threshold=1.0):
    plt = _mpl()
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    times = _time_axis(records)
    n_eff = _series(records, "n_eff")
    resampled = _series(records, "resampled")
    entropy = _series(records, "weight_entropy")
    weight_max = _series(records, "weight_max")
    cov_trace = [r["cov_xx"] + r["cov_yy"] for r in records]
    eig_ratio = [eigenvalue_ratio(r["cov_xx"], r["cov_yy"], r["cov_xy"]) for r in records]
    frac_hit = _series(records, "frac_hit")
    frac_short = _series(records, "frac_short")
    frac_long = _series(records, "frac_long")
    frac_clamped = _series(records, "frac_clamped")
    frac_nonfinite = _series(records, "frac_nonfinite")
    t_propose = _series(records, "t_propose")
    t_motion = _series(records, "t_motion")
    t_sensor = _series(records, "t_sensor")
    t_norm = _series(records, "t_norm")
    update_hz = [update_rate(r.get("dt_update")) for r in records]

    t_err, trans_err, reason = compute_error_series(
        records, times, gt_bag_path, time_basis, gt_time_source, max_dt,
    )

    fig, axes = plt.subplots(6, 1, figsize=(11, 16), sharex=True)
    plot_error_panel(axes[0], t_err, trans_err, reason, onset_threshold=onset_threshold)
    axes[0].set_title("Translational error vs GT", loc="left", fontsize=10)
    plot_neff_panel(axes[1], times, n_eff, resampled, max_particles, ess_threshold_ratio)
    axes[1].set_title("N_eff, ESS threshold, resample events", loc="left", fontsize=10)
    plot_entropy_panel(axes[2], times, entropy, weight_max)
    axes[2].set_title("Weight entropy and weight_max", loc="left", fontsize=10)
    plot_covariance_panel(axes[3], times, cov_trace, eig_ratio)
    axes[3].set_title("Pose covariance trace and xy eigenvalue ratio (ridge signature)",
                       loc="left", fontsize=10)
    plot_beam_categories_panel(axes[4], times, frac_hit, frac_short, frac_long,
                                frac_clamped, frac_nonfinite)
    axes[4].set_title("Beam category fractions", loc="left", fontsize=10)
    plot_timing_panel(axes[5], times, t_propose, t_motion, t_sensor, t_norm, update_hz)
    axes[5].set_title("Per-stage timing and update rate", loc="left", fontsize=10)

    fig.suptitle(
        "MCL per-update diagnostics (%d records, N=%d particles)"
        % (len(records), max_particles),
        fontsize=12,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    path = out_dir / ("%s-mcl-diagnostics.png" % prefix)
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


# ---------------------------------------------------------------------------
# CLI.
# ---------------------------------------------------------------------------

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("diag_jsonl", type=Path, help="Path to the MCL diagnostics JSONL")
    ap.add_argument("--out-dir", type=Path, default=Path("docs/reports/assets"))
    ap.add_argument("--prefix", default="2dlidar-phase3d")
    ap.add_argument("--gt-bag", default=None,
                     help="Path to NDT ground-truth rosbag2 (nav_msgs/Odometry on "
                          "/localization/kinematic_state). Omit to render panel 1 as unavailable.")
    ap.add_argument("--time-basis", choices=["stamp_wall", "stamp_scan"], default="stamp_wall",
                     help="Which diag timestamp field to join against the GT bag on "
                          "(default: stamp_wall -- see module docstring).")
    ap.add_argument("--gt-time-source", choices=["stamp", "bag"], default="bag",
                     help="Clock basis for reading the GT bag (default: bag -- pairs with "
                          "--time-basis=stamp_wall; see module docstring).")
    ap.add_argument("--max-dt", type=float, default=0.1,
                     help="Max |dt| (s) for nearest-timestamp GT pairing")
    ap.add_argument("--max-particles", type=int, default=4000,
                     help="N used for the N_eff/ESS-threshold reference line (not in the "
                          "JSONL schema; pass the run's max_particles)")
    ap.add_argument("--ess-threshold-ratio", type=float, default=0.5,
                     help="ess_threshold_ratio used for the N_eff reference line")
    ap.add_argument("--onset-threshold", type=float, default=1.0,
                     help="Translational error (m) above which a sustained excursion is "
                          "marked as the divergence onset")
    args = ap.parse_args(argv)

    records = load_records(args.diag_jsonl)
    path = render_figure(
        records, args.out_dir, args.prefix,
        gt_bag_path=args.gt_bag, time_basis=args.time_basis,
        gt_time_source=args.gt_time_source, max_dt=args.max_dt,
        max_particles=args.max_particles, ess_threshold_ratio=args.ess_threshold_ratio,
        onset_threshold=args.onset_threshold,
    )
    print("wrote %s (%d records)" % (path, len(records)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
