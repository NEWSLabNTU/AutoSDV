#!/usr/bin/env python3
"""Render localization trajectories over the occupancy grid.

Produces three figures from one invocation, with markers every
``--mark-every`` seconds so the two tracks can be compared in time as well
as in space:

    <prefix>-ndt.png      NDT ground truth alone
    <prefix>-mcl.png      AutoSDV 2D-MCL alone
    <prefix>-overlay.png  both on one axis

"2D-MCL" is the AutoSDV localization stack: the vendored Roboracer
``particle_filter`` with the Phase 3e sensor-model fixes
(``sensor_model_variant=normalized_short``, ``skip_nonfinite_beams``,
``update_on_new_scan_only``). See docs/research/localization/2d_mcl_algorithm.md.

Bag reading comes from compare_poses.py (rosbag2_py-backed) so there is one
implementation of the GT/PF time-basis handling; note that PF pose stamps are
wall-clock (algorithm doc 5.3), which is why the GT bag is read on its storage
clock by default.

Requires the ROS environment:

    bash -c 'source /opt/autoware/1.5.0/setup.bash && \\
        python3 scripts/2dlidar/plot_trajectories.py'
"""
import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml
from PIL import Image

import compare_poses as cp

REPO_DIR = Path(__file__).resolve().parents[2]

NDT_LABEL = "NDT (Autoware, ground truth)"
MCL_LABEL = "AutoSDV 2D-MCL"
NDT_COLOUR = "forestgreen"
MCL_COLOUR = "royalblue"


def load_grid(map_yaml):
    """Return (image array, extent) for the occupancy grid behind a map yaml."""
    meta = yaml.safe_load(open(map_yaml))
    path = Path(map_yaml).parent / meta["image"]
    grid = np.array(Image.open(path))
    res = meta["resolution"]
    ox, oy = meta["origin"][:2]
    height, width = grid.shape
    return grid, [ox, ox + width * res, oy, oy + height * res]


def time_markers(ax, track, colour, mark_every, t0):
    """Annotate a track with elapsed-time markers every mark_every seconds.

    t0 is supplied by the caller and shared by every track in the figure (and
    across the three figures), so a given label marks the same instant on both
    tracks. Deriving it per-track instead would silently shift one track's
    labels relative to the other whenever they start at different times --
    which they do here, since 2D-MCL publishes from startup while the NDT
    ground truth only begins once its own initialisation converges.
    """
    span = track[-1][0] - t0
    first = track[0][0] - t0
    for mark in range(0, int(span) + 1, mark_every):
        if mark < first - mark_every:
            continue          # this track had not started publishing yet
        idx = int(np.argmin([abs(row[0] - (t0 + mark)) for row in track]))
        x, y = track[idx][1], track[idx][2]
        ax.plot(x, y, marker="o", ms=7, mfc="white", mec=colour, mew=2, zorder=5)
        ax.annotate(
            f"{mark}s",
            (x, y),
            textcoords="offset points",
            xytext=(8, -3),
            fontsize=9,
            color=colour,
            fontweight="bold",
            zorder=6,
        )
    return span


def draw(tracks, title, out_path, grid, extent, limits, mark_every, t0):
    """Draw one or more (track, label, colour) tuples onto the grid."""
    fig, ax = plt.subplots(figsize=(10, 11))
    ax.imshow(
        np.flipud(grid), origin="lower", extent=extent,
        cmap="gray_r", vmin=0, vmax=254, alpha=0.5,
    )
    spans = []
    for track, label, colour in tracks:
        xs = [row[1] for row in track]
        ys = [row[2] for row in track]
        ax.plot(xs, ys, "-", c=colour, lw=2.6, label=f"{label} ({len(track)} poses)", zorder=3)
        spans.append(time_markers(ax, track, colour, mark_every, t0))
        ax.plot(xs[0], ys[0], marker="s", ms=11, mfc=colour, mec="k", zorder=7)
        ax.plot(xs[-1], ys[-1], marker="X", ms=13, mfc=colour, mec="k", zorder=7)

    (xlo, xhi), (ylo, yhi) = limits
    ax.set_xlim(xlo, xhi)
    ax.set_ylim(ylo, yhi)
    ax.set_aspect("equal")
    ax.set_xlabel("map x (m)")
    ax.set_ylabel("map y (m)")
    ax.set_title(f"{title}\nsquare = start, X = end, markers every {mark_every} s "
                 f"({max(spans):.0f} s run, shared clock)", fontsize=13)
    ax.legend(loc="lower right", fontsize=10)
    ax.grid(alpha=0.2)
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)
    print(f"wrote {out_path}")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gt-bag", default=str(REPO_DIR / "data/rosbags/phase3/sample_ndt_gt"))
    parser.add_argument("--mcl-bag", default=str(REPO_DIR / "data/rosbags/phase3/seedmatrix/fixed_s1"))
    parser.add_argument("--map", default=str(
        REPO_DIR / "data/sample-rosbag-replay/sample-map-rosbag/occupancy_grid_scanaccum_mh1r05.yaml"))
    parser.add_argument("--gt-time-source", choices=("stamp", "bag"), default="bag")
    parser.add_argument("--mcl-topic", default="/pf/viz/inferred_pose")
    parser.add_argument("--mcl-type", default="PoseStamped")
    parser.add_argument("--mark-every", type=int, default=5, help="seconds between time markers")
    parser.add_argument("--pad-m", type=float, default=18.0, help="margin around the tracks")
    parser.add_argument("--out-dir", default=str(REPO_DIR / "docs/reports/assets"))
    parser.add_argument("--prefix", default="2dlidar-trajectories")
    args = parser.parse_args(argv)

    gt = cp.read_gt_bag(args.gt_bag, time_source=args.gt_time_source)
    mcl = cp.read_pf_bag(args.mcl_bag, topic=args.mcl_topic, pf_type=args.mcl_type)
    if not gt or not mcl:
        raise SystemExit("empty track: check bag paths and topic names")

    grid, extent = load_grid(args.map)
    xs = [row[1] for row in gt] + [row[1] for row in mcl]
    ys = [row[2] for row in gt] + [row[2] for row in mcl]
    pad = args.pad_m
    limits = ((min(xs) - pad, max(xs) + pad), (min(ys) - pad, max(ys) + pad))

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    # one clock for all three figures: earliest pose of either track
    t0 = min(gt[0][0], mcl[0][0])
    common = dict(grid=grid, extent=extent, limits=limits,
                  mark_every=args.mark_every, t0=t0)

    draw([(gt, NDT_LABEL, NDT_COLOUR)], NDT_LABEL,
         out_dir / f"{args.prefix}-ndt.png", **common)
    draw([(mcl, MCL_LABEL, MCL_COLOUR)], MCL_LABEL,
         out_dir / f"{args.prefix}-mcl.png", **common)
    draw([(gt, NDT_LABEL, NDT_COLOUR), (mcl, MCL_LABEL, MCL_COLOUR)],
         f"{MCL_LABEL} vs {NDT_LABEL}",
         out_dir / f"{args.prefix}-overlay.png", **common)


if __name__ == "__main__":
    main()
