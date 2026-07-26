#!/usr/bin/env python3
"""Phase 3e Task 6: GT vs PF trajectory overlay, two configs side by side.

Reuses compare_poses.py's read_gt_bag/read_pf_bag (rosbag2_py-backed) --
no new bag-reading logic. Defaults to the seed-1 upstream/fixed pair from
the Phase 3e seed matrix against the sample-site NDT ground-truth bag, the
figure referenced by docs/reports/2dlidar-phase3e-model-fixes.md.

Requires the ROS/rosbag2_py environment, e.g.:
    bash -c 'source /opt/autoware/1.5.0/setup.bash && source install/setup.bash && \\
        python3 scripts/2dlidar/plot_trajectory_compare.py'
"""
import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

import compare_poses as cp

REPO_DIR = Path(__file__).resolve().parents[2]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--gt-bag", default=str(REPO_DIR / "data/rosbags/phase3/sample_ndt_gt")
    )
    parser.add_argument(
        "--left-bag",
        default=str(REPO_DIR / "data/rosbags/phase3/seedmatrix/upstream_s1"),
    )
    parser.add_argument(
        "--left-label", default="upstream (seed 1)"
    )
    parser.add_argument(
        "--right-bag",
        default=str(REPO_DIR / "data/rosbags/phase3/seedmatrix/fixed_s1"),
    )
    parser.add_argument(
        "--right-label", default="fixed (seed 1)"
    )
    parser.add_argument(
        "--out",
        default=str(REPO_DIR / "docs/reports/assets/2dlidar-phase3e-trajectory-compare.png"),
    )
    parser.add_argument(
        "--title",
        default=(
            "Phase 3e: PF trajectory vs NDT ground truth, upstream vs fixed "
            "sensor model (occupancy_grid_scanaccum_mh1r05, seed 1)"
        ),
    )
    args = parser.parse_args(argv)

    gt = cp.read_gt_bag(args.gt_bag, time_source="bag")
    left = cp.read_pf_bag(args.left_bag, time_source="stamp")
    right = cp.read_pf_bag(args.right_bag, time_source="stamp")

    gt_x = [p[1] for p in gt]
    gt_y = [p[2] for p in gt]

    fig, axes = plt.subplots(1, 2, figsize=(12, 6), sharex=True, sharey=True)

    for ax, track, title, color in [
        (axes[0], left, args.left_label, "tab:red"),
        (axes[1], right, args.right_label, "tab:green"),
    ]:
        ax.plot(gt_x, gt_y, color="tab:blue", linewidth=2, label="NDT ground truth", zorder=2)
        px = [p[1] for p in track]
        py = [p[2] for p in track]
        ax.plot(px, py, color=color, linewidth=1.2, alpha=0.85, label="PF (%s)" % title, zorder=3)
        ax.scatter([px[0]], [py[0]], color=color, marker="o", s=40, zorder=4, label="PF start")
        ax.set_title(title)
        ax.set_xlabel("x (m)")
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best", fontsize=8)

    axes[0].set_ylabel("y (m)")
    fig.suptitle(args.title)
    fig.tight_layout()
    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=150)
    print("wrote", out_path)
    print("gt points:", len(gt), "left points:", len(left), "right points:", len(right))


if __name__ == "__main__":
    main()
