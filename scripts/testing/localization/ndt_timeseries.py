#!/usr/bin/env python3
"""Record NDT's internal signals against time and plot them.

    python3 scripts/testing/localization/ndt_timeseries.py --seconds 300 -o tmp/ndt

Writes <out>.csv and <out>.png. Built for one question: when localization walks
into a wall, WHICH signal moved first?

The signals, and what each one tells you:

  iteration_num    optimiser effort. Hitting max_iterations means it ran out of
                   budget before converging -- the pose published is wherever it
                   had got to, not a converged solution. This is the earliest
                   honest warning available.
  NVTL / TP        fit scores. NVTL is a mean per-point likelihood, so it also
                   moves with how the input is sampled; read it as a trend
                   against its own gate, never as an absolute quality.
  init_to_result   how far NDT moved the EKF's prior. Small and steady is
                   healthy. A jump means the prior and the scan disagreed, which
                   is what a mis-tracked turn looks like from inside.
  skipping_publish_num  consecutive rejected results. NDT deactivates when this
                   passes its limit, and then the EKF dead-reckons in silence.
  yaw rate/speed   plotted underneath so a divergence can be lined up against
                   the manoeuvre that caused it, which for this bag is a turn.

Everything is stamped on the SIM clock, so the x axis lines up with bag time and
with anything else recorded from the same replay.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time

# Keep to the apt python stack: ~/.local's numpy shadows the apt one that the
# apt scipy/matplotlib were built against.
if os.environ.get("PYTHONNOUSERSITE") != "1":
    os.environ["PYTHONNOUSERSITE"] = "1"
    os.execv(sys.executable, [sys.executable] + sys.argv)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--seconds", type=float, default=300.0)
    parser.add_argument("-o", "--out", default="tmp/ndt_timeseries",
                        help="output prefix; relative paths land "
                             "under the repo's ./tmp (see CLAUDE.md)")
    parser.add_argument("--label", default="")
    args = parser.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from autoware_internal_debug_msgs.msg import Float32Stamped, Int32Stamped
    from diagnostic_msgs.msg import DiagnosticArray
    from nav_msgs.msg import Odometry

    B = "/localization/pose_estimator/"
    rows = []          # dicts keyed by signal, stamped on sim time
    state = {}
    t0 = [None]

    def stamp(node):
        t = node.get_clock().now().nanoseconds * 1e-9
        if t0[0] is None:
            t0[0] = t
        return t - t0[0]

    class Rec(Node):
        def __init__(self):
            super().__init__(
                "ndt_timeseries",
                parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
            f = lambda k: (lambda m: state.__setitem__(k, m.data))  # noqa: E731
            self.create_subscription(Float32Stamped, B + "nearest_voxel_transformation_likelihood",
                                     f("nvtl"), 30)
            self.create_subscription(Float32Stamped, B + "transform_probability", f("tp"), 30)
            self.create_subscription(Int32Stamped, B + "iteration_num", f("iters"), 30)
            self.create_subscription(Float32Stamped, B + "exe_time_ms", f("exe_ms"), 30)
            self.create_subscription(Float32Stamped, B + "initial_to_result_distance",
                                     self.on_i2r, 30)
            self.create_subscription(Odometry, "/localization/kinematic_state", self.on_odom, 30)
            self.create_subscription(DiagnosticArray, "/diagnostics", self.on_diag, 50)
            self.prev_yaw = None
            self.prev_t = None

        def on_diag(self, msg):
            for s in msg.status:
                if "ndt_scan_matcher" in s.name and "scan_matching_status" in s.name:
                    kv = {k.key: k.value for k in s.values}
                    for key, dst in (("skipping_publish_num", "skip"),
                                     ("sensor_points_size", "n_pts"),
                                     ("is_activated", "activated")):
                        if key in kv:
                            state[dst] = kv[key]
                    state["diag_msg"] = s.message[:60]

        def on_i2r(self, msg):
            state["i2r"] = msg.data

        def on_odom(self, msg):
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            t = stamp(self)
            v = msg.twist.twist.linear
            speed = math.sqrt(v.x * v.x + v.y * v.y)
            yaw_rate = 0.0
            if self.prev_yaw is not None and self.prev_t is not None and t > self.prev_t:
                d = yaw - self.prev_yaw
                d = math.atan2(math.sin(d), math.cos(d))
                yaw_rate = math.degrees(d / (t - self.prev_t))
            self.prev_yaw, self.prev_t = yaw, t
            # One row per pose: the pose stream is the densest and every other
            # signal is carried forward from its last value, so the rows line up
            # on one clock instead of being interleaved at different rates.
            rows.append(dict(t=t, x=p.x, y=p.y, speed=speed, yaw_rate=yaw_rate,
                             nvtl=state.get("nvtl"), tp=state.get("tp"),
                             iters=state.get("iters"), exe_ms=state.get("exe_ms"),
                             i2r=state.get("i2r"), skip=state.get("skip"),
                             activated=state.get("activated"),
                             diag=state.get("diag_msg", "")))

    rclpy.init()
    node = Rec()
    deadline = time.time() + args.seconds
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

    if not rows:
        print("no data — is the replay running and NDT activated?", file=sys.stderr)
        return 1

    import csv
    # A relative --out is resolved against the repo root, not the cwd, so the
    # default lands in ./tmp wherever the script is run from.
    out = args.out
    if not os.path.isabs(out):
        repo_root = os.path.dirname(os.path.dirname(os.path.dirname(
            os.path.dirname(os.path.abspath(__file__)))))
        out = os.path.join(repo_root, out)
    os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
    csv_path = out + ".csv"
    keys = ["t", "x", "y", "speed", "yaw_rate", "nvtl", "tp", "iters", "exe_ms",
            "i2r", "skip", "activated", "diag"]
    with open(csv_path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    def col(k):
        return [r[k] if r[k] is not None else float("nan") for r in rows]

    t = col("t")
    fig, ax = plt.subplots(5, 1, figsize=(14, 13), sharex=True)
    ttl = f"NDT internals over time{'  —  ' + args.label if args.label else ''}"
    fig.suptitle(ttl, fontsize=13)

    ax[0].plot(t, col("nvtl"), lw=1.0, label="NVTL")
    ax[0].plot(t, col("tp"), lw=1.0, alpha=0.7, label="transform probability")
    ax[0].axhline(1.3, ls="--", c="r", lw=1, label="NVTL gate 1.3")
    ax[0].set_ylabel("score"); ax[0].legend(loc="upper right", fontsize=8)

    ax[1].plot(t, col("iters"), lw=1.0, c="tab:purple")
    ax[1].axhline(30, ls="--", c="r", lw=1, label="max_iterations 30")
    ax[1].set_ylabel("iterations"); ax[1].legend(loc="upper right", fontsize=8)

    ax[2].plot(t, col("i2r"), lw=1.0, c="tab:orange")
    ax[2].set_ylabel("init→result [m]")

    ax[3].plot(t, col("exe_ms"), lw=1.0, c="tab:green")
    ax[3].set_ylabel("exe time [ms]")

    ax[4].plot(t, col("speed"), lw=1.0, c="tab:blue", label="speed [m/s]")
    ax4b = ax[4].twinx()
    ax4b.plot(t, col("yaw_rate"), lw=0.8, c="tab:red", alpha=0.7, label="yaw rate [deg/s]")
    ax4b.set_ylabel("yaw rate [deg/s]", color="tab:red")
    ax[4].set_ylabel("speed [m/s]"); ax[4].set_xlabel("bag time [s]")
    ax[4].legend(loc="upper left", fontsize=8)

    for a in ax:
        a.grid(alpha=0.3)
    fig.tight_layout()
    png = out + ".png"
    fig.savefig(png, dpi=110)

    # A second figure: the path, coloured by NVTL, so a divergence can be located
    # on the map rather than only in time.
    fig2, bx = plt.subplots(figsize=(9, 9))
    nv = col("nvtl")
    sc = bx.scatter(col("x"), col("y"), c=nv, s=6, cmap="viridis")
    bx.plot(rows[0]["x"], rows[0]["y"], "r*", ms=14, label="start")
    bx.set_aspect("equal"); bx.grid(alpha=0.3)
    bx.set_xlabel("map x [m]"); bx.set_ylabel("map y [m]")
    bx.set_title(f"path coloured by NVTL{'  —  ' + args.label if args.label else ''}")
    fig2.colorbar(sc, ax=bx, label="NVTL")
    bx.legend()
    png2 = out + "_path.png"
    fig2.savefig(png2, dpi=110, bbox_inches="tight")

    # Summary, so configurations can be compared without opening the figures.
    # Ranked on what moves FIRST: iteration_num saturates well before NVTL
    # crosses its gate, so it is the early warning and the score is the lagging
    # one.
    import statistics as st

    def med(vals):
        vals = [v for v in vals if v is not None and not (isinstance(v, float) and math.isnan(v))]
        return st.median(vals) if vals else float("nan")

    sp = col("speed")
    it = col("iters")
    nv = col("nvtl")
    i2 = col("i2r")
    moving = [i for i, s_ in enumerate(sp) if s_ > 0.5]
    parked = [i for i, s_ in enumerate(sp) if s_ <= 0.5]
    first = lambda idxs: (t[idxs[0]], rows[idxs[0]]["x"], rows[idxs[0]]["y"]) if idxs else None  # noqa: E731
    cap = [i for i in range(len(t)) if it[i] and it[i] >= 30]
    dead = [i for i in range(len(t)) if t[i] > 5 and it[i] == 0 and nv[i] == 0]

    print()
    for name, ev in (("motion starts", first(moving)),
                     ("iterations hit cap", first(cap)),
                     ("NDT flatlines", first(dead))):
        if ev:
            print(f"  {name:20s} t={ev[0]:6.1f}s  at ({ev[1]:.0f},{ev[2]:.0f})")
        else:
            print(f"  {name:20s} never")
    for name, idxs in (("parked", parked), ("moving", moving)):
        if not idxs:
            continue
        print(f"    {name:7s} NVTL {med([nv[i] for i in idxs]):5.2f}   "
              f"iters {med([it[i] for i in idxs]):5.1f}   "
              f"init->result {med([i2[i] for i in idxs]):5.2f} m")
    print()
    print(f"  {len(rows)} samples over {t[-1]:.0f} s of bag time")
    print(f"  csv  {csv_path}")
    print(f"  plot {png}")
    print(f"  path {png2}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
