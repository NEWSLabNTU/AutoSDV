#!/usr/bin/env python3
"""Audit the two inputs the EKF prior is built from: IMU and VelocityReport.

    python3 scripts/testing/localization/check_imu_velocity.py --seconds 200

NDT moves the incoming pose ~1 m per scan while moving against 9 cm parked, and
no amount of resampling either point cloud changes that. That is a prior problem,
and the prior is gyro_odometer(IMU, VelocityReport) fused by the EKF. This checks
those two signals for the three ways they go wrong:

  COORDINATES  is the IMU in REP-103 body axes (x forward, y left, z up)? Read
               off gravity while parked: a level REP-103 IMU reports az near
               +9.81, and a sign or axis swap shows up immediately. Yaw rate must
               also share a sign convention with the pose: if wz and the pose's
               yaw rate disagree in sign, every turn is integrated backwards.

  SCALE        integrate each signal over a window where NDT is still tracking
               and compare against the pose the map produced. A velocity scale
               error of a few percent is invisible per scan and is exactly what
               drags a prior a metre behind over a turn. Reported as a ratio, so
               1.00 is correct and 0.90 means the vehicle really moved 11% more
               than the wheel/VCU said.

  QUALITY      gyro bias and noise while stationary, publication rate, and gaps.
               A rate that sags or a stream with holes makes the EKF extrapolate,
               which produces exactly the speed-proportional lag term seen in the
               init->result regression.

Comparisons use the NDT pose while it is still converged, not the EKF output --
the EKF is downstream of the very signals under test, so scoring them against it
would hide a shared error.
"""

from __future__ import annotations

import argparse
import math
import os
import statistics as st
import sys
import time

if os.environ.get("PYTHONNOUSERSITE") != "1":
    os.environ["PYTHONNOUSERSITE"] = "1"
    os.execv(sys.executable, [sys.executable] + sys.argv)


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def rates(stamps):
    if len(stamps) < 3:
        return float("nan"), float("nan")
    d = [b - a for a, b in zip(stamps, stamps[1:]) if b > a]
    return (1.0 / st.median(d) if d else float("nan")), (max(d) if d else float("nan"))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seconds", type=float, default=200.0)
    # The raw IMU topic follows imu_source: the ZED publishes
    # /sensing/camera/zedxm/imu/data and the MPU9250 publishes
    # /sensing/imu/mpu9250/imu_raw. Only the corrected topic is fixed.
    ap.add_argument("--raw-topic", default="/sensing/camera/zedxm/imu/data",
                    help="raw IMU topic, before imu_corrector "
                         "(default: the ZED's)")
    args = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
    from sensor_msgs.msg import Imu
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from autoware_vehicle_msgs.msg import VelocityReport

    imu, raw, vel, ndt = [], [], [], []

    class Chk(Node):
        def __init__(self):
            super().__init__(
                "check_imu_velocity",
                parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
            be = QoSProfile(depth=50, history=QoSHistoryPolicy.KEEP_LAST,
                            reliability=QoSReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(Imu, "/sensing/imu/imu_data", self.on_imu, 50)
            self.create_subscription(Imu, args.raw_topic, self.on_raw, be)
            self.create_subscription(VelocityReport, "/vehicle/status/velocity_status",
                                     self.on_vel, 50)
            self.create_subscription(PoseWithCovarianceStamped,
                                     "/localization/pose_estimator/pose_with_covariance",
                                     self.on_ndt, 50)

        @staticmethod
        def _t(msg):
            return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        def on_imu(self, m):
            imu.append((self._t(m), m.header.frame_id,
                        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z,
                        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z))

        def on_raw(self, m):
            raw.append((self._t(m), m.header.frame_id,
                        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z,
                        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z))

        def on_vel(self, m):
            vel.append((self._t(m), m.longitudinal_velocity, m.lateral_velocity,
                        m.heading_rate))

        def on_ndt(self, m):
            p = m.pose.pose
            ndt.append((self._t(m), p.position.x, p.position.y, yaw_of(p.orientation)))

    rclpy.init()
    node = Chk()
    deadline = time.time() + args.seconds
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

    if not imu or not vel or len(ndt) < 20:
        print("missing data — imu {} vel {} ndt {}".format(len(imu), len(vel), len(ndt)),
              file=sys.stderr)
        print("  Is the replay running with NDT activated?", file=sys.stderr)
        return 1

    print(f"\n=== rates and continuity ===")
    for name, arr in (("imu_data (corrected)", imu), ("raw imu", raw),
                      ("velocity_status", vel), ("ndt pose", ndt)):
        if not arr:
            print(f"  {name:22s} NO MESSAGES")
            continue
        hz, gap = rates([a[0] for a in arr])
        print(f"  {name:22s} {len(arr):5d} msgs   {hz:6.1f} Hz   largest gap {gap * 1000:6.0f} ms")

    print(f"\n=== coordinates ===")
    print(f"  imu_data frame: {imu[0][1]}    raw frame: {raw[0][1] if raw else '-'}")
    # Stationary window: the vehicle is parked for ~1 min at the start of these bags.
    still = [v[0] for v in vel if abs(v[1]) < 0.05]
    t_still = max(still) if still else imu[len(imu) // 4][0]
    s = [a for a in imu if a[0] <= t_still]
    if len(s) > 20:
        ax, ay, az = (st.fmean([a[i] for a in s]) for i in (2, 3, 4))
        gx, gy, gz = (st.fmean([a[i] for a in s]) for i in (5, 6, 7))
        print(f"  gravity while parked   ax {ax:+6.2f}  ay {ay:+6.2f}  az {az:+6.2f} m/s^2"
              f"   |a| {math.sqrt(ax*ax+ay*ay+az*az):.2f}")
        print("     REP-103 body frame, level: expect az near +9.81, ax and ay near 0")
        print(f"  gyro bias while parked gx {gx:+.5f}  gy {gy:+.5f}  gz {gz:+.5f} rad/s")
        print(f"  gyro noise (stdev)     gz {st.pstdev([a[7] for a in s]):.5f} rad/s")

    # ── scale, against the NDT pose while it is still tracking ──────────────
    print(f"\n=== scale (NDT pose as reference, not the EKF) ===")
    ndt.sort()
    pairs = []
    for i in range(1, len(ndt)):
        t0, x0, y0, yw0 = ndt[i - 1]
        t1, x1, y1, yw1 = ndt[i]
        dt = t1 - t0
        if not (0.05 < dt < 0.5):
            continue
        d = math.hypot(x1 - x0, y1 - y0)
        dyaw = math.atan2(math.sin(yw1 - yw0), math.cos(yw1 - yw0))
        # nearest reports in the interval
        vs = [v[1] for v in vel if t0 <= v[0] <= t1]
        gs = [a[7] for a in imu if t0 <= a[0] <= t1]
        if not vs or not gs:
            continue
        pairs.append((d / dt, st.fmean(vs), dyaw / dt, st.fmean(gs)))

    # Gate on the REPORTED speed, not on the NDT-derived one. Parked NDT poses
    # jitter by a few centimetres between scans, which at 7 Hz is ~0.5 m/s of
    # apparent motion -- enough to pass a naive threshold, and those samples carry
    # a genuine VelocityReport of 0, so they drag the ratio to zero and swamp the
    # real moving samples.
    moving = [p for p in pairs if p[1] > 1.0 and p[0] > 1.0]
    if len(moving) > 30:
        vr = [p[1] / p[0] for p in moving]
        print(f"  speed:  VelocityReport / NDT-derived   ratio median {st.median(vr):.3f}"
              f"   ({len(moving)} samples)")
        print("     1.00 = correct. Below 1 means the vehicle moved further than reported.")
        turning = [p for p in moving if abs(p[2]) > 0.05]
        if len(turning) > 20:
            yr = [p[3] / p[2] for p in turning if abs(p[2]) > 0.05]
            print(f"  yaw rate: gyro wz / NDT-derived      ratio median {st.median(yr):.3f}"
                  f"   ({len(turning)} samples while turning)")
            print("     a NEGATIVE ratio means the gyro sign is inverted for this frame")
    else:
        print(f"  only {len(moving)} moving samples — run longer, or NDT died early")
    return 0


if __name__ == "__main__":
    sys.exit(main())
