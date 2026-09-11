#!/usr/bin/env python3
"""Report whether ndt_scan_matcher is ACTIVATED, not merely alive.

    python3 scripts/testing/localization/check_ndt_activated.py --timeout 20

Exit 0 when activated, 1 when not. Prints why when it can.

**Why the initialize service's own success code is not enough.** Reading the
Autoware sources (autoware_pose_initializer, autoware_ndt_scan_matcher, 1.5.0):

    on_initialize(AUTO):
        change_node_trigger(false)     # NDT and EKF DEACTIVATED
        ndt_->align_pose(pose)         # calls /ndt_align_srv
        ...
        change_node_trigger(true)      # reactivated -- only if we get here

`is_activated_` in ndt_scan_matcher is written *only* by that trigger service;
the node never activates itself. And service_ndt_align_main returns
success=false whenever an input is missing:

    - TF map <- pose.frame_id
    - is_set_map_points    (getInputTarget()  == nullptr, map not loaded)
    - is_set_sensor_points (getInputSource()  == nullptr, no scan accepted yet)

LocalizationModule::align_pose THROWS on !success, the exception leaves
on_initialize, and `change_node_trigger(true)` is never reached. NDT is then
latched OFF and stays off: nothing retries it, and the EKF keeps dead-reckoning
on IMU and velocity, so /localization/kinematic_state still runs at 40 Hz and
the vehicle still moves across the map. Nothing in that picture says "broken".

getInputSource() is itself conditional: callback_sensor_points returns BEFORE
setInputSource() when the sensor->base_link transform fails, or when the scan's
max point distance is under `sensor_points.required_distance` (default 10 m).
So a short-range or untransformable scan silently starves the align.
"""

from __future__ import annotations

import argparse
import sys
import time

# The two diagnostics that carry is_activated, most specific first.
STATUS_KEYS = ("scan_matching_status", "trigger_node_service_status")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--timeout", type=float, default=20.0,
                        help="seconds to wait for an activated report (default 20)")
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from diagnostic_msgs.msg import DiagnosticArray

    state: dict[str, tuple[bool, str, dict]] = {}

    class Watch(Node):
        def __init__(self):
            # Replays run on bag time; without this the node's clock is wall time
            # and any future timestamp logic here would be nonsense.
            super().__init__(
                "check_ndt_activated",
                parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
            )
            self.create_subscription(DiagnosticArray, "/diagnostics", self.on_diag, 50)

        def on_diag(self, msg):
            for s in msg.status:
                if "ndt_scan_matcher" not in s.name:
                    continue
                kv = {k.key: k.value for k in s.values}
                if "is_activated" not in kv:
                    continue
                for key in STATUS_KEYS:
                    if key in s.name:
                        state[key] = (kv["is_activated"].lower() == "true", s.message, kv)

    rclpy.init()
    node = Watch()
    deadline = time.time() + args.timeout
    activated = False
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        for key in STATUS_KEYS:
            if key in state and state[key][0]:
                activated = True
                break
        if activated:
            break
    node.destroy_node()
    rclpy.shutdown()

    if activated:
        if not args.quiet:
            print("  ndt_scan_matcher is ACTIVATED")
        return 0

    if not args.quiet:
        if not state:
            print("  no ndt_scan_matcher diagnostics on /diagnostics — is the stack up?",
                  file=sys.stderr)
        for key in STATUS_KEYS:
            if key not in state:
                continue
            _, message, kv = state[key]
            print(f"  {key}: {message}", file=sys.stderr)
            # These three are the align preconditions; whichever is False is the
            # reason initialization threw before it could reactivate the node.
            for k in ("is_activated", "is_set_map_points", "is_set_sensor_points",
                      "is_succeed_transform_sensor_points", "sensor_points_max_distance",
                      "sensor_points_delay_time_sec"):
                if k in kv:
                    print(f"      {k}: {kv[k]}", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
