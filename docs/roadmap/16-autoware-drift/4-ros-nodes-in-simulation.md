# Phase 4: ROS nodes in simulation

**Status:** planned. **Needs:** phase 3. Uses phase 2's measured specs once they exist.

[← Phase 3](3-scale-and-course.md) · [Overview](README.md) · [Phase 5 →](5-cameras-and-estimator.md)

## Goal

The race stack runs as ROS 2 nodes inside AutoSDV's Autoware, on Autoware's seams, and
drives repeated laps — first on the RC simulator, then on car A's Isaac twin on the COSS
world — under the speed cap, with the true state standing in for the estimator.

## autoware-drift (generic)

- **4.1 `race_msgs`**: `RacePlan` (per point: s, pose, speed, sideslip, yaw rate, mode).
- **4.2 Planner node** (`race_planner_ros`) around `live.Driver`: subscribes `/race/state`
  and the `VelocityLimit` on `/planning/scenario_planning/max_velocity_default`; publishes
  `/race/plan` and Autoware's `/planning/scenario_planning/trajectory`; L1 re-plans in
  worker processes so the node's executor never blocks.
- **4.3 C++ NMPC controller** (`race_controller`): a ROS 2 component; model and cost in
  acados' Python front end, generated C linked by the node; references path, speed,
  sideslip, yaw rate from `/race/plan`; publishes `/control/trajectory_follower/control_cmd`.
  Checked against `tracker.py` on recorded states: same inputs, commands within a stated
  tolerance, as a test.
- **4.4 Estimator, stage zero**: `race_estimator` relays the simulator's truth
  (`/localization/kinematic_state`) as `/race/state`, with the CoM offset from `base_link`.
  The filter is phase 5's.
- **4.5 `race_launch`**: a generic `race.launch.xml` taking course, spec, scale and cap.
- Builds skip themselves without CasADi or acados.

## AutoSDV

- **4.6 Race launch** on `waypoint_follow.launch.xml`'s pattern: AutoSDV's vehicle stack,
  the gate with `vel_lim` = cap + margin, operation modes, MRM,
  `launch_trajectory_follower:=false`, the race nodes, and an `autosdv-race-main.yaml`
  diagnostic graph (overspeed and leaving the course corridor as faults).
- **4.7 Laps past the route's goal.** Find what Autoware does on route ARRIVED (route state,
  operation mode, anything that stops the car) with a test run; then keep it from ending
  the race (for example a goal the planner never reaches, or a route reset each lap), and
  write down which and why.
- **4.8 `speed_cap:=`** sets the planner's `v_max`, the published `VelocityLimit` and the
  gate's `vel_lim` (the vehicle interface's guard joins in phase 6).
- **4.9 `just race sim`**: the RC simulator in race mode, reading the car's
  `vehicle_spec.yaml` passed explicitly (the lab default stays as released); the
  simulator's plant must agree with the spec's model where both have the quantity.
- **4.10 `just race twin`**: car A's twin (rear-drive spec from phase 2) on the COSS Isaac
  world, domain 47, as `just twin` runs it.

## Exit

- On the RC sim and on the twin: ten consecutive laps of the COSS course at a cap of
  1.5 m/s, without a stop at the route's goal, never over the cap, no MRM.
- Lab 2 Part 1's grader and Lab 3's `sim_lap.py` give the same results as before the phase;
  `just test` passes.
- Lap times and tracking error against the offline plan, in the phase's report.
