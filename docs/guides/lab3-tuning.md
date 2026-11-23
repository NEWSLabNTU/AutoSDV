# Lab 3: files that shape the autonomous lap

What makes the car stop near walls and corners, slow in curves, or trigger an emergency
stop, and the file that sets each. Gathered from `lab3/run.sh`'s launch
(`planning_profile:=rc`, Autoware 1.5.0 presets) on 2026-11-23. Values are as shipped.

`/opt/...` below is `/opt/autoware/1.5.0/share/autoware_launch/config`, root-owned.

**The copies a team edits.** With `planning_profile:=rc` (`lab3/run.sh`) the stack reads
seven of these files from this repository instead, copied unchanged from Autoware 1.5.0
(values not pre-tuned):

| copy (under `src/launcher/autosdv_launch/config/`) | upstream |
|---|---|
| `planning_rc/obstacle_stop.param.yaml` | `/opt/.../motion_velocity_planner/obstacle_stop.param.yaml` |
| `planning_rc/obstacle_slow_down.param.yaml` | `/opt/.../motion_velocity_planner/obstacle_slow_down.param.yaml` |
| `planning_rc/velocity_smoother.param.yaml` | `/opt/.../planning/scenario_planning/common/autoware_velocity_smoother/velocity_smoother.param.yaml` |
| `control_rc/mpc.param.yaml` | `/opt/.../control/trajectory_follower/lateral/mpc.param.yaml` |
| `control_rc/autonomous_emergency_braking.param.yaml` | `/opt/.../control/autoware_autonomous_emergency_braking/...` |
| `control_rc/control_validator.param.yaml` | `/opt/.../control/control_validator/...` |
| `control_rc/lane_departure_checker.param.yaml` | `/opt/.../control/lane_departure_checker/...` |

The others below stay read-only references. `control_profile` follows `planning_profile`;
`control_profile:=stock` keeps rc planning with Autoware's control files. Checked with
`play_launch dump`: every node's parameters are the same as before the copies.

## How a wall turns into a stop

Perception clusters the LiDAR points that the compare-map filter does not remove (points
farther than `distance_threshold` from the map) into `unknown` objects. A wall corner the
map holds only roughly, or a person or a bag near the loop, becomes an object. Planning
then stops or slows for it. Separately, control's emergency braking (AEB) checks the raw
point cloud against the predicted path.

## 1. Stops for obstacles (motion velocity planner)

| file | parameter | value | effect |
|---|---|---|---|
| `/opt/.../motion_velocity_planner/obstacle_stop.param.yaml` | `stop_margin` | 5.0 m | how far before an obstacle the car stops; 5 m is a car's length on a city street and almost a whole corridor leg here |
| same | `terminal_stop_margin`, `min_behavior_stop_margin` | 3.0 m | the same, at the goal and behind behaviour stops |
| same | `obstacle_filtering.lateral_margin.nominal` (default, unknown) | 0.3 m | an object this close to the footprint's side counts as in the path |
| same | `stop_on_curve.*`, `rss_params.*` | — | curve and braking-distance variants of the margin |
| `/opt/.../motion_velocity_planner/obstacle_slow_down.param.yaml` | `min_lat_margin` / `max_lat_margin` | −10 / 1.1 m | objects within 1.1 m of the path band slow the car; the speed falls as the gap narrows (the per-type `min/max_lat_margin` 0.2 / 1.0 m) |
| `/opt/.../motion_velocity_planner/obstacle_cruise.param.yaml` | — | — | following a moving object; rarely the cause indoors |
| `/opt/.../motion_velocity_planner/motion_velocity_planner.param.yaml` | `pointcloud_preprocessing.lateral_margin` | 2.0 m | which points the planner considers at all |
| `/opt/.../motion_velocity_planner/obstacle_velocity_limiter.param.yaml` | `dynamic_source`, `distance_buffer` | static_only, 0.0 | caps speed by the distance the car could brake to an obstacle |
| `/opt/.../motion_velocity_planner/boundary_departure_prevention.param.yaml` | `boundary_types_to_detect`, `slow_down_behavior.*`, `abnormality.localization.footprint_envelop` | road_border; 0.25 m | slows or stops near a lanelet `road_border`; inactive if the map's bounds are not tagged `road_border` |
| `/opt/.../motion_velocity_planner/out_of_lane.param.yaml`, `run_out.param.yaml`, `dynamic_obstacle_stop.param.yaml`, `road_user_stop.param.yaml` | — | — | stops for objects entering the lane; on in the preset |

The modules switch on and off in `/opt/.../planning/preset/default_preset.yaml`
(`launch_obstacle_stop_module`, `launch_obstacle_slow_down_module`, ...).

## 2. Emergency stops (control)

| file | parameter | value | effect |
|---|---|---|---|
| `/opt/.../control/autoware_autonomous_emergency_braking/autonomous_emergency_braking.param.yaml` | `use_pointcloud_data` | true | AEB brakes for raw points along the predicted path |
| same | `expand_width`, `longitudinal_offset_margin`, `t_response`, `a_ego_min`, `collision_keeping_sec` | −0.2 m, 1.0 m, 1.0 s, −3.0, 3.0 s | the width it checks, the distance it keeps, how long it holds the stop |
| `/opt/.../control/control_validator/control_validator.param.yaml` | `thresholds.max_distance_deviation` | 1.0 m | path-following error that triggers the emergency stop (MRM) |
| `/opt/.../control/lane_departure_checker/lane_departure_checker.param.yaml` | `footprint_margin_scale`, `footprint_extra_margin` | 1.0, 0.0 | when the car counts as leaving its lane |
| `/opt/.../control/preset/default_preset.yaml` | `launch_autonomous_emergency_braking`, `launch_collision_detector`, `launch_control_validator` | true | the checks on and off |

In the 23 Nov screencast the MRM panel showed `emergency_stop` about 3 s after Auto.

## 3. Curves and speed

| file | parameter | value | effect |
|---|---|---|---|
| `/opt/.../planning/scenario_planning/common/autoware_velocity_smoother/velocity_smoother.param.yaml` | `max_lateral_accel`, `min_curve_velocity`, `decel_distance_before_curve` / `_after_curve` | —, 2.0 m/s, 3.5 / 2.0 m | slowing for curves |
| same | `engage_velocity`, `engage_acceleration`, `stop_dist_to_prohibit_engage` | 0.25 m/s, 0.5, 0.5 m | pulling away from a stop |
| `/opt/.../planning/scenario_planning/common/common.param.yaml` | `max_vel`, `normal.min_acc` / `max_acc`, `limit.*` | 4.17 m/s, −1.0 / 1.0 | speed and acceleration caps |
| `src/planning/autosdv_speed_limit/config/speed_limit.yaml` | `max_speed_kmh` | 3.0 | the overall limit (allowed now) |
| the lanelet map | `speed_limit` per lanelet | ≤ 10 | per-segment limit (allowed now) |

## 4. The path itself (already in the repository)

`src/launcher/autosdv_launch/config/planning_rc/` (README.md there says why each
changed): `behavior_path_planner.param.yaml` (path resolution, drivable area),
`path_optimizer.param.yaml` (the car's curvature), `trajectory_checker.param.yaml`
(the planning validator's limits), `mission_planner.param.yaml`, `goal_planner.param.yaml`,
`start_planner.param.yaml`.

## 5. Tracking (control, already in the repository or per car)

- MPC: `/opt/.../control/trajectory_follower/lateral/mpc.param.yaml` (the main stack's;
  the copy under `config/control/trajectory_follower/` is for the pursuit launches).
- Longitudinal PID: `src/launcher/autosdv_launch/config/control/autoware/longitudinal/pid.param.yaml`.
- Per car: `individual_params/config/<car>/autosdv_vehicle/speed_controller.param.yaml`
  and `actuator.param.yaml` (shared by the three teams on a car).

## 6. What becomes an obstacle (perception)

| file | parameter | value | effect |
|---|---|---|---|
| `src/lab/autosdv_lab3/config/pointcloud_map_filter_indoor.param.yaml` | `distance_threshold`, `down_sample_voxel_size` | 0.5 m, 0.1 m | points within 0.5 m of the map are dropped before clustering; a map that misses a wall or a desk leaves those points as objects |
| `src/launcher/autosdv_launch/config/perception/obstacle_segmentation/`, `object_recognition/detection/` | crop box, ground filter, clustering | — | which points reach the planner and AEB |

A clean point-cloud map (Part 1) is the first lever: what the map holds is filtered out
and never becomes an obstacle.
