# 2D-LiDAR Planning & Control Integration — Design Spec

Date: 2026-07-25
Status: approved (design review in session)
Companion doc: `docs/design/f1tenth-2dlidar-integration.typ` (localization side; to be revised per this spec)

## Goal

Extend the 2D-LiDAR low-cost AutoSDV variant beyond localization: wire the 2D
LiDAR (`/scan`) and its derived occupancy grid / obstacle pointcloud into
Autoware 1.5.0 planning and control, and define exactly which planning/control
modules are kept, reconfigured, or disabled. No behavior change for stock 3D
builds.

## Key audit findings (Autoware 1.5.0 at `/opt/autoware/1.5.0`)

1. **The primary obstacle channel is the pointcloud, not the occupancy grid.**
   `/perception/obstacle_segmentation/pointcloud` feeds:
   `motion_velocity` obstacle_stop, behavior_velocity `detection_area`, AEB,
   `collision_detector`, and parking `costmap_generator`. The occupancy grid
   (`/perception/occupancy_grid_map/map`) is consumed only by `goal_planner`
   goal search, `obstacle_velocity_limiter` (opt-in source), and
   behavior_path; no enabled behavior_velocity module requires it.
2. **`motion_velocity_obstacle_stop_module` natively stops on raw,
   unclassified pointcloud clusters** (`pointcloud_segmentation` namespace).
   No custom 2D obstacle-stop node is needed.
3. **`LaserscanBasedOccupancyGridMapNode` can consume a real `/scan`
   directly** when launched standalone (bypassing its stock launch's internal
   `pointcloud_to_laserscan` wrapper) with both pointcloud-input flags false.
4. **`empty_objects_publisher` is load-bearing.** Behavior/motion planners
   gate readiness on a `PredictedObjects` message when object-consuming
   modules are loaded; an empty message unblocks them and modules no-op
   gracefully. AutoSDV already runs it when `launch_perception:=false`.
5. **The `traffic_light` module stalls the vehicle** at any mapped traffic
   light when no traffic-light recognition exists — must be disabled.
6. **Gap: AutoSDV's `config/planning/**` and `config/control/**` trees are
   dormant.** `autosdv_autoware.launch.xml` includes stock
   `tier4_planning_component.launch.xml` / `tier4_control_component.launch.xml`,
   which resolve all params from `/opt/autoware/1.5.0/share/autoware_launch/config/`.
   Any preset/param work is inert until this is fixed.
7. 1.5.0's `run_out` module (motion_velocity) is predicted-objects only — the
   legacy pointcloud "Points" detection method no longer exists.

## Architecture

```
/scan (urg_node, 2D LiDAR)
  ├─→ particle_filter → pose relay → ekf_localizer        (localization; existing design doc)
  ├─→ laserscan_to_pointcloud_node
  │      → /perception/obstacle_segmentation/pointcloud
  │        └─→ obstacle_stop · detection_area · AEB · collision_detector · costmap_generator
  └─→ LaserscanBasedOccupancyGridMapNode (standalone)
         → /perception/occupancy_grid_map/map
           └─→ goal_planner goal search · obstacle_velocity_limiter · behavior_path
empty_objects_publisher → /perception/object_recognition/objects   (readiness unblock)
```

New planning/control code: **none**. Two stock nodes rewired + configuration.
Localization-side custom code remains the single pose relay from the existing
design doc.

## Components

### 1. Forked component launches (fixes finding 6)

New in `src/launcher/autosdv_launch/launch/components/`:

- `autosdv_planning_component.launch.xml` — copy of
  `tier4_planning_component.launch.xml`, config root repointed to
  `$(find-pkg-share autosdv_launch)/config/planning/`.
- `autosdv_control_component.launch.xml` — same for control.

`autosdv_autoware.launch.xml` switches to these includes. Consequences:

- The existing dormant config mirror becomes live (verify diff vs
  `/opt` configs on first bring-up; resolve the velocity_smoother
  JerkFiltered-vs-Analytical mismatch explicitly).
- Upgrade cost: re-diff the two forked launch files on each Autoware bump.

### 2. Planning preset `2dlidar`

`src/launcher/autosdv_launch/config/planning/preset/2dlidar_preset.yaml`
(follows `<name>_preset.yaml` convention), selected via
`planning_preset:=2dlidar` (implied by the 2D sensor suite).

Minimal-trust module policy:

| Action | Modules | Rationale |
|---|---|---|
| Keep (fully functional) | mission_planner, scenario_selector, path_smoother (elastic_band), path_optimizer, velocity_smoother, freespace_planner, stop_line, walkway, merge_from_private, no_stopping_area, side_shift, goal_planner, start_planner | lanelet2 + odometry + grid/costmap only |
| Keep — pointcloud-driven safety | motion_velocity `obstacle_stop`; behavior_velocity `detection_area` | native raw-pointcloud support |
| Keep — reconfigured | `obstacle_velocity_limiter` (`dynamic_source: occupancy_grid`), `costmap_generator`, optional `obstacle_slow_down` (`object_type.pointcloud: true`) | grid/pointcloud sources |
| Keep — degraded, documented | crosswalk, intersection, blind_spot (geometric stops, cannot yield to unseen agents), static_obstacle_avoidance (inert) | harmless; preserves Autoware surface |
| Disable | lane_change L/R, avoidance_by_lane_change | would execute blind maneuvers — RSS check passes trivially with empty objects |
| Disable | traffic_light | stalls at mapped lights without TL recognition |
| Disable | obstacle_cruise, dynamic_obstacle_stop, out_of_lane, run_out, road_user_stop | predicted-objects-only; permanent no-ops in this stack |
| Note | planning_validator: Trajectory/Latency checkers active; RearCollisionChecker non-functional (needs 3D z-clustering); IntersectionCollisionChecker inert | document, no config change required |

### 3. Param overrides (2D variants in `autosdv_launch/config/`)

- `costmap_generator`: `use_objects: false`; lower
  `minimum_lidar_height_thres` so the fixed-height scan ring passes the
  base_link height gate.
- `collision_detector`: `use_pointcloud: true`, `use_dynamic_object: false`.
- AEB: unchanged (stock topics now fed by scan-derived cloud; objects path
  already off; `use_imu_path` stays on — IMU exists).
- `laserscan_based_occupancy_grid_map.param.yaml`: frames set,
  `height_filter.use_height_filter: false`.

### 4. Sensing wiring

In the sensor kit / launcher, gated by new `lidar_model:=2d` (or a new
`sensor_suite`):

- 2D LiDAR driver (`urg_node2` for Hokuyo; `rplidar_ros` alternative) +
  static TF `base_link → laser`.
- `laserscan_to_pointcloud_node` (from installed `pointcloud_to_laserscan`
  pkg): `scan_in:=/scan`, `cloud:=/perception/obstacle_segmentation/pointcloud`,
  `target_frame:=base_link`.
- `LaserscanBasedOccupancyGridMapNode` standalone:
  `~/input/laserscan:=/scan`, `input_obstacle_pointcloud:=false`,
  `input_obstacle_and_raw_pointcloud:=false`,
  `~/output/occupancy_grid:=/perception/occupancy_grid_map/map`.
- Assert `empty_objects_publisher` runs whenever the 2D variant is active.

### 5. Control

All enabled control components keep working unchanged: trajectory_follower
(MPC lateral + PID longitudinal), vehicle_cmd_gate, shift_decider,
operation_mode_transition_manager, external_cmd selector/converter,
lane_departure_checker, control_validator, control_evaluator. AEB and
collision_detector become functional via the scan-derived cloud (see §3).
`obstacle_collision_checker` and `predicted_path_checker` stay disabled.

## Vendoring / merge-back plan

- Roboracer (F1TENTH) packages needed by the localization side
  (`particle_filter`, `range_libc`) are added as **git submodules pointing at
  upstream URLs**. Fork to **NEWSLabNTU** only if AutoSDV patches are needed,
  then repoint the submodule.
- Planning/control work in this spec lands in `autosdv_launch`
  (superproject) — no submodule churn.
- 2D sensor-kit wiring lands in `autosdv_sensor_kit_launch` (NEWSLabNTU
  submodule): branch, push, bump pin in superproject.

## Safety & error handling

- Primary backstop: `obstacle_stop` on raw scan clusters; secondary: AEB on
  the same cloud.
- Maps used with the 2D variant must have no traffic-light regulatory
  elements (module disabled), or the limitation must be documented per map.
- Known blind spots documented in the book: no yielding to dynamic agents,
  no obstacle detection outside the scan plane, no rear 3D collision check.

## Testing

- Planning-sim smoke test: `2dlidar` preset boots with empty objects; no
  module blocks readiness; trajectory published.
- Rosbag replay with recorded `/scan`: occupancy grid published at expected
  rate; obstacle_stop inserts stop at a planted obstacle — verified before
  any actuation.
- Topic liveness gates: `/perception/obstacle_segmentation/pointcloud`,
  `/perception/occupancy_grid_map/map`, `/localization/kinematic_state`.
- Closed-loop low-speed lane following per the phased plan in the design
  doc (phases 5–7 unchanged).

## Documentation updates

Revise `docs/design/f1tenth-2dlidar-integration.typ`:

- Replace the proposed custom "2D obstacle stop node" with the stock
  `obstacle_stop` + `detection_area` wiring.
- Correct the occupancy-grid role: pointcloud channel is the primary
  obstacle path; grid serves goal search / velocity limiting.
- Add the module keep/degrade/disable table and the forked component-launch
  note (finding 6).
