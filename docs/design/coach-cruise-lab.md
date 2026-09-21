# Course lab: vehicle interface control and feedback, with coach-car cruise

A lab for the college autonomous driving course, placed **before** localization
and NDT are taught. Students implement the vehicle interface — the control
command path down to PWM, and the velocity feedback path back up — and then
prove it in the field by writing a cruise controller that follows a "coach car".

The coach car is a reflective calibration board carried ahead of the vehicle,
detected in the LiDAR cloud by the detector in
[LCTK](https://github.com/NEWSLabNTU/LCTK) (`~/repos/LCTK`).

The design constraint that shapes everything below: **no map, no localization,
no NDT.** The lab must run without any of it, because none of it has been
taught yet. It should also use the Autoware stack wherever that is honest,
rather than routing around it.

---

## 1. What the lab teaches

Cascaded control. Two loops, both written by students, each with its own
feedback signal:

- **Inner loop — the vehicle interface.** Target velocity in, PWM out, hall
  sensor velocity back. This is the lab's nominal subject.
- **Outer loop — cruise.** Measured headway to the coach board in, target
  velocity out. This is the field exercise, and it is what makes the inner loop
  worth getting right: a sloppy inner loop is invisible on a bench and obvious
  when the vehicle is chasing something.

Everything between the two loops is Autoware, unmodified, and students cannot
bypass it.

---

## 2. Architecture

```
  LiDAR cloud  (/sensing/lidar/top/pointcloud_raw)
       │
       ▼
  ┌──────────────────────────────────────────────────────────────┐
  │ lidar_board_detector            [LCTK, detection_mode: bbox] │  PROVIDED
  └──────────────────────────────┬───────────────────────────────┘
       /perception/coach/detections   vision_msgs/Detection3DArray
       │
       ▼
  ┌──────────────────────────────────────────────────────────────┐
  │ coach_target_tracker                                         │  PROVIDED
  │ constant-velocity track; extrapolates to control time        │  (tier 2: student)
  └──────────────────────────────┬───────────────────────────────┘
       /perception/coach/state        range, range_rate, bearing
       │
       ▼
  ┌──────────────────────────────────────────────────────────────┐
  │ cruise_controller               OUTER LOOP                   │  STUDENT
  └──────────────────────────────┬───────────────────────────────┘
       │  autoware_control_msgs/Control
       ▼
  ┌──────────────────────────────────────────────────────────────┐
  │ safety clamp                                                 │  PROVIDED
  │ speed cap, minimum standoff, detection-loss timeout          │  (not editable)
  └──────────────────────────────┬───────────────────────────────┘
       /control/trajectory_follower/control_cmd
       │
       ▼
  ┌──────────────────────────────────────────────────────────────┐
  │ autoware_vehicle_cmd_gate                                    │  AUTOWARE
  │ engage service, external emergency stop, rate limit, timeout │
  └──────────────────────────────┬───────────────────────────────┘
       /control/command/control_cmd
       │
       ▼
  ┌──────────────────────────────────────────────────────────────┐
  │ actuator                        INNER LOOP                   │  STUDENT
  │ Control → speed PID → PCA9685 PWM                            │
  └──────────────────────────────┬───────────────────────────────┘
                                 ▼  I2C
                            motor ESC, steering servo

  hall sensor ─► velocity_report ─► /vehicle/status/velocity_status   STUDENT
                                    (feedback to BOTH loops)
```

The gate is not decoration. It supplies the engage service, the external
emergency stop, command rate limiting and a command timeout. Students get all
four without writing them, and — because their controller publishes *upstream*
of the gate — cannot remove them. That is simultaneously the safety story and
the answer to "use Autoware if possible".

### Topic and message contract

| Topic | Type | Written by |
|---|---|---|
| `/perception/coach/detections` | `vision_msgs/Detection3DArray` | LCTK detector |
| `/perception/coach/state` | (see §5) | provided tracker |
| `/control/trajectory_follower/control_cmd` | `autoware_control_msgs/Control` | student, via clamp |
| `/control/command/control_cmd` | `autoware_control_msgs/Control` | `vehicle_cmd_gate` |
| `/vehicle/status/velocity_status` | `autoware_vehicle_msgs/VelocityReport` | student |

The gate input and output topic names are fixed by
`/opt/autoware/1.5.0/share/tier4_control_launch/launch/control.launch.xml:96,114`.
The actuator's subscriptions are remapped to the same names in
`src/vehicle/autosdv_vehicle_launch/autosdv_vehicle_launch/launch/vehicle_interface.launch.xml`.

---

## 3. Running the Autoware control stack without localization

`vehicle_cmd_gate`, `operation_mode_transition_manager` and
`trajectory_follower` all subscribe to `/localization/kinematic_state` and
`/localization/acceleration` (`control.launch.xml:111-112,141,163-165`). With no
map and no NDT, those topics have to come from somewhere else.

**The node that produces them already exists.**
`src/localization/autosdv_mcl_launch/autosdv_mcl_launch/wheel_imu_odom.py`
(entry point `mcl_wheel_imu_odom`) integrates `VelocityReport` against the IMU
yaw rate with a unicycle model and publishes `nav_msgs/Odometry`. It was written
for `pose_source:=mcl`, where the particle filter consumes it as a motion model.
The lab needs the same node pointed somewhere else.

```
  /vehicle/status/velocity_status   (VelocityReport, from velocity_report)
  /sensing/imu/imu_data             (Imu, from autoware_imu_corrector)
       │
       ▼
  mcl_wheel_imu_odom          EXISTS, needs a frame_id parameter
       │  /localization/kinematic_state   (frame map → base_link)
       │  + TF map → base_link            (publish_tf:=true)
       ├──────────────────────────────────┐
       ▼                                  ▼
  vehicle_cmd_gate,              autoware_twist2accel
  operation_mode                        │  /localization/acceleration
                                        ▼
                                 vehicle_cmd_gate
```

It fuses wheel speed with IMU yaw rate itself, which is what `gyro_odometer`
exists to do — so `gyro_odometer` and `vehicle_velocity_converter` are not in
this chain at all. `twist2accel` accepts `input/odom` directly, so it reads the
same odometry rather than needing a separate twist. Two existing nodes and one
parameter, in place of a four-stage chain and a new node.

### What has to change in `wheel_imu_odom`

- `header.frame_id` is hardcoded `"odom"` (`wheel_imu_odom.py:103`, and
  `:112` for the TF). It needs a parameter so the lab can publish `map`, which
  is the frame Autoware's control stack assumes.
- `odom_topic` and `publish_tf` are already parameters; the lab sets
  `/localization/kinematic_state` and `true`.
- The node populates no covariance, deliberately — its docstring says the
  particle filter's motion model uses pose deltas only. Whether the control
  chain needs it is recorded in §3.4.

### 3.1 What must not be launched

`launch_localization:=false`. Everything under
`src/localization/tier4_localization_launch/launch/localization.launch.xml` is a
package deal: `pose_twist_estimator`, `pose_twist_fusion_filter` and
`localization_error_monitor` are included unconditionally (`:46-63`), and inside
the fusion filter **none** of `ekf_localizer`, `stop_filter`, `twist2accel` or
`pose_instability_detector` is gated on anything
(`pose_twist_fusion_filter.launch.xml:4-43`). `autoware_pose_initializer` is
likewise ungated (`pose_twist_estimator.launch.xml:172-182`).

Two consequences worth knowing:

- `/localization/kinematic_state` is normally published by **`autoware_stop_filter`**,
  not by `ekf_localizer` — the EKF publishes
  `/localization/pose_twist_fusion_filter/kinematic_state` and the stop filter
  renames it (`pose_twist_fusion_filter.launch.xml:8,24`). So the topic the lab
  is taking over belongs to the stop filter, and with localization off nothing
  contends for it.
- `use_mapless_mode` does **not** turn localization off. It only swaps the
  perception lidar-model parameter directory
  (`autosdv_autoware.launch.xml:244,251`). Mapless operation is
  `launch_localization:=false`, as `autosdv.launch.yaml:175` shows.

The sensing side survives `launch_localization:=false`, which is what the lab
depends on: `autoware_imu_corrector` publishes `/sensing/imu/imu_data` for both
IMU sources (`imu.launch.xml:55-59`), and that is the topic the lab should use —
**not** `wheel_imu_odom`'s default `/sensing/camera/zedxm/imu/data`, and
certainly not the `/sensing/imu/tamagawa/imu_raw` that
`mcl_localization.launch.xml:34` passes, which this sensor kit does not publish
at all.

### 3.2 `imu_yaw_sign` must be measured, not copied

`wheel_imu_odom`'s `imu_yaw_sign` parameter multiplies `angular_velocity.z`
before integration, and `mcl_localization.launch.xml:36` defaults it to `-1.0`
for a sample-sensor-kit Tamagawa IMU. That value is a property of that IMU's
sign convention, not of this vehicle. AutoSDV's two IMU sources do not even
agree on a frame: the MPU9250 driver hard-codes `header.frame_id = "base_link"`
(`src/sensor_component/external/ros2_mpu9250_driver/src/mpu9250driver.cpp:48`)
while the kit URDF defines a separate `imu_link`
(`sensor_kit.xacro:112-115`), and the ZED publishes in `zedxm_imu_link`.

Determine the sign on a bag by turning the vehicle one way and checking that the
integrated yaw moves the same way. Getting it backwards produces an odometry
that steers into its own error, which on a following vehicle is not a subtle
failure.

### 3.3 The one apparent feedback path is benign

`gyro_bias_estimator` is launched on the sensing side and subscribes to
`/localization/kinematic_state` (`imu.launch.xml:62-66`). In the lab that topic
is derived from the IMU, which looks like a loop: a gyro bias estimated from the
gyro's own integrated output. It is not, for three reasons:

- It uses the odometry as a **straight-motion gate**, not as an attitude
  reference. `libgyro_bias_estimator.so` exports `callback_odom(Odometry)` and
  `should_skip_update(double)`, and the governing parameter is
  `straight_motion_ang_vel_upper_limit: 0.015 # [rad/s]`
  (`autoware_imu_corrector/config/gyro_bias_estimator.param.yaml`).
- The part that *does* use a pose is gyro **scale** estimation
  (`estimate_scale_gyro`, `update_rate_ekf`, `compute_yaw_rate_from_quat`),
  driven by `~/input/pose_ndt` — a topic the lab never publishes, so that path
  stays dormant.
- Its outputs are `~/output/gyro_bias` and `~/output/imu_scaled`, and
  `imu.launch.xml:62-66` remaps neither into `/sensing/imu/imu_data`. A wrong
  estimate cannot reach the corrected IMU stream that the odometry consumes.

So the estimator can be left running. Worst case its bias estimate becomes
self-referential and nothing downstream reads it.

### 3.4 What the control chain actually requires of the odometry

Autoware 1.5.0 ships no source and no debug symbols, so the following comes
from the installed launch files and param yaml where possible, and from symbol
and string inspection of the shared objects where not. Items resting on the
latter are marked.

- **Only `header.stamp` and `twist.twist.linear.x` are needed.**
  `vehicle_cmd_gate`'s filter API is entirely scalar — `interpolateFromSpeed`,
  `limitLongitudinalWithVel/WithAcc/WithJerk`, `limitLateralWithLatAcc` — and
  nothing in `libvehicle_cmd_gate_node.so` references the odometry pose or
  either covariance block *(binary inspection)*. So `wheel_imu_odom`'s missing
  covariance is not a problem for the gate, and the pose matters only to
  consumers the lab does not run.
- **The topic name is not remappable.** The literal `input/kinematics` does not
  appear in the gate binary; `/localization/kinematic_state` does, referenced
  from the gate's constructor *(binary inspection)*. The remap at
  `control.launch.xml:111` is a no-op. Publish that exact name — which is what
  the component above does.
- **The gate withholds all output until each input has arrived once**
  (`isDataReady`, and the log string `waiting topics...`), at
  `update_rate: 10.0` (`vehicle_cmd_gate.param.yaml:3`). A missing
  `/localization/acceleration` is therefore silent: the vehicle simply never
  moves.
- **The stamp must advance.** Stop detection runs through
  `VehicleStopChecker`, which buffers `TwistStamped` over
  `velocity_buffer_time_sec = 10.0` and answers
  `isVehicleStopped(stop_check_duration: 1.0)`
  (`motion_utils/vehicle/vehicle_state_checker.hpp:38-61`,
  `vehicle_cmd_gate.param.yaml:14`). `wheel_imu_odom` copies the
  `VelocityReport` stamp, so this holds as long as the vehicle interface's
  stamps are real.
- **QoS is depth 1, RELIABLE, VOLATILE** for both topics
  (`component_interface_specs/localization.hpp:44-59`). The node publishes
  depth 10 RELIABLE, which is compatible.
- **`twist2accel` needs nothing else.** `use_odom: true` and
  `accel_lowpass_gain: 0.9` are its entire configuration
  (`twist2accel.param.yaml:1-4`), so `/localization/acceleration` is a
  low-passed derivative of the odometry's own twist. The `in_twist` argument is
  wired in the component for completeness and is unused at that default.
- **Neither the gate nor the transition manager looks up TF** — no `tf2`
  linkage in either shared object *(binary inspection)*. But
  `component_state_monitor`'s `topics.yaml:183-195` watches `map → base_link`
  on `/tf` as a `type: autonomous` entry with `error_rate: 1.0` and
  `timeout: 1.0`, so a missing TF marks autonomous mode unavailable through the
  diagnostic graph. Hence `publish_tf` defaults to `true` in the component.

### 3.5 The engage path is the open risk

`autoware_operation_mode_transition_manager` is what turns the gate on, and two
of its conditions need checking before the first field run:

- `enable_engage_on_driving: false`
  (`operation_mode_transition_manager.param.yaml:8`) means the vehicle must be
  stationary at engage time, judged from the odometry twist. Fine. The exact
  epsilon is not in any installed file.
- Its availability check carries the log string `Engage unavailable:
  trajectory size must be > 2` *(binary inspection)*, and its stable check
  compares the odometry pose against `/planning/trajectory`
  (`stable_check.dist_threshold: 1.5`, `yaw_threshold: 0.262`, `:22-27`).
  **Tier 1 has no planner and publishes no trajectory.** Whether
  `check_engage_condition: false` (`:10`, the default) bypasses this could not
  be determined without source.

If it does not bypass, tier 1 has two ways out, and the choice should be made
by testing rather than by reading:

1. Publish a stub `/planning/trajectory` of two or three points straight ahead,
   purely to satisfy the manager. Harmless, because tier 1 does not run
   `trajectory_follower` — and it is the first step toward tier 2, which
   publishes a real one.
2. Skip the transition manager and drive the gate directly: `GateMode` AUTO on
   `/control/gate_mode_cmd` plus the engage service, with
   `/system/operation_mode/state` published by the lab. Fewer moving parts,
   but it fakes a system-level state, which is the kind of shortcut that
   teaches the wrong thing.

Prefer 1. Settle it on the bench, before anyone stands near the vehicle.

### 3.6 Why the drift is acceptable

The coach board is measured in `base_link` on every frame, and nothing in the
lab is referenced to a global frame. So the pose is free to walk away — and
students can watch it walk away in RViz, which is the bridge to the NDT lecture
that follows. The hole they can see is the hole NDT fills.

---

## 4. Constraints imposed by the LCTK detector

These are findings from reading the detector, not guesses. They should appear in
the student handout as constraints to respect, not as things to discover.

### 4.1 `bbox_free` mode cannot be used

`rust/board-cluster-detector/src/background.rs` opens with: *"a static-mounted
sensor sees the same room every frame, so voxels that are reliably occupied are
background and points landing in never-occupied voxels are foreground."* Ego
motion invalidates the model on the first metre travelled.

Use `detection_mode: "bbox"` with a crop box ahead of `base_link`. This suits
the lab anyway — the coach car is by definition in front — and it means the lab
needs a session-local crop-box config, which
`ros/lctk_launch/config/board/hollow_1000/velodyne_bbox.json5` documents as
mandatory for that mode.

### 4.2 Detector latency is 100–400 ms per frame

`LCTK/docs/superpowers/specs/2026-08-12-initial-board-pose-inplane-rotation.md:453`
records the quarter-turn hypothesis search as costing *"up to 4× ICP (~100 ms →
~400 ms per frame"*. LCTK's own `CLAUDE.md` adds that the rclrs executor queues
all messages internally regardless of QoS depth, so a slow node backlogs rather
than dropping.

At 1 m/s that is 0.1–0.4 m of measurement lag. Two things follow:

1. A tracker that extrapolates the last detection to control time is not
   optional. It is provided in tier 1 and becomes student work in tier 2.
2. Cruise control needs range and bearing. It does not need a 6-DoF pose with
   the board's in-plane rotation disambiguated against its cutout pattern. That
   is the expensive part and it is being paid for nothing.

So measure first, on a bag, and then pick a path (§8, item 2). If the latency is
too high, the escape is a thin locator that keeps the crop box, DBSCAN and the
planar/size gate and drops ICP entirely — about sixty lines of Python,
publishing a centroid.

### 4.3 The board's physical mounting is constrained

From `velodyne_bbox.json5`:

- `stance_floor: 0.9` — the gate measures the normalized dot product of the
  board diagonal with scene up, where ~0.71 is flat-edge and ~1.0 is
  corner-standing. The board must be hung **corner-up, as a diamond.**
- `initial_inplane_rotation_deg: 0.0` is correct only for a diamond hang; the
  config comment is explicit that this is not a tuning dial.
- `isolation: true`, `isolation_max_density: 0.3`, band 0.05–0.30 m — the gate
  rejects a board embedded in coplanar clutter. **A board bolted flush to the
  coach car's flat back panel will be rejected.** Mount the plate on a slim pole
  or frame so its perimeter is clear of the vehicle body.

### 4.4 Use the 1000 mm plate, and keep the headway under 8 m

The `square_geometric_residual_max` comment in
`rust/board-cluster-detector/src/config.rs` records the measurement (their
issue H-17): a 600 mm plate at 7–8 m is crossed by about four VLP-32C rings, so
roughly half its perimeter bins can never hold a point and the coverage residual
gate becomes unreachable.

Use `hollow_1000_aruco_4_v1`, cap the working headway at about 8 m, and expect
to raise `cluster_eps` above its 0.15 m default for the far end of that range —
the config comment flags exactly this case.

### 4.5 There is no steering feedback

`/vehicle/status/steering_status` republishes the steering *command*; the vehicle
has no steering angle sensor (see `docs/reports/steering-status-has-no-feedback.md`
and the Known Issues in `CLAUDE.md`). Any lateral controller is therefore closed
on its own output.

Keep the lab's lateral control simple — proportional on measured bearing — and
say why. It is also the reason tier 2's MPC will underperform (§6).

---

## 5. The coach state message

The tracker publishes what the outer loop actually consumes:

- `range` — longitudinal distance, `base_link` x, metres
- `range_rate` — closing speed, m/s, from the track
- `bearing` — lateral angle to the board, rad
- `stamp` — the time the state is valid for, after extrapolation
- `valid` — false when the track has aged out

Whether this is a small custom message or a `Float32MultiArrayStamped` (already
used by the actuator's debug output) is an implementation detail; a typed message
is better for students reading `ros2 topic echo`.

---

## 6. Three tiers

| Tier | Student writes | Autoware used | Runs on |
|---|---|---|---|
| 0 — bench | Measurement extraction and the PID math, offline | none | rosbag + plots |
| 1 — graded | `velocity_report`, `actuator`, `cruise_controller` emitting `Control` | cmd_gate, operation mode, pseudo-odometry | vehicle |
| 2 — stretch | Emit `autoware_planning_msgs/Trajectory` in the odom frame instead of a `Control` | + `trajectory_follower` (MPC lateral, PID longitudinal) | vehicle |

Students write Python and C++. Every node in the student column above is
available in both: `rclpy` for the quick path and `rclcpp` for a group that
wants the control loop at a rate Python cannot hold. The provided nodes are a
mix of Python and one Rust detector, and none of them are student-editable.

Tier 0 exists so nobody's first contact with their own PID is on a moving
vehicle. It needs only the bags from §8, item 1.

Tier 1 is the deliverable.

Tier 2 is the fullest answer to "use the Autoware stack": the student becomes
the planner and Autoware does the control. Offer it as a bonus rather than the
main path — with no steering feedback (§4.5) the MPC will disappoint, and that
disappointment is a lesson about what a controller can and cannot do with the
sensors it is given.

---

## 7. What ships as a skeleton

Students fill holes; they do not start from an empty file. Each student package
ships with its node structure, its ROS wiring, its parameter loading and its
tests present and working, and with the algorithm bodies removed.

| File | Ships | Blanked |
|---|---|---|
| `velocity_report.py` | GPIO setup, edge timing, message assembly, publication | the edge-to-edge velocity computation and its filter |
| `actuator.py` | subscriptions, PCA9685 init, mode state machine, debug publishers, all of `actuator.yaml` | the PID body, the direction mapping, the PWM clamp |
| `cruise_controller` | node, subscriptions, `Control` assembly, publication timing | the headway law and the bearing law, entirely |

The mode state machine in `actuator.py` is worth shipping rather than blanking:
emergency brake, full stop, deadband hold and active control are a safety
structure, and a student reinventing it will reinvent it wrong. The PID inside
active control is the teaching content.

`actuator.yaml` ships complete, with the calibrated PWM values and the speed
gains **as a starting point that works**, so that a student whose PID is not yet
written can still drive the vehicle, and so that a student whose tuning goes
wrong can get back to a known-good state. The exception is `max_pwm`, capped
below the calibrated value (§9).

**The reference implementation is what is being withheld, and it already exists
in git history.** `git log` on
`src/vehicle/autosdv_vehicle_launch/autosdv_vehicle_interface/` hands a student
the answer. So the skeletons must be published from a repository with no
history of the reference — a fresh course repository, not a branch of this one —
and the grading harness (§8, item 8) keeps the reference on the instructor
side, where it doubles as the baseline that student scores are compared
against.

---

## 8. Work items to build the lab

1. **Record bags.** 1000 mm hollow board hung diamond on a pole, static and then
   walked, across 2–8 m, on the course vehicle's LiDAR. Needed for tier 0, for
   the grading harness, and for item 2.
2. **Measure the detector.** Publication rate and per-frame latency on those
   bags. Decide between the LCTK detector plus tracker and the thin locator
   (§4.2). Nothing else in this design depends on the answer, but the scope of
   item 9 does.
3. **Point `wheel_imu_odom` at the control stack** (§3). ***Done, untested.***
   `frame_id` is now a parameter on the node, and
   `autosdv_launch/launch/components/autosdv_dead_reckoning_component.launch.xml`
   runs it alongside a standalone `twist2accel`. Not yet run against hardware
   or a bag — the workspace it was written in is unbuilt. What remains:
   measure `imu_yaw_sign` (§3.2) and settle the engage path (§3.5).
4. **Crop-box config and launch file.** A `coach_cruise.launch.yaml` in
   `autosdv_launch` bringing up sensing, the detector, the pseudo-odometry
   chain, `vehicle_cmd_gate`, and the vehicle interface — with no map, no
   localization and no perception module.
5. **`coach_target_tracker`**, provided (§5).
6. **The safety clamp**, between student output and the gate input: speed cap,
   minimum standoff, detection-loss timeout. Provided, and not student-editable.
7. **The three skeletons** (§7), in a course repository with no history of the
   reference implementation.
8. **Grading harness.** Replay a bag against the student's controller; score
   headway RMS, overshoot, minimum gap, and behaviour on detection loss.
   Compares against the withheld reference as the baseline.
9. **Extract `board-target-detection`** and submodule it into LCTK and AutoSDV;
   add the `colcon-cargo-ros2` step to `setup/autosdv_setup/registry.py`. Not on
   the critical path — see Sequencing below.
10. **The handout**, carrying §4 as stated constraints and §9 verbatim.

### Code sharing: one repository, two superprojects

The detector is shared between LCTK (which uses it to calibrate) and AutoSDV
(which uses it to find a coach car). Rather than vendor a copy into each, the
shared part is extracted into its own repository under `NEWSLabNTU` and
submoduled into both. `colcon-cargo-ros2` is added to `setup/` as a step in
`setup/autosdv_setup/registry.py`, so AutoSDV can build Rust.

Proposed repository: **`NEWSLabNTU/board-target-detection`**. The name is
deliberately not "calibration" — AutoSDV's use of it has nothing to do with
calibration, and a repository named for one consumer's use case ages badly.

What moves into it, from reading the dependency graph:

```
  board-target-detection/
  ├── rust/
  │   ├── calibration-target/            target definition loader
  │   ├── board-cluster-detector/        the detection algorithm
  │   └── calibration-target-detector/   the glue between the two
  ├── ros/
  │   ├── board_target_msgs/             CalibrationTargetIdentity.msg
  │   └── lidar_board_detector/          the rclrs node
  └── config/
      ├── targets/                       hollow_1000_aruco_4_v1.json5, solid_600_…
      └── board/                         per-sensor detector tuning presets
```

The crate names stay as they are; renaming them is churn with no payoff.

**What stays in LCTK.** `lctk_interfaces` keeps its thirteen solver `.srv`
files and loses only `CalibrationTargetIdentity.msg`, which seven LCTK packages
consume (the two solvers, `filter_box_tuner`, `lctk_quality`, both aruco nodes,
`interactive_solver_controller`). Those packages then depend on
`board_target_msgs` as well — the message is genuinely shared, so this is the
honest split rather than a convenience.

**The extraction fixes a real defect, not just the packaging.**
`rust/board-cluster-detector/Cargo.toml` carries this note today:

> as a root member it now shares the ROS-poisoned root resolve
> (aruco-detector -> sensor_msgs = "*", yanked), so plain `cargo test` here no
> longer works — build/test only via colcon

A ROS-free detection crate that cannot be tested without colcon is a testing
tax paid by everyone. In its own workspace, with no ROS crate in the resolve,
`cargo test` works again. So the new repository must be an **independent Cargo
workspace**, listed in LCTK's root `exclude`, with LCTK path-depending across
the boundary. Both consumers get a crate they can unit-test in seconds.

**System dependencies travel with it.** `lidar_board_detector` pulls
`petal-decomposition` with the `openblas-system` feature and `pcd-rs`, so
OpenBLAS is required. That belongs in the new repository's own setup, and in
the AutoSDV registry step that installs it.

**Pinning.** `.gitmodules` in both superprojects gets a `branch = main` line.
It is not a fork carrying a patch series, but it is a repository we develop, and
per the submodule conventions in `CLAUDE.md` the branch line is what tells a
reader which branch to commit to. Cut a tag per release and let each
superproject move its pin independently — LCTK and AutoSDV drifting to
different commits is expected and fine.

The lockstep rule applies with a new innermost level: a change reaching into
the detector is pushed in the shared repository first, then pinned in LCTK and
in AutoSDV separately.

**One thing to watch.** The target definition and the detector tuning preset
are now shared artifacts. If the same physical board is used for calibration
and for the lab, both superprojects must be pinned to a commit where that
board's definition agrees. A board redefined for the lab and not re-pinned in
LCTK silently changes what calibration is solving against.

### Sequencing

The extraction does not block the lab. Bring the lab up first against LCTK's
`install/` overlaid on AutoSDV's — which costs nothing but a `source` line —
and extract once the lab's detector path is settled by the §4.2 measurement. If
that measurement sends the lab to the thin locator, the extraction shrinks to
the target-definition crates and the configs, and `lidar_board_detector` stays
in LCTK.

---

## 9. Field safety

This section belongs in the student handout verbatim.

- A safety driver holds a physical kill switch that cuts power to the ESC,
  independent of ROS, at all times. The gate's emergency stop is the second line
  of defence, never the first.
- Cap `max_pwm` in the students' `actuator.yaml` below the calibrated value, so
  that no tuning mistake can command full throttle. A cap at roughly 2 m/s
  equivalent is appropriate for the course.
- The minimum standoff distance and the stop-on-detection-loss timeout live in
  the provided clamp layer, downstream of the student controller. A student gain
  error must not be able to close the gap.
- The coach car moves under human control only, and never reverses toward the
  ego vehicle.
- The first field run uses a static board with the ego vehicle approaching it.
  The board only starts moving after a clean stop has been demonstrated.

---

## 10. Decisions taken

Recorded here because both were open while this document was first written, and
both shaped it.

**Students fill skeletons, and that includes the vehicle interface.**
`velocity_report.py` and `actuator.py` are student work, not provided
infrastructure — so the inner loop is genuinely theirs and the lab's nominal
subject is real. §7 defines what is removed and what is kept, and notes the
consequence that the skeletons cannot be published from this repository, whose
history contains the answers.

**The shared code is extracted into its own repository rather than vendored
twice.** `colcon-cargo-ros2` goes into `setup/`, so AutoSDV can build the Rust
detector directly, and the detector, the target-definition crates and the
`CalibrationTargetIdentity` message move to `NEWSLabNTU/board-target-detection`,
submoduled into both LCTK and AutoSDV. The alternative — overlaying a prebuilt
LCTK workspace — remains the right way to bring the lab up before the extraction
lands.
