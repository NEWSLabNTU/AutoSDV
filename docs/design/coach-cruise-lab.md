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
map and no NDT, those topics have to come from somewhere else. Dead reckoning is
enough, and every piece but one already exists:

```
  velocity_report
       │
       ▼
  autoware_vehicle_velocity_converter
       │  /sensing/vehicle_velocity_converter/twist_with_covariance
       ▼
  autoware_gyro_odometer              ◄── IMU
       │  /localization/twist_estimator/twist_with_covariance
       ├─────────────────────────────────────────┐
       ▼                                         ▼
  dead_reckon_odometry   NEW           autoware_twist2accel
       │  /localization/kinematic_state          │  /localization/acceleration
       │  + TF map → base_link                   │
       ▼                                         ▼
                    vehicle_cmd_gate, operation mode
```

`dead_reckon_odometry` is the only new node: integrate the fused twist into a
pose, publish `nav_msgs/Odometry` on `/localization/kinematic_state` with
`frame_id: map`, `child_frame_id: base_link`, and broadcast the matching TF.
Roughly eighty lines.

**The drift does not matter, and that is the point.** The coach board is measured
in `base_link` on every frame; nothing in the lab is referenced to a global
frame. So the pose is free to walk away — and students can watch it walk away in
RViz, which is the bridge to the NDT lecture that follows. The hole they can see
is the hole NDT fills.

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

So measure first, on a bag, and then pick a path (§7, item 2). If the latency is
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
vehicle. It needs only the bags from §7.

Tier 1 is the deliverable.

Tier 2 is the fullest answer to "use the Autoware stack": the student becomes
the planner and Autoware does the control. Offer it as a bonus rather than the
main path — with no steering feedback (§4.5) the MPC will disappoint, and that
disappointment is a lesson about what a controller can and cannot do with the
sensors it is given.

---

## 7. Work items to build the lab

1. **Record bags.** 1000 mm hollow board hung diamond on a pole, static and then
   walked, across 2–8 m, on the course vehicle's LiDAR. Needed for tier 0, for
   the grading harness, and for item 2.
2. **Measure the detector.** Publication rate and per-frame latency on those
   bags. Decide between the LCTK detector plus tracker and the thin locator
   (§4.2). Everything else is unaffected by the choice.
3. **Crop-box config and launch file.** A `coach_cruise.launch.yaml` in
   `autosdv_launch` bringing up sensing, the detector, the pseudo-odometry
   chain, `vehicle_cmd_gate`, and the vehicle interface — with no map, no
   localization and no perception module.
4. **`dead_reckon_odometry`** (§3).
5. **`coach_target_tracker`** (provided) and a `cruise_controller` skeleton with
   both loops blanked.
6. **The safety clamp**, between student output and the gate input: speed cap,
   minimum standoff, detection-loss timeout. Provided, and not student-editable.
7. **Grading harness.** Replay a bag against the student's controller; score
   headway RMS, overshoot, minimum gap, and behaviour on detection loss.
8. **The handout**, carrying §4 as stated constraints.

### Workspace integration

Two viable paths. Students write Python and C++ only, so in either case the
Rust detector is provided infrastructure that they never build, edit or read.

**Path A — overlay a prebuilt LCTK (recommended for the first run of the
course).** Build LCTK once on the lab machine, then:

```bash
source /opt/autoware/1.5.0/setup.bash
source ~/repos/LCTK/install/setup.bash
source ~/repos/AutoSDV/install/setup.bash
```

Nothing is added to the AutoSDV build, and a student's build time stays bounded
by their own two packages. The cost is a second workspace to keep in sync and a
sourcing order that has to be right.

**Path B — vendor the detector into AutoSDV.** `colcon-cargo-ros2` can be added
to `setup/` as a step in `setup/autosdv_setup/registry.py`, so this is
available. It buys a single workspace and a single `just build`, at the cost of
four known sharp edges inherited from LCTK — all of them documented in that
repo, none of them hypothetical:

- Binding generation runs once per `build/` tree, guarded by
  `build/.colcon/bindgen.lock`; deleting parts of `build/` breaks it.
- A bare `cargo update` re-resolves the wildcard ROS message crates against
  crates.io and aborts on the yanked `sensor_msgs`. Dependency updates have to
  run inside the sourced build environment.
- The workspace-root `.cargo/config.toml` needs synthesising from the
  per-package configs on older `colcon-cargo-ros2`; LCTK carries
  `setup/scripts/sync-root-cargo-config.sh` for exactly this.
- LCTK's own build ignores the `conflux` submodule packages because their git
  `rclrs` conflicts with the crates.io one. Only `lidar_board_detector` and the
  `board-cluster-detector` crate are needed here, so that conflict is avoidable
  — but it constrains what can be vendored.

Recommendation: run the first iteration of the course on path A, and move to
path B only if maintaining two workspaces turns out to be the bigger tax. The
decision does not affect anything else in this design — the detector's interface
is the same either way.

If §4.2's measurement sends the lab to the thin locator instead, this whole
question disappears: that node is Python in `autosdv_launch`, and LCTK is then
needed only to produce the board target definition.

---

## 8. Field safety

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

## 9. Open question

Does the graded lab include `actuator.py` and `velocity_report.py` as student
work — shipping skeletons and withholding the working reference — or only the
cruise layer on top of the interface as it stands? Both nodes exist and work
today (`src/vehicle/autosdv_vehicle_launch/autosdv_vehicle_interface/`), so this
is a decision about what to take away, and it changes work items 5 through 7
substantially.
