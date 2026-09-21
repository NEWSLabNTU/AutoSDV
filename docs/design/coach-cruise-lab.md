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
3. **`dead_reckon_odometry`** (§3). Independent of everything above — the
   cleanest thing to build while the bags are being recorded.
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
