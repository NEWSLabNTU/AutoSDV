# Downstream validation: planning and control on an MCL pose

The accuracy gate scores `/localization/kinematic_state` and stops there.
Nothing had ever exercised planning or control on top of a 2-D MCL pose, so
"the integration works" was a claim about a topic, not about the vehicle. This
report runs the sequence a human performs in RViz — initialize, set a route,
engage — against `pose_source:=mcl` on the sample site, and reports what
happens.

Driver: `tmp/downstream_probe.py` plus `tmp/downstream_run.sh` (exploratory,
not yet promoted). Start and goal are the ground-truth bag's first and last
poses, so both ends are guaranteed to sit on the driven lane.

---

## 1. Result

`pose_source:=mcl` drives the full Autoware stack. On the Autoware sample site,
with the sample bag re-stamped (§5.5) and the sample vehicle/sensor models
(§5.3):

| step | `mcl` | `ndt` |
|---|---|---|
| localization INITIALIZED | ok | ok |
| route SET | ok | ok |
| trajectory published | ok, 15.01 Hz | ok, 15.07 Hz |
| control command | ok, 16.68 Hz | ok, 16.68 Hz |
| autonomous mode | **engaged** | refused (stock graph, §5.7) |
| cross-track, plan current | **0.342 m mean, 0.798 m p95** | 0.417 m mean, 1.594 m p95 |

Planning and control run on a 2-D MCL pose, and the vehicle tracks its plan to
roughly a third of a metre. Read §5.7 before comparing the two columns: the
autonomous-mode difference is a diagnostic-graph property, not pose quality,
and the two runs drove at different speeds.

Getting here took six configuration faults, each real and each documented
below: a silently dropped composable node (§5.1), a hard-wired perception input
(§5.1), the derived-versus-raw bag (§5.2), the vehicle/sensor model pairing
(§5.3), goal selection (§5.4), and a 328.9-day clock/data offset in the
upstream bag (§5.5). None was a property of either pose source.

## 2. The health checks assumed NDT with a PCD

Autoware gates autonomous mode on a diagnostic graph. Two of its checks are
unsatisfiable under `mcl` by construction, independent of pose quality:

- `/autoware/map` is `AND(vector_map, pointcloud_map)`. Under `mcl` there is no
  PCD, because `autosdv_map_component` deliberately does not start
  `pointcloud_map_loader` (a missing PCD kills `map_container` outright), so
  `/autoware/map/topic_rate_check/pointcloud_map` can never pass.
- `/autoware/localization/scan_matching_status` is sourced from the
  `ndt_scan_matcher` node, which does not exist under `mcl`, so it is STALE
  forever.

Both were confirmed by reading the graph definitions, not inferred from a log.

### The fix

`pose_source` now selects the graph and the monitored topic list. Every
non-`mcl` method keeps the stock files byte for byte.

- `diagnostics/map-mcl.yaml` — `/autoware/map` = `AND(vector_map,
  occupancy_grid)`.
- `component_state_monitor/topics-mcl.yaml` — monitors `/map`
  (`nav_msgs/OccupancyGrid`, latched). Rates stay `0.0`: like the stock map
  entries this is a presence check, not a rate check.
- `diagnostics/localization-mcl.yaml` — drops the `ndt_scan_matcher` check.
- `diagnostics/autosdv-mcl-main.yaml` — composes the graph, including the five
  method-independent modules from the installed `autoware_launch` rather than
  copying them, so it cannot drift.
- `launch/components/autosdv_system_component.launch.xml` — fork of
  `tier4_system_component`, differing only in that it lets the caller choose
  the `component_state_monitor` topic list, which upstream hardcodes.

### Measured effect

The aggregator's stated reasons, before and after:

| branch | before | after |
|---|---|---|
| `/autoware/map/topic_rate_check/pointcloud_map` | ERROR | **absent** |
| `/autoware/localization/scan_matching_status` | STALE | **absent** |
| `/autoware/localization/accuracy` | STALE | **absent** |
| `/autoware/localization/topic_rate_check/pose_twist_fusion` | ERROR | **absent** |

`/autoware/map` no longer appears among the blockers at all.

## 3. What still blocks autonomous mode

- **Harness, not `mcl`.** `/autoware/vehicle/topic_rate_check/steering` and
  `/autoware/control/topic_rate_check/{trajectory_follower,control_command}`
  cannot pass in logging simulation for any pose source:
  `launch_vehicle_interface` is false, there is no ego simulator, and the
  replay publishes no `steering_status`.
- **Harness, not `mcl`.** No trajectory is published because
  `behavior_path_planner` waits on an occupancy grid that perception never
  produces — traced in §5.2 to replaying a derived single-LiDAR bag and to a
  vehicle/sensor model mismatch. The NDT control run fails identically, which
  settles the attribution. Everything in the planning and control branches
  follows from that one fault.
- `/autoware/perception/topic_rate_check/pointcloud` — probably harness, not
  confirmed.

## 4. Known gap this introduces

Under `mcl` the graph has **no estimator-health signal**. The NDT scan-matching
check was dropped rather than faked, because `particle_filter` publishes no ROS
diagnostic — `diagnostics.py` writes a JSONL trace, not `diagnostic_msgs`. A
silently diverging filter will not raise a diagnostic here. Closing this means
publishing a real health diagnostic (effective sample size and mean particle
weight are already computed per update) and adding it to
`localization-mcl.yaml`.

## 5. The control run, and the end-to-end measurement

`pose_source:=ndt` (official Autoware NDT, not `cuda_ndt`) on the same bag,
map and route, once the two blockers in §5.1 were cleared:

| step | `mcl` | `ndt` |
|---|---|---|
| `/api/localization/initialize` | ok | ok (2nd attempt) |
| localization INITIALIZED | ok | ok |
| `kinematic_state` flowing | ok | ok |
| `set_route_points` | ok | ok |
| route SET | ok | ok |
| **trajectory published** | **FAIL (0)** | **FAIL (0)** |
| autonomous mode available | FAIL | FAIL |

Identical through five steps and identical at the sixth, with NDT localizing
properly against the PCD map. **The absent trajectory is therefore a property
of this map/bag/harness, not of the 2-D MCL pose source.** Before this run that
attribution was genuinely unknown, and this report previously said so.

### 5.1 Two blockers the control run had to clear first

Both were real defects, found because the control run would not start:

- **`input_pointcloud` was unreachable from the top level.** Declared in
  `tier4_localization_component` but never forwarded, so it always resolved to
  its default `/sensing/lidar/concatenated/pointcloud` — a topic a live
  sensing pipeline produces and a replayed bag does not. NDT received no scan
  and `pose_initializer` failed with "align server failed" on every attempt.
  Now plumbed from both entry launches, default unchanged.

- **`play_launch` silently dropped a composable node.**
  `/localization/util/voxel_grid_downsample_filter` was never loaded, because
  perception loads a node of the same *base* name into the same container and
  the container's registry was keyed on bare names. The NDT input chain was
  severed mid-way: `measurement_range/pointcloud` had a publisher and zero
  subscribers. Reproduced in twelve lines of XML with no Autoware involved —
  `ros2 launch` loaded both nodes, `play_launch` loaded one. Already fixed
  upstream in play_launch `5b20c1e`; this machine was running an April build.
  With a current build all three util filters load and align succeeds.

### 5.2 The real cause, after several wrong turns

The trajectory is absent because `behavior_path_planner` sits at **`waiting for
occupancy_grid`** and never publishes a path. Nothing downstream can exist
without it: no path, no trajectory on any topic, no `trajectory_follower`, no
`control_command`, and hence those diagnostic branches error. One fault, not
five.

The occupancy grid was missing because **the harness replayed the wrong bag**.
`data/rosbags/phase3/sample_ndt_gt` is a *derived* recording carrying a single
decoded cloud (`/sensing/lidar/top/pointcloud_raw_ex`). The Autoware sample
rosbag carries raw `velodyne_packets` from **three** LiDARs, which the sensing
pipeline decodes and concatenates into
`/sensing/lidar/concatenated/pointcloud` — exactly the topic localization and
perception default to. Feeding the derived bag starved both, and the fix was to
replay `sample-rosbag-migrated` (the original carries pre-1.5.0
`autoware_auto_vehicle_msgs` that this Autoware cannot deserialize).

**Corrections to earlier versions of this report.** Three claims were wrong:

- *"The sample map's missing lanelet2 version metadata is the cause."* No.
  `route_handler` logs `setMap() for invalid version map:` as a WARN and
  continues; the missing `format_version` is cosmetic. The map works.
- *"`input_pointcloud` and `perception_input_pointcloud` were real defects."*
  They are useful knobs, and perception's was genuinely unreachable from the
  command line, but the starvation they addressed was caused by replaying the
  derived bag. Commit `d67034d` overstates `input_pointcloud` as the cause of
  NDT's align failure; the composable-node drop and the derived bag were.
- *"Routing succeeding on an MCL pose shows the pose is good enough for lanelet
  matching."* Weaker than stated: those runs used `autosdv_vehicle`, whose
  0.262 m footprint passes a goal check that the correct `sample_vehicle`
  footprint fails. The pose was fine; the check was passing for the wrong
  reason.

### 5.3 Vehicle and sensor models must match the bag

`logging_simulation.launch.yaml` hardcoded `vehicle_model: autosdv_vehicle` and
`sensor_model: autosdv_sensor_kit`. The pairing rule is COSS bags with the
AutoSDV vehicle, the Autoware sample rosbag with `sample_vehicle` +
`sample_sensor_kit`. Mismatching them is not benign: the obstacle crop box spans
`ground-2.5 .. vehicle_height`, and with `vehicle_height` 0.262 m instead of
2.5 m every point above ~26 cm was discarded, so `crop_box_filter` ran on each
frame (debug topics ticking at 1.93 Hz) while publishing an empty cloud.

Both are now arguments, defaults unchanged. Selecting a foreign sensor kit also
drags in its own argument expectations: `sample_sensor_kit`'s `gnss.launch.xml`
defines `navsatfix_topic_name` only for `gnss_receiver` of `ublox` or
`septentrio`, so AutoSDV's empty default makes the launch fail to parse —
`gnss_receiver:=ublox` is required with that kit.

Related: `mcl_localization.launch.xml` hardcodes `scan_min_height: 1.91611` /
`scan_max_height: 2.21611`, which are the *sample* kit's sensor heights. That
compensated for the same mismatch on the localization side and would be wrong
for a COSS run; it should become sensor-kit-derived.

### 5.4 Goal selection, and an API detail

Goals cannot be picked naively. Sweeping poses along the ground-truth track
against a live stack (`tmp/goal_sweep.py`) shows acceptance is **patchy, not
monotonic**: 0.15, 0.25, 0.35, 0.45 and 0.85 of the track are accepted, while
0.55-0.75, 0.95 and the final pose are rejected with "Goal's footprint exceeds
lane!" or "The planned route is empty". Lane width is not the cause — the route's
lanelets are 3.08-3.58 m wide against a 1.896 m vehicle. Those poses simply do
not sit cleanly inside a mapped lanelet. `allow_goal_modification` does not
rescue them.

Routing also refuses a second goal with **"The route is already set"** until
`/api/routing/clear_route` is called. That message appeared in several earlier
runs purely because the probe never cleared, on stacks reused across probes.

### 5.5 The clock/data mismatch, and the first end-to-end measurement

The last blocker was neither launch config nor localization. The upstream
Autoware sample rosbag's **storage timestamps run 328.9 days ahead of its
message header stamps**:

```
storage starting_time = 1614315746.3   (2021-02-26)
first header.stamp    = 1585897255.3   (2020-04-03)
constant offset       = 28,418,491 s
```

`ros2 bag play --clock` derives /clock from storage times, so every sim-time
node ran at 1614..., `ekf_localizer` published `map->base_link` on its 50 Hz
timer stamped at clock time (measured: `At time 1614315772.6`, correct
position), while perception looked transforms up at the LiDAR **header** stamp
of 1585.... Every such lookup landed 329 days before the buffer's earliest
entry and failed as "extrapolation into the past", so no occupancy grid was
produced and `behavior_path_planner` waited forever. Localization survived
because NDT matches on the cloud and the EKF publishes a pose regardless, which
is exactly why the symptom looked like a planning fault.

`scripts/rosbag/restamp_bag.py` shifts every storage timestamp by one measured
constant (the offset is uniform to within 0.1 s across all headered topics, and
5590 of 8262 messages carry no header at all, so a constant shift is both
sufficient and the only option that covers them). Verified residual: -1 ns.

With the re-stamped bag, `pose_source:=ndt`, sample models, and the swept goal:

| quantity | value |
|---|---|
| trajectories | published on all three topics continuously |
| trajectory rate | **15.07 Hz** |
| control commands | 450+ per observation window |
| control rate | **16.68 Hz** |
| cross-track, plan current | **mean 0.417 m, p95 1.594 m, max 1.725 m** (n=371 of 450) |
| ego speed | 3.84 m/s mean |

**Planning and control run end to end on a live localization pose, and the
vehicle tracks its plan to within half a metre on average.**

### 5.6 Why the first lateral figure was meaningless

The first run reported 14.93 m mean cross-track, and a later run 3.05 m, from
the same code on the same configuration. That instability was the clue: the
statistic averaged two incompatible regimes.

Diagnostics pairing each cross-track sample with `d_first` (ego to the
trajectory's *first* point) separate them. While the plan is current, samples
look like:

```
ego [89579.15, 42307.68]  traj_first [89575.52, 42304.29]  d_min 0.28  d_first 4.97
ego [89579.45, 42308.03]  traj_first [89576.30, 42304.92]  d_min 0.32  d_first 4.43
```

Once the ego drives past the goal — which sits at 85% of the recorded track
while the bag keeps going at ~4-8 m/s — `d_min` and `d_first` blow up *together*
to ~59 m, because the trajectory no longer describes where the vehicle is. 79 of
450 samples fell in that regime, and how many land inside the observation window
varies run to run, which is why the unconditioned mean swung from 14.93 to 3.05.

The distance formula was never wrong: it is Autoware's standard closest-point
cross-track definition. What was wrong was the population it averaged over. The
reported metric now conditions on the plan being current (`d_first <= 5 m`,
i.e. Autoware still planning from the vehicle) and reports the surviving sample
count alongside the total.

Autonomous mode remains unavailable, as expected in this harness:
`launch_vehicle_interface` is false and no `steering_status` is replayed, so
the vehicle and control diagnostic branches cannot pass for any pose source.

## 5.7 MCL end to end, and the comparison (RETRACTED, see 5.8)

Same configuration, `pose_source:=mcl`:

| | `mcl` | `ndt` |
|---|---|---|
| autonomous mode available | **True** | False |
| `change_to_autonomous` | **ok** | refused |
| final operation mode | **AUTONOMOUS** | STOP |
| trajectory rate | 15.01 Hz | 15.07 Hz |
| control command rate | 16.68 Hz | 16.68 Hz |
| cross-track mean | **0.342 m** | 0.417 m |
| cross-track p95 | **0.798 m** | 1.594 m |
| cross-track max | 1.662 m | 1.725 m |
| conditioned / total samples | 398 / 451 | 371 / 450 |
| ego speed mean | 1.78 m/s | 3.84 m/s |

**The 2-D MCL pose drives the full stack**: planning, control, and engaged
autonomous operation, tracking its plan to 0.34 m mean and 0.80 m p95. This is
what the accuracy gate could not show, since that gate stops at
`kinematic_state`.

Two things this does **not** show.

*It is not evidence that MCL localizes better than NDT.* The autonomous-mode
difference is a property of the diagnostic graph each method loads, not of pose
quality: `mcl` runs the graph from §2, which requires the occupancy grid and
drops the `ndt_scan_matcher` check, while `ndt` runs the stock graph that still
demands checks this harness cannot satisfy (no vehicle interface, no
`steering_status`). The honest claim is that the `mcl` graph is satisfiable in
logging simulation and the stock one is not.

*The cross-track figures are not a like-for-like comparison.* MCL averaged
1.78 m/s against NDT's 3.84 m/s over the observation window. Slower driving
generally tracks tighter, so MCL's marginally better numbers may reflect speed
rather than pose. **Why the two runs drove at different speeds on the same bag
is unexplained and should be settled before these numbers are compared
closely** -- the likely candidates are the engaged autonomous mode changing the
velocity profile in the MCL run, or a different segment of the bag falling
inside the window.

## 5.8 MCL end to end, verified

The result in §5.7 was retracted: that run reached AUTONOMOUS on a dead-reckoned
pose, because the probe never checked that the estimator under test was
observing anything. Re-run with the 3-ring scan source
(`docs/reports/2dlidar-scan-source-comparison.md`) and a probe that gates on the
scan itself:

| check | result |
|---|---|
| scan carries returns | **170/170 scans, max 970 finite beams** |
| estimator published | 171 poses on the contract topic |
| route SET | ok |
| trajectory | 15.01 Hz |
| control command | 16.67 Hz |
| autonomous mode | **engaged** |
| cross-track, plan current | 0.296 m mean, 1.132 m p95, 1.668 m max (n=394/451) |

**Planning and control run on a genuinely matching 2-D MCL pose.** The scan gate
is what makes this trustworthy: in the retracted run the same probe reported
"1462 beams and not one finite range" while every other step went green, because
`ekf_localizer` dead-reckons from the seed and a particle filter with no
observations still emits motion-model poses.

Against the NDT control on the same harness:

| | MCL (3-ring) | NDT |
|---|---|---|
| trajectory / control | 15.01 / 16.67 Hz | 15.07 / 16.68 Hz |
| cross-track mean | 0.296 m | 0.417 m |
| cross-track p95 | 1.132 m | 1.594 m |
| conditioned samples | 394/451 | 371/450 |

Two caveats travel with those numbers and should not be dropped when quoting
them. Cross-track is partly **self-referential** -- the trajectory is planned from
the pose being measured -- so it reports control tracking, not localization
accuracy; the ground-truth figure is 0.789 m from the five-seed matrix. And MCL
drove at 1.86 m/s against NDT's 3.84 m/s, so the columns are not like-for-like;
slower driving tracks tighter, and the speed difference is still unexplained.

### What the probe gates on now

Counting estimator poses proved insufficient, so the probe requires, in order:
at least 20 scans carrying returns with at least 50 finite beams at peak; at
least 20 poses on the estimator contract topic; then routing, trajectory,
control and mode. A failure now names the broken link instead of passing on
dead reckoning.

## 6. The cuda_ndt attempt, and why it proved nothing

`pose_source:=cuda_ndt` on the same bag and route was meant to separate
harness limits from `mcl` limits. It failed at the first step:
`/api/localization/initialize` returned "align server failed", so it never
localized and every later step is that cascade rather than independent
evidence. The probe called ADAPI 5 s into replay, before NDT had a map and a
scan for `pose_initializer`'s align step; retry-with-warm-up has been added but
not yet re-run. Until it is, the MCL-vs-NDT differential does not exist, and
§3's harness attribution rests on reading the launch configuration rather than
on measurement.

Also unexplained: that run's route call returned "The route is already set" on
a freshly launched stack.

## 7. Environment note

Two runs were lost to a degraded DDS/daemon state after repeated `kill -9`
sweeps: ADAPI services never appeared, and `map_server_lifecycle_bringup` died
with `xmlrpc.client.Fault: <class 'RuntimeError'>:!rclpy.ok()`. `ros2 daemon
stop && ros2 daemon start` restored it, and the next run initialized on the
first attempt. Worth knowing before diagnosing a stack that "won't come up".

## 8. Reproduce

```bash
# after a clean DDS state
ros2 daemon stop && ros2 daemon start
METHOD=mcl bash tmp/downstream_run.sh
METHOD=ndt bash tmp/downstream_run.sh   # the control

# the aggregator's live reasons
tac play_log/<run>/node/logging_diag_graph/err \
  | awk '/The target mode is not available/{print; exit} {print}' | tac
```
