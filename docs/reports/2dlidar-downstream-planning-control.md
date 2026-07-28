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

| step | before the fix | after the fix |
|---|---|---|
| `/api/localization/initialize` | ok | ok |
| localization reaches INITIALIZED | ok | ok |
| `kinematic_state` flowing | ok | ok |
| `/api/routing/set_route_points` | ok | ok |
| route reaches SET | ok | ok |
| trajectory published | **FAIL** (0) | **FAIL** (0) |
| autonomous mode available | **FAIL** | **FAIL** |

**Routing succeeds on an MCL pose.** That is a real result and not a small one:
accepting a route requires the pose to resolve onto a lanelet, so the 2-D MCL
estimate is good enough for the map-matching that mission planning performs.

**Autonomous mode is still unavailable**, but for a different and much shorter
list of reasons than before — and the ones that remain are not `mcl`'s: the NDT
control run fails at exactly the same step (§5), traced to harness
configuration rather than to either pose source (§5.2-§5.4).

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

## 5. The NDT control run: the trajectory gap is not MCL's

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
