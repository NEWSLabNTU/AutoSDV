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
control run fails at exactly the same step (§5), traced to the sample map
lacking lanelet2 version metadata (§5.2).

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
- **Map data, not `mcl`.** No trajectory is ever published, because
  `route_handler` rejects the sample lanelet2 map as an "invalid version map"
  and `scenario_selector` then waits on a route forever. The NDT control run
  fails identically, which settles the attribution — see §5.2. Everything in
  the planning and control branches follows from this one fault.
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

### 5.2 The shared cause of the missing trajectory

`route_handler` warns `setMap() for invalid version map:` — with an empty
version string — and `scenario_selector` then waits on a route forever. The
route exists at the mission-planner level, which is why ADAPI reports SET and
the first five steps pass, but it never reaches the scenario selector.

The sample map has no lanelet2 version metadata:

| map | `format_version` / `map_version` |
|---|---|
| `sample-rosbag-replay/sample-map-rosbag/lanelet2_map.osm` | **absent** |
| `COSS-map-planning/lanelet2_map.osm` | `format_version="1"`, `map_version="2"` |

So this is map data, not localization and not AutoSDV. It also explains the
remaining diagnostic failures as one fault rather than five: no route to the
selector means no trajectory, hence no `trajectory_follower` and no
`control_command`, hence those branches error too.

The prediction this makes, untested: the same probe on the COSS map — which
carries the version metadata — should produce a trajectory.

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
