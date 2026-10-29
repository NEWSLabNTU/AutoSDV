# play_launch health issues: COSS logging simulation

What play_launch 0.20.2's health page reports for the COSS replay
(`just coss logging-sim`, CPU backend: `pose_source:=ndt launch_perception:=false`,
no RViz) on the workstation `newslab-server133`, on 2026-10-29, and which of the
issues are ours to fix.

Snapshot at the end of one 157 s replay: **1 error, 3 warnings, 15 info**, plus a
fourth warning that only shows while the bag plays. Two items were in this
repository and are fixed; every other item is the host's
configuration, an expected consequence of how the run was set up, or Autoware's
own configuration.

## Reading the issues programmatically

The web UI is a client of a JSON API on the same port (default 8081). The health
page is `/api/health/issues`:

```bash
curl -s localhost:8081/api/health/issues | jq -r '.open[] | "\(.severity)\t\(.kind)\t\(.title)"'
curl -s localhost:8081/api/health/issues | jq '.open[] | select(.severity=="error")'
curl -sN localhost:8081/api/health/issues/stream    # server-sent events, one snapshot per line
```

Each open issue has `key`, `kind`, `severity` (`error`, `warning`, `info`),
`subject`, `title`, `detail`, and `opened_ms`/`last_seen_ms`/`closed_ms`.

| Endpoint | Returns |
|---|---|
| `/api/health` | process, node, container and composable-node counts: running, stopped, failed |
| `/api/health/issues`, `/api/health/issues/stream` | the open issues |
| `/api/health/dds_report` | plain text: RMW, kernel socket buffers, UDP drops, participants, `/dev/shm` |
| `/api/diagnostics/counts`, `/list`, `/history`, `/stream` | `/diagnostics` statuses |
| `/api/nodes`, `/api/nodes/<name>/logs/stderr` | nodes and their logs |
| `/api/metrics/system`, `/api/metrics/node/<name>` | CPU and memory |

A run's log directory (`play_log/<run>/`) also keeps `health.jsonl`,
`dds_report.txt` and `diagnostics.csv`, so the same information is available
after the stack has stopped.

## Before the issues: the stack did not start

On the first launch 37 of 52 processes exited at once with

```
rmw_create_node: failed to create domain, error Error
failed to increase socket receive buffer size to at least 10485760 bytes, current is 2097152 bytes
```

`cyclonedds.xml` asks for `<SocketReceiveBufferSize min="10MB"/>`, and Cyclone
DDS treats `min` as a requirement: on a host whose `net.core.rmem_max` is lower
(here 1 MiB) it refuses to create the domain rather than run with the smaller
buffer. This host had not been configured for DDS (`setup.sh` prints the same
warning: "Network not configured for optimal DDS performance").

The fix is host configuration and needs root:

```bash
sudo sysctl -w net.core.rmem_max=2147483647
```

For this run, without root, the stack was started with a copy of the
configuration that drops the minimum (`min="default"`), passed through
`CYCLONEDDS_URI`. The repository's `cyclonedds.xml` was not changed.

## The issues

| Severity | Kind | Subject | Cause | Action |
|---|---|---|---|---|
| error | `sim_time_no_clock` | `/clock` | the replay had ended; nothing publishes `/clock`, so 95 sim-time nodes stand at zero and all 132 `/diagnostics` entries go stale | expected; play the bag again |
| warning | `dds_udp_drops` | domain 0 | about 63 datagrams a minute dropped on 120 DDS sockets, from the 1 MiB `rmem_max` | host configuration (above) |
| warning | `foreign_node` | `/managed_tf_listener_impl_…` | the domain had 147 nodes against this run's 37: other processes on the host use `ROS_DOMAIN_ID` 0 | host configuration; a separate `ROS_DOMAIN_ID` isolates a run |
| warning | `near_miss_topic` | `/perception/object_recognition/objects` | ten planning and control nodes subscribe; nothing publishes it because the CPU backend runs with `launch_perception:=false` | expected by design |
| info | `shm_zombies` | host | 121 Fast DDS shared-memory files (44 MiB) left in `/dev/shm` by processes that died; this run uses Cyclone DDS | host housekeeping (`fastdds shm clean`) |
| info | `param_ignored` ×13 | see below | parameters in `autoware_launch`'s configuration files that the 1.5.0 nodes never declare | Autoware's, not ours |
| info | `param_ignored` | `/sensing/lidar/passthrough_as_concatenate` | `remove_nan`, `float_min`, `float_max` set by our sensor kit | **fixed** |
| warning | `log_flood` | `autoware_ekf_localizer_node-1` | 955 lines/s (2.8 MiB/min) while the bag plays: `show_debug_info: true` in our EKF config | **fixed** |

The `log_flood` warning is absent from the snapshot above because it was taken
after the replay ended, when the EKF had nothing to log; it opens as soon as
data flows.

### `param_ignored` in Autoware's configuration

These nodes are loaded with the stock `autoware_launch` files from
`/opt/autoware/1.5.0/share/autoware_launch/config/` (our control and planning
components pass those paths deliberately; only the PID file is our own copy).
The parameters named are present in those files and are not declared by the
nodes, either because the release's config and code disagree or because the
module that declares them is not enabled:

| Node | Ignored |
|---|---|
| `autoware_operation_mode_transition_manager` | `ego_nearest_dist_threshold`, `ego_nearest_yaw_threshold` |
| `trajectory_follower/controller_node_exe` | `acceleration_limit`, `velocity_time_constant` (MPC file) |
| `lane_departure_checker_node` | `delta_yaw_threshold_for_closest_point`, `max_lateral_deviation`, `max_longitudinal_deviation`, `max_yaw_deviation_deg` |
| `map/pointcloud_map_loader` | `leaf_size` (declared only with `enable_downsampled_whole_load`) |
| `manual_lane_change_handler` | `map_frame` |
| `planning_validator` | `latency_checker.is_critical`, `planning_hz` |
| `external_velocity_limit_selector` | the velocity smoother's parameters (same file) |
| `behavior_path_planner` | `avoidance_by_lane_change.target_filtering.*` and others (109) |
| `behavior_velocity_planner` | `merge_from_private.*`, `no_drivable_lane.*`, `occlusion_spot.*` and others (65) |
| `motion_velocity_planner` | `obstacle_cruise.cruise_planning.optimization_based_planner.*` and others (19) |
| `path_optimizer` | `option.debug.enable_calculation_time_info` |
| `freespace_planner` | `rrtstar.*` |
| `component_state_monitor` | `topic_monitor_names` |

The installed Autoware is official Autoware (see `CLAUDE.md`), so these are
upstream findings; editing a copy here would hide them rather than fix them. Our
`config/map/pointcloud_map_loader.param.yaml` is a verbatim copy of Autoware's
and carries `leaf_size` for the same reason.

### The two that were ours

**EKF debug trace.** `config/localization/ekf_localizer.param.yaml` had
`show_debug_info: true`, left from fusion tuning ("Enable debug output for fusion
tuning"). At 50 Hz prediction the filter logs a banner for every timer call,
prediction, pose and twist update: 18,415 stderr and 16,920 stdout lines in under
two minutes of replay, 7.5 MB, against 25 KB for the next-loudest node. It ran on
every launch, the car included, and nothing in the repository parses it. Now
`false` (Autoware's default), with a note on when to turn it on.

**Passthrough settings.** `autosdv_sensor_kit_launch/launch/pointcloud_preprocessor.launch.py` loads
`PassThroughFilterComponent` as `passthrough_as_concatenate`, the stand-in for
the concatenator on a one-LiDAR kit, and gave it `remove_nan: true`,
`float_min: -999.0` and `float_max: 999.0`. The component declares none of them:
it copies the cloud unchanged, and the filter base supplies only the transform
to `output_frame`. The three values never took effect, so NaN points were never
removed by this node either. They are deleted, with a comment saying why; the
node's behaviour is unchanged.

### After the fixes

The same launch and replay, snapshot taken 60 s into playback: **0 errors,
3 warnings, 14 info**. The warnings are `dds_udp_drops`, `foreign_node` and
`near_miss_topic`, all explained above; the info items are the 13 Autoware
`param_ignored` and `shm_zombies`. The EKF wrote 1.2 KB of stderr, down from
4.7 MB.
