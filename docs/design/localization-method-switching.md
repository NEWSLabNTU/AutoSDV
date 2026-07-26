# Switching AutoSDV's Localization Method

How `pose_source` actually dispatches today, what contract a pose estimator must
satisfy, and what it would take to make AutoSDV 2D-MCL selectable the same way
NDT and CUDA NDT are. Written from the shipped launch files, not from the design
proposal in `modular_localization_launch.md` (which describes a different,
unimplemented scheme).

Related: `docs/research/localization/2d_mcl_algorithm.md`,
`docs/research/localization/mcl_initialization_and_covariance.md`,
`docs/reports/2dlidar-phase4-init-gating.md`.

---

## 1. There is no single dispatcher — there are two

`pose_source` is consumed in two unrelated places by two different mechanisms.

**Layer A — string equality in the top-level YAML.** `isaac` and `visual` are
handled by `<group if>` blocks in `autosdv.launch.yaml` (L93-131) and
`logging_simulation.launch.yaml` (L113-...). These groups *add* Isaac nodes as
siblings of the main include; they suppress nothing downstream.

**Layer B — a computed package name, for everything NDT-shaped.** The value is
forwarded down four files (`autosdv.launch.yaml` → `autosdv_autoware.launch.xml`
→ `components/tier4_localization_component.launch.xml` →
`tier4_localization_launch/launch/localization.launch.xml`) to the real selector,
`pose_twist_estimator.launch.xml`:

```xml
13  <let name="is_external_plugin" value="$(eval "'$(var pose_source_package)' != ''")"/>
16  <let name="available_args" value="['ndt','yabloc','eagleye','artag','lidar-marker']"/>
17  <let name="split_function" value="list(set('$(var pose_source)'.split('_')).intersection($(var available_args)))"/>
22  <let name="use_ndt_pose" value="$(eval "'ndt' in $(var pose_sources)")"/>
36  <group if="$(eval "$(var use_ndt_pose) and not $(var is_external_plugin)")">   <!-- built-in NDT -->
55  <group if="$(eval "$(var use_ndt_pose) and $(var is_external_plugin)")">       <!-- plugin -->
57    <include file="$(find-pkg-share $(var pose_source_package))/launch/pose_estimator.launch.xml">
```

Three consequences that are easy to miss:

- **`cuda_ndt` is not a distinct case.** `'cuda_ndt'.split('_')` gives
  `{'cuda','ndt'}`; intersected with `available_args` that is `{'ndt'}`. So
  `pose_source=cuda_ndt` and `pose_source=ndt` are *identical* as far as the
  selector is concerned. The CUDA-ness comes only from `pose_source_package`.
- **A plugin can only be substituted where NDT would have run.** The plugin
  group is guarded by `use_ndt_pose`, so `pose_source` must contain the token
  `ndt` to reach it at all. `pose_source:=mcl` yields an empty source set: no
  estimator, no `util.launch.xml` pointcloud downsampling (L161), and
  `ndt_enabled: false` for the pose initializer (L144).
- **This is an AutoSDV addition.** Diffing against
  `/opt/autoware/1.5.0/share/tier4_localization_launch/` shows exactly three new
  hunks (the `pose_source_package` arg, the `not is_external_plugin` guard, the
  plugin group). Upstream has no plugin mechanism;
  `autoware_pose_estimator_arbiter` exists but is a runtime relay for concurrent
  estimators, not a dispatcher.

## 2. Defect: `pose_source:=ndt` does not give you CPU NDT

`pose_source_package` defaults to `cuda_ndt_matcher_launch` in **all four**
files that declare it (`autosdv.launch.yaml:56`,
`logging_simulation.launch.yaml:60`, `autosdv_autoware.launch.xml:33`,
`components/tier4_localization_component.launch.xml:12`), and nothing anywhere
derives it from `pose_source`. Since `ndt` and `cuda_ndt` are indistinguishable
to the selector (§1), **both run the CUDA plugin**.

CLAUDE.md documents otherwise:

> `pose_source:=ndt       # Autoware NDT (OpenMP CPU, fallback)`

Worse, the obvious workaround does not exist: `ros2 launch` **rejects an empty
argument value** —

```
$ ros2 launch ... pose_source_package:=
malformed launch argument 'pose_source_package:=', expected format '<name>:=<value>'
```

— so before the §5.1 fix, built-in CPU NDT was not reachable from the command
line at all; it required editing a launch file. See §5.1.

## 3. The plugin contract

Defined solely by the caller (`pose_twist_estimator.launch.xml:55-66`); there is
no upstream spec. A pose source must ship
`$(find-pkg-share <pose_source_package>)/launch/pose_estimator.launch.xml`
declaring these seven args, all passed by name:

| arg | value the caller passes |
|---|---|
| `input_pointcloud` | `/localization/util/downsample/pointcloud` |
| `input_initial_pose_topic` | `/localization/pose_twist_fusion_filter/biased_pose_with_covariance` |
| `input_regularization_pose_topic` | `/sensing/gnss/pose_with_covariance` |
| `input_service_trigger_node` | `/localization/pose_estimator/trigger_node` |
| `output_pose_topic` | `/localization/pose_estimator/pose` |
| `output_pose_with_covariance_topic` | `/localization/pose_estimator/pose_with_covariance` |
| `client_map_loader` | `/map/get_differential_pointcloud_map` |

It is included under `push-ros-namespace pose_estimator` inside namespace
`localization`, so relative names resolve under `/localization/pose_estimator/`.

**Beyond the args, the estimator must also serve two services** — implicit,
un-plumbed parts of the contract:

- `ndt_align_srv` (`autoware_internal_localization_msgs/srv/PoseWithCovarianceStamped`) —
  `pose_initializer.launch.xml:15` remaps `ndt_align` to it, and calls it
  whenever `ndt_enabled: true`. This is the align step discussed in
  `mcl_initialization_and_covariance.md` §3.
- `trigger_node` (`std_srvs/srv/SetBool`) — `pose_initializer.launch.xml:20`;
  without it the initializer cannot pause/resume the estimator and
  initialization blocks.

**What `ekf_localizer` consumes** (`pose_twist_fusion_filter.launch.xml:4-16`):
`/localization/pose_estimator/pose_with_covariance`, type
`geometry_msgs/PoseWithCovarianceStamped`, frame `map`. The plain `PoseStamped`
on `/localization/pose_estimator/pose` is debug only. The 3-D reset topic is
`/initialpose3d`, **not** `/initialpose`.

## 4. Two integration patterns already exist in this repo

**cuda_ndt_matcher — the plugin.** Ships
`cuda_ndt_matcher_launch/launch/pose_estimator.launch.xml`, serves
`ndt_align_srv` and `trigger_node` (`init.rs:436-542`), publishes the contract
topics. Its header calls itself a "drop-in replacement for
autoware_ndt_scan_matcher". This is the clean, arbitrable path.

**Isaac — the bypass.** There is *no* `pose_estimator.launch.xml` anywhere under
`autoware_isaac_localization/`. It is launched as a sibling group by Layer A and
simply *publishes into the contract topic from outside*:

```xml
isaac_slam.launch.xml:58  <remap from="output/pose_with_covariance" to="/localization/pose_estimator/pose_with_covariance"/>
```

Initialization goes through a bridge that calls the ADAPI service
`/localization/initialize` rather than serving `ndt_align`.

The bypass is much cheaper and needs no edit to `available_args`. The cost is
that the estimator is invisible to the initializer's align step and to the
arbiter.

## 5. Recommendations

### 5.1 Fix the ndt / cuda_ndt coupling (small, do first)

Make `pose_source_package` derive from `pose_source` unless explicitly
overridden, so the documented options behave as documented:

| `pose_source` | resolved package | result |
|---|---|---|
| `cuda_ndt` (new default) | `cuda_ndt_matcher_launch` | CUDA NDT — today's actual default behaviour |
| `ndt` | `""` | built-in Autoware CPU NDT |
| explicit `pose_source_package:=X` | `X` | escape hatch for a third-party estimator |

Implemented as a `let` resolving an `auto` sentinel, verified against
`ros2 launch` for all reachable cases:

| invocation | resolved package |
|---|---|
| `<defaults>` | `cuda_ndt_matcher_launch` |
| `pose_source:=ndt` | `` (built-in NDT) |
| `pose_source:=isaac` | `` |
| `pose_source:=ndt pose_source_package:=my_pkg` | `my_pkg` |

Changing the default `pose_source` from `ndt` to `cuda_ndt` at the same time
keeps today's *behaviour* identical for anyone using defaults, while making
`ndt` mean what CLAUDE.md says.

### 5.2 Add 2D-MCL as a pose source

Recommended: **start with the Isaac-style bypass, keep the plugin as the
end state.** The bypass needs no fork of `pose_twist_estimator.launch.xml` and
no `available_args` edit, and it lets the 21 gaps below be closed incrementally
against a running system.

Gaps between 2D-MCL today and the contract, from most to least structural:

**Interface**
1. Publishes `PoseStamped` on `/pf/viz/inferred_pose`; EKF needs
   `PoseWithCovarianceStamped` on
   `/localization/pose_estimator/pose_with_covariance`. Type *and* name differ;
   no adapter exists.
2. `/pf/pose/odom` writes a 3×3 `(x,y,θ)` particle covariance into
   `covariance[0:9]` of a row-major **6×6** field, so `σ_yy` lands in the `x–z`
   slot and `σ_θθ` in `y–x`. The covariance is present but structurally wrong.
3. Pose publication is gated on `DO_VIZ` *and* on
   `pose_pub.get_subscription_count() > 0`, so the topic silently goes dead when
   nothing is listening — hostile to a late-joining EKF.
4. Frame ids carry leading slashes (`'/map'`, `'/laser'`), which tf2 rejects,
   and the TF is `map → laser`; Autoware needs `map → base_link` semantics. The
   node does no tf2 lookups at all, so the sensor→base offset must be composed
   externally.
5. Pose messages are stamped `get_clock().now()` while the TF uses the scan
   stamp — the two disagree, and EKF's delay compensation keys off the pose
   stamp. (This is defect §5.3 of the algorithm doc, still open.)

**Services and initialization**
6. No `ndt_align_srv`, no `trigger_node`. With `ndt_enabled: true` the
   initializer's align step has no server.
7. Subscribes `/initialpose` (2-D RViz topic) rather than `/initialpose3d`, and
   never calls `/localization/initialize`.
8. Also subscribes `/clicked_point` into the same callback — not an Autoware
   pattern.

**Map and inputs**
9. Consumes a 2-D `OccupancyGrid` via `nav2_map_server`'s `GetMap` service, not
   Autoware's PCD map via `/map/get_differential_pointcloud_map`. This is
   inherent to the method, not a defect — but it means the map-loading half of
   the contract can never be satisfied, which is an argument for the bypass.
10. Consumes `LaserScan` on `/scan` and `Odometry` on `/odom`; the contract
    supplies `PointCloud2` on `/localization/util/downsample/pointcloud`. The
    harness bridges this with `pointcloud_to_laserscan` plus a *runtime-generated*
    QoS bridge script and `wheel_imu_odom.py` — none of it in a launch file.
11. Updates are driven from `odomCB`, so a stalled twist source kills pose
    output entirely.
12. Ignores the EKF feedback (`biased_pose_with_covariance`) and the GNSS
    regularization pose — no closed loop with the fusion filter.

**Packaging**
13. No `launch/pose_estimator.launch.xml`; the only launch file is the upstream
    f1tenth `localize_launch.py`, which starts its own `map_server` and accepts
    none of the seven args.
14. Everything is orchestrated by `scripts/2dlidar/run-particle-filter.sh` via
    `setsid ros2 run` with a params file written at runtime — not includable,
    and its hard-coded absolute `/pf/...` names would not relocate under a
    pushed namespace anyway.
15. Package is named `particle_filter`, not `*_launch`; a wrapper package is
    needed regardless.
16. `pose_source` would have to contain the token `ndt` (e.g. `mcl_ndt`) to
    reach the plugin branch, or `available_args` must gain `'mcl'` plus a new
    `use_mcl_pose` branch — exactly the per-source editing the design proposal
    set out to eliminate.

### 5.3 Suggested order of work

1. Fix §5.1 (launch defaults) — independent of everything else.
2. A `mcl_pose_relay` node: subscribe the filter's pose, publish
   `PoseWithCovarianceStamped` on the contract topic with a *correctly laid out*
   6×6 covariance from the particle spread, stamped with the scan time, frame
   `map`, pose transformed to `base_link`. This closes gaps 1-5 and is the
   pose-covariance relay the original design report always called for.
3. An `autosdv_mcl_launch` package wrapping the filter, the scan chain and the
   relay in real launch files (gaps 10, 13-15), initially wired Isaac-style.
4. Serve `trigger_node`, then `ndt_align_srv` backed by the existing log-space
   field search (`score_sensor_model.py`) — which is also recommendation 3 of
   the initialization study, and would make GNSS seeding robust.
5. Only then consider the true plugin slot, which requires the
   `available_args`/`use_mcl_pose` edit in the forked
   `pose_twist_estimator.launch.xml`.
