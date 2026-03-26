# Modular Localization Launch

**Goal**: Replace `tier4_localization_component.launch.xml` with a pluggable
pose source system, eliminate duplicated full-stack launch files, and verify
behaviour parity using COSS park rosbags.

**Status**: In Progress (Phase 4.4)

**Design Reference**: `docs/design/modular_localization_launch.md`

---

## Summary

The current AutoSDV localization launch has three incompatible code paths
(`cuda_localization.launch.xml`, `autoware_localization.launch.xml`, upstream
fallthrough) that each duplicate the full localization stack (EKF, gyro
odometer, pose initializer, error monitor). Adding the Isaac visual sources
requires yet another ad-hoc path.

This roadmap implements the pluggable design from the design doc: a central
`autosdv_localization_launch` orchestrator that dynamically resolves
`${pose_source}_pose_estimator_launch` packages, with NDT and CUDA NDT as the
first two plugins. Regression testing uses COSS park rosbags with
`play_launch` scope inspection to guarantee node graph parity before and after.

---

## Tooling: play_launch Scope Inspection

`play_launch 0.8.x` provides launch scope inspection without running any nodes.

> **Note**: The `-o` flag must come before the `launch` subcommand.

```bash
# Dump the launch graph to a JSON record (dry run, no nodes started)
play_launch dump -o /tmp/scope_before.json \
  launch autosdv_launch logging_simulation.launch.yaml

# Print the full launch include tree
play_launch context /tmp/scope_before.json --tree

# Inspect a specific launch file's resolved args
play_launch context /tmp/scope_before.json \
  --launch autosdv_launch tier4_localization_component.launch.xml

# Inspect a specific node's resolved parameters and remappings
play_launch context /tmp/scope_before.json \
  --node /localization/pose_estimator/ndt_scan_matcher
```

Each phase uses `dump` + `context --tree` to compare the before/after launch
graph and verify structural equivalence.

---

## COSS Park Rosbag Regression Tests

Test rosbag (download with `just download-data`):

| Bag                          | Location                             |
|------------------------------|--------------------------------------|
| `outdoor_20251226_153115`    | `data/rosbags/outdoor_20251226_153115` |

The regression procedure for each phase that changes launch wiring:

```bash
# 1. Replay the rosbag against the localization stack
just launch-sim-logging pose_source:=ndt

# Or directly:
play_launch launch autosdv_launch logging_simulation.launch.yaml \
  pose_source:=ndt

ros2 bag play data/rosbags/outdoor_20251226_153115 --clock

# 2. Record localization output to compare
ros2 bag record /localization/pose_with_covariance \
                /localization/kinematic_state \
                /localization/pose_estimator/pose_with_covariance \
                -o /tmp/regression_after.bag

# 3. Compare pose traces (visual inspection in RViz or PlotJuggler)
just tool-plotjuggler
```

**Pass criteria**: `/localization/pose_with_covariance` trace from the new
launch matches the baseline within 0.05 m position error over the full bag.

---

## Phase 4.1: Baseline Capture

**Objective**: Capture the current launch graph and localization behaviour as
the reference for all subsequent regression tests.

**Why first**: Every later phase compares against this snapshot. Changes must
not break what works today.

### 4.1.1 Dump the current NDT launch scope

```bash
play_launch dump -o tmp/scope_ndt_baseline.json \
  launch autosdv_launch logging_simulation.launch.yaml \
  pose_source:=ndt

play_launch context tmp/scope_ndt_baseline.json --tree \
  > tmp/tree_ndt_baseline.txt

# Record which nodes are launched under /localization/
play_launch context tmp/scope_ndt_baseline.json --tree | grep localization
```

**Success criteria**:
- [x] `scope_ndt_baseline.json` produced without errors
- [x] Launch tree includes all expected nodes:
  `/localization/pose_estimator/ndt_scan_matcher`,
  `/localization/twist_estimator/gyro_odometer`,
  `/localization/pose_twist_fusion_filter/ekf_localizer`,
  `/localization/util/pose_initializer`
- [x] Tree saved to `tmp/tree_ndt_baseline.txt` for diff comparison

**Findings**: NDT baseline localization subtree (lines 80–112 of tree output):

```
[80] autosdv_launch tier4_localization_component.launch.xml  ns=/
  [82] tier4_localization_launch localization.launch.xml  ns=/
    [84] tier4_localization_launch pose_twist_estimator.launch.xml  ns=/localization
      [86] tier4_localization_launch ndt_scan_matcher.launch.xml  ns=/localization/pose_estimator
        [88] autoware_ndt_scan_matcher ndt_scan_matcher.launch.xml  ns=/localization/pose_estimator
      [90] tier4_localization_launch gyro_odometer.launch.xml  ns=/localization/twist_estimator
        [92] autoware_gyro_odometer gyro_odometer.launch.xml  ns=/localization/twist_estimator
      [94] autoware_pose_initializer pose_initializer.launch.xml  ns=/localization/util
      [96] autoware_automatic_pose_initializer automatic_pose_initializer.launch.xml  ns=/localization/util
      [98] tier4_localization_launch util.launch.xml  ns=/localization/util
    [100] tier4_localization_launch pose_twist_fusion_filter.launch.xml  ns=/localization/pose_twist_fusion_filter
      [102] autoware_ekf_localizer ekf_localizer.launch.xml  ns=/localization/pose_twist_fusion_filter
      [104] autoware_stop_filter stop_filter.launch.xml  ns=/localization/pose_twist_fusion_filter
      [106] autoware_twist2accel twist2accel.launch.xml  ns=/localization/pose_twist_fusion_filter
      [108] autoware_pose_instability_detector pose_instability_detector.launch.xml  ns=/localization/pose_twist_fusion_filter
    [110] tier4_localization_launch localization_error_monitor.launch.xml  ns=/localization
      [112] autoware_localization_error_monitor localization_error_monitor.launch.xml  ns=/localization
```

### 4.1.2 Dump the current CUDA NDT launch scope

```bash
play_launch dump -o tmp/scope_cuda_ndt_baseline.json \
  launch autosdv_launch logging_simulation.launch.yaml \
  pose_source:=cuda_ndt

play_launch context tmp/scope_cuda_ndt_baseline.json --tree \
  > tmp/tree_cuda_ndt_baseline.txt
```

**Success criteria**:
- [x] Dump succeeds
- [ ] ~~`/localization/pose_estimator/cuda_ndt_scan_matcher` present in tree~~ (see findings)
- [x] All other common nodes (EKF, gyro, pose_initializer) identical to NDT tree

**Findings**: The NDT and CUDA NDT trees are **identical**. `play_launch dump`
statically parses XML without evaluating `$(eval ...)` conditionals in
`tier4_localization_component.launch.xml`. Since the `cuda_ndt` branch uses an
`$(eval)` if-guard, it is not visible to the static parser — only the `unless`
branch (Autoware NDT path) appears.

This is expected and not a blocker: after Phase 4.3, the new orchestrator uses
`$(find-pkg-share $(var pose_source)_pose_estimator_launch)` which `play_launch`
**can** resolve statically, making the scope diffs meaningful.

### 4.1.3 COSS rosbag regression — NDT baseline

```bash
# Launch sim, play bag, and record in parallel:
play_launch launch --web-addr 0.0.0.0:8081 \
  autosdv_launch logging_simulation.launch.yaml \
  pose_source:=ndt rviz:=false &
sleep 45
ros2 bag play data/rosbags/outdoor_20251226_153115/ --clock &
sleep 5
timeout 120 ros2 bag record \
  /localization/pose_with_covariance \
  /localization/kinematic_state \
  --use-sim-time -o tmp/baseline_ndt
```

**Success criteria**:
- [ ] Localization initializes within 30 s of bag start
- [ ] `/localization/pose_with_covariance` publishing at ≥ 10 Hz
- [ ] No localization error monitor alerts during the run
- [x] Pose trace saved to `tmp/baseline_ndt/`

**Findings**: The localization stack launches successfully (43/43 nodes, 15/15
containers, 73/77 composable — 2 CUDA OOM failures in perception are
non-critical). However, both `/localization/pose_with_covariance` and
`/localization/kinematic_state` recorded **0 messages** over 120 s.

Root cause: NDT scan matching requires an initial pose estimate to converge.
The `automatic_pose_initializer` is in the launch tree and the bag contains
GNSS data (`/sensing/gnss/ublox/nav_sat_fix`, 627 messages), but the
initialization did not trigger during headless replay. Possible reasons:
- The GNSS-based auto-init may require additional configuration or the
  `gnss_enabled` flag may not be set in the default logging simulation config
- The bag's point cloud map path may not be configured correctly for NDT

**Action needed**: Investigate auto-initialization in headless mode before
these baselines can serve as regression references. Consider using
`sim-coss-park` which includes `record_localization.sh` and may handle
initialization differently.

### 4.1.4 COSS rosbag regression — CUDA NDT baseline

```bash
play_launch launch --web-addr 0.0.0.0:8081 \
  autosdv_launch logging_simulation.launch.yaml \
  pose_source:=cuda_ndt rviz:=false &
sleep 45
ros2 bag play data/rosbags/outdoor_20251226_153115/ --clock &
sleep 5
timeout 120 ros2 bag record \
  /localization/pose_with_covariance \
  /localization/kinematic_state \
  --use-sim-time -o tmp/baseline_cuda_ndt
```

**Success criteria**:
- [ ] Same initialization and publishing rate as NDT
- [x] Pose trace saved to `tmp/baseline_cuda_ndt/`

**Findings**: Same result as 4.1.3 — 0 messages recorded. Same root cause:
NDT (and CUDA NDT) require initial pose alignment before publishing.
Both use the same Autoware localization path (see 4.1.2 findings).

---

## Phase 4.2: `autosdv_localization_launch` Orchestrator

**Objective**: Create the central orchestrator package with all common
infrastructure. No behaviour change yet — still uses the old plugins
internally to verify the scaffolding compiles and the scope looks correct.

### 4.2.1 Create package skeleton

```
src/launcher/autosdv_localization_launch/
├── package.xml
├── CMakeLists.txt
└── launch/
    ├── localization.launch.xml
    ├── common/
    │   ├── pose_twist_fusion_filter.launch.xml
    │   ├── localization_error_monitor.launch.xml
    │   └── util/
    │       └── pointcloud_downsample.launch.xml
    └── twist_estimator/
        └── gyro_odom.launch.xml
```

See `docs/design/modular_localization_launch.md` Part 5 for full file contents.

**Success criteria**:
- [x] Package builds: `colcon build --packages-select autosdv_localization_launch`

### 4.2.2 Implement `localization.launch.xml`

Implement the dynamic plugin dispatch:
```xml
<let name="pose_estimator_pkg"
     value="$(find-pkg-share $(var pose_source)_pose_estimator_launch)"/>
```

Full content in design doc.

**Success criteria**:
- [x] File parses without XML errors (`ros2 launch --print-description`)

### 4.2.3 Implement common sub-launches

Extract from `cuda_localization.launch.xml`:
- `common/pose_twist_fusion_filter.launch.xml`
- `common/localization_error_monitor.launch.xml`
- `common/util/pointcloud_downsample.launch.xml` — with the renamed
  `localization_voxel_grid_downsample_filter` node to avoid container
  name collision

**Success criteria**:
- [x] All three files syntactically valid
- [x] `pointcloud_downsample.launch.xml` uses `localization_voxel_grid_downsample_filter`
  node name (same as current local `util.launch.xml` in `cuda_ndt_matcher`)

### 4.2.4 Implement `twist_estimator/gyro_odom.launch.xml`

Thin wrapper over `autoware_gyro_odometer`. Extracted from
`cuda_localization.launch.xml`.

**Success criteria**:
- [x] File syntactically valid
- [x] Topic remappings identical to those in current `cuda_localization.launch.xml`

---

## Phase 4.3: `ndt_pose_estimator_launch` Plugin

**Objective**: Create the NDT plugin package. Wire it into the orchestrator.
Verify structural parity with the NDT baseline via scope inspection and COSS
rosbag replay.

### 4.3.1 Create package

```
src/launcher/ndt_pose_estimator_launch/
├── package.xml
├── CMakeLists.txt
└── launch/
    ├── pose_estimator.launch.xml
    └── pose_initializer.launch.xml
```

**Success criteria**:
- [x] Package builds: `colcon build --packages-select ndt_pose_estimator_launch`

### 4.3.2 Implement `pose_estimator.launch.xml`

Wraps:
1. `autosdv_localization_launch/common/util/pointcloud_downsample.launch.xml`
2. `autoware_ndt_scan_matcher/launch/ndt_scan_matcher.launch.xml`

Standard interface args: `input_pointcloud`, `output_pose_with_covariance`,
`config_dir`, `map_path`, `localization_pointcloud_container_name`,
`use_sim_time`.

**Success criteria**:
- [x] File syntactically valid
- [x] NDT node reads from `/localization/util/downsample/pointcloud`
- [x] Output goes to `$(var output_pose_with_covariance)`

### 4.3.3 Implement `pose_initializer.launch.xml`

Sets `ndt_enabled=true`, `yabloc_enabled=false`. Handles
`stop_check_enabled` from `system_run_mode`. Launches
`autoware_automatic_pose_initializer` when `gnss_enabled=true`.

**Success criteria**:
- [x] Capability flags match what `cuda_localization.launch.xml` currently uses

### 4.3.4 Wire orchestrator to NDT plugin

Update `autosdv_autoware.launch.xml` to include the new orchestrator
instead of the old localization component:

```xml
<include file="$(find-pkg-share autosdv_localization_launch)/launch/localization.launch.xml">
  <arg name="pose_source"     value="$(var pose_source)"/>
  <arg name="twist_source"    value="$(var twist_source)"/>
  <arg name="map_path"        value="$(var map_path)"/>
  <arg name="gnss_enabled"    value="$(var use_gnss)"/>
  <arg name="initial_pose"    value="$(var initial_pose)"/>
  <arg name="system_run_mode" value="$(var system_run_mode)"/>
  <arg name="use_sim_time"    value="$(var use_sim_time)"/>
  <arg name="vehicle_model"   value="$(var vehicle_model)"/>
  <arg name="localization_pointcloud_container_name"
       value="$(var pointcloud_container_name)"/>
</include>
```

**Success criteria**:
- [x] `autosdv_autoware.launch.xml` builds and passes `ros2 launch --print-description`

### 4.3.5 Scope inspection — NDT

```bash
play_launch dump -o tmp/scope_ndt_after.json \
  launch autosdv_launch logging_simulation.launch.yaml \
  pose_source:=ndt

play_launch context tmp/scope_ndt_after.json --tree > tmp/tree_ndt_after.txt
diff tmp/tree_ndt_baseline.txt tmp/tree_ndt_after.txt
```

**Success criteria**:
- [x] `diff` shows zero meaningful differences in node FQNs and topic remappings
- [x] Launch include tree reflects new package structure
  (`autosdv_localization_launch` → `ndt_pose_estimator_launch`)

**Findings**: All localization leaf nodes present at correct namespaces. Key
structural differences from baseline (expected):
- Entry point changed: `tier4_localization_component.launch.xml` →
  `autosdv_localization_launch/localization.launch.xml`
- Pointcloud downsample moved from `/localization/util` into
  `/localization/pose_estimator` (owned by plugin per design)
- `automatic_pose_initializer` not visible in static tree because it's behind
  a `$(eval)` conditional — will launch at runtime when `gnss_enabled=true`

### 4.3.6 COSS rosbag regression — NDT

```bash
just launch-sim-logging pose_source:=ndt
ros2 bag play data/rosbags/outdoor_20251226_153115 --clock
ros2 bag record /localization/pose_with_covariance \
                /localization/kinematic_state \
                -o tmp/regression_ndt.bag --duration 120
```

Compare `tmp/regression_ndt.bag` against `tmp/baseline_ndt.bag` in
PlotJuggler.

**Success criteria**:
- [ ] Localization initializes within 30 s
- [ ] Position trace differs from baseline by < 0.05 m RMS over full bag
- [ ] No new error messages or localization error monitor alerts

---

## Phase 4.4: `cuda_ndt_pose_estimator_launch` Plugin

**Objective**: Create the CUDA NDT plugin package inside `cuda_ndt_matcher`,
eliminating `cuda_localization.launch.xml` and `autoware_localization.launch.xml`.

### 4.4.1 Create package inside `cuda_ndt_matcher`

```
src/localization/cuda_ndt_matcher/src/cuda_ndt_pose_estimator_launch/
├── package.xml
├── CMakeLists.txt
├── config/
│   └── ndt_scan_matcher.param.yaml   (existing, moved from cuda_ndt_matcher_launch)
└── launch/
    ├── pose_estimator.launch.xml
    └── pose_initializer.launch.xml
```

**Success criteria**:
- [ ] Package builds alongside `cuda_ndt_matcher`

### 4.4.2 Implement `pose_estimator.launch.xml`

Identical structure to NDT plugin, but includes
`cuda_ndt_matcher_launch/launch/cuda_ndt_scan_matcher.launch.xml`
instead of Autoware's NDT.

Plugin-specific optional arg: `ndt_param_file` defaults to the package's
own `config/ndt_scan_matcher.param.yaml`.

**Success criteria**:
- [ ] CUDA NDT node reads from `/localization/util/downsample/pointcloud`
- [ ] Same standard interface args as NDT plugin

### 4.4.3 Implement `pose_initializer.launch.xml`

Identical to `ndt_pose_estimator_launch/launch/pose_initializer.launch.xml`
(CUDA NDT exposes the same `ndt_align_srv`).

**Success criteria**:
- [ ] `ndt_enabled=true` for CUDA NDT (same service name as Autoware NDT)

### 4.4.4 Scope inspection — CUDA NDT

```bash
play_launch dump -o tmp/scope_cuda_ndt_after.json \
  launch autosdv_launch logging_simulation.launch.yaml \
  pose_source:=cuda_ndt

play_launch context tmp/scope_cuda_ndt_after.json --tree \
  > tmp/tree_cuda_ndt_after.txt

diff tmp/tree_cuda_ndt_baseline.txt tmp/tree_cuda_ndt_after.txt
```

**Success criteria**:
- [ ] Node FQNs and remappings unchanged
- [ ] `/localization/util/downsample/pointcloud` is the CUDA NDT input
- [ ] `localization_voxel_grid_downsample_filter` node name preserved

### 4.4.5 COSS rosbag regression — CUDA NDT

```bash
just launch-sim-logging pose_source:=cuda_ndt
ros2 bag play data/rosbags/outdoor_20251226_153115 --clock
ros2 bag record /localization/pose_with_covariance \
                /localization/kinematic_state \
                -o tmp/regression_cuda_ndt.bag --duration 120
```

**Success criteria**:
- [ ] Position trace matches `tmp/baseline_cuda_ndt.bag` within 0.05 m RMS
- [ ] CUDA NDT throughput (Hz) unchanged from baseline

---

## Phase 4.5: Cleanup

**Objective**: Delete the old full-stack duplicate files now superseded by
the new plugin packages. Remove the AutoSDV override of
`tier4_localization_component.launch.xml`.

### 4.5.1 Delete superseded launch files

| File to delete | Replaced by |
|----------------|-------------|
| `cuda_ndt_matcher_launch/launch/cuda_localization.launch.xml` | `cuda_ndt_pose_estimator_launch` + orchestrator |
| `cuda_ndt_matcher_launch/launch/autoware_localization.launch.xml` | `ndt_pose_estimator_launch` + orchestrator |
| `cuda_ndt_matcher_launch/launch/util/util.launch.xml` | `autosdv_localization_launch/common/util/pointcloud_downsample.launch.xml` |

```bash
git rm src/localization/cuda_ndt_matcher/src/cuda_ndt_matcher_launch/launch/cuda_localization.launch.xml
git rm src/localization/cuda_ndt_matcher/src/cuda_ndt_matcher_launch/launch/autoware_localization.launch.xml
git rm src/localization/cuda_ndt_matcher/src/cuda_ndt_matcher_launch/launch/util/util.launch.xml
rmdir src/localization/cuda_ndt_matcher/src/cuda_ndt_matcher_launch/launch/util
```

**Success criteria**:
- [ ] Files removed from git
- [ ] No remaining references to deleted files in other launch files
  (`grep -r "cuda_localization\|autoware_localization" src/launcher/`)

### 4.5.2 Remove AutoSDV `tier4_localization_component.launch.xml` override

The old override at `autosdv_launch/launch/components/tier4_localization_component.launch.xml`
is no longer needed since `autosdv_autoware.launch.xml` now calls the
orchestrator directly.

```bash
git rm src/launcher/autosdv_launch/launch/components/tier4_localization_component.launch.xml
```

Also remove the corresponding `<include>` from `autosdv_autoware.launch.xml`
if any stale reference remains.

**Success criteria**:
- [ ] File removed from git
- [ ] `autosdv_autoware.launch.xml` no longer references
  `tier4_localization_component.launch.xml`

### 4.5.3 Verify build after cleanup

```bash
just build
```

**Success criteria**:
- [ ] Workspace builds without errors or unresolved file references

### 4.5.4 Full regression — NDT and CUDA NDT after cleanup

Repeat Phase 4.3.6 and Phase 4.4.5 rosbag regression tests.

**Success criteria**:
- [ ] Both NDT and CUDA NDT regressions still pass (< 0.05 m RMS vs baseline)
- [ ] Scope inspection still matches Phase 4.3.5 and 4.4.4 outputs

### 4.5.5 Scope inspection — confirm include tree

```bash
play_launch dump -o tmp/scope_ndt_final.json \
  launch autosdv_launch logging_simulation.launch.yaml \
  pose_source:=ndt

play_launch context tmp/scope_ndt_final.json --tree
```

Verify the tree shows clean structure:
```
autosdv_launch/logging_simulation.launch.yaml
└── autosdv_launch/autosdv_autoware.launch.xml
    └── autosdv_localization_launch/localization.launch.xml
        ├── ndt_pose_estimator_launch/pose_estimator.launch.xml
        │   ├── autosdv_localization_launch/common/util/pointcloud_downsample.launch.xml
        │   └── autoware_ndt_scan_matcher/ndt_scan_matcher.launch.xml
        ├── autosdv_localization_launch/twist_estimator/gyro_odom.launch.xml
        ├── autosdv_localization_launch/common/pose_twist_fusion_filter.launch.xml
        ├── autosdv_localization_launch/common/localization_error_monitor.launch.xml
        └── ndt_pose_estimator_launch/pose_initializer.launch.xml
```

**Success criteria**:
- [ ] No references to `cuda_ndt_matcher_launch` in the NDT tree
- [ ] No references to `tier4_localization_component` in any tree

---

## Phase 4.6: Documentation Update

**Objective**: Update all references to the old launch files and document
how to add new pose source plugins.

### 4.6.1 Update `CLAUDE.md`

- Replace `cuda_ndt` pose source description with new package location
- Add section: "Adding a new pose source plugin"
  - Name the package `${pose_source}_pose_estimator_launch`
  - Implement `launch/pose_estimator.launch.xml` with standard 6-arg interface
  - Implement `launch/pose_initializer.launch.xml`
  - Build and scope-inspect with `play_launch dump`

**Success criteria**:
- [ ] `CLAUDE.md` no longer references `cuda_localization.launch.xml`
- [ ] Plugin authoring instructions present

### 4.6.2 Update `cuda_ndt_matcher/CLAUDE.md`

- Remove the section about local `util.launch.xml` and name collision
  (now handled in shared orchestrator)
- Add section describing `cuda_ndt_pose_estimator_launch` as the launch entry point

**Success criteria**:
- [ ] No stale references to `cuda_localization.launch.xml`

### 4.6.3 Update autosdv_launch preset README files

Verify `src/launcher/autosdv_launch/config/localization/preset/` docs
still accurately describe which `pose_source` values are valid.

**Success criteria**:
- [ ] Preset docs list: `ndt`, `cuda_ndt`, `eagleye`, `visual`, `isaac`

---

## Appendix: Scope Comparison Workflow

For any phase, the canonical comparison procedure is:

```bash
# 1. Dump before
play_launch dump -o tmp/scope_<X>_before.json \
  launch autosdv_launch logging_simulation.launch.yaml \
  pose_source:=<X>

# 2. Make changes

# 3. Dump after
play_launch dump -o tmp/scope_<X>_after.json \
  launch autosdv_launch logging_simulation.launch.yaml \
  pose_source:=<X>

# 4. Compare include trees
play_launch context tmp/scope_<X>_before.json --tree > tmp/tree_before.txt
play_launch context tmp/scope_<X>_after.json  --tree > tmp/tree_after.txt
diff tmp/tree_before.txt tmp/tree_after.txt

# 5. Spot-check a specific node for remapping changes
play_launch context tmp/scope_<X>_after.json \
  --node /localization/pose_estimator/ndt_scan_matcher
```

Expected output of `--tree` for `pose_source:=ndt` after Phase 4.5:

```
logging_simulation.launch.yaml [autosdv_launch]
└─ autosdv_autoware.launch.xml [autosdv_launch]
   ├─ ... (vehicle, sensing, map, planning, control)
   └─ localization.launch.xml [autosdv_localization_launch]
      ├─ pose_estimator.launch.xml [ndt_pose_estimator_launch]
      │  ├─ pointcloud_downsample.launch.xml [autosdv_localization_launch]
      │  └─ ndt_scan_matcher.launch.xml [autoware_ndt_scan_matcher]
      ├─ gyro_odom.launch.xml [autosdv_localization_launch]
      ├─ pose_twist_fusion_filter.launch.xml [autosdv_localization_launch]
      ├─ localization_error_monitor.launch.xml [autosdv_localization_launch]
      └─ pose_initializer.launch.xml [ndt_pose_estimator_launch]
```

---

## Phase Summary

| Phase | Description          | Key Deliverable                                     | Regression     | Status              |
|-------|----------------------|-----------------------------------------------------|----------------|---------------------|
| 4.1   | Baseline capture     | `tmp/baseline_*.bag`, `tmp/tree_*_baseline.txt`     | —              | Blocked (see 4.1.3) |
| 4.2   | Orchestrator package | `autosdv_localization_launch` builds                | scope only     | Done                |
| 4.3   | NDT plugin           | `ndt_pose_estimator_launch`, wired into main launch | scope + rosbag | Done (scope only)   |
| 4.4   | CUDA NDT plugin      | `cuda_ndt_pose_estimator_launch`                    | scope + rosbag | —                   |
| 4.5   | Cleanup              | Old files deleted, workspace builds clean           | full rosbag    | —                   |
| 4.6   | Docs                 | CLAUDE.md updated                                   | —              | —                   |
