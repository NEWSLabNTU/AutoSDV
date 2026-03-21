# Modular Localization Launch

**Goal**: Replace `tier4_localization_component.launch.xml` with a pluggable
pose source system, eliminate duplicated full-stack launch files, and verify
behaviour parity using COSS park rosbags.

**Status**: Planning

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

`play_launch 0.8.0` provides launch scope inspection without running any nodes:

```bash
# Dump the launch graph to a JSON record (dry run, no nodes started)
play_launch dump launch autosdv_launch logging_simulation.launch.yaml \
  --output /tmp/scope_before.json

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

Available rosbags under `rosbags/`:

| Bag | Date |
|-----|------|
| `localization_test_20260205_084127` | 2026-02-05 (most recent) |
| `localization_test_20260204_225143` | 2026-02-04 |
| `localization_test_20260204_105613` | 2026-02-04 |

The regression procedure for each phase that changes launch wiring:

```bash
# 1. Replay the rosbag against the localization stack
just launch-sim-logging pose_source:=ndt

# Or directly:
play_launch launch autosdv_launch logging_simulation.launch.yaml \
  pose_source:=ndt

ros2 bag play rosbags/localization_test_20260205_084127 --clock

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

## Phase 1: Baseline Capture

**Objective**: Capture the current launch graph and localization behaviour as
the reference for all subsequent regression tests.

**Why first**: Every later phase compares against this snapshot. Changes must
not break what works today.

### 1.1 Dump the current NDT launch scope

```bash
play_launch dump launch autosdv_launch logging_simulation.launch.yaml \
  pose_source:=ndt \
  --output tmp/scope_ndt_baseline.json

play_launch context tmp/scope_ndt_baseline.json --tree \
  > tmp/tree_ndt_baseline.txt

# Record which nodes are launched under /localization/
play_launch context tmp/scope_ndt_baseline.json --tree | grep localization
```

**Success criteria**:
- [ ] `scope_ndt_baseline.json` produced without errors
- [ ] Launch tree includes all expected nodes:
  `/localization/pose_estimator/ndt_scan_matcher`,
  `/localization/twist_estimator/gyro_odometer`,
  `/localization/pose_twist_fusion_filter/ekf_localizer`,
  `/localization/util/pose_initializer`
- [ ] Tree saved to `tmp/tree_ndt_baseline.txt` for diff comparison

### 1.2 Dump the current CUDA NDT launch scope

```bash
play_launch dump launch autosdv_launch logging_simulation.launch.yaml \
  pose_source:=cuda_ndt \
  --output tmp/scope_cuda_ndt_baseline.json

play_launch context tmp/scope_cuda_ndt_baseline.json --tree \
  > tmp/tree_cuda_ndt_baseline.txt
```

**Success criteria**:
- [ ] Dump succeeds
- [ ] `/localization/pose_estimator/cuda_ndt_scan_matcher` present in tree
- [ ] All other common nodes (EKF, gyro, pose_initializer) identical to NDT tree

### 1.3 COSS rosbag regression — NDT baseline

```bash
just launch-sim-logging pose_source:=ndt
# (in separate terminal)
ros2 bag play rosbags/localization_test_20260205_084127 --clock

# Capture output
ros2 bag record /localization/pose_with_covariance \
                /localization/kinematic_state \
                -o tmp/baseline_ndt.bag --duration 120
```

**Success criteria**:
- [ ] Localization initializes within 30 s of bag start
- [ ] `/localization/pose_with_covariance` publishing at ≥ 10 Hz
- [ ] No localization error monitor alerts during the run
- [ ] Pose trace saved to `tmp/baseline_ndt.bag`

### 1.4 COSS rosbag regression — CUDA NDT baseline

```bash
just launch-sim-logging pose_source:=cuda_ndt
ros2 bag play rosbags/localization_test_20260205_084127 --clock
ros2 bag record /localization/pose_with_covariance \
                /localization/kinematic_state \
                -o tmp/baseline_cuda_ndt.bag --duration 120
```

**Success criteria**:
- [ ] Same initialization and publishing rate as NDT
- [ ] Pose trace saved to `tmp/baseline_cuda_ndt.bag`

---

## Phase 2: `autosdv_localization_launch` Orchestrator

**Objective**: Create the central orchestrator package with all common
infrastructure. No behaviour change yet — still uses the old plugins
internally to verify the scaffolding compiles and the scope looks correct.

### 2.1 Create package skeleton

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
- [ ] Package builds: `colcon build --packages-select autosdv_localization_launch`

### 2.2 Implement `localization.launch.xml`

Implement the dynamic plugin dispatch:
```xml
<let name="pose_estimator_pkg"
     value="$(find-pkg-share $(var pose_source)_pose_estimator_launch)"/>
```

Full content in design doc.

**Success criteria**:
- [ ] File parses without XML errors (`ros2 launch --print-description`)

### 2.3 Implement common sub-launches

Extract from `cuda_localization.launch.xml`:
- `common/pose_twist_fusion_filter.launch.xml`
- `common/localization_error_monitor.launch.xml`
- `common/util/pointcloud_downsample.launch.xml` — with the renamed
  `localization_voxel_grid_downsample_filter` node to avoid container
  name collision

**Success criteria**:
- [ ] All three files syntactically valid
- [ ] `pointcloud_downsample.launch.xml` uses `localization_voxel_grid_downsample_filter`
  node name (same as current local `util.launch.xml` in `cuda_ndt_matcher`)

### 2.4 Implement `twist_estimator/gyro_odom.launch.xml`

Thin wrapper over `autoware_gyro_odometer`. Extracted from
`cuda_localization.launch.xml`.

**Success criteria**:
- [ ] File syntactically valid
- [ ] Topic remappings identical to those in current `cuda_localization.launch.xml`

---

## Phase 3: `ndt_pose_estimator_launch` Plugin

**Objective**: Create the NDT plugin package. Wire it into the orchestrator.
Verify structural parity with the NDT baseline via scope inspection and COSS
rosbag replay.

### 3.1 Create package

```
src/launcher/ndt_pose_estimator_launch/
├── package.xml
├── CMakeLists.txt
└── launch/
    ├── pose_estimator.launch.xml
    └── pose_initializer.launch.xml
```

**Success criteria**:
- [ ] Package builds: `colcon build --packages-select ndt_pose_estimator_launch`

### 3.2 Implement `pose_estimator.launch.xml`

Wraps:
1. `autosdv_localization_launch/common/util/pointcloud_downsample.launch.xml`
2. `autoware_ndt_scan_matcher/launch/ndt_scan_matcher.launch.xml`

Standard interface args: `input_pointcloud`, `output_pose_with_covariance`,
`config_dir`, `map_path`, `localization_pointcloud_container_name`,
`use_sim_time`.

**Success criteria**:
- [ ] File syntactically valid
- [ ] NDT node reads from `/localization/util/downsample/pointcloud`
- [ ] Output goes to `$(var output_pose_with_covariance)`

### 3.3 Implement `pose_initializer.launch.xml`

Sets `ndt_enabled=true`, `yabloc_enabled=false`. Handles
`stop_check_enabled` from `system_run_mode`. Launches
`autoware_automatic_pose_initializer` when `gnss_enabled=true`.

**Success criteria**:
- [ ] Capability flags match what `cuda_localization.launch.xml` currently uses

### 3.4 Wire orchestrator to NDT plugin

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
  <arg name="localization_pointcloud_container_name"
       value="$(var pointcloud_container_name)"/>
</include>
```

**Success criteria**:
- [ ] `autosdv_autoware.launch.xml` builds and passes `ros2 launch --print-description`

### 3.5 Scope inspection — NDT

```bash
play_launch dump launch autosdv_launch logging_simulation.launch.yaml \
  pose_source:=ndt \
  --output tmp/scope_ndt_after.json

play_launch context tmp/scope_ndt_after.json --tree > tmp/tree_ndt_after.txt
diff tmp/tree_ndt_baseline.txt tmp/tree_ndt_after.txt
```

**Success criteria**:
- [ ] `diff` shows zero meaningful differences in node FQNs and topic remappings
- [ ] Launch include tree reflects new package structure
  (`autosdv_localization_launch` → `ndt_pose_estimator_launch`)

### 3.6 COSS rosbag regression — NDT

```bash
just launch-sim-logging pose_source:=ndt
ros2 bag play rosbags/localization_test_20260205_084127 --clock
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

## Phase 4: `cuda_ndt_pose_estimator_launch` Plugin

**Objective**: Create the CUDA NDT plugin package inside `cuda_ndt_matcher`,
eliminating `cuda_localization.launch.xml` and `autoware_localization.launch.xml`.

### 4.1 Create package inside `cuda_ndt_matcher`

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

### 4.2 Implement `pose_estimator.launch.xml`

Identical structure to NDT plugin, but includes
`cuda_ndt_matcher_launch/launch/cuda_ndt_scan_matcher.launch.xml`
instead of Autoware's NDT.

Plugin-specific optional arg: `ndt_param_file` defaults to the package's
own `config/ndt_scan_matcher.param.yaml`.

**Success criteria**:
- [ ] CUDA NDT node reads from `/localization/util/downsample/pointcloud`
- [ ] Same standard interface args as NDT plugin

### 4.3 Implement `pose_initializer.launch.xml`

Identical to `ndt_pose_estimator_launch/launch/pose_initializer.launch.xml`
(CUDA NDT exposes the same `ndt_align_srv`).

**Success criteria**:
- [ ] `ndt_enabled=true` for CUDA NDT (same service name as Autoware NDT)

### 4.4 Scope inspection — CUDA NDT

```bash
play_launch dump launch autosdv_launch logging_simulation.launch.yaml \
  pose_source:=cuda_ndt \
  --output tmp/scope_cuda_ndt_after.json

play_launch context tmp/scope_cuda_ndt_after.json --tree \
  > tmp/tree_cuda_ndt_after.txt

diff tmp/tree_cuda_ndt_baseline.txt tmp/tree_cuda_ndt_after.txt
```

**Success criteria**:
- [ ] Node FQNs and remappings unchanged
- [ ] `/localization/util/downsample/pointcloud` is the CUDA NDT input
- [ ] `localization_voxel_grid_downsample_filter` node name preserved

### 4.5 COSS rosbag regression — CUDA NDT

```bash
just launch-sim-logging pose_source:=cuda_ndt
ros2 bag play rosbags/localization_test_20260205_084127 --clock
ros2 bag record /localization/pose_with_covariance \
                /localization/kinematic_state \
                -o tmp/regression_cuda_ndt.bag --duration 120
```

**Success criteria**:
- [ ] Position trace matches `tmp/baseline_cuda_ndt.bag` within 0.05 m RMS
- [ ] CUDA NDT throughput (Hz) unchanged from baseline

---

## Phase 5: Cleanup

**Objective**: Delete the old full-stack duplicate files now superseded by
the new plugin packages. Remove the AutoSDV override of
`tier4_localization_component.launch.xml`.

### 5.1 Delete superseded launch files

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

### 5.2 Remove AutoSDV `tier4_localization_component.launch.xml` override

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

### 5.3 Verify build after cleanup

```bash
just build
```

**Success criteria**:
- [ ] Workspace builds without errors or unresolved file references

### 5.4 Full regression — NDT and CUDA NDT after cleanup

Repeat Phase 3.6 and Phase 4.5 rosbag regression tests.

**Success criteria**:
- [ ] Both NDT and CUDA NDT regressions still pass (< 0.05 m RMS vs baseline)
- [ ] Scope inspection still matches Phase 3.5 and 4.4 outputs

### 5.5 Scope inspection — confirm include tree

```bash
play_launch dump launch autosdv_launch logging_simulation.launch.yaml \
  pose_source:=ndt --output tmp/scope_ndt_final.json

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

## Phase 6: Documentation Update

**Objective**: Update all references to the old launch files and document
how to add new pose source plugins.

### 6.1 Update `CLAUDE.md`

- Replace `cuda_ndt` pose source description with new package location
- Add section: "Adding a new pose source plugin"
  - Name the package `${pose_source}_pose_estimator_launch`
  - Implement `launch/pose_estimator.launch.xml` with standard 6-arg interface
  - Implement `launch/pose_initializer.launch.xml`
  - Build and scope-inspect with `play_launch dump`

**Success criteria**:
- [ ] `CLAUDE.md` no longer references `cuda_localization.launch.xml`
- [ ] Plugin authoring instructions present

### 6.2 Update `cuda_ndt_matcher/CLAUDE.md`

- Remove the section about local `util.launch.xml` and name collision
  (now handled in shared orchestrator)
- Add section describing `cuda_ndt_pose_estimator_launch` as the launch entry point

**Success criteria**:
- [ ] No stale references to `cuda_localization.launch.xml`

### 6.3 Update autosdv_launch preset README files

Verify `src/launcher/autosdv_launch/config/localization/preset/` docs
still accurately describe which `pose_source` values are valid.

**Success criteria**:
- [ ] Preset docs list: `ndt`, `cuda_ndt`, `eagleye`, `visual`, `isaac`

---

## Appendix: Scope Comparison Workflow

For any phase, the canonical comparison procedure is:

```bash
# 1. Dump before
play_launch dump launch autosdv_launch logging_simulation.launch.yaml \
  pose_source:=<X> --output tmp/scope_<X>_before.json

# 2. Make changes

# 3. Dump after
play_launch dump launch autosdv_launch logging_simulation.launch.yaml \
  pose_source:=<X> --output tmp/scope_<X>_after.json

# 4. Compare include trees
play_launch context tmp/scope_<X>_before.json --tree > tmp/tree_before.txt
play_launch context tmp/scope_<X>_after.json  --tree > tmp/tree_after.txt
diff tmp/tree_before.txt tmp/tree_after.txt

# 5. Spot-check a specific node for remapping changes
play_launch context tmp/scope_<X>_after.json \
  --node /localization/pose_estimator/ndt_scan_matcher
```

Expected output of `--tree` for `pose_source:=ndt` after Phase 5:

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

| Phase | Description          | Key Deliverable                                     | Regression     |
|-------|----------------------|-----------------------------------------------------|----------------|
| 1     | Baseline capture     | `tmp/baseline_*.bag`, `tmp/tree_*_baseline.txt`     | —              |
| 2     | Orchestrator package | `autosdv_localization_launch` builds                | scope only     |
| 3     | NDT plugin           | `ndt_pose_estimator_launch`, wired into main launch | scope + rosbag |
| 4     | CUDA NDT plugin      | `cuda_ndt_pose_estimator_launch`                    | scope + rosbag |
| 5     | Cleanup              | Old files deleted, workspace builds clean           | full rosbag    |
| 6     | Docs                 | CLAUDE.md updated                                   | —              |
