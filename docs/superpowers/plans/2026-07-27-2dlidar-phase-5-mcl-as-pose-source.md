# 2D-LiDAR Phase 5 — Make `pose_source:=mcl` a First-Class Option

> **For agentic workers:** REQUIRED SUB-SKILL: superpowers:subagent-driven-development. Tasks 1–4 are the map layer and are independent of 5–7 (the pose interface); Task 8 integrates and must run last.

**Goal:** `just launch ARGS="pose_source:=mcl"` brings up AutoSDV localizing on a 2-D occupancy grid, with the pose reaching `ekf_localizer` through the standard contract topic, and `just map-check` able to tell a user whether their map directory is valid before they try.

**Architecture:** Two halves that meet at the end. The **map layer** replaces the stock map component with an AutoSDV-owned one that loads the PCD only for NDT-family sources and a `nav2_map_server` grid only for `mcl`, and adds map-building/validation recipes. The **pose interface** wraps the existing filter, scan chain and odometry helper in a real launch package and adds a relay that republishes the filter's pose as `PoseWithCovarianceStamped` on `/localization/pose_estimator/pose_with_covariance` — the Isaac-style bypass, chosen because 2D-MCL can never satisfy the plugin contract's PCD map-loader client.

**Design references (read before starting):**
- `docs/design/map-handling-per-localization-method.md` — the map design, including the frame rule and the `autosdv_map.yaml` sidecar
- `docs/design/localization-method-switching.md` — how `pose_source` dispatches, the plugin contract, the 16 gaps, the recommended order
- `docs/research/localization/2d_mcl_algorithm.md` §5 — the filter's remaining defects (5.4, 5.5 still open)
- `docs/research/localization/mcl_initialization_and_covariance.md` — why heading seeding is still fragile

**Tech Stack:** ROS 2 launch (XML + YAML frontends), `nav2_map_server`, `autoware_map_loader`, the `particle_filter` fork (`NEWSLabNTU/particle_filter@autosdv`), the `tier4_localization_launch` fork (NEWSLabNTU-owned), numpy/PIL, pytest, `just`.

## Global Constraints

Carried forward from Phases 3–4, all still binding:

- fish login shell → run ROS things via `bash -c` or a script file; **never** bare-source a ROS setup in a top-level eval (it breaks `BASH_SOURCE` and triggers the interactive `setup.sh`)
- `range_libc`/`rosbag2_py` imports stay inside functions and run under a sourced env
- apt numpy 1.21.5 — never pip-install or upgrade numpy
- `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`; pure tests must pass in a plain shell
- temp files under `./tmp/`; recorded bags under `data/rosbags/phase3/` (gitignored)
- `set -euo pipefail` + `set +u`/`set -u` sourcing guard + `setsid` and PGID kill traps for background processes + `|| VAR=""` on command substitutions
- **ROS rejects int literals for double-typed parameters** — pass `60.0`, not `60`
- fork commits go on `autosdv` with a superproject gitlink bump; verify the pin is reachable at the URL in `.gitmodules`
- commit trailers as used on this branch; never commit `record.json`, `resume.org`, root `rosbags/`, or the design PDF

Specific to this phase:

- **Every change must be a no-op for `pose_source:=cuda_ndt` and `ndt`.** Those are the paths that run on the vehicle. Each task states how it verifies that.
- **Verify in `logging_simulation`, not on hardware.** Every task's runtime check uses the recorded sample-site bag.
- The Phase 3c variance caveat still applies: no accuracy claim from a single run.

## File Structure

```
src/launcher/autosdv_launch/launch/components/
    autosdv_map_component.launch.xml      # NEW: method-aware map loading
src/launcher/autosdv_launch/launch/
    autosdv_autoware.launch.xml           # MODIFIED: use the new component; forward use_pointcloud_map
src/localization/autosdv_mcl_launch/      # NEW package: the pose-source half
    launch/mcl_localization.launch.xml    #   filter + scan chain + odom + relay
    config/mcl.param.yaml
    package.xml, CMakeLists.txt
src/localization/autosdv_mcl_launch/src/
    mcl_pose_relay.py                     # NEW: PoseStamped -> PoseWithCovarianceStamped
    test_mcl_pose_relay.py
src/localization/tier4_localization_launch/launch/pose_twist_estimator/
    pose_twist_estimator.launch.xml       # MODIFIED (fork): 'mcl' in available_args
scripts/map/
    check_map.py                          # NEW: the validator behind `just map-check`
    test_check_map.py
justfile                                  # MODIFIED: map-check, map-grid-from-pcd, map-grid-from-bag
```

---

### Task 1 — Method-aware map component

**Files:** create `src/launcher/autosdv_launch/launch/components/autosdv_map_component.launch.xml`; modify `src/launcher/autosdv_launch/launch/autosdv_autoware.launch.xml`.

**Interfaces:** takes `map_path`, `lanelet2_map_file`, `pointcloud_map_file`, `occupancy_grid_file` (default `occupancy_grid.yaml`), and the resolved `pose_source`. Produces the same `/map/*` topics and services as the stock component for NDT sources, plus `/map_server/map` (`nav_msgs/srv/GetMap`) when `pose_source` is `mcl`.

Start from the stock file — copy
`/opt/autoware/1.5.0/share/autoware_launch/launch/components/tier4_map_component.launch.xml`
and the pieces of `tier4_map_launch/launch/map.launch.xml` it includes — then make
`pointcloud_map_loader` conditional. Note from the design doc §1 that the PCD
loader is a **composable node inside `map_container`**, so skipping it means
composing a container without that node, not adding an `if=` to a standalone node.
`lanelet2_map_loader`, `map_projection_loader`, `lanelet2_map_visualization`,
`vector_map_tf_generator` and `map_hash_generator` stay unconditional.

For `mcl`, add a `nav2_map_server` node. Two constraints that are easy to miss:
its **node name must be `map_server`**, because `particle_filter.py:530` hardcodes
the client `/map_server/map`; and it is a **lifecycle node**, so it needs
configure+activate — reuse the retry helper pattern from
`scripts/2dlidar/check-map-load.sh` rather than assuming one attempt works.

Also wire AutoSDV's `config/map/*.param.yaml`, which the design doc found to be
dead config (the stock include passes no arguments, so `/opt` configs load).

- [ ] **Step 1: verify the NDT path is untouched.** Launch logging_simulation with defaults, confirm `/map/pointcloud_map` and `/map/vector_map` both appear and NDT initialises, exactly as today. Record the topic list as the baseline.
- [ ] **Step 2: implement the component**, defaulting every new argument so the NDT path resolves identically.
- [ ] **Step 3: re-run Step 1's check** and diff the topic list against the baseline — it must match.
- [ ] **Step 4: verify the `mcl` path.** `pose_source:=mcl`: `/map/pointcloud_map` must be **absent**, `/map/vector_map` present, and `ros2 service call /map_server/map nav_msgs/srv/GetMap` must return a grid with `width > 0`.
- [ ] **Step 5: verify the failure mode is legible.** Point `map_path` at a directory with no grid and confirm the error names the missing file, rather than a lifecycle timeout or a bare exception.
- [ ] **Step 6: commit.**

---

### Task 2 — Forward `use_pointcloud_map`

**Files:** modify `src/launcher/autosdv_launch/launch/autosdv_autoware.launch.xml`.

The perception presets set `use_pointcloud_map` but it is never forwarded
(`autosdv_autoware.launch.xml:117-121` passes only `data_path` and
`pointcloud_container_name`), so the upstream default `true` always wins. Forward
it, and force it `false` when `pose_source:=mcl` — with no PCD,
`voxel_based_compare_map_filter` has nothing to filter against.

- [ ] **Step 1:** confirm the gap first — launch with `lidar_only` preset and check `ros2 node list` for the compare-map filter, showing the preset's value is ignored today. Quote the evidence.
- [ ] **Step 2:** forward the argument; resolve `false` for `mcl`.
- [ ] **Step 3:** verify — with `cuda_ndt` the filter node still appears (no regression); with `mcl` it does not, and obstacle segmentation still publishes.
- [ ] **Step 4: commit.**

---

### Task 3 — Bypass `map_height_fitter` for `mcl`

**Files:** modify the localization component/config wiring as needed.

`pose_initializer` uses `map_height_fitter` to snap a 2-D RViz click or a GNSS fix
onto the PCD surface (`gnss_initial_pose_auto_fix_target: pointcloud_map`). With
no PCD that either fails or hangs, and for a planar filter the height is
meaningless anyway.

Determine the minimal correct change — likely a parameter making the fit target
`none`/skip for `mcl` — and **verify that RViz "2D Pose Estimate" and GNSS
auto-init both still produce an `/initialpose3d`** in the `mcl` configuration.
If height must be supplied, take it from the lanelet2 map or hold it at the
grid's plane and say so explicitly in the report.

- [ ] **Step 1:** reproduce the problem — `pose_source:=mcl` with no PCD, attempt a GNSS auto-init, and capture what actually happens (hang, error, or silent no-op).
- [ ] **Step 2:** implement the bypass.
- [ ] **Step 3:** verify both init routes work, and that `cuda_ndt` height fitting is unchanged.
- [ ] **Step 4: commit.**

---

### Task 4 — `just map-check`

**Files:** create `scripts/map/check_map.py`, `scripts/map/test_check_map.py`; modify `justfile`.

**Interface:** `check_map.py <map_dir> [--pose-source mcl|cuda_ndt|ndt]`, exit 0 when the directory is usable for that source. Output format per the design doc §4 — one line per artefact, `ok`/`FAIL`/`-`, and on failure a message naming the remedy.

Checks: lanelet2 present and parseable; `map_projector_info.yaml` present, valid
`projector_type`, and whether it is georeferenced or `Local` (report which, and
that `Local` means no GNSS init); PCD present when required; grid `.yaml`/`.pgm`
present when required, with dimensions, resolution, origin and cell counts; and
**the frame-extent comparison** — the lanelet2 bounding box in map coordinates
versus the grid's coverage. That last check is the point of the whole task; it is
what catches a grid built in the wrong frame, the failure class that cost Phase 3
several weeks.

For a georeferenced map, deriving the lanelet2 bounding box in map coordinates
means projecting its lat/lon the same way `lanelet2_map_loader` does. Prefer
reusing Autoware's projection (`autoware_geography_utils`, or `lanelet2_python`
if available) over reimplementing a projection; if neither is usable from a plain
script, state that and compare `local_x`/`local_y` tags where present, reporting
"cannot verify" rather than guessing. **A check that silently passes on a wrong
frame is worse than no check.**

- [ ] **Step 1:** TDD the pure parts — grid yaml/pgm parsing, extent overlap maths (fully inside, partial, disjoint), the `Local` versus georeferenced branch, and a synthetic wrong-frame case that must FAIL.
- [ ] **Step 2:** implement; add the `just map-check` recipe.
- [ ] **Step 3:** run against three real directories: `data/COSS-map-planning` (has both PCD and grid), `data/sample-rosbag-replay/sample-map-rosbag` (has grid variants, MGRS), and a deliberately broken copy with a wrong-origin grid — which must FAIL with the remedy message.
- [ ] **Step 4: commit** with the three outputs quoted in the report.

---

### Task 5 — Map-building recipes and the sidecar

**Files:** modify `justfile`; modify `scripts/map/pcd_to_pgm.py` and `scripts/2dlidar/scan_accumulate_grid.py` only as needed to emit `autosdv_map.yaml`.

**Interfaces:**
`just map-grid-from-pcd MAP_DIR [--z-min A --z-max B --resolution R]` and
`just map-grid-from-bag BAG MAP_DIR [--min-hits N --resolution R]`. Both write
`occupancy_grid.{pgm,yaml}` into `MAP_DIR`, emit/update `autosdv_map.yaml` per the
design doc §4, and finish by running `map-check`.

The z-band is the one judgement the user must make, and Phase 1 produced an
unusable 230-cell grid by getting it wrong. So when `--z-min`/`--z-max` are
omitted, print the cloud's z distribution and a **suggested** band derived from
the per-cell ground estimate, and require the user to pass them explicitly rather
than silently picking. Guessing here is how the earlier failure happened.

- [ ] **Step 1:** implement `autosdv_map.yaml` emission plus tests for it.
- [ ] **Step 2:** add both recipes; make the no-band invocation print the distribution and exit asking for a band.
- [ ] **Step 3:** regenerate the COSS grid through the recipe and confirm byte-identical (or explain any difference) against the committed `data/COSS-map-planning/occupancy_grid.pgm`.
- [ ] **Step 4: commit.**

---

### Task 6 — `mcl_pose_relay`

**Files:** create `src/localization/autosdv_mcl_launch/` package with `src/mcl_pose_relay.py` and `test_mcl_pose_relay.py`.

**Interface:** subscribes the filter's pose, publishes
`geometry_msgs/PoseWithCovarianceStamped` on
`/localization/pose_estimator/pose_with_covariance` (remappable). This closes
gaps 1–5 of `localization-method-switching.md` §5.2, and it is the
pose-covariance relay the original design report always specified.

It must fix, not inherit, the filter's interface defects:

- **Covariance laid out correctly.** The filter writes a 3×3 `(x,y,θ)` covariance into `covariance[0:9]` of a row-major 6×6, so `σ_yy` lands in the `x–z` slot. The relay must place the planar terms at indices 0, 1, 5 / 6, 7, 11 / 30, 31, 35 and put sane large values on the unused z/roll/pitch diagonals.
- **Stamp with the measurement time**, not `now()` — the filter stamps poses with wall clock while its TF uses the scan stamp (algorithm doc §5.3). Prefer `/pf/pose/odom`, which carries covariance, over `/pf/viz/inferred_pose`, which cannot.
- **Frame ids without leading slashes** (`map`, not `/map`) — tf2 rejects them.
- **Pose expressed for `base_link`**, not `laser`; compose the sensor→base transform via tf2 rather than assuming identity.
- Do not gate publication on subscriber count (the filter does; a late EKF gets nothing).

- [ ] **Step 1:** TDD the pure conversions — 3×3 → 6×6 index placement (assert each term's destination index explicitly), quaternion/yaw handling, frame-id normalisation.
- [ ] **Step 2:** implement the node.
- [ ] **Step 3:** verify against a recorded run: replay `data/rosbags/phase3/seedmatrix_initgate/fixed_s1`, confirm the relay's output has correct covariance placement, monotonically sane stamps, and `frame_id == "map"`.
- [ ] **Step 4: commit.**

---

### Task 7 — `autosdv_mcl_launch`

**Files:** the launch file, config and package files in `src/localization/autosdv_mcl_launch/`.

Turn `scripts/2dlidar/run-particle-filter.sh` into real launch: the filter, the
`pointcloud_to_laserscan` chain, the QoS bridge, `wheel_imu_odom.py` and the
relay. The script stays as the experiment harness; the launch file is what
`pose_source:=mcl` uses.

Two known wrinkles: the QoS bridge is currently a **runtime-generated Python
file** (`tmp/scan_qos_bridge.py`) — promote it to a real node in the package. And
the filter's topic names are hard-coded absolute `/pf/...`, so pushing a namespace
will not relocate them; decide whether to parameterise them in the fork or accept
the global names, and say which.

- [ ] **Step 1:** create the package; `colcon build --packages-select autosdv_mcl_launch`.
- [ ] **Step 2:** port the node lineup, carrying over every hard-won parameter (`IMU_YAW_SIGN=-1.0`, `SCAN_MIN_HEIGHT`/`MAX_HEIGHT` from the sensor height, `SCAN_RANGE_MAX=60.0`, `sensor_model_variant=normalized_short`, `skip_nonfinite_beams`, `update_on_new_scan_only`, `require_initialpose`). Defaults should be the Phase 4 configuration, since that is the one that passes the gate.
- [ ] **Step 3:** verify standalone — launch it against the replay bag and confirm the relay publishes on the contract topic at roughly scan rate.
- [ ] **Step 4: commit.**

---

### Task 8 — Name it `mcl` and integrate

**Files:** modify `src/localization/tier4_localization_launch/launch/pose_twist_estimator/pose_twist_estimator.launch.xml` (fork); modify `autosdv.launch.yaml` / `logging_simulation.launch.yaml`.

Add `'mcl'` to `available_args` and a `use_mcl_pose` branch that includes
`autosdv_mcl_launch`. Follow the Isaac precedent for initialization (the pose
reaches the contract topic from outside; `ndt_enabled` stays false for `mcl` so
`pose_initializer` does not wait on an `ndt_align` service we do not serve).

Then the end-to-end gate: replay the sample bag with `pose_source:=mcl` and
confirm `/localization/kinematic_state` is produced and tracks NDT ground truth
to roughly the Phase 4 accuracy — **five seeds, median and range**, per the
variance caveat. Regenerate the three trajectory figures with
`scripts/2dlidar/plot_trajectories.py`.

- [ ] **Step 1:** fork edit + launch wiring.
- [ ] **Step 2:** verify `cuda_ndt` and `ndt` still behave identically (topic list diff against Task 1's baseline).
- [ ] **Step 3:** five-seed end-to-end run with `pose_source:=mcl`, through `ekf_localizer`, compared against ground truth.
- [ ] **Step 4:** figures, report to `docs/reports/2dlidar-phase5-mcl-pose-source.md`, commit, push branch and fork.

---

## Out of Scope

- `slam_toolbox` / `just map-survey` — the survey path stays documentation until a site genuinely lacks a PCD. Whoever implements it owns the frame decision from the map design §3.
- Algorithm doc §5.4 (beam correlation) and §5.5 (range clamping) — inherent to the beam model.
- The `ndt_align_srv` heading-refinement step (initialization study recommendation 3). It is the real fix for the 104° GNSS yaw seed and should be its own phase; the Isaac-style bypass deliberately does not need it.
- Cross-site validation (COSS, live hardware). Everything here is verified on the sample site.

## Self-Review Notes

- Tasks 1–4 (map) and 6–7 (pose) are independent; only Task 8 needs both, so they can proceed in parallel if two workers are available.
- Every task states how it proves the `cuda_ndt`/`ndt` path is unaffected, because that is the path that runs on the vehicle.
- Task 4's frame-extent check is the highest-value item per line of code in the phase; if it cannot be done honestly for georeferenced maps, it must report "cannot verify" rather than pass.
- Task 6 fixes interface defects rather than propagating them; the covariance index placement is worth asserting term by term in tests, since the existing bug is exactly a mis-indexed copy.
