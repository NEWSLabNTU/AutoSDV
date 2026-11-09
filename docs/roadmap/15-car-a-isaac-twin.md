# 15. Car A Isaac Twin Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Car A's digital twin drives a COSS route in Isaac Sim 6.1 under AutoSDV's own
Autoware.

**Architecture:**
- The twin car `twin` (individual_params) and the spec `autosdv_car_a.yaml` hold the same
  numbers. swb's generators turn the spec into the Isaac rig, the catalog and `spec_hash`.
- aib's runner hosts the car in a COSS world built from the scan by swb's
  `scripts/ntu_twin` pipeline, parameterized per site.
- AutoSDV's logging-simulation launch runs with drivers off and sim time on, and talks to aib
  through the I5 contract.

**Tech Stack:**
- Isaac Sim 6.1.0 (pip venv, Python 3.12) and aib / swb / sim_vehicle_spec / sim_vehicle_ecu.
- Autoware 1.5.0 and ROS 2 Humble.
- Python: usd-core, numpy, open3d, scipy, laspy.

**Spec:** `docs/superpowers/specs/2026-11-10-car-a-isaac-twin-design.md`

## Status (2026-11-10): done

All nine tasks are done; the guide, `docs/guides/isaac-twin.md`, carries the commands.
Measured on server133 (RTX 5090):

| what | result |
|---|---|
| vehicle check, flat plane | PASS; creep at rest 0.002 cm/s |
| COSS world | 12 tiles, 3.1 M triangles; lanes 100 % meshed, p99 roughness 2.6 mm |
| colour transfer | 60 % of points within 1.5 m of an original, 11 % within 6 cm |
| NDT on `coss_loop` | NVTL min 2.71, p50 2.99 (2395 scans) |
| `coss_loop` drive | 65.2 m in 40 s, top speed 2.005 m/s against the 2.0 cap |
| standing obstacle | not detected: perception emits no object, the car drives into it |

Deviations from the plan:
- Autoware's `ndt` instead of `cuda_ndt`, and the newest CUDA put first for NVRTC (the guide
  says why);
- a standing box in a world variant instead of a spawned pedestrian (the runner has no actor
  spawn);
- no `twin.launch.py`: `just twin up` starts aib's runner and Autoware as two process
  groups, since the runner is standalone and play_launch gives Autoware its multi-stage kill.

## Global Constraints

- **Host:** newslab-server133 (RTX 5090, driver 590.48). `ISAAC_ROOT=/home/aeon/isaac-sim`;
  161 GB free on `/`.
- **Never modify** `car_a`, `car_b`, `car_c` or `sim` in individual_params.
- **Mounts** (base_link, metres; every roll/pitch/yaw 0):

  | sensor | x | y | z |
  |---|---|---|---|
  | vlp16 = sensor_kit_base_link | 0.066 | 0 | 0.36 |
  | zedxm_camera_link | 0.32 | 0 | 0.14 |
  | gnss_base_link | −0.03 | −0.07 | 0.26 |
  | imu_link | 0.16 | 0 | 0.08 |

- **Geometry:** verbatim from `autosdv_vehicle_description/config/vehicle_info.param.yaml`.
- **Mass and limits:** mass 5.0 kg; centre of mass 0.15 m high, 45 % on the front axle;
  motor ceiling 2.0 m/s.
- **Launch settings:** `planning_profile:=rc`, `use_gnss:=false`, `camera_model:=none`,
  `vehicle_id:=twin`.
- **Names:**
  - spec and catalog entry: `autosdv_car_a`;
  - body catalogue id: `vehicle.newslab.autosdv_car_a`;
  - twin package: `src/simulation/autosdv_twin`;
  - recipes: `just twin <verb>`.
- **swb changes:** go to `~/repos/scenario-world-bridge` and its submodules, branch `main`,
  pushed innermost first. Never revert or commit another session's uncommitted work there:
  the swb submodule working trees are dirty today, so check `git status` before touching
  one.
- **No swb code copied** into AutoSDV. World assets are built into `data/coss-isaac/`
  (gitignored), never committed.
- **Temp files** go in `./tmp`.
- **Process hygiene:** kill only your own PGIDs; don't touch display :2.

## Review Focus

1. **Spec ↔ `twin` drift.** Editing one mount in either file must fail
   `just twin check`. Test: Task 2 `test_mounts_match`.
2. **Missing `map_projector_info.yaml`** beside the world's lanelet map sends aib's GNSS or
   initial pose about 34 km off. Test: Task 7 `test_world_layout`.
3. **Holes or pits under a lanelet:** the car falls through or snags. Test: Task 6
   `test_lane_cells_meshed` and roughness p99 ≤ 0.01 m.
4. **Autoware launched without `planning_profile:=rc`:** full-size limits, silently.
   Test: Task 8 `test_twin_launch_args`.
5. **Sensor nodes the sim lacks** (the ZED driver, a ZED IMU) start anyway and die, or
   publish nothing. Test: Task 8 step that lists the running nodes and topics.

---

## Phase A — Isaac on this host

### Task 1: Install Isaac Sim 6.1 and bring aib up

**Files:**
- Create: `docs/guides/isaac-twin.md` (install section only, extended later)

- [x] **Install.** `uv venv --python 3.12 /home/aeon/isaac-sim`, then
  `uv pip install "isaacsim[all,extscache]==6.1.0" --extra-index-url https://pypi.nvidia.com`.
  Record the exact versions installed.
- [x] **Accept the EULA and check the version.** Run
  `OMNI_KIT_ACCEPT_EULA=YES /home/aeon/isaac-sim/bin/python -c "import isaacsim; print('ok')"`.
  Expected: `ok`.
- [x] **Set up and test aib.** In `src/autoware_isaac_bridge` of swb, check that the working
  tree equals its pin (`git status`). If it is dirty, stop and report rather than reset.
  Then run `ISAAC_ROOT=/home/aeon/isaac-sim just setup && just build && just test && just test-isaac`.
  Expected: all pass.
- [x] **Smoke run.** `DISPLAY=:8 just isaac-ego duration:=20` on the flat plane (Model 3).
  Expected: `ros2 topic hz /clock` and the lidar topic tick, then a clean exit.
  - Use a fresh display. Check `:8` isn't someone else's first; never use `:2`.
- [x] **Write the guide's install section** with the exact commands and versions, then commit
  it to AutoSDV develop.

## Phase B — The twin car

### Task 2: `twin` instance, the spec, and the consistency check

**Files:**
- Create: `src/param/autoware_individual_params/individual_params/config/twin/**`
  - Copy car_a's `autosdv.preset.yaml`, `autosdv_vehicle/*`, and
    `autosdv_sensor_kit/imu_corrector.param.yaml`.
  - New `sensors_calibration.yaml`: base_link → sensor_kit_base_link =
    (0.066, 0, 0.36, 0, 0, 0).
  - New `sensor_kit_calibration.yaml`: vlp16 at 0. zedxm_camera_link
    (0.254, 0, −0.22). gnss_base_link (−0.096, −0.07, −0.10). imu_link (0.094, 0, −0.28).
    Every other kit entry is copied from car_a unchanged.
  - Each file's header says "twin: car A as known, mounts estimated from photos
    2026-11-10, see roadmap 15".
- Create: `src/simulation/autosdv_twin/{package.xml, CMakeLists.txt}` (ament_cmake,
  installing `vehicle isaac catalogs launch config`).
- Create: `src/simulation/autosdv_twin/vehicle/autosdv_car_a.yaml` (spec_version 2, the
  shape of golf cart's `newslab_golfcart.yaml`):
  - `body.catalogue_model: vehicle.newslab.autosdv_car_a`.
  - The geometry and mirror verbatim.
  - The inertia and mass above.
  - Sensors:
    - `vlp16`: lidar, 16 channels, 10 Hz, range 100, vertical_fov [−15, 15].
      Topic and frame are those AutoSDV's VLP-16 pipeline consumes; read them from
      `autosdv_sensor_kit_launch` for `sensor_suite` vlp16_zed.
    - `zed`: camera, `isaac.enabled: true`.
    - `imu`: `isaac.enabled: true`.
    - `gnss`: `isaac.enabled: false`.
  - Full `provenance`.
- Create: `src/simulation/autosdv_twin/test/test_twin_consistency.py`.
- Modify: `src/param/autoware_individual_params` README or instance list, wherever instances
  are enumerated (grep `car_c` to find them).

**Interfaces:**
- **Produces:**
  - the spec path `$(find-pkg-share autosdv_twin)/vehicle/autosdv_car_a.yaml`;
  - `vehicle_id:=twin`.
- **Test:** `test_twin_consistency.py` composes base_link → sensor from the two twin
  calibration files and compares it with `spec.sensor_kit.mount` ∘ `sensors.<x>.mount`.

- [x] **Write the failing tests.**
  - `test_mounts_match`: every spec sensor's base_link pose equals the twin calibration's,
    within 1e-6.
  - `test_geometry_matches_vehicle_info`: the spec's `geometry` equals `vehicle_info.param.yaml`.
  - `test_car_a_untouched`: `git diff --quiet HEAD -- .../config/car_a` (subprocess) returns 0.
- [x] **Run `pytest src/simulation/autosdv_twin/test -q`.** Expected: FAIL (files missing).
- [x] **Write the instance and the spec.**
- [x] **Validate the spec** with
  `PYTHONPATH=~/repos/scenario-world-bridge/src/sim_vehicle_spec python3 -c "from sim_vehicle_spec.spec import load; load('src/simulation/autosdv_twin/vehicle/autosdv_car_a.yaml')"`
  (use the loader's actual name in `spec.py`).
  - Expected: no error.
  - If the format can't express something car A needs, extend sim_vehicle_spec on `main`
    with a test there, push it, and note it in provenance.
- [x] **Run the tests again.** Expected: PASS.
- [x] **Check the URDF.**
  `xacro .../autosdv_sensor_kit_description/urdf/sensors.xacro config_dir:=<twin dir>`
  must yield `vlp16` at z 0.36 in base_link. Use `tf2`-free math, or `check_urdf` plus a
  small script.
- [x] **Commit and push.** Commit individual_params in its submodule, push, then commit the
  pin and `autosdv_twin` in AutoSDV develop and push.

### Task 3: Generated Isaac assets — rig, catalog, spec_hash, VLP-16 lidar

**Files:**
- Create in `src/simulation/autosdv_twin/`:
  - `isaac/autosdv_car_a.rig.yaml`
  - `isaac/spec_hash.yaml`
  - `catalogs/vehicles.xosc`
  - `isaac/make_lidars.py`: VLP-16 only, from Nebula's VLP16 factory calibration (find it
    under `/opt/autoware/1.5.0`'s nebula share).
  - `isaac/lidars/VLP16.usda`
- Create: `just/twin.just` with `mod twin`, plus the private `default` and
  `set working-directory := '..'` per CLAUDE.md.
  - `generate`: runs aib.gen, sim_vehicle_spec.gen and make_lidars.
  - `check`: the same with `--check`, plus Task 2's pytest.
  - Register `mod twin 'just/twin.just'` in the root justfile.

- [x] **`just twin generate`.** Expected: four files written. A second run changes nothing
  (`git diff --quiet`).
- [x] **Failing check.** Hand-edit one mount in the spec, then run `just twin check`.
  Expected: FAIL naming the stale file. Revert the edit.
- [x] **Load the lidar.** Open `VLP16.usda` with Isaac's Python (`pxr.Usd.Stage.Open`).
  Expected: an OmniLidar prim with 16 emitters at the factory elevations −15…+15°.
- [x] **Commit and push** (AutoSDV develop).

### Task 4: Body USD, ego.yaml, car-A mesh for the vehicle description

**Files:**
- Create in `src/simulation/autosdv_twin/`:
  - `body/make_body.py`: procedural low-poly USD (usd-core + numpy), after golfcart_twin's.
    Parts: chassis tub, 4 wheels at the geometry's positions, shocks, the red rear mount
    with the GNSS puck, the Jetson box on red posts, the top plate, the VLP-16 on standoffs,
    the interface box, and the ZED.
  - `body/render_previews.py`
  - `isaac/body/vehicle.newslab.autosdv_car_a/{vehicle.newslab.autosdv_car_a.usda, wheels.usda, entry.json}`
  - `isaac/ego.yaml`: the rig, `node_name: isaac_vehicle`, and the body's `entry.json`.
- Body coordinates: origin on the ground at mid-length, x forward, z up, with the rear-axle
  offset declared in `entry.json` as golfcart's does.
- Create: `src/vehicle/autosdv_vehicle_launch/autosdv_vehicle_description/mesh/car_a.dae`,
  exported from the same body.
- Modify: `autosdv_vehicle_description/urdf/vehicle.xacro`. It takes the mesh from a
  `vehicle_mesh` value read from the instance's `individual_params` (the key is absent, so
  the box is kept, for car_b, car_c, sim and default; `car_a.dae` for car_a and twin).
  - Adding a mesh key to car_a's individual_params is the one permitted car_a edit, and it
    is visual only. Note it in the commit.

- [x] **Render previews** from the photos' four viewpoints (side ×2, front, rear) next to
  the photos in `tmp/twin_body/`. Expected:
  - the silhouettes overlap the photos within about 1 cm at the wheels, the lidar and the
    ZED, by eye;
  - lidar top at 0.39 m.
- [x] **Check the body box** with
  `python3 -c` reading `entry.json`: length 0.421 ± 0.01, width 0.333 ± 0.01, rear axle
  offset matching `rear_overhang`.
- [x] **Check the URDF meshes.** `xacro vehicle.xacro` with each instance resolves the mesh
  for car_a and twin, and the box for the others. Expected: diff of the car_b URDF against
  HEAD's is empty.
- [x] **Commit and push** (submodule first, then AutoSDV develop).

## Phase C — The car drives

### Task 5: vehicle-check on the flat plane; 1/10-scale dynamics

**Files:**
- Modify, as needed and in swb `main`:
  - the spec's `isaac.physx`, tires, suspension and brakes (in AutoSDV's spec);
  - aib or sim_vehicle_ecu defaults only when a value cannot be set per car. Message swb 241
    first in that case, giving the value and the reason.
- Create: `src/simulation/autosdv_twin/launch/twin_vehicle_check.launch.py`, or a `just twin
  vehicle-check` recipe that runs aib's `isaac-ego ego:=<twin ego.yaml>` and aib's
  `vehicle-check` against it.

- [x] **Run `just twin vehicle-check`** and record the first result in `tmp/twin_vc/`.
- [x] **Tune until it passes:**
  - topics, frames and stamps;
  - an acceleration step to 1.0 m/s;
  - braking to a stop;
  - a steering sweep to ±0.349;
  - parked for 30 s: creep < 0.5 cm/s, no wheel jitter.
  Each change is a spec edit with its reason in provenance.
- [x] **Regenerate and check.** `just twin generate && just twin check`.
- [x] **Commit and push**, swb submodules first if touched.

## Phase D — The COSS world

### Task 6: Parameterize swb's ntu_twin pipeline and build COSS

**Files (swb `main`):**
- Modify `scripts/ntu_twin/*.py`. Move the NTU constants (MGRS zone, SHIFT, MAP_ORIGIN,
  tile grid) into a site file, `scripts/ntu_twin/sites/{ntu,coss}.yaml`, selected by
  `--site`.
  - NTU's outputs must be byte-identical before and after: compare the output of one NTU
    tile.
- Create `scripts/ntu_twin/color_transfer.py`:
  `transfer(map_pcd: Path, originals: list[Path], out: Path, north_tol=0.15, tight_tol=0.03) -> Stats`.
  - Estimate the TWD97 → map offset from `map_projector_info.yaml`, refine it by ICP on a
    subsample, then search anisotropically for the nearest neighbour and write
    `x y z rgb intensity`.
- Create `scripts/ntu_twin/test/test_color_transfer.py`.
- Create (AutoSDV): a `just twin world` recipe that runs the pipeline with `--site coss`
  into `data/coss-isaac/`, and a `.gitignore` entry for it.

- [x] **Failing test.** `test_color_transfer.py::test_anisotropic_match`:
  - a synthetic precise cloud, plus a copy whose northing is quantized to 0.25 and shifted
    by a known offset, carrying distinct colours;
  - assert that ≥ 99 % of points get their own colour, and that the recovered offset is
    within 0.02 m.
- [x] **Run it.** Expected: FAIL. Implement until it passes.
- [x] **NTU regression.** Run one NTU tile before and after the refactor. Expected:
  identical outputs (`cmp`).
- [x] **Colour transfer on COSS** (reading the NAS originals, about 2.5 GB, with timeout
  guards). Expected: a stats line, unmatched < 2 %, and a ground orthophoto in which lane
  paint is visible (look at a 512² crop).
- [x] **Ground at 5 cm under lanelets.** Run `smooth_dtm` and `build_meshes --lanes --clear-road`.
  Add test `test_lane_cells_meshed`: every DTM cell under a lanelet polygon has a triangle.
  Expected:
  - roughness p99 |z − 3×3 median| ≤ 0.01 m under lanelets (printed by the recipe);
  - no pit deeper than 0.05 m under a lanelet.
- [x] **world_usd with Isaac's Python.** Run `world_usd --site coss`. Expected:
  - `data/coss-isaac/isaac/world.yaml`;
  - the USD loads in under 30 s;
  - VRAM recorded.
- [x] **Commit and push** swb `main`, then the AutoSDV recipe on develop.

### Task 7: World layout and alignment

**Files:**
- Create (AutoSDV), via `just twin world`: the map dir `data/coss-isaac/map/` holding
  `lanelet2_map.osm`, `map_projector_info.yaml` and `pointcloud_map.pcd`.
  - These are symlinks to `data/COSS-map-planning`, so the PCD Autoware loads is the cloud
    the world was meshed from.
  - `world.yaml` sits two levels below that map dir, as aib expects. Check aib's resolver
    for the exact relative path.
- Create: `src/simulation/autosdv_twin/test/test_world_layout.py`.

- [x] **Failing test** `test_world_layout`:
  - `map_projector_info.yaml` is found where aib resolves it from `world.yaml`;
  - `world.yaml`'s USD path is relative and exists;
  - the world's origin equals map frame − map_origin.
- [x] **Run, fix the layout, run.** Expected: PASS.
- [x] **Projector check.** Read aib's projector code for TransverseMercator with a non-zero
  origin. A twin pose given with `--map-pose` at a known lanelet point must appear at that
  map point in `/localization` (±0.05 m) once Autoware is up (closed in Task 8). If aib
  mishandles a TM origin, fix it in aib `main` with a test.
- [x] **Commit and push.**

## Phase E — Autoware drives the twin

### Task 8: Twin launch with our Autoware

**Files:**
- Modify: `src/launcher/autosdv_launch/launch/logging_simulation.launch.yaml`. Declare and
  forward `planning_profile` (default `stock`, unchanged behaviour).
- Create in `src/simulation/autosdv_twin/`:
  - `launch/twin_autoware.launch.xml`: includes AutoSDV's logging simulation with
    `vehicle_id:=twin`, `planning_profile:=rc`, `use_gnss:=false`, `camera_model:=none`,
    `use_sim_time:=true`, drivers off and the vehicle interface off, and its own args
    prefixed `twin_`.
  - `launch/twin.launch.py`: swb's `isaac.launch.xml` with `ego:=`, `world:=`,
    `ego_launch:="autosdv_twin twin_autoware.launch.xml"`, `vehicle:=autosdv_car_a`,
    `spec_hash_file:=` and `spawn:=x,y,z,yaw`.
  - `config/cyclonedds-unicast-lo.xml`
  - `config/routes.yaml`: one COSS route of 50–100 m along lanelets, start and goal in the
    map frame.
  - `test/test_twin_launch_args.py`
- Create: the recipes `just twin up` (world + Autoware, play_launch, PGID recorded in
  `tmp/twin.pgid`), `just twin drive` (set the route and engage, as golfcart's `twin_go.py`
  does) and `just twin down` (kill that PGID).

- [x] **Failing test.** `test_twin_launch_args`: parse `twin_autoware.launch.xml` and assert
  that it passes `vehicle_id=twin`, `planning_profile=rc`, `camera_model=none` and
  `use_gnss=false` to the logging simulation, and that the logging simulation declares
  `planning_profile`.
- [x] **Run, implement, run.** Expected: PASS.
- [x] **`just twin up`.** Expected within 3 min (TensorRT 10.8 runtime first, per
  `scripts/trt-runtime-env.sh`):
  - no node from the ZED driver, the Velodyne driver, the PCA9685 or the hall sensor;
  - `/sensing/lidar/...` (the spec topic), `/clock` and `/vehicle/status/velocity_status`
    ticking;
  - no process died in `play_log/latest`.
- [x] **Localization.** Initial pose from `config/routes.yaml` start. Expected: NDT converges
  within 30 s, NVTL ≥ 2.5 on the route (report it with
  `scripts/testing/localization/ndt_quality_report.py`).
- [x] **`just twin drive`.** Expected: the route completes. Max speed ≤ 2.0 m/s, no AEB
  stop on shrubs. Record a bag of the drive in `rosbags/twin/`.
- [x] **Standing obstacle.** Spawn a standing pedestrian 4 m ahead on the route (the swb
  runner or the world server's spawn) and record whether it is detected and whether the car
  stops. Report the result even if it fails: it is a finding, not a gate.
- [x] **`just twin down`.** Expected: no orphan nodes (`ros2 node list` empty).
- [x] **Commit and push.**

### Task 9: Guide and wrap-up

**Files:**
- Modify: `docs/guides/isaac-twin.md`. Cover install, `just twin generate|check|vehicle-check|world|up|drive|down`,
  the twin instance, what is estimated, the measured results (vehicle-check, NVTL, route),
  and the known gaps.
- Modify: `CLAUDE.md`. Add one paragraph under Simulation, listing the `twin` recipe
  module, the `twin` instance and a pointer to the guide.
- Modify: this file. Mark the tasks done and record the measured numbers.

- [x] **Run every check once more** (`just twin check`, pytest), and confirm `git status`
  is clean in AutoSDV and swb.
- [x] **Commit and push.**
