# Car A digital twin on Isaac Sim — design

Status: draft for approval, 2026-11-10.

## Intent

A digital twin of AutoSDV **car A** (1/10 RC chassis, Velodyne VLP-16, ZED X Mini,
GNSS puck, Jetson AGX Orin) in Isaac Sim, driven by **AutoSDV's own Autoware**
exactly as on the real car. Only the hardware is replaced. The purpose is a replica of
the real world: the same stack, the same parameters, a world built from a real scan, so
that what Autoware does in the twin predicts what it does on car A.

It is built on scenario-world-bridge (swb) and its Isaac vehicle bridge (aib), the same way
NEWSLab's golf cart twin was (`2026-golf-cart` `src/simulation/golfcart_twin`, spec
`newslab_golfcart`). That build is the reference for every step below. The swb sessions
that built it (swb 118: the golf cart twin; swb 133: the NTU world) were consulted.
Their answers are summarized under "Lessons carried over".

## Goals

1. **A twin car, `twin`, that mimics everything known about car A.** It is a new
   `individual_params` instance beside `car_a`, `car_b`, `car_c` and `sim`. It copies car A's
   preset (VLP-16 + ZED), actuator, speed-controller and IMU files. Its sensor calibration
   holds the photo-estimated mounts below.
2. **One vehicle spec**, `autosdv_car_a.yaml` (`spec_version: 2`, sim_vehicle_spec
   format):
   - geometry verbatim from `vehicle_info.param.yaml`;
   - sensor mounts verbatim from the `twin` calibration, so Autoware's URDF and Isaac's
     rig agree by construction;
   - mass, drive, tires and suspension as provenance-marked estimates.
3. **Isaac assets generated from the spec:**
   - the rig;
   - the SSv2 catalog entry and `spec_hash`;
   - a VLP-16 OmniLidar profile from Nebula's VLP16 factory calibration;
   - a low-poly visual body of car A, checked against the 14 photos;
   - `ego.yaml`.
4. **A COSS world** in Isaac, built from the COSS scan:
   - a textured ground mesh;
   - meshed objects;
   - colliders;
   - `world.yaml` aligned to the Autoware map frame.
5. **Our Autoware drives the twin.** AutoSDV's logging-simulation launch runs with:
   - drivers off and the PCA9685 interface off;
   - `use_sim_time`;
   - `vehicle_id:=twin`;
   - `camera_model:=none`.

   aib provides the sensors, `/vehicle/status/*` and `/clock`, and takes
   `/control/command/*` (the I5 contract).
6. **A real car-A model in `autosdv_vehicle_description`.** It replaces the box and the
   placeholder Lexus mesh for car A and `twin`, selected per `vehicle_id` (a mesh named in
   the instance's `individual_params`). car_b, car_c and sim keep today's box.

## Non-goals

- Changing `car_a`, `car_b`, `car_c` or `sim`. The real cars' calibration is not touched.
  The photo estimates live in `twin` only. Lab 2 (due 2026-11-10) and the RC simulator are
  unaffected.
- Modelling the PCA9685 → ESC → motor PWM chain or the hall sensor. In the twin, Autoware's
  `control_cmd` is realised by aib's PhysX vehicle and sim_vehicle_ecu, as for every swb car.
  Lab 2's PWM/speed-PID behaviour is out of scope.
- An indoor world. There is no indoor scan yet. COSS is the first world; an indoor one
  follows when data exists.
- Multi-vehicle, SSv2 scenarios beyond a free drive and a route.

## Anti-goals

- **A twin whose spec and Autoware URDF disagree.** Every mount exists in one file per side,
  and the two are compared at build time.
- **A world that does not match the PCD map Autoware localizes against.** The world mesh and
  the map PCD must come from the same points (swb 133 lesson: NVTL fell below 2.3 when the
  geometry diverged).
- **Copying swb code into AutoSDV.** Generators and the world pipeline are run from swb (and
  changed there when they do not fit); only their outputs and the twin's own parameters are
  committed here.

## Car A, as known

### Geometry

From `autosdv_vehicle_description/config/vehicle_info.param.yaml`, verbatim:

| key | value |
|---|---|
| wheel_base | 0.319 |
| wheel_tread | 0.263 |
| wheel_radius | 0.052 |
| wheel_width | 0.049 |
| front_overhang | 0.043 |
| rear_overhang | 0.059 |
| left_overhang | 0.035 |
| right_overhang | 0.035 |
| vehicle_height | 0.262 |
| max_steer_angle | 0.349 |

The photos measure a wheel radius of 0.054–0.056, a front overhang of about 0.053 (the
tire) and a height of 0.39 to the top of the lidar. `vehicle_info` stays as it is, so
Autoware and the twin agree. The differences are recorded in provenance.

### Sensor mounts

Estimated from photos 78ce8998, 23aca77f, c1aff61f and 59a9c2e8. Scale comes from the
0.319 m wheelbase and the VLP-16's 103 × 72 mm housing, with a perspective correction
between the wheel plane and the centreline.

`base_link` is on the ground at the rear-axle centre. `sensor_kit_base_link` is put at the
VLP-16, as the golf cart does with its VLP-32C:

| link | base_link → link (x, y, z) | in kit frame | ± |
|---|---|---|---|
| sensor_kit_base_link = vlp16 optical centre | 0.066, 0.00, 0.36 | 0, 0, 0 | 1.5 cm |
| zedxm_camera_link (centre) | 0.32, 0.00, 0.14 | 0.254, 0.00, −0.22 | 2 cm |
| gnss_base_link (top of puck) | −0.03, −0.07, 0.26 | −0.096, −0.07, −0.10 | 3 cm |
| imu_link (not visible; mid-chassis) | 0.16, 0.00, 0.08 | 0.094, 0.00, −0.28 | assumed |

- Every roll, pitch and yaw is 0. car_a's VLP-16 yaw of −0.221 was derived from replay, not
  from the mount, and the photos show none.
- The IMU source in the twin is the one aib publishes, at `imu_link`.

### Mass and drive (assumed)

- **Total mass: 5.0 kg.** Chassis 1.5, VLP-16 0.83, AGX Orin 1.4, LiPo 0.4, plates 0.6,
  the rest 0.3.
- **Centre of mass:** height 0.15 m, 45 % on the front axle.
- **Planning profile and top speed:** the twin runs `planning_profile:=rc`
  (`config/planning_rc/`), as car A does in Lab 3. `logging_simulation.launch.yaml` does not
  forward `planning_profile` today, so the twin's launch must pass it. The motor's ceiling
  (`motor_max_rpm`) is set at 2.0 m/s (assumed: car A's ESC can go faster, but nothing above
  that is a speed Autoware is run at on it), with a latched speed limit below it.
- **Tires, suspension, brakes and PhysX parameters:** scaled from the golf cart and Model 3
  per kg and per metre. See the risks below.

## The world: COSS

### Sources

| data | where | points | colour | geometry |
|---|---|---|---|---|
| originals | NAS `autoveh/dataset/2024-04 COSS Map HDMap/點雲/pt0{1,2,3}_TWD97.pcd` | 133 M | real RGB + intensity | float32 TWD97: northing quantized to 0.25 m, easting to 0.03 m (verified) |
| map | `data/COSS-map-planning/pointcloud_map.pcd` (= NAS `20251226/output_downsampled_edited_01.pcd`) | 4.9 M | none (false colour) | precise, local map frame, 130 × 75 × 22 m |
| lanelet2 | `data/COSS-map-planning/lanelet2_map.osm` | | | map frame |
| projector | `map_projector_info.yaml`: TransverseMercator, origin 25.0201, 121.5423 | | | |

### Pipeline

swb's NTU pipeline (`scripts/ntu_twin/`, roadmap 059), parameterized for COSS instead of
copied:

1. **Colour transfer.**
   - Register the originals to the map cloud: estimate the TWD97 → map offset, then ICP.
   - Then give each map point the nearest original's RGB, searching anisotropically
     (±0.15 m in northing, tight in easting and z).
   - Lane paint smearing north–south by up to ~10 cm is expected and accepted.
2. **Ground.**
   - SMRF-like ground filter, then a DTM on a **5 cm** grid under paths, smoothed:
     non-ground capped at ground + 5 cm, 5 × 5 median, then Gaussian.
   - Roughness target: p99 |z − 3×3 median| ≤ 1 cm under lanelets, because the wheel radius
     is 5 cm.
   - Close drain pits.
   - Mesh every cell under a lanelet.
   - Texture the ground with an orthophoto at 4–5 cm, splatting the originals' colours.
3. **Objects.**
   - Screened Poisson on the non-ground points, with vertex colours written linear and with
     normals.
   - Drop "other" triangles within a margin of lanelets (`--clear-road`).
   - Trunks get invisible cylinder colliders.
4. **USD.** `world_usd.py` with Isaac's Python writes one tile, `world.yaml` (map frame
   minus map_origin), sun and sky, colliders on ground and buildings only, and semantic
   labels.
5. **Same points for both.** The PCD that Autoware localizes against in the twin is
   `pointcloud_map.pcd` itself, the cloud the world was meshed from.

## Packages and layout

Proposed (builder may adjust within these boundaries):

```
src/param/autoware_individual_params/individual_params/config/twin/   # the twin car
  autosdv.preset.yaml               # car A's preset (vlp16 + zed)
  autosdv_vehicle/{actuator,speed_controller}.param.yaml  # car A's
  autosdv_sensor_kit/{sensors_calibration,sensor_kit_calibration,imu_corrector.param}.yaml
src/simulation/autosdv_twin/        # mirrors golfcart_twin
  vehicle/autosdv_car_a.yaml        # the spec
  isaac/{ego.yaml, autosdv_car_a.rig.yaml, spec_hash.yaml, lidars/VLP16.usda, make_lidars.py}
  isaac/body/vehicle.newslab.autosdv_car_a/{*.usda, entry.json}
  body/{make_body.py, render_previews.py}
  catalogs/vehicles.xosc
  launch/{twin.launch.py, twin_autoware.launch.xml}
  config/{routes.yaml, cyclonedds-unicast-lo.xml}
  test/                             # spec <-> twin calibration consistency check
src/vehicle/autosdv_vehicle_launch/autosdv_vehicle_description/
  mesh/car_a.{dae|glb}, urdf/vehicle.xacro    # the real car-A model
just/twin.just                      # `just twin generate|check|world|up|drive|vehicle-check`
```

The world's assets (meshes and textures, likely hundreds of MB) do not go in git. They are
built by a recipe into `data/coss-isaac/`, and the scripts and parameters are committed.

## Validation, in order

1. `just twin check`:
   - spec ↔ `twin` calibration ↔ URDF compare;
   - aib.gen and sim_vehicle_spec.gen `--check` (generated files current).
2. aib's `just vehicle-check` on the empty pad: topics, frames and stamps, then
   acceleration, braking and steering steps against the spec.
3. Car on the COSS world: rests without creeping, drives over the paths without snagging,
   lidar returns from everything.
4. Autoware on the twin:
   - NDT converges (NVTL comparable to swb's 2.9–3.1) from an initial pose given in the map
     frame (aib runner `--map-pose`, or RViz). Car A runs `use_gnss: false`, so `twin` does
     too; the GNSS sensor stays in the spec with `isaac.enabled: false`;
   - a route along COSS lanelets completes.
5. Standing pedestrian or obstacle detection with the VLP-16, tested early. CenterPoint
   missed standing pedestrians in the NTU twin, and a VLP-16 is sparser.

## Lessons carried over (swb 118, swb 133)

- **Sensors the sim does not provide still start:** use `camera_model:=none` and an IMU
  source the sim publishes.
- **Launch include args are not scoped:** prefix the twin's own args (e.g. `twin_rviz`).
- **DDS:** a unicast loopback CycloneDDS profile, domain ≤ 57.
- **TensorRT** 10.8 runtime first on `LD_LIBRARY_PATH`, or CenterPoint rebuilds every start.
- **Speed cap:** a latched speed-limit node, not a hand-published cap.
- **world.yaml:** the world USD path relative to the map dir.
- **`map_projector_info.yaml`** must sit where aib looks (two levels above `world.yaml`).
  Verify aib's TransverseMercator handling with a non-zero origin before trusting GNSS init.
- **PCD:** a single `.pcd` avoids Autoware 1.5's integer-tile-corner metadata rule.

## Risks

1. **PhysX at 1/10 scale.** At 5 kg and a 0.32 m wheelbase, every tuned value (substeps,
   contact offsets, tire stiffness, sticky-tire damping, suspension) was set for 750–1900 kg.
   Expect a tuning loop under `vehicle-check`. swb 241 has the aib dynamics history; ask
   it before changing shared defaults, and change car-specific values in the spec.
2. **The ECU model is a road car's.** sim_vehicle_ecu's motor, brake and steering models
   don't represent an ESC + servo, and the spec format may lack fields for them. Where the
   spec cannot express car A, the gap is listed, not faked.
3. **Scale of COSS.** Paths and lanelets are sized for full-size vehicles. Planning margins
   come from `vehicle_info` and should scale, but lane widths are 10× the car.
4. **Ground roughness** at the 1 cm level under a 5 cm wheel.
5. **Shrub and vegetation blobs** at the VLP-16's 0.36 m height.

## Decided by the user

- Twin of car A; Autoware is AutoSDV's own, driving through aib like the golf cart.
- Sensor mounts estimated from the photos.
- A separate twin car (`twin`) that mimics everything known on car A; car_a is untouched.
- World: COSS for now (no indoor data), coloured from the NAS originals.
- **swb is ours to change.** The builder edits scenario-world-bridge and its submodules
  (sim_vehicle_spec, aib, sim_vehicle_ecu, ...) and pushes them to their `main`, innermost
  first. The NTU world scripts are parameterized for COSS there, and spec fields and
  PhysX/ECU changes for a 5 kg car are made there, not listed as gaps.
- **Host: this workstation (.133, RTX 5090).** Isaac Sim 6.1 gets installed here (pip
  venv at aib's `ISAAC_ROOT`, overridden to a path on this host). Isaac and our Autoware run
  on one machine over loopback DDS. The Orin's timing is not reproduced.
- **Fidelity this round: plausible.** The same stack drives it, `vehicle-check` passes and
  a COSS route completes. Comparison against recorded car A data is a later round.

## Calls made by the author (check these)

- The instance is named `twin`. The spec and catalog entry are `autosdv_car_a`, and the body
  is `vehicle.newslab.autosdv_car_a`.
- `sensor_kit_base_link` sits at the VLP-16 in `twin` (in car_a it sits at base_link).
- `vehicle_info` is kept as it is, despite the photos' 0.054 wheel radius.
- Mass 5.0 kg, centre of mass 0.15 m high and 45 % front.
- The twin package lives in AutoSDV (`src/simulation/autosdv_twin`), as golfcart_twin does in
  the golf cart repo, with swb as an external checkout (`~/repos/scenario-world-bridge`)
  rather than a submodule.
- World assets are built, not committed.
- Planning profile `rc`; motor ceiling 2.0 m/s.
- Initial pose from the map frame, no GNSS, as car A.
- The car-A mesh is chosen per `vehicle_id`; other cars unchanged.

## Left to the builder

- Body modelling detail: a procedural low-poly USD like `make_body.py`, checked with rendered
  previews against the photos.
- The format of the car-A mesh for `autosdv_vehicle_description` (it can be exported from the
  same procedural body).
- PhysX and ECU tuning values, within `vehicle-check` acceptance.
- The COSS pipeline's parameters (thresholds, Poisson depth, margins), within the roughness
  and NDT targets above.
