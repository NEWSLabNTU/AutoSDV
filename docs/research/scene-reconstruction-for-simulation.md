# Scene reconstruction for closed-loop simulation

Research notes on getting a **drivable, sensor-producing scene** for AutoSDV
without a 3D artist: what to scan it with, what to convert it into, and which
simulator consumes the result.

Date surveyed: 2026-09-25.
Companion to [simulator_vehicle_modeling.md](simulator_vehicle_modeling.md),
which covers the *vehicle*; this file covers the *world*.

---

## 1. What is missing today

The three simulation tiers in `docs/guides/simulation_testing.md` do not
overlap in the way this project needs:

| Tier | Closed loop? | Sensors? | Scene source |
|---|---|---|---|
| Planning simulator | yes | no — kinematic ego, no LiDAR | lanelet2 only |
| Logging simulation | **no** — the bag does not react | real, recorded | the drive that was recorded |
| CARLA bridge | yes | simulated | CARLA towns, not our sites |

So perception and localization are only ever exercised open-loop, on a
recording, and the closed loop is only ever exercised with no sensors at all.
A scanned AutoSDV site, rendered as a scene a simulated LiDAR can hit, closes
that gap: NDT, the CUDA preprocessing chain, planning and control all run
against a world that responds to the vehicle's own actions.

## 2. Hardware gates the choice before anything else

**Isaac Sim cannot run on `arm-a100`.** Two independent reasons, both hard:

- Interactive rendering and the RTX LiDAR require dedicated **RT cores**. The
  A100 has none — it is a compute part.
- Isaac Sim's aarch64 builds are supported **only on DGX Spark**. Jetson
  (Orin, Thor) is explicitly unsupported for the same RT-core reason.

Isaac Sim 6.0 therefore runs on the x86 RTX workstation, Ubuntu 22.04, NVIDIA
driver 580+, ~45 GB of disk. `arm-a100` remains useful for the parts that are
pure compute: point-cloud meshing, reconstruction training, batch replay.

## 3. Scene sources, ranked for COSS Park

COSS Park is the first target, and it is the cheapest one, because **the scene
has already been scanned**: `data/COSS-map-planning/pointcloud_map.pcd` is a
75 MB metric LiDAR map of the site, with a lanelet2 map and a 0.05 m occupancy
grid beside it. Nothing needs to be captured to get started.

### Path 0 — extrude the occupancy grid (hours)

`occupancy_grid.pgm` at 0.05 m, origin `[-65, -25, 0]`, already marks what is
solid. Extruding occupied cells into boxes and laying a ground plane under them
produces a watertight OBJ in a few hundred lines of numpy + trimesh.

- Metrically exact, in the map frame, so NDT and the planner agree with it.
- Produces real LiDAR returns and real collisions.
- Looks like a Minecraft level, and flattens anything the z-band missed —
  the COSS grid was sliced at `z_band: [9.1, 9.4]`, so only structure crossing
  that 30 cm band exists at all.

Its value is not fidelity, it is that it makes every *other* piece of the
pipeline testable today: the bridge, the vehicle dynamics, the sensor graph,
the acceptance tests. Build it first and treat it as scaffolding.

**This one is built**: `scripts/sim/grid_to_mesh.py`.

```bash
python3 scripts/sim/grid_to_mesh.py data/COSS-map-planning \
    --floor-z 8.9 --wall-height 2.0 -o tmp/coss_scene.obj
```

COSS Park, measured: the 2601 x 1502 grid is 130.1 x 75.1 m; 82,915 occupied
cells merge along rows into 31,888 boxes, 382,658 triangles, 14.9 MB of OBJ, in
about twenty seconds. `--floor-z 8.9` comes from the point cloud itself, whose
1st-percentile z is 8.84 and 10th is 9.08 — the map frame sits about 9 m above
the sensor origin, which is also why the grid was sliced at `[9.1, 9.4]`.

The mesh was checked against the cloud rather than looked at: for 500 sampled
box centres, the nearest point in the 9.1–9.4 m band is a **median 0.026 m**
away (p95 0.033, max 0.050) — half a cell, which is what a correct
cell-to-world mapping gives. The same test on a y-mirrored copy gives median
0.526 m and p95 21.3 m. That asymmetry is the point: PGM row 0 is the *top* of
the image and therefore the *highest* y in the map frame, and getting it
backwards produces a scene that looks plausible and localizes nowhere.

Two known crudities, neither blocking: runs merge along x only (a column-wise
pass would cut the box count further), and every wall is the same height, so
overhangs and anything outside the z-band are absent.

### Path 1 — mesh the point-cloud map (days)

Segment ground from structure, then surface-reconstruct. Poisson (Open3D) is
the standard answer and is hyperparameter-sensitive — octree depth and the
vertex-density trim threshold both change the result visibly — so expect
iteration rather than one command. Ball-pivoting or a 2.5-D Delaunay ground
plus Poisson for the vertical structure is often steadier outdoors, where
vegetation defeats a single global fit.

This is the honest version of the scene, and it has a property no other path
has: **it is the same geometry NDT matches against**, so a simulated scan of
the mesh should localize against `pointcloud_map.pcd`. That turns the
acceptance test into a measurement rather than a screenshot (§6).

`arm-a100` can do this work; nothing here needs RT cores.

### Path 2 — photoreal skin from phone video (weeks, optional)

NVIDIA Omniverse **NuRec** with **3DGUT** ingests camera (and lidar) data and
packages the reconstruction as a **USDZ** — a USD scene plus a trained
checkpoint — which Isaac Sim 6.0 renders natively through the Fabric Scene
Delegate, and which CARLA also consumes. Capture is a phone video with ~60%
frame overlap and steady lighting.

Three limits decide how it is used, and they have not moved:

- **Splats are visual only.** No collision, no rigid-body presence. A proxy
  mesh must be shipped alongside — which is exactly what Path 1 produces.
- **Splats return no LiDAR.** Camera simulation only. (Research such as SiMUli
  extends 3DGUT toward LiDAR rendering; it is not in a shipping product.)
- **Scale drifts** against real dimensions. The mesh stays authoritative for
  geometry; the splat is a skin.

So Path 2 is worth doing when camera perception matters — traffic lights,
segmentation, the ZED path — and is dead weight for a LiDAR-only stack.

### Not available here: Scaniverse USDZ

Niantic added USDZ export to Scaniverse in July 2026, and it is the one capture
path that hands over a **splat and an aligned mesh together**, importable into
Isaac Sim directly. It needs the LiDAR sensor on an iPhone/iPad Pro, which this
project does not have. Worth revisiting if one appears — it collapses Paths 1
and 2 into a single 20-minute walk-around, and it is the right answer for
*indoor* scenes, where no prior PCD map exists.

## 4. Simulator: Isaac Sim, with CARLA as the fallback we already own

| | Isaac Sim 6.0 | CARLA 0.9.16 (bridge exists) |
|---|---|---|
| Import a scanned mesh | USD/OBJ, direct | FBX through the UE editor |
| Custom site as a map | a prim in a scene | needs OpenDRIVE + a packaged map |
| Vehicle from our URDF | importer, then PhysX Vehicle Wizard | skeletal-mesh rigging in Unreal |
| Autoware bridge | write it (reference below) | **done** — `jerry73204/autoware_carla_bridge` |
| NuRec splats | native | supported |

The bridge is the one column where CARLA wins, and `autowarefoundation/`
`autoware_off-road_sim` removes most of that advantage: it is Isaac Sim 6.0 +
ROS 2 Humble + Autoware, publishing `/ego/point_cloud`, `/ego/rgb`, `/ego/imu`,
`/ego/odom`, `/ego/gnss` and subscribing `autoware_control_msgs/Control` on
`/ego/control`, with a Docker build and a headless mode. It is a working
reference for exactly the wiring we would otherwise design from scratch.

Everything else favours Isaac: the scene is a mesh we generate, not a map we
author, and the vehicle is the URDF we already ship.

## 5. Vehicle and sensor wiring — the parts that will bite

- **The URDF is a box, and that is fine.** `vehicle.xacro` is a
  0.421 × 0.306 × 0.262 m box on `car_link`; the visual mesh in the tree
  (`mesh/lexus.dae`) is a full-size passenger car and must not be used for a
  0.319 m wheelbase vehicle. Physics comes from the PhysX Vehicle Wizard, sized
  from `vehicle_info.param.yaml`: `wheel_base` 0.319, `wheel_tread` 0.263,
  `wheel_radius` 0.052, `max_steer_angle` 0.349 rad.
- **Steering limits must match the planner's.** `max_steering_angle` in
  `actuator.yaml` and `max_steer_angle` in `vehicle_info.param.yaml` are already
  required to agree; the simulated vehicle is a third copy of that number.
- **The simulated cloud will not carry per-point time.** Isaac's RTX LiDAR
  publishes `sensor_msgs/PointCloud2`; AutoSDV's CUDA preprocessing requires
  `PointXYZIRCAEDT` and refuses anything without a per-point time offset (this
  is why `cube1` is refused on that path). So a simulated run is
  `pointcloud_backend:=cpu` until a field-adding node exists. Worth knowing
  before concluding the CUDA path is broken.
- **RTX LiDAR is configured by JSON**, rotating or solid-state, with a library
  of models loaded by name. A VLP-32C profile has to be checked for or written;
  ring count, FOV and rate must match `vlp32c` or the ring-based tooling
  (`scripts/sensor/inspect_rings.py`, MCL's ring extraction) measures a
  different sensor than the vehicle has.
- **GNSS is free in sim** and auto-initializes localization, as the CARLA
  bridge already demonstrates — no manual pose seeding in a CI run.

## 6. Acceptance tests, so "realistic" is a number

1. **NDT locks in sim.** Run the stack against `data/COSS-map-planning` with
   the simulated LiDAR feeding it. If the mesh is faithful, NDT converges and
   holds; report with `ndt_quality_report.py` and `ndt_alignment_report.py`.
   This is the single strongest test, because sim geometry and map geometry
   come from the same source and any meshing error shows up as residual.
2. **Scan statistics against the recording.** Park the simulated vehicle at a
   pose the COSS bag visits and compare per-ring range histograms and point
   counts with the real scan. Divergence localizes the problem to the mesh, the
   sensor profile, or the mounting.
3. **Closed-loop route.** Drive a lanelet2 route end to end, headless, and
   record the same localization topics `scripts/rosbag/record_localization.sh`
   records. That is the artifact CI can diff.

## 7. Proposed sequence

| Phase | Work | Where | Output |
|---|---|---|---|
| A | Grid-extrusion mesh of COSS, OBJ | `arm-a100` | **done** — `tmp/coss_scene.obj`, 383k tris |
| B | Isaac Sim 6.0 install; import scene; Wizard vehicle from `vehicle_info` | RTX box | vehicle drives with a keyboard |
| C | Sensor graph + ROS 2 bridge (LiDAR, IMU, GNSS, odom, control in) | RTX box | Autoware topics |
| D | Test 1: NDT against the real map | RTX box | pose-error report |
| E | Poisson/ground-split mesh from `pointcloud_map.pcd`, swap it in | `arm-a100` | fidelity, re-run tests 1–2 |
| F | Optional: phone video → NuRec/3DGUT splat over the Path 1 collision mesh | both | camera realism |

Phases A–D are the pilot: they answer whether a closed-loop AutoSDV runs at all
before any effort goes into making the world look right.

## 8. Open questions

- Does `tmp/coss_scene.obj` import into Isaac Sim 6.0 as-is, or does it want
  USD conversion and a collision-approximation choice per box?
- Which x86 RTX box, and does its driver clear 580? Isaac Sim 6.0 wants it.
- Is there a VLP-32C RTX LiDAR profile in the shipped library, or is one to be
  written from the datasheet?
- Does the COSS z-frame offset (`z_band: [9.1, 9.4]`, grid origin z 0) need an
  explicit transform when the mesh is placed, or does keeping everything in the
  map frame make it a non-issue?
- Does a per-point-time-stamping node belong in the sim bridge or in
  `cuda_pointcloud_filters` as a general adapter?

## References

- [Isaac Sim requirements (6.0)](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/installation/requirements.html)
- [Isaac Sim: RTX Lidar sensor](https://docs.isaacsim.omniverse.nvidia.com/latest/sensors/isaacsim_sensors_rtx_lidar.html)
- [Isaac Sim: publish RTX Lidar point cloud to ROS 2](https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/tutorial_ros2_rtx_lidar.html)
- [Isaac Sim: neural volume rendering (NuRec assets)](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/assets/usd_assets_nurec.html)
- [autowarefoundation/autoware_off-road_sim](https://github.com/autowarefoundation/autoware_off-road_sim)
- [Omniverse NuRec](https://developer.nvidia.com/omniverse/nurec) and [NuRec docs](https://docs.nvidia.com/nurec/index.html)
- [NVIDIA: reconstruct a scene in Isaac Sim using only a smartphone](https://developer.nvidia.com/blog/reconstruct-a-scene-in-nvidia-isaac-sim-using-only-a-smartphone/)
- [Isaac Sim discussion: collider mesh for gaussian splats](https://github.com/isaac-sim/IsaacSim/discussions/192)
- [Niantic Spatial: USDZ export in Scaniverse](https://www.nianticspatial.com/blog/usdz-scaniverse)
- [Open3D: point cloud surface reconstruction](https://www.open3d.org/docs/release/tutorial/geometry/pointcloud.html)
- [Rmagine: range-sensor simulation in polygonal maps](https://arxiv.org/pdf/2209.13397)
