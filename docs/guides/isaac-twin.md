# Car A's digital twin on Isaac Sim

Car A (VLP-16 + ZED X Mini) in Isaac Sim 6.1, driven by AutoSDV's own Autoware through
scenario-world-bridge's Isaac vehicle bridge (aib). Design:
`docs/superpowers/specs/2026-11-10-car-a-isaac-twin-design.md`; plan and status:
`docs/roadmap/15-car-a-isaac-twin.md`.

## What it is made of

| piece | where |
|---|---|
| the twin car, `vehicle_id:=twin` | `individual_params/config/twin/`: car A's files, with sensor mounts estimated from photos |
| the vehicle spec | `src/simulation/autosdv_twin/vehicle/autosdv_car_a.yaml` |
| generated Isaac assets | `src/simulation/autosdv_twin/{isaac,catalogs}/` (`just twin generate`) |
| scenario-world-bridge | an external checkout, `SWB_ROOT` (default `~/repos/swb-twin`) |
| Isaac Sim 6.1 | a pip venv, `ISAAC_ROOT` (default `/mnt/p5plus/aeon/isaac-sim`) |

## Install (once per host)

Isaac Sim 6.1 is installed by scenario-world-bridge's script, into `$ISAAC_ROOT/venv`. Kit's
caches (`~/.cache/ov`, `~/.local/share/ov`, `~/.nvidia-omniverse`) are best symlinked under
`ISAAC_ROOT` first, so the shader cache stays off `/home`:

```bash
git clone --recurse-submodules https://github.com/NEWSLabNTU/scenario-world-bridge.git ~/repos/swb-twin
export ISAAC_ROOT=/mnt/p5plus/aeon/isaac-sim
bash ~/repos/swb-twin/src/swb_isaac_world/scripts/install_isaac.sh   # ~33 GB; isaacsim 6.1.0.0
cd ~/repos/swb-twin/src/autoware_isaac_bridge
just setup        # Autoware's messages rebuilt for Isaac's Python 3.12, into $ISAAC_ROOT/msgs
just build && just test && just test-isaac
```

On newslab-server133 it was already installed (2026-10-30), at `/mnt/p5plus/aeon/isaac-sim`.

### Two host settings every twin process needs

Both are in the twin's recipes; set them yourself only when running aib by hand.

- **DDS: unicast on loopback.** AutoSDV's `cyclonedds.xml` uses multicast on `lo` and asks for
  10 MB socket buffers. On a host without the `multicast-lo` service, or with
  `net.core.rmem_max` below 10 MB (it was 1 MB on server133), Isaac's Cyclone fails with
  `rmw_create_node: failed to create domain`. Every process of the twin, Isaac's and
  Autoware's, uses `src/simulation/autosdv_twin/config/cyclonedds-unicast-lo.xml` instead:

  ```bash
  export CYCLONEDDS_URI=file://$PWD/src/simulation/autosdv_twin/config/cyclonedds-unicast-lo.xml
  ```

- **CUDA: no foreign MPS daemon.** When another user runs `nvidia-cuda-mps-control` on the
  default pipe (`/tmp/nvidia-mps`), every CUDA client joins it and fails
  (`cudaErrorMpsConnectionFailed`). Point the client at an empty directory and it runs
  without MPS:

  ```bash
  mkdir -p tmp/no-mps
  export CUDA_MPS_PIPE_DIRECTORY=$PWD/tmp/no-mps CUDA_MPS_LOG_DIRECTORY=$PWD/tmp/no-mps
  ```

### Pick a free ROS domain

aib's default ego config (`src/aib/config/ego.yaml`, the Model 3) pins **ROS domain 1**,
which may carry someone else's Autoware on a shared host. Never run it there. The twin's
own ego config joins the terminal's `ROS_DOMAIN_ID`. Choose one nobody uses
(`ros2 topic list` on it shows only `/parameter_events` and `/rosout`), and keep it at 57 or
below for the unicast profile's port range.

## The twin car's files

```bash
just twin generate   # rig, SSv2 catalog, spec_hash, VLP-16 lidar USD, from the spec
just twin check      # fails on a stale generated file, or the spec and `twin` apart
```

Edit the spec (`vehicle/autosdv_car_a.yaml`) and the `twin` calibration together: the check
composes every sensor's pose in `base_link` from both and refuses a difference.

## Vehicle check on a flat plane

```bash
just twin vehicle-check   # aib's vehicle_check against the spec; writes tmp/twin_vc/<time>/report.json
```

The contract (topics, types, rates, frames, stamps, `/robot_description`) and the dynamics run
(acceleration and brake steps, a steering sweep against the bicycle model), with the speeds
scaled to the car's 2.04 m/s top speed. Result on server133: **PASS**, creep at rest
0.002 cm/s.

## The COSS world

```bash
just twin world            # all steps, into data/coss-isaac/ (gitignored)
just twin world report     # rerun from a step: color, ground, refill, smooth, meshes, usd, report
```

The last step, `report`, measures the lanes (swb's `world_report.py`) and fails the build on
a hole under a lanelet or p99 roughness above 1 cm (`scripts/world_gate.py`); a pit deeper
than the site fills only warns.

scenario-world-bridge's `scripts/ntu_twin` pipeline under its `coss` site: colours from the
NAS originals (the 2024 survey) onto `data/COSS-map-planning/pointcloud_map.pcd`, a 0.25 m
ground model under the lanelets, meshed tiles, and `map/isaac/world.yaml` beside the map's
own lanelets, projector and PCD (symlinks), so Autoware and Isaac read one map frame.

Measured: 12 tiles, 3.1 M triangles; lanes 100 % meshed, p99 roughness 2.6 mm; 60 % of the
points coloured within 1.5 m of an original (11 % within 6 cm: the map PCD is not the
coloured survey). COSS has no building classes, so only the ground collides. One real 9 cm
kerb step crosses a path near (-12.6, -14.5); routes avoid it.

## Driving it

```bash
just twin up      # Isaac on the COSS world + our Autoware, in the background; logs in tmp/twin_run/
just twin drive   # localize at the route's start, set the goal, engage, wait for arrival
just twin down    # stop both process groups, if they are still the ones `up` started
```

`up` takes a route and a world: `just twin up coss_loop data/coss-isaac/map/isaac/world.yaml`
(the defaults). Routes are in `src/simulation/autosdv_twin/config/routes.yaml`. Autoware is
AutoSDV's logging simulation as car A runs it (`twin_autoware.launch.xml`): `vehicle_id:=twin`,
`planning_profile:=rc`, VLP-16 and the ZED IMU, no GNSS, no drivers, no vehicle interface,
sim time. The domain is 47 unless `TWIN_DOMAIN` says otherwise.

Measured on `coss_loop` (server133, RTX 5090): NDT NVTL min 2.71, p50 2.99 over 2395 scans;
65.2 m in 38-40 s; no node left on the domain after `down`. `drive` latches Autoware's
speed cap at 2.0 m/s, as AutoSDV's drive tool does: planning then peaks at 2.000 m/s and the
car at 2.007. Without the cap Autoware planned up to 3.46 m/s and only the plant's motor
(2.04 m/s) held the car back.

Two settings in the recipes work around defects found here:

- **Autoware's NDT, not cuda_ndt.** cuda_ndt publishes its status as a bare
  `scan_matching_status`; Autoware's diagnostic graph waits for
  `ndt_scan_matcher: scan_matching_status`, so under cuda_ndt autonomous mode never becomes
  available. `twin_pose_source:=cuda_ndt` brings it back once that is fixed.
- **The newest CUDA for Autoware.** cuda_ndt compiles its kernels with NVRTC at start-up, and
  CUDA 12.6's NVRTC rejects an RTX 5090 (`invalid value for --gpu-architecture`). `twin env`
  puts the newest `/usr/local/cuda-X.Y/lib64` first (`TWIN_CUDA_LIB` overrides it).

## Known gaps

- **Perception sees no objects.** A 0.4 x 0.5 x 1.7 m box standing on the route (a world
  variant, `world_ped.usda`, sublayering the COSS world) gets 1.5-5 k lidar points and keeps
  100-750 through obstacle segmentation, yet perception emits no object in the whole run and
  planning never stops: the car drives into it. Every trial of obstacle handling on the twin
  starts here.
- **The lidar is not the VLP-16's timing.** aib publishes a full 360 deg sweep every 20 Hz
  frame (the spec says 10 Hz), about 43 k points against the VLP-16's 29 k, with every
  per-point `time_stamp` zero, so deskewing has nothing to correct.
- **Rates follow aib's 20 Hz frame**, not the hardware: the IMU at 20 Hz (the ZED's is about
  200 Hz), the camera at 10 Hz.
- **Estimated, not measured:** the sensor mounts (from photos; ZED +-2 cm, GNSS +-3 cm), the
  mass (5 kg), the motor (0.2 N m, 3000 rpm) and the drivetrain (all-wheel, 50/50).
- **The host's DDS drops datagrams.** With `net.core.rmem_max` at 1 MB, play_launch reports
  thousands of UDP drops a minute on the twin's domain. Raising it needs root.
