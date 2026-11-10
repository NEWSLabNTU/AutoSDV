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

### Two host settings the twin depends on

The recipes handle both; set them yourself only when running aib by hand.

- **DDS: the setup steps, or the unicast profile.** The twin uses the workspace's
  `CYCLONEDDS_URI`, as every AutoSDV process does. That profile uses multicast on `lo` and
  asks for 10 MB socket buffers, so it needs setup's `multicast-lo` and `cyclonedds-sysctl`
  steps; without them Isaac's Cyclone fails with `rmw_create_node: failed to create domain`.
  On server133 `cyclonedds-sysctl` had been overridden by the ZED SDK's
  `/etc/sysctl.d/60-zed-buffers.conf` (`rmem_max` 1 MB) until the step was re-run
  (2026-11-10); with it, the twin drives on the default profile with no UDP drops. On a host
  you cannot fix, point the twin at the unicast loopback profile, which needs neither:

  ```bash
  export TWIN_CYCLONEDDS_URI=file://$PWD/src/simulation/autosdv_twin/config/cyclonedds-unicast-lo.xml
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

The twin localizes with `cuda_ndt`, car A's matcher (`twin_pose_source:=ndt` runs
Autoware's). It needs a cuda_ndt_matcher from c2f979a on: older pins named their status
without the node prefix, which kept autonomous mode unavailable (known-config-defects §9).
Measured with it on `coss_loop`: engaged and arrived, 65.0 m, top speed 2.01 m/s.

cuda_ndt compiles its kernels with NVRTC at start-up, and CUDA 12.6's NVRTC rejects an RTX
5090 (`invalid value for --gpu-architecture`). From cuda_ndt_matcher 514db0b its build embeds
the newest toolkit that can compile for the GPU as an rpath when the default cannot, so no
setting is needed; rebuild it after installing a toolkit.

## The sensors as their drivers publish them

| sensor | real (car A) | twin |
|---|---|---|
| VLP-16 | 600 rpm, one message a revolution, 16 lasers 2.304 us apart every 55.296 us (at most 28,928 points), per-point `time_stamp`, the motion of the revolution in the points | the same: 10.0 Hz, about 21.7 k points a revolution on COSS (the sky returns none), 16 channels, times over 100 ms, motion distortion that Autoware's distortion corrector removes |
| ZED IMU | `sensors_pub_rate` 100 Hz (`zed_imu_minimal.yaml`; 99.9 Hz in a car bag) | 99.6 Hz, stamps exactly 10 ms apart, sampled from the physics substeps |

The lidar's timing is its profile, `isaac/make_lidars.py`: RTX's firing clock is
`patternFiringRateHz` (18,080, the VLP-16's 55.296 us cycle rounded to divide by 10 Hz).
How aib publishes it is the rig's `isaac:` block: `publish: scan|frame`, `point_times`,
`motion_distortion`, each defaulting to the real behaviour for the `xyzircaedt` point format.
The rates are the spec's `rate_hz`.

## Obstacles

A 0.4 x 0.5 x 1.7 m box standing 22 m along `coss_loop` (a world variant: a USD that
sublayers `world.usdc` and adds the box, `just twin up coss_loop <world_ped.yaml>`) is
tracked as an UNKNOWN object from about 20 m; obstacle_stop engages at 10 m and the car,
from 1.95 m/s, halts 6.0 m short of it. The plain route still runs clean: no objects, no
stop. Two perception settings made this work, and both apply to the real cars
(known-config-defects §11): obstacle segmentation now keeps points up to 1.26 m, not the
car's 0.262 m height, and the lidar-only preset merges the clustering's objects instead of
tracking CenterPoint's alone.

## Recording a proof run

```bash
just bag record-proof        # on car A
just twin record             # on the twin, while `just twin drive` runs
```

Both record `scripts/rosbag/proof_run_topics.txt`, so a car bag and a twin bag on the same
route hold the same topics.

## Known gaps

- **The lidar's channel numbers** rank the lasers by elevation (aib's `rings_by_elevation`);
  Nebula numbers them by laser id. Each laser keeps one channel either way.
- **The camera** renders every other 20 Hz frame (10 Hz), not the hardware's 15-60 Hz, and
  nothing on the twin reads it.
- **Estimated, not measured:** the sensor mounts (from photos; ZED +-2 cm, GNSS +-3 cm), the
  mass (5 kg), the motor (0.2 N m, 3000 rpm) and the drivetrain (all-wheel, 50/50).
