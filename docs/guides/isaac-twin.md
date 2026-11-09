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
