# Simulation Guide

AutoSDV supports three simulation tiers for testing without a physical vehicle.

## Simulation Tiers

| Tier | Mode | What it tests | Requirements |
|------|------|---------------|-------------|
| 1 | Planning Simulation | Planning + control | Map only, no GPU |
| 2 | Logging Simulation | Full stack (rosbag replay) | Recorded data + GPU |
| 3 | CARLA Simulation | Full stack (live simulation) | CARLA 0.9.16 + GPU |

## Tier 1: Planning Simulation

The Autoware planning simulator runs the planning and control stack with the
AutoSDV vehicle model and COSS Park map. No sensors are involved.

```bash
just sim planning
```

Open [http://localhost:8081](http://localhost:8081). In RViz, set a 2D Pose
Estimate, then a 2D Goal Pose to watch the vehicle plan and follow a route.

**What it tests**: Mission planning, behavior planning, motion planning,
vehicle control.

## Tier 2: Logging Simulation (Rosbag Replay)

Replay recorded sensor data through the full Autoware stack.

```bash
just bag download                  # Download COSS Park recording (~2.8 GB)
just sim logging             # Start Autoware in sim mode
# In second terminal:
ros2 bag play data/rosbags/outdoor_20251226_153115/ --clock
```

**What it tests**: Sensing preprocessing, NDT localization, LiDAR perception,
planning, control — the same code that runs on the physical vehicle.

### Available test data

| Dataset | Size | Sensors | Download |
|---------|------|---------|----------|
| COSS Park outdoor | 2.8 GB | Velodyne VLP-32C, ZED IMU, u-blox GNSS | `just bag download` |
| Leo Drive Bus-ODD | 10.9 GB | VLP-16, VLP-32C x2, cameras, GNSS/INS | `cd scripts/leodrive-bus-launch && just setup` |

### Automated scenario: COSS Park

Run the full stack with automated rosbag playback and localization recording:

```bash
just sim coss-park
```

This launches three parallel processes: logging simulation, rosbag playback
(looped), and localization topic recording (60 seconds). Requires GNU
parallel (`sudo apt install parallel`).

### Monitoring tools

```bash
just tool plotjuggler   # Plot any ROS topic over time
just tool tui           # Terminal dashboard (pose, speed, states)
just tool rviz          # Additional RViz instance
```

## Tier 3: CARLA Simulation

CARLA provides a full 3D simulation environment with photorealistic rendering,
configurable sensors, traffic, and weather. The
[autoware_carla_bridge](https://github.com/jerry73204/autoware_carla_bridge)
connects CARLA 0.9.16 to Autoware 1.5.0 via a native Rust ROS 2 bridge.

### What it provides

- 3D LiDAR (128-channel ray cast), camera, IMU, GNSS
- GNSS auto-initializes localization (no manual pose needed)
- Pre-converted maps for CARLA Town01, 02, 03, 05, 10
- End-to-end autonomous driving demo
- Headless mode for CI

### Setup

The bridge is a separate repository:

```bash
cd ~/repos
git clone https://github.com/jerry73204/autoware_carla_bridge.git
cd autoware_carla_bridge
just setup          # Install deps, Autoware, CARLA maps
just build          # Build Rust bridge with colcon
just build-engines  # Pre-build TensorRT engines (first time, 2-5 min)
```

CARLA 0.9.16 must be installed separately. See the bridge README for details.

### Running the demo

```bash
# Terminal 1: Start CARLA server
just carla-start        # Starts as systemd service, ~30s

# Terminal 2: Launch full stack
just run-demo           # Autoware + bridge + scenario + pilot

# Monitor
just carla-status
just carla-logs
```

The demo loads Town01, spawns a vehicle, auto-initializes localization via
GNSS, sets a route, and drives autonomously. Open
[http://localhost:8080](http://localhost:8080) for the web UI.

### Architecture

```
CARLA 0.9.16          Rust Bridge (rclrs)        Autoware 1.5.0
┌─────────┐          ┌──────────────┐          ┌──────────────┐
│ Physics  │◄────────►│ Sensor data  │─────────►│ Sensing      │
│ Rendering│ TCP/IP   │ TF2 frames   │ ROS 2    │ Localization │
│ Traffic  │          │ Vehicle ctrl │◄─────────│ Perception   │
└─────────┘          │ Clock sync   │          │ Planning     │
                     └──────────────┘          │ Control      │
                                               └──────────────┘
```

Key design decisions:

- **One bridge per vehicle** — single Rust process manages one CARLA vehicle
- **Scenario script as ticker** — external Python script owns the simulation
  loop (`world.tick()`), bridge passively receives data
- **Standard Autoware topics** — publishes to `/sensing/*`, `/vehicle/*`,
  `/clock` directly, no remapping needed
- **Coordinate conversion** — CARLA left-handed → ROS right-handed (Y-flip
  on LiDAR points, poses, velocities)

### Map generation

Generate Autoware-compatible maps from any CARLA town:

```bash
cd ~/repos/autoware_carla_bridge
just generate-map /path/to/output    # Generates Lanelet2 + PCD
```

### Individual commands

```bash
just run-autoware   # Autoware only (planning simulator mode)
just run-bridge     # Bridge only
just run-scenario   # Scenario only (spawn vehicle, run tick loop)
just run-monitor    # Manual control GUI
just run-pilot      # Autonomous driving pilot
just run-sim        # Full stack without auto-drive (manual control)
```

## Choosing a Simulation Tier

| Question | Recommendation |
|----------|---------------|
| Just exploring planning? | Tier 1 — no data or GPU needed |
| Testing localization/perception changes? | Tier 2 — replay real data |
| Testing full autonomy in varied scenarios? | Tier 3 — CARLA |
| Running CI/regression tests? | Tier 2 (deterministic) or Tier 3 (headless) |
| No GPU available? | Tier 1 only |

## References

- [Autoware Planning Simulation](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/planning-simulation/)
- [Autoware Rosbag Replay](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/rosbag-replay-simulation/)
- [CARLA Documentation](https://carla.readthedocs.io/)
- [autoware_carla_bridge](https://github.com/jerry73204/autoware_carla_bridge)
