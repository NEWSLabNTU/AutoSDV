# AutoDRIVE-Autoware vs AutoSDV: Architectural Comparison

This document compares two Autoware-based autonomous vehicle platforms for small-scale vehicles.

## Executive Summary

| Aspect            | AutoDRIVE-Autoware                         | AutoSDV                                   |
|-------------------|--------------------------------------------|-------------------------------------------|
| **Repository**    | github.com/Tinker-Twins/AutoDRIVE-Autoware | github.com/NCKU-AUV/AutoSDV               |
| **Approach**      | Full Autoware Universe fork                | Minimal overlay on pre-installed Autoware |
| **Package Count** | ~450 packages                              | ~41 packages                              |
| **Target**        | Digital twin / simulator                   | Real hardware deployment                  |
| **Vehicle Type**  | RoboRacer (1:10 RC)                        | Custom small-scale vehicle                |
| **Sensor Focus**  | 2D LiDAR, basic camera                     | 3D LiDAR, stereo camera, GNSS/RTK         |
| **Communication** | WebSocket bridge                           | Direct I2C/GPIO hardware                  |

---

## 1. Architecture Philosophy

### AutoDRIVE-Autoware
**Full Fork Approach**
- Includes complete Autoware Universe codebase in repository
- All 178+ local packages + 20 imported dependencies in `src/`
- Self-contained: no external Autoware installation required
- Uses Docker for deployment (autodriveecosystem/autodrive_autoware)

```
AutoDRIVE-Autoware/
├── common/           # 18 packages
├── control/          # 20 packages
├── localization/     # 8 packages
├── perception/       # 51 packages
├── planning/         # 25 packages
├── sensing/          # 14 packages
├── system/           # 27 packages
├── vehicle/          # 4 packages
├── launch/           # 12 tier4_*_launch packages
├── autodrive/        # Custom RoboRacer integration
└── src/              # vcs-imported dependencies
```

### AutoSDV
**Minimal Overlay Approach**
- Uses pre-installed Autoware at `/opt/autoware/1.5.0/`
- Only custom/modified packages in repository
- Smaller footprint, easier to maintain
- Version-controlled via `versions.yaml`

```
AutoSDV/
├── src/
│   ├── launcher/autosdv_launch/     # Custom launch & configs
│   ├── vehicle/                      # Vehicle interface
│   ├── sensor_kit/                   # Sensor configurations
│   ├── localization/                 # Isaac SLAM integration
│   ├── param/                        # Individual parameters
│   ├── system/                       # Web monitor, runtime
│   ├── sensor_component/external/    # Sensor drivers (submodules)
│   └── calibration/                  # Calibration tools
└── data/                             # Maps, models
```

---

## 2. Dependency Management

### AutoDRIVE-Autoware
Uses `.repos` files with `vcs import`:

**build_depends_humble.repos** (16 repos):
- autoware_msgs v1.10.0
- autoware_core v1.2.0
- autoware_utils v1.4.2
- tier4_autoware_msgs v0.41.0
- nebula v0.2.6 (LiDAR driver abstraction)
- agnocast v2.1.2 (ROS middleware)
- cuda_blackboard v0.2.0

**packages_above.repos** (4 repos):
- autoware_tools (main)
- autoware_launch (main)
- transport_drivers (custom fork)

### AutoSDV
Uses git submodules + system installation:

**External Sensor Drivers** (submodules in `src/sensor_component/external/`):
- autoware_zed (ZED camera integration)
- zed-ros2-wrapper
- seyond_ros_driver (Robin-W LiDAR)
- ros2_blickfeld_driver (Cube1 LiDAR)
- ros2_mpu9250_driver (IMU)
- ros-nmea-reader, gnss_locator

**Base System**:
- Autoware 1.5.0 pre-installed via setup script
- ROS 2 Humble from apt

---

## 3. Vehicle Interface

### AutoDRIVE-Autoware
**WebSocket Bridge Architecture**

```python
# autodrive_bridge.py - Socket.IO based communication
sio = socketio.Server(async_mode='gevent')

@sio.on('Bridge')
def bridge(sid, data):
    # Parse incoming sensor data
    autodrive.throttle = float(data["V1 Throttle"])
    autodrive.steering = float(data["V1 Steering"])
    autodrive.lidar_range_array = np.fromstring(
        gzip.decompress(base64.b64decode(data["V1 LIDAR Range Array"]))
    )

    # Publish to ROS 2
    publish_lidar_scan(autodrive.lidar_scan_rate, autodrive.lidar_range_array)

    # Send commands back
    sio.emit('Bridge', data={
        'V1 Throttle': str(autodrive.throttle_command),
        'V1 Steering': str(autodrive.steering_command)
    })
```

**Key Characteristics**:
- Designed for digital twin / simulator integration
- All sensor data flows through WebSocket
- Base64-encoded, gzip-compressed data transfer
- 2D LiDAR (1080 points, 270-degree scan)
- Fixed frame IDs (lidar, imu, front_camera)

### AutoSDV
**Direct Hardware Control**

```python
# actuator.py - PCA9685 I2C PWM control
class AckermannActuator(Node):
    def __init__(self):
        self.pwm_controller = PCA9685()
        self.motor_pid = AckermannPID(kp=0.8, ki=0.1, kd=0.05)

    def set_motor_pwm(self, pwm_value: int):
        # Direct I2C communication
        self.pwm_controller.set_pwm(self.motor_channel, 0, pwm_value)

    def control_callback(self, msg: Control):
        # Multi-mode control state machine
        if self.mode == MotorState.BRAKE_LOCKED:
            self._handle_emergency_brake()
        else:
            output = self.motor_pid.compute(current_velocity, target_velocity)
            self.set_motor_pwm(self._velocity_to_pwm(output))
```

**Key Characteristics**:
- Direct hardware control via I2C (PCA9685)
- Hall effect sensor for velocity feedback (GPIO)
- Multi-mode controller (Emergency Brake, Full Stop, Deadband Hold, Active Control)
- Custom PID with anti-windup protection
- Real-time velocity reporting from wheel encoders

---

## 4. Sensor Support

### AutoDRIVE-Autoware
| Sensor | Type | Resolution |
|--------|------|------------|
| LiDAR | 2D LaserScan | 1080 points @ 270° |
| Camera | RGB | 192x108 |
| IMU | 6-DOF | Standard |
| Encoders | Wheel | 2 channels |

**Simulated Environment**:
- Indoor Positioning System (IPS) for ground truth
- Race timing data (lap count, lap time)
- Collision detection

### AutoSDV
| Sensor | Models | Resolution |
|--------|--------|------------|
| LiDAR | Seyond Robin-W, Velodyne VLP-32C, Blickfeld Cube1 | 3D point clouds (32-128 channels) |
| Camera | ZED X Mini (stereo), USB cameras | HD stereo / 1080p |
| IMU | ZED IMU, MPU9250 | 9-DOF |
| GNSS | u-blox ZED-F9R, Septentrio, Garmin | RTK-capable |

**Real-World Deployment**:
- Outdoor localization (NDT + GNSS)
- Indoor localization (Isaac ROS Visual SLAM)
- Multi-sensor fusion support

---

## 5. Launch System

### AutoDRIVE-Autoware
Uses standard Autoware launch structure:

```xml
<!-- autoware.launch.xml (from autoware_launch) -->
<launch>
  <arg name="vehicle_model" default="sample_vehicle"/>
  <arg name="sensor_model" default="sample_sensor_kit"/>
  <arg name="perception_mode" default="lidar"/>

  <include file="tier4_sensing_component.launch.xml"/>
  <include file="tier4_localization_component.launch.xml"/>
  <include file="tier4_perception_component.launch.xml"/>
  <include file="tier4_planning_component.launch.xml"/>
  <include file="tier4_control_component.launch.xml"/>
</launch>
```

**Custom Integration**:
- `autodrive/vehicles/roboracer/` with bringup launches
- RViz configurations for RoboRacer visualization

### AutoSDV
Uses preset-based configuration system:

```yaml
# autosdv.launch.yaml
launch:
  - arg:
      name: perception_preset
      default: "lidar_only"

  - include:
      file: "config/perception/preset/$(var perception_preset)_preset.yaml"

  - include:
      file: "autosdv_autoware.launch.xml"
      if: "$(var launch_autoware)"
```

**Preset Examples**:
```bash
# Perception presets
just launch perception_preset:=lidar_only
just launch perception_preset:=camera_lidar_fusion

# Localization presets
just launch localization_preset:=default    # NDT + gyro
just launch localization_preset:=eagleye    # GNSS-based

# Sensor suites
just launch sensor_suite:=robin_zed
just launch sensor_suite:=vlp32c_zed_imu
```

---

## 6. Configuration Approach

### AutoDRIVE-Autoware
- Uses Autoware's standard tier4 configurations
- Minimal custom configuration
- Focus on simulator/digital twin scenarios

### AutoSDV
**Hierarchical Configuration**:
```
config/
├── perception/
│   ├── preset/
│   │   ├── lidar_only_preset.yaml
│   │   ├── camera_lidar_fusion_preset.yaml
│   │   └── minimal_preset.yaml
│   ├── object_recognition/
│   ├── obstacle_segmentation/
│   └── traffic_light_arbiter/
├── localization/
│   ├── preset/
│   │   ├── default_preset.yaml
│   │   └── eagleye_preset.yaml
│   ├── ndt_scan_matcher/
│   └── yabloc/
├── control/
│   ├── trajectory_follower/
│   └── vehicle_cmd_gate/
└── planning/
    ├── mission_planning/
    └── scenario_planning/
```

---

## 7. Use Case Comparison

### AutoDRIVE-Autoware
**Best For**:
- Academic research and education
- Digital twin development
- Algorithm prototyping in simulation
- Scaled vehicle competitions (F1Tenth style)
- Testing Autoware without physical hardware

**Workflow**:
1. Start AutoDRIVE Simulator
2. Launch ROS 2 bridge (WebSocket)
3. Run Autoware stack
4. Iterate on algorithms in simulation

### AutoSDV
**Best For**:
- Real-world hardware deployment
- Outdoor autonomous navigation
- Production-grade sensor integration
- Multi-sensor fusion development
- Modifying/extending specific Autoware components

**Workflow**:
1. Power on vehicle hardware
2. Source Autoware: `source /opt/autoware/1.5.0/setup.bash`
3. Build overlays: `just build`
4. Launch: `just launch sensor_suite:=robin_zed`

---

## 8. Development Experience

### AutoDRIVE-Autoware

| Aspect | Details |
|--------|---------|
| **Setup** | Docker-based, self-contained |
| **Build Time** | Long (full Autoware compilation) |
| **Iteration** | Modify any Autoware component directly |
| **Updates** | Manual merge from upstream Autoware |
| **CI/CD** | Codecov integration, extensive testing |

### AutoSDV

| Aspect | Details |
|--------|---------|
| **Setup** | `./setup.sh` interactive installer |
| **Build Time** | Fast (only custom packages) |
| **Iteration** | Symlink-install for YAML/Python changes |
| **Updates** | Update base Autoware independently |
| **CI/CD** | Justfile recipes, modular testing |

---

## 9. Documentation

### AutoDRIVE-Autoware
- Installation guide (`autodrive/install.md`)
- YouTube tutorials playlist
- Academic papers with BibTeX citations
- Standard Autoware documentation applies

### AutoSDV
- MkDocs book (`book/`) with Material theme
- Comprehensive CLAUDE.md for AI-assisted development
- Sensor integration guides
- Vehicle calibration documentation
- Research notes (NDT tuning, Isaac SLAM testing)

---

## 10. Key Takeaways

### When to Choose AutoDRIVE-Autoware
1. You need a self-contained simulation environment
2. You want to modify core Autoware packages
3. You're doing academic research with digital twins
4. You need reproducible experiments with Docker
5. You're working with the RoboRacer platform specifically

### When to Choose AutoSDV
1. You're deploying on real hardware
2. You need specific 3D LiDAR or GNSS support
3. You want minimal maintenance burden
4. You need quick iteration on configuration
5. You want to track Autoware updates easily
6. You need production-grade system monitoring

---

## Appendix: Package Count Breakdown

### AutoDRIVE-Autoware (~450 packages)
```
perception/          51 packages
system/              27 packages
planning/            25 packages
control/             20 packages
common/              18 packages
sensing/             14 packages
launch/              12 packages
simulator/           9 packages
localization/        8 packages
evaluator/           8 packages
vehicle/             4 packages
vcs-imported (src/)  ~254 packages
```

### AutoSDV (~41 packages)
```
autosdv_launch              1
autosdv_vehicle_*           3
autosdv_sensor_kit_*        2
autosdv_system_*            2
autosdv_isaac_slam_launch   1
individual_params           1
control_test                1
calibration tools           19
sensor drivers (external)   7
misc                        4
```
