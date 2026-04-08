# CARLA Simulation Guide

This guide covers using the CARLA simulator with AutoSDV for vehicle physics
tuning and scenario configuration. For general simulation setup and the
Autoware bridge, see [Simulation Guide](./simulation_testing.md#tier-3-carla-simulation).

## Overview

[CARLA](https://carla.org/) is an open-source autonomous driving simulator
built on Unreal Engine. AutoSDV uses CARLA 0.9.16 via the
[autoware_carla_bridge](https://github.com/jerry73204/autoware_carla_bridge),
a native Rust ROS 2 bridge that provides full sensor simulation (3D LiDAR,
camera, IMU, GNSS) and end-to-end autonomous driving with Autoware 1.5.0.

## Setup

See [Simulation Guide — CARLA](./simulation_testing.md#tier-3-carla-simulation)
for installation and running the demo.

## Vehicle Profiles

CARLA provides two methods for vehicle customization:

1. **Runtime Physics Tuning** - Modify parameters via Python API (no rebuild)
2. **Full Custom Vehicle** - Add new 3D model in Unreal Engine

### Runtime Physics Tuning

Modify any existing vehicle's physics at runtime:

#### Basic Example

```python
import carla

# Connect to CARLA
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()

# Spawn a base vehicle
blueprint_library = world.get_blueprint_library()
vehicle_bp = blueprint_library.find('vehicle.tesla.model3')
spawn_point = world.get_map().get_spawn_points()[0]
vehicle = world.spawn_actor(vehicle_bp, spawn_point)

# Get current physics control
physics_control = vehicle.get_physics_control()

# Modify parameters
physics_control.mass = 1500.0  # kg
physics_control.max_rpm = 6000.0
physics_control.drag_coefficient = 0.3

# Apply changes
vehicle.apply_physics_control(physics_control)
```

#### Complete Vehicle Profile

```python
import carla

def create_autosdv_profile(vehicle):
    """
    Configure vehicle physics to match AutoSDV platform.

    AutoSDV specs (from vehicle_interface):
    - Small-scale autonomous vehicle
    - Max steering angle: ~20 degrees (0.349 rad)
    - PWM-based motor control
    """
    physics_control = vehicle.get_physics_control()

    # ================================================================
    # Engine Parameters
    # ================================================================

    # Torque curve: (RPM, Torque in Nm)
    # Adjust based on your motor characteristics
    physics_control.torque_curve = [
        carla.Vector2D(x=0, y=400),      # Torque at 0 RPM
        carla.Vector2D(x=1300, y=600),   # Peak torque
        carla.Vector2D(x=5000, y=400),   # Torque at max RPM
    ]
    physics_control.max_rpm = 6000.0
    physics_control.moi = 1.0  # Moment of inertia (kg*m^2)

    # ================================================================
    # Damping Rates
    # ================================================================

    physics_control.damping_rate_full_throttle = 0.15
    physics_control.damping_rate_zero_throttle_clutch_engaged = 2.0
    physics_control.damping_rate_zero_throttle_clutch_disengaged = 0.35

    # ================================================================
    # Transmission
    # ================================================================

    physics_control.use_gear_autobox = True
    physics_control.gear_switch_time = 0.5  # seconds
    physics_control.clutch_strength = 10.0
    physics_control.final_ratio = 4.0

    # Gear ratios
    physics_control.forward_gears = [
        carla.GearPhysicsControl(ratio=3.5, down_ratio=0.5, up_ratio=0.65),
        carla.GearPhysicsControl(ratio=2.0, down_ratio=0.5, up_ratio=0.65),
        carla.GearPhysicsControl(ratio=1.5, down_ratio=0.5, up_ratio=0.65),
        carla.GearPhysicsControl(ratio=1.0, down_ratio=0.5, up_ratio=0.65),
    ]

    # ================================================================
    # Vehicle Body
    # ================================================================

    physics_control.mass = 1500.0  # kg (adjust for your vehicle)
    physics_control.drag_coefficient = 0.3

    # Center of mass (relative to vehicle center, in cm)
    # Lower Z = more stable, positive X = forward
    physics_control.center_of_mass = carla.Vector3D(x=0.0, y=0.0, z=-0.5)

    # ================================================================
    # Steering Curve
    # ================================================================

    # (speed in km/h, steering multiplier)
    # Reduces steering sensitivity at high speed
    physics_control.steering_curve = [
        carla.Vector2D(x=0, y=1.0),    # Full steering at 0 km/h
        carla.Vector2D(x=30, y=0.7),   # 70% at 30 km/h
        carla.Vector2D(x=60, y=0.5),   # 50% at 60 km/h
        carla.Vector2D(x=120, y=0.3),  # 30% at 120 km/h
    ]

    # ================================================================
    # Wheel Physics
    # ================================================================

    # Front wheels (steering enabled)
    front_wheel = carla.WheelPhysicsControl(
        tire_friction=3.0,           # Friction coefficient (1.0-5.0)
        damping_rate=0.25,           # Wheel damping
        max_steer_angle=70.0,        # Max steering angle (degrees)
        radius=35.0,                 # Wheel radius (cm)
        max_brake_torque=1500.0,     # Max brake torque (Nm)
        max_handbrake_torque=0.0,    # No handbrake on front
        lat_stiff_max_load=2.0,      # Lateral stiffness max load
        lat_stiff_value=17.0,        # Lateral stiffness
        long_stiff_value=1000.0,     # Longitudinal stiffness
        position=carla.Vector3D(x=130, y=-75, z=30)  # FL position (cm)
    )

    # Rear wheels (no steering, handbrake enabled)
    rear_wheel = carla.WheelPhysicsControl(
        tire_friction=3.0,
        damping_rate=0.25,
        max_steer_angle=0.0,         # No steering on rear
        radius=35.0,
        max_brake_torque=1500.0,
        max_handbrake_torque=3000.0, # Handbrake on rear
        lat_stiff_max_load=2.0,
        lat_stiff_value=17.0,
        long_stiff_value=1000.0,
        position=carla.Vector3D(x=-130, y=-75, z=30)  # RL position (cm)
    )

    # Assign wheels: [Front-Left, Front-Right, Rear-Left, Rear-Right]
    physics_control.wheels = [
        front_wheel,
        carla.WheelPhysicsControl(  # Front-Right (mirror Y position)
            tire_friction=3.0, damping_rate=0.25, max_steer_angle=70.0,
            radius=35.0, max_brake_torque=1500.0, max_handbrake_torque=0.0,
            lat_stiff_max_load=2.0, lat_stiff_value=17.0, long_stiff_value=1000.0,
            position=carla.Vector3D(x=130, y=75, z=30)
        ),
        rear_wheel,
        carla.WheelPhysicsControl(  # Rear-Right (mirror Y position)
            tire_friction=3.0, damping_rate=0.25, max_steer_angle=0.0,
            radius=35.0, max_brake_torque=1500.0, max_handbrake_torque=3000.0,
            lat_stiff_max_load=2.0, lat_stiff_value=17.0, long_stiff_value=1000.0,
            position=carla.Vector3D(x=-130, y=75, z=30)
        ),
    ]

    # Apply all changes
    vehicle.apply_physics_control(physics_control)
    return physics_control
```

### Parameter Reference

#### VehiclePhysicsControl

| Parameter | Type | Description | Typical Range |
|-----------|------|-------------|---------------|
| `mass` | float | Vehicle mass (kg) | 800 - 3000 |
| `drag_coefficient` | float | Aerodynamic drag | 0.25 - 0.45 |
| `max_rpm` | float | Engine max RPM | 4000 - 8000 |
| `moi` | float | Moment of inertia (kg*m²) | 0.5 - 2.0 |
| `center_of_mass` | Vector3D | CoM position (cm) | varies |
| `torque_curve` | Vector2D[] | (RPM, Torque Nm) pairs | varies |
| `steering_curve` | Vector2D[] | (km/h, multiplier) pairs | varies |
| `use_gear_autobox` | bool | Automatic transmission | true/false |
| `gear_switch_time` | float | Gear change time (s) | 0.3 - 1.0 |
| `clutch_strength` | float | Clutch strength | 5 - 20 |

#### WheelPhysicsControl

| Parameter | Type | Description | Typical Range |
|-----------|------|-------------|---------------|
| `tire_friction` | float | Friction coefficient | 1.0 - 5.0 |
| `damping_rate` | float | Wheel damping | 0.1 - 1.0 |
| `max_steer_angle` | float | Max angle (degrees) | 0 - 70 |
| `radius` | float | Wheel radius (cm) | 25 - 50 |
| `max_brake_torque` | float | Brake torque (Nm) | 500 - 3000 |
| `max_handbrake_torque` | float | Handbrake torque (Nm) | 0 - 5000 |
| `lat_stiff_value` | float | Lateral stiffness | 10 - 25 |
| `long_stiff_value` | float | Longitudinal stiffness | 500 - 2000 |
| `position` | Vector3D | Wheel position (cm) | varies |

### Save/Load Vehicle Profiles

Create reusable JSON profiles:

```python
import json
import carla

def save_vehicle_profile(physics_control, filename):
    """Serialize VehiclePhysicsControl to JSON file."""
    profile = {
        'mass': physics_control.mass,
        'drag_coefficient': physics_control.drag_coefficient,
        'max_rpm': physics_control.max_rpm,
        'moi': physics_control.moi,
        'damping_rate_full_throttle': physics_control.damping_rate_full_throttle,
        'damping_rate_zero_throttle_clutch_engaged': physics_control.damping_rate_zero_throttle_clutch_engaged,
        'damping_rate_zero_throttle_clutch_disengaged': physics_control.damping_rate_zero_throttle_clutch_disengaged,
        'use_gear_autobox': physics_control.use_gear_autobox,
        'gear_switch_time': physics_control.gear_switch_time,
        'clutch_strength': physics_control.clutch_strength,
        'center_of_mass': {
            'x': physics_control.center_of_mass.x,
            'y': physics_control.center_of_mass.y,
            'z': physics_control.center_of_mass.z,
        },
        'torque_curve': [[p.x, p.y] for p in physics_control.torque_curve],
        'steering_curve': [[p.x, p.y] for p in physics_control.steering_curve],
        'wheels': [{
            'tire_friction': w.tire_friction,
            'damping_rate': w.damping_rate,
            'max_steer_angle': w.max_steer_angle,
            'radius': w.radius,
            'max_brake_torque': w.max_brake_torque,
            'max_handbrake_torque': w.max_handbrake_torque,
            'lat_stiff_max_load': w.lat_stiff_max_load,
            'lat_stiff_value': w.lat_stiff_value,
            'long_stiff_value': w.long_stiff_value,
            'position': {'x': w.position.x, 'y': w.position.y, 'z': w.position.z}
        } for w in physics_control.wheels]
    }

    with open(filename, 'w') as f:
        json.dump(profile, f, indent=2)
    print(f"Saved profile to {filename}")


def load_vehicle_profile(filename, vehicle):
    """Load JSON profile and apply to vehicle."""
    with open(filename, 'r') as f:
        profile = json.load(f)

    physics_control = vehicle.get_physics_control()

    # Apply scalar parameters
    physics_control.mass = profile['mass']
    physics_control.drag_coefficient = profile['drag_coefficient']
    physics_control.max_rpm = profile['max_rpm']
    physics_control.moi = profile['moi']
    physics_control.damping_rate_full_throttle = profile['damping_rate_full_throttle']
    physics_control.damping_rate_zero_throttle_clutch_engaged = profile['damping_rate_zero_throttle_clutch_engaged']
    physics_control.damping_rate_zero_throttle_clutch_disengaged = profile['damping_rate_zero_throttle_clutch_disengaged']
    physics_control.use_gear_autobox = profile['use_gear_autobox']
    physics_control.gear_switch_time = profile['gear_switch_time']
    physics_control.clutch_strength = profile['clutch_strength']

    # Apply center of mass
    com = profile['center_of_mass']
    physics_control.center_of_mass = carla.Vector3D(x=com['x'], y=com['y'], z=com['z'])

    # Apply curves
    physics_control.torque_curve = [
        carla.Vector2D(x=p[0], y=p[1]) for p in profile['torque_curve']
    ]
    physics_control.steering_curve = [
        carla.Vector2D(x=p[0], y=p[1]) for p in profile['steering_curve']
    ]

    # Apply wheel physics
    wheels = []
    for w in profile['wheels']:
        wheels.append(carla.WheelPhysicsControl(
            tire_friction=w['tire_friction'],
            damping_rate=w['damping_rate'],
            max_steer_angle=w['max_steer_angle'],
            radius=w['radius'],
            max_brake_torque=w['max_brake_torque'],
            max_handbrake_torque=w['max_handbrake_torque'],
            lat_stiff_max_load=w['lat_stiff_max_load'],
            lat_stiff_value=w['lat_stiff_value'],
            long_stiff_value=w['long_stiff_value'],
            position=carla.Vector3D(x=w['position']['x'], y=w['position']['y'], z=w['position']['z'])
        ))
    physics_control.wheels = wheels

    vehicle.apply_physics_control(physics_control)
    print(f"Loaded profile from {filename}")
    return physics_control


# Usage example
if __name__ == "__main__":
    client = carla.Client('localhost', 2000)
    world = client.get_world()

    # Spawn vehicle
    bp = world.get_blueprint_library().find('vehicle.tesla.model3')
    spawn = world.get_map().get_spawn_points()[0]
    vehicle = world.spawn_actor(bp, spawn)

    # Create and save profile
    physics = create_autosdv_profile(vehicle)
    save_vehicle_profile(physics, 'autosdv_profile.json')

    # Later: load profile
    # load_vehicle_profile('autosdv_profile.json', vehicle)
```

## Direct Vehicle Control

### VehicleControl API

Control vehicle directly each simulation tick:

```python
import carla
import time

client = carla.Client('localhost', 2000)
world = client.get_world()

# Spawn vehicle
bp = world.get_blueprint_library().find('vehicle.tesla.model3')
vehicle = world.spawn_actor(bp, world.get_map().get_spawn_points()[0])

# Control loop
try:
    while True:
        # Create control command
        control = carla.VehicleControl(
            throttle=0.5,       # 0.0 to 1.0
            steer=0.0,          # -1.0 (left) to 1.0 (right)
            brake=0.0,          # 0.0 to 1.0
            hand_brake=False,
            reverse=False,
            manual_gear_shift=False,
            gear=0
        )

        # Apply control
        vehicle.apply_control(control)

        # Get vehicle state
        transform = vehicle.get_transform()
        velocity = vehicle.get_velocity()
        speed_kmh = 3.6 * (velocity.x**2 + velocity.y**2 + velocity.z**2)**0.5

        print(f"Speed: {speed_kmh:.1f} km/h, Location: ({transform.location.x:.1f}, {transform.location.y:.1f})")

        time.sleep(0.05)  # 20 Hz control loop

finally:
    vehicle.destroy()
```

### Ackermann Control

For more realistic vehicle control using Ackermann steering:

```python
import carla

# Enable Ackermann control
vehicle.apply_ackermann_control(carla.VehicleAckermannControl(
    steer=0.1,           # Steering angle (radians)
    steer_speed=0.5,     # Steering rate (rad/s)
    speed=10.0,          # Target speed (m/s)
    acceleration=2.0,    # Acceleration (m/s²)
    jerk=1.0             # Jerk (m/s³)
))
```

## Multi-Vehicle Simulation

### Traffic Manager

Control multiple NPC vehicles:

```python
import carla
import random

client = carla.Client('localhost', 2000)
world = client.get_world()

# Get Traffic Manager
tm = client.get_trafficmanager(8000)
tm.set_synchronous_mode(True)

# Spawn multiple vehicles
blueprints = world.get_blueprint_library().filter('vehicle.*')
spawn_points = world.get_map().get_spawn_points()

vehicles = []
for i, spawn_point in enumerate(spawn_points[:20]):  # Spawn 20 vehicles
    bp = random.choice(blueprints)
    vehicle = world.try_spawn_actor(bp, spawn_point)
    if vehicle:
        vehicle.set_autopilot(True, tm.get_port())
        vehicles.append(vehicle)

# Configure Traffic Manager behavior
for v in vehicles:
    # Speed: percentage difference from speed limit (-50% to +50%)
    tm.vehicle_percentage_speed_difference(v, random.uniform(-20, 10))

    # Following distance
    tm.distance_to_leading_vehicle(v, random.uniform(3, 8))

    # Traffic light compliance (0-100% chance to ignore)
    tm.ignore_lights_percentage(v, 0)

    # Lane change behavior
    tm.random_left_lanechange_percentage(v, 10)
    tm.random_right_lanechange_percentage(v, 10)

    # Collision avoidance
    tm.auto_lane_change(v, True)

# Cleanup
for v in vehicles:
    v.destroy()
```

### Custom Vehicle Paths

Define specific routes for vehicles:

```python
import carla

client = carla.Client('localhost', 2000)
world = client.get_world()
map = world.get_map()

# Get waypoints for a route
spawn = map.get_spawn_points()[0]
start_wp = map.get_waypoint(spawn.location)

# Build route: follow road for 100m, then turn left
route = []
current_wp = start_wp
for _ in range(50):  # ~100m at 2m spacing
    route.append(current_wp)
    next_wps = current_wp.next(2.0)
    if next_wps:
        current_wp = next_wps[0]

# Take left turn at junction
if current_wp.is_junction:
    left_wps = current_wp.get_left_lane()
    if left_wps:
        current_wp = left_wps

# Continue route
for _ in range(25):
    route.append(current_wp)
    next_wps = current_wp.next(2.0)
    if next_wps:
        current_wp = next_wps[0]

# Spawn vehicle and set path
bp = world.get_blueprint_library().find('vehicle.tesla.model3')
vehicle = world.spawn_actor(bp, spawn)
vehicle.set_autopilot(True)

tm = client.get_trafficmanager()
tm.set_path(vehicle, [wp.transform.location for wp in route])
```

## OpenSCENARIO Scenarios

For complex multi-vehicle scenarios, use [ScenarioRunner](https://github.com/carla-simulator/scenario_runner):

### Example: Cut-In Scenario

```xml
<?xml version="1.0" encoding="UTF-8"?>
<OpenSCENARIO>
  <FileHeader description="Cut-in scenario" author="AutoSDV"/>

  <ParameterDeclarations>
    <ParameterDeclaration name="egoSpeed" parameterType="double" value="15.0"/>
    <ParameterDeclaration name="cutInDistance" parameterType="double" value="30.0"/>
  </ParameterDeclarations>

  <CatalogLocations/>

  <RoadNetwork>
    <LogicFile filepath="Town04"/>
  </RoadNetwork>

  <Entities>
    <ScenarioObject name="hero">
      <Vehicle name="vehicle.tesla.model3" vehicleCategory="car">
        <ParameterDeclarations/>
        <Performance maxSpeed="70" maxAcceleration="10" maxDeceleration="10"/>
        <BoundingBox>
          <Center x="1.5" y="0.0" z="0.9"/>
          <Dimensions width="2.1" height="1.8" length="4.5"/>
        </BoundingBox>
        <Axles>
          <FrontAxle maxSteering="0.5" wheelDiameter="0.6" trackWidth="1.8" positionX="3.1" positionZ="0.3"/>
          <RearAxle maxSteering="0.0" wheelDiameter="0.6" trackWidth="1.8" positionX="0.0" positionZ="0.3"/>
        </Axles>
      </Vehicle>
    </ScenarioObject>

    <ScenarioObject name="adversary">
      <Vehicle name="vehicle.audi.a2" vehicleCategory="car">
        <ParameterDeclarations/>
        <Performance maxSpeed="70" maxAcceleration="10" maxDeceleration="10"/>
        <BoundingBox>
          <Center x="1.5" y="0.0" z="0.9"/>
          <Dimensions width="2.1" height="1.8" length="4.5"/>
        </BoundingBox>
        <Axles>
          <FrontAxle maxSteering="0.5" wheelDiameter="0.6" trackWidth="1.8" positionX="3.1" positionZ="0.3"/>
          <RearAxle maxSteering="0.0" wheelDiameter="0.6" trackWidth="1.8" positionX="0.0" positionZ="0.3"/>
        </Axles>
      </Vehicle>
    </ScenarioObject>
  </Entities>

  <Storyboard>
    <Init>
      <Actions>
        <Private entityRef="hero">
          <PrivateAction>
            <TeleportAction>
              <Position>
                <WorldPosition x="100" y="200" z="0.5" h="1.57"/>
              </Position>
            </TeleportAction>
          </PrivateAction>
          <PrivateAction>
            <LongitudinalAction>
              <SpeedAction>
                <SpeedActionDynamics dynamicsShape="step" value="0" dynamicsDimension="time"/>
                <SpeedActionTarget>
                  <AbsoluteTargetSpeed value="$egoSpeed"/>
                </SpeedActionTarget>
              </SpeedAction>
            </LongitudinalAction>
          </PrivateAction>
        </Private>

        <Private entityRef="adversary">
          <PrivateAction>
            <TeleportAction>
              <Position>
                <RelativeRoadPosition entityRef="hero" ds="$cutInDistance" dt="3.5"/>
              </Position>
            </TeleportAction>
          </PrivateAction>
          <PrivateAction>
            <LongitudinalAction>
              <SpeedAction>
                <SpeedActionDynamics dynamicsShape="step" value="0" dynamicsDimension="time"/>
                <SpeedActionTarget>
                  <AbsoluteTargetSpeed value="$egoSpeed"/>
                </SpeedActionTarget>
              </SpeedAction>
            </LongitudinalAction>
          </PrivateAction>
        </Private>
      </Actions>
    </Init>

    <Story name="CutInStory">
      <Act name="CutInAct">
        <ManeuverGroup name="CutInManeuverGroup" maximumExecutionCount="1">
          <Actors selectTriggeringEntities="false">
            <EntityRef entityRef="adversary"/>
          </Actors>
          <Maneuver name="CutInManeuver">
            <Event name="CutInEvent" priority="overwrite">
              <Action name="CutInLaneChange">
                <PrivateAction>
                  <LateralAction>
                    <LaneChangeAction>
                      <LaneChangeActionDynamics dynamicsShape="sinusoidal" value="3" dynamicsDimension="time"/>
                      <LaneChangeTarget>
                        <RelativeTargetLane entityRef="hero" value="0"/>
                      </LaneChangeTarget>
                    </LaneChangeAction>
                  </LateralAction>
                </PrivateAction>
              </Action>
              <StartTrigger>
                <ConditionGroup>
                  <Condition name="CutInTrigger" delay="0" conditionEdge="rising">
                    <ByEntityCondition>
                      <TriggeringEntities triggeringEntitiesRule="any">
                        <EntityRef entityRef="hero"/>
                      </TriggeringEntities>
                      <EntityCondition>
                        <RelativeDistanceCondition entityRef="adversary"
                                                   relativeDistanceType="cartesianDistance"
                                                   value="20"
                                                   freespace="false"
                                                   rule="lessThan"/>
                      </EntityCondition>
                    </ByEntityCondition>
                  </Condition>
                </ConditionGroup>
              </StartTrigger>
            </Event>
          </Maneuver>
        </ManeuverGroup>
        <StartTrigger>
          <ConditionGroup>
            <Condition name="ActStart" delay="0" conditionEdge="rising">
              <ByValueCondition>
                <SimulationTimeCondition value="0" rule="greaterThan"/>
              </ByValueCondition>
            </Condition>
          </ConditionGroup>
        </StartTrigger>
      </Act>
    </Story>

    <StopTrigger/>
  </Storyboard>
</OpenSCENARIO>
```

### Running Scenarios

```bash
# Install ScenarioRunner
git clone https://github.com/carla-simulator/scenario_runner.git
cd scenario_runner
pip install -r requirements.txt

# Run scenario
python scenario_runner.py --scenario CutIn_1 --reloadWorld

# Or with OpenSCENARIO file
python scenario_runner.py --openscenario /path/to/cut_in.xosc
```

## Autoware Integration

### CARLA-Autoware Bridge

Connect CARLA to Autoware:

```bash
# Clone bridge
git clone https://github.com/carla-simulator/carla-autoware.git
cd carla-autoware

# Build
colcon build --symlink-install

# Source and run
source install/setup.bash
ros2 launch carla_autoware carla_autoware.launch.py
```

### Zenoh-Based Multi-Vehicle

For multiple Autoware instances with CARLA:

```bash
# See: https://autoware.org/running-multiple-autoware-powered-vehicles-in-carla-using-zenoh/
git clone https://github.com/evshary/autoware_carla_launch.git
```

## References

- [CARLA Documentation](https://carla.readthedocs.io/)
- [CARLA Python API](https://carla.readthedocs.io/en/latest/python_api/)
- [Vehicle Physics Control](https://carla.readthedocs.io/en/latest/tuto_G_control_vehicle_physics/)
- [Add Custom Vehicle](https://carla.readthedocs.io/en/latest/tuto_A_add_vehicle/)
- [Traffic Manager](https://carla.readthedocs.io/en/latest/adv_traffic_manager/)
- [ScenarioRunner](https://github.com/carla-simulator/scenario_runner)
- [OpenSCENARIO Support](https://scenario-runner.readthedocs.io/en/latest/openscenario_support/)
- [CARLA-Autoware Bridge](https://github.com/carla-simulator/carla-autoware)
