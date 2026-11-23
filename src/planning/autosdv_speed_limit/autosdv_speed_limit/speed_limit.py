"""The overall speed limit of the car, set in a config file: 3 km/h as shipped.

    speed_limit        the node autosdv.launch.yaml starts (use_speed_limit, default
                       true; parameter config_file, default config/speed_limit.yaml of
                       this package, autosdv.launch.yaml's speed_limit_config). It publishes
                       Autoware's external velocity limit on
                       /planning/scenario_planning/max_velocity_default from the file's
                       max_speed_kmh, and re-reads the file whenever it changes, so an
                       edit takes effect within a second, without a restart. A value not
                       above 0 or above 10 km/h is refused and the previous one kept.

The file is the one place the overall limit is set: 3 km/h at the start, raised later by
editing it, never above 10 km/h (the RC car; Lab 3's lanelet maximum too). The lanelets
carry the per-segment limits; Autoware drives at the lower of the two. RViz's Set Velocity Limit publishes on the same topic; the node
takes such a value over (clamped to 10) until the file next changes. Lab 3's run recording
keeps the limit in force (/planning/scenario_planning/max_velocity).
"""

import os
import sys
from typing import Optional, Tuple

DEFAULT_KMH = 3.0              # the shipped config/speed_limit.yaml
MAX_KMH = 10.0                 # never raise the RC car above it (Lab 3: the lanelets' maximum)
NODE = 'autosdv_speed_limit'
KEY = 'max_speed_kmh'
TOPIC = '/planning/scenario_planning/max_velocity_default'
SELECTED_TOPIC = '/planning/scenario_planning/max_velocity'
REPEAT_S = 1.0                 # s between repeats of the current limit (and file checks)


def kmh_to_mps(kmh: float) -> float:
    return kmh / 3.6


def refusal(kmh) -> Optional[str]:
    """Why `kmh` is not an acceptable limit, or None when it is."""
    if isinstance(kmh, bool) or not isinstance(kmh, (int, float)):
        return f'{kmh!r} is not a number'
    if not kmh > 0:
        return f'{kmh:g} km/h is not above 0'
    if kmh > MAX_KMH:
        return f'{kmh:g} km/h is above the {MAX_KMH:g} km/h maximum'
    return None


def read_config(path: str) -> Tuple[Optional[float], Optional[str]]:
    """(km/h, None) from the config file, or (None, why) when it is unusable."""
    import yaml
    try:
        with open(path, encoding='utf-8') as f:
            data = yaml.safe_load(f)
    except (OSError, yaml.YAMLError) as e:
        return None, f'cannot read {path}: {e}'
    if not isinstance(data, dict) or KEY not in data:
        return None, f'{path}: no {KEY}'
    kmh = data[KEY]
    why = refusal(kmh)
    if why:
        return None, f'{path}: {KEY}: {why}'
    return float(kmh), None


def adopted_kmh(max_velocity_mps: float) -> Optional[float]:
    """The limit to take over from another sender's message, km/h: clamped to MAX_KMH;
    None for a value that is no limit (not above 0)."""
    kmh = round(max_velocity_mps * 3.6, 2)
    if not kmh > 0:
        return None
    return min(kmh, MAX_KMH)


def velocity_limit(kmh: float):
    from autoware_internal_planning_msgs.msg import VelocityLimit
    msg = VelocityLimit()
    msg.max_velocity = float(kmh_to_mps(kmh))
    msg.use_constraints = False
    msg.sender = NODE
    return msg


def default_config() -> str:
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('autosdv_speed_limit'), 'config', 'speed_limit.yaml')


def _mtime(path: str) -> Optional[float]:
    try:
        return os.stat(path).st_mtime
    except OSError:
        return None


def main(argv=None) -> int:
    import rclpy
    from autoware_internal_planning_msgs.msg import VelocityLimit
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    rclpy.init(args=argv)
    node = Node(NODE)
    path = node.declare_parameter('config_file', '').value or default_config()
    log = node.get_logger()
    kmh, why = read_config(path)
    if why or kmh is None:
        log.error(f'{why}; using {DEFAULT_KMH:g} km/h')
        kmh = DEFAULT_KMH
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(VelocityLimit, TOPIC, latched)
    state = {'kmh': kmh, 'mtime': _mtime(path)}

    def publish(value: float, source: str):
        state['kmh'] = value
        pub.publish(velocity_limit(value))
        log.info(f'speed limit {value:g} km/h ({kmh_to_mps(value):.2f} m/s), from {source}')

    def tick():
        m = _mtime(path)
        if m != state['mtime']:
            state['mtime'] = m
            value, why = read_config(path)
            if why or value is None:
                log.error(f'{why}; keeping {state["kmh"]:g} km/h')
            else:
                publish(value, path)
                return
        # Autoware's velocity-limit selector subscribes without durability: a node that
        # starts after this one would never receive a latched message, so repeat it.
        pub.publish(velocity_limit(state['kmh']))

    def on_other(msg):
        # RViz's Set Velocity Limit on the same topic: take it over (clamped), so the
        # repeat does not undo it a second later.
        if msg.sender == NODE:
            return
        value = adopted_kmh(msg.max_velocity)
        if value is not None and value != state['kmh']:
            publish(value, msg.sender or 'another node')

    node.create_subscription(VelocityLimit, TOPIC, on_other, 10)
    node.create_timer(REPEAT_S, tick)
    publish(kmh, path)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
