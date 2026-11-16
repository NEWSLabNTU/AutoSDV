"""The Lab 3 speed limit: 3 km/h unless the team raises it, explicitly.

Two entry points:

    lab3_speed_limit   the node lab3_run.launch.xml starts. It publishes Autoware's
                       default velocity limit, latched, on
                       /planning/scenario_planning/max_velocity_default, set from its
                       parameter max_speed_kmh (default 3). A parameter change
                       republishes it; a value above 10 km/h or not above 0 is refused.
    lab3_speed [KMH]   sets that parameter (`just lab3 speed 5`), or prints the current
                       limit without an argument.

The lanelets carry the upper bound (10 km/h); Autoware drives at the lower of the
lanelet's limit and this one. RViz's Set Velocity Limit publishes on the same topic, so
it is a second explicit way to change the speed; the run recording keeps the selected
limit (/planning/scenario_planning/max_velocity) either way.
"""

import argparse
import json
import sys
from typing import Optional

DEFAULT_KMH = 3.0
MAX_KMH = 10.0                 # the lanelets' speed limit; never raise the car above it
NODE = 'lab3_speed_limit'
PARAM = 'max_speed_kmh'
TOPIC = '/planning/scenario_planning/max_velocity_default'
SELECTED_TOPIC = '/planning/scenario_planning/max_velocity'
EVENTS_TOPIC = '/lab3/route_events'   # lab3_route's legs and arrivals (route.EVENTS_TOPIC)
REPEAT_S = 1.0                 # s between repeats of the current limit


LAP1_RULE = (f'the first lap runs at {DEFAULT_KMH:g} km/h: raise the speed once '
             f'just lab3 route has printed "lap 1: arrived"')


def lap_refusal(kmh: float, lap1_done: bool) -> Optional[str]:
    """Why `kmh` may not be set yet: above the default before the first lap is complete."""
    if not lap1_done and kmh > DEFAULT_KMH:
        return LAP1_RULE
    return None


def completes_lap1(event: dict) -> bool:
    """True for lab3_route's announcement that the last leg of lap 1 arrived."""
    return (event.get('event') == 'arrived' and event.get('lap') == 1
            and event.get('leg') is not None and event.get('leg') == event.get('legs'))


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


def main(argv=None) -> int:
    """The node: publish the limit, latched, and republish it on every accepted change."""
    import rclpy
    from autoware_internal_planning_msgs.msg import VelocityLimit
    from rcl_interfaces.msg import SetParametersResult
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    rclpy.init(args=argv)
    node = Node(NODE)
    kmh = float(node.declare_parameter(PARAM, DEFAULT_KMH).value or 0.0)
    why = refusal(kmh)
    if why:
        node.get_logger().error(f'{PARAM}: {why}')
        return 2
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(VelocityLimit, TOPIC, latched)

    current = {'kmh': kmh}
    state = {'lap1_done': False}

    def publish(value: float):
        current['kmh'] = value
        pub.publish(velocity_limit(value))
        node.get_logger().info(f'speed limit {value:g} km/h ({kmh_to_mps(value):.2f} m/s)')

    # Autoware's velocity-limit selector subscribes without durability, so a node that
    # starts after this one would never receive a latched message: repeat it every second.
    node.create_timer(REPEAT_S, lambda: pub.publish(velocity_limit(current['kmh'])))

    def on_set(params):
        for p in params:
            if p.name != PARAM:
                continue
            why = refusal(p.value) or lap_refusal(float(p.value), state['lap1_done'])
            if why:
                return SetParametersResult(successful=False, reason=why)
        for p in params:
            if p.name == PARAM:
                publish(float(p.value))
        return SetParametersResult(successful=True)

    node.add_on_set_parameters_callback(on_set)

    def on_other(msg):
        # Another sender on the same topic (RViz's Set Velocity Limit): adopt its value,
        # clamped to the maximum, so the repeat above does not undo it a second later.
        if msg.sender == NODE:
            return
        value = adopted_kmh(msg.max_velocity)
        if value is None:
            return
        if lap_refusal(value, state['lap1_done']):
            node.get_logger().warning(LAP1_RULE + f'; {value:g} km/h held at {DEFAULT_KMH:g}')
            value = DEFAULT_KMH
        node.get_logger().info(f'limit from {msg.sender or "another node"}: '
                               f'{msg.max_velocity * 3.6:.1f} km/h')
        from rclpy.parameter import Parameter
        node.set_parameters([Parameter(PARAM, Parameter.Type.DOUBLE, value)])

    node.create_subscription(VelocityLimit, TOPIC, on_other, 10)

    def on_event(msg):
        try:
            event = json.loads(msg.data)
        except ValueError:
            return
        if not state['lap1_done'] and completes_lap1(event):
            state['lap1_done'] = True
            node.get_logger().info('lap 1 arrived: the speed limit may now be raised')

    from std_msgs.msg import String
    node.create_subscription(String, EVENTS_TOPIC, on_event, QoSProfile(
        depth=100, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL))
    publish(kmh)
    from rclpy.executors import ExternalShutdownException
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


def cli(argv=None) -> int:
    """`lab3_speed [KMH]`: set the running stack's limit, or print it."""
    ap = argparse.ArgumentParser(
        prog='lab3_speed',
        description=f'Set the Lab 3 speed limit (default {DEFAULT_KMH:g} km/h, at most '
                    f'{MAX_KMH:g}) on the running stack, or print it.')
    ap.add_argument('kmh', nargs='?', type=float, help='the new limit, km/h')
    ap.add_argument('--timeout', type=float, default=5.0, help='s to wait for the stack')
    args = ap.parse_args(argv)
    if args.kmh is not None and (why := refusal(args.kmh)):
        print(f'error: {why}', file=sys.stderr)
        return 2

    import rclpy
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import GetParameters, SetParameters
    from rclpy.node import Node

    rclpy.init()
    node = Node('lab3_speed')
    try:
        if args.kmh is None:
            client = node.create_client(GetParameters, f'/{NODE}/get_parameters')
            req = GetParameters.Request(names=[PARAM])
        else:
            client = node.create_client(SetParameters, f'/{NODE}/set_parameters')
            value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(args.kmh))
            req = SetParameters.Request(parameters=[Parameter(name=PARAM, value=value)])
        if not client.wait_for_service(timeout_sec=args.timeout):
            print(f'error: no /{NODE}: is the Lab 3 stack (lab3_run.launch.xml) running?',
                  file=sys.stderr)
            return 1
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=args.timeout)
        res = future.result()
        if res is None:
            print(f'error: /{NODE} did not answer', file=sys.stderr)
            return 1
        if args.kmh is None:
            print(f'speed limit {res.values[0].double_value:g} km/h')
            return 0
        r = res.results[0]
        if not r.successful:
            print(f'error: refused: {r.reason}', file=sys.stderr)
            return 1
        print(f'speed limit {args.kmh:g} km/h ({kmh_to_mps(args.kmh):.2f} m/s)')
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
