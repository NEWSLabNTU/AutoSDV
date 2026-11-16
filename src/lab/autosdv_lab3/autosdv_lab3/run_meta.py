"""The week-2 stack's own record of itself, for grading: `lab3_run_meta`.

Publishes a latched /lab3/run_meta (std_msgs/String, JSON) at launch, repeated every 5 s:

    {"lab": "lab3", "kind": "stack", "launch_time": <ROS time, s>, "map_dir": ...,
     "map_hash": ..., "speed_limits": {lanelet id: km/h}, "vehicle_id": ...,
     "autosdv": <git rev>, "lanelet2_map": <the lanelet2 file's text>}

Transient local, so a recorder started later still receives it. The grader reads
the lanelet2 map from it, so that a run bag alone grades the lap (lab3_grade): the loop
and its lanelets come from the file the stack loaded, not from a folder on the grading
machine; map_hash says which map that was.
"""

import json
import os
import sys

from autosdv_lab3 import mapdir

TOPIC = '/lab3/run_meta'
REPEAT_S = 5.0


def speed_limits(map_dir: str) -> dict:
    """{lanelet id: speed_limit tag} of the loaded lanelet file; {} when it is unreadable."""
    from autosdv_lab3.route import MapError, read_lanelets
    try:
        lanelets = read_lanelets(os.path.join(map_dir, mapdir.LANELET))
    except MapError:
        return {}
    return {str(lid): ll.tags.get('speed_limit') for lid, ll in sorted(lanelets.items())}


def lanelet_text(map_dir: str) -> str:
    try:
        with open(os.path.join(map_dir, mapdir.LANELET), encoding='utf-8') as f:
            return f.read()
    except OSError:
        return ''


def build(map_dir: str, vehicle_id: str, launch_time: float) -> dict:
    return {'lab': 'lab3', 'kind': 'stack', 'launch_time': launch_time,
            'map_dir': os.path.abspath(map_dir), 'map_hash': mapdir.folder_hash(map_dir),
            'speed_limits': speed_limits(map_dir),
            'vehicle_id': vehicle_id, 'autosdv': mapdir.git_rev(),
            'lanelet2_map': lanelet_text(map_dir)}


def main(argv=None) -> int:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    rclpy.init(args=argv)
    node = Node('lab3_run_meta')
    map_dir = node.declare_parameter('map_path', '').value
    vehicle_id = node.declare_parameter('vehicle_id', '').value
    if not map_dir or not os.path.isdir(map_dir):
        node.get_logger().error(f'map_path {map_dir!r} is not a folder')
        return 2
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(String, TOPIC, latched)
    meta = build(map_dir, vehicle_id, node.get_clock().now().nanoseconds * 1e-9)
    msg = String(data=json.dumps(meta))
    pub.publish(msg)
    # latched for a recorder that matches the durability, and repeated for one that does
    # not: the grader is started after the stack, and its bag must still hold this
    node.create_timer(REPEAT_S, lambda: pub.publish(msg))
    node.get_logger().info(f'map_hash {meta["map_hash"][:12]} ({meta["map_dir"]})')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
