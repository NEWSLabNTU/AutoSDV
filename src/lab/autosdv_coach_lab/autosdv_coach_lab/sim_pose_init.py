"""sim_pose_init: place the simulated vehicle once, through the ADAPI.

The planning simulator does not move until /api/localization/initialize is
called. This node calls it with a configured pose, retrying until the stack is
up enough to accept it, and then exits. Simulator only.
"""

import math
import time

import rclpy
from autoware_adapi_v1_msgs.srv import InitializeLocalization
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


def main(args=None):
    rclpy.init(args=args)
    node = Node('sim_pose_init')
    for k, v in (('x', 0.0), ('y', 0.0), ('z', 0.0), ('yaw', 0.0),
                 ('frame_id', 'map'), ('retry_period', 3.0), ('give_up_after', 600.0)):
        node.declare_parameter(k, v)
    gp = lambda k: node.get_parameter(k).value  # noqa: E731
    cli = node.create_client(InitializeLocalization, '/api/localization/initialize')
    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = gp('frame_id')
    pose.pose.pose.position.x = float(gp('x'))
    pose.pose.pose.position.y = float(gp('y'))
    pose.pose.pose.position.z = float(gp('z'))
    yaw = float(gp('yaw'))
    pose.pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.pose.orientation.w = math.cos(yaw / 2.0)
    req = InitializeLocalization.Request()
    req.pose = [pose]
    ok = False
    # Retry by wall-clock deadline, sleeping between attempts. The API answers
    # "not success" immediately while localization is still coming up, so a
    # retry loop that does not sleep burns any attempt budget in milliseconds
    # (it did: 100 attempts in 30 ms, and the simulator never moved).
    deadline = time.monotonic() + float(gp('give_up_after'))
    attempt = 0
    try:
        while time.monotonic() < deadline:
            attempt += 1
            if cli.wait_for_service(timeout_sec=float(gp('retry_period'))):
                pose.header.stamp = node.get_clock().now().to_msg()
                fut = cli.call_async(req)
                rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
                res = fut.result()
                if res is not None and res.status.success:
                    node.get_logger().info(f'initialised after {attempt} attempt(s)')
                    ok = True
                    break
                msg = res.status.message if res is not None else 'timeout'
                node.get_logger().info(
                    f'attempt {attempt} refused ({msg!r}); retrying',
                    throttle_duration_sec=10.0)
            time.sleep(float(gp('retry_period')))
    except KeyboardInterrupt:
        pass
    if not ok:
        node.get_logger().error('gave up initialising the simulator pose')
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
