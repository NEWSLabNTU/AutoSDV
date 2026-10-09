"""virtual_coach: a scripted coach board for the planning simulator.

Stands in for the LiDAR board detector. It reads the simulator's true ego pose
from /localization/kinematic_state, moves a board along the ego's start
heading per the script (coach_script.CoachScript), and publishes what the
detector would: /perception/coach/board, a PoseStamped in base_link stamped
with the sample time, after `latency`, with noise and dropouts
(coach_script.DetectorModel).

Script time starts at engage (operation mode AUTONOMOUS) by default, so the
time it takes to bring the stack up and engage does not eat the static phase.
Until then the board sits `initial_distance` ahead of the ego, wherever the ego
is.

Also publishes the ground truth, for logging and grading:
  ~/truth        PoseStamped  board in the odometry frame
"""

import collections
import math

import rclpy
from autoware_adapi_v1_msgs.msg import OperationModeState
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from autosdv_coach_lab.coach_script import CoachScript, DetectorModel, world_to_base_link
from autosdv_coach_lab.pursuit import yaw_from_quaternion


class VirtualCoach(Node):
    def __init__(self):
        super().__init__('virtual_coach')
        dflt_s = CoachScript()
        dflt_d = DetectorModel()
        self.declare_parameter('odom_topic', '/localization/kinematic_state')
        self.declare_parameter('output_topic', '/perception/coach/board')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('start_on_engage', True)
        self.declare_parameter('operation_mode_topic', '/api/operation_mode/state')
        self.declare_parameter('initial_distance', dflt_s.initial_distance)
        self.declare_parameter('lateral_offset', dflt_s.lateral_offset)
        self.declare_parameter('segment_durations', dflt_s.durations)
        self.declare_parameter('segment_speeds', dflt_s.speeds)
        self.declare_parameter('accel', 1.0)
        self.declare_parameter('noise_std', dflt_d.noise_std)
        self.declare_parameter('latency', dflt_d.latency)
        self.declare_parameter('dropout_prob', dflt_d.dropout_prob)
        # [start, end, ...] in script seconds. A ROS double array may not be
        # empty in a param file, so [-1.0, -1.0] means "none".
        self.declare_parameter('dropout_windows', [-1.0, -1.0])
        self.declare_parameter('max_range', dflt_d.max_range)
        self.declare_parameter('fov_deg', math.degrees(dflt_d.fov))
        self.declare_parameter('seed', -1)
        gp = lambda k: self.get_parameter(k).value  # noqa: E731

        self.script = CoachScript(
            initial_distance=float(gp('initial_distance')),
            lateral_offset=float(gp('lateral_offset')),
            durations=[float(x) for x in gp('segment_durations')],
            speeds=[float(x) for x in gp('segment_speeds')],
            accel=float(gp('accel')))
        seed = int(gp('seed'))
        self.detector = DetectorModel(
            noise_std=float(gp('noise_std')), latency=float(gp('latency')),
            dropout_prob=float(gp('dropout_prob')),
            dropout_windows=[float(x) for x in gp('dropout_windows')],
            max_range=float(gp('max_range')), fov=math.radians(float(gp('fov_deg'))),
            seed=None if seed < 0 else seed)
        self.frame_id = gp('frame_id')
        self.start_on_engage = bool(gp('start_on_engage'))

        self.odom = None
        self.anchor = None      # (x, y, yaw) of the ego when the script started
        self.t0 = None          # node-clock seconds at script start
        self.queue = collections.deque()

        self.pub = self.create_publisher(PoseStamped, gp('output_topic'), 10)
        self.pub_truth = self.create_publisher(PoseStamped, '~/truth', 10)
        self.create_subscription(Odometry, gp('odom_topic'), self.on_odom, 1)
        mode_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            OperationModeState, gp('operation_mode_topic'), self.on_mode, mode_qos)
        self.create_timer(1.0 / float(gp('rate')), self.on_timer)
        self.get_logger().info(
            f'script: {self.script.initial_distance} m ahead, segments '
            f'{list(zip(self.script.durations, self.script.speeds))}, dropouts '
            f'{self.detector.dropout_windows}')

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def ego(self):
        p = self.odom.pose.pose
        q = p.orientation
        return p.position.x, p.position.y, yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def start(self):
        if self.t0 is None and self.odom is not None:
            self.anchor = self.ego()
            self.t0 = self.now_sec()
            self.get_logger().info('script started')

    def on_odom(self, msg):
        self.odom = msg
        if not self.start_on_engage:
            self.start()

    def on_mode(self, msg):
        if msg.mode == OperationModeState.AUTONOMOUS:
            self.start()

    def on_timer(self):
        if self.odom is None:
            return
        now = self.now_sec()
        ex, ey, eyaw = self.ego()
        if self.t0 is None:
            t_script = 0.0
            anchor = (ex, ey, eyaw)
        else:
            t_script = now - self.t0
            anchor = self.anchor
        bx, by = self.script.board_world(t_script, anchor)

        truth = PoseStamped()
        truth.header.stamp = self.get_clock().now().to_msg()
        truth.header.frame_id = self.odom.header.frame_id or 'map'
        truth.pose.position.x, truth.pose.position.y = bx, by
        truth.pose.position.z = self.odom.pose.pose.position.z
        truth.pose.orientation.w = 1.0
        self.pub_truth.publish(truth)

        rx, ry = world_to_base_link(bx, by, ex, ey, eyaw)
        det = self.detector.sample(t_script, rx, ry)
        if det is not None:
            self.queue.append((self.get_clock().now(), det))
        lat_ns = int(self.detector.latency * 1e9)
        now_t = self.get_clock().now()
        while self.queue and (now_t - self.queue[0][0]).nanoseconds >= lat_ns:
            t_sample, (x, y) = self.queue.popleft()
            msg = PoseStamped()
            msg.header.stamp = t_sample.to_msg()
            msg.header.frame_id = self.frame_id
            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.position.z = 1.0   # handheld board centre
            yaw = math.atan2(-y, -x)    # board face toward the vehicle
            msg.pose.orientation.z = math.sin(yaw / 2.0)
            msg.pose.orientation.w = math.cos(yaw / 2.0)
            self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VirtualCoach()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
