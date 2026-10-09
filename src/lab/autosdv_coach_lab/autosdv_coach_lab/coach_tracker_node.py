"""coach_tracker: /perception/coach/board (PoseStamped, base_link)
-> /perception/coach/state (autosdv_lab_msgs/CoachState) at a fixed rate.

Reads /localization/kinematic_state to track the board in the odometry frame
(see tracker.py for why); without odometry it tracks in base_link.

The state is published whether or not the board is seen; `valid` carries the
difference, so a consumer that stops hearing from this node altogether (a
crash) is distinguishable from one that hears "no board".
"""

import math

import rclpy
from autosdv_lab_msgs.msg import CoachState
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from autosdv_coach_lab.pursuit import yaw_from_quaternion
from autosdv_coach_lab.tracker import IDENTITY, AlphaBetaTracker, PoseHistory, TrackerParams


def _stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class CoachTracker(Node):
    def __init__(self):
        super().__init__('coach_tracker')
        d = TrackerParams()
        self.declare_parameter('input_topic', '/perception/coach/board')
        self.declare_parameter('output_topic', '/perception/coach/state')
        self.declare_parameter('odom_topic', '/localization/kinematic_state')
        # Odometry older than this is not used; the tracker falls back to
        # base_link (and resets, since the frames differ).
        self.declare_parameter('odom_timeout', 0.5)
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('rate', 20.0)
        # A detection stamped further than this from the node clock is taken as
        # a clock mismatch (sim time vs wall time, an unset stamp) and filed at
        # the arrival time instead, with a warning.
        self.declare_parameter('max_stamp_skew', 1.0)
        self.declare_parameter('alpha', d.alpha)
        self.declare_parameter('beta', d.beta)
        self.declare_parameter('timeout', d.timeout)
        self.declare_parameter('gate', d.gate)
        self.declare_parameter('max_consecutive_rejects', d.max_consecutive_rejects)
        self.declare_parameter('max_board_speed', d.max_board_speed)
        self.declare_parameter('min_range', d.min_range)
        self.declare_parameter('max_range', d.max_range)

        gp = self.get_parameter
        self.frame_id = gp('frame_id').value
        self.max_skew = gp('max_stamp_skew').value
        self.tracker = AlphaBetaTracker(TrackerParams(
            alpha=gp('alpha').value, beta=gp('beta').value,
            timeout=gp('timeout').value, gate=gp('gate').value,
            max_consecutive_rejects=gp('max_consecutive_rejects').value,
            max_board_speed=gp('max_board_speed').value,
            min_range=gp('min_range').value, max_range=gp('max_range').value))

        self.pub = self.create_publisher(CoachState, gp('output_topic').value, 10)
        # sensor_data (best effort) matches a reliable publisher as well as a
        # best-effort one, so it is the safe choice for a detector we do not own.
        self.create_subscription(
            PoseStamped, gp('input_topic').value, self.on_board, qos_profile_sensor_data)
        self.create_subscription(Odometry, gp('odom_topic').value, self.on_odom, 10)
        self.odom_timeout = gp('odom_timeout').value
        self.history = PoseHistory(horizon=2.0)
        self.ego_speed = 0.0
        self.odom_rx = None
        self.use_odom = False
        self.create_timer(1.0 / gp('rate').value, self.on_timer)
        self.n_accepted = 0
        self.n_rejected = 0

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        p = msg.pose.pose.position
        t = _stamp_to_sec(msg.header.stamp)
        if abs(self.now_sec() - t) > self.max_skew:
            t = self.now_sec()
        self.history.add(t, p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))
        self.ego_speed = msg.twist.twist.linear.x
        self.odom_rx = self.now_sec()

    def odom_fresh(self) -> bool:
        return self.odom_rx is not None and self.now_sec() - self.odom_rx <= self.odom_timeout

    def check_frame(self):
        """Odometry appearing or vanishing changes the tracking frame."""
        fresh = self.odom_fresh()
        if fresh != self.use_odom:
            self.get_logger().info(
                'tracking in the odometry frame' if fresh else
                'no odometry: tracking in base_link')
            self.tracker.reset()
            self.use_odom = fresh

    def on_board(self, msg: PoseStamped):
        if msg.header.frame_id and msg.header.frame_id != self.frame_id:
            self.get_logger().error(
                f'board pose in frame {msg.header.frame_id!r}, expected '
                f'{self.frame_id!r}; dropped', throttle_duration_sec=5.0)
            return
        now = self.now_sec()
        t = _stamp_to_sec(msg.header.stamp)
        if abs(now - t) > self.max_skew:
            self.get_logger().warn(
                f'board stamp is {now - t:+.2f} s from the node clock; using '
                'arrival time (check use_sim_time)', throttle_duration_sec=5.0)
            t = now
        p = msg.pose.position
        self.check_frame()
        pose = self.history.at(t) if self.use_odom else IDENTITY
        if self.tracker.update(t, p.x, p.y, pose):
            self.n_accepted += 1
        else:
            self.n_rejected += 1

    def on_timer(self):
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9
        self.check_frame()
        if self.use_odom:
            s = self.tracker.predict(t, self.history.at(t), self.ego_speed)
        else:
            s = self.tracker.predict(t)
        msg = CoachState()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id
        msg.valid = s.valid
        msg.range = float(s.range)
        msg.range_rate = float(s.range_rate)
        msg.bearing = float(s.bearing)
        msg.age = float(s.age) if math.isfinite(s.age) else -1.0
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CoachTracker()
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
