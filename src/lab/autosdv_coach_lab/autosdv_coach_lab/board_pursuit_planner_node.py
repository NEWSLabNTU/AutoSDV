"""board_pursuit_planner: CoachState + odometry -> /planning/trajectory.

Replaces Autoware's planning stack for the coach-board lab. Every cycle it asks
the loaded CruiseController for a target speed, clamps it (pursuit.clamp_speed),
and publishes a fixed-length straight trajectory toward the board in the frame
of /localization/kinematic_state, with the stop encoded in velocity: a braking
ramp to zero at s_stop (pursuit.velocity_profile).

The controller is chosen by import path, `controller_class`, and receives every
`controller.*` parameter as a plain dict with the prefix removed. A controller
that raises is caught: the cycle commands a stop and the error is logged, so a
student bug stops the car instead of killing the planner (which would also stop
it, via MRM, but noisily and without saying why).
"""

import math

import rclpy
from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint
from autosdv_lab_msgs.msg import CoachState, PursuitDebug
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from autosdv_coach_lab import pursuit

DEFAULT_CONTROLLER = 'autosdv_coach_lab.reference.cruise_controller:CruiseController'


class BoardPursuitPlanner(Node):
    def __init__(self):
        super().__init__(
            'board_pursuit_planner',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True)
        d = pursuit.PlannerParams()
        defaults = {
            'controller_class': DEFAULT_CONTROLLER,
            'state_topic': '/perception/coach/state',
            'odom_topic': '/localization/kinematic_state',
            'trajectory_topic': '/planning/trajectory',
            'debug_topic': '/planning/coach_pursuit/debug',
            'operation_mode_topic': '/api/operation_mode/state',
            'rate': 10.0,
            # CoachState older than this (by arrival) counts as invalid: the
            # tracker itself has gone quiet.
            'state_timeout': 0.5,
            'odom_timeout': 0.5,
            'v_max': d.v_max,
            'accel_max': d.accel_max,
            'decel_max': d.decel_max,
            'brake_decel': d.brake_decel,
            'envelope_decel': d.envelope_decel,
            'reaction_time': d.reaction_time,
            'standoff_min': d.standoff_min,
            'path_length': d.path_length,
            'path_step': d.path_step,
        }
        for k, v in defaults.items():
            if not self.has_parameter(k):
                self.declare_parameter(k, v)
        gp = lambda k: self.get_parameter(k).value  # noqa: E731

        self.p = pursuit.PlannerParams(
            v_max=float(gp('v_max')), accel_max=float(gp('accel_max')),
            decel_max=float(gp('decel_max')), standoff_min=float(gp('standoff_min')),
            brake_decel=float(gp('brake_decel')), envelope_decel=float(gp('envelope_decel')),
            reaction_time=float(gp('reaction_time')),
            path_length=float(gp('path_length')), path_step=float(gp('path_step')))
        self.state_timeout = float(gp('state_timeout'))
        self.odom_timeout = float(gp('odom_timeout'))

        ctrl_params = {
            name: p.value
            for name, p in self.get_parameters_by_prefix('controller').items()}
        spec = gp('controller_class')
        self.controller = pursuit.load_controller(spec, ctrl_params)
        self.get_logger().info(f'controller {spec} with {ctrl_params}')

        self.state = None
        self.state_rx = None
        self.odom = None
        self.odom_rx = None
        self.autonomous = False
        self.prev_valid = False
        self.v_prev = 0.0
        self.t_prev = None

        self.pub_traj = self.create_publisher(Trajectory, gp('trajectory_topic'), 1)
        self.pub_dbg = self.create_publisher(PursuitDebug, gp('debug_topic'), 10)
        self.create_subscription(CoachState, gp('state_topic'), self.on_state, 10)
        self.create_subscription(Odometry, gp('odom_topic'), self.on_odom, 1)
        mode_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            OperationModeState, gp('operation_mode_topic'), self.on_mode, mode_qos)
        self.create_timer(1.0 / float(gp('rate')), self.on_timer)

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_state(self, msg):
        self.state = msg
        self.state_rx = self.now_sec()

    def on_odom(self, msg):
        self.odom = msg
        self.odom_rx = self.now_sec()

    def on_mode(self, msg):
        autonomous = msg.mode == OperationModeState.AUTONOMOUS
        if autonomous and not self.autonomous:
            # Engage: start the law from a clean slate, and the command from
            # zero, whatever it computed while it was not driving.
            self.controller.reset()
            self.v_prev = 0.0
        self.autonomous = autonomous

    def on_timer(self):
        now = self.now_sec()
        if self.odom is None or now - self.odom_rx > self.odom_timeout:
            self.get_logger().warn(
                'no fresh odometry; not publishing a trajectory',
                throttle_duration_sec=5.0)
            return
        dt = 0.0 if self.t_prev is None else now - self.t_prev
        self.t_prev = now

        st = self.state
        valid = (st is not None and st.valid
                 and now - self.state_rx <= self.state_timeout)
        if valid and not self.prev_valid:
            self.controller.reset()
        self.prev_valid = valid

        ego_speed = self.odom.twist.twist.linear.x
        v_raw = float('nan')
        if st is not None:
            try:
                v_raw = float(self.controller.update(st, ego_speed, dt))
            except Exception as e:  # noqa: BLE001 -- student code
                self.get_logger().error(
                    f'controller.update raised {type(e).__name__}: {e}',
                    throttle_duration_sec=2.0)
                v_raw = float('nan')
        if not self.autonomous:
            # Not driving: whatever the law asks, the profile the vehicle would
            # start from is the one the rate limit lets it reach from rest.
            self.v_prev = 0.0
        rng = st.range if valid else 0.0
        v_cmd, limit = pursuit.clamp_speed(v_raw, self.v_prev, dt, rng, valid, self.p)
        self.v_prev = v_cmd
        s_stop = pursuit.stop_distance(v_cmd, ego_speed, rng, valid, self.p)

        pose = self.odom.pose.pose
        q = pose.orientation
        yaw = pursuit.yaw_from_quaternion(q.x, q.y, q.z, q.w)
        heading = yaw + (st.bearing if valid else 0.0)
        pts = pursuit.build_trajectory(
            pose.position.x, pose.position.y, heading, v_cmd, s_stop, self.p,
            v_ego=ego_speed)

        stamp = self.get_clock().now().to_msg()
        frame = self.odom.header.frame_id or 'map'
        traj = Trajectory()
        traj.header.stamp = stamp
        traj.header.frame_id = frame
        qz, qw = math.sin(heading / 2.0), math.cos(heading / 2.0)
        for x, y, _, v, acc in pts:
            tp = TrajectoryPoint()
            tp.pose.position.x = x
            tp.pose.position.y = y
            tp.pose.position.z = pose.position.z
            tp.pose.orientation.z = qz
            tp.pose.orientation.w = qw
            tp.longitudinal_velocity_mps = float(v)
            tp.acceleration_mps2 = float(acc)
            traj.points.append(tp)
        self.pub_traj.publish(traj)

        dbg = PursuitDebug()
        dbg.header.stamp = stamp
        dbg.header.frame_id = frame
        dbg.valid = valid
        dbg.autonomous = self.autonomous
        if st is not None:
            dbg.range = float(st.range)
            dbg.range_rate = float(st.range_rate)
            dbg.bearing = float(st.bearing)
        dbg.ego_speed = float(ego_speed)
        dbg.v_controller = v_raw
        dbg.v_command = float(v_cmd)
        dbg.s_stop = float(s_stop)
        dbg.limit = limit
        self.pub_dbg.publish(dbg)


def main(args=None):
    rclpy.init(args=args)
    node = BoardPursuitPlanner()
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
