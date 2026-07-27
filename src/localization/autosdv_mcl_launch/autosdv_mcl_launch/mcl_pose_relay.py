"""``mcl_pose_relay`` -- adapt the 2D-MCL particle filter's pose output into
the ``PoseWithCovarianceStamped`` contract that ``ekf_localizer`` consumes.

See ``docs/design/localization-method-switching.md`` Sec 5.2 (gaps 1-5) and
``docs/research/localization/2d_mcl_algorithm.md`` Sec 5.3 for the defects
this node exists to fix rather than propagate:

1. **Covariance layout.** ``particle_filter`` writes a row-major 3x3
   ``(x, y, yaw)`` covariance into ``covariance[0:9]`` of a row-major 6x6
   field (see ``publish_tf`` in
   ``src/localization/external/particle_filter/particle_filter/particle_filter.py``),
   so most planar terms land in the wrong slots of the 6x6 (e.g. index 2,
   the x-z slot, receives cov(x, yaw) instead of cov(x, z)). This module
   re-reads the same flat 9 values -- which are still the correct 3x3
   ``(x, y, yaw)`` matrix, just misplaced -- and writes them to the
   correct row-major 6x6 destinations.
2. **Stamp.** ``particle_filter`` stamps both ``/pf/viz/inferred_pose`` and
   ``/pf/pose/odom`` with ``get_clock().now()`` (wall clock) while its
   ``map -> laser`` TF broadcast is stamped with the scan's own header
   stamp (``self.last_stamp``, see ``publish_tf``'s ``stamp`` argument and
   its caller in ``update()``). This node prefers the TF's scan-accurate
   stamp when available and only falls back to the (mis-)stamped message
   header as a last resort, logging a warning once so the discrepancy is
   never silent.
3. **Frame ids.** The filter emits ``'/map'`` and ``'/laser'`` with leading
   slashes, which tf2 rejects. All frame ids are normalised before use.
4. **Sensor vs. base frame.** The filter's inferred pose is the *laser*
   pose, not ``base_link``. This node composes the ``base_link <- laser``
   static transform via tf2 rather than assuming identity, and refuses to
   publish (fails loudly, logging an error) if that transform cannot be
   resolved.
5. **No subscriber-count gating.** ``particle_filter.visualize()`` only
   publishes when someone is already subscribed, which starves a
   late-joining EKF. This node always publishes on receipt of an input
   message.
"""

from typing import Optional, Sequence, Tuple

import numpy as np
import transforms3d.euler
import transforms3d.quaternions

# --------------------------------------------------------------------------
# Pure, ROS-free helpers. These are the functions under direct unit test in
# test/test_mcl_pose_relay.py; none of them touch rclpy, a node, or a clock.
# --------------------------------------------------------------------------

#: Row-major 6x6 covariance flat-index layout used by every Autoware pose
#: message (``geometry_msgs/PoseWithCovariance.covariance``), for reference:
#:
#:   row 0 (x):     0  1  2  3  4  5
#:   row 1 (y):     6  7  8  9  10 11
#:   row 2 (z):     12 13 14 15 16 17
#:   row 3 (roll):  18 19 20 21 22 23
#:   row 4 (pitch): 24 25 26 27 28 29
#:   row 5 (yaw):   30 31 32 33 34 35
#:
#: The particle filter's source data is a row-major flattened 3x3
#: ``(x, y, yaw)`` matrix (``np.cov(..., rowvar=False).flatten()``), i.e.
#: 9 values in the order xx, xy, x_yaw, yx, yy, y_yaw, yaw_x, yaw_y,
#: yaw_yaw. This maps each source index to its correct destination index
#: in the 6x6 layout above.
PLANAR_SRC_TO_DEST_INDEX = {
    0: 0,   # xx        -> (x, x)
    1: 1,   # xy        -> (x, y)
    2: 5,   # x_yaw     -> (x, yaw)
    3: 6,   # yx        -> (y, x)
    4: 7,   # yy        -> (y, y)
    5: 11,  # y_yaw     -> (y, yaw)
    6: 30,  # yaw_x     -> (yaw, x)
    7: 31,  # yaw_y     -> (yaw, y)
    8: 35,  # yaw_yaw   -> (yaw, yaw)
}

#: Diagonal indices for the axes 2D MCL never observes (z, roll, pitch).
UNUSED_AXIS_DIAG_INDICES = (14, 21, 28)

#: Diagonal indices for the axes 2D MCL does observe (x, y, yaw), used as
#: the "no covariance available at all" fallback.
PLANAR_DIAG_INDICES = (0, 7, 35)

COVARIANCE_LEN = 36


def normalize_frame_id(frame_id: str) -> str:
    """Strip leading slashes tf2 rejects (e.g. ``'/map'`` -> ``'map'``).

    tf2 has refused frame ids with a leading ``/`` since ROS 2's tf2
    rewrite; ``particle_filter`` still emits ``'/map'``/``'/laser'``
    (defect 3). Only leading slashes are stripped -- an empty or
    already-clean frame id passes through unchanged.
    """
    return frame_id.lstrip('/')


def remap_planar_covariance(
    source_flat9: Sequence[float],
    unused_axis_variance: float,
) -> list:
    """Place a flat 9-element row-major 3x3 ``(x, y, yaw)`` covariance into
    the correct slots of a flat row-major 6x6 covariance.

    ``source_flat9`` is exactly ``covariance[0:9]`` as written by
    ``particle_filter``'s ``publish_tf`` (see module docstring defect 1) --
    it is *already* the right 3x3 matrix, just aimed at the wrong 6x6
    destination. This function is the fix: every term is copied to its
    explicit, individually-justified destination index
    (``PLANAR_SRC_TO_DEST_INDEX``), and the unobserved z/roll/pitch
    diagonal entries are set to ``unused_axis_variance`` rather than left
    at zero (a zero variance tells a Kalman filter the value is known
    exactly, which is false and would make the filter overconfident/
    numerically brittle).
    """
    if len(source_flat9) != 9:
        raise ValueError(
            f'expected a flat 9-element 3x3 (x, y, yaw) covariance, got '
            f'{len(source_flat9)} elements')

    dest = [0.0] * COVARIANCE_LEN
    for src_idx, dest_idx in PLANAR_SRC_TO_DEST_INDEX.items():
        dest[dest_idx] = float(source_flat9[src_idx])
    for idx in UNUSED_AXIS_DIAG_INDICES:
        dest[idx] = float(unused_axis_variance)
    return dest


def unknown_planar_covariance(unused_axis_variance: float) -> list:
    """Covariance to publish when the input carries none at all (e.g. the
    node is configured against ``/pf/viz/inferred_pose``, a ``PoseStamped``
    with no covariance field). All six diagonal terms -- the three planar
    ones we cannot observe from this input, plus the three MCL never
    observes -- are set to ``unused_axis_variance`` so the EKF treats the
    whole pose as maximally uncertain instead of exact.
    """
    dest = [0.0] * COVARIANCE_LEN
    for idx in PLANAR_DIAG_INDICES + UNUSED_AXIS_DIAG_INDICES:
        dest[idx] = float(unused_axis_variance)
    return dest


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """Planar yaw (radians) -> ROS-ordered ``(x, y, z, w)`` quaternion."""
    w, x, y, z = transforms3d.euler.euler2quat(0.0, 0.0, yaw, axes='sxyz')
    return (x, y, z, w)


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """ROS-ordered ``(x, y, z, w)`` quaternion -> yaw (radians).

    Uses the full roll/pitch/yaw decomposition (not a small-angle
    shortcut) so a non-planar input quaternion still yields the correct
    yaw component instead of a silently wrong one.
    """
    _roll, _pitch, yaw = transforms3d.euler.quat2euler((w, x, y, z), axes='sxyz')
    return yaw


Vec3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]
Pose = Tuple[Vec3, Quat]


def compose_poses(pose_a: Pose, pose_b: Pose) -> Pose:
    """Compose two rigid-body poses: ``result = pose_a (x) pose_b``.

    Both poses and the result are ``(translation_xyz, quaternion_xyzw)``
    tuples. If ``pose_a`` is the pose of frame B in frame A (``T_A_B``)
    and ``pose_b`` is the pose of frame C in frame B (``T_B_C``), the
    result is the pose of frame C in frame A (``T_A_C``) -- standard
    homogeneous transform composition. Used to compose the MCL's
    ``map -> laser`` pose with the static ``laser -> base_link`` transform
    into a ``map -> base_link`` pose (defect 4).
    """
    t_a, q_a = pose_a
    t_b, q_b = pose_b

    r_a = transforms3d.quaternions.quat2mat(_xyzw_to_wxyz(q_a))
    r_b = transforms3d.quaternions.quat2mat(_xyzw_to_wxyz(q_b))

    r = r_a @ r_b
    t = np.asarray(t_a, dtype=np.float64) + r_a @ np.asarray(t_b, dtype=np.float64)
    q_wxyz = transforms3d.quaternions.mat2quat(r)

    return (
        (float(t[0]), float(t[1]), float(t[2])),
        _wxyz_to_xyzw(q_wxyz),
    )


def _xyzw_to_wxyz(q: Quat) -> Tuple[float, float, float, float]:
    x, y, z, w = q
    return (w, x, y, z)


def _wxyz_to_xyzw(q) -> Quat:
    w, x, y, z = q
    return (float(x), float(y), float(z), float(w))


def is_planar_quaternion(q: Quat, tol: float = 1e-3) -> bool:
    """True if ``q`` represents a pure yaw rotation (roll == pitch == 0)
    within ``tol`` radians. Used to sanity-check MCL's orientation input,
    which is only ever supposed to carry yaw (Utils.angle_to_quaternion).
    """
    x, y, z, w = q
    roll, pitch, _yaw = transforms3d.euler.quat2euler((w, x, y, z), axes='sxyz')
    return abs(roll) <= tol and abs(pitch) <= tol


def transform_to_pose(translation, rotation) -> Pose:
    """Convert a ``geometry_msgs/Transform``-shaped pair (objects with
    ``.x .y .z`` / ``.x .y .z .w``) into a plain ``(xyz, xyzw)`` tuple pair
    that :func:`compose_poses` accepts. Kept separate from the ROS message
    types so the composition math stays testable without constructing
    real messages.
    """
    return (
        (float(translation.x), float(translation.y), float(translation.z)),
        (float(rotation.x), float(rotation.y), float(rotation.z), float(rotation.w)),
    )


# --------------------------------------------------------------------------
# The ROS 2 node. Imports rclpy lazily-at-module-scope (not inside a
# function) to match the rest of this repo's convention (see
# particle_filter/particle_filter.py, test/test_seed_covariance.py), but
# every algorithmic decision above this line is import-safe without a ROS
# graph.
# --------------------------------------------------------------------------

import rclpy  # noqa: E402
from rclpy.duration import Duration  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy  # noqa: E402
from rclpy.time import Time  # noqa: E402

import tf2_ros  # noqa: E402

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402


class MclPoseRelay(Node):
    """Subscribe the 2D-MCL particle filter's pose output and republish it
    as ``PoseWithCovarianceStamped`` on the Autoware pose-estimator
    contract topic, fixing defects 1-5 documented in the module docstring.
    """

    def __init__(self):
        super().__init__('mcl_pose_relay')

        self.declare_parameter('input_topic', '/pf/pose/odom')
        self.declare_parameter('input_type', 'odometry')  # 'odometry' | 'pose_stamped'
        self.declare_parameter(
            'output_topic', '/localization/pose_estimator/pose_with_covariance')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('sensor_frame', 'laser')
        self.declare_parameter('unused_axis_variance', 1_000_000.0)
        self.declare_parameter('tf_timeout_sec', 0.2)
        # Task 8 end-to-end diagnosis (2026-07-27): the map_frame -> sensor_frame
        # lookup below is meant to recover particle_filter's OWN scan-accurate
        # broadcast stamp (defect 2 in this module's docstring). It is only
        # correct when nothing ELSE in the TF graph publishes into that same
        # frame pair. Replaying a bag that was itself originally recorded from
        # a real localized run (this repo's sample-site bag) puts a genuine,
        # unrelated "map -> base_link" chain onto the graph, and the lookup
        # silently resolves against THAT instead of failing over -- publishing
        # a frozen/wrong stamp that wrecks a downstream EKF's delay
        # compensation (confirmed via a three-way GT comparison: ~4 m raw
        # filter vs ~92 m after this relay, same underlying estimate). Default
        # ('tf') preserves every existing caller's behavior unchanged;
        # 'header' skips the TF lookup and always uses the input message's own
        # header.stamp, which is what a caller should set once it cannot
        # guarantee sensor_frame is not also populated by an unrelated source.
        self.declare_parameter('stamp_source', 'tf')  # 'tf' | 'header'

        self.input_topic = self.get_parameter('input_topic').value
        self.input_type = self.get_parameter('input_type').value
        self.output_topic = self.get_parameter('output_topic').value
        self.base_frame = normalize_frame_id(self.get_parameter('base_frame').value)
        self.map_frame = normalize_frame_id(self.get_parameter('map_frame').value)
        self.sensor_frame = normalize_frame_id(self.get_parameter('sensor_frame').value)
        self.unused_axis_variance = float(
            self.get_parameter('unused_axis_variance').value)
        self.tf_timeout = Duration(
            seconds=float(self.get_parameter('tf_timeout_sec').value))
        self.stamp_source = self.get_parameter('stamp_source').value
        if self.stamp_source not in ('tf', 'header'):
            raise ValueError(
                f"stamp_source must be 'tf' or 'header', got {self.stamp_source!r}")

        if self.input_type not in ('odometry', 'pose_stamped'):
            raise ValueError(
                f"input_type must be 'odometry' or 'pose_stamped', got "
                f"{self.input_type!r}")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._warned_wall_clock_stamp = False
        self._warned_frame_mismatch = False

        # Reliable + volatile, matching the pose-estimator contract topic's
        # QoS elsewhere in the launch tree (ekf_localizer expects reliable
        # delivery, not best-effort sensor QoS).
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.pub = self.create_publisher(PoseWithCovarianceStamped, self.output_topic, qos)

        if self.input_type == 'odometry':
            self.sub = self.create_subscription(
                Odometry, self.input_topic, self._odometry_callback, 10)
        else:
            self.sub = self.create_subscription(
                PoseStamped, self.input_topic, self._pose_stamped_callback, 10)

        self.get_logger().info(
            f'mcl_pose_relay: {self.input_topic} ({self.input_type}) -> '
            f'{self.output_topic}, base_frame={self.base_frame}, '
            f'map_frame={self.map_frame}, sensor_frame={self.sensor_frame}, '
            f'unused_axis_variance={self.unused_axis_variance}')

    # -- subscription callbacks -------------------------------------------

    def _odometry_callback(self, msg: Odometry):
        cov9 = [msg.pose.covariance[i] for i in range(9)]
        self._handle_pose(msg.header, msg.pose.pose, cov9)

    def _pose_stamped_callback(self, msg: PoseStamped):
        self._handle_pose(msg.header, msg.pose, None)

    # -- shared relay logic -------------------------------------------------

    def _handle_pose(self, header, pose, cov9: Optional[Sequence[float]]):
        frame_in = normalize_frame_id(header.frame_id) or self.map_frame
        if frame_in != self.map_frame and not self._warned_frame_mismatch:
            self.get_logger().warn(
                f"mcl_pose_relay: input frame_id '{frame_in}' (normalised) "
                f"does not match configured map_frame '{self.map_frame}'; "
                f"publishing under '{frame_in}' as received. (Warned once.)")
            self._warned_frame_mismatch = True

        # Defect 4: compose the sensor -> base_link static transform via
        # tf2 rather than assuming identity. Fail loudly (drop the message,
        # do not publish a wrong pose) if it cannot be resolved.
        try:
            sensor_to_base = self.tf_buffer.lookup_transform(
                self.sensor_frame, self.base_frame, Time(), timeout=self.tf_timeout)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            self.get_logger().error(
                f"mcl_pose_relay: cannot resolve transform "
                f"'{self.sensor_frame}' <- '{self.base_frame}' ({exc}); "
                f"dropping this pose instead of publishing an uncorrected "
                f"(laser-frame) pose as map-frame base_link.")
            return

        pose_map_sensor = (
            (pose.position.x, pose.position.y, pose.position.z),
            (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
        )
        sensor_to_base_pose = transform_to_pose(
            sensor_to_base.transform.translation, sensor_to_base.transform.rotation)
        pose_map_base = compose_poses(pose_map_sensor, sensor_to_base_pose)

        # Defect 2: prefer the scan-accurate stamp carried by the filter's
        # map -> sensor TF broadcast (particle_filter.py's publish_tf is
        # called with the scan's own header stamp) over the wall-clock
        # stamp baked into the pose/odom message header. Fall back to the
        # message's own (wall-clock) stamp, once-logged, if no such TF is
        # available -- e.g. replaying a rosbag that only contains the pose
        # topics and no TF, as in this package's verification bag.
        stamp = header.stamp
        if self.stamp_source == 'tf':
            try:
                map_to_sensor = self.tf_buffer.lookup_transform(
                    self.map_frame, self.sensor_frame, Time(), timeout=self.tf_timeout)
                stamp = map_to_sensor.header.stamp
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                if not self._warned_wall_clock_stamp:
                    self.get_logger().warn(
                        f"mcl_pose_relay: no '{self.map_frame}' -> "
                        f"'{self.sensor_frame}' TF available to recover the "
                        f"scan-accurate stamp; falling back to this message's "
                        f"own (wall-clock) header stamp. EKF delay "
                        f"compensation may be degraded as a result. "
                        f"(Warned once.)")
                    self._warned_wall_clock_stamp = True
        # else: stamp_source == 'header' -- explicitly configured to skip the
        # TF lookup entirely and always use the input message's own stamp;
        # see the stamp_source declare_parameter comment for why.

        # Defect 1: correct covariance placement (or explicit "unknown"
        # covariance when the input carries none at all).
        if cov9 is not None:
            covariance = remap_planar_covariance(cov9, self.unused_axis_variance)
        else:
            covariance = unknown_planar_covariance(self.unused_axis_variance)

        out = PoseWithCovarianceStamped()
        out.header.stamp = stamp
        out.header.frame_id = frame_in
        out.pose.pose.position.x = pose_map_base[0][0]
        out.pose.pose.position.y = pose_map_base[0][1]
        out.pose.pose.position.z = pose_map_base[0][2]
        out.pose.pose.orientation.x = pose_map_base[1][0]
        out.pose.pose.orientation.y = pose_map_base[1][1]
        out.pose.pose.orientation.z = pose_map_base[1][2]
        out.pose.pose.orientation.w = pose_map_base[1][3]
        out.pose.covariance = covariance

        # Defect 5: no subscriber-count gate -- always publish so a
        # late-joining EKF still receives the next pose.
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MclPoseRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
