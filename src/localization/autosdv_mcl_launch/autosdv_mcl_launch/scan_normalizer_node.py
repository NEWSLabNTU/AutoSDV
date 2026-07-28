#!/usr/bin/env python3
"""ROS node wrapping the scan normaliser.

Sits between the sensor kit and `particle_filter`, doing three jobs that would
otherwise be spread across the kit, a QoS bridge, and nobody:

1. **QoS** -- subscribes with SensorDataQoS (BEST_EFFORT compatible) so a driver
   publishing sensor-style QoS is accepted, and republishes RELIABLE, which is
   what `particle_filter`'s default subscription expects. This replaces
   `scan_qos_bridge`.
2. **Frame** -- re-expresses ranges about `base_link`, because
   `particle_filter.py` treats the scan as originating at the particle pose. See
   `scan_normalizer.py` for the geometry and why it matters.
3. **Diagnostics** -- every scan-side failure in this project's history was
   silent: an empty slab, a missing TF, a bag with no scan at all. Each
   condition below is logged once on entry rather than per message, so a wrong
   setup announces itself instead of producing an inexplicably idle filter.
"""
from __future__ import annotations

import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

from autosdv_mcl_launch.scan_normalizer import (
    ScanSpec,
    is_identity,
    planar_offset,
    retarget_ranges,
)


class ScanNormalizer(Node):
    def __init__(self):
        super().__init__("mcl_scan_normalizer")

        self.declare_parameter("input_topic", "/scan_raw")
        self.declare_parameter("output_topic", "/scan")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tf_timeout_s", 0.2)
        self.declare_parameter("warn_tilt_deg", 2.0)
        self.declare_parameter("watchdog_period_s", 5.0)
        # Output extent. Left at 0.0 to inherit the input scan's own geometry,
        # which is the right default: a kit that publishes a 270 deg Hokuyo scan
        # should not have it silently widened to 360.
        self.declare_parameter("output_angle_min", 0.0)
        self.declare_parameter("output_angle_max", 0.0)
        self.declare_parameter("output_angle_increment", 0.0)
        self.declare_parameter("output_range_min", 0.0)
        self.declare_parameter("output_range_max", 0.0)

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.base_frame = self.get_parameter("base_frame").value
        self.tf_timeout = Duration(
            seconds=float(self.get_parameter("tf_timeout_s").value))
        self.warn_tilt = math.radians(float(self.get_parameter("warn_tilt_deg").value))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(
            LaserScan, self.output_topic,
            QoSProfile(depth=5, reliability=QoSReliabilityPolicy.RELIABLE))
        self.sub = self.create_subscription(
            LaserScan, self.input_topic, self._on_scan, qos_profile_sensor_data)

        self.scans_in = 0
        self.scans_out = 0
        # Latched so each condition is reported once, not per message.
        self._said = set()
        self.create_timer(float(self.get_parameter("watchdog_period_s").value),
                          self._watchdog)
        self.get_logger().info(
            f"normalising {self.input_topic} -> {self.output_topic} "
            f"about {self.base_frame}")

    def _once(self, key: str, level: str, message: str):
        """Log `message` at most once per `key`.

        Each severity gets its own call site on purpose. rclpy caches a
        logger's severity per *caller location*, so dispatching several
        severities through one `getattr(logger, level)(...)` line raises
        "Logger severity cannot be changed between calls" the moment a second
        severity is used from it -- which killed this node's subscription
        callback on the first scan it received, silently starving the filter.
        """
        if key in self._said:
            return
        self._said.add(key)
        log = self.get_logger()
        if level == "error":
            log.error(message)
        elif level == "warning":
            log.warning(message)
        else:
            log.info(message)

    def _watchdog(self):
        if self.scans_in == 0:
            self.get_logger().warning(
                f"no LaserScan on {self.input_topic} yet; the sensor kit is "
                "expected to publish it (or launch with "
                "scan_source:=test_pointcloud to synthesise one from a 3-D cloud)")

    def _output_spec(self, scan: LaserScan) -> ScanSpec:
        def pick(name, fallback):
            v = float(self.get_parameter(name).value)
            return v if v != 0.0 else fallback

        return ScanSpec(
            angle_min=pick("output_angle_min", scan.angle_min),
            angle_max=pick("output_angle_max", scan.angle_max),
            angle_increment=pick("output_angle_increment", scan.angle_increment),
            range_min=pick("output_range_min", scan.range_min),
            range_max=pick("output_range_max", scan.range_max),
        )

    def _lookup(self, scan: LaserScan):
        """Planar offset base_frame -> scan frame, or None when TF is unavailable."""
        frame = scan.header.frame_id.lstrip("/")
        if frame == self.base_frame:
            return (0.0, 0.0, 0.0, 0.0)
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, frame, scan.header.stamp, self.tf_timeout)
        except Exception as e:
            self._once(
                "tf", "error",
                f"{self.input_topic} is in frame '{frame}' but no transform to "
                f"{self.base_frame} is available ({type(e).__name__}); check the "
                "sensor kit's calibration. Passing the scan through unchanged, "
                "which leaves a lever-arm error equal to the mounting offset.")
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        return planar_offset((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))

    def _on_scan(self, scan: LaserScan):
        self.scans_in += 1
        if not any(math.isfinite(r) and r > 0.0 for r in scan.ranges):
            self._once(
                "empty", "error",
                f"{self.input_topic} carries {len(scan.ranges)} beams and not one "
                "finite range; ring selection, the z band, or the range limits "
                "are likely wrong")

        offset = self._lookup(scan)
        if offset is None:
            out = scan                        # pass through; already reported
        elif is_identity(offset):
            self._once("identity", "info",
                       f"scan frame coincides with {self.base_frame}; passing through")
            out = scan
        else:
            dx, dy, dyaw, tilt = offset
            self._once("offset", "info",
                       f"re-targeting scan from '{scan.header.frame_id}' by "
                       f"dx={dx:.3f} dy={dy:.3f} dyaw={math.degrees(dyaw):.1f}deg")
            if tilt > self.warn_tilt:
                self._once(
                    "tilt", "warning",
                    f"sensor is tilted {math.degrees(tilt):.1f} deg out of plane; a "
                    "planar re-target cannot represent roll or pitch, so ranges "
                    "will be biased with range. Mount level, or project the scan "
                    "through laser_geometry instead.")
            spec = self._output_spec(scan)
            out = LaserScan()
            out.header = scan.header
            out.angle_min = spec.angle_min
            out.angle_max = spec.angle_max
            out.angle_increment = spec.angle_increment
            out.time_increment = scan.time_increment
            out.scan_time = scan.scan_time
            out.range_min = spec.range_min
            out.range_max = spec.range_max
            out.ranges = retarget_ranges(
                list(scan.ranges), scan.angle_min, scan.angle_increment, offset, spec)
            out.intensities = []          # invalidated by re-binning

        # The frame is the contract: downstream treats the scan as taken at the
        # particle pose, so it must claim base_frame once re-targeted.
        stamped = LaserScan()
        stamped.header.stamp = out.header.stamp
        stamped.header.frame_id = self.base_frame
        stamped.angle_min = out.angle_min
        stamped.angle_max = out.angle_max
        stamped.angle_increment = out.angle_increment
        stamped.time_increment = out.time_increment
        stamped.scan_time = out.scan_time
        stamped.range_min = out.range_min
        stamped.range_max = out.range_max
        stamped.ranges = list(out.ranges)
        stamped.intensities = list(out.intensities)
        self.pub.publish(stamped)
        self.scans_out += 1


def run(args=None):
    rclpy.init(args=args)
    node = ScanNormalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    run(args)


if __name__ == "__main__":
    main()
