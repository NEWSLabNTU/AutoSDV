#!/usr/bin/env python3
"""Republish VelocityReport with the longitudinal speed scaled.

The COSS bags carry /vehicle/status/velocity_status straight from the hall
sensor, which reads about 1.8x high (see docs/reports/cuda-ndt-coss-replay.md,
finding 4). Those numbers are already in the bag, so no parameter change can
undo them: the EKF over-predicts between scans, NDT drags the pose back on
every frame, and the vehicle visibly lurches.

Play the bag with the raw topic remapped aside and run this to feed the stack a
corrected copy:

    ros2 bag play <bag> --clock \
        --remap /vehicle/status/velocity_status:=/vehicle/status/velocity_status_raw
    SCALE=0.5 python3 velocity_scaler.py

This is a replay workaround. The real fix is on the vehicle: verify
markers_per_rotation in autosdv_vehicle_interface/params/velocity_report.yaml.
"""
import os

import rclpy
from autoware_vehicle_msgs.msg import VelocityReport
from rclpy.node import Node

IN_TOPIC = "/vehicle/status/velocity_status_raw"
OUT_TOPIC = "/vehicle/status/velocity_status"


class VelocityScaler(Node):
    def __init__(self):
        super().__init__("velocity_scaler")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.scale = float(os.environ.get("SCALE", "0.5"))
        self.count = 0
        self.pub = self.create_publisher(VelocityReport, OUT_TOPIC, 10)
        self.sub = self.create_subscription(VelocityReport, IN_TOPIC, self.cb, 10)
        self.get_logger().info(
            f"scaling longitudinal velocity by {self.scale}: {IN_TOPIC} -> {OUT_TOPIC}")

    def cb(self, msg):
        out = VelocityReport()
        out.header = msg.header
        out.longitudinal_velocity = msg.longitudinal_velocity * self.scale
        out.lateral_velocity = msg.lateral_velocity * self.scale
        out.heading_rate = msg.heading_rate  # from the gyro, not the wheel
        self.pub.publish(out)
        self.count += 1
        if self.count % 500 == 0:
            self.get_logger().info(f"republished {self.count} messages")


def main():
    rclpy.init()
    node = VelocityScaler()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
