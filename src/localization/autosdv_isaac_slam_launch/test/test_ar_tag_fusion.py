#!/usr/bin/env python3
"""Integration tests for AR Tag + Isaac VSLAM fusion."""

import unittest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
import threading


class TestARTagFusion(unittest.TestCase):
    """Test suite for AR Tag + Isaac VSLAM fusion."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS 2."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS 2."""
        rclpy.shutdown()

    def test_ar_tag_detection(self):
        """Test AR tag detection and pose publication."""
        node = rclpy.create_node('test_ar_tag_detection')

        # Subscribe to AR tag pose
        pose_received = []

        def pose_callback(msg):
            pose_received.append(msg)

        sub = node.create_subscription(
            PoseWithCovarianceStamped,
            '/localization/pose_estimator/pose_with_covariance',
            pose_callback,
            10
        )

        # Spin in background thread
        spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
        spin_thread.start()

        # Wait for message (with timeout)
        timeout = 10.0  # seconds
        rate = node.create_rate(10)  # 10 Hz
        elapsed = 0.0
        while not pose_received and elapsed < timeout:
            rate.sleep()
            elapsed += 0.1

        node.destroy_node()

        # Verify message received
        self.assertTrue(len(pose_received) > 0, "No AR tag pose received")
        if pose_received:
            self.assertIsNotNone(pose_received[0].header.stamp, "Pose has invalid timestamp")
            self.assertEqual(pose_received[0].header.frame_id, "map",
                           "Pose frame_id should be 'map'")

    def test_vslam_twist_output(self):
        """Test Isaac VSLAM twist output."""
        node = rclpy.create_node('test_vslam_twist')

        # Subscribe to VSLAM twist
        twist_received = []

        def twist_callback(msg):
            twist_received.append(msg)

        sub = node.create_subscription(
            TwistWithCovarianceStamped,
            '/localization/twist_estimator/twist_with_covariance',
            twist_callback,
            10
        )

        # Spin in background thread
        spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
        spin_thread.start()

        # Wait for message
        timeout = 10.0
        rate = node.create_rate(10)
        elapsed = 0.0
        while not twist_received and elapsed < timeout:
            rate.sleep()
            elapsed += 0.1

        node.destroy_node()

        # Verify message received
        self.assertTrue(len(twist_received) > 0, "No VSLAM twist received")
        if twist_received:
            # Check frame_id (should be base_link or odom depending on configuration)
            self.assertIn(twist_received[0].header.frame_id, ["base_link", "odom"],
                        f"Twist frame_id unexpected: {twist_received[0].header.frame_id}")

    def test_ekf_fusion(self):
        """Test EKF fusion of AR tag pose + VSLAM twist."""
        node = rclpy.create_node('test_ekf_fusion')

        # Subscribe to EKF output
        ekf_pose_received = []

        def ekf_callback(msg):
            ekf_pose_received.append(msg)

        sub = node.create_subscription(
            PoseWithCovarianceStamped,
            '/localization/pose_with_covariance',
            ekf_callback,
            10
        )

        # Spin in background thread
        spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
        spin_thread.start()

        # Wait for messages (need at least 10 for rate calculation)
        timeout = 10.0
        rate = node.create_rate(10)
        elapsed = 0.0
        while len(ekf_pose_received) < 10 and elapsed < timeout:
            rate.sleep()
            elapsed += 0.1

        node.destroy_node()

        # Verify fusion output
        self.assertTrue(len(ekf_pose_received) >= 10,
                       f"Not enough EKF poses received (got {len(ekf_pose_received)}, need 10)")

        # Check update rate (should be >5 Hz minimum)
        if len(ekf_pose_received) >= 2:
            first_stamp = ekf_pose_received[0].header.stamp
            last_stamp = ekf_pose_received[-1].header.stamp
            dt = (last_stamp.sec - first_stamp.sec) + \
                 (last_stamp.nanosec - first_stamp.nanosec) * 1e-9

            if dt > 0:
                rate_hz = len(ekf_pose_received) / dt
                self.assertGreater(rate_hz, 5.0,
                                 f"EKF update rate too low: {rate_hz:.1f} Hz (expected >5 Hz)")

        # Verify covariance is populated
        if ekf_pose_received:
            cov = ekf_pose_received[0].pose.covariance
            self.assertTrue(any(c != 0.0 for c in cov),
                          "EKF covariance should be populated (all zeros)")

    def test_topic_consistency(self):
        """Test that all localization topics use consistent frame IDs."""
        node = rclpy.create_node('test_topic_consistency')

        received_msgs = {
            'ar_tag': [],
            'twist': [],
            'ekf': []
        }

        # Subscribe to all topics
        def ar_callback(msg):
            received_msgs['ar_tag'].append(msg)

        def twist_callback(msg):
            received_msgs['twist'].append(msg)

        def ekf_callback(msg):
            received_msgs['ekf'].append(msg)

        subs = [
            node.create_subscription(
                PoseWithCovarianceStamped,
                '/localization/pose_estimator/pose_with_covariance',
                ar_callback, 10),
            node.create_subscription(
                TwistWithCovarianceStamped,
                '/localization/twist_estimator/twist_with_covariance',
                twist_callback, 10),
            node.create_subscription(
                PoseWithCovarianceStamped,
                '/localization/pose_with_covariance',
                ekf_callback, 10)
        ]

        # Spin in background
        spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
        spin_thread.start()

        # Wait for at least one message from each
        timeout = 15.0
        rate = node.create_rate(10)
        elapsed = 0.0
        while elapsed < timeout:
            if (len(received_msgs['ar_tag']) > 0 and
                len(received_msgs['twist']) > 0 and
                len(received_msgs['ekf']) > 0):
                break
            rate.sleep()
            elapsed += 0.1

        node.destroy_node()

        # Verify we got messages
        self.assertTrue(len(received_msgs['ekf']) > 0,
                       "No EKF pose messages received")

        # Check frame IDs are valid
        if received_msgs['ar_tag']:
            self.assertEqual(received_msgs['ar_tag'][0].header.frame_id, "map",
                           "AR tag pose should be in 'map' frame")

        if received_msgs['ekf']:
            self.assertEqual(received_msgs['ekf'][0].header.frame_id, "map",
                           "EKF pose should be in 'map' frame")


if __name__ == '__main__':
    unittest.main()
