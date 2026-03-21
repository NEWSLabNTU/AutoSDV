# Phase 6: Integration Testing

**Status:** ⏸️ Pending
**Duration:** 3-5 days
**Dependencies:** Phase 5

[← Previous: EKF Fusion](phase_5_fusion.md) | [Back to Overview](README.md) | [Next: Deployment →](phase_7_deployment.md)

---

## Objectives

Validate complete system in realistic scenarios with comprehensive testing including unit tests, integration tests, and rosbag-based testing.

---

## Tasks

### Task 6.1: Create Integration Test Suite

**Description:**
Develop automated integration tests for AR tag fusion system to validate functionality and prevent regressions.

**Test File:** `src/localization/autosdv_isaac_slam_launch/test/test_ar_tag_fusion.py`

```python
#!/usr/bin/env python3
"""Integration tests for AR Tag + Isaac VSLAM fusion."""

import unittest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry

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

        # Wait for message (with timeout)
        timeout = 10.0  # seconds
        start_time = node.get_clock().now()
        while not pose_received and (node.get_clock().now() - start_time).nanoseconds < timeout * 1e9:
            rclpy.spin_once(node, timeout_sec=0.1)

        node.destroy_node()

        # Verify message received
        self.assertTrue(len(pose_received) > 0, "No AR tag pose received")
        self.assertIsNotNone(pose_received[0].header.stamp, "Pose has invalid timestamp")

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

        # Wait for message
        timeout = 10.0
        start_time = node.get_clock().now()
        while not twist_received and (node.get_clock().now() - start_time).nanoseconds < timeout * 1e9:
            rclpy.spin_once(node, timeout_sec=0.1)

        node.destroy_node()

        # Verify message received
        self.assertTrue(len(twist_received) > 0, "No VSLAM twist received")
        self.assertEqual(twist_received[0].header.frame_id, "base_link", "Twist frame_id incorrect")

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

        # Wait for messages
        timeout = 10.0
        start_time = node.get_clock().now()
        while len(ekf_pose_received) < 10 and (node.get_clock().now() - start_time).nanoseconds < timeout * 1e9:
            rclpy.spin_once(node, timeout_sec=0.1)

        node.destroy_node()

        # Verify fusion output
        self.assertTrue(len(ekf_pose_received) >= 10, "Not enough EKF poses received")

        # Check update rate (should be >10 Hz)
        if len(ekf_pose_received) >= 2:
            dt = (ekf_pose_received[-1].header.stamp.sec - ekf_pose_received[0].header.stamp.sec) + \
                 (ekf_pose_received[-1].header.stamp.nanosec - ekf_pose_received[0].header.stamp.nanosec) * 1e-9
            rate = len(ekf_pose_received) / dt
            self.assertGreater(rate, 5.0, f"EKF update rate too low: {rate:.1f} Hz")

if __name__ == '__main__':
    unittest.main()
```

**Running Tests:**

```bash
# Build package with tests
colcon build --base-paths src/localization --symlink-install

# Run integration tests
colcon test --base-paths src/localization/autosdv_isaac_slam_launch --event-handlers console_direct+

# View test results
colcon test-result --verbose
```

**Acceptance Criteria:**
- [ ] Test suite created with ≥3 integration tests
- [ ] Tests pass in local environment
- [ ] Tests integrated into CI pipeline (if available)

---

### Task 6.2: Rosbag-based Testing

**Description:**
Test with recorded camera data to validate localization accuracy and system behavior in realistic scenarios.

**Step 1: Record Test Data**

```bash
# Record test rosbag with camera and sensor data
ros2 bag record -o ar_tag_test_data \
  /sensing/camera/zedxm/zed_node/rgb/image_rect_color \
  /sensing/camera/zedxm/zed_node/rgb/camera_info \
  /sensing/camera/zedxm/zed_node/left/image_rect_color \
  /sensing/camera/zedxm/zed_node/right/image_rect_color \
  /sensing/camera/zedxm/zed_node/left/camera_info \
  /sensing/camera/zedxm/zed_node/right/camera_info \
  /sensing/imu/imu_data \
  /tf /tf_static

# Recording guidelines:
# - Duration: 2-5 minutes of driving in test area
# - Include multiple AR tag encounters
# - Vary lighting conditions if testing outdoor
# - Include sections without tag visibility
```

**Step 2: Playback Testing**

```bash
# Terminal 1: Launch system
ros2 launch autosdv_launch autosdv.launch.yaml \
  pose_source:=artag \
  map_path:=./data/ar_tag_test_map \
  use_gnss:=false

# Terminal 2: Play rosbag
ros2 bag play ar_tag_test_data

# Terminal 3: Monitor localization
ros2 run tf2_ros tf2_echo map base_link

# Terminal 4: Record output for analysis
ros2 bag record -o ar_tag_test_output \
  /localization/pose_estimator/pose_with_covariance \
  /localization/twist_estimator/twist_with_covariance \
  /localization/pose_with_covariance \
  /tf
```

**Step 3: Analyze Results**

```python
#!/usr/bin/env python3
"""Analyze localization accuracy from rosbag test."""

import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import matplotlib.pyplot as plt

# Load rosbag
reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri='ar_tag_test_output', storage_id='sqlite3'),
            rosbag2_py.ConverterOptions('', ''))

# Extract pose data
poses = []
timestamps = []

while reader.has_next():
    topic, data, timestamp = reader.read_next()
    if topic == '/localization/pose_with_covariance':
        msg = deserialize_message(data, PoseWithCovarianceStamped)
        poses.append(msg.pose.pose.position)
        timestamps.append(timestamp)

# Plot trajectory
x = [p.x for p in poses]
y = [p.y for p in poses]

plt.figure(figsize=(10, 8))
plt.plot(x, y, 'b-', linewidth=2, label='Estimated trajectory')
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.title('AR Tag + VSLAM Localization Trajectory')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.savefig('localization_trajectory.png')
plt.show()

print(f"Total poses: {len(poses)}")
print(f"Trajectory length: {sum(np.linalg.norm([x[i+1]-x[i], y[i+1]-y[i]]) for i in range(len(x)-1)):.2f} m")
```

**Acceptance Criteria:**
- [ ] Test rosbag recorded successfully
- [ ] Playback completes without errors
- [ ] Localization outputs validated (no NaN values, reasonable trajectory)
- [ ] Performance metrics documented

---

## System Testing Scenarios

### Scenario 1: Static Pose Estimation

**Objective:** Validate pose accuracy at known position

**Procedure:**
1. Place vehicle at known position in test area
2. Ensure AR tag is visible
3. Launch system and wait for pose convergence
4. Compare estimated pose to ground truth

**Acceptance:** <10cm position error, <5° orientation error

---

### Scenario 2: Moving Vehicle with Tag Visibility

**Objective:** Verify drift-free tracking when tags are visible

**Procedure:**
1. Drive through area with visible AR tags
2. Monitor drift between tag detections
3. Record pose trajectory

**Acceptance:** No drift accumulation when tags detected

---

### Scenario 3: Moving Vehicle without Tag Visibility

**Objective:** Measure drift characteristics without global reference

**Procedure:**
1. Drive through area without AR tags
2. Measure drift over time/distance
3. Return to tagged area, measure correction

**Acceptance:** Document drift characteristics (e.g., <5% of distance traveled)

---

### Scenario 4: Tag Re-Detection

**Objective:** Verify pose consistency on tag re-encounter

**Procedure:**
1. Drive past AR tag, record pose
2. Drive away, return to same tag
3. Compare poses on re-detection

**Acceptance:** <20cm position difference

---

### Scenario 5: Environmental Challenges

**Objective:** Define operational envelope

**Test Cases:**
- Varying lighting (dawn, noon, dusk, indoor/outdoor)
- Partial tag occlusion (50%, 75%)
- Maximum detection distance
- Moving camera (motion blur)

**Acceptance:** Define and document operational limits

---

## Deliverables

- [ ] Integration test results (all tests passing)
- [ ] Test rosbags recorded and validated
- [ ] Performance metrics documented
- [ ] Known limitations documented
- [ ] Operational envelope defined

---

## Testing Checklist

**Functional Tests:**
- [ ] AR tag detection at 3m, 5m, 10m distances
- [ ] Pose accuracy within 0.5m when tag visible
- [ ] Twist output from Isaac VSLAM continuous
- [ ] EKF fusion stable for >5 minutes
- [ ] No crashes or errors during operation

**Performance Tests:**
- [ ] Pose update rate >10 Hz
- [ ] Detection latency <100ms
- [ ] CPU usage reasonable (<50% on target hardware)
- [ ] Memory usage stable (no leaks)

**Edge Cases:**
- [ ] No tags visible (graceful degradation)
- [ ] Multiple tags visible simultaneously (multi-tag fusion)
- [ ] Rapid tag transitions (tag A → tag B)
- [ ] False positive rejection (incorrect marker detection)

---

## Next Phase

Once all testing is complete and results are satisfactory, proceed to [Phase 7: Documentation & Deployment](phase_7_deployment.md).
