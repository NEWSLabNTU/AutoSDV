# Phase 4: Isaac VSLAM Modification

**Status:** ⏸️ Pending
**Duration:** 2-3 days
**Dependencies:** Phase 3

[← Previous: AR Tag Localizer](phase_3_localizer.md) | [Back to Overview](README.md) | [Next: EKF Fusion →](phase_5_fusion.md)

---

## Objectives

Convert Isaac VSLAM from pose source to twist source by modifying the odometry bridge to output twist/velocity instead of pose.

---

## Tasks

### Task 4.1: Modify odometry_pose_bridge for Twist Output

**Description:**
Modify the `odometry_pose_bridge` package to support outputting twist (velocity) data in addition to or instead of pose data.

**Current Implementation:** `src/localization/odometry_pose_bridge/src/odometry_pose_bridge.cpp`

**Required Changes:**

**Step 1: Add Twist Publisher**

```cpp
// In class declaration (odometry_pose_bridge.hpp)
rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_pub_;

// In constructor (odometry_pose_bridge.cpp)
twist_pub_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "/localization/twist_estimator/twist_with_covariance", 10);
```

**Step 2: Extract Twist from Odometry**

```cpp
void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Existing pose conversion...

    // NEW: Extract and publish twist
    geometry_msgs::msg::TwistWithCovarianceStamped twist_msg;
    twist_msg.header = msg->header;
    twist_msg.header.frame_id = "base_link";  // Twist in vehicle frame
    twist_msg.twist.twist = msg->twist.twist;
    twist_msg.twist.covariance = msg->twist.covariance;

    twist_pub_->publish(twist_msg);
}
```

**Step 3: Add Launch Parameter for Mode Selection**

Edit `src/localization/odometry_pose_bridge/launch/odometry_pose_bridge.launch.xml`:

```xml
<!-- Output mode: pose|twist|both -->
<arg name="output_mode" default="pose" description="pose|twist|both"/>

<node pkg="odometry_pose_bridge" exec="odometry_pose_bridge_node" name="odometry_pose_bridge_node">
  <param name="output_mode" value="$(var output_mode)"/>
  <!-- Existing parameters... -->
</node>
```

**Step 4: Implement Mode Logic**

```cpp
// In class declaration
std::string output_mode_;

// In constructor
declare_parameter("output_mode", "pose");
output_mode_ = get_parameter("output_mode").as_string();

// In callback
void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    if (output_mode_ == "pose" || output_mode_ == "both") {
        // Publish pose (existing code)
        // ...
    }

    if (output_mode_ == "twist" || output_mode_ == "both") {
        // Publish twist (new code)
        // ...
    }
}
```

**Alternative Approach: Create Separate Node**

If you prefer not to modify existing code:

```bash
cd src/localization
cp -r odometry_pose_bridge odometry_twist_bridge

# Modify package.xml, CMakeLists.txt, source files
# Change all references from "pose" to "twist"
# Remove pose-related code, keep only twist output
```

**Files Modified:**
- `src/localization/odometry_pose_bridge/src/odometry_pose_bridge.cpp`
- `src/localization/odometry_pose_bridge/include/odometry_pose_bridge/odometry_pose_bridge.hpp`
- `src/localization/odometry_pose_bridge/launch/odometry_pose_bridge.launch.xml`
- `src/localization/odometry_pose_bridge/package.xml` (add twist message dependency)

**Acceptance Criteria:**
- [ ] Code compiles successfully
- [ ] Twist messages published on `/localization/twist_estimator/twist_with_covariance`
- [ ] Twist values validated against odometry input
- [ ] Mode parameter (`output_mode`) functional

---

### Task 4.2: Update Isaac VSLAM Launch Configuration

**Description:**
Update Isaac VSLAM launch file to use the modified odometry bridge with twist output mode.

**File:** `src/localization/autosdv_isaac_slam_launch/launch/autosdv_isaac_slam.launch.py`

**Changes:**

```python
# Modify odometry bridge node configuration
odometry_bridge_node = Node(
    package='odometry_pose_bridge',
    executable='odometry_pose_bridge_node',
    name='isaac_vslam_twist_bridge',
    parameters=[{
        'output_mode': 'twist',  # NEW: Output twist instead of pose
        'target_frame': 'base_link',
        'covariance_scale': 1.0
    }],
    remappings=[
        ('input/odometry', '/visual_slam/tracking/odometry'),
        ('output/twist_with_covariance', '/localization/twist_estimator/twist_with_covariance')
    ]
)
```

**Update Conditional Launch:**

Ensure Isaac VSLAM is NOT launched when `pose_source:=artag`:

```yaml
# In autosdv.launch.yaml
- group:
    if: $(eval "'$(var pose_source)' == 'isaac'")
  children:
    - include:
        file: $(find-pkg-share autosdv_isaac_slam_launch)/launch/autosdv_isaac_slam.launch.py
        # Launch Isaac VSLAM as pose source

# When pose_source:=artag, Isaac VSLAM twist can still be used
# by adding a separate twist_source parameter (future enhancement)
```

**Files Modified:**
- `src/localization/autosdv_isaac_slam_launch/launch/autosdv_isaac_slam.launch.py`

**Acceptance Criteria:**
- [ ] Launch file updated
- [ ] Twist topic published correctly (`/localization/twist_estimator/twist_with_covariance`)
- [ ] No duplicate pose publishers when using AR tags

---

## Deliverables

- [ ] Modified `odometry_pose_bridge` node (or new `odometry_twist_bridge`)
- [ ] Updated package dependencies in `package.xml`
- [ ] Updated Isaac VSLAM launch file
- [ ] Twist validation test results

---

## Testing

**Test Twist Output:**

```bash
# Build changes
cd ~/repos/AutoSDV/2025.02
colcon build --base-paths src/localization --symlink-install

# Launch Isaac VSLAM with twist output
ros2 launch autosdv_isaac_slam_launch autosdv_isaac_slam.launch.py output_mode:=twist

# Monitor twist topic
ros2 topic echo /localization/twist_estimator/twist_with_covariance

# Verify twist data
# - Linear velocity should match vehicle motion
# - Angular velocity should match turning
# - Covariance matrix should be reasonable
```

**Validation:**
```bash
# Compare twist against odometry
ros2 topic echo /visual_slam/tracking/odometry --field twist.twist

# Should match twist output (within frame transformation)
```

---

## Next Phase

Once Isaac VSLAM twist output is validated, proceed to [Phase 5: EKF Fusion Configuration](phase_5_fusion.md) to fuse AR tag pose and VSLAM twist.
