# Phase 7: Documentation & Deployment

**Status:** ⏸️ Pending
**Duration:** 1-2 days
**Dependencies:** Phase 6

[← Previous: Testing](phase_6_testing.md) | [Back to Overview](README.md)

---

## Objectives

Finalize documentation, update user guides, and prepare system for production deployment.

---

## Tasks

### Task 7.1: Update CLAUDE.md Documentation

**Description:**
Update project-level documentation with AR tag localization configuration and usage instructions.

**File:** `CLAUDE.md`

**Add Section:**

```markdown
### AR Tag-Based Localization (Camera-Only Global Reference)

AutoSDV supports AR Tag-based localization for camera-only global reference, ideal for GPS-denied environments.

**How It Works:**
- AprilTag 16h5 markers placed at known locations
- Camera detects tags and calculates vehicle pose
- Fused with Isaac Visual SLAM for drift-free tracking

**Configuration:**
```bash
# Enable AR tag localization
make launch ARGS="pose_source:=artag map_path:=./data/ar_tag_test_map"
```

**Setup Requirements:**
1. Print AprilTag 16h5 markers (0.6m × 0.6m)
2. Place markers in environment
3. Measure tag corner positions (global coordinates)
4. Create Lanelet2 map with tag landmarks

**Map Format:**
See `docs/roadmaps/ar_tag_integration/` for implementation details.

**Parameters:**
- `marker_size`: Physical tag size (default: 0.6m)
- `target_tag_ids`: Marker IDs to detect (default: ['0'-'6'])
- `distance_threshold`: Max detection range (default: 13m)

**Fusion with Isaac Visual SLAM:**
```bash
# AR tags for global reference, Isaac VSLAM for local tracking
make launch ARGS="pose_source:=artag map_path:=./data/ar_tag_test_map use_gnss:=false"
```

**Monitoring:**
```bash
# Check AR tag detections
ros2 topic echo /diagnostics | grep -A 5 "ar_tag"

# Visualize detected tags
ros2 topic echo /localization/ar_tag_based_localizer/debug/detected_tag

# Monitor pose accuracy
ros2 run tf2_ros tf2_echo map base_link
```

**Troubleshooting:**
- **No tags detected**: Check lighting, tag visibility, camera focus
- **Pose jumps**: Verify tag measurements, check EKF covariance tuning
- **High drift without tags**: Increase tag density or AR tag coverage area
```

**Acceptance Criteria:**
- [ ] CLAUDE.md updated with AR tag section
- [ ] All configuration options documented
- [ ] Examples provided
- [ ] Troubleshooting guide included

---

### Task 7.2: Create AR Tag Setup Guide

**Description:**
Create comprehensive user guide for setting up AR tag-based localization from scratch.

**File:** `docs/guides/ar_tag_setup.md`

**Contents:**

```markdown
# AR Tag Setup Guide for AutoSDV

## Overview

Step-by-step guide for setting up AR tag-based localization in AutoSDV for camera-only global reference.

## Prerequisites

### Hardware Requirements
- ZED X Mini stereo camera (or equivalent stereo camera)
- Printer for A3/A2 size prints (or access to print shop)
- Laser distance meter (±1cm accuracy)
- Rigid backing material (foam board, cardboard, or wood)
- Mounting supplies (tape, adhesive, screws)

### Software Requirements
- AutoSDV with AR tag integration (Phase 1-6 complete)
- ROS 2 Humble
- Autoware 1.5.0

## Step 1: Generate AR Tags

### Option 1: Online Generator

1. Visit: https://chev.me/arucogen/
2. Settings:
   - Dictionary: **AprilTag 16h5**
   - Marker ID: 0-6 (or more as needed)
   - Marker size: **600mm**
3. Download each marker as PNG/PDF

### Option 2: Python Script

```bash
pip install opencv-contrib-python

python3 << 'EOF'
import cv2
from cv2 import aruco

dict_type = aruco.DICT_APRILTAG_16h5
dictionary = aruco.getPredefinedDictionary(dict_type)

for marker_id in range(7):
    marker_img = aruco.generateImageMarker(dictionary, marker_id, 800)
    border_size = 100
    bordered_img = cv2.copyMakeBorder(
        marker_img, border_size, border_size, border_size, border_size,
        cv2.BORDER_CONSTANT, value=255
    )
    filename = f'apriltag_16h5_id_{marker_id}.png'
    cv2.imwrite(filename, bordered_img)
    print(f'Generated {filename}')
EOF
```

## Step 2: Print and Mount Tags

1. **Print at actual size**: 600mm × 600mm (adjust printer settings)
2. **Mount on rigid backing**: Foam board or cardboard (prevents warping)
3. **Laminate** (optional): For outdoor durability
4. **Label back side**: Write marker ID clearly

## Step 3: Place Tags in Environment

### Placement Strategy

**Indoor:**
- Height: 1.5-2.5m (eye level to ceiling)
- Spacing: 3-5m apart
- Mounting: Walls, pillars, stands

**Outdoor:**
- Height: 1.5-3.0m
- Spacing: 5-10m apart
- Mounting: Poles, signposts, building walls

### Coverage Guidelines
- Vehicle should see ≥1 tag from any location
- Overlapping coverage (≥2 tags visible) preferred
- Clear line of sight, no occlusions
- Stable mounting (resistant to wind)

## Step 4: Measure Tag Positions

### Define Coordinate Frame

1. Choose origin point (e.g., room corner, building entrance)
2. Define axes:
   - X: Forward/East
   - Y: Left/North
   - Z: Up
3. Mark origin physically

### Measure Each Tag's Corners

**Corner order** (looking at tag front):
```
Corner 2 -------- Corner 3
    |                |
    |    TAG ID X    |
    |                |
Corner 1 -------- Corner 4
```

**Create measurement file** `ar_tag_measurements.yaml`:

```yaml
tags:
  - tag_id: 0
    marker_type: apriltag_16h5
    size: 0.6
    corners:
      corner_1: {x: 2.35, y: 5.12, z: 1.80}
      corner_2: {x: 2.35, y: 5.12, z: 2.40}
      corner_3: {x: 2.95, y: 5.12, z: 2.40}
      corner_4: {x: 2.95, y: 5.12, z: 1.80}
  # Repeat for all tags...
```

## Step 5: Create Lanelet2 Map

### Generate Map from Measurements

```bash
# Use the map generator script from Phase 2
cd ~/repos/AutoSDV/2025.02
python3 scripts/tools/generate_ar_tag_map.py

# Output: data/ar_tag_test_map/lanelet2_map.osm
```

### Verify Map

```bash
# Check map file
cat data/ar_tag_test_map/lanelet2_map.osm

# Verify:
# - All tags present
# - Marker IDs correct
# - Coordinates match measurements
```

## Step 6: Configure and Test

### Launch System

```bash
cd ~/repos/AutoSDV/2025.02

# Launch with AR tag localization
make launch ARGS="pose_source:=artag map_path:=./data/ar_tag_test_map use_gnss:=false"
```

### Verify Detection

```bash
# Monitor AR tag detections
ros2 topic echo /localization/pose_estimator/pose_with_covariance

# Check diagnostic status
ros2 topic echo /diagnostics | grep -A 5 "ar_tag"

# Visualize in RViz
# Expected: See vehicle pose update when tags are visible
```

### Tune Parameters (if needed)

Edit `src/launcher/autosdv_launch/config/localization/ar_tag_based_localizer.param.yaml`:

- `marker_size`: Match your printed tag size
- `distance_threshold`: Adjust for your camera (test empirically)
- `base_covariance`: Tune based on detection accuracy

## Troubleshooting

### No Tags Detected

**Symptoms:** `/localization/pose_estimator/pose_with_covariance` not publishing

**Solutions:**
- Check camera is working: `ros2 topic hz /sensing/camera/zedxm/zed_node/rgb/image_rect_color`
- Verify map loaded: `ros2 topic echo /map/vector_map`
- Improve lighting (tags need >200 lux)
- Move closer to tags (<10m for initial testing)
- Check tag IDs match configuration

### Pose Jumps / Instability

**Symptoms:** Large sudden changes in vehicle pose

**Solutions:**
- Verify tag corner measurements (re-measure if needed)
- Check for duplicate tag IDs in environment
- Tune EKF covariance (increase `pose_smoothing_steps`)
- Ensure stable tag mounting (no wind movement)

### High Drift Without Tags

**Symptoms:** Pose drifts significantly when no tags visible

**Solutions:**
- Increase AR tag density (add more tags)
- Expand tag coverage area
- Tune Isaac VSLAM parameters for better odometry
- Document operational envelope (max distance between tags)

### Detection Range Too Short

**Symptoms:** Tags only detected at close range (<5m)

**Solutions:**
- Increase tag size (0.6m → 1.0m)
- Improve camera exposure/focus settings
- Clean tag surfaces (no glare or dirt)
- Increase `distance_threshold` parameter (test empirically)

## Appendix: Checklist

### Setup Checklist

- [ ] AR tags generated (7+ tags)
- [ ] Tags printed and mounted
- [ ] Coordinate frame defined
- [ ] Tag positions measured
- [ ] Lanelet2 map created
- [ ] System configured
- [ ] Detection verified

### Operational Checklist

- [ ] All tags visible from expected vehicle paths
- [ ] Tag mounting stable (no movement)
- [ ] Lighting adequate (>200 lux)
- [ ] Map file in correct location
- [ ] Configuration parameters tuned

## Support

For issues or questions:
- Review implementation roadmap: `docs/roadmaps/ar_tag_integration/README.md`
- Check CLAUDE.md for quick reference
- File issue on project repository
```

**Acceptance Criteria:**
- [ ] Guide created with all sections
- [ ] Screenshots/diagrams included (if applicable)
- [ ] Tested by following guide end-to-end
- [ ] User feedback incorporated

---

## Deliverables

- [ ] Complete documentation (CLAUDE.md updated)
- [ ] User setup guide (`docs/guides/ar_tag_setup.md`)
- [ ] Troubleshooting guide (in setup guide)
- [ ] RViz configuration files (if created)

---

## Deployment Checklist

### Code Repository

- [ ] All code changes committed
- [ ] Branch merged to main (after review)
- [ ] Tagged release version (e.g., v1.0.0-ar-tag)
- [ ] CI/CD tests passing

### Documentation

- [ ] CLAUDE.md updated
- [ ] Setup guide complete
- [ ] Roadmap marked as complete
- [ ] Known limitations documented

### Configuration Files

- [ ] AR tag localizer config in repository
- [ ] EKF localizer config updated
- [ ] Launch files tested and documented
- [ ] Example map provided

### Testing Artifacts

- [ ] Test rosbags archived
- [ ] Performance metrics documented
- [ ] Operational envelope defined
- [ ] Edge case behavior documented

---

## Next Steps

**After Deployment:**

1. **Monitor System in Production**
   - Track detection rates
   - Monitor pose accuracy
   - Log any issues or failures

2. **Gather User Feedback**
   - Setup difficulty
   - Operational challenges
   - Feature requests

3. **Continuous Improvement**
   - Tune parameters based on real-world usage
   - Add more tags as needed
   - Update documentation based on feedback

4. **Future Enhancements**
   - Automatic tag map generation from rosbag
   - Multi-tag simultaneous detection
   - Dynamic tag addition/removal
   - RViz plugins for visualization

---

## Success Metrics

**Technical Metrics:**
- ✅ Camera-only localization functional
- ✅ Position accuracy <0.5m with tags visible
- ✅ Pose update rate >10 Hz
- ✅ System stable for >1 hour continuous operation

**Operational Metrics:**
- ✅ Setup time <1 day for new environment
- ✅ No manual interventions required during operation
- ✅ Clear documentation enables setup by non-experts

**Integration Complete!** 🎉

[← Back to Overview](README.md)
