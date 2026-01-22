# Phase 1: Setup & Preparation

**Status:** 🚧 In Progress
**Duration:** 1-2 days
**Dependencies:** None

[← Back to Overview](README.md) | [Next Phase: Map Creation →](phase_2_map_creation.md)

---

## Objectives

Prepare infrastructure and test environment for AR tag-based localization integration.

---

## Tasks

### Task 1.1: Review Autoware AR Tag Localizer

**Location:** `/opt/autoware/1.5.0/src/universe/autoware.universe/localization/autoware_landmark_based_localizer/autoware_ar_tag_based_localizer/`

**Description:**
Study Autoware's AR tag localizer implementation to understand input/output topics, parameter configuration, and integration points.

**Steps:**

1. Read the AR tag localizer documentation:
```bash
cat /opt/autoware/1.5.0/src/universe/autoware.universe/localization/autoware_landmark_based_localizer/autoware_ar_tag_based_localizer/README.md
```

2. Review source code:
```bash
cd /opt/autoware/1.5.0/src/universe/autoware.universe/localization/autoware_landmark_based_localizer/
find . -name "*.cpp" -o -name "*.hpp" | xargs cat
```

3. Check launch configuration:
```bash
cat autoware_ar_tag_based_localizer/launch/ar_tag_based_localizer.launch.xml
```

**Files to Review:**
- `autoware_ar_tag_based_localizer/src/ar_tag_based_localizer.cpp` - Main implementation
- `autoware_ar_tag_based_localizer/config/ar_tag_based_localizer.param.yaml` - Configuration
- `autoware_landmark_manager/src/landmark_manager.cpp` - Landmark map parsing

**Acceptance Criteria:**
- [ ] Understand input/output topics
- [ ] Understand parameter configuration
- [ ] Identify integration points with AutoSDV

---

### Task 1.2: Acquire AprilTag Markers

**Description:**
Generate and print AprilTag 16h5 markers for physical deployment in test environment.

**Marker Specifications:**
- **Family:** AprilTag 16h5
- **IDs:** 0-6 (7 markers minimum)
- **Size:** 0.6m × 0.6m (or 0.4m × 0.4m for smaller areas)
- **Material:** Printed on rigid backing (foam board, cardboard, or laminated paper)

**Generation Methods:**

**Option 1: Online Generator**
```bash
# Visit: https://chev.me/arucogen/
# Settings:
# - Dictionary: AprilTag 16h5
# - Marker ID: 0-6
# - Marker size: 600mm
# - Download each marker as PNG/PDF
```

**Option 2: Python Script**
```bash
# Install OpenCV with ArUco
pip install opencv-contrib-python

# Generate markers
python3 << 'EOF'
import cv2
from cv2 import aruco
import numpy as np

# AprilTag 16h5 dictionary
dict_type = aruco.DICT_APRILTAG_16h5
dictionary = aruco.getPredefinedDictionary(dict_type)

# Generate markers 0-6
for marker_id in range(7):
    # Create 800x800 pixel image
    marker_img = aruco.generateImageMarker(dictionary, marker_id, 800)

    # Add white border (recommended for better detection)
    border_size = 100
    bordered_img = cv2.copyMakeBorder(
        marker_img, border_size, border_size, border_size, border_size,
        cv2.BORDER_CONSTANT, value=255
    )

    # Save
    filename = f'apriltag_16h5_id_{marker_id}.png'
    cv2.imwrite(filename, bordered_img)
    print(f'Generated {filename}')
EOF
```

**Printing:**
- Print at actual size (600mm × 600mm)
- Use high-quality printer (600+ DPI)
- Mount on rigid backing for stability
- Laminate for outdoor durability

**Acceptance Criteria:**
- [ ] 7+ AprilTag 16h5 markers generated
- [ ] Markers printed and mounted
- [ ] Marker IDs clearly labeled on back

---

### Task 1.3: Define Test Environment Coordinate Frame

**Description:**
Establish a consistent global coordinate system for AR tag placement and vehicle localization.

**Setup Steps:**

1. **Choose Origin Point:**
   - Select a fixed, easily identifiable location (e.g., corner of room, building corner)
   - Mark physically (e.g., tape, paint marker)

2. **Define Axes:**
   - **X-axis:** Forward/East direction (vehicle forward path)
   - **Y-axis:** Left/North direction (perpendicular to X)
   - **Z-axis:** Up direction (vertical)

3. **Measurement Tools:**
   - Laser distance meter (±1cm accuracy)
   - Total station (for large outdoor areas)
   - Measuring tape (backup)
   - Level for vertical alignment

4. **Document Frame:**
   ```yaml
   coordinate_frame:
     name: "AutoSDV Test Area"
     origin:
       description: "Southwest corner of Building X entrance"
       lat: 25.0xxxxx  # If available
       lon: 121.5xxxxx # If available
     axes:
       x_direction: "East (magnetic heading 90°)"
       y_direction: "North (magnetic heading 0°)"
       z_direction: "Up (gravity-aligned)"
     measurement_unit: "meters"
   ```

**Acceptance Criteria:**
- [ ] Origin point physically marked
- [ ] Axes clearly defined and documented
- [ ] Measurement tools available and calibrated

---

## Deliverables

- [ ] Printed AR tags (0.6m × 0.6m recommended)
- [ ] Test area coordinate frame defined and documented
- [ ] Dependencies verified (ROS packages, libraries)
- [ ] Understanding of AR tag localizer architecture

---

## Testing

**Verification Steps:**
1. Confirm AR tag markers are printed correctly (check size and quality)
2. Verify coordinate frame documentation is complete
3. Test measurement tools for accuracy
4. Review AR tag localizer source code for integration points

---

## Next Phase

Once all deliverables are complete, proceed to [Phase 2: AR Tag Map Creation](phase_2_map_creation.md).
