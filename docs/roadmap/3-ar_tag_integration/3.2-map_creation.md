# Phase 2: AR Tag Map Creation

**Status:** ⏸️ Pending
**Duration:** 2-3 days
**Dependencies:** Phase 1

[← Previous: Setup](phase_1_setup.md) | [Back to Overview](README.md) | [Next: AR Tag Localizer →](phase_3_localizer.md)

---

## Objectives

Create Lanelet2 map file with AR tag landmarks by placing markers in test environment and measuring their positions.

---

## Tasks

### Task 2.1: Place AR Tags in Environment

**Description:**
Deploy AprilTag markers in test environment according to placement strategy for optimal coverage and detection.

**Placement Strategy:**

**Indoor Environment:**
- **Quantity:** 5-10 tags
- **Height:** 1.5-2.5m (eye level to ceiling)
- **Spacing:** 3-5m apart
- **Orientation:** Facing expected vehicle paths
- **Mounting:** Walls, pillars, stands

**Outdoor Environment:**
- **Quantity:** 7-15 tags
- **Height:** 1.5-3.0m
- **Spacing:** 5-10m apart
- **Orientation:** Facing roads/paths
- **Mounting:** Poles, signposts, building walls

**Coverage Requirements:**
- Vehicle should see ≥1 tag from any location
- Overlapping coverage (≥2 tags visible) preferred
- Clear line of sight, no occlusions
- Stable mounting (no wind movement)

**Placement Checklist:**
```
Tag ID: ___
Location: _____________________
Height above ground: _____ m
Facing direction: _____ degrees
Mounted on: _____________________
Clear visibility: [ ] Yes [ ] No
Photo reference: IMG______.jpg
```

**Acceptance Criteria:**
- [ ] All tags placed according to strategy
- [ ] Placement documented with photos
- [ ] No physical obstructions in camera field of view

---

### Task 2.2: Measure AR Tag Corner Positions

**Description:**
Accurately measure the 3D coordinates of each AR tag's four corners in the global coordinate frame.

**Equipment:**
- Laser distance meter
- Clipboard with measurement forms
- Camera for reference photos
- Leveling tool

**For Each Tag:**

1. **Identify Corners:**
   ```
   Looking at tag front:

   Corner 2 -------- Corner 3
       |                |
       |    TAG ID X    |
       |                |
   Corner 1 -------- Corner 4

   Order: Counter-clockwise starting from bottom-left
   ```

2. **Measure Coordinates:**
   - From origin point to each corner
   - Record X, Y, Z in meters
   - Precision: ±0.01m (1cm)

3. **Measurement Template:**
   ```yaml
   tag_id: 0
   marker_type: apriltag_16h5
   size: 0.6  # meters
   corners:
     corner_1:  # Bottom-left
       x: 2.35
       y: 5.12
       z: 1.80
     corner_2:  # Top-left
       x: 2.35
       y: 5.12
       z: 2.40
     corner_3:  # Top-right
       x: 2.95
       y: 5.12
       z: 2.40
     corner_4:  # Bottom-right
       x: 2.95
       y: 5.12
       z: 1.80
   ```

4. **Validation:**
   - Check distances between corners match tag size
   - Verify counter-clockwise order
   - Confirm Z values increase from bottom to top

**Acceptance Criteria:**
- [ ] All tag corners measured
- [ ] Measurements validated for consistency
- [ ] Data recorded in structured format (YAML or spreadsheet)

---

### Task 2.3: Create Lanelet2 Map with AR Tags

**Description:**
Convert measurement data into Lanelet2 OSM XML format for use by AR tag localizer.

**Option 1: Manual OSM Editing (Small Maps)**

Create file: `data/ar_tag_test_map/lanelet2_map.osm`

```xml
<?xml version='1.0' encoding='UTF-8'?>
<osm version='0.6'>
  <!-- Meta information -->
  <MetaInfo format_version="1.0"/>

  <!-- Node definitions for Tag 0 -->
  <node id="1000" lat="25.0xxxxx" lon="121.5xxxxx">
    <tag k="mgrs_code" v="51RUQ12345678"/>
    <tag k="local_x" v="2.35"/>
    <tag k="local_y" v="5.12"/>
    <tag k="ele" v="1.80"/>
  </node>
  <node id="1001" lat="25.0xxxxx" lon="121.5xxxxx">
    <tag k="local_x" v="2.35"/>
    <tag k="local_y" v="5.12"/>
    <tag k="ele" v="2.40"/>
  </node>
  <node id="1002" lat="25.0xxxxx" lon="121.5xxxxx">
    <tag k="local_x" v="2.95"/>
    <tag k="local_y" v="5.12"/>
    <tag k="ele" v="2.40"/>
  </node>
  <node id="1003" lat="25.0xxxxx" lon="121.5xxxxx">
    <tag k="local_x" v="2.95"/>
    <tag k="local_y" v="5.12"/>
    <tag k="ele" v="1.80"/>
  </node>

  <!-- Polygon for Tag 0 -->
  <way id="2000">
    <nd ref="1000"/>
    <nd ref="1001"/>
    <nd ref="1002"/>
    <nd ref="1003"/>
    <tag k="type" v="pose_marker"/>
    <tag k="subtype" v="apriltag_16h5"/>
    <tag k="area" v="yes"/>
    <tag k="marker_id" v="0"/>
  </way>

  <!-- Repeat for tags 1-6 with unique node IDs (1004-1027) and way IDs (2001-2006) -->

</osm>
```

**Option 2: Python Map Generator Script**

Create `scripts/tools/generate_ar_tag_map.py`:

```python
#!/usr/bin/env python3
"""Generate Lanelet2 map with AR tag landmarks from measurement data."""

import yaml
import xml.etree.ElementTree as ET
from xml.dom import minidom

def generate_map(measurement_file, output_file):
    # Load measurements
    with open(measurement_file, 'r') as f:
        data = yaml.safe_load(f)

    # Create OSM structure
    osm = ET.Element('osm', version='0.6')

    # Meta information
    meta = ET.SubElement(osm, 'MetaInfo', format_version='1.0')

    node_id = 1000
    way_id = 2000

    for tag in data['tags']:
        # Create 4 corner nodes
        corner_ids = []
        for corner_name in ['corner_1', 'corner_2', 'corner_3', 'corner_4']:
            corner = tag['corners'][corner_name]
            node = ET.SubElement(osm, 'node', id=str(node_id),
                                 lat='0.0', lon='0.0')  # Dummy lat/lon
            ET.SubElement(node, 'tag', k='local_x', v=str(corner['x']))
            ET.SubElement(node, 'tag', k='local_y', v=str(corner['y']))
            ET.SubElement(node, 'tag', k='ele', v=str(corner['z']))
            corner_ids.append(node_id)
            node_id += 1

        # Create way (polygon)
        way = ET.SubElement(osm, 'way', id=str(way_id))
        for cid in corner_ids:
            ET.SubElement(way, 'nd', ref=str(cid))
        ET.SubElement(way, 'tag', k='type', v='pose_marker')
        ET.SubElement(way, 'tag', k='subtype', v='apriltag_16h5')
        ET.SubElement(way, 'tag', k='area', v='yes')
        ET.SubElement(way, 'tag', k='marker_id', v=str(tag['tag_id']))
        way_id += 1

    # Pretty print
    xml_str = minidom.parseString(ET.tostring(osm)).toprettyxml(indent='  ')

    # Write to file
    with open(output_file, 'w') as f:
        f.write(xml_str)

    print(f"Generated map: {output_file}")

if __name__ == '__main__':
    generate_map('ar_tag_measurements.yaml', 'data/ar_tag_test_map/lanelet2_map.osm')
```

**Usage:**
```bash
# Create measurement YAML file
vim ar_tag_measurements.yaml

# Run map generator
python3 scripts/tools/generate_ar_tag_map.py

# Verify output
cat data/ar_tag_test_map/lanelet2_map.osm
```

**Acceptance Criteria:**
- [ ] Lanelet2 .osm file created
- [ ] All AR tags included with correct IDs
- [ ] Corner coordinates match measurements
- [ ] Map validates with Lanelet2 tools (if available)

---

## Deliverables

- [ ] Lanelet2 map with AR tag landmarks (`data/ar_tag_test_map/lanelet2_map.osm`)
- [ ] Tag placement documentation (with photos)
- [ ] Coordinate measurement spreadsheet or YAML file

---

## Testing

**Verification Steps:**
1. Visually inspect all tag placements (no obstructions, stable mounting)
2. Validate measurement accuracy (check distances between corners)
3. Load Lanelet2 map in RViz to visualize tag positions
4. Cross-check tag IDs in map match physical tag labels

---

## Next Phase

Once the AR tag map is complete and validated, proceed to [Phase 3: AR Tag Localizer Integration](phase_3_localizer.md).
