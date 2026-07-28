# 2-D MCL: unified scan input Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** MCL consumes one `sensor_msgs/LaserScan` from the sensor kit, in any TF-connected frame, with no scan geometry configuration of its own.

**Architecture:** The kit owns the driver and the choice of physical plane (ring selection); MCL owns matching and the pose contract. A normaliser inside MCL absorbs QoS, frame offset and diagnostics, replacing today's `pointcloud_to_laserscan` + `scan_qos_bridge` pair. A test-only adapter stays in MCL so the Autoware sample bags — whose kit publishes no scan — remain runnable.

**Tech Stack:** ROS 2 Humble, Python (rclpy), `autoware_pointcloud_preprocessor` (`PassThroughFilterUInt16Component`), `pointcloud_to_laserscan`.

## Global Constraints

- `/scan` contract: `sensor_msgs/LaserScan`, any frame TF connects to `base_link`, RELIABLE out of the normaliser.
- `particle_filter.py` treats the scan as originating at the particle pose, so the normaliser MUST re-express ranges about `base_link`. A pass-through when `frame_id == base_link` already.
- Production defaults must not change behaviour for a kit that publishes `/scan`; the test adapter defaults **off**.
- No new hardcoded sensor heights anywhere in `autosdv_mcl_launch`.
- Absolute topic names in the vendored fork (`/scan`, `/pf/...`, `/map_server/map`) stay as they are; renaming them is out of scope.
- The five-seed accuracy matrix is the regression gate: median mean < 1.0 m, p95 < 2.5 m, mean |yaw| < 0.2 rad.
- Temp files under `./tmp/`. `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` for pytest. apt numpy only.

---

### Task 1: `mcl_scan_normalizer` — pure geometry

**Files:**
- Create: `src/localization/autosdv_mcl_launch/autosdv_mcl_launch/scan_normalizer.py`
- Test: `src/localization/autosdv_mcl_launch/test/test_scan_normalizer.py`

**Interfaces:**
- Consumes: nothing (pure functions).
- Produces: `planar_offset(translation, rotation_quat) -> (dx, dy, dyaw, tilt_rad)`;
  `retarget_ranges(ranges, angle_min, angle_increment, offset, out_spec) -> list[float]`;
  `ScanSpec(angle_min, angle_max, angle_increment, range_min, range_max)`.

The geometry: a beam at index *i* in the sensor frame ends at
`p_L = (r·cosθ, r·sinθ)` with `θ = angle_min + i·angle_increment`. With a planar
offset `(dx, dy, dyaw)` from `base_link` to the sensor, the same point in
`base_link` is `p_B = R(dyaw)·p_L + (dx, dy)`, whose range and bearing are
`hypot(p_B)` and `atan2(p_B)`. Re-bin those into the output spec, keeping the
**minimum** range per bin (a nearer return occludes a farther one).

- [ ] **Step 1: Write the failing tests**

```python
def test_identity_offset_is_passthrough():
    spec = ScanSpec(-math.pi, math.pi, 0.01, 0.1, 60.0)
    ranges = [5.0] * 629
    out = retarget_ranges(ranges, -math.pi, 0.01, (0.0, 0.0, 0.0, 0.0), spec)
    finite = [r for r in out if math.isfinite(r)]
    assert finite and all(abs(r - 5.0) < 1e-6 for r in finite)

def test_pure_translation_shifts_ranges():
    """Sensor 1 m forward: a return 5 m dead ahead is 6 m from base_link."""
    spec = ScanSpec(-math.pi, math.pi, 0.01, 0.1, 60.0)
    n = int((spec.angle_max - spec.angle_min) / spec.angle_increment) + 1
    ranges = [float('inf')] * n
    ranges[n // 2] = 5.0                      # beam at angle 0
    out = retarget_ranges(ranges, spec.angle_min, spec.angle_increment,
                          (1.0, 0.0, 0.0, 0.0), spec)
    assert min(r for r in out if math.isfinite(r)) == pytest.approx(6.0, abs=1e-6)

def test_yaw_rotates_bearing():
    """A sensor yawed +90 deg puts its forward beam on base_link's +y."""
    spec = ScanSpec(-math.pi, math.pi, 0.01, 0.1, 60.0)
    n = int((spec.angle_max - spec.angle_min) / spec.angle_increment) + 1
    ranges = [float('inf')] * n
    ranges[n // 2] = 5.0
    out = retarget_ranges(ranges, spec.angle_min, spec.angle_increment,
                          (0.0, 0.0, math.pi / 2, 0.0), spec)
    idx = min((i for i, r in enumerate(out) if math.isfinite(r)),
              key=lambda i: out[i])
    bearing = spec.angle_min + idx * spec.angle_increment
    assert bearing == pytest.approx(math.pi / 2, abs=0.02)

def test_nearer_return_wins_a_shared_bin():
    spec = ScanSpec(-math.pi, math.pi, 0.5, 0.1, 60.0)   # coarse: collisions
    ranges = [10.0, 3.0] + [float('inf')] * 11
    out = retarget_ranges(ranges, -math.pi, 0.02, (0.0, 0.0, 0.0, 0.0), spec)
    assert min(r for r in out if math.isfinite(r)) == pytest.approx(3.0, abs=1e-6)

def test_out_of_range_is_dropped():
    spec = ScanSpec(-math.pi, math.pi, 0.01, 1.0, 10.0)
    out = retarget_ranges([0.5, 50.0], -math.pi, 0.01, (0.0, 0.0, 0.0, 0.0), spec)
    assert not [r for r in out if math.isfinite(r)]

def test_non_finite_input_is_ignored():
    spec = ScanSpec(-math.pi, math.pi, 0.01, 0.1, 60.0)
    out = retarget_ranges([float('nan'), float('inf'), -1.0], -math.pi, 0.01,
                          (0.0, 0.0, 0.0, 0.0), spec)
    assert not [r for r in out if math.isfinite(r)]

def test_planar_offset_reports_tilt():
    """Roll/pitch cannot be represented planar-ly; the caller must be told."""
    import math
    q = (math.sin(math.radians(5)), 0.0, 0.0, math.cos(math.radians(5)))  # 10 deg roll
    dx, dy, dyaw, tilt = planar_offset((0.0, 0.0, 0.0), q)
    assert tilt == pytest.approx(math.radians(10), abs=1e-3)
```

- [ ] **Step 2: Run them and watch them fail**

Run: `cd src/localization/autosdv_mcl_launch && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_scan_normalizer.py -v`
Expected: FAIL, `ImportError`.

- [ ] **Step 3: Implement the pure functions** — no rclpy import at module top.

- [ ] **Step 4: Tests pass.** Same command, all green.

- [ ] **Step 5: Commit.**

---

### Task 2: `mcl_scan_normalizer` — the node

**Files:**
- Modify: `src/localization/autosdv_mcl_launch/autosdv_mcl_launch/scan_normalizer.py`
- Modify: `src/localization/autosdv_mcl_launch/setup.py` (entry point)

**Interfaces:**
- Consumes: `input_topic` (default `/scan_raw`), `sensor_msgs/LaserScan`, SensorDataQoS so a BEST_EFFORT driver is accepted.
- Produces: `output_topic` (default `/scan`), RELIABLE — replacing `scan_qos_bridge`.
- Params: `base_frame` (`base_link`), `tf_timeout_s` (0.2), `warn_tilt_deg` (2.0),
  `watchdog_period_s` (5.0), plus the output `ScanSpec` fields defaulting to the
  input's own when unset.

Behaviour: look up `base_frame -> scan.header.frame_id`; when the transform is
identity within 1 mm / 0.1°, republish unchanged (cheap path). Otherwise
`retarget_ranges`. Publish with `frame_id = base_frame`.

Diagnostics — the point of the node as much as the geometry, because every
failure in this project's history was silent:

| condition | message |
|---|---|
| no input for `watchdog_period_s` | `no LaserScan on <topic> after Ns; the sensor kit is expected to publish it` |
| TF missing | `/scan is in frame '<f>' but no TF connects it to <base_frame>; check the kit calibration` |
| all ranges non-finite | `<topic> carries N beams, all non-finite; ring selection or range limits are likely wrong` |
| tilt > `warn_tilt_deg` | `sensor is tilted <x> deg; a planar re-target cannot represent roll/pitch` |

- [ ] **Step 1:** implement the node around the Task 1 functions; log each condition once (state-change triggered, not per message).
- [ ] **Step 2:** add the `mcl_scan_normalizer` console script; `colcon build --packages-select autosdv_mcl_launch`.
- [ ] **Step 3:** verify by hand against the sample bag: publish a LaserScan in a frame 1 m forward of `base_link` and confirm ranges shift by 1 m.
- [ ] **Step 4: Commit.**

---

### Task 3: rework `mcl_localization.launch.xml`

**Files:**
- Modify: `src/localization/autosdv_mcl_launch/launch/mcl_localization.launch.xml`
- Create: `src/localization/autosdv_mcl_launch/launch/scan_source_test_pointcloud.launch.xml`
- Delete: `config/mcl_pointcloud_to_laserscan.param.yaml` (moves to the test adapter)

**Interfaces:**
- New args: `scan_topic` (`/scan_raw`), `scan_source` (`external` | `test_pointcloud`).
- Removed args: `mcl_input_pointcloud`, `scan_min_height`, `scan_max_height`, `scan_range_max`, `sensor_frame`.

`scan_source:=external` (default, production) wires the kit's scan straight into
the normaliser. `scan_source:=test_pointcloud` additionally launches the old
`pointcloud_to_laserscan` chain, explicitly labelled test scaffolding, so the
Autoware sample bags stay runnable — their kit publishes no scan. This is the
same testing-versus-production split already used for the 3-D-to-2-D conversion
elsewhere in this work.

- [ ] **Step 1:** move the `pointcloud_to_laserscan` node and its param file into the test adapter, keeping the sample-kit band **there** with a comment that it is sample-specific.
- [ ] **Step 2:** replace `scan_qos_bridge` with `mcl_scan_normalizer` in the main file; drop `sensor_frame` from the relay's invocation (it is `base_link` by construction now).
- [ ] **Step 3:** `colcon build`; launch with `scan_source:=test_pointcloud` and confirm `/scan` appears with `frame_id: base_link` and the filter initialises.
- [ ] **Step 4: Commit.**

---

### Task 4: kit-side scan production

**Files:**
- Create: `src/sensor_kit/autosdv_sensor_kit_launch/autosdv_sensor_kit_launch/launch/scan_from_ring.launch.xml`
- Create: `.../config/scan_from_ring.param.yaml`
- Modify: `.../launch/lidar.launch.xml`

**Interfaces:**
- `scan_from_ring.launch.xml` args: `input_topic`, `output_topic` (`/scan_raw`), `ring`, `container`.
- `lidar.launch.xml` new args: `publish_scan` (`false`), `scan_ring` (`16`).

Chain: cloud → `PassThroughFilterUInt16Component` (`filter_field_name: channel`,
`filter_limit_min == filter_limit_max == ring`) → `pointcloud_to_laserscan` with
**generous** z limits.

The generous limits are load-bearing, not laziness: one ring is a cone, not a
plane, so its height above the sensor grows with range (±1 m at 60 m for 1°).
A tight band would silently clip the far half of the very ring just selected.
Ring selection alone does the work and is vehicle-height independent.

- [ ] **Step 1:** create the adapter; `filter_limit_min`/`max` both set to `ring`.
- [ ] **Step 2:** dispatch it from `lidar.launch.xml` under `publish_scan`, for `vlp32c` first.
- [ ] **Step 3:** verify on `data/sample-rosbag-replay/sample-rosbag-restamped` — that bag's decoded cloud carries `channel`, confirmed — that `/scan_raw` appears and is populated.
- [ ] **Step 4: Commit.**

---

### Task 5: regression gate

**Files:** modify `scripts/2dlidar/run-mcl-e2e-matrix.sh` only if the launch arguments it passes changed.

- [ ] **Step 1:** run the five-seed matrix with `scan_source:=test_pointcloud` (the configuration the gate was set on).
- [ ] **Step 2:** compare against the recorded gate: median mean 0.849 m, p95 2.173 m, yaw 0.034 rad. Any regression beyond seed spread blocks the task.
- [ ] **Step 3:** run the downstream probe once with `pose_source:=mcl` and confirm it still reaches AUTONOMOUS.
- [ ] **Step 4:** record both in `docs/reports/`, then commit.

---

### Task 6: documentation

**Files:** modify `docs/design/mcl-user-setup-ux.md` (mark implemented), `CLAUDE.md` (MCL section), `docs/design/mcl-scan-input-and-sensor-configurability.md` (cross-ref).

- [ ] **Step 1:** document the contract, the two `scan_source` values, and the diagnostics table as shipped.
- [ ] **Step 2:** note what is still unimplemented: the `inspect_rings` tool, native-2-D validation (no such bag exists), and the `laser_geometry` path for tilted mounts.
- [ ] **Step 3: Commit and push.**

---

## Out of scope

- `inspect_rings` (Task 4 of the UX doc) — useful, but not needed to ship the contract.
- Renaming the fork's absolute topics.
- Native 2-D LiDAR validation: no 2-D bag exists in the repo, so the path can be
  implemented but not verified. Do not claim it works.
- Tilted-mount `laser_geometry` re-projection; the normaliser warns instead.
