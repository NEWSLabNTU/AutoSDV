# 2-D MCL: sensor-kit configurability and a unified scan input

Two problems, one shared cause: `autosdv_mcl_launch` hardcodes the geometry of
one specific sensor on one specific vehicle.

1. **The scan band is the sample kit's sensor height.** It works on the Autoware
   sample bag and captures nothing on the AutoSDV vehicle.
2. **Only one input shape is supported** — a 3-D cloud flattened by a z-slab.
   A native 2-D LiDAR (Hokuyo) has no cloud to slab, and a Velodyne single-ring
   extraction is a cone rather than a plane, which the slab handles badly.

This explores both and lists the revision work.

---

## 1. Where the geometry is hardcoded today

`mcl_localization.launch.xml`:

```xml
<arg name="scan_min_height" default="1.91611"/>
<arg name="scan_max_height" default="2.21611"/>
```

and the same numbers again in `config/mcl_pointcloud_to_laserscan.param.yaml`.
They are the sample kit's `velodyne_top` height above `base_link`, measured
from the calibration chain:

| kit | `base_link → sensor_kit_base_link` z | lidar z within kit | lidar above `base_link` |
|---|---|---|---|
| `sample_sensor_kit` | **2.0 m** | 0.0 | ~2.0 m |
| `autosdv_sensor_kit` | **0.0 m** | 0.0 | **~0.0 m** |

The band `1.916 … 2.216` therefore selects the sample kit's sensor plane, and
on AutoSDV selects an empty slab two metres above every return. Nothing errors;
the filter simply receives an empty scan. This is the same failure mode as the
perception crop box (`vehicle_height` 0.262 versus 2.5): geometry silently
mismatched, no diagnostic.

The current numbers were correct in earlier phases **by coincidence** — the
work happened on the sample bag — and this is already flagged as a latent bug
in `docs/reports/2dlidar-downstream-planning-control.md`.

## 2. What the filter actually requires

`particle_filter.py` consumes `LaserScan.ranges` with the scan's own
`angle_min`/`angle_increment`, and broadcasts `map → /laser`. It has no notion
of a sensor offset: it treats the scan as originating **at the particle pose**,
i.e. at `base_link`. That is why `pointcloud_to_laserscan` runs with
`target_frame: base_link` — flattening in `base_link` keeps the frame-naive
sensor model correctly oriented, and `mcl_pose_relay`'s `sensor_frame` is
`base_link` so its composition is an identity.

**This constrains the whole design**: whatever the sensor, `/scan` must be
expressed in `base_link`. Handing the filter a scan in a sensor frame would
introduce a silent lever-arm error equal to the mounting offset.

## 3. Unified input contract

> `/scan` — `sensor_msgs/LaserScan`, `frame_id: base_link`, RELIABLE QoS,
> one revolution per message, ranges in metres.

Everything upstream is an **adapter** chosen by sensor type. The filter, the
relay, and the map path stay identical across sensors — only the adapter
changes. Three adapters cover the cases in play:

### 3.1 Native 2-D LiDAR (Hokuyo, and the production target)

The driver already publishes `LaserScan`, but in the **sensor's** frame, which
by §2 is not directly usable unless the sensor sits at `base_link`'s origin.

- *Cheap and wrong*: consume it directly and accept the lever arm. For a
  bumper-mounted unit ~0.5 m forward of `base_link`, that is a fixed 0.5 m bias
  in every range — an error the filter cannot distinguish from a pose error.
- *Correct*: `laser_geometry` projects the scan to a cloud, transform to
  `base_link`, re-flatten with `pointcloud_to_laserscan`. One extra hop, exact,
  and reuses the existing flattening node.
- *Correct and cheaper*: keep `/scan` in the sensor frame and set
  `mcl_pose_relay`'s `sensor_frame` to the real laser frame so the pose
  composition removes the offset. The relay already takes `sensor_frame` and
  does this composition; today it is `base_link` only because flattening
  already happened there.

The third is preferable when the sensor is mounted level (pure translation);
the second is needed when it is mounted with roll/pitch, since a tilted 2-D
scan is not a horizontal plane at all.

### 3.2 3-D LiDAR, z-band flattening (what exists)

Keep it, but derive the band from the sensor kit (§4) instead of hardcoding.
Appropriate when the 3-D sensor is the only LiDAR and a synthetic plane is
acceptable — the current testing configuration.

### 3.3 3-D LiDAR, single-ring extraction (the Velodyne case)

Autoware already ships the tool: `autoware_pointcloud_preprocessor`'s
`PassThroughFilterUInt16Component`, used as `ring_filter` by
`lidar_marker_localizer`, filters a `uint16` field by range:

```yaml
filter_field_name: "channel"   # "ring" on some drivers
filter_limit_min: 5
filter_limit_max: 5            # a single ring
```

Chain: cloud → `ring_filter` (one ring) → `pointcloud_to_laserscan` → `/scan`.

**Why this is better than a z-slab for a Velodyne**, and the subtlety that
matters: a single ring is a **cone**, not a plane. Ring *k* leaves the sensor at
a fixed elevation angle, so its height above `base_link` grows with range — a
1° ring is ±1 m in z at 60 m. A z-band tight enough to isolate one ring near
the sensor will clip that same ring at distance, and a band loose enough to
keep it will admit neighbouring rings. So:

- in **ring mode**, select the ring and set the z limits *generously* (or
  disable them); ring selection alone does the work, and it is
  vehicle-height-independent by construction;
- in **slab mode**, the band does the work and must match the mounting height.

Mixing a tight band with ring selection is the one combination that silently
loses far returns.

Ring mode is also the faithful emulation of a 2-D LiDAR: a real Hokuyo *is* a
single elevation, so testing on the Velodyne bags with ring extraction
exercises the production geometry far better than a slab does.

## 4. Deriving the band, four options

| option | how | verdict |
|---|---|---|
| **A. Per-kit param file** selected by `sensor_model`, mirroring AutoSDV's existing preset pattern | `config/scan/<sensor_model>.param.yaml` | **Recommended.** Explicit, inspectable, no runtime ordering. Duplicates a number per kit, which is how Autoware handles kit config anyway. |
| B. Resolve from TF at runtime | a node looks up `base_link → <lidar>` and sets the params | Params must exist before `pointcloud_to_laserscan` starts; TF is not up that early. Ordering problem for little gain. |
| C. Compute at launch from the calibration YAMLs | `$(command resolve_scan_band.py …)` | Same source of truth TF is built from, no ordering problem — but `play_launch` warns on command substitutions, and each kit's YAML is shaped differently. |
| D. Flatten in the sensor frame instead | `target_frame: <lidar frame>`, band ±0.15 m | Vehicle-independent, but violates §2 unless the relay compensates. Viable *combined* with the §3.1 relay composition; not on its own. |

**Recommendation: A, plus a runtime check.** The check is the part that stops
this recurring: at startup, look up `base_link → <lidar frame>`, compare the
sensor's z against the configured band, and log an error when they do not
intersect. That converts today's silent empty-scan into a named failure. The
same check belongs on the perception crop box, which failed identically.

## 5. Revision work

Ordered by value per unit of risk.

1. **Band as per-kit config, with a mismatch check.**
   `mcl_localization.launch.xml`, `config/mcl_pointcloud_to_laserscan.param.yaml`,
   new `config/scan/{sample,autosdv}_sensor_kit.param.yaml`, plus a small
   validator node or a startup check inside `scan_qos_bridge`. Removes the
   latent AutoSDV breakage and makes the failure loud.

2. **Ring-extraction adapter for 3-D LiDARs.**
   New `launch/scan_source_ring.launch.xml` inserting
   `PassThroughFilterUInt16Component` before `pointcloud_to_laserscan`, with
   generous z limits per §3.3. Testable immediately on the sample bags, which
   carry three Velodynes. Needs the field name checked per driver
   (`channel` vs `ring`).

3. **`scan_source` selector argument.**
   `scan_source:={pointcloud_slab,pointcloud_ring,native_2d}` in
   `mcl_localization.launch.xml`, including the adapter by name. Same dispatch
   shape as `pose_source`, so it will read as idiomatic here.

4. **Native 2-D adapter.**
   `launch/scan_source_native.launch.xml`: QoS bridge only for a level mount,
   with the relay's `sensor_frame` set to the laser frame; `laser_geometry`
   re-projection for a tilted mount. This is the production path and is
   currently untested end to end — no 2-D LiDAR bag exists in the repo.

5. **Drop `mcl_input_pointcloud`'s sample-specific default.**
   It defaults to `/sensing/lidar/top/pointcloud_raw_ex`, another sample-kit
   assumption; it should follow the selected kit.

## 6. What to verify, and how

- **Slab versus ring on the same bag.** Run the existing five-seed accuracy
  matrix under both adapters on the sample site. Ring mode should not be worse;
  if it is materially better, that is an argument for making it the default for
  3-D sensors, and it also predicts production behaviour more honestly.
- **AutoSDV-kit band.** Any COSS bag currently produces an empty scan under the
  hardcoded band. After item 1 it should produce a populated scan; that alone
  is the regression test.
- **The mismatch check fires.** Deliberately configure a band that misses the
  sensor and confirm the error names the sensor height and the band, rather
  than the run merely producing nothing.

## 7. Open questions

- ~~Does the Velodyne driver publish `channel` or `ring`?~~ **Answered:** the
  decoded cloud in `data/rosbags/phase3/sample_ndt_gt` carries
  `[x, y, z, intensity, return_type, channel, azimuth, elevation, distance,
  time_stamp]`, so `filter_field_name: "channel"` is correct for this data and
  matches `lidar_marker_localizer`. It also carries `elevation` directly, which
  makes picking the horizontal ring measurable rather than guessed (see the next
  question). The Robin-W and Cube1 drivers still need checking.
- Which ring corresponds to horizontal for the VLP-32C's non-uniform elevation
  layout? It is not the middle index, and picking wrong tilts the synthetic
  scan plane. Since the cloud carries a per-point `elevation` field, this can be
  resolved from the data: histogram `elevation` by `channel` and take the
  channel whose median elevation is nearest zero.
- For a real 2-D LiDAR mounted low, is the ground plane inside `range_min`?
  A Hokuyo at 0.2 m looking level clips very little, but any pitch puts ground
  returns in the scan, which the filter would treat as obstacles.
