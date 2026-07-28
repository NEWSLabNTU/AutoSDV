# Using 2-D MCL: the target setup experience

Supersedes the adapter placement in
[`mcl-scan-input-and-sensor-configurability.md`](mcl-scan-input-and-sensor-configurability.md).
That draft put scan production (z-slab flattening, ring extraction) inside
`autosdv_mcl_launch`. It belongs in the **sensor kit** instead: which ring of a
Velodyne represents the useful horizontal plane depends on where that LiDAR sits
on that vehicle, which is precisely what a sensor kit describes. MCL should not
know.

This document defines the boundary and walks the setup a user actually performs.

---

## 1. The boundary

```
  ┌─────────────────── user's sensor kit ───────────────────┐
  │  Hokuyo driver ──────────────────────────► LaserScan    │
  │                                                          │
  │  Velodyne driver ─► ring extractor ──────► LaserScan    │
  │  (user picks the ring: depends on vehicle                │
  │   height and LiDAR mounting)                             │
  └──────────────────────────┬───────────────────────────────┘
                             │  /scan  (sensor_msgs/LaserScan)
                             │  + TF: base_link → <scan frame>
                             ▼
  ┌─────────────────── autosdv_mcl_launch ──────────────────┐
  │  scan normaliser ─► particle_filter ─► mcl_pose_relay   │
  └──────────────────────────┬───────────────────────────────┘
                             │  /localization/pose_estimator/pose_with_covariance
                             ▼
                        ekf_localizer  (unchanged)
```

**The kit owns**: the driver, the choice of which physical plane to use, ring
selection, and publishing one `LaserScan`.

**MCL owns**: consuming that scan, matching it against the occupancy grid, and
publishing the Autoware pose contract.

The contract between them is deliberately thin:

> **`/scan`** — `sensor_msgs/LaserScan`, in **any frame** that TF connects to
> `base_link`, RELIABLE QoS. MCL reads `header.frame_id` and resolves the
> mounting offset itself.

Two consequences worth stating, because they are the reason for the design:

- **The user never configures a z-band.** Today's `scan_min_height` /
  `scan_max_height` are the sample kit's `velodyne_top` height and would select
  an empty slab on the AutoSDV vehicle (kit z = 0.0 versus sample 2.0). Once the
  kit produces the scan, that whole class of mismatch disappears from MCL.
- **MCL must handle the mounting offset**, not the kit. `particle_filter.py`
  treats the scan as originating at the particle pose, so a scan in a laser
  frame 0.5 m forward of `base_link` would bias every range by 0.5 m. Requiring
  the kit to pre-flatten into `base_link` would push that subtlety onto every
  kit author; doing it once inside MCL (a `laser_geometry` re-projection, §5.1)
  is both correct and invisible.

## 2. What a user does, start to finish

Bringing MCL up on a new vehicle is four steps, only two of which involve
thought.

### Step 1 — publish a 2-D scan from the kit

Whatever the sensor, end at one `LaserScan`. The kit's existing `lidar.launch.xml`
already dispatches on `lidar_model`, so this is the natural home:

```xml
<!-- native 2-D: the driver already does it -->
<group if="$(eval &quot;'$(var lidar_model)'=='hokuyo'&quot;)">
  <include file="$(find-pkg-share urg_node)/launch/urg_node.launch.py"/>
</group>

<!-- 3-D + ring extraction: user picks the ring for their mounting -->
<group if="$(eval &quot;'$(var lidar_model)'=='vlp32c'&quot;)">
  <include file=".../velodyne_launch_all_hw.xml"/>
  <include file="$(find-pkg-share autosdv_sensor_kit_launch)/launch/scan_from_ring.launch.xml">
    <arg name="ring" value="$(var scan_ring)"/>
  </include>
</group>
```

Choosing the ring is the one judgement here, and it is the user's: it depends on
vehicle height and where the LiDAR sits. The kit should help rather than make the
user guess — see §4.

**Ring extraction applies to spinning LiDARs only.** It assumes
constant-elevation rings. Solid-state sensors with restricted FOV (Robin-W,
Cube1 in this kit) have no such structure, and a narrow wedge constrains the
filter poorly against a 360 deg grid regardless; they belong on the 3-D NDT path
or alongside a native 2-D LiDAR.

**Measured: use a small ring GROUP, not a single ring.** Against NDT ground truth
on the sample site (`docs/reports/2dlidar-scan-source-comparison.md`), three
adjacent channels beat both one ring and the thicker z-slab on every metric —
mean 0.789 m against 0.992 and 0.821, seed spread 0.037 m against 0.317 and
0.072, and mean |yaw| 0.0159 rad against 0.0321 and 0.0339. A single ring is
geometrically a perfect plane but too sparse on a 128-ring spinner: two of five
seeds missed the 1.0 m gate. Three rings span 0.22° here, which is *tighter than
the 0.30 m slab* beyond 30 m, so the group is both denser and more faithful. The
group is sensor-specific and must be measured: 0.11° channel spacing is a
property of this VLS128.

### Step 2 — build the occupancy grid

Already supported:

```bash
# from an existing PCD map
just map-grid-from-pcd data/my_site                       # prints z distribution + suggested band
just map-grid-from-pcd data/my_site --z-min 9.1 --z-max 9.4

# or accumulate scans from a recorded drive
just map-grid-from-bag my_run.bag data/my_site
```

Note the asymmetry, which is intentional: the **map** band is a property of the
site (what height counts as an obstacle), so the user chooses it. The **scan**
band is a property of the mounting, which the kit now owns.

### Step 3 — validate before launching

```bash
just map-check data/my_site mcl
```

Reports the grid, the lanelet2 map, and the frame-extent agreement between them.

### Step 4 — launch

```bash
just launch pose_source:=mcl map_path:=data/my_site
```

No scan geometry arguments. If the kit publishes `/scan` and TF is right, this
is the whole invocation.

## 3. What MCL should tell the user when it is wrong

Every failure in this project's history was silent: an empty scan, an empty crop
box, a missing grid. The setup experience is only as good as its diagnostics, so
MCL should refuse to run quietly.

| condition | what the user should see |
|---|---|
| no `/scan` publisher after N s | `waiting for /scan — the sensor kit is expected to publish it (lidar_model=<x>)` |
| `/scan` arrives, TF `base_link → <frame>` missing | `/scan is in frame 'laser' but no TF connects it to base_link; check the kit's calibration` |
| every range non-finite | `/scan carries N beams, all non-finite — ring selection or range limits are likely wrong` |
| scan and grid disagree in scale | `scan max range 60 m but the grid spans 12 m; wrong map?` |
| grid never becomes ACTIVE | `map_server did not activate; run just map-check <dir> mcl` |

The first three are the ones that would have saved days on this project.

## 4. Helping the user pick the ring

Ring choice is the user's call, but it should be an informed one. A small
offline tool, in the kit rather than in MCL:

```bash
$ ros2 run autosdv_sensor_kit_launch inspect_rings --bag my_drive.bag
channel  median elevation  points  median range
   14          -1.9°       41203       18.2 m
   15          -0.9°       44117       23.6 m
   16          +0.1°       45882       31.4 m   <- nearest horizontal
   17          +1.1°       43904       28.7 m
```

This is measurable, not guesswork: the decoded cloud carries a per-point
`elevation` field alongside `channel` (verified on
`data/rosbags/phase3/sample_ndt_gt`), so the horizontal ring can be identified
directly rather than inferred from the datasheet's non-uniform layout.

Also worth surfacing: for a vehicle whose LiDAR sits at height *h*, the ring
nearest horizontal sees the ground at range *h / tan(elevation)*. At *h* = 0.3 m
and 1° that is 17 m — beyond which the ring is looking at the sky and returns
nothing. A tool that prints this saves the user from selecting a ring that is
geometrically useless on a low vehicle.

## 5. Revision work

1. **Move scan production out of MCL into the kit.**
   Delete `pointcloud_to_laserscan`, `scan_min_height`, `scan_max_height`,
   `scan_range_max`, and `mcl_input_pointcloud` from `mcl_localization.launch.xml`.
   Add `scan_from_ring.launch.xml` (ring filter + flattening) to
   `autosdv_sensor_kit_launch`, dispatched from `lidar.launch.xml` by
   `lidar_model`, with the ring as a kit argument.

   *Breaking*: any caller passing the removed arguments. The five-seed accuracy
   matrix and `run-particle-filter.sh` both pass `SCAN_MIN_HEIGHT`, so they
   change with it.

2. **Normalise the scan frame inside MCL.**
   A `scan_normaliser` node: `LaserScan` → `laser_geometry` projection →
   transform to `base_link` → re-flatten → `/scan_base_link`, which
   `particle_filter` consumes. Removes the lever-arm error and lets the kit
   publish in its natural frame. Skip the work when `frame_id` is already
   `base_link`.

3. **The diagnostics in §3**, as a `scan_watchdog` in MCL. Cheap, and it is what
   converts a silent empty scan into a named failure.

4. **`inspect_rings` tool** in the kit (§4).

5. **A `mcl` section in the book**: the four steps of §2, the contract, and the
   diagnostics table.

## 6. Open questions

- **Does the scan need to be one full revolution?** `particle_filter` uses
  `angle_min`/`angle_increment` and downsamples; a 270° Hokuyo should work, but
  the map-wide initialisation assumes reasonable angular coverage and this has
  never been tested below 360°.
- **What rate does MCL need?** The sample runs are ~5 Hz decoded. A Hokuyo at
  40 Hz would change the resampling dynamics, possibly for the better, but
  `update_on_new_scan_only` and the ESS gate were tuned at the low rate.
- **Should the kit publish `/scan` or a namespaced topic?**
  `particle_filter.py` hardcodes absolute `/scan`, `/pf/...` and
  `/map_server/map`, which already forced an unnamespaced MCL group. Fixing that
  in the fork would let the contract be `<ns>/scan` and remove a wart, but it is
  a fork change with its own risk.
