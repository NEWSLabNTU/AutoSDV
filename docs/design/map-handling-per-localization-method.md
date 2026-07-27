# Map Handling Per Localization Method

Target design for making `map_path` mean the right thing when `pose_source` is
`mcl` (2-D occupancy grid) instead of `ndt`/`cuda_ndt` (3-D PCD), plus the
map-building UX that has to come with it.

Status: design. Nothing here is implemented yet.

Related: `docs/design/localization-method-switching.md` (how `pose_source`
dispatches), `docs/research/localization/2d_mcl_algorithm.md`,
`docs/research/localization/mcl_initialization_and_covariance.md`.

---

## 1. What the map layer does today

`map_path` is forwarded to **stock** Autoware
(`/opt/autoware/1.5.0/share/autoware_launch/launch/components/tier4_map_component.launch.xml`),
which concatenates four fixed filenames and starts five nodes under `/map`:

| node | reads | publishes | conditional? |
|---|---|---|---|
| `pointcloud_map_loader` | `<map_path>/pointcloud_map.pcd`, `pointcloud_map_metadata.yaml` | `/map/pointcloud_map`, `get_differential_pointcloud_map` service | **no** |
| `lanelet2_map_loader` | `<map_path>/lanelet2_map.osm` | `/map/vector_map` | only inside the projector-info callback |
| `map_projection_loader` | `<map_path>/map_projector_info.yaml` | `/map/map_projector_info` | no |
| `lanelet2_map_visualization`, `vector_map_tf_generator`, `map_hash_generator` | — | markers, `map`→`viewer` TF, map hash | no |

Three facts that shape everything below:

1. **The PCD load is mandatory and fatal.** `pointcloud_map_loader` has no
   `if=` guard and no disable parameter. With a `pointcloud_map_metadata.yaml`
   present (the COSS layout) a missing PCD throws
   `"Missing PCD segments. Exiting map loader..."`, which kills the node and
   errors out `map_container`. `scripts/map/generate_dummy_pcd.py` exists purely
   to feed this requirement.
2. **The projector info gates the vector map.** `lanelet2_map_loader` does all
   loading inside `on_map_projector_info()`. No projector info ⇒ `/map/vector_map`
   never appears ⇒ planning never starts. This is why a projection file is still
   needed for `mcl`, even though the grid itself never consumes it.
3. **AutoSDV's `config/map/*.param.yaml` are dead.** The include at
   `autosdv_autoware.launch.xml:93` passes no arguments, so the stock configs
   under `autoware_launch` are what actually load.

Dropping the PCD also disturbs two things outside localization:

- `voxel_based_compare_map_filter` in perception, enabled because
  `use_pointcloud_map` defaults to `true`.

  **Correction (Phase 5 Task 2):** an earlier draft of this document claimed the
  presets' `use_pointcloud_map` was never forwarded, since
  `autosdv_autoware.launch.xml` passes only `data_path` and
  `pointcloud_container_name` to the perception component. That did not
  reproduce — ROS 2's *global* launch configuration already carries the preset
  value through for the real entry points, so the value does take effect. The
  demonstrable problem is different and specific to this work: with the default
  `lidar_only` preset (`use_pointcloud_map: true`) and `pose_source:=mcl`, the
  compare-map filter still attempts to load and **hangs mid-construction**
  waiting for a map that will never arrive. The fix is therefore to tie
  `use_pointcloud_map` to `pose_source` (forced `false` for `mcl`) rather than to
  repair a forwarding chain that was not broken.
- `map_height_fitter`, used by `pose_initializer` and the RViz initial-pose
  adaptor to snap a 2-D click onto the PCD surface. For a planar filter the
  height is meaningless, so this should be bypassed rather than fed.

## 2. Map directory: one concept, method-dependent contents

`map_path` keeps its meaning — "a map directory" — and its contents vary:

```
my_site/                          NDT / CUDA NDT        MCL
├── lanelet2_map.osm              required              required
├── map_projector_info.yaml       required              required
├── pointcloud_map.pcd            required              not used
├── pointcloud_map_metadata.yaml  required              not used
├── occupancy_grid.pgm            —                     required
├── occupancy_grid.yaml           —                     required
└── autosdv_map.yaml              optional              optional   (see §4)
```

A site can hold both sets, and the same directory then serves either method —
which is the normal case, because the recommended way to build the grid is to
slice the PCD you already have.

**Loader behaviour**, driven by the resolved `pose_source`:

| `pose_source` | PCD loader | grid server | lanelet2 + projector |
|---|---|---|---|
| `cuda_ndt`, `ndt` | on | off | on |
| `mcl` | **off** | **on** | on |
| `isaac`, `visual` | on (unchanged) | off | on |

This requires an AutoSDV-owned map component, because the stock one cannot skip
the PCD. That is consistent with what the repo already does for localization
(`src/localization/tier4_localization_launch` is already a fork). Concretely:
`src/launcher/autosdv_launch/launch/components/autosdv_map_component.launch.xml`,
which conditionally includes the stock `map.launch.xml` pieces and adds a
`nav2_map_server` for the grid, and which finally makes AutoSDV's
`config/map/*.param.yaml` live instead of dead.

The same switch must forward `use_pointcloud_map:=false` into the perception
component when there is no PCD, closing the gap noted in §1.

## 3. Do we need to revise the projection file? No — and here is why

`map_projector_info.yaml` already expresses exactly the axis that matters, via
`projector_type`:

| mode | `projector_type` | lanelet2 geometry | GNSS init | typical use |
|---|---|---|---|---|
| georeferenced | `MGRS`, `TransverseMercator`, `LocalCartesian(UTM)` | lat/lon, projected into `map` at load | works | outdoor sites, our sample + COSS maps |
| local | `Local` | `local_x`/`local_y` tags used directly | **unavailable** | indoor, private sites, survey-only maps |

Verified in the loader source: for `Local` the parser does literally nothing
(`;  // do nothing`) — no origin required — and `lanelet2_map_loader` switches to
`lanelet2_local_projector.hpp`, reading the local tags. So "support both, specify
it in the projection file" is already how it works; no schema change is needed.

**And the schema should not be extended.** The parser reads named keys and
ignores everything else, so AutoSDV keys added here would load silently and mean
nothing to Autoware — a file that looks authoritative while half its content is
unenforced. `map_projector_info.yaml` also mirrors a fixed ROS message
(`autoware_map_msgs/MapProjectorInfo`). Keep it pure Autoware; put AutoSDV
metadata in its own sidecar (§4).

### The frame rule that actually matters

The grid's `origin` in `occupancy_grid.yaml` is expressed **in the `map` frame,
in metres**, and the `map` frame is defined by the projector. Therefore:

- **Grid sliced from the PCD** (`pcd_to_pgm.py`): frame-correct by construction,
  because the PCD is already in `map`. Works with any `projector_type`. This is
  why the slice path is recommended.
- **Grid built by survey** (SLAM): the SLAM frame is arbitrary. Two honest
  options — georeference it (solve the transform, bake it into the grid origin,
  keep `MGRS`/`TransverseMercator`, GNSS init keeps working), or declare the
  SLAM frame *to be* the map frame (`projector_type: Local`, author lanelet2 in
  matching `local_x`/`local_y`, give up GNSS init and ADAPI lat/lon).

A mismatch here is silent and catastrophic — it is the class of bug that cost
this project several phases (see the Phase 3 reports). Hence §4.

## 4. `autosdv_map.yaml` — a small sidecar, and a check command

Autoware models the projection but not "which geometry maps does this directory
contain, and are they mutually consistent". A sidecar covers that:

```yaml
# my_site/autosdv_map.yaml — optional, AutoSDV-only
geometry:
  pointcloud: pointcloud_map.pcd        # omit if absent
  occupancy_grid: occupancy_grid.yaml   # omit if absent
occupancy_grid_provenance:
  method: pcd_slice                     # pcd_slice | scan_accumulation | slam
  source: pointcloud_map.pcd
  z_band: [9.1, 9.4]                    # metres, map frame
  resolution: 0.05
  frame_consistent_with_lanelet2: true  # asserted by the tool that built it
```

Its only job is to let tooling answer questions a user cannot answer by looking
at two binary files. It is optional: absent means "infer from what is on disk".

**The UX payoff is one command**, which is where the real value is:

```bash
$ just map-check data/COSS-map-planning pose_source=mcl
map: data/COSS-map-planning
  lanelet2_map.osm            ok    (4.2 MB)
  map_projector_info.yaml     ok    TransverseMercator @ 25.0201, 121.5423 — georeferenced, GNSS init available
  occupancy_grid.yaml         ok    2601x1502 @ 0.05 m, origin [-65.000, -25.000]
  occupancy_grid.pgm          ok    82915 occupied / 483465 free / 3340322 unknown
  grid vs lanelet2 extent     ok    grid covers 100% of the lanelet2 bounding box
  pointcloud_map.pcd          -     not required for pose_source=mcl
READY for pose_source:=mcl
```

and, when it is wrong, an error that names the fix rather than a stack trace:

```
  grid vs lanelet2 extent     FAIL  lanelet2 spans x[89512,89604] y[42276,42416]
                                    but the grid covers x[-65,65] y[-25,50]
                                    -> the grid is not in the map frame. Either rebuild it from
                                       the PCD (just map-grid-from-pcd), or set
                                       projector_type: Local and author lanelet2 in local coords.
```

That single check is the difference between "2D-MCL is 30 m off and nobody knows
why" and "your grid is in the wrong frame, here is what to do".

## 5. Building the grid — two documented paths

### 5.1 From an existing PCD (recommended)

```bash
just map-grid-from-pcd data/COSS-map-planning --z-min 9.1 --z-max 9.4
```

Wraps `scripts/map/pcd_to_pgm.py`, writes `occupancy_grid.{pgm,yaml}` into the
map directory, updates `autosdv_map.yaml`, then runs `map-check`.

The one judgement the user must make is the z-band: a slice at the 2-D scanner's
mounting height, expressed in **map-frame** metres, not height above ground. The
recipe should print the cloud's z distribution and the derived ground level to
make that choice concrete rather than a guess — getting it wrong is what produced
a 230-cell unusable grid in Phase 1.

### 5.2 From a survey

Two sub-paths, matching §3:

```bash
# a) accumulate scans at known poses (needs a localized run — e.g. an NDT bag)
just map-grid-from-bag my_run.bag data/my_site --min-hits 1 --resolution 0.05

# b) SLAM a new site with no prior map  (slam_toolbox — NOT yet implemented)
just map-survey                      # teleop + slam_toolbox, saves grid + posegraph
```

Path (a) exists today as `scripts/2dlidar/scan_accumulate_grid.py` and is
frame-correct because it uses map-frame ground-truth poses. Path (b) is currently
**documentation only** — `slam_toolbox` appears nowhere in the tree. Whoever
implements it owns the frame decision from §3: georeference the result, or commit
to `projector_type: Local`.

## 6. Target usage, end to end

```bash
# existing site, already has a PCD map: add a grid and switch method
just map-grid-from-pcd data/COSS-map-planning --z-min 9.1 --z-max 9.4
just map-check          data/COSS-map-planning pose_source=mcl
just launch ARGS="pose_source:=mcl"                       # map_path default
just launch ARGS="pose_source:=cuda_ndt"                  # same directory, 3-D path

# a different site
just launch ARGS="pose_source:=mcl map_path:=/data/my_site"

# replay
just launch-sim-logging ARGS="pose_source:=mcl"
```

`map_path` never changes meaning; `pose_source` decides which geometry map inside
it gets loaded. A user who only ever runs NDT sees no change at all.

## 7. Work items, in dependency order

1. **`autosdv_map_component.launch.xml`** — conditional PCD, conditional
   `nav2_map_server`, lanelet2 + projector always; wire AutoSDV's dead
   `config/map/*.param.yaml`. Unblocks everything else.
2. **Forward `use_pointcloud_map`** from the perception preset (closes the
   pre-existing wiring gap) and set it `false` for `mcl`.
3. **Bypass `map_height_fitter`** for `mcl`, since RViz initial-pose and GNSS
   init currently snap height against the PCD.
4. **`just map-check`** with the frame-extent comparison of §4 — highest UX value
   per line of code, and it is what prevents silent frame mismatches.
5. **`just map-grid-from-pcd` / `map-grid-from-bag`** wrappers plus
   `autosdv_map.yaml` emission.
6. **`pose_source:=mcl`** in the localization fork: add `'mcl'` to
   `available_args` and a `use_mcl_pose` branch (the fork is NEWSLabNTU-owned, so
   this is ours to change).
7. `just map-survey` / `slam_toolbox`, last, and only if a site genuinely has no
   PCD.

Items 1-3 are what make `pose_source:=mcl` *runnable*; 4-5 are what make it
*usable*; 6 is what makes it *named honestly*.
