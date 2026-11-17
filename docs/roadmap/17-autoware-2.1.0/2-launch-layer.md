# Phase 2: launch layer

**Status:** done on the workstation (2026-11-18), play_launch 0.38.0 (Rust parser).

## Mechanism

- **Per-version files, chosen at build time.** A file that differs between Autoware
  versions lives once per version under `<package>/autoware/<version>/`, in the package's
  own layout, and never in the shared tree as well. `autosdv_cmake`'s
  `autosdv_install_autoware_overlay()` installs the overlay of `AUTOWARE_VERSION` (each
  version has its own workspace) and fails the configure step when the version folders
  hold different files or a file is both shared and versioned. Used by `autosdv_launch`,
  `autosdv_sensor_kit_launch` and `tier4_localization_launch`.
- **Stock parameter paths: `autosdv_autoware_stock`.** 2.1.0 moved control, localization,
  map, perception and planning parameters from `autoware_launch/config/<area>` into
  `autoware_<area>_config/config`, same layout beneath. The package installs
  `share/autosdv_autoware_stock/<area>` as a link to whichever exists, so a shared launch
  file writes `$(find-pkg-share autosdv_autoware_stock)/control/...`.
- **Parameter copies: the union of both versions' keys.** A node ignores parameters it does
  not declare, so one copy carries 1.5.0's keys and 2.1.0's additions, each addition marked
  `added for Autoware 2.1.0`. Inserted with 2.1.0's default values, except
  `lanelet2_map_metadata_path` (upstream's value is a launch argument we do not declare;
  `""` is 1.5.0's behaviour).
- **Dependencies by version:** `package.xml` conditions, e.g.
  `<exec_depend condition="$AUTOWARE_VERSION == '2.1.0'">`.

## What changed for 2.1.0

| where | 1.5.0 | 2.1.0 |
|---|---|---|
| sensing | `tier4_sensing_component` | `component_sensing` (includes the kit's `sensing.launch.xml` directly); wrapper `autosdv_sensing_component` |
| perception | fork of `tier4_perception_component` | wrapper over `component_perception` (LiDAR only; params from `autoware_perception_config`) |
| planning | fork of `tier4_planning_component` | wrapper over `autoware_planning_launch/planning.launch.xml`, passing only the `rc` profile's files |
| control | fork of `tier4_control_component` | `component_control`'s body with our longitudinal PID file |
| pose initializer | `autoware_automatic_pose_initializer` | `autoware_pose_initialization_requester` |
| Velodyne (car A) | `nebula_ros`, `nebula_decoders/calibration/velodyne/` | `nebula_velodyne`, `nebula_velodyne_decoders/calibration/` |
| simulator component | — | requires `launch_fault_injection` (we pass `true`: 1.5.0 always ran it) |
| diagnostics graphs | `/autoware/debug/tools` | removed; dropped from our copies (no mode links it) |
| parameters | — | 36 keys added to 12 files the 2.1.0 stack loads |

## Result

- `play_launch dump` on both versions: `autosdv.launch.yaml` (car A and car B), the logging
  simulation, the planning simulation, the RC simulator, the coach pursuit simulator, and
  every launch file in `autosdv_launch` that resolves on 1.5.0 resolves on 2.1.0.
- Runtime, 90 s on 2.1.0 (domain 87, perception off, no bag): logging simulation 50/50
  nodes and composables, planning simulation 48/48 and 41/41.
- Tests of the changed packages: no failure beyond phase 1's baseline.

## Open (later phases)

- **play_launch #0095:** the default parser drops 2.1.0's `<autoware_node>` (3 perception
  nodes; `WARN Unsupported action type`). Fixed on play_launch main (0e6c42b4), released in
  0.39.0 when published; until then `--parser python` with perception on. Phase 3.
- **Perception parity** (phase 3): our map filter, mapless CenterPoint and ground
  segmentation files need a perception config package of ours; camera and fusion modes,
  detection by tracker, traffic lights and the image-segmentation filter have no 2.1.0
  pipeline here. 22 perception parameter copies lack 2.1.0 keys but are not loaded on 2.1.0
  yet (the tracker's association matrix changed shape).
- **localrepo:** 2.1.0's launch files default `data_path` to `~/autoware_data/ml_models`,
  its model downloader writes into `~/autoware_data/`. AutoSDV passes `data_path`, so
  unaffected; reported.
- Hardware launches (sensors, vehicle interface) are resolved, not run: phase 5.
- `radar_object_tracker.param.yaml` names `autoware_launch/config/.../radar_object_tracker/tracking/`,
  which exists in neither version (radar unused).
