# Phase 4: simulation regression

Run on both versions and compare:

- `just demo run` (COSS NDT replay): NDT activation, pose quality, alignment
  (`scripts/testing/localization/*`).
- `just sim planning` and `just sim logging` (CUDA NDT, MCL).
- `just sim rc`, `just sim grade` (Lab 2 Part 1), `lab3_check_lanelet` / `lab3_sim_lap` on
  the Lab 3 example map, the coach sim, `just twin` on domain 47.
- CUDA pipeline (`pointcloud_backend:=cuda`, `localization_pointcloud_backend:=cuda`).

**Exit:** every run completes on 2.1.0 with numbers within 1.5.0's (or a named, accepted
difference recorded here).

## Result (2026-11-18, workstation)

1.5.0 here is localrepo 1.5.0-humble-3, which cannot load four libraries on this host
(centerpoint, concatenate_data, cuda_pointcloud_preprocessor, its lib: the
diagnostic_updater link bug fixed in 1.5.0-humble-4). Comparisons are therefore made
with perception off, where 1.5.0 is whole. The workstation is shared (load average
5-12 from other users), so a single replay's timing is noisy; one 2.1.0 CPU-NDT run
dropped a third of its scans to contention and its rerun did not (below).

| run | 1.5.0 | 2.1.0 |
|---|---|---|
| Lab 2 Part 1 grade, reference SpeedPID | 15/15, 8 variants 16/16 | identical: all 401 measured values equal |
| RC sim smoke (straight, stock planning) | PASS, at rest 121.49 m (line 121.53) | PASS, 121.51 m |
| Lab 3 sim lap (example map, rc profile) | 125.6 s (Lab 3 slides) | PASS, 96.4 m in 127 s |
| coach bench checks | 7/8 | 7/8, same values within 0.05 |
| Lab 2 waypoint bench checks | | 5/5 |
| COSS replay, `cuda_ndt`, perception off: drive NVTL / exe ms / i2r m | 2.784 / 26.5 / 0.048 | 2.785 / 26.4 / 0.048 |
| same, poses / worst gap | 1321 / 0.115 s | 1236 / 0.303 s |
| COSS replay, CPU `ndt`, perception off: drive NVTL / exe ms | 2.774 / 6.2 | 2.777 / 6.0 (rerun) |
| same, poses / worst gap | 1386 / 0.115 s | 1321 / 0.399 s (rerun; first run 828 / 0.8 s under load) |
| COSS replay, `cuda_ndt` + CUDA localization chain | does not load (humble-3) | NVTL 2.784, 1133 poses |
| planning simulator (COSS map) | | starts, 48/48 nodes |
| Lab 2 Part 2 integration test (bench, virtual camera) | | PASS, grade 35/45, fix error p95 0.21 m |
| runtime sweep: 9 launches started for 50 s each | same failures | same failures (no GPIO, no LiDAR, no display) |

`follow.grade` in the coach checks fails identically on both: the harness cannot parse
the retired coach lab's `ta/grade.py` output. Not a migration item.

With perception on, 2.1.0 runs CenterPoint and 1.5.0 does not (it fails to load), so the
2.1.0 run's i2r (0.24 m vs 0.05 m) measures the CPU the detector takes on this shared
host, not localization.

### Found and fixed

- **Simulated perception moved** (phase 3 entry): RC and coach simulators never engaged.
- **`projector_type: local`.** 1.5.0's map_projection_loader accepts `local` and `Local`;
  2.1.0 accepts only `Local` and exits on `local`, after which the mission planner waits
  for a lanelet map forever ("Route API is not ready"). Eight waypoint-lab courses, the RC
  sim straight, and five generators wrote `local`; all now write `Local`, and
  `just map check` refuses `local` with that reason.
- **Inline lanelet2 loaders.** `lab2_part1_sim` and `aruco_localization` start the loader
  with their own parameter list; 2.1.0 exits ("expected [bool] got [not set]") without
  `enable_selected_map_loading` and `lanelet2_map_metadata_path`. Both now pass them.
- **RC smoke** waits for the routing state and clears a route that is not UNSET before
  setting one, as Lab 3's client already did, and prints the refusal it got.
- **Car A's URDF and `ros2 launch`.** Version-independent, found by play_launch 0.39.0's
  Python parser: `tier4_vehicle_launch` passes `robot_description` untyped, so launch_ros
  reads the xacro output as YAML, and a comment containing a colon and a space failed
  it for car A. play_launch's Rust parser accepted it. Comment reworded
  (`autosdv_vehicle_launch` 131b0b7).

play_launch 0.39.0 (#0095 fixed) and its Python parser now give the same nodes, remaps
and parameters for car A, car B, the logging simulation and the planning simulator on
2.1.0, so `--parser python` is no longer needed.

Not run here: `just twin` (Isaac Sim) and `pointcloud_backend:=cuda` on a replay, both
GPU-heavy on a GPU other users share; they move to phase 5's Orin runs, where the CUDA
pipeline matters.
