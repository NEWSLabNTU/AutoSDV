# 2D-LiDAR Phase 3b — MCL vs NDT on the Autoware Sample Site

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Re-run the Phase 3 PF-vs-NDT comparison on the official Autoware sample map + sample rosbag, where NDT is known-good out of the box — replacing the COSS run whose NDT ground truth was itself untuned/unreliable.

**Architecture:** Migrate the sample bag's `autoware_auto_*` vehicle messages to Autoware 1.5.0 types (existing leodrive migration script). Run stock `autoware_launch logging_simulator.launch.xml` with `sample_vehicle`/`sample_sensor_kit` over the migrated bag, recording an **enriched bag**: NDT `kinematic_state` + the decoded top-LiDAR pointcloud + IMU + velocity status. Then run the existing PF pipeline against the enriched bag (env overrides — the scripts were built for this) on a regenerated sample-map grid sliced at the top-LiDAR scan-plane height. Finally re-run `compare_poses.py` and commit a second report.

**Tech Stack:** Everything already exists: `migrate-to-autoware15.py` (leodrive submodule), `pcd_to_pgm.py`, `wheel_imu_odom.py`, `run-particle-filter.sh`, `compare_poses.py`, stock `autoware_launch` + `sample_sensor_kit`/`sample_vehicle` (verified installed at /opt/autoware/1.5.0).

## Global Constraints

Same as Phase 3 plan (`2026-07-25-2dlidar-phase-3.md`): fish→`bash -lc`, sourcing guard idioms, setsid+PGID traps, `|| VAR=""` guards, never await play_launch/ros2 launch foreground, apt numpy only, `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`, temps → `./tmp/`, bags → `data/rosbags/phase3/` (gitignored), commit trailers as on this branch. Plus:

- Sample bag: `data/sample-rosbag-replay/sample-rosbag`, **29.9 s**, contains `/clock` (play WITHOUT `--clock`... verify: prefer playing with `--clock` and let sim time come from replay clock publisher; the recorded /clock topic can be excluded via `--topics`-less play + remap-free approach — decide empirically and document). Vehicle topics are `autoware_auto_vehicle_msgs` (NOT installed) — migration is mandatory before anything consumes them.
- LiDAR: packets only; the logging_simulator's sensing module decodes them (VLP-16 top). The enriched recording must capture the decoded top pointcloud topic (find exact name at runtime: `/sensing/lidar/top/pointcloud_raw_ex` or similar).
- Tutorial reference: map `data/sample-rosbag-replay/sample-map-rosbag`, `vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit`; tutorial plays at `-r 0.2` with full stack — we disable perception/planning (logging_simulator args `perception:=false planning:=false control:=false rviz:=false` — check exact arg names in the launch file) so a higher rate may hold; start at `-r 0.5`, drop to 0.2 if NDT lags.
- Sample-map ground z ≈ −3.4…−3.2 (measured in Phase 2); top VLP-16 mount height read from `sample_sensor_kit` calibration (`sensors_calibration.yaml`) at runtime — grid z-band = ground + sensor height ± 0.15 m.

## Tasks

### Task 1: Migrate sample bag to Autoware 1.5.0 message types

- Run `scripts/leodrive-bus-launch/scripts/migrate-to-autoware15.py data/sample-rosbag-replay/sample-rosbag data/sample-rosbag-replay/sample-rosbag-migrated` (check script CLI first — it may need `just setup-python`/rosbags pkg; the leodrive justfile documents it).
- Verify: `ros2 bag info` shows `autoware_vehicle_msgs/*` types, counts preserved (873 vehicle msgs, 288 top packets).
- No commit (bag gitignored) unless the migration script needed fixes (commit those in the leodrive submodule per vendoring policy — flag controller first).

### Task 2: Sample-site NDT ground truth (enriched capture)

- Create `scripts/2dlidar/record-sample-ndt-groundtruth.sh` modeled on `record-ndt-groundtruth.sh`, but launching stock: `ros2 launch autoware_launch logging_simulator.launch.xml map_path:=$(pwd)/data/sample-rosbag-replay/sample-map-rosbag vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit` with perception/planning/control/rviz disabled (verify arg names). Kill by PGID (no play_launch here; plain ros2 launch — same PGID discipline).
- Record to `data/rosbags/phase3/sample_ndt_gt`: `/localization/kinematic_state`, the decoded top pointcloud topic, `/sensing/imu/tamagawa/imu_raw`, `/vehicle/status/velocity_status`, `/tf`, `/tf_static`.
- Replay the MIGRATED bag. Init: GNSS auto-init expected (tutorial behavior); manual `/initialpose` fallback.
- PASS gate: kinematic_state count > 200 AND decoded pointcloud count > 100.
- Commit the script.

### Task 3: Sample-map grid + PF run

- Read top-LiDAR height from sample_sensor_kit calibration; regenerate grid: `pcd_to_pgm.py` on sample-map PCD, z-band = (ground + sensor_z) ± 0.15, resolution 0.1 m, min-points 1 (sparse map — Phase 2 needed 0.2 m/1; try 0.1 first, fall back). Output: `data/sample-rosbag-replay/sample-map-rosbag/occupancy_grid_scanplane.{pgm,yaml}` (gitignored dir — regenerate on demand; record the command in the report).
- Verify with `check-map-load.sh <yaml>`.
- Run PF via `run-particle-filter.sh` env overrides (add overrides if missing — keep defaults COSS-compatible): `BAG=data/rosbags/phase3/sample_ndt_gt` (the enriched bag), `POINTCLOUD_TOPIC=<decoded top topic>`, `MAP_YAML=<scanplane yaml>`, `GT_BAG=data/rosbags/phase3/sample_ndt_gt`, odom node topics: velocity `/vehicle/status/velocity_status`, imu `/sensing/imu/tamagawa/imu_raw`. Scan z-band in sensor frame stays ±0.15.
- Output `data/rosbags/phase3/sample_pf_run`; PASS gate inferred_pose > 200.
- Commit script modifications (env-override generalization only — no behavior change for COSS defaults).

### Task 4: Comparison + report

- `compare_poses.py data/rosbags/phase3/sample_ndt_gt data/rosbags/phase3/sample_pf_run --out docs/reports/2dlidar-phase3b-sample-site.md` — full-overlap thresholds unchanged (mean < 1.0 m, p95 < 2.5 m, yaw < 0.2 rad). No motion-window split needed (bag moves throughout) — if the tool requires it, window = full span.
- Sanity overlay (like the COSS debug): scan at GT pose on the scanplane grid, one figure, attach path in the report.
- Commit report (+ any small compare_poses generalization). Push branch at the end.

## Self-Review Notes

- Reuses every Phase 3 artifact; new code surface = one capture script + env-override plumbing.
- Known risk: logging_simulator arg names for disabling modules differ across versions — Task 2 verifies against the installed launch file before running.
- The COSS Phase 3 report remains committed as the honest record of that run; Phase 3b report is additive, and should state why the COSS ground truth was demoted (untuned NDT on COSS — see docs/research/localization/ndt_parameter_tuning_coss_map.md).
