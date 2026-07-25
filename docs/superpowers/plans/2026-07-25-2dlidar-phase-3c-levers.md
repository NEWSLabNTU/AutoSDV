# 2D-LiDAR Phase 3c — Remaining Levers Against Corridor Aliasing

> **For agentic workers:** REQUIRED SUB-SKILL: superpowers:subagent-driven-development. Sequential levers with early exit — stop as soon as the Phase 3 thresholds pass.

**Goal:** Close the last Phase 3 failure mode: PF divergence on the sample site's long straight (corridor aliasing + thin map tail). Try the documented levers in order of expected value; stop at the first PASS (mean < 1.0 m, p95 < 2.5 m, yaw < 0.2 rad, full overlap).

**State entering 3c** (see `docs/reports/2dlidar-phase3b-tuned.md` and ledger): IMU sign + scan frame fixed; PF sub-meter through the curve (≤1.3 m to rel 28 s); divergence onset on the straight; best aggregate so far mean 18.2 m / p95 107 m / yaw 0.57 rad. Odometry proven good (0.79 m mean dead-reckon over the full run).

## Global Constraints

Same as Phase 3/3b plans (shell idioms, apt numpy, PYTEST flag, ./tmp, no bare-source, setsid+PGID, bags gitignored under data/rosbags/phase3/, honest verdicts, trailers). All runs: scan-accumulated-map lineage, `IMU_YAW_SIGN=-1.0`, frame fix active, tuned params as baseline (`PF_MAX_RANGE=60 SCAN_RANGE_MAX=60 PF_SQUASH=3.0 PF_DISP_THETA=0.1`), compare with `--no-motion-window --gt-time-source bag`, report the rel_t error table (0/5/14/28/43/57 s) in every report.

## Levers (execute in order; early-exit on PASS)

### Lever 1 — Dense accumulation map (`--min-hits 1`, res 0.1)
The straight's walls were dropped by `min_hits: 3` on a single fast pass. Regenerate: `scan_accumulate_grid.py sample_ndt_gt <prefix>_mh1 --min-hits 1 --resolution 0.1`; report occupied-count delta vs the min-hits-3 map (82,341); rerun PF (`OUT_BAG=…_lever1`), compare → `docs/reports/2dlidar-phase3c-lever1.md`.

### Lever 2 — Fine-resolution map (min-hits 1, res 0.05)
Longitudinal microfeatures (poles, driveway gaps, wall breaks) survive 0.05 m binning that 0.1 m blurs. Same flow, `--resolution 0.05` (`_mh1r05`); watch CDDT init time with the larger grid — if PF map-load exceeds ~120 s, note and continue. Report → `…lever2.md`.

### Lever 3 — PF resampling/robustness work (vendored-code change)
Only if 1–2 fail. Options inside `particle_filter` (fork trigger — CL2-UWaterloo upstream → NEWSLabNTU fork per policy): low-variance resampler with effective-sample-size gate (resample only when N_eff < N/2) to stop aliased collapses, and/or per-update particle-injection around the odometry prior. Scope one change, TDD where the math is pure, rerun, report → `…lever3.md`.

### Lever 4 — AMCL cross-check (arbitration)
Independent of 3 (run even if 3 passes, cheap): nav2 AMCL on the same /scan + /odom + map (needs `ros-humble-nav2-amcl`; ask controller for apt if missing). Same comparison → `…lever4-amcl.md`. Outcome interpretation: AMCL good + our PF bad → port-specific defect; both bad → environment/sensor-model limit for 2D MCL at this speed/site.

## Exit

Whichever lever ends the sequence: update `docs/reports/2dlidar-phase3b-tuned.md`'s successor pointer (one line at top: "superseded by …lever N report"), append final verdict to the typ design doc's risk table when the phase closes, push.
