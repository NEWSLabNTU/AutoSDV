# 2D-LiDAR Phase 3d — MCL Instrumentation and Visualisation

> **For agentic workers:** REQUIRED SUB-SKILL: superpowers:subagent-driven-development. Tasks are mostly independent; Task 5 depends on Tasks 1–2.

**Goal:** Make the particle filter's internal state observable — per-update diagnostic logging, offline chart suite, and a live likelihood field in RViz — so the structural defects catalogued in `docs/research/localization/2d_mcl_algorithm.md` §5 can be seen rather than inferred.

**Architecture:** The PF node (our fork, `NEWSLabNTU/particle_filter@autosdv`) gains an opt-in diagnostics module: per-update records to a JSONL file plus scalar `std_msgs` topics for live PlotJuggler, and an optional coarse likelihood-field publisher (`nav_msgs/OccupancyGrid`) evaluated around the current estimate. A standalone offline script turns a JSONL run into a multi-panel chart set. A second standalone script renders high-resolution likelihood fields and sensor-model diagnostics from recorded bags, needing no live run.

**Tech Stack:** rclpy, numpy, matplotlib (Agg), range_libc (direct use for offline fields), `std_msgs`, `nav_msgs`, PlotJuggler (already in the repo via `just tool-plotjuggler`), RViz.

## Global Constraints

Same as Phase 3/3b/3c: fish shell → `bash -lc` or script files, **never** bare-source a ROS setup in a top-level eval; apt numpy 1.21.5 (never pip-install numpy); `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` for pytest; temp/scratch under `./tmp/`; `set -euo pipefail` + `set +u`/`set -u` sourcing guard + `setsid` and PGID kill traps for every background process + `|| VAR=""` on command substitutions; ROS rejects int literals for double-typed parameters (pass `60.0`, not `60`); commit trailers as used on this branch; bags stay gitignored under `data/rosbags/phase3/`.

Plus, specific to this phase:

- **Every diagnostic is opt-in and default-off.** With all new parameters at their defaults the filter must behave bit-identically to today, and the Phase 3c reports must remain reproducible. Overhead when enabled should stay under roughly 10% of update time, except the likelihood field, which is explicitly rate-limited.
- Diagnostics are **written from the PF fork**, so every change there is a fork commit on `autosdv`, and the superproject records a gitlink bump.
- Respect the Phase 3c **variance caveat**: instrumentation changes nothing about run-to-run spread, so no chart may present a single run as evidence of a lever's effect.

## File Structure

```
src/localization/external/particle_filter/particle_filter/
    diagnostics.py              # NEW (fork): DiagnosticsRecorder + pure helpers
    particle_filter.py          # MODIFIED (fork): opt-in hooks, likelihood-field publisher
    test/test_diagnostics.py    # NEW (fork): pure-function tests
scripts/2dlidar/
    plot_mcl_diagnostics.py     # NEW: JSONL -> chart suite
    plot_sensor_model.py        # NEW: sensor-model table + likelihood-field renders
    rviz/mcl_debug.rviz         # NEW: RViz config wired to the debug topics
    run-particle-filter.sh      # MODIFIED: pass-through env for the new params
docs/reports/
    2dlidar-phase3d-instrumentation.md   # NEW: what each chart shows, on a real run
    assets/2dlidar-phase3d-*.png         # NEW: generated figures
```

---

### Task 1 — Diagnostics module in the PF fork (per-update JSONL)

**Files:** create `particle_filter/diagnostics.py` and `test/test_diagnostics.py`; modify `particle_filter/particle_filter.py`. All inside the submodule.

**Interfaces:**
- `class DiagnosticsRecorder(path, flush_every=20)` — `.record(dict)` appends one JSON object per line; `.close()` flushes. No ROS imports, so it is unit-testable in a plain shell.
- Pure helpers (module level, all tested): `effective_sample_size(weights)` (already exists in `particle_filter.py` — **move it here** and re-export to avoid duplication), `weight_entropy(weights)` → Shannon entropy in nats, `pose_covariance(particles, weights)` → 2×2 weighted xy covariance, `beam_categories(observed_m, predicted_m, resolution, max_range_px, sigma_px)` → dict of fractions `{hit, short, long, clamped, nonfinite}` where a beam counts as *hit* when `|r−d| ≤ 3·sigma`, *clamped* when `r/res ≥ max_range_px`, *nonfinite* when `r` is not finite.

**New PF parameters** (all default-off / no-op): `diag_enable` (bool, `False`), `diag_path` (string, `""` → auto `./tmp/mcl_diag_<pid>.jsonl`), `diag_every` (int, `1` — record every Nth update), `diag_beam_arrays` (bool, `False` — also dump observed and best-particle predicted range arrays, which are the bulky part).

**Record schema** (one line per recorded update; the offline plotter depends on these exact keys):

```
iter, stamp_scan, stamp_wall, dt_update,
action_dx, action_dy, action_dtheta,
n_eff, weight_entropy, weight_max,
pose_x, pose_y, pose_theta, cov_xx, cov_yy, cov_xy,
resampled (bool), frac_hit, frac_short, frac_long, frac_clamped, frac_nonfinite,
t_propose, t_motion, t_sensor, t_norm,
observed (optional array), predicted_best (optional array)
```

Steps: write failing tests for the four pure helpers (uniform weights → `N_eff = N`; one-hot → `N_eff = 1`; entropy of uniform = `ln N`; covariance of a known weighted cloud; beam categories on a hand-built case covering all five buckets) → run to see them fail → implement `diagnostics.py` → make tests pass → wire the hooks into `MCL`/`update()` behind `diag_enable`, recording `resampled` from the existing ESS-gate branch → verify default-off produces no file and no measurable slowdown → commit in the fork, bump the gitlink in a superproject commit.

---

### Task 2 — Live scalar topics + `run-particle-filter.sh` pass-through

**Files:** modify `particle_filter/particle_filter.py` (fork), `scripts/2dlidar/run-particle-filter.sh`.

**Interfaces:** with `diag_topics` (bool, default `False`), publish one `std_msgs/Float32` per update on `/pf/debug/n_eff`, `/pf/debug/weight_entropy`, `/pf/debug/pose_cov_trace`, `/pf/debug/update_hz`, `/pf/debug/frac_clamped`, `/pf/debug/frac_short`. Plain `std_msgs` deliberately — no new message package, and PlotJuggler subscribes to these directly.

Script pass-through env vars, every one defaulting to today's behaviour: `PF_DIAG_ENABLE=false`, `PF_DIAG_PATH=""`, `PF_DIAG_EVERY=1`, `PF_DIAG_BEAM_ARRAYS=false`, `PF_DIAG_TOPICS=false`, `PF_LIKELIHOOD_FIELD=false`, `PF_LF_WINDOW_M=40.0`, `PF_LF_RES_M=0.5`, `PF_LF_PERIOD_S=1.0`. Remember the float-formatting normalisation already in the script (`to_float`) — the new doubles must go through it.

Steps: implement the publishers → run the existing Lever-2 configuration with `PF_DIAG_ENABLE=true PF_DIAG_TOPICS=true`, confirm the JSONL grows and `ros2 topic hz /pf/debug/n_eff` reports ≈ update rate → confirm a default-flag run still produces no diagnostics → commit (fork + gitlink + script).

---

### Task 3 — Offline chart suite

**Files:** create `scripts/2dlidar/plot_mcl_diagnostics.py`, `scripts/2dlidar/test_plot_mcl_diagnostics.py`.

**Interface:** `plot_mcl_diagnostics.py <diag.jsonl> --out-dir docs/reports/assets [--gt-bag <bag>] [--prefix 2dlidar-phase3d]`. With `--gt-bag`, join against `/localization/kinematic_state` (reuse the alignment helpers in `scripts/2dlidar/compare_poses.py` — import them rather than reimplementing; note its `--gt-time-source` distinction, since PF stamps are wall-clock until §5.3 of the algorithm doc is fixed) so the error panel is real rather than self-reported.

Panels, one figure with shared time axis:

1. translational error vs GT (if provided) with divergence onset marked
2. `N_eff` (with the `ess_threshold_ratio · N` line) and resample events as rug marks
3. weight entropy and `weight_max`
4. pose covariance trace, plus its xy eigenvalue ratio — **elongation is the ridge signature**
5. beam category fractions, stacked
6. per-stage timing (`t_propose`/`t_motion`/`t_sensor`/`t_norm`) and update rate

Steps: TDD the pure parts (JSONL loading with a malformed-line skip and a clear error when the file has no valid records; the derived-series maths such as eigenvalue ratio) → implement → run on the Task 2 JSONL → commit script, tests, and the generated PNG.

---

### Task 4 — Live likelihood field (`nav_msgs/OccupancyGrid`) + RViz config

**Files:** modify `particle_filter/particle_filter.py` (fork); create `scripts/2dlidar/rviz/mcl_debug.rviz`.

**Interface:** with `likelihood_field_enable` (bool, default `False`), every `lf_period_s` seconds build a coarse pose grid centred on the current inferred pose — `lf_window_m` across at `lf_res_m` spacing, heading fixed at the inferred theta — evaluate it through the *existing* `calc_range_repeat_angles` + `eval_sensor_model` path with the current scan, and publish the result as `nav_msgs/OccupancyGrid` on `/pf/debug/likelihood_field`.

Encoding matters for interpretability: take `log` of the raw per-pose weights, subtract the max (so the best pose is 0), clip at `-lf_log_floor` (default `20.0`), and map linearly to 0…100 with 100 = most likely. Grid `info.origin` must place the window correctly in the map frame so RViz overlays it on the map without manual nudging.

Cost check: 40 m at 0.5 m → 81×81 = 6,561 poses × 82 beams ≈ 538k raycasts, comparable to 1.6 filter updates, once per second. Reuse the preallocated buffers where possible; if allocation per call proves significant, cache by grid shape.

The RViz config should show: map, `/pf/debug/likelihood_field` (colour scheme `costmap`, alpha ~0.6), `/pf/viz/particles`, `/pf/viz/inferred_pose`, `/scan`, `/pf/viz/fake_scan` in a contrasting colour, and TF.

Steps: implement behind the flag → run with `PF_LIKELIHOOD_FIELD=true`, verify publish rate ≈ 1 Hz and that update rate does not drop more than ~10% → capture two screenshots or offline renders, one while tracking and one at divergence onset, for the report → commit (fork + gitlink + rviz config).

---

### Task 5 — Sensor-model and frozen-scan renders + report

**Files:** create `scripts/2dlidar/plot_sensor_model.py`; create `docs/reports/2dlidar-phase3d-instrumentation.md`.

**Interface:** `plot_sensor_model.py --resolution 0.05 --max-range 60.0 [--z-hit .75 --z-short .01 --z-max .07 --z-rand .12 --sigma-px 8.0] --out-dir docs/reports/assets` — rebuilds the table with the *same* formula as `precompute_sensor_model` (import it if practical, otherwise duplicate with a comment naming the source line, and assert agreement against a small reference case) and renders:

1. normalised columns `P(r|d)` for d = 10/20/50 m, on one axis, in metres
2. effective mixture-weight breakdown vs predicted range — the §5.1 table as a chart
3. the `P(no-return) / P(perfect match)` ratio vs predicted range — the §5.2 inversion
4. the same for two resolutions side by side, to show the pixel-unit coupling

Plus a frozen-scan high-resolution likelihood field: `--frozen-scan <gt-bag> --map <yaml> --time <rel_s>` reconstructs the scan for one timestamp exactly as `run-particle-filter.sh` does (z-band in `base_link`, same decimation), then evaluates a fine grid (e.g. 60 m at 0.1 m) around the GT pose via `range_libc` directly. Render two: one early (tracking) and one at divergence onset, so the ridge is visible or not.

The report ties the whole phase together: one section per chart, stating what it shows and which defect from `2d_mcl_algorithm.md` §5 it makes visible; the n=1 caveat; and the exact commands used to regenerate everything.

Steps: implement → generate all figures from the recorded Phase 3b/3c bags → write the report → commit script, report, assets → push the branch and the fork branch.

---

## Out of Scope

Fixing any of the §5 defects. This phase only builds the observation apparatus; the fixes are Phase 3e, and they should be evaluated with N ≥ 5 seeds per the Phase 3c variance caveat.

## Self-Review Notes

- Every new PF parameter defaults to off, so Phase 3c reproducibility is preserved; Task 2's steps verify that explicitly.
- `effective_sample_size` currently lives in `particle_filter.py` (added in Lever 3). Task 1 moves it into `diagnostics.py` and re-exports, so there is one definition; the existing Lever 3 tests must keep passing.
- Task 3 imports alignment helpers from `compare_poses.py` rather than duplicating them, and inherits its wall-clock-stamp caveat, which is itself defect §5.3.
- Tasks 1–4 each end in a runnable verification against the existing sample-site bags; no new data capture is required for this phase.
