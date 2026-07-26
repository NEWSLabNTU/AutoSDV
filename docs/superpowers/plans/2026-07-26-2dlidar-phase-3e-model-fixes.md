# 2D-LiDAR Phase 3e — Fixing the Measurement Model

> **For agentic workers:** REQUIRED SUB-SKILL: superpowers:subagent-driven-development. Task 1 is a prerequisite for Tasks 2–3 (it is how they are measured). Task 4 is measurable only end-to-end.

**Goal:** Repair the three implementation defects in the beam sensor model identified in `docs/research/localization/2d_mcl_algorithm.md` §5.1–5.3, measuring each one offline before any end-to-end claim, then re-run the accuracy gate with N ≥ 5 seeds.

**Architecture:** Every fix lands in the PF fork behind a parameter that defaults to today's behaviour, so Phase 3c/3d results stay reproducible and each fix is an A/B flag rather than a rewrite. A new offline harness scores a sensor-model configuration against recorded ground truth by computing, over several frozen scans, how far the likelihood maximum sits from the true pose — the metric that exposed the defect in the first place. Fixes are accepted or rejected on that metric before anyone runs a replay.

**Tech Stack:** the PF fork (`NEWSLabNTU/particle_filter@autosdv`), range_libc, numpy, matplotlib, the Phase 3d instrumentation (`plot_sensor_model.py` frozen-field code, `diagnostics.py`), `run-particle-filter.sh`, `compare_poses.py`.

## Global Constraints

Carried forward from Phases 3–3d, all still binding: fish shell → `bash -c` or script files, **never** bare-source a ROS setup in a top-level eval; `range_libc`/`rosbag2_py` imports stay inside functions and run under a sourced env; apt numpy 1.21.5 (never pip-install numpy); `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`, pure tests must pass in a plain shell; temps under `./tmp/`; `set -euo pipefail` + `set +u`/`set -u` sourcing guard + `setsid` and PGID kill traps for background processes + `|| VAR=""` on command substitutions; ROS rejects int literals for double-typed parameters; bags gitignored under `data/rosbags/phase3/`; commit trailers as on this branch; fork commits go on `autosdv` with a superproject gitlink bump.

Specific to this phase:

- **Every fix is opt-in and default-off.** With defaults, the filter must behave exactly as it does at `12b6815`. Each task verifies this explicitly.
- **No end-to-end accuracy claim from a single run.** The Phase 3c variance caveat stands: three near-identical configurations spanned 17.4–26.5 m. Task 5 exists because of it.
- **The offline metric is the gate for Tasks 2 and 3.** A fix that does not reduce the GT-to-argmax gap does not get carried into Task 5, regardless of how principled it looks.
- Do not "fix" §5.4 (beam correlation) or §5.5 (range clamping) here. They are inherent to the beam model, not implementation defects, and AMCL hit §5.4 too.

## Baseline to beat

From `docs/reports/2dlidar-phase3d-instrumentation.md`, measured with the upstream model at map resolution 0.05 m:

| quantity | tracking frame (t≈9.8 s) | divergence frame (t≈23.3 s) |
|---|---|---|
| GT-to-argmax gap | 47.53 nats | 50.53 nats |
| GT-to-argmax distance | ≈29.4 m | ≈30.4 m |

End-to-end baseline (sample site, Lever 2 configuration, n=1): mean 17.43 m, p95 61.04 m, yaw 0.486 rad. Gate: mean < 1.0 m, p95 < 2.5 m, yaw < 0.2 rad.

## File Structure

```
src/localization/external/particle_filter/particle_filter/
    sensor_model.py          # NEW (fork): table construction, variants, pure + testable
    particle_filter.py       # MODIFIED (fork): variant params, beam masking, scan-gated update, seed
    test/test_sensor_model.py# NEW (fork)
scripts/2dlidar/
    score_sensor_model.py    # NEW: offline A/B harness — the phase's gate
    test_score_sensor_model.py
    run-particle-filter.sh   # MODIFIED: pass-through for the new params
docs/reports/
    2dlidar-phase3e-model-fixes.md   # NEW: per-fix offline deltas + multi-seed end-to-end
    assets/2dlidar-phase3e-*.png
```

---

### Task 1 — Offline scoring harness (the gate)

**Files:** create `scripts/2dlidar/score_sensor_model.py`, `scripts/2dlidar/test_score_sensor_model.py`.

**Interface:** `score_sensor_model.py --gt-bag <bag> --map <yaml> --times 5,10,15,20,23 [--variant upstream|normalized_short] [--skip-nonfinite] [--window-m 60 --field-res-m 0.25] --out-json <path> [--out-fig <path>]`.

For each requested timestamp it reconstructs the scan exactly as `run-particle-filter.sh` does (z-band 1.91611–2.21611 m in `base_link` via the `/tf_static` chain, azimuth binning at 0.0043 rad, `[::18]` decimation — reuse the code Phase 3d Task 5b already wrote in `plot_sensor_model.py`, do not duplicate it), evaluates a log-space field over a pose grid centred on the GT pose with heading fixed at GT theta, and reports per timestamp:

- `gap_nats` — log-likelihood at argmax minus log-likelihood at the GT pose (0 is perfect)
- `dist_m` — distance from GT pose to argmax
- `gt_rank_pct` — GT pose's percentile rank among all grid poses
- `local_max` — whether GT sits at a local maximum (compare against its 8 neighbours)

Plus an aggregate summary (median and worst-case of each). **Use at least five timestamps** spread across the run, so no conclusion rests on one frozen scan — that was a stated limitation of the Phase 3d finding.

The `--variant`/`--skip-nonfinite` switches must build the table and beam mask exactly as the fork's corresponding parameters will, so the offline score predicts the online behaviour. Import the table builder from the fork (`particle_filter.sensor_model` once Task 2 creates it) rather than reimplementing; for this task, implement the `upstream` variant only and structure the code so Task 2 adds `normalized_short` without touching the harness.

Steps: TDD the pure parts (field statistics on a synthetic field with a known argmax; percentile rank; local-max detection) → implement → run with `--variant upstream` and confirm it reproduces the Phase 3d baseline within rounding at the two known timestamps → commit script, tests, and the baseline JSON.

---

### Task 2 — Normalise `p_short` (§5.1)

**Files:** create `particle_filter/sensor_model.py` and `test/test_sensor_model.py` (fork); modify `particle_filter/particle_filter.py` (fork), `scripts/2dlidar/run-particle-filter.sh`.

**Interfaces:** move the table construction out of `precompute_sensor_model` into `sensor_model.py` as a pure function `build_table(max_range_px, z_hit, z_short, z_max, z_rand, sigma_px, variant)` returning the column-normalised table. `variant="upstream"` must reproduce the current table **bit-for-bit** (assert this in a test against the existing code path). `variant="normalized_short"` normalises the short component per column before mixing, so the configured weights mean what they say:

    p_short(r|d) = eta * lambda_short * exp(-lambda_short * r)  for 0 <= r <= d
    eta = 1 / (1 - exp(-lambda_short * d))

with `lambda_short` a new parameter (`sensor_model_lambda_short`, default 1.0 in 1/pixel units — document the unit, and note that unlike the ramp this component's *mass* no longer scales with `d`). New PF parameter `sensor_model_variant` (string, default `"upstream"`), plus script pass-through `PF_SENSOR_MODEL_VARIANT=upstream`, `PF_LAMBDA_SHORT=1.0`.

**Acceptance:** run Task 1's harness with `--variant normalized_short` over ≥5 timestamps and report the gap/distance deltas against the upstream baseline. Also chart the effective mixture weights before and after (extend the Phase 3d Part A figure) — configured `z_hit = 0.75` should now yield an effective weight close to 0.75 across ranges instead of degrading to 6.8%.

Steps: TDD `build_table` including the bit-for-bit upstream equivalence test → wire the parameter → verify defaults unchanged → run the harness → commit fork + gitlink + script.

---

### Task 3 — Stop rewarding no-return beams (§5.2)

**Files:** modify `particle_filter/particle_filter.py` (fork), `scripts/2dlidar/run-particle-filter.sh`.

**Interface:** new parameter `skip_nonfinite_beams` (bool, default `False`); script pass-through `PF_SKIP_NONFINITE=false`.

The defect: `inf` observations are clamped into the max-range bucket, where they score 1.87× a perfect match, and roughly 30% of our beams are non-finite. The evaluation loop lives in C++ (`RangeLib.h:533`) and takes a fixed beam count, so the cleanest Python-side fix is to **drop non-finite beams from the beam set before calling it**: build the mask in `lidarCB`/`update()`, pass only finite beams and their matching angles. Note `downsampled_angles` and the preallocated buffers are sized once on the first scan, so a varying beam count needs either per-update reallocation (measure the cost — the sensor stage is only 15 ms of a 65 ms update, so there is headroom) or a fixed-size buffer with a beam-count argument. Choose one, justify it in the report, and keep the fast path unchanged when the flag is off.

If dropping beams proves structurally awkward, the fallback is to substitute a neutral likelihood for non-finite beams (multiply by a constant, i.e. contribute nothing) — measurably equivalent to dropping them and much simpler. Prefer whichever is simpler to verify; state which you chose.

**Acceptance:** harness delta with `--skip-nonfinite` on top of Task 2's best variant, ≥5 timestamps. Also report the observed beam count per update from the diagnostics JSONL.

---

### Task 4 — Update only on new scans (§5.3)

**Files:** modify `particle_filter/particle_filter.py` (fork), `scripts/2dlidar/run-particle-filter.sh`.

**Interface:** new parameter `update_on_new_scan_only` (bool, default `False`); script pass-through `PF_UPDATE_ON_SCAN_ONLY=false`.

`update()` currently fires from `odomCB` at 20 Hz while scans arrive at 10 Hz, so each scan is multiplied into two consecutive Bayes updates. With the flag on, `lidarCB` records the scan's stamp, and `update()` runs the correction only when the stamp is new; odometry deltas accumulate between scans (the existing `odometry_data` accumulator already sums them — verify, do not assume). Keep publishing pose at odometry rate if that is cheap to preserve; if not, note the rate change in the report, since downstream consumers see it.

**Measurement:** the offline field metric cannot see this (it changes filter dynamics, not the likelihood). Verify instead that the diagnostics JSONL shows correction steps at scan rate rather than odom rate, and carry the flag into Task 5's end-to-end matrix.

---

### Task 5 — Multi-seed end-to-end evaluation

**Files:** modify `particle_filter/particle_filter.py` (fork: seeding), `scripts/2dlidar/run-particle-filter.sh`.

**Interface:** new parameter `random_seed` (int, default `-1` = don't seed, today's behaviour); when ≥ 0, call `np.random.seed(random_seed)` during init so a run is reproducible. Script pass-through `PF_RANDOM_SEED=-1`.

Run the sample-site configuration for **5 seeds × 2 configurations** (baseline `upstream` versus all Phase 3e fixes enabled), 10 replays total, each compared with `compare_poses.py --no-motion-window --gt-time-source bag`. Report per-configuration **median and range** of mean error, p95 and yaw across seeds — never a single number. State plainly whether the gate (mean < 1.0 m, p95 < 2.5 m, yaw < 0.2 rad) is met, and if it is not, whether the improvement is nonetheless outside the seed spread.

Note runtime: each replay is ~60 s plus ~48 s of CDDT init, so budget roughly 25 minutes of wall clock. Run them sequentially, never in parallel — concurrent ROS graphs on one machine cross-talk.

---

### Task 6 — Report and documentation follow-through

**Files:** create `docs/reports/2dlidar-phase3e-model-fixes.md`; modify `docs/research/localization/2d_mcl_algorithm.md`, `docs/design/f1tenth-2dlidar-integration.typ`.

The report carries: the offline gate table (gap/distance per fix per timestamp, with medians), the multi-seed end-to-end table with spreads, what each fix did and did not buy, and an honest verdict.

Then close the loop on the documents that made claims about this: mark §5.1–5.3 in the algorithm doc as fixed (with the measured deltas) or as attempted-and-insufficient; and update the design report's Phase 3 verdict box — if the gate now passes, 2D MCL returns as a viable localization source and the recommendation changes; if not, say so and keep NDT. Recompile the typ PDF locally (it is not tracked). Push both the branch and the fork branch.

---

## Out of Scope

§5.4 (beam correlation) and §5.5 (range clamping) — inherent to the beam model rather than implementation defects. Any Autoware integration work (`ekf_localizer` bridge, Phase 4) stays out until this phase reports.

## Self-Review Notes

- Task 1 must land before Tasks 2–3 report anything, since it is their measurement instrument; Task 4 is explicitly exempt and measured only in Task 5.
- The bit-for-bit upstream equivalence test in Task 2 is what protects Phase 3c/3d reproducibility from a refactor regression; it is not optional.
- Five timestamps in Task 1 and five seeds in Task 5 are both direct responses to the Phase 3c variance finding — do not reduce either for speed.
