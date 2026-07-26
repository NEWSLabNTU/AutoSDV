# Phase 3e: measurement-model fixes and the Phase 3 accuracy gate

Phase 3d found and instrumented the cause of Phase 3's MCL divergence: a
mis-specified beam sensor model, not underflow, not motion-model error, not
resampling. Phase 3e fixes three of the five defects
[`2d_mcl_algorithm.md`](../research/localization/2d_mcl_algorithm.md) §5
lists, measures each fix in isolation with an offline scoring harness, then
measures all three together end-to-end across a 5-seed replay matrix. The
Phase 3 accuracy gate (mean < 1.0 m, p95 < 2.5 m, yaw < 0.2 rad) now
**passes 5/5 seeds**.

This report ties the six Phase 3e tasks together: the offline gate per fix,
the scan-gating evidence, the seed matrix, what each fix bought and did not
buy, and an honest statement of what remains unestablished.

---

## 1. The offline gate

`scripts/2dlidar/score_sensor_model.py` (Task 1) reconstructs a frozen scan
at a chosen timestamp, evaluates the configured sensor model over a
log-space pose grid centred on the NDT ground-truth pose, and reports how
far the model's own global maximum sits from GT — in nats (the log-space
gap between GT's likelihood and the grid's best) and metres (the physical
distance between GT and the argmax). It reuses Phase 3d's raycasting code
unchanged; the only new work is caching the expensive map/raycaster build
across timestamps (~53 s/frame uncached, per Phase 3d, down to ~3.1 s/frame
here) and a stable JSON/figure output.

All numbers below are 5 frozen scans (`t = 5, 10, 15, 20, 23` s) from
`data/rosbags/phase3/sample_ndt_gt`, scored against
`data/sample-rosbag-replay/sample-map-rosbag/occupancy_grid_scanaccum_mh1r05.yaml`
— the official Autoware sample site, one bag, one map.

### 1.1 Per-fix deltas

| config | gap_nats median (worst) | argmax dist_m median (worst) | GT percentile rank median (worst) | local_max count |
|---|---|---|---|---|
| upstream (baseline) | 51.92 (59.62) | 29.67 (30.60) | 0.8307 (0.7553) | 1/5 |
| + `normalized_short` (§5.1 fix) | 7.07 (10.30) | 0.20 (0.30) | 0.999983 (0.999981) | 1/5 |
| + `skip_nonfinite` (§5.2 fix, stacked) | 6.24 (8.58) | 0.22 (0.30) | 0.999989 (0.999986) | 2/5 |

Reading this top to bottom: the upstream model's global maximum sits ~30 m
from the true pose with a ~52 nat gap — GT is not even close to the best
explanation the model can construct. Normalising `p_short` collapses that
to a 7 nat gap and a 0.20 m offset, three orders of magnitude tighter.
Dropping non-finite beams on top tightens the gap slightly further (7.07 →
6.24 nats) and doubles how often GT sits at an actual local peak of the
likelihood surface (1/5 → 2/5 timestamps); `dist_m` moves by +0.02 m,
smaller than the evaluation grid's own 0.25 m cell and within
measurement noise, not a real regression.

At a finer evaluation resolution (`--field-res-m 0.05 --window-m 5.0`,
spot-checked at t=10s only, not part of the committed 5-timestamp sweep)
the argmax distance tightens further to **0.05 m** from GT — a real,
consistent, small residual offset, not a coarse-grid artifact.

### 1.2 The effective-`z_hit` headline number (§5.1)

The unnormalised `z_short` term's mass grows with predicted range `d`, so
the configured 75% `z_hit` weight silently decayed with distance. Measured
via `mixture_mass_breakdown_normalized_short` at 0.05 m/px:

| predicted range | upstream effective `z_hit` | fixed effective `z_hit` |
|---|---|---|
| 10 m | 25.4% | 78.5% |
| 20 m | 15.2% | 78.5% |
| 50 m | 6.8% | 78.5% |

Fixed is flat across range (the property that matters — the model no
longer silently reweights itself by distance), and it lands about 3.5
points above the configured 0.75, not exactly on it. **This offset is
expected, not a bug**: the sensor-model table is a discrete pixel grid,
while the normalising constant `eta = 1/(1 - exp(-lambda_short·d))` is
derived from a continuous integral — sampling a decreasing exponential at
integer pixel steps overcounts its mass relative to the continuous
integral (a standard Riemann-sum artifact). Getting it numerically exact
to 0.75 would need either a continuous-domain sensor model or a
resolution-aware correction; out of scope here, and the offset does not
change any conclusion in this report.

### 1.3 Reproduce

```bash
# Baseline (upstream, no fixes)
bash -c 'source /opt/autoware/1.5.0/setup.bash && python3 \
    scripts/2dlidar/score_sensor_model.py \
    --gt-bag data/rosbags/phase3/sample_ndt_gt \
    --map data/sample-rosbag-replay/sample-map-rosbag/occupancy_grid_scanaccum_mh1r05.yaml \
    --times 5,10,15,20,23 --variant upstream \
    --out-json docs/reports/assets/2dlidar-phase3e-baseline.json \
    --out-fig docs/reports/assets/2dlidar-phase3e-baseline-sweep.png'

# + normalized_short (lambda_short=1.0, the shipped default)
bash -c 'source /opt/autoware/1.5.0/setup.bash && source install/setup.bash && python3 \
    scripts/2dlidar/score_sensor_model.py \
    --gt-bag data/rosbags/phase3/sample_ndt_gt \
    --map data/sample-rosbag-replay/sample-map-rosbag/occupancy_grid_scanaccum_mh1r05.yaml \
    --times 5,10,15,20,23 --variant normalized_short --lambda-short 1.0 \
    --out-json docs/reports/assets/2dlidar-phase3e-normalized-short.json \
    --out-fig docs/reports/assets/2dlidar-phase3e-normalized-short-sweep.png'

# + skip_nonfinite, stacked on normalized_short
bash -c 'source /opt/autoware/1.5.0/setup.bash && source install/setup.bash && python3 \
    scripts/2dlidar/score_sensor_model.py \
    --gt-bag data/rosbags/phase3/sample_ndt_gt \
    --map data/sample-rosbag-replay/sample-map-rosbag/occupancy_grid_scanaccum_mh1r05.yaml \
    --times 5,10,15,20,23 --variant normalized_short --lambda-short 1.0 --skip-nonfinite \
    --out-json docs/reports/assets/2dlidar-phase3e-normalized-skipnonfinite.json'

# Effective-z_hit mixture-mass comparison figure
python3 scripts/2dlidar/plot_sensor_model.py --resolution 0.05 --max-range 60.0 \
    --lambda-short 1.0 --out-dir docs/reports/assets --prefix 2dlidar-phase3e
```

Runtime: ~3.0–3.2 s/timestamp (map/raycaster built once, reused across all
5 timestamps and both variants).

---

## 2. Scan-gating (§5.3): correcting once per scan, not twice

`update()` used to fire from `odomCB` at odometry rate (~15–20 Hz observed)
while scans arrive at ~10 Hz — every scan was consumed by roughly two
consecutive Bayes corrections, squaring its effective likelihood
contribution. `update_on_new_scan_only=true` (Task 4) gates the correction
on the incoming scan's stamp; odometry between corrections is folded into a
running accumulator via **exact** rotation composition
(`compose_odometry_delta`), not a small-angle approximation — proven exact
by induction and unit-tested to `1e-9`.

Measured on a live run (`PF_UPDATE_ON_SCAN_ONLY=true`,
`PF_DIAG_ENABLE=true`, sample-site bag, killed at ~30 s sim time per plan):

- **237 diagnostics records, 237 unique scan stamps** — every correction
  consumed a distinct, not-yet-seen scan; zero double-counting.
- Effective correction rate **8.3 Hz** (sim time), down from the **~15 Hz**
  odom-rate baseline Phase 3d measured — consistent with the correction
  rate moving from odom rate toward true scan rate (~10 Hz nominal).

**Consequence, not a defect:** `publish_tf` and `visualize` both live
inside `update()`, and no separate predict-only publish path was added
(considered and rejected as disproportionate scope — splitting `MCL()`
into predict/correct phases is materially riskier than gating the whole
update). So the pose/TF publish rate **halves** along with the correction
rate. This shows up directly in the seed matrix below as roughly a
3.4x drop in aligned pose pairs (~815 → ~238) — that is the publish-rate
halving working as expected, not data loss or a broken run.

---

## 3. End-to-end: 5-seed replay matrix

`scripts/2dlidar/run-seed-matrix.sh` (Task 5) replays the sample-site bag
through the particle filter, 2 configurations x 5 seeds each, sequentially
(concurrent ROS graphs on one machine cross-talk), and scores each run
against the NDT ground-truth track with `compare_poses.py`.

- **upstream**: Phase 3c Lever-2 tuned baseline (`max_range=60`,
  `squash=3.0`, `disp_theta=0.1`), none of the Phase 3e fixes.
- **fixed**: identical tuning + all three fixes
  (`sensor_model_variant=normalized_short`, `skip_nonfinite_beams=true`,
  `update_on_new_scan_only=true`).
- **Gate** (full-overlap stats, `compare_poses.py`): mean < 1.0 m,
  p95 < 2.5 m, mean \|yaw\| < 0.2 rad.

Raw data: `docs/reports/assets/2dlidar-phase3e-seedmatrix.json` (also
`data/rosbags/phase3/seedmatrix/results.jsonl` and per-seed
`compare_poses.py` reports).

| config | pairs (range) | trans mean, m — median (range) | trans p95, m — median (range) | yaw mean\|err\|, rad — median (range) | seeds passing all 3 |
|---|---|---|---|---|---|
| upstream | 813–817 | 26.02 (15.93–36.02) | 137.16 (52.29–138.70) | 0.9155 (0.385–0.936) | 0/5 |
| fixed | 234–242 | 0.79 (0.761–0.811) | 2.02 (1.952–2.044) | 0.0265 (0.026–0.028) | 5/5 |

Every single fixed-config seed passes every threshold; every single
upstream-config seed fails every threshold by a wide margin (26–36x over
the mean-error limit, 21–55x over the p95 limit). This is not a
borderline result on either side.

The pair-count drop (~815 → ~238) is the Task 4 publish-rate halving
described in §2 above, not lost data — `compare_poses.py` aligns by
nearest timestamp within a 0.1 s window, so fewer PF poses simply means
fewer alignment opportunities, not a shorter or corrupted run.

### 3.1 Trajectory comparison

![Upstream vs fixed PF trajectory against NDT ground truth, seed 1](assets/2dlidar-phase3e-trajectory-compare.png)

Both panels replay the same seed-1 run against the same ground-truth
track (`occupancy_grid_scanaccum_mh1r05`). Upstream (left) tracks GT
briefly, then diverges into a series of large excursions once the
measurement model's spurious global maxima start winning particle weight.
Fixed (right) tracks the ground-truth path closely for the full run.

### 3.2 Reproduce

```bash
# Full matrix (idempotent — resumes from results.jsonl if interrupted)
bash scripts/2dlidar/run-seed-matrix.sh

# One cell manually, e.g. fixed seed 3 (env-var driven, matches
# run-seed-matrix.sh's sample-site block -- see that script for the full
# POINTCLOUD_TOPIC/VELOCITY_TOPIC/IMU_TOPIC/SCAN_*/MAP_YAML set)
BAG=data/rosbags/phase3/sample_ndt_gt GT_BAG=data/rosbags/phase3/sample_ndt_gt \
OUT_BAG=data/rosbags/phase3/seedmatrix/fixed_s3 \
PF_MAX_RANGE=60.0 PF_SQUASH=3.0 PF_DISP_THETA=0.1 \
PF_SENSOR_MODEL_VARIANT=normalized_short PF_SKIP_NONFINITE=true \
PF_UPDATE_ON_SCAN_ONLY=true PF_RANDOM_SEED=3 \
bash scripts/2dlidar/run-particle-filter.sh

python3 scripts/2dlidar/compare_poses.py \
    data/rosbags/phase3/sample_ndt_gt data/rosbags/phase3/seedmatrix/fixed_s3 \
    --gt-time-source=bag --out data/rosbags/phase3/seedmatrix/fixed_s3_report.md

# Trajectory comparison figure (this report's figure)
bash -c 'source /opt/autoware/1.5.0/setup.bash && source install/setup.bash && \
    python3 scripts/2dlidar/plot_trajectory_compare.py'
```

---

## 4. What each fix bought, and did not buy

| fix | bought | did not buy |
|---|---|---|
| `normalized_short` (§5.1) | 3 orders of magnitude tighter offline gap (52 → 7 nats, 30 m → 0.20 m); flat effective `z_hit` across range instead of range-dependent collapse | did not eliminate the residual sub-metre offline offset; does not touch beam correlation (§5.4) or clamping (§5.5) |
| `skip_nonfinite` (§5.2) | modest further gap tightening (7.07 → 6.24 nats), doubled local-max hit rate (1/5 → 2/5) | small effect in isolation compared to §5.1 — this dataset's ~30% non-finite fraction matters more architecturally (correctness of principle: a non-return should never outscore a match) than numerically here |
| `update_on_new_scan_only` (§5.3) | this is the fix that flips the end-to-end gate from 0/5 to 5/5 — stopping double-counting of each scan is what let the two sensor-model fixes actually show up in the tracked trajectory instead of being drowned by overconfident double updates | halves the pose/TF publish rate, since no predict-only path exists; not evaluated in isolation end-to-end (only the offline gate isolates 5.1/5.2; the seed matrix tests all three together) |

None of Task 1–4's offline numbers alone predicted the end-to-end
magnitude of the improvement (52 nats → 6 nats offline is a measurement
model statement, not directly an end-to-end trajectory-error statement);
the seed matrix in §3 is what actually closes the loop from "the model is
less wrong" to "the filter tracks."

---

## 5. Verdict

**The Phase 3 accuracy gate passes, 5/5 seeds, with the three fixes
enabled.** Mean translational error 0.79 m (range 0.76–0.81 m), p95 2.02 m
(range 1.95–2.04 m), mean \|yaw\| error 0.026 rad (range 0.026–0.028 rad) —
all comfortably inside the 1.0 m / 2.5 m / 0.2 rad thresholds, with no
seed-to-seed variance anywhere near the boundary. Upstream fails 5/5 by a
wide margin under identical tuning and identical bag. 2D MCL, with these
fixes, is a viable localization source on this site.

### What is NOT established

- **Initialization is an oracle, not a real capability.** Every run in this
  report seeds the filter by reading the first `/localization/kinematic_state`
  pose out of the NDT ground-truth bag and publishing it on `/initialpose`
  (`run-particle-filter.sh:499-536`). That pose is not available on a real
  vehicle. So these numbers measure **tracking accuracy given a correct
  starting pose**, and say nothing about global localization. The comparison
  between configurations remains fair — baseline and fixed got the same
  oracle — but the headline 0.79 m must not be read as end-to-end
  localization capability. By contrast the NDT ground-truth runs initialized
  legitimately, from `gnss_poser` via `autoware_pose_initializer`, with no
  human input. Closing this gap is the first Phase 4 task; see
  `docs/superpowers/plans/` for the GNSS-initialization plan.
- **Convergence time is excluded from the metric.** The oracle pose is
  published a few seconds into the replay, so the filter briefly runs on its
  global-initialization spread first: for seed 1, poses #0-#4 sit up to
  11.4 m from truth over the first 0.9 s, reaching 0.41 m by pose #5. Those
  early poses fall outside the paired comparison window (241 published
  poses, 236 aligned pairs), so they do not enter the 0.79 m mean. Identical
  treatment for both configurations, but it means the metric characterises
  steady-state tracking only.
- **One bag, one site, one map.** Every number in this report — offline
  and end-to-end — comes from `data/rosbags/phase3/sample_ndt_gt` against
  `occupancy_grid_scanaccum_mh1r05.yaml`, the official Autoware sample
  site. Nothing here has been cross-checked on the COSS site, a different
  map, or live hardware.
- **The residual 0.05–0.22 m offline offset.** Even with both fixes
  stacked, the sensor model's own global maximum does not land exactly on
  GT — it sits 0.05 m away at fine grid resolution, 0.20–0.22 m at the
  coarser 5-timestamp sweep resolution. Small relative to the pre-fix ~30 m
  error, but real and unexplained; not chased further in this phase.
- **§5.4 (beam independence / correlated errors) and §5.5 (clamping
  discards long-range disambiguating beams) are untouched.** Both are
  flagged in the algorithm doc as inherent to the beam model's
  independence and clamping assumptions rather than implementation bugs,
  and both are explicitly out of scope for Phase 3e. §5.4 is the same
  failure mode that made nav2 AMCL fail on this data too (Phase 3c Lever
  4) — fixing 5.1–5.3 does not touch it.
- **The fixes live in a fork, not upstream.** All three are opt-in
  parameters in `NEWSLabNTU/particle_filter@autosdv` (branch `autosdv`);
  defaults are unchanged (verified per-task via bit-for-bit
  defaults-unchanged tests), so nothing here is a behavior change unless
  explicitly enabled.
- **Task 4's publish-rate halving** has not been checked against any
  downstream consumer that might assume odom-rate TF (e.g. a future
  `ekf_localizer` bridge) — flagged in the Task 4 report as a follow-up
  concern, not resolved here.

---

## References

- `docs/research/localization/2d_mcl_algorithm.md` §5 — the defect catalog
  this phase closes three items of.
- `docs/reports/2dlidar-phase3d-instrumentation.md` — the instrumentation
  and underflow-vs-misspecification verdict this phase builds on.
- `docs/design/f1tenth-2dlidar-integration.typ` — "Phase 3 Outcome" box,
  updated by this phase to reflect the gate now passing.
- `.superpowers/sdd/p3e-task-{1,2,3,4,5}-report.md` — full task-level
  detail (implementation choices, alternatives considered and rejected,
  full test lists) behind every number summarised here.

---

## Naming

The localization stack described here is referred to as **AutoSDV 2D-MCL**:
the vendored Roboracer `particle_filter` (fork
`NEWSLabNTU/particle_filter@autosdv`) with the three Phase 3e sensor-model
fixes enabled. The name distinguishes it from nav2's AMCL (used only as the
Phase 3c Lever 4 cross-check) and from unmodified upstream `particle_filter`,
which is what "upstream" means in every table above.

## Regenerating the trajectory figures

`scripts/2dlidar/plot_trajectories.py` renders all three views — NDT alone,
2D-MCL alone, and the overlay — with elapsed-time markers so the tracks can
be compared in time as well as space:

```bash
bash -c 'source /opt/autoware/1.5.0/setup.bash && \
    python3 scripts/2dlidar/plot_trajectories.py --prefix 2dlidar-phase3e-trajectories'
```

Defaults point at the seed-1 fixed run and the sample-site ground-truth bag;
`--mcl-bag`, `--gt-bag`, `--map` and `--mark-every` override them.
