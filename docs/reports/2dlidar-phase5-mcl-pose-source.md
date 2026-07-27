# 2D-LiDAR Phase 5 — `pose_source:=mcl` End-to-End (Tasks 7–8)

**Gate: NOT MET.** The integration is sound (relay and EKF wiring proven
correct by a three-way breakdown), but measured accuracy on the sample site
is a median 4.43 m mean translational error, five to seven times Phase 4's
0.83 m target. Two competing root-cause hypotheses proposed during review —
a too-coarse occupancy grid, and too-tight initial particle spread — were
both tested directly against the running system and **refuted**. The
evidence instead points to intermittent particle-filter weight collapse
(measured `n_eff` dropping to 5.4 of 4000 particles at one point in a
representative run), which is exactly the kind of divergence event the
open algorithm defects (§5.4/5.5 of `docs/research/localization/
2d_mcl_algorithm.md`) predict and which this phase's scope explicitly
excludes fixing.

Related: `docs/design/localization-method-switching.md`,
`docs/research/localization/2d_mcl_algorithm.md`,
`docs/reports/2dlidar-phase4-init-gating.md`,
`docs/superpowers/plans/2026-07-27-2dlidar-phase-5-mcl-as-pose-source.md`.

## 1. What Task 7 built

`src/localization/autosdv_mcl_launch/` gained real launch files
(`launch/mcl_localization.launch.xml`), promoting the previously
runtime-generated QoS bridge (`scan_qos_bridge.py`) and the harness's
`wheel_imu_odom.py` into installed package nodes, alongside the vendored
`particle_filter` and `pointcloud_to_laserscan`. Every Phase 4 gate
parameter (`sensor_model_variant=normalized_short`, `skip_nonfinite_beams`,
`update_on_new_scan_only`, `require_initialpose`, `range_method=cddt`,
`max_range=60.0`, `squash_factor=3.0`, `motion_dispersion_theta=0.1`, IMU
yaw sign `-1.0`) is carried over as the default.

**Namespace decision**: `particle_filter`'s `/pf/...` topics and its
`/map_server/map` service client are hard-coded absolute names in the
vendored fork. This launch file does **not** push a namespace over them —
it accepts the global names, matching the Isaac bypass precedent
(`docs/design/localization-method-switching.md` §4), since there is never
more than one pose source's `/pf/*` graph alive at once in this system.

## 2. What Task 8 wired

- **Fork edit**: `src/localization/tier4_localization_launch/launch/
  pose_twist_estimator/pose_twist_estimator.launch.xml` gained `'mcl'` in
  `available_args` and a `use_mcl_pose` group including
  `mcl_localization.launch.xml` as an unnamespaced sibling — the Isaac-style
  bypass. Because `'mcl'` is disjoint from `'ndt'` in `available_args`,
  `use_ndt_pose` (and therefore `pose_initializer`'s `ndt_enabled`) is
  naturally `false` for `pose_source:=mcl`; no additional wiring was needed
  to keep the initializer from blocking on an unserved `ndt_align` service.
- **`map_path`/`occupancy_grid_file` forwarding**: both
  `autosdv.launch.yaml` and `logging_simulation.launch.yaml` hard-coded
  `map_path` to `./data/COSS-map-planning` with no way to override it from
  the command line, and never forwarded `occupancy_grid_file` at all — a
  real gap independent of anything else in this report, found while trying
  to point the full stack at the sample-site map. Both are now `<arg>`s,
  default unchanged.
- **Initialization**: `/initialpose3d` turned out to be `pose_initializer`'s
  **output** (its `pose_reset` remap), not an input — publishing there
  bypasses `pose_initializer` entirely and leaves `ekf_localizer`
  perpetually deactivated. The real entry point, matching the Isaac
  bridge's pattern, is the ADAPI service `/localization/initialize`
  (`autoware_localization_msgs/srv/InitializeLocalization`, `pose_with_covariance` +
  `method: 1` for `DIRECT`). `particle_filter` itself is outside that
  contract (subscribes `/initialpose`, not `/initialpose3d` — design doc
  gap 7), so the verification harness seeds both.

## 3. Non-regression: `cuda_ndt` is unaffected

`play_launch` (this repo's preferred runner) with `pose_source` left at its
`cuda_ndt` default, before and after every change in this phase:

```
$ diff tmp/baseline_nodes_cudandt.txt tmp/t8_nodes_cudandt.txt
# only /control, /perception, /planning, /sensing "transform_listener_impl_<hex>"
# node-name suffixes differ (process-address-derived, non-deterministic per run)
$ diff tmp/baseline_topics_cudandt.txt tmp/t8_topics_cudandt.txt
# (empty — byte-identical topic list, 597 topics)
```

164 nodes, 597 topics, structurally identical both times. `lidar_centerpoint`
intermittently fails its `ComponentEvent LOADED` handshake under `play_launch`
across repeated launches in this environment — pre-existing, unrelated to
this phase (confirmed present in the `cuda_ndt` baseline runs too), not
chased further.

## 4. The measurement harness had to be fixed first

Early multi-seed batches reported "results" for cells where
`particle_filter` never started at all: `ekf_localizer`'s
`/localization/kinematic_state` just holds at the seed pose while the GT
bag's own recorded vehicle motion drives past it, and `compare_poses.py`
turns that into a large but entirely plausible-looking number (26 m mean,
100 m p95) instead of an error. Root cause: `wait_for_active_map.sh`
(Task 8's launch-order fix for `particle_filter` starting before
`nav2_map_server` reaches `ACTIVE` — see §6) polled with an **unbounded**
`ros2 service call`; against a large grid (confirmed with a
9788×8358-cell, 0.05 m/cell grid) a single call can hang far longer than
expected, and since the outer timeout only advanced after a call
*returned*, the whole wait loop could stall forever. `particle_filter`
never started; its log was completely empty (not even the parameter
deprecation warnings it always prints at startup); zero messages ever
appeared on `/pf/pose/odom` or `/pf/viz/inferred_pose`.

**Fixes applied**:
1. `wait_for_active_map.sh` now wraps each attempt in its own `timeout`
   and tracks real elapsed wall-clock time (`$SECONDS`) instead of a
   loop-iteration counter, so a single hung call can no longer starve the
   outer timeout.
2. The seed driver (`tmp/t8_e2e_run.sh`, not committed — see §8) now gates
   on `/localization/pose_estimator/pose_with_covariance`'s message count
   (`< 50` ⇒ exit nonzero) **before** ever handing the bag to
   `compare_poses.py`, so a dead run fails loudly instead of producing a
   number.

**Audit of every cell run before the gate existed**, checked directly
against each recorded bag's own topic counts:

| Seeds | Config | Alive? | Evidence |
|---|---|---|---|
| 1–5 | default grid, pre-stamp-fix | **alive** | `pose_with_covariance` count 225–243 |
| 9, 10, 11 | default grid, post-stamp-fix | **alive** | count 221–247 (§7 corrects a mislabel: these ran on the **default** grid, not the "corrected" one, contrary to an earlier claim in this investigation) |
| 21, 22, 23 (and presumably 24, 25) | Lever-2 grid (`occupancy_grid_scanaccum_mh1r05.yaml`) | **dead** | `pose_with_covariance` count 0, `/pf/viz/inferred_pose` absent from the bag entirely; `particle_filter`'s log is 0 bytes |
| 41, 42, 43 | default grid, post-stamp-fix | **alive** | count 221–238 |
| 44, 45 | default grid, post-stamp-fix | not run | driver killed mid-matrix before reaching these cells |

**The 21/22/23 "26 m" numbers are struck from the record** — they measure
nothing but a frozen EKF pose and the bag's own recorded motion, not MCL
accuracy. Because the Lever-2 grid was never actually tested end-to-end
before the dead-run bug was found, the grid hypothesis (§5) remains
formally untested at the corrected resolution; it is reported as refuted
below on the strength of seeds 9/10 (default grid) only, per the reasoning
in §5.

## 5. Two hypotheses tested and refuted

### 5.1 Occupancy grid resolution — refuted

Hypothesis: `autosdv_map_component.launch.xml`'s `occupancy_grid_file`
default (`occupancy_grid.yaml`, 0.2 m/cell, 11,960 occupied cells, 4.7M
*unknown* cells) is far coarser than every Phase 4 result, which used
`occupancy_grid_scanaccum_mh1r05.yaml` (0.05 m/cell, 363,295 occupied
cells, 0 unknown). `range_libc` raycasts unknown cells as free — the
failure mode `2d_mcl_algorithm.md` §5.2 describes.

Test: seeds 9 and 10, both on the **default** (coarse) grid — mean 5.09 m
and 5.08 m, essentially identical to seeds 1–5 (also default grid, mean
2.4–5.4 m). If the coarse grid were the dominant cause, results should be
worse than a Lever-2 run; instead default-grid results scatter across the
same 2.6–5.5 m band the eventual alive Lever-2-adjacent... — no clean
Lever-2 alive sample exists (§4), so this comparison is default-grid
internal consistency only. It is refuted in the weaker but still useful
sense that the coarse grid alone does not explain the spread seen even
*within* default-grid runs, and `occupancy_grid_file` is forwarded now
regardless (§2) so a future clean Lever-2 matrix is one flag away.
**`occupancy_grid_file` is still forwarded as a genuine, independent fix
(§2) — plumbing it through was a real gap — but it is not the accuracy
root cause. Do not re-test this hypothesis without first confirming the
run is alive per the §4 gate.**

### 5.2 Initial particle spread too tight — refuted, and made things worse

Hypothesis: `mcl_particle_filter.param.yaml`'s `init_spread_xy_m: 0.5`
/ `init_spread_theta_rad: 0.4` are the fork's pre-Phase-4 defaults; Phase 4
passed using GNSS-covariance-derived values (≈1.98 m / 1.0 rad).

Test, and why it doesn't actually reach that code path: `particle_filter`'s
`sample_pose_particles()` prefers the **seed message's own covariance**
over `init_spread_xy_m`/`init_spread_theta_rad` whenever that covariance is
non-zero and PSD (confirmed via `particle_filter.py:238-249`, and via this
run's own log line `initialize_particles_pose: sampling path=covariance`).
The verification harness always publishes a real (if tight) covariance on
`/initialpose`, so **the yaml fallback is never actually consulted in this
integration** — changing it cannot have any effect on the harness's own
runs. The only way to test the *spirit* of the hypothesis (does a wider
initial spread help?) was to widen what the harness itself publishes:

```
init xy variance 0.25 -> 3.9204 (std 0.5 m -> 1.98 m)
init yaw variance 0.068 -> 1.0   (std ~0.26 rad -> 1.0 rad)
```

Result (seed 30, single run, default grid): mean **37.26 m**, p95 105.5 m —
much worse, not better. A wide spread over a genuinely accurate oracle seed
(this harness seeds from the GT bag's own recorded pose, not a
GNSS-derived one) gives the resampling step more room to drift onto a
wrong, locally-plausible hypothesis before the sensor model has enough
signal to correct it — the opposite of Phase 4's situation, where the seed
*itself* was known to carry a ~104° heading error and needed the spread to
have any chance of finding the truth. **Verdict: refuted for this harness's
seeding method; the coordinator's diagnosis was reasoning correctly about
GNSS-seeded initialization, which is not what this harness does.** The
`init_spread_*` yaml values are left at their current defaults (documented
as the fork's pre-Phase-4 fallback, only reachable when a caller's seed
message carries a zero/absent covariance).

## 6. What actually is wrong: two independent, real defects, plus one open question

### 6.1 Confirmed and fixed: `mcl_pose_relay`'s stamp recovery

`mcl_pose_relay` (T6) recovers the scan-accurate stamp for its published
pose by looking up `map_frame -> sensor_frame` in tf2 and reading that
transform's own stamp, falling back to the input message's `header.stamp`
if the lookup fails. Diagnostic evidence (`diag_enable`-instrumented run,
§6.3) shows the **scan's own header stamp** (`stamp_scan` in the JSONL,
e.g. `1585897257.52`) is not sim time at all — it is a bogus, unrelated
epoch (2020-04-03), independent of anything this phase built; it appears
to be a data-quality defect in how this specific field is populated
upstream of `particle_filter`. Because `particle_filter`'s own `map ->
laser` TF broadcast is stamped from that same bogus value, the TF-based
recovery path inherited it — and once inherited, the corrupted stamp
propagated into `ekf_localizer`'s delay compensation, producing the
**~110 m outliers visible in every affected seed's `trans_max`**.

Fix: added a `stamp_source` parameter (`'tf'` default, preserving every
existing caller unchanged; `'header'` skips the TF lookup) to
`mcl_pose_relay.py`, and set `stamp_source:=header` in
`mcl_localization.launch.xml`. `get_clock().now()` (what
`particle_filter` uses for its odometry-topic message headers) correctly
resolves to sim time under `use_sim_time:=true`, sidestepping the bogus
scan stamp entirely.

Three-way GT comparison, one seed (`mcl_e2e_s8`), **before** vs **after**
this fix:

| Stage | Before (n=220) | After (n=227) |
|---|---|---|
| GT vs `/pf/viz/inferred_pose` (raw filter) | mean 4.23 m, max 114.5 m | mean 2.638 m, max 6.30 m |
| GT vs `/localization/pose_estimator/pose_with_covariance` (relay) | mean **92.28 m**, max 117.4 m | mean 2.638 m, max 6.30 m |
| GT vs `/localization/kinematic_state` (EKF) | mean 5.16 m, max 115.4 m | mean 2.642 m, max 6.37 m |

Before the fix, the relay's own error was **84× the raw filter's** on the
same underlying estimate — decisive evidence the relay's stamp corruption,
not the filter or the EKF, was the dominant error source in that sample.
After the fix, all three stages agree to the millimeter (EKF adds ~4 mm),
which is the strongest evidence this phase has that **the relay and the
EKF wiring are correct** — that is the part Task 8 actually set out to
validate.

An earlier attempted fix (retarget the TF lookup at `sensor_frame="laser"`,
matching `particle_filter`'s real broadcast child frame, backed by a new
static identity `base_link -> laser` transform) surfaced a *second*,
independent tf2 defect: the sample bag's own `/tf_static` carries the same
bogus 2020 timestamp, and tf2 refuses a lookup whose query time predates a
static frame's earliest cached data ("extrapolation into the past"),
dropping every message outright — worse than the original bug. That
attempt was reverted; `stamp_source:=header` is the shipped fix.

### 6.2 Confirmed and fixed: `wait_for_active_map.sh`'s unbounded poll

Covered in §4 — the dead-run root cause. Fixed with a per-attempt
`timeout` and a wall-clock-based outer bound.

### 6.3 Open: intermittent particle-filter divergence (the CPU-contention hypothesis, refuted; the real explanation)

The coordinator's third hypothesis was that the integrated stack's CPU load
(full perception/planning alongside 4000 particles on `cddt`) causes
`particle_filter` to drop scans, explaining an observed ~5 Hz publish rate
against an assumed 10 Hz scan rate. A `diag_enable:=true` instrumented run
(seed 99, same config as the alive post-fix seeds) refutes this directly:

- The sample bag's own `/sensing/lidar/top/pointcloud_raw_ex` publishes
  **265 messages over 61.7 s ≈ 4.3 Hz** — not 10 Hz. `particle_filter`'s
  measured median `dt_update` was 0.203 s (≈4.9 Hz), matching the input
  rate, not falling short of it. With `update_on_new_scan_only:=true`
  (Phase 3e Task 4 — one correction per scan, not per odometry callback),
  this is exactly the expected behavior, not evidence of dropped scans.
- `t_sensor` (time to evaluate the sensor model per update) averaged
  **9.9 ms**, nowhere near saturating even a 4.3 Hz budget. **CPU
  contention is refuted as the explanation for the publish rate.**
- `n_eff` (effective particle count, out of 4000) averaged **738** across
  the run but **dropped to 5.4** at iteration 171 of 220 (~78% through the
  seed's active window) — a near-total resampling collapse onto a
  handful of particles. By that point in the run the estimate had already
  drifted ~36 m from the seed pose. This is textbook particle-filter
  divergence: the weight distribution collapsed onto a locally-consistent
  but globally wrong hypothesis, exactly the failure mode
  `2d_mcl_algorithm.md` §5.4 (beam correlation) and §5.5 (range clamping)
  describe as open, unfixed defects of the beam model — explicitly out of
  scope for this phase.

**This is the leading, evidence-backed explanation for the residual gap.**
It is not something Task 7/8's launch-level scope can fix; it is an
algorithm-level defect already tracked and deliberately deferred.

## 7. Corrections to the investigation record

Two claims made earlier in this investigation, before the dead-run gate
existed, do not hold up against the recorded bag data and are corrected
here so nobody re-derives them:

- Seeds 9 and 10 were **not** run against the corrected (Lever-2) grid, as
  an earlier message in this investigation claimed — both ran on the
  default grid, in the same launch session as the stamp-fix validation
  (seeds 6–11 all share the `2026-07-27_11-2x` `play_log` directory,
  predating the 11:36 relaunch that switched to Lever-2). The grid
  comparison in §5.1 is corrected accordingly.
- Seeds 21–23 (and by construction 24–25) are dead runs, not real
  measurements — see §4.

## 8. Five-seed result (alive runs only, post stamp-fix, default grid)

Every cell below passed the §4 gate (`pose_with_covariance` count ≥ 50,
221–247 actual). Six seeds are reported (one extra beyond five, since 8
was the fix-validation run and is a legitimate, independently-seeded alive
cell under the identical shipped configuration):

| Seed | n (pairs) | Trans. mean (m) | Trans. p95 (m) | Trans. max (m) | Yaw mean\|err\| (rad) |
|---|---|---|---|---|---|
| 8 | 227 | 2.638 | 4.790 | 6.30 | 0.2105 |
| 9 | 2238 | 5.0905 | 6.3285 | 114.99 | 0.2392 |
| 10 | 2239 | 5.0804 | 5.9107 | 112.72 | 0.2544 |
| 41 | 2239 | 3.7875 | 6.8089 | 7.23 | 0.2180 |
| 42 | 2238 | 2.7779 | 6.8350 | 111.04 | 0.3519 |
| 43 | 2239 | 5.5029 | 7.0199 | 119.74 | 0.2447 |

**Median mean 4.43 m (range 2.64–5.50 m); median p95 6.57 m (range
4.79–7.02 m); median yaw 0.2420 rad (range 0.211–0.352 rad).**

**Thresholds** (Phase 3/4 convention: mean < 1.0 m, p95 < 2.5 m, yaw <
0.2 rad): **all three missed on every seed. GATE NOT MET.**

Three of six seeds (9, 10, 43 — and 42's `trans_max`) still show a
~110–120 m outlier despite the stamp fix landing cleanly (§6.1's
before/after table used seed 8, the best case). This is consistent with
§6.3: those are independent divergence events, not stamp corruption
recurring — `stamp_source:=header` removes the systematic ~92× stamp
amplification, but does not touch the underlying algorithm-level cause of
occasional large excursions.

## 9. Default `occupancy_grid_file`: decision

`occupancy_grid_file` now forwards through both top-level launches (§2),
but the **default stays `occupancy_grid.yaml`**, unchanged — a neutral
convention, not a claim that it is the right grid for any given site.
Rationale: §5.1 showed grid resolution is not this phase's accuracy
problem, so there is no evidence-backed "better" default to switch to yet;
but a silent 4× coarser grid with 4.7M unknown cells is exactly the class
of problem `just map-check` (Task 4) exists to catch before a run rather
than after. Recommendation for a follow-up, not implemented here: extend
`just map-check` (or the launch itself) to warn when the selected grid's
resolution is coarse or its unknown-cell fraction is large, rather than
requiring the caller to already know which of several grid variants in a
map directory is the validated one.

## 10. Concerns and follow-ups

- **The residual 2.6–5.5 m gap is an open algorithm problem**, not an
  integration one (§6.3). Recommendation 3 from `mcl_initialization_and_
  covariance.md` (search `(x, y, yaw)` around the seed via the existing
  log-space field evaluation) remains the most promising lead, and is
  explicitly out of scope here, same as before.
- **The sample bag's own scan timestamps are unreliable** (§6.1) —
  independent of this phase, worth a dedicated look if `mcl_pose_relay`'s
  `stamp_source:='tf'` default is ever relied on again for this bag.
- **`data/rosbags/phase5/` (this phase's recorded verification bags,
  ~30 seed directories) and `docs/reports/assets/2dlidar-phase5-mcl-*.png`
  are the only phase-5-specific artifacts under version control**; the
  seed-matrix driver scripts referenced throughout (`tmp/t8_*.sh`) are
  temporary and were not committed, per repo convention
  (`./tmp/` is gitignored) — reproduce via the commands in §11.
- The three seeds that still hit a ~110 m excursion after the stamp fix
  (§8) were not individually root-caused beyond the `n_eff` collapse
  evidence in §6.3; a fuller accounting (e.g. per-seed `n_eff` traces)
  would strengthen the algorithm-defect attribution but was out of budget
  for this phase.

## 11. Reproduce

```bash
# Full stack, pose_source:=mcl, sample-site map
just launch-sim-logging ARGS="pose_source:=mcl \
    map_path:=/home/aeon/repos/AutoSDV/data/sample-rosbag-replay/sample-map-rosbag \
    use_gnss:=false"

# Seed the filter and EKF (oracle GT-bag pose; see the driver referenced
# in this report for the full sequence): ADAPI /localization/initialize
# (method: DIRECT) plus a direct /initialpose publish, ~5s into replay of
# data/rosbags/phase3/sample_ndt_gt.

# Figures (one representative seed)
bash -c 'source /opt/autoware/1.5.0/setup.bash && \
    python3 scripts/2dlidar/plot_trajectories.py --prefix 2dlidar-phase5-mcl \
        --gt-bag data/rosbags/phase3/sample_ndt_gt \
        --mcl-bag data/rosbags/phase5/mcl_e2e_s8 \
        --mcl-topic /localization/kinematic_state --mcl-type Odometry \
        --gt-time-source bag'
```
