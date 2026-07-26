# Phase 4 Task 1: replacing the oracle initialization with real GNSS auto-init

Phase 3e's "What is NOT established" section flagged the elephant in the
room: every Phase 3e run seeded AutoSDV 2D-MCL by reading the first
`/localization/kinematic_state` pose out of the NDT ground-truth bag and
publishing it on `/initialpose`
(`scripts/2dlidar/run-particle-filter.sh:499-536` at the time). That is an
oracle no real vehicle has — it measures tracking accuracy given a correct
starting pose, not global localization. This task replaces it with
Autoware's real GNSS auto-init path and re-measures the Phase 3 gate under
honest initialization.

---

## 1. Wiring choice

**Option chosen: run `autoware_gnss_poser` directly and relay its output to
`/initialpose`** (a thin, oracle-free relay — not a new initializer),
rather than standing up the full `autoware_pose_initializer` +
`autoware_automatic_pose_initializer` service chain used by
`tier4_localization_launch`'s `pose_twist_estimator.launch.xml`.

Why: `autoware_pose_initializer_node` is a statically-typed C++ node whose
constructor also wires a `map_height_fitter` (services
`~/pointcloud_map`, `~/partial_map_load`, `~/vector_map` from
`autoware_map_loader`), an `ndt_align` service client, and EKF/NDT trigger
clients. This lightweight PF harness runs `nav2_map_server` against a 2D
occupancy grid — it has none of the pointcloud-map/vector-map/EKF
machinery `pose_initializer` expects, and standing all of that up just to
re-derive a message AutoSDV's PF ignores half of anyway (z/roll/pitch)
would mean building a second, parallel NDT-GT-style stack purely to
produce a five-field pose. `gnss_poser`'s job — NavSatFix + a real MGRS
projection (`autoware::geography_utils`) → map-frame pose — is the
substantive, non-trivial machinery worth reusing; converting its output to
`/initialpose` is a direct type match (`gnss_poser` publishes
`geometry_msgs/msg/PoseWithCovarianceStamped` on `gnss_pose_cov`, exactly
what `/initialpose` expects) requiring no field-level conversion.

**New script:** `scripts/2dlidar/gnss_init_seed.py`. Given a source bag of
raw `sensor_msgs/msg/NavSatFix`, it starts three real Autoware nodes as
subprocesses — `robot_state_publisher` (over the sample vehicle +
`sample_sensor_kit` xacro, see §1.1), `autoware_map_projection_loader`
(loads the sample map's `map_projector_info.yaml`, MGRS grid `54SUE` — the
same grid the occupancy-grid map and the NDT ground truth live in), and
`autoware_gnss_poser` — publishes the source bag's first two NavSatFix
messages to `gnss_poser`'s input, and prints the resulting
`gnss_pose_cov` message (position, orientation, and covariance diagonal)
to stdout before tearing everything down. Nothing in this script
reimplements gnss_poser's coordinate transform or the vehicle's mounting
geometry; it only starts real nodes, feeds them, and reads the result.

### 1.1 Correction made mid-task: identity-TF fallback → real static transform

First pass fed `gnss_poser` from `data/sample-rosbag-replay/sample-rosbag-migrated`
(the raw Autoware sample bag — the only bag on this site carrying
`/sensing/gnss/ublox/nav_sat_fix`, 30 msgs) without any `/tf_static`. That
bag has **no** `/tf_static` topic at all (confirmed via `ros2 bag info`);
only the enriched `sample_ndt_gt` GT bag has one (5 static transforms,
recorded from the full `logging_simulator`'s `robot_state_publisher`), and
reading GT_BAG's `/tf_static` to feed the seed would smuggle the very
ground-truth dependency this task exists to remove back in through the
side door. Without any `/tf_static`, `gnss_poser`'s
`get_static_transform(gnss_link, base_link, ...)` lookup failed and it
silently fell back to an **identity** gnss→base_link transform (its own
source catches the `tf2::TransformException` and logs `"Please publish TF
gnss_link to base_link"` rather than erroring out — not a hang, a quiet
approximation). The fix: `gnss_init_seed.py` now runs a real
`robot_state_publisher` over `tier4_vehicle_launch/urdf/vehicle.xacro`
with `vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit` — the
same xacro invocation `tier4_vehicle_launch/vehicle.launch.xml` uses on a
real bring-up — so the full `base_link → ... → gnss_link` static chain is
published for real, sourced from the vehicle/sensor-kit URDF (which any
real vehicle has), not from any recorded bag. This is still fully
oracle-free: only GNSS fixes, `map_projector_info.yaml`, and the URDF's
static mounting geometry feed the seed; `/localization/kinematic_state`
is never read.

Effect on the seed: the TF warning disappeared and the position seed
tightened from 1.03 m to **0.58 m** from NDT GT (see §2) — the antenna
mount offset was itself around half a metre. The yaw error was unaffected
(§2 explains why: it comes from the heading-estimation method, not the
mounting transform).

### 1.2 `INITPOSE_SOURCE` switch

`scripts/2dlidar/run-particle-filter.sh` gained `INITPOSE_SOURCE`
(`gt_bag`, default — today's oracle, byte-identical to every Phase 3e
run — or `gnss`), plus `GNSS_BAG` / `GNSS_FIX_TOPIC` /
`MAP_PROJECTOR_INFO_YAML` / `LANELET2_MAP_OSM` (only consulted when
`INITPOSE_SOURCE=gnss`). When `gnss`, the script runs
`gnss_init_seed.py` **before** writing `pf_params.yaml` (the spread
parameters below are read once at particle_filter startup and cannot be
changed after the node comes up), then publishes the resulting pose (and
its real covariance, not the oracle's fixed 0.25 m²/0.068 rad² matrix) on
`/initialpose` at the same `INITPOSE_DELAY` as before. Unset, every
existing invocation is unchanged.

`scripts/2dlidar/run-seed-matrix.sh` gained `MATRIX_CONFIGS` (default
`"upstream fixed"`, unchanged) and `MATRIX_OUT_SUBDIR` (default
`seedmatrix`, unchanged) so a `MATRIX_CONFIGS="fixed" INITPOSE_SOURCE=gnss
MATRIX_OUT_SUBDIR="seedmatrix_gnssinit"` run writes to its own
`results.jsonl`/output directory without touching Phase 3e's.

---

## 2. Covariance handling: real spread instead of a fixed oracle sigma

`initialize_particles_pose()` in the `particle_filter` fork ignores the
incoming `/initialpose` covariance entirely (confirmed by direct source
read; documented in `run-particle-filter.sh`'s source-findings block) —
it always drew from a hardcoded `sigma=0.5 m / 0.4 rad` Gaussian. That is
sized for a centimetre-scale oracle seed; passing it unchanged to a
metre-scale, rad-scale GNSS seed would produce an identically-tight
particle cloud regardless of source, silently discarding exactly the
uncertainty information the honest seed carries.

**Fix applied (not deferred):** `particle_filter/particle_filter.py`
(fork `NEWSLabNTU/particle_filter@autosdv`) gained two new parameters,
`init_spread_xy_m` (default `0.5`) and `init_spread_theta_rad` (default
`0.4`) — declared with the exact prior hardcoded values as defaults, so
`INITPOSE_SOURCE=gt_bag` runs are bit-for-bit unchanged. `run-particle-filter.sh`
derives `PF_INIT_SPREAD_XY`/`PF_INIT_SPREAD_THETA` from `gnss_poser`'s own
output covariance (`sqrt(max(cov_xx, cov_yy))`, `sqrt(cov_yaw)`) when
`INITPOSE_SOURCE=gnss` and the caller hasn't overridden them explicitly —
on this run that resolved to **1.977 m** / **1.0 rad**, roughly 4x/2.5x
wider than the oracle's spread, giving the filter a real chance to recover
from a GNSS-scale seed error via resampling instead of starting collapsed
around a possibly-wrong hypothesis. This is the cheap "pass a sensible
spread through" version the task allowed in lieu of literally consuming
the 6x6 covariance matrix (still ignored past the diagonal; full
covariance-aware initialization is future work, not attempted here).

This is real, not cosmetic: the actual yaw seed error on this run was
**1.81 rad** (see below) — inside 2σ of the derived 1.0 rad spread, but
would have been ~4.5σ under the oracle's fixed 0.4 rad spread, i.e.
essentially unrecoverable without this change.

---

## 3. Diagnostic run: does the seed arrive, does the filter converge?

`INITPOSE_SOURCE=gnss`, seed 1, sample-site bag/map (same config as the
Phase 3e "fixed" cell). Full run: `data/rosbags/phase3/gnssinit/diag_s1`.

**GNSS-derived seed pose** (from `gnss_poser`, robot_state_publisher +
map_projection_loader live):

```
x=89570.626  y=42300.899  z=37.980
qx=-0.00415  qy=-0.00627  qz=-0.57739  qw=0.81643
cov_xx=3.909 m²  cov_yy=3.909 m²  cov_yaw=1.0 rad²
(from NavSatFix messages at t=0.0s, t=0.986s into the raw sample bag)
```

**Seeding error vs NDT ground truth at the matching bag-relative time**
(NDT GT pose at t≈0s: `x=89571.141 y=42301.172`, yaw `0.583 rad`):

- **Position: 0.58 m** — well inside the GNSS 1σ (√3.909 ≈ 1.98 m), a
  strong result for raw single-fix GNSS.
- **Yaw: 1.81 rad** (≈104°) — large, and expected: `use_gnss_ins_orientation`
  is `false` here (no dedicated GNSS-INS orientation topic exists on this
  site), so `gnss_poser` falls back to
  `get_quaternion_by_position_difference()` — a course-over-ground heading
  from consecutive fixes. The vehicle moved only ~0.05 m between the two
  fixes used (0.986 s apart, near-stationary at bag start), so that
  heading estimate is dominated by GNSS position noise, not real motion.
  This is a known, honest limitation of single-antenna GNSS heading at low
  speed — not a bug in this integration. See §5 for why it's deferred
  rather than patched.

**Convergence trace** (poses paired to nearest-timestamp NDT GT,
`t=0` = bag start; compare to Phase 3e's oracle trace: "poses #0-#4 up to
11.4 m off, 0.41 m by #5"):

| pose # | t (s) | dist (m) | yaw err (rad) |
|---|---|---|---|
| 0–11 | −3.4 to −0.3 | 43–60 | 1.77–2.96 | *(pre-seed: PF's own default/global init, before `/initialpose` arrives at t≈0)* |
| 12 (first post-seed) | 0.05 | **0.531** | **0.020** |
| 13 | 0.60 | 0.453 | 0.035 |
| 17 | 1.33 | 0.400 | 0.036 |
| 20 | 2.20 | 0.375 | 0.036 |
| 24 | 3.20 | 0.374 | 0.037 |

Despite a 1.81 rad seed yaw error, the filter is already at 0.53 m / 0.02
rad on the very first post-seed pose and stable at ~0.37–0.4 m within 3
seconds — the wide GNSS-covariance-derived spread (§2) let the sensor
model resolve the yaw ambiguity essentially immediately once real scan
corrections began, rather than converging from the seed's nominal 1.81 rad
error. Full-overlap comparison for this run: **mean 0.80 m, p95 2.00 m,
yaw 0.028 rad — PASS** (`data/rosbags/phase3/gnssinit/diag_s1_report.md`).

---

## 4. 5-seed matrix under honest initialization

`MATRIX_CONFIGS="fixed" INITPOSE_SOURCE=gnss MATRIX_OUT_SUBDIR="seedmatrix_gnssinit"
bash scripts/2dlidar/run-seed-matrix.sh` — fixed config only (Phase 3e's
three fixes: `normalized_short`, `skip_nonfinite_beams`,
`update_on_new_scan_only`), 5 seeds, real GNSS init each time (fresh
`gnss_poser` invocation per seed — the seed pose is not cached across
cells). ~2m15s/cell (~11 min total), dominated by `robot_state_publisher`/
`gnss_poser`/`map_projection_loader` startup plus the ~50 s CDDT
precompute already present in every run.

Raw data: `docs/reports/assets/2dlidar-phase4-gnssinit-seedmatrix.json`
(also `data/rosbags/phase3/seedmatrix_gnssinit/results.jsonl` and
per-seed `compare_poses.py` reports).

| seed | pairs | trans mean (m) | trans p95 (m) | yaw mean\|err\| (rad) |
|---|---|---|---|---|
| 1 | 224 | 0.8008 | 2.0489 | 0.0266 |
| 2 | 229 | 0.8580 | 2.0882 | 0.0281 |
| 3 | 227 | 0.7607 | 1.9809 | 0.0263 |
| 4 | 228 | 0.7723 | 2.0154 | 0.0267 |
| 5 | 228 | 0.7707 | 1.9951 | 0.0257 |

**Median (range):** mean 0.7723 m (0.7607–0.8580), p95 2.0154 m
(1.9809–2.0882), yaw 0.0266 rad (0.0257–0.0281).

**Gate (mean < 1.0 m, p95 < 2.5 m, yaw < 0.2 rad): PASS, 5/5 seeds.**

These numbers are essentially indistinguishable from Phase 3e's
oracle-seeded "fixed" results (median mean 0.79 m, p95 2.02 m, yaw 0.0265
rad, range 0.76–0.81 m / 1.95–2.04 m / 0.026–0.028 rad) — honest GNSS
initialization costs nothing measurable on this site once the filter
reaches steady state, because (a) the position seed is already close
(0.5–1 m class, well inside the sensor model's basin of attraction) and
(b) the covariance-derived spread (§2) lets the filter absorb the large
yaw seed error within the first second, before the full-overlap
comparison window even starts. The gate result changes from "passes under
an oracle no real vehicle has" to "passes under a seed a real vehicle can
actually produce" — this is the headline result of this task.

---

## 5. Figures

`scripts/2dlidar/plot_trajectories.py --mcl-bag data/rosbags/phase3/seedmatrix_gnssinit/fixed_s1
--prefix 2dlidar-phase4-gnssinit` (unmodified script, per plan):

- `assets/2dlidar-phase4-gnssinit-ndt.png` — NDT ground truth alone.
- `assets/2dlidar-phase4-gnssinit-mcl.png` — 2D-MCL alone (GNSS-seeded
  seed-1 run).
- `assets/2dlidar-phase4-gnssinit-overlay.png` — both tracks on one axis;
  the two lines are visually near-coincident for the full ~58 s run, start
  and end markers essentially on top of each other.

---

## 6. What remains unestablished / deferred

- **Yaw seeding relies on course-over-ground heading, which is unreliable
  near-stationary.** `use_gnss_ins_orientation=false` was used because
  this site's bag carries no dedicated GNSS-INS orientation topic; the
  resulting 1.81 rad yaw seed error (§3) is real and would be worse on a
  site where the vehicle is still slower-moving at power-on. **Not
  patched here** because the covariance-derived spread (§2) already
  absorbs it on this site within ~1 s of real scan corrections, and a
  proper fix (dual-antenna GNSS-INS orientation, or waiting for a
  longer/faster baseline before trusting the heading) is a sensor/protocol
  decision outside this task's scope — flagged as the natural next Phase 4
  task if a site is found where the wider spread isn't enough to recover.
- **One bag, one site, one map**, same caveat as Phase 3e — nothing here
  crosses to COSS or live hardware.
- **Position seed accuracy (0.58 m) is specific to this bag's GNSS
  quality** (`position_covariance` ≈ 3.9 m² diag, i.e. σ≈2 m single-fix
  GPS) and this map's known-good MGRS alignment; a lower-quality receiver
  or an uncalibrated map projector would seed worse.
- **Full 6x6 covariance is still not consumed** by `initialize_particles_pose`
  (only the diagonal, converted to two scalar spreads outside the filter) —
  documented in §2, not attempted further here.
- **The `pose_initializer`/`automatic_pose_initializer` service chain was
  deliberately not stood up** (§1) — if a future task needs the full
  Autoware localization stack's retry/EKF-trigger/NDT-align machinery
  (e.g. for a live vehicle bring-up rather than this offline PF harness),
  that is a materially different integration, not a superset of this one.
