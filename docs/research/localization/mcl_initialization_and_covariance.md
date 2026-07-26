# 2D-MCL Initialization and Covariance Handling

Study of how AutoSDV 2D-MCL acquires its initial pose, how it (mis)handles the
covariance that comes with that pose, and what Autoware does differently.
Written after Phase 4 Task 1, which replaced the ground-truth oracle seed with
real GNSS auto-init (`docs/reports/2dlidar-phase4-gnss-init.md`).

Companion: `docs/research/localization/2d_mcl_algorithm.md` (the filter itself).

---

## 1. Two defects, both visible in one figure

`docs/reports/assets/2dlidar-phase4-gnssinit-overlay.png` shows the 2D-MCL
track leaving its `0 s` marker roughly 55 m north-east of where the vehicle
actually is, running a straight line down to the true start area, and only then
following ground truth. That segment is not a localization failure — it is the
filter publishing before it has been told anything.

### 1.1 The filter publishes garbage before it is seeded

`particle_filter.py:425` calls `initialize_global()` unconditionally in the
constructor:

```python
def initialize_global(self):
    permissible_x, permissible_y = np.where(self.permissible_region == 1)
    indices = np.random.randint(0, len(permissible_x), size=self.MAX_PARTICLES)
    ...                      # particles spread over the WHOLE map
```

`update()` then runs as soon as map, scan and odometry are all present, and
`visualize()` publishes `expected_pose` — the weighted mean of a map-wide
particle cloud. That mean is a meaningless centroid; in our run it lands at
(89583, 42357), about 55 m from truth. Only when `/initialpose` arrives
(`INITPOSE_DELAY`, default 5 s into the replay) does `clicked_pose` →
`initialize_particles_pose` snap the cloud onto the seed.

Measured consequence, GNSS-seeded seed-1 run: poses **#0–#11 predate the
seed**; the first post-seed pose (#12) is already 0.53 m from truth and settles
to 0.37–0.40 m by t = 3 s.

**Autoware does not behave this way.** Its localization state machine runs
`UNINITIALIZED → INITIALIZING → INITIALIZED`, and nothing downstream consumes a
pose until `pose_initializer` reports success. Global initialization over an
entire map is also a bad idea here specifically: Phase 3c established that this
site aliases badly along its corridor, so a map-wide spread is likely to
converge to a wrong mode rather than a right one.

### 1.2 The covariance that arrives with the seed is discarded

`initialize_particles_pose` (`particle_filter.py:677`) samples each axis
independently from a scalar sigma:

```python
self.particles[:,0] = pose.position.x + np.random.normal(scale=self.INIT_SPREAD_XY_M, ...)
self.particles[:,1] = pose.position.y + np.random.normal(scale=self.INIT_SPREAD_XY_M, ...)
self.particles[:,2] = quaternion_to_angle(pose.orientation) + np.random.normal(scale=self.INIT_SPREAD_THETA_RAD, ...)
```

Note the signature: it takes `pose`, not the `PoseWithCovarianceStamped`, so
**the 36-element covariance never reaches it**. Phase 4 Task 1 worked around
this by deriving the two scalars outside the node — `sqrt(max(cov_xx, cov_yy))`
and `sqrt(cov_yaw)` — and passing them as parameters. That is strictly better
than the previous hardcoded 0.5 m / 0.4 rad, but it still throws away:

- **anisotropy** — GNSS error is an ellipse, not a circle; `max(xx, yy)` inflates the narrow axis
- **correlation** — the `xy` and `x–yaw` off-diagonals, which orient that ellipse
- **per-run variation** — the spread is frozen at launch instead of tracking the seed actually received

## 2. What the GNSS covariance is actually worth

Measured from `autoware_gnss_poser` on the sample bag:

| term | value | implied sigma |
|---|---|---|
| `cov_xx` | 3.909 m² | 1.98 m |
| `cov_yy` | 3.909 m² | 1.98 m |
| `cov_yaw` | 1.000 rad² | 1.00 rad (57°) |

Two things stand out. `cov_xx == cov_yy` exactly, so the position covariance is
isotropic — for *this* source, `max(xx, yy)` loses nothing, though that is a
property of the input, not of the method. And `cov_yaw` is **exactly 1.0**,
which is a placeholder rather than an estimate.

That matters, because the measured yaw seed error was **1.81 rad (104°)** —
about 1.8x the reported sigma. The heading is not merely uncertain, it is
wrong, and the covariance does not say so.

The cause is visible in the shipped config
(`/opt/autoware/1.5.0/share/autoware_gnss_poser/config/gnss_poser.param.yaml`):

```yaml
use_gnss_ins_orientation: true
```

`gnss_poser` expects heading from a GNSS/INS unit. The Autoware sample bag has
ublox `NavPVT` but no INS orientation stream, so the heading falls back to
course-over-ground — meaningless at the near-standstill start of the run — with
a constant covariance attached.

**Conclusion: honouring the covariance properly is correct and worth doing for
position, but it cannot fix heading, because the heading covariance carries no
information.** Any design that leans on GNSS yaw at low speed is fragile
regardless of how carefully the covariance is sampled.

## 3. How Autoware solves the heading problem

`pose_initializer.launch.xml:15` is the clue:

```xml
<remap from="ndt_align" to="/localization/pose_estimator/ndt_align_srv"/>
```

Autoware **does not trust the GNSS pose directly**. GNSS supplies a coarse
prior; `pose_initializer` then calls NDT's align service, which registers the
current scan against the map over a search range — resolving exactly the
heading ambiguity that GNSS cannot. Only the refined result initialises the
filter.

We already have the equivalent machinery, built for a different purpose:
`scripts/2dlidar/score_sensor_model.py` evaluates the (now-fixed) sensor model
in log space over a pose grid and returns the argmax. Phase 3e measured that
argmax landing **0.05–0.22 m from ground truth**. A "2D align" that searches
(x, y, yaw) around the GNSS seed and initialises particles at the winner is
therefore not speculative work — it is existing, measured code moved into the
initialization path.

## 4. Recommendations, in dependency order

1. **Gate publishing on initialization** (`require_initialpose`, default off for
   reproducibility). Skip `initialize_global()`, suppress MCL updates and pose
   publication until a seed arrives. Removes the pre-seed garbage segment and
   matches Autoware's state-machine semantics. Cheap, contained, no measurement
   risk.
2. **Sample the seed covariance properly.** Pass the full
   `PoseWithCovarianceStamped` into `initialize_particles_pose`, extract the
   (x, y, yaw) marginal — covariance indices `[0,1,5]`, i.e. `cov[0]`, `cov[1]`,
   `cov[5]`, `cov[6]`, `cov[7]`, `cov[11]`, `cov[30]`, `cov[31]`, `cov[35]` in
   the row-major 6x6 — and draw from a multivariate normal so anisotropy and
   correlation survive. Fall back to the scalar parameters when the covariance
   is all zeros, which some publishers emit. Subsumes today's behaviour as the
   isotropic special case.
   Note for comparison: nav2's AMCL also consumes only the diagonal, so doing
   this properly puts us slightly ahead of the reference implementation rather
   than merely level with it.
3. **Add a 2D align step for heading** (the real fix for §2). Search
   (x, y, yaw) around the seed using the existing log-space field evaluation,
   initialise at the argmax, and fall back to the raw seed if the field is flat
   (a legitimate outcome in a symmetric corridor — better to report low
   confidence than to invent a heading). This is what makes GNSS initialization
   trustworthy rather than lucky.
4. **Do not** attempt to consume the full 6x6 covariance downstream of
   initialization. Roll, pitch and z are meaningless for a planar filter; the
   (x, y, yaw) marginal is the whole of the useful information.

## 5. Why the current numbers still stand

Phase 4's 5-seed matrix passed the accuracy gate (median 0.77 m, 5/5 seeds)
*with* the 104° yaw seed error, because the covariance-derived 1.0 rad spread
was wide enough for scan matching to recover heading within a second. That is a
real result, but it is luck-adjacent: the same seed error in a corridor with
two-fold symmetry would plausibly latch onto the reversed heading and stay
there. Recommendations 1 and 3 turn that from luck into design.
