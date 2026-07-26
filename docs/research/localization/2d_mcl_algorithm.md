# How 2D Monte-Carlo Localization Works in AutoSDV

Reference for the vendored `particle_filter` (Roboracer/F1TENTH lineage,
`src/localization/external/particle_filter`, fork `NEWSLabNTU/particle_filter@autosdv`)
and its `range_libc` raycaster. Written while diagnosing the Phase 3 divergence;
all numbers are measured from this repository's runs, not from the papers.

Companion documents:

- `docs/reports/2dlidar-phase3b-*.md`, `docs/reports/2dlidar-phase3c-*.md` — the runs
- `docs/design/f1tenth-2dlidar-integration.typ` — why 2D LiDAR at all
- `docs/superpowers/plans/2026-07-26-2dlidar-phase-3d-instrumentation.md` — the
  instrumentation built to observe what this document describes

---

## 1. The data

### 1.1 The scan — `sensor_msgs/LaserScan`

A 2D scan is not a point list. It is a fixed angular sweep in which position is
implied by array index:

| field | our value | meaning |
|---|---|---|
| `angle_min` / `angle_max` | −π / +π | angle of first / last beam, sensor frame |
| `angle_increment` | 0.0043 rad | → **1461 beams** over the full circle |
| `range_min` / `range_max` | 0.1 / 60.0 m | validity window |
| `ranges[1461]` | float32, metres | distance to first obstacle per beam; `inf` = no return |

Beam *k* points along `angle_min + k · angle_increment`; `ranges[k]` is its measured
distance. The angle array is never transmitted, only reconstructed.

In AutoSDV this message is **synthetic**. `pointcloud_to_laserscan` takes the
Velodyne `PointCloud2`, keeps points inside a horizontal slab
(`min_height`/`max_height`, 1.916–2.216 m in `base_link` for the sample sensor kit),
computes each survivor's azimuth `atan2(y, x)`, maps it to a bin, and keeps the
closest range per bin. Bins with no point become `inf`.

Measured on one sample-site frame: 209,772 cloud points → 9,649 in the slab →
1,385 of 1,461 bins filled (94.8%), median range 19.7 m, p90 49.3 m, 30% of returns
beyond 30 m.

The filter then decimates (`lidarCB`, `particle_filter.py:395`): `ranges[::18]`
(`angle_step: 18`) → **82 beams**, with matching angles computed once on the first
scan.

### 1.2 The map — `nav_msgs/OccupancyGrid` → `PyOMap`

| field | meaning |
|---|---|
| `info.resolution` | metres per cell (0.05 or 0.1 in our runs) |
| `info.width` / `height` | cells (e.g. 4180 × 4895) |
| `info.origin` | pose of cell (0,0), the lower-left corner, in the map frame |
| `data[width*height]` | `int8`, row-major: 0 free, 100 occupied, −1 unknown |

`map_server` builds this from our `.pgm`/`.yaml`: dark → 100, light → 0, mid-grey → −1.

`PyOMap` (`RangeLibc.pyx:154`) then converts it for raycasting:

```python
if array_255[x, y] > 10:
    self.thisptr.grid[x][y] = True
```

Two consequences worth remembering:

1. The grid becomes **binary** — occupancy probabilities do not survive.
2. **Unknown (−1) fails the test and is therefore treated as free space.**

`PyOMap` also caches `world_scale = resolution`, `world_origin_x/y` and
`world_angle`, so the C++ side converts metric poses to cell indices itself.

### 1.3 The belief — particles

```
self.particles : float64 (4000, 3)   # (x, y, theta), map metric frame
self.weights   : float64 (4000,)     # normalised, sums to 1
```

That is the entire state estimate: 4,000 pose hypotheses with confidences.

### 1.4 The action — odometry delta

`odomCB` (`particle_filter.py:410`) never uses absolute odometry, only the
increment between consecutive messages, rotated into the *previous* pose's frame:

```python
rot    = rotation_matrix(-theta_prev)
delta  = position - last_position
action = (local_dx, local_dy, theta - theta_prev)
```

So absolute odometry drift is irrelevant; only per-step increments matter.
**`odomCB` also calls `update()`**, meaning the filter ticks at odometry rate
(20 Hz in our runs), not scan rate (10 Hz). See §5.3.

---

## 2. The theory

### 2.1 Bayes filter

We want the belief over pose given all evidence:

    bel(x_t) = p(x_t | z_1..t, u_1..t, m)

with state `x = (x, y, theta)`, scans `z`, odometry `u`, map `m`. It factors into a
two-step recursion:

    prediction:  bel_bar(x_t) = ∫ p(x_t | x_t-1, u_t) · bel(x_t-1) dx_t-1
    correction:  bel(x_t)     = eta · p(z_t | x_t, m) · bel_bar(x_t)

Everything else is a choice of how to represent `bel` and how to evaluate those two
probabilities.

### 2.2 Why particles instead of a Gaussian

Along a corridor or a repetitive street, many poses explain one scan equally well —
the posterior is genuinely multi-modal, so a mean and covariance cannot hold it.
MCL represents `bel` as N weighted samples and evaluates the prediction integral by
**importance sampling**: draw from the previous belief, push each sample through the
motion model (the proposal distribution), weight by the measurement likelihood. It
converges to the true posterior as N → ∞; at N = 4000 it converges to something
less nice.

### 2.3 Measurement model (beam model)

For a candidate pose, raycast each beam in the map to obtain the **predicted** range
`d_k` — where the wall should be if the vehicle were there — and compare with the
**observed** `r_k`. Assuming conditional independence across beams:

    p(z | x, m) = Π_k p(r_k | d_k)

Each beam's likelihood is a four-component mixture (Thrun et al.,
*Probabilistic Robotics*, ch. 6), because range finders fail in four distinct ways:

| component | configured weight | physical meaning |
|---|---|---|
| `p_hit` | `z_hit = 0.75` | correct reading with noise, `N(r; d, sigma^2)`, `sigma_hit = 8.0` |
| `p_short` | `z_short = 0.01` | unmapped obstacle closer than the wall |
| `p_max` | `z_max = 0.07` | no return — spike at max range |
| `p_rand` | `z_rand = 0.12` | uniform garbage |

The textbook requires each component to be a **normalised density** before mixing,
with the four weights summing to 1. Section 5.1 shows what happens here because one
of them is not.

### 2.4 Resampling, degeneracy, squash

After weighting, particles are resampled with replacement in proportion to weight.
Skip it and all weight concentrates on one particle while the rest waste compute
(*degeneracy*); do it too eagerly and every particle descends from one ancestor, so
the filter cannot recover (*sample impoverishment*). Diversity is measured by

    N_eff = 1 / Σ w_i²

The `squash_factor` applies `w ← w^(1/2.2)`. This is not part of the derivation; it
is a temperature that flattens overconfident likelihoods. Its presence is a hint
that the model produces sharper likelihoods than the evidence supports.

---

## 3. The algorithm as implemented

### 3.1 Once, at startup

**`get_omap()`** (`particle_filter.py:196`) — fetch the grid over the `GetMap`
service, build `PyOMap`, construct the raycaster. We use **CDDT** (Compressed
Directional Distance Transform): for each of `theta_discretization = 112` discrete
headings it precomputes per-column obstacle positions so a raycast is a lookup plus
interpolation rather than a walk along the ray. That is what makes hundreds of
thousands of raycasts per update affordable on CPU.

**`precompute_sensor_model()`** (`particle_filter.py:210`) — build the mixture as a
table so no exponentials are evaluated at runtime:

```python
table_width = MAX_RANGE_PX + 1               # 60 m / 0.05 m = 1200 → 1201
for d in range(table_width):                 # predicted range, in PIXELS
    for r in range(table_width):             # observed range, in PIXELS
        prob  = z_hit * exp(-(r-d)**2 / (2*sigma**2)) / (sigma*sqrt(2*pi))
        if r < d:             prob += 2 * z_short * (d - r) / d
        if r == MAX_RANGE_PX: prob += z_max
        if r <  MAX_RANGE_PX: prob += z_rand / MAX_RANGE_PX
        table[r, d] = prob
    table[:, d] /= table[:, d].sum()          # normalise each column
```

The table is indexed in **map pixels**, not metres, so `sigma_hit = 8.0` means
0.4 m at 0.05 m/cell and 0.8 m at 0.1 m/cell — **map resolution silently retunes the
sensor model**. The 1201² pure-Python double loop is why PF startup takes ~48 s at
0.05 m / 60 m. The finished table is uploaded to C++ via `set_sensor_model`.

### 3.2 Every update (`MCL`, `particle_filter.py:470`)

Triggered by each odometry message. Inputs: `o` = 82 observed ranges in metres
(possibly containing `inf`), `a = (dx, dy, dtheta)`.

**Step 1 — resample**

```python
idx      = np.random.choice(self.particle_indices, 4000, p=self.weights)
proposal = self.particles[idx, :]
```

**Step 2 — motion model** (`particle_filter.py:434`)

```python
proposal[:,0] += cos(theta)*dx - sin(theta)*dy
proposal[:,1] += sin(theta)*dx + cos(theta)*dy
proposal[:,2] += dtheta
proposal[:,0] += N(0, 0.05)     # motion_dispersion_x
proposal[:,1] += N(0, 0.025)    # motion_dispersion_y
proposal[:,2] += N(0, 0.25)     # motion_dispersion_theta  (~14 deg per update)
```

The noise is **fixed**: it does not scale with distance travelled, so standing still
injects as much uncertainty as moving half a metre.

**Step 3 — raycast**

```python
range_method.calc_range_repeat_angles(queries, downsampled_angles, ranges)
# (4000,3) poses x (82,) angles -> (328000,) predicted ranges, metres
```

328,000 raycasts per update; at ~14 Hz that is ≈4.6 M rays/s.

**Step 4 — evaluate** (`range_libc/includes/RangeLib.h:533`, the real inner loop)

```cpp
r = clamp(obs[j]           / world_scale, 0, table_width-1);   // metres -> pixels
d = clamp(ranges[i*82 + j] / world_scale, 0, table_width-1);
weight *= sensor_model[(int)r][(int)d];
```

An 82-term product per particle. `inf / world_scale` stays `inf`, so **every
no-return beam is clamped into the max-range bucket**.

**Step 5 — squash and normalise**

```python
weights **= 1/2.2          # 0.4545
weights /= weights.sum()
```

**Step 6 — report** (`expected_pose`) — weighted mean of x and y, circular mean of
theta; published on `/pf/viz/inferred_pose` and broadcast as TF `map → laser`.

---

## 4. One beam, real numbers

Resolution 0.05 m, max range 60 m, predicted `d` = 20 m (400 px). Values are from
the actual normalised table column:

| observation | index | `P(r|d)` | comment |
|---|---|---|---|
| perfect match, 20.00 m | 400 | 0.007576 | |
| off by 0.4 m (1 sigma) | 407 | 0.005173 | graceful |
| off by 2 m | 440 | 0.000020 | 380x penalty — very sharp |
| short read, 12 m | 240 | 0.001636 | 4.6x penalty |
| **no return (`inf`)** | 1200 | **0.014141** | **1.87x better than a perfect match** |

---

## 5. Structural defects this exposes

**Update (Phase 3e):** three of these five defects — 5.1, 5.2, 5.3 — are
now **FIXED**, measured, and shipped in the `NEWSLabNTU/particle_filter@autosdv`
fork (opt-in via `sensor_model_variant=normalized_short`,
`skip_nonfinite_beams=true`, `update_on_new_scan_only=true`). With all
three enabled, the offline GT-to-argmax gap collapses from 51.9 to 6.2
nats (median, 5 frozen scans) and the end-to-end Phase 3 accuracy gate now
passes 5/5 replay seeds (0.79 m mean translational error, down from 0/5
passing and a 26.0 m median, 15.9–36.0 m across seeds, upstream). See
`docs/reports/2dlidar-phase3e-model-fixes.md` for the full measurement.
5.4 and 5.5 remain open — they are inherent to the beam model's
independence/clamping assumptions, not implementation bugs, and are out of
scope for Phase 3e.

### 5.1 `z_short` is unnormalised and swallows the model — **FIXED (Phase 3e)**

The ramp `2·z_short·(d−r)/d` integrates to `z_short·d` — mass that grows linearly
with predicted range **in pixels**. Because normalisation happens afterwards per
column, the effective mixture weights bear little relation to the configured ones:

| predicted range | effective `z_hit` | effective `z_short` |
|---|---|---|
| 10 m | 25.4 % | 68.1 % |
| 20 m | 15.2 % | 81.0 % |
| 50 m | 6.8 % | 91.4 % |

Configured `z_hit` is 0.75; for a beam predicting 50 m the Gaussian that does the
actual matching retains **0.068** of the probability mass. Since `d` is in pixels,
halving the resolution doubles the ramp's mass — so a "finer map" simultaneously
halves the matching width and further suppresses the hit term. Those two effects
were confounded in every Phase 3c lever comparison.

Canonical fix: normalise `p_short` per column before mixing (textbook uses
`eta · lambda_short · exp(−lambda_short · r)` with `eta = 1/(1−exp(−lambda_short·d))`),
so the configured weights mean what they say.

**Fixed (Phase 3e Task 2):** implemented exactly as above —
`sensor_model_variant=normalized_short` (`lambda_short=1.0`, 1/px) in the
fork's `particle_filter/sensor_model.py`. Effective `z_hit` is now flat
across range instead of range-dependent:

| predicted range | effective `z_hit`, before | effective `z_hit`, after |
|---|---|---|
| 10 m | 25.4 % | 78.5 % |
| 20 m | 15.2 % | 78.5 % |
| 50 m | 6.8 % | 78.5 % |

(78.5% vs. the configured 75% is a ~3.5-point discrete-grid Riemann
overcounting artifact — the table is pixel-indexed while `eta` normalises
a continuous integral — not a bug; see the Phase 3e report.) Offline gate,
5 frozen scans, sample site: GT-to-argmax gap 51.92 → 7.07 nats (median),
argmax distance from GT 29.67 → 0.20 m (median). Measurement:
`docs/reports/2dlidar-phase3e-model-fixes.md`.

### 5.2 A beam that sees nothing outscores a beam that matches — **FIXED (Phase 3e)**

0.014141 versus 0.007576 (§4). Worse, with `inf` observed, a pose predicting 20 m
scores 1.66x one predicting max range — the filter is rewarded for hypotheses that
place walls where the scan reports nothing. With ~5% of bins empty by construction
this is a small constant bias; where the map is thin it is a large one.

Canonical fix: drop non-finite beams from the product entirely, or give `p_max` mass
only when the *predicted* range is also at max.

**Fixed (Phase 3e Task 3):** implemented as an exact drop —
`skip_nonfinite_beams=true` removes non-finite observed beams (and their
predicted-range column) from the `eval_sensor_model` call entirely, for
every particle, every update; the persistent ray-cast buffers are
untouched, so only the ~15 ms evaluation stage is affected. ~30% of beams
are non-finite in this dataset (measured 56–59 of 82, 68–72% finite,
across 5 timestamps) — not an edge case, routine. Stacked on top of 5.1's
fix, offline gate: GT-to-argmax gap 7.07 → 6.24 nats (median), local-max
count (GT sitting at an actual peak of the likelihood surface) 1/5 → 2/5
timestamps. Measurement: `docs/reports/2dlidar-phase3e-model-fixes.md`.

### 5.3 Each scan is used twice — **FIXED (Phase 3e)**

`update()` fires from `odomCB` at 20 Hz while scans arrive at 10 Hz, and
`observation` is simply whatever `self.downsampled_ranges` currently holds.
Consecutive Bayes updates therefore multiply in the same measurement, roughly
squaring its likelihood — textbook overconfidence, and plausibly what the
`squash_factor` is compensating for.

Canonical fix: run the correction step only on new scans (track the scan stamp), or
run `update()` from `lidarCB` and accumulate odometry between scans.

**Fixed (Phase 3e Task 4):** `update_on_new_scan_only=true` gates the
correction step on the scan stamp; odometry deltas between corrections are
composed via exact rotation composition (`compose_odometry_delta`, not a
small-angle approximation — proven exact by induction and unit-tested to
1e-9). Measured: 237 corrections over 237 unique scan stamps (zero
double-counting), correction rate 8.3 Hz (sim time) vs. the previous
~15 Hz odom-rate baseline. **Consequence:** since `publish_tf`/
`visualize` live inside `update()` and no predict-only publish path was
added, the pose/TF publish rate halves along with it — documented, not a
data-quality regression (see the Phase 3e report). This is the fix that,
combined with 5.1/5.2, took the end-to-end gate from 0/5 to 5/5 passing
seeds.

### 5.4 Beam independence fails hardest where we fail — still open

82 beams along a corridor mostly strike the same two walls, so their errors are
strongly correlated, yet the product treats them as 82 independent votes. The result
is a likelihood ridge along the corridor far sharper than the information supports —
exactly the geometry where both our PF and nav2 AMCL collapsed
(`docs/reports/2dlidar-phase3c-lever4-amcl.md`).

Mitigations are approximate by nature: further beam decimation, likelihood
tempering (what squash does), or an explicit correlated-noise model.

### 5.5 Clamping discards the disambiguating beams — still open

30% of returns exceed 30 m; everything past `max_range` lands in the pose-independent
max-range bucket. The long down-corridor beams that could fix longitudinal position
are the ones thrown away. Raising `max_range` costs table size `O((range/res)²)` and
startup time.

---

## 6. Where to observe each of these

| Quantity | Source | Reveals |
|---|---|---|
| `/pf/viz/fake_scan` (already published) | `publish_scan`, gated on `viz: 1` | per-beam residuals at the inferred pose |
| `N_eff = 1/Σw²` | `effective_sample_size()` (added in Lever 3) | collapse timing, diversity |
| Likelihood field over candidate poses | `calc_range_repeat_angles` on a synthetic pose grid | ridge vs peak — §5.4 directly |
| Beam category fractions (hit / short / clamped) | observed vs predicted ranges | how much mass is pose-independent — §5.2, §5.5 |
| Sensor-model columns `table[:, d]` | `self.sensor_model_table` | effective mixture weights — §5.1 |
| Particle cloud spread | `/pf/viz/particles` | ridge shape, jump moments |

Phase 3d builds this instrumentation; see
`docs/superpowers/plans/2026-07-26-2dlidar-phase-3d-instrumentation.md`.
