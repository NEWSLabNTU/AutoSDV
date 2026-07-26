# Phase 4 Task 2: initialization gating and covariance sampling

Implements recommendations 1 and 2 of
[`mcl_initialization_and_covariance.md`](../research/localization/mcl_initialization_and_covariance.md).
Recommendation 3 (a 2D align step for heading) is deliberately not attempted
here.

Fork commit `373af5a` (`NEWSLabNTU/particle_filter@autosdv`).

---

## 1. Gate publishing on initialization

New parameter `require_initialpose` (default `False`). When enabled, the
constructor skips `initialize_global()` and the node performs no MCL update and
publishes no pose or TF until an `/initialpose` message has been received.

Measured on the sample site, seed 1, GNSS-initialised both times:

| | GNSS init, no gate | GNSS init + gate |
|---|---|---|
| poses published | 231 | 216 |
| first published pose, distance from GT | **11.47 m** | **0.85 m** |
| poses > 5 m from GT | 7 | **0** |
| worst pose in the run | 14.59 m | **0.85 m** |

The pre-seed excursion is gone. Previously poses #0–#11 were published before
the seed arrived and the track began roughly 55 m from the vehicle (visible as a
long straight run-in on
`assets/2dlidar-phase4-gnssinit-overlay.png`); the gated run's very first pose
is already at the seed, and that first pose is also the worst pose of the entire
run. Compare `assets/2dlidar-phase4-initgate-overlay.png`, where the 2D-MCL
track begins at the start square and the 25 s–55 s markers pair with ground
truth throughout.

## 2. Sample the seed covariance

`initialize_particles_pose` now receives the covariance rather than a bare
`Pose`. Two pure functions carry the logic:

- `build_pose_covariance_marginal(covariance)` — reshapes the 36-element
  row-major 6×6 to the planar `(x, y, yaw)` marginal via indices `[0, 1, 5]`,
  then symmetrises (`0.5 * (M + Mᵀ)`), because publishers round the two
  triangles inconsistently and `numpy.random.multivariate_normal` warns on
  asymmetric input.
- `sample_pose_particles(...)` — draws from the multivariate normal so
  anisotropy and correlation survive, falling back to independent scalar
  sampling from `init_spread_xy_m` / `init_spread_theta_rad` when the covariance
  is absent, all-zero, or not positive semi-definite. The chosen path is logged.

Behaviour is unchanged when no covariance is supplied, which is what keeps the
oracle path and every earlier run reproducible.

`gnss_init_seed.py` and `run-particle-filter.sh` now populate the real
`gnss_poser` covariance into the published `/initialpose` message, so the node
consumes it directly instead of only receiving two derived scalars. The derived
scalars remain as the documented fallback.

Covariance received from `gnss_poser` on this bag: `cov_xx = cov_yy = 3.909 m²`
(σ ≈ 1.98 m), `cov_yaw = 1.000 rad²`. It is diagonal, so the multivariate draw
and the previous scalar approximation coincide numerically **for this source** —
the value of the change is that an anisotropic or correlated seed is now handled
correctly rather than silently flattened, not a change in today's numbers.

## 3. Accuracy is unaffected

Five seeds, GNSS-initialised, both new flags enabled
(`data/rosbags/phase3/seedmatrix_initgate/results.jsonl`):

| metric | median | range | limit | |
|---|---|---|---|---|
| mean translational | 0.826 m | 0.797–0.858 | < 1.0 | PASS |
| p95 translational | 1.999 m | 1.932–2.055 | < 2.5 | PASS |
| mean \|yaw\| | 0.0270 rad | 0.0265–0.0285 | < 0.2 | PASS |

**5/5 seeds pass.** Against the ungated GNSS runs (median 0.772 m / 2.015 m /
0.0266 rad), p95 improves slightly and mean rises slightly; both shifts sit
inside the seed spread, so the honest reading is that gating costs no accuracy
while removing the pathological startup behaviour. Pair counts fall from
224–229 to 198–225 because the filter no longer publishes during the pre-seed
window.

Fork tests: 54 passed.

## 4. What remains open

- **Heading still depends on luck, not design.** The GNSS yaw seed is 1.81 rad
  (104°) wrong and `gnss_poser`'s `cov_yaw` of exactly 1.0 rad² is a placeholder,
  not an estimate — see the study's §2. Recovery here relies on the wide spread
  plus scan matching. Recommendation 3 (search `(x, y, yaw)` around the seed
  with the existing log-space field evaluation and initialise at the argmax) is
  the real fix and is the next task.
- **Single site, single map, single bag**, as with every result in this series.
- The full 6×6 covariance is still not consumed beyond the planar marginal,
  which is deliberate: roll, pitch and z carry no meaning for a planar filter.

## 5. Reproduce

```bash
# Gated, GNSS-initialised single run
BAG=data/rosbags/phase3/sample_ndt_gt GT_BAG=data/rosbags/phase3/sample_ndt_gt \
POINTCLOUD_TOPIC=/sensing/lidar/top/pointcloud_raw_ex \
VELOCITY_TOPIC=/vehicle/status/velocity_status IMU_TOPIC=/sensing/imu/tamagawa/imu_raw \
IMU_YAW_SIGN=-1.0 \
MAP_YAML=data/sample-rosbag-replay/sample-map-rosbag/occupancy_grid_scanaccum_mh1r05.yaml \
SCAN_MIN_HEIGHT=1.91611 SCAN_MAX_HEIGHT=2.21611 SCAN_RANGE_MAX=60.0 \
PF_MAX_RANGE=60.0 PF_SQUASH=3.0 PF_DISP_THETA=0.1 \
PF_SENSOR_MODEL_VARIANT=normalized_short PF_SKIP_NONFINITE=true \
PF_UPDATE_ON_SCAN_ONLY=true INITPOSE_SOURCE=gnss PF_REQUIRE_INITIALPOSE=true \
    ./scripts/2dlidar/run-particle-filter.sh

# Figures (shared clock across all three)
bash -c 'source /opt/autoware/1.5.0/setup.bash && \
    python3 scripts/2dlidar/plot_trajectories.py --prefix 2dlidar-phase4-initgate \
        --mcl-bag data/rosbags/phase3/seedmatrix_initgate/fixed_s1'
```
