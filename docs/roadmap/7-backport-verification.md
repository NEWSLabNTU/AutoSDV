# Verifying the backport

**Goal**: turn `docs/roadmap/6-golfcart-backport.md`'s ten phases from "written
and compiled" into "seen working". Nothing in that campaign was launched.

**Status**: proposed

---

## What is already verified, and what that is worth

Worth distinguishing, because the campaign's commits say both:

| Verified | How |
|----------|-----|
| `cuda_pointcloud_filters` kernels | 12 unit tests against real device memory, passing on this desktop's GPU |
| The CUDA preprocessor accepts our new point layout | a synthetic `PointXYZIRCAEDT` cloud, processed at the full 10 Hz input rate |
| The layout itself is right | **a recorded Velodyne cloud has exactly it** — see below |
| Launch-logic branches | `launch_setup` driven with a stub context: cpu, cuda, and each refusal |
| The mode strip's rendering | 29 checks against a graph fixture captured from a live aggregator |
| The setup registry | `--list`, `--status`, `--run --dry-run` for two profiles |
| Every systemd unit | `systemd-analyze verify` on the rendered units |

None of that is a running stack. Everything below is.

## The bags change what needs a vehicle

`/home/aeon/nas/autoveh/dataset/2025-11-14 AutoSDV Localization RosBags in COSS`
holds four recordings, 9.3 GB:

| Bag | Duration | LiDAR topic | Layout |
|-----|----------|-------------|--------|
| `vlp32_1`, `vlp32_2` | 28.5 s each | `/sensing/lidar/velodyne_points`, 42,711 pts/frame | **`PointXYZIRCAEDT`, `point_step` 32** |
| `robin_1` | 153 s | `/sensing/lidar/iv_points`, 85,210 pts/frame | `PointXYZIRC`, `point_step` 16 |
| `robin_2` | 74.6 s | same | same |

All four carry `/sensing/imu/imu_data`, `/vehicle/status/*`, GNSS and
`/sensing/lidar/concatenated/pointcloud`, so they can drive
`just sim logging` without any sensor attached.

Two consequences that decide the order of everything below.

**The Velodyne bags already contain the layout this campaign taught the Seyond
driver to publish.** Read from `vlp32_1`, field for field: x0 y4 z8,
intensity12, return_type13, channel14, azimuth16, elevation20, distance24,
time_stamp28. That is what `point_xyzircaedt.h` declares. The layout is
therefore confirmed against a real recording, not only against the header file
it was copied from — and the whole CUDA sensing chain can be exercised on this
desktop, today, with no vehicle.

**The Robin-W bags cannot validate the new driver.** They were recorded with the
old one: 16 bytes per point, no `time_stamp` field. The per-point time cannot be
reconstructed from them — the information is not there. Anything about deskewing
the Robin-W needs a **new** bag off the rebuilt driver, and a *moving* one,
because deskew is invisible while stationary.

Note the bandwidth this makes concrete: 85,210 points per frame at 16 bytes is
1.36 MB; at 32 it is 2.73 MB, at 10 Hz. That is the cost of the new layout on
the sensor this vehicle actually runs, and it lands on the same deliver thread
the QoS fix unblocked.

---

## Phase A — Build, on this machine

Nothing else can start until this passes, and it has never been done since the
campaign began: this workspace has no `install/`.

```bash
just build 2>&1 | tee tmp/build.log
just test
```

Watch for, specifically:

1. `cuda_ndt_matcher` at its new pin (22 commits, never compiled here).
2. `seyond` at the rebased pin, default `POINT_TYPE=PointXYZIRCAEDT`.
3. `cuda_pointcloud_filters` as a submodule rather than an in-tree package.
4. `autosdv_system_monitor` with its new template and `rosbridge_server`
   dependency — `rosdep install` must resolve that, or the launch fails at run
   time rather than build time.

**Pass**: a clean build and `colcon test` with no new failures.

## Phase B — Launch resolution, no nodes started

`play_launch` resolves the whole tree without running anything, which is the
cheapest way to catch a launch file that parses but cannot resolve.

```bash
play_launch dump launch autosdv_launch autosdv.launch.yaml -o tmp/rust.json
play_launch dump launch autosdv_launch autosdv.launch.yaml --parser python -o tmp/py.json
play_launch context tmp/rust.json --tree | head -50
```

Three things this settles at once:

1. **Phase 5's parser parity**, the one item that phase left open. Compare node
   counts between the two dumps, as the golf cart did for its five entry points.
2. **The new arguments resolve**: `pointcloud_backend`,
   `localization_pointcloud_backend`, `ndt_param_file`, `use_gnss` →
   `gnss_enabled`.
3. **`gnss_enabled` actually reaches `pose_twist_estimator`** — the bug fixed in
   phase 3 was precisely that it did not, and a dump shows the resolved
   parameter:

```bash
play_launch context tmp/rust.json --node /localization/pose_initializer | grep -i gnss
play_launch dump launch autosdv_launch autosdv.launch.yaml use_gnss:=false -o tmp/nognss.json
```

**Pass**: identical node sets under both parsers; `gnss_enabled` false in the
second dump and true in the first.

## Phase C — The CUDA sensing chain, on a Velodyne bag

This is the check the bags make possible without hardware.

```bash
# terminal 1
just sim logging ARGS="pointcloud_backend:=cpu"
# terminal 2
ros2 bag play "/home/aeon/nas/.../vlp32_1" --clock
ros2 topic hz /sensing/lidar/concatenated/pointcloud
```

then the same with `pointcloud_backend:=cuda`, and with
`localization_pointcloud_backend:=cuda` added.

Measure, per arm, with `scripts/profiling/`:

```bash
scripts/profiling/jetson_gpu_sampler.py -o tmp/gpu-cuda.csv     # on the Orin
scripts/profiling/kernel_cpu_report.sh                          # either host
```

**Pass**:

- `concatenated/pointcloud` holds the bag's own rate (about 10 Hz) in **both**
  arms. A lower rate in the `cuda` arm is the finding, not a nuisance: the golf
  cart's concatenator lost 55% of its frames to a `timeout_sec` shorter than it
  needed, and AutoSDV's chain ends in a passthrough precisely to avoid that.
- The cloud stays in `base_link` in both arms. The CUDA preprocessor does not
  transform, so if the passthrough were bypassed this would silently become the
  sensor frame and everything downstream would be wrong in a way no error
  reports.
- CPU falls and GPU rises in the `cuda` arm. The golf cart measured −23.7 points
  of container CPU on its Orin; ours ends differently, so the number will
  differ. Record what it is rather than expecting theirs.

## Phase D — Localization on the same bags

```bash
just sim logging ARGS="pose_source:=cuda_ndt"
python3 scripts/testing/localization/check_ndt_activated.py --timeout 30
python3 scripts/testing/localization/ndt_quality_report.py --seconds 60
python3 scripts/testing/localization/ndt_alignment_report.py --seconds 60
```

The COSS map is `data/COSS-map-planning`, which is what these bags were recorded
against, and what `ndt_alignment_report.py` now defaults to.

**Pass**: NDT activates; scan-to-map p50 within roughly one `ndt.resolution`
voxel; `localization_pointcloud_backend:=cuda` gives the same poses as `:=cpu`
within noise. The second is the real point — that switch has only ever been
checked for *correctness of wiring*, never for producing the same answer.

Then the one measurement phase 2 explicitly left undone: time the localization
chain in both arms. The CPU chain is about 19% of a core; the CUDA one has never
been timed and could be slower.

## Phase E — The Robin-W, which needs the vehicle

Everything above runs on a desktop. This does not.

1. **Rebuild and record.** With the rebased driver, record a short bag with the
   Robin-W **moving** — stationary cannot show deskew.
2. **Layout**: `point_step` 32 and ten fields at the offsets in phase C's table.
3. **The QoS fix**: the driver's own deliver-queue counters over a run of a few
   minutes, `added` against `dropped`. The golf cart saw 60% dropped before and
   the fix removes the cause; this is the confirmation.
4. **Deskew**: `pointcloud_backend:=cuda lidar_model:=robin-w` on that bag, and
   look at structure that the vehicle drove past — a wall, a pole — before and
   after. If the time offsets have the wrong origin, the cloud is plausible and
   sheared, which is why this needs eyes on real geometry rather than a rate
   check.
5. **Bandwidth**: 2.73 MB per frame at 10 Hz on this sensor. Watch the deliver
   thread and the concatenator, not just the topic rate.

## Phase F — The diagnostic graph, on the board

Settles the open entries in `docs/known-config-defects.md`.

```bash
just launch            # then open the monitor at :8080
```

1. **The mode availability strip** against a real `autoware_diagnostic_graph_aggregator`.
   It has only been driven from a fixture. Watch specifically for the
   index-join: chips labelled with the wrong mode look entirely plausible.
2. **How much of the graph is red.** The golf cart measured 31.1% of reports
   ERROR or STALE, nearly all configuration. Record ours.
3. **`system_monitor`**: confirm the four monitors that cannot pass on a Jetson,
   then decide whether to copy the golf cart's fix (name the interfaces, do not
   launch the three a parameter cannot fix).
4. **`topic_state_monitor_initialpose3d`**: whether it is ERROR on 100% of
   reports here too. If so, that is a report to make upstream — the all-zero
   thresholds are Autoware's own value, byte for byte.

## Phase G — The golf cart

The other vehicle now shares the driver, and one decision is waiting on a
measurement rather than on an edit: whether its Falcon moves from
`PASSTHROUGH_LIDARS` into `PREPROCESSED_LIDARS`. That changes what
`concatenate_and_time_sync_node.param.yaml` lists, and that concatenator is
known to be timing-sensitive — a silent second LiDAR halved its rate through
`timeout_sec` alone, and the timeout has since been brought under the scan
period. Do it against a bag with both sensors live, and compare the rate before
and after.

---

## Order, and what each phase costs

| Phase | Needs | Rough cost |
|-------|-------|-----------|
| A — build | this machine | one build |
| B — launch resolution | a built workspace | minutes |
| C — CUDA sensing | a built workspace + `vlp32_*` | an hour |
| D — localization | same | an hour |
| E — Robin-W | **the vehicle**, a new bag | a session |
| F — diagnostic graph | **the board** | a session |
| G — golf cart Falcon | **that vehicle**, a two-LiDAR bag | a session |

A through D need no hardware at all, which was not true before the bags turned
up. They should be done first, and in order: each one's failure would make the
next one's result meaningless.
