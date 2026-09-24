# IMU reorg: drop the MPU family, make the ZED IMU required — and give topic names an SSoT

**Goal**: two changes that turned out to belong together.

1. **Drop the InvenSense MPU IMU family** (MPU9250, MPU9150, MPU6050/60xx). Its
   accuracy is not good enough for this platform. The ZED camera's IMU becomes
   the only supported source, and a required one.
2. **Give topic names a single source of truth.** The IMU reorg is where this
   surfaced: the repo already carries two wrong IMU topic names that nobody
   noticed, because the same name is written as a literal in launch XML, again
   as a Python parameter default, and again in monitor and record-list YAML.

**Status**: surveyed, not started. Four read-only surveys are complete and
recorded below; no reorg edit has been made. Two decisions are settled, two
questions block the first commit, and one pre-existing defect has to be fixed
before "ZED IMU is required" can be true.

**Why this doc exists**: the survey cost four agent runs over the launch tree,
the installed Autoware binaries, and `play_launch`'s parser. The findings are
cited to `file:line` so the work can restart without repeating any of it.

---

## Decisions taken

| question | decision |
|---|---|
| `cube1_usb`, the one suite with no ZED | **requires a ZED X Mini for its IMU** |
| version bump | **`0.2.0` → `0.3.0`** — pre-1.0 exemption from the MAJOR rule, though removing an `imu_source` value is a sensor-config and launch-API break |

## Questions that block the first commit

1. **The SSoT mechanism.** See "What is actually possible" below. The shortlist
   is: add a `$(yaml file key)` substitution to `play_launch` and use
   `$(command get-topic.sh …)` until it lands, or generate a committed launch
   fragment from `topics.yaml` and enforce freshness with a test. A sub-question:
   `play_launch` is not on PATH, its source is at `/home/aeon/repos/play_launch`
   (`3b01e54d`) and a separately packaged parser exists at
   `~/.nros/sdk/play_launch_parser/0.1.0-nros1/bin/` — **which build `just launch`
   resolves to was not determined**, and "we can fix play_launch" depends on the
   answer.
2. **The ZED SDK becomes mandatory, and that contradicts the workshop.** Three
   places promise the opposite in writing:
   - `src/sensor_component/external/zed-ros2-wrapper/zed_components/CMakeLists.txt:94-150`
     skips the package when the SDK is absent, with the rationale that a hard
     `find_package` "failed the WHOLE workspace build on any machine without
     them -- which is every clean install, and every laptop in a classroom"
   - `setup/autosdv_setup/registry.py:571-574` — "safe to leave selected: a
     machine with no ZED loses nothing", and the `zed-sdk` step is
     `profiles=_on(*DEV)`, i.e. opt-in
   - `docs/guides/zed_camera.md` — "Without the SDK, nothing else breaks"

   The classroom case those were written for is the case a mandatory IMU breaks.
   The container ([roadmap 12](12-student-container.md)) may already settle it;
   that was not checked.

---

## The blocking defect: two shipped suites have no IMU TF

**`robin_zed` and `vlp32c_zed` are broken today**, and the reorg cannot claim a
working ZED IMU until this is fixed.

With `camera_model:=zedxm`, `imu_source:=zed`, `launch_driver:=true`, nothing
publishes `base_link → zedxm_imu_link`:

- `zed_with_object_detection.launch.xml:16` defaults `publish_imu_tf` to
  `false`, and passes it to the node at `:64`
- `camera.launch.xml:23-28` never forwards a `publish_imu_tf` value
- `imu.launch.xml:11`'s group is skipped, because its guard is
  `imu_source=='zed' and camera_model!='zedxm'` — so neither `zed_imu_only.launch.xml`
  (which sets it `true` at `:38-41`) nor the static `zed_imu_tf.launch.xml` runs
- the ZED vendor URDF has no IMU link at all: grep `imu` in
  `zed-ros2-wrapper/zed_wrapper/urdf/zed_macro.urdf.xacro` returns nothing

`autoware_imu_corrector` then drops every message and logs `Please publish TF
base_link to zedxm_imu_link`. That is precisely the failure recorded in
`docs/reports/cuda-ndt-coss-replay.md:59-81` — 15320 log lines, `gyro_odometer`
publishing zero twists, the EKF frozen. The 2026 fix at `imu.launch.xml:28-31`
covered only `camera_model=none` + `launch_driver=false`.

Nothing in `docs/known-config-defects.md` records it. **Deciding who owns
`publish_imu_tf` is the first design task of the reorg**, not a detail.

Only two things ever publish that edge, and they must stay mutually exclusive:
the wrapper itself when `sensors.publish_imu_tf` is true
(`zed_camera_component_main.cpp:4759-4835`, early return at `:4800`), using real
factory extrinsics; and the static publisher in `zed_imu_tf.launch.xml:40-44`,
whose six offsets all default to `0.0`. Today exclusivity holds only because the
static one is confined to `unless="$(var launch_driver)"`.

---

## Survey 1: the MPU footprint

**Four affected files live in submodules.** An AutoSDV-only commit will appear
to work in-tree and regress on a fresh `git submodule update`. Order: each
submodule first, then its pin.

| submodule | branch | what it holds |
|---|---|---|
| `src/sensor_kit/autosdv_sensor_kit_launch` | `main` | `imu.launch.xml`, `sensing.launch.xml`, `sensor_kit.xacro`, its `sensor_kit_calibration.yaml`, `README.md` |
| `src/param/autoware_individual_params` | `1.5.0` | `imu_corrector.param.yaml`, the live `sensor_kit_calibration.yaml` |
| `src/system/autosdv_system_monitor` | `1.5.0` | `monitor_topics.yaml` |
| `src/sensor_component/external/ros2_mpu9250_driver` | `main` | the driver being deleted |

The driver submodule is `.gitmodules:20-23`, url
`https://github.com/NEWSLabNTU/ros2_mpu9250_driver.git`, pinned at
`e90ef9782c98791ad1fb6cec608fd89546bddf33`.

**No `package.xml` anywhere declares `mpu9250driver`.** The only coupling is
runtime: `imu.launch.xml:48` (`pkg=`) and `:50`
(`$(find-pkg-share mpu9250driver)/params/mpu9250.yaml`). That second line is the
top trap — delete the submodule while it stands and launch fails at resolve time.

**Nothing in `setup/` touches the MPU.** Zero hits. The single `i2c` mention,
`registry.py:13`, is prose about the PCA9685 *actuator* bus and stays.

### Two undefined-variable traps

Neither `imu_raw_topic` nor `suite_imu_source` has an unconditional default:

- `imu.launch.xml:41,43` — one `<let>` per `imu_source` value, both guarded by `if=`
- `sensing.launch.xml:24-72` — one `<let>` per suite, all guarded by `if=`

So a surviving `imu_source:=mpu9250` or `sensor_suite:=*_mpu` — shell history, a
script, the book — becomes an undefined-variable abort rather than a readable
error. **Add fallbacks as part of this work**; the removal is the occasion, not
the cause.

### Suite fate

Suites are defined in exactly one place, `sensing.launch.xml:20-75`.

| suite | `imu_source` today | after |
|---|---|---|
| `robin_zed` | `zed` | unchanged; canonical Robin suite. **Has the TF defect above.** |
| `robin_zed_mpu` | `mpu9250` | byte-for-byte duplicate of `robin_zed` → delete |
| `vlp32c_zed` | `zed` | unchanged. **Has the TF defect above.** |
| `vlp32c_zed_mpu` | `mpu9250` | byte-for-byte duplicate of `vlp32c_zed` → delete |
| `vlp32c_zed_imu` | `zed` | unchanged; repo default (`autosdv.launch.yaml:66-68`). `camera_model=none`, so it takes the IMU-only path. |
| `cube1_usb` | `mpu9250` | rewrite to `zed`; now requires a ZED X Mini (decided) |
| `custom` | `mpu9250` | rewrite to `zed`; it is also `sensing.launch.xml:8`'s default suite |

### Everything else to touch

Branch deletions and wording: `sensing.launch.xml:8,14,29-35,45-51,61` ·
`imu.launch.xml:3,41,45-52` · `autosdv.launch.yaml:68,84` ·
`logging_simulation.launch.yaml:98,114` ·
`autosdv_dead_reckoning_component.launch.xml:31` (wording only) ·
`config/localization/preset/README.md:19`

Live defects if left: `monitor_topics.yaml:14` lists the MPU raw topic and the
monitor subscribes to every entry, so it becomes a permanent IMU fault in the
monitor UI. `scripts/testing/rosbag/sensing-topics.txt:15` records the MPU topic
and does **not** record the ZED's — after removal no raw IMU topic is recorded at
all, so add the ZED topic in the same change or replay-based IMU debugging loses
its input.

Docs needing a rewrite rather than a deletion:
`docs/guides/sensor_configuration.md:13-14,19-20,25-29,36-41,61-62` (the primary
user-facing doc) · `docs/guides/lio_sam_mapping.md:58,77,119` (both worked
examples pass `imu_source:=mpu9250` as the only IMU) and `:164-168` (an
MPU-specific noise block) · `src/sensor_kit/autosdv_sensor_kit_launch/README.md:26,55,58`
(MPU is the only IMU the kit documents) · `docs/design/coach-cruise-lab.md:193-196`
(its `imu_yaw_sign` argument is built on the deleted driver file; the conclusion
survives and must be re-derived) · `docs/roadmap/8-book-revision.md:310` (its
"correct value" column enumerates the `_mpu` suites as ground truth — re-point it
or the names come back).

### Two pre-existing errors found in passing

- `CLAUDE.md:809` describes `vlp32c_zed_imu` as "Velodyne + ZED + ZED IMU +
  MPU9250". It is `camera_model=none` + `imu_source=zed`. Wrong today.
- `docs/guides/sensor_configuration.md:68` claims "The system automatically
  relays IMU data from the camera node to avoid launching duplicate ZED
  drivers." There is no relay node; `imu.launch.xml:41-43` only switches which
  topic `imu_corrector` subscribes to.

### Do not delete `imu_link`

`sensor_kit.xacro:101-121` defines `imu_joint` and `imu_link`, at
`z: -0.055` in both copies of `sensor_kit_calibration.yaml`. That number is
almost certainly a placeholder: its sibling `gnss_base_link` is
`z: +0.055 # random value`. Nothing has ever published in `imu_link` — the MPU
driver stamped `base_link` (`mpu9250driver.cpp:48`), the ZED stamps
`zedxm_imu_link` — so the removal does not orphan it further. But three RViz
configs list it in their TF display (`rviz/autosdv.rviz:104`,
`rviz/workshop.rviz:104`, `rviz/mapless.rviz:105`), and the URDF link is what
makes `robot_state_publisher` emit it.

---

## Survey 2: the ZED IMU path

### Two launch paths, one node identity

| condition | ZED node from | `publish_imu_tf` | uses `zed_imu_minimal.yaml` |
|---|---|---|---|
| `camera_model=zedxm`, `launch_driver=true` | `zed_with_object_detection.launch.xml` | **false** | no |
| `camera_model=zedxm`, `launch_driver=false` | none (`zed_tf_only.launch.xml`, TF only) | n/a | no |
| `camera_model∈{usb,none}`, `imu_source=zed`, `launch_driver=true` | `zed_imu_only.launch.xml` | **true** | yes |
| `camera_model∈{usb,none}`, `imu_source=zed`, `launch_driver=false` | none (`zed_imu_tf.launch.xml`, static TF) | n/a | no |

Both paths build *identically named* entities — container `zed_container` in
`/sensing/camera/zedxm`, composable node `zedxm` in `/sensing/camera`, and a
`zedxm_state_publisher` — so if both ever fire, the second `sl::Camera::open()`
fails and the component `exit(EXIT_FAILURE)`s, taking the container with it.

**The only guard is `imu.launch.xml:11`'s compound condition**, and it works
solely because `sensing.launch.xml:117` forwards the same `final_camera_model`
to both includes. The reorg breaks it if it (a) unconditionally launches a ZED
IMU node, (b) stops forwarding `camera_model`, or (c) adds a camera value that
is not literally the string `zedxm` — the `!=` then reads as "no camera path".
**The safe shape is a single owner of the ZED node.**

A third, out-of-graph launcher would also collide:
`src/vehicle/control_test/launch/basic_control.launch.xml:18-25` and
`just/tool.just:37-42` (`just tool zed`).

### Without a ZED attached, nothing works

`zed_camera_component_main.cpp:3021-3034` retries `open()` and, after
`mMaxReconnectTemp * mCamTimeoutSec`, logs `Camera detection timeout` and the
node exits. So `/sensing/camera/zedxm/imu/data` never appears and
`/sensing/imu/imu_data` is dead. Without the *SDK*, `zed_components` is not even
built, so `load_composable_node` fails.

Note the default suite already starts a ZED driver purely for its IMU under
`camera_model:=none`.

### Numbers, and what is missing

- **Rate: 100 Hz** on both paths. `common_stereo.yaml:64` (`sensors_pub_rate: 100.`)
  and `zed_imu_minimal.yaml:52`; measured at "~100 Hz" in
  `docs/superpowers/plans/2026-07-25-2dlidar-phase-3.md:19`.
- **Noise and bias are not the ZED's.**
  `individual_params/.../autosdv_sensor_kit/imu_corrector.param.yaml:3-8`
  (`angular_velocity_stddev_*: 0.00339`, offsets `-0.005799 / -0.007148 /
  -0.001499`) is byte-identical to Autoware's `sample_sensor_kit` Tamagawa
  values. And `imu.launch.xml:38` feeds the same file for both sources with no
  per-source branch — a defect in its own right. **Re-measure for the ZED.**
- **The ZED's yaw-rate sign is undetermined in-repo.** Both live defaults are
  `1.0`; the `-1.0` in `mcl_localization.launch.xml:36` is explicitly a Tamagawa
  property and must not be inherited. `docs/design/coach-cruise-lab.md:187-203`
  gives the measurement procedure.
- **`zedxm_camera_link` mounting is guessed**, per
  `docs/reports/cuda-ndt-coss-replay.md:175-176`: "The x/y/z entries there are
  still unmeasured, as are the ZED, GNSS and IMU mounts." The only empirically
  derived rotation in the repo is the VLP-32C yaw `-0.2210`.

### Also noted

- The description-package copy of `sensor_kit_calibration.yaml` is **stale**: it
  has `vlp32c: yaw: 0.0` where the authoritative `individual_params` copy has
  `-0.2210`. `autosdv_autoware.launch.xml:167` makes `individual_params` the live
  one.
- Two `robot_state_publisher`s publish the ZED body frames (the vehicle URDF via
  `sensor_kit.xacro:158-167`, unconditionally regardless of `camera_model`, and
  the per-path vendor one). Geometry is identical, so benign — but it is two
  publishers of the same edges.
- `CLAUDE.md:383` documents `data/zed-sdk/`. **The directory does not exist** and
  nothing references it. Stale until confirmed.
- `imu.launch.xml:42` and `monitor_topics.yaml:15` annotate the ZED topic with
  "(ZED 5.1.0)"; the in-repo SDK is 5.4.1 (`versions.yaml:170-175`).

---

## Survey 3: what is actually possible for a topic-name SSoT

### The drift being fixed

The same IMU topic is written three ways, and one of them matches nothing:

- `mcl_localization.launch.xml:35` — `/sensing/imu/tamagawa/imu_raw`, an Autoware
  sample-kit name **this kit never publishes**
- `wheel_imu_odom.py:71` — `/sensing/camera/zedxm/imu/data`
- `imu.launch.xml:41,43` — `mpu9250/imu_raw` or `/sensing/camera/zedxm/imu/data`

So the launch default and the node default of the same node disagree.

### What the two runners support

`play_launch` implements exactly: `Text`, `$(var)`, `$(env)`, `$(optenv)`,
`$(command)`, `$(find-pkg-share)`, `$(dirname)`, `$(filename)`, `$(anon)`,
`$(eval)` — `substitution/types.rs:36-68`. Unknown names are a **hard error**
(`substitution/parser.rs:374-377`), which is at least loud.

Relative to ROS 2 Humble it is therefore missing `$(file-content)`, `$(param)`,
`$(find-pkg-prefix)`, `$(exec-in-pkg)`, `$(find-exec)`, `$(launch_log_dir)` and
every boolean substitution (`$(if) $(and) $(or) $(not) $(equals) $(not-equals)
$(any) $(all)`). It also **silently drops** `<set_parameters_from_file>`
(`traverser/ir_builder.rs:438-440`, debug log only) — the same failure class as
the already-documented `if=` on `<composable_node>` and `launch-prefix` bugs.

**Two findings that change the design:**

- **`$(eval)` can import modules under `ros2 launch`, and cannot under
  `play_launch`.** Upstream's `python_expression.py:88` is
  `eval(expr, {}, math.__dict__)`, and CPython injects `__builtins__` into a
  globals dict lacking it, so `$(eval "__import__('yaml').safe_load(...)")`
  works — verified live through the real frontend. `play_launch` installs a
  32-name builtins allowlist (`crates/pyexec/src/eval_impl.rs:34-78`) whose
  comment says it exists to prevent exactly that. **Upstream's behaviour is an
  accident, not a feature**; building the SSoT on it means the SSoT works on
  only one runner, and "fix play_launch to match" means deleting a sandbox.
- **XML include scoping differs, and not the way one would guess.** Real
  `ros2 launch` **leaks** a bare XML include's `<let>`s upward into the parent
  (`include_launch_description.py:146-185`; only `GroupAction` scopes, and it
  defaults `scoped=True`). `play_launch` **isolates** XML includes but propagates
  YAML ones (`ir_builder.rs:60-65`). So a `<let>`-only `topics.launch.xml`
  included bare works under one runner and silently no-ops under the other.

### What neither runner has

**There is no `$(yaml file key)` substitution anywhere.** Not in Humble, not in
`play_launch`. `$(param)` is a dead end regardless: it reads
`context.launch_configurations['global_params']` but matches only *tuple*
entries, which only `<set_parameter>` produces, while
`<set_parameters_from_file>` appends a filename string that is never parsed
(`launch_ros/actions/set_parameters_from_file.py:77-81`).

So something must be built either way. The portable intersection today is
`$(command …)`.

### Autoware's own answer cannot be reused

`component_interface_specs` (`/opt/autoware/1.5.0/include/autoware/component_interface_specs/*.hpp`)
pins topic name + message type + the full QoS triple as `static constexpr`, e.g.
`localization.hpp:43-50` for `KinematicState`. But it is **header-only C++ with
no Python or YAML export anywhere in the install**, and Autoware's own launch
files ignore it: `/localization/kinematic_state` appears as a raw literal **59
times** across `/opt/autoware/1.5.0/share/*/launch/*.xml`, and only four
packages declare the dependency. Deferring to it is a policy we write down, not
a mechanism we call.

### Prior art: `versions.yaml`

`versions.yaml` is the repo's working SSoT, exposed two ways, both PyYAML in a
`python3 -c` heredoc, deliberately — `scripts/version/get-version.sh:32-36`
explains why not `yq` ("exists in two incompatible flavors … a wrong guess fails
by printing nothing while exiting 0"). `export-versions.sh:32-67` is a *manual*
allowlist of 20 pairs, and its header doubles as the schema.
`setup/autosdv_setup/registry.py:37-45` just imports yaml and reads the file.

Nothing in that chain reaches launch XML. A `get-topic.sh` mirroring
`get-version.sh` would be reachable from launch through `$(command …)`, which
both runners support.

### Where an enforcement test must live

- `just test` runs `colcon test --base-paths src`, so **a repo-root test is never
  collected**.
- **`src/launcher/autosdv_launch` is `ament_cmake` with no `ament_add_pytest_test`
  anywhere in the repo.** Its `test/test_flake8.py`, `test_copyright.py` and
  `test_pep257.py` are **dead files** — the equivalent lint runs via
  `ament_lint_auto` from `<test_depend>`, not from them. A pytest dropped there —
  the natural home, since most launch XML lives there — **would silently never
  run.** This is the foot-gun to avoid.
- The working precedent and the right template is
  `src/localization/autosdv_mcl_launch/test/test_launch_xml_wellformed.py`: an
  `ament_python` package whose tests do run, already file-checking rather than
  behaviour-testing, globbing launch XML, asserting the glob is non-empty, and
  reporting `file:line`.
- A repo-wide scan would also see the vendored trees
  (`src/calibration/CalibrationTools` ≈ 100 launch XML,
  `src/localization/tier4_localization_launch`,
  `src/localization/cuda_ndt_matcher/tests/**`), which hold thousands of upstream
  literals — so it needs an explicit allowlist of AutoSDV-owned paths. There is
  no repo-root `pytest.ini`, `conftest.py` or `tox.ini` to hang shared config on.

---

## Work order, once the two questions are answered

1. **Fix `publish_imu_tf` ownership** so `robin_zed` and `vlp32c_zed` publish
   `base_link → zedxm_imu_link`, preserving single-owner exclusivity with
   `zed_imu_tf.launch.xml`. Record the defect in
   `docs/known-config-defects.md`. This is independently worth doing and should
   land first, on its own.
2. **Decide the ZED node's single owner** across the camera and IMU paths, so a
   mandatory IMU cannot produce two drivers on one camera.
3. **Submodule commits, innermost first**: `autosdv_sensor_kit_launch` (launch
   rewrite, suite table, README), `autoware_individual_params` (per-source
   `imu_corrector` params), `autosdv_system_monitor` (drop
   `monitor_topics.yaml:14`). Push each, then pin.
4. **Superproject**: remove the `ros2_mpu9250_driver` submodule and its
   `.gitmodules` stanza, add the `<let>` fallbacks, fix the top-level arg
   descriptions and the record-topic list, bump `versions.yaml` to `0.3.0`.
5. **Docs**: `sensor_configuration.md`, `lio_sam_mapping.md`, `CLAUDE.md:809,814`,
   the preset README, `coach-cruise-lab.md:193-196`, and re-point
   `8-book-revision.md:310`.
6. **`topics.yaml` and the SSoT**, by the chosen route, with the enforcement test
   in an `ament_python` package. The three IMU topic names are its first
   entries, and `mcl_localization.launch.xml:35` is its first fix.
7. **Measure what is currently inherited**: the ZED's `imu_yaw_sign`, its gyro
   stddev and offsets, and — separately — the ZED and GNSS mountings.

## Adjacent, deliberately out of scope

- `mcl_localization.launch.xml:35`'s tamagawa topic is fixed by step 6, not by
  the IMU removal.
- The stale `vlp32c: yaw: 0.0` in the description-package
  `sensor_kit_calibration.yaml`.
- The double `robot_state_publisher` on the ZED body frames.
- Whether `data/zed-sdk` in `CLAUDE.md:383` ever meant anything.
