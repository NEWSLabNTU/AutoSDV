# Book revision: installation and launch, for the v0.2 release

**Goal**: bring `AutoSDV-book` back in step with the code, so the site published
at <https://newslabntu.github.io/autosdv-book/> describes the stack that
actually exists. Two payloads: the **installation chapter**, which is wrong in
every one of its version numbers and describes a setup script that was
replaced; and the **launch chapter**, which teaches `just launch` and leaves the
reader unable to launch anything themselves.

**Status**: Phase 0 done, 2026-09-12. Phase 1 next.

**Book repository**: `~/repos/AutoSDV-book`, branch `main`. It is a *separate
repository*, not a submodule — `feb8a6f Remove book submodule` took it out of
this tree. `CLAUDE.md` in this repo still tells you to `cd book && just setup`;
that path does not exist.

---

## Why now

The book's last commit is `03a38ee`, 2026-01-26. The live site is older still —
it is built from tag `book-v0.2.1` (`17274a3`, 2026-01-15), so three commits on
`main` are unpublished. Since that tag this repository has landed the setup
rewrite, the justfile module split, the CUDA point cloud pipeline, MCL, and the
demo runner. None of it is in the book, and several pages now give commands that
fail.

### Version numbers, and which "v0.2" this is

The book already has tags `book-v0.1.0`, `book-v0.2.0`, `book-v0.2.1`. Its
versioning is its own and is already past 0.2. The **AutoSDV** version in
`versions.yaml` is `0.1.0-dev`, so the v0.2 being cut is the *project's*, and
the book release that accompanies it is `book-v0.3.0`.

---

## A correction to roadmap 5 before anything else

`docs/roadmap/5-book_simulation_tutorials.md` is marked **Done**, with every
success criterion ticked. The work is not in the book. Searched the whole of
`AutoSDV-book` history:

```
git log --all --oneline -- 'src/simulation/*' \
                           'src/getting-started/installation/recommended.md'
(no output)
```

No `src/simulation/` directory, no `planning-simulation.md`, no
`logging-simulation.md`, no `datasets.md`, no `coss-park-scenario.md`, and no
`installation/recommended.md` has ever existed on any branch of that repository.
`mkdocs.yml` has no `Simulation Guide` nav section. The pages roadmap 5 claims
to have written do not exist anywhere.

Two consequences:

1. Roadmap 5's status must be corrected to reflect reality before it is cited as
   a completed dependency. This roadmap does not assume any of it landed.
2. Its **content plan is still good** and is largely what phase 3 below needs.
   Treat `5-book_simulation_tutorials.md` as a specification to execute, not as
   history.

Do not rely on a roadmap's checkboxes over the repository. This is the same
habit `CLAUDE.md` states for submodules — read the config out of git, not out of
a working tree — applied to our own status documents.

---

## Phase 0 — Release readiness gate

Nothing here is prose. It establishes that there is a v0.2 worth documenting.

### 0.1 Verify the workspace builds and tests clean

```bash
just build
just test
```

Record what fails. A book that documents a build which does not succeed on the
maintainer's own machine is worse than a stale one.

### 0.2 Decide and set the project version

`versions.yaml` holds `autosdv.version: "0.1.0-dev"`. For the release:

| Branch | Value |
|--------|-------|
| `develop` | `0.2.0-dev` |
| release | `0.2.0`, channel `stable` |

Per the table in `CLAUDE.md` this is a MINOR bump at minimum (new sensor/feature
support: MCL, the CUDA pipeline). It is **not** a MAJOR: Autoware stays at
1.5.0 and no vehicle-interface or launch API contract was broken.

### 0.3 Fix the book path in this repo's CLAUDE.md

The "Documentation" section still says:

> **Setup**: `cd book && just setup`
> **Source**: `book/src/`

Replace with the separate-repository location and the `book-v*` tag flow.

### 0.4 Audit the book's own toolchain

`just lint` in the book runs `mkdocs build --strict`. Confirm it still passes on
`main` before adding pages, so a later failure is attributable.

**Success criteria**:
- [x] `just build` outcome recorded
- [x] `just test` outcome recorded
- [x] `versions.yaml` bumped
- [x] `CLAUDE.md` book path corrected
- [x] book `just lint` passes on `main`

### Phase 0 results — 2026-09-12

**Build: passes.** `just build`, exit 0, 33 packages in 53.8 s. This was an
**incremental** build over an existing `build/` tree, so it establishes that the
current source compiles, not that a clean build from an empty workspace does.
A clean build is still untested and matters more for the book than for us: it is
what every reader performs.

**Tests: exit 1, and it does not block the release.** 743 tests, 1 error,
415 failures, 43 skipped. Every failure is a **linter** — `copyright`, `flake8`,
`pep257`, `cpplint`, `uncrustify`, `lint_cmake`. Zero functional tests failed.

| Package | Failing linters | In a submodule |
|---------|-----------------|----------------|
| `autosdv_system_monitor` | flake8 125, pep257 36, copyright 3 | yes |
| `autoware_zed_converter` | cpplint 64, flake8 36, copyright 4, uncrustify 3 | yes |
| `autoware_isaac_localization_launch` | flake8 43, copyright 7, pep257 6 | yes |
| `autoware_manual_control` | cpplint 18, copyright 5, lint_cmake 4, uncrustify 3 | yes |
| `zed_components`, `zed_launch`, `zed_ros2` | copyright 5, flake8 10, lint_cmake 5 | yes |
| `autosdv_sensor_kit_launch` | copyright 2 | yes |
| `autoware_isaac_pose_bridge` | cpplint 2 | yes |

Every one of them is a submodule, so clearing them means a pull request per
fork and a pin bump per fix — not v0.2 work. Recorded, not fixed.

**`versions.yaml` was largely fiction, and is now repaired.** The file held
three sections; `export-versions.sh` indexes eight. `source
./scripts/version/export-versions.sh` therefore died on `KeyError: 'prerelease'`
**and still exited 0**, because the failing Python runs inside a command
substitution. The documented way to read the single source of truth silently
exported nothing, and had done so for as long as the extra keys have been read.

Repaired with values taken from the repository, never invented:

- `autosdv.version` `0.1.0-dev` → **`0.2.0`**, with `prerelease: dev`,
  `channel: development`
- Autoware Debian filenames, URL base and both SHA256 checksums lifted from
  `setup/scripts/install-autoware-debian.sh`, which is where they actually live
- **JetPack 6.2**, not 6.0 — the arm64 package is
  `autoware-localrepo-1-5-0_1.5.0-1jetpack62_all.deb`. The book's
  "JetPack SDK version: 6.0 (exactly - not 6.1 or 6.2)" is now doubly wrong, and
  phase 1.1 must correct it alongside the Autoware version.
- The amd64 CUDA/cuDNN/TensorRT triple is **declared and left empty on
  purpose.** `setup.sh` does not install any of them — they come from the host
  image — and the values CLAUDE.md advertised (CUDA 12.3, TensorRT 8.6) match no
  machine this has been built on; the workstation used here runs CUDA 12.8,
  driver 580, TensorRT 10.15. Guessing them into the SSOT would make the book
  repeat the guess. They need the autoware-localrepo build manifest.

**CLAUDE.md corrected** in two places: the book is a separate repository with
`src/`, not `book/src/`, and is published only by a `book-v*` tag; and the
versioning section no longer describes fields that do not exist.

**Book lint: `mkdocs build --strict` passes** on `main`. The translation checker
reports the baseline this campaign inherits:

- 3 missing translations — `build_the_vehicle.md`, `replay_simulation.md`,
  `reference/software/vehicle-interface.md`, the same three orphans phase 3.6
  disposes of
- 2 structural mismatches, both in pages phases 1 and 2 rewrite anyway:
  `getting-started/usage.zh-TW.md` (code block 10) and
  `getting-started/installation/overview.zh-TW.md` (code blocks 5, 6, 7)
- 25 pages up to date

**Left for later, deliberately**: the ROS `package.xml` versions are all
`0.1.0` and do not track `versions.yaml`. Aligning them is a separate decision
about whether per-package versions mean anything here.

---

## Phase 1 — Installation chapter

**Objective**: every version number true, every step that the install actually
requires present.

### 1.1 Correct the Autoware version, everywhere

The book says `2025.02`; the pin is `1.5.0`, installed at `/opt/autoware/1.5.0/`.

| File | Line | Current |
|------|------|---------|
| `getting-started/installation/overview.md` | 89 | `- Autoware 2025.02` |
| `getting-started/installation/manual-environment.md` | 20 | `Autoware 2025.02 Debian packages` |
| `getting-started/installation/manual-environment.md` | 43 | `git clone ... -b release/2025.02` |
| `getting-started/installation/docker.md` | 59 | `git clone -b 2025.02 ... AutoSDV.git` |
| `getting-started/installation/docker.md` | 81 | `Autoware 2025.02 pre-installed` |
| `getting-started/installation/docker.md` | 105 | `**Autoware 2025.02** binary release` |

Plus the `.zh-TW.md` counterpart of each.

### 1.2 Rewrite Step 3 around the step registry

`overview.md` describes a bash script that prompts for three things. The setup
program is now Python with a curses menu and a single ordered step list in
`setup/autosdv_setup/registry.py`. What the page must teach instead:

- **Profiles**, not a flat list: `dev`, `vehicle`, `ci`; `all` and `none` are
  computed. `./setup.sh --run --profile vehicle --yes` is a complete unattended
  install.
- The real flags: `--status`, `--list`, `--run`, `--rerun <id>`, `--dry-run`,
  `--skip <id>`, `--plain` (for terminals curses cannot drive).
- **Opt-in steps are opt-in on purpose** and the reader must choose: `zed-sdk`,
  `blickfeld` (Cube1 only), `isaac-ros`, `tensorrt-engines`,
  `turbovnc-virtualgl`. None are in any default profile.
- `--status` re-reads the machine where a step carries a `verify` command, so it
  can contradict its own recorded run — after a reboot, a JetPack OTA, or an
  Autoware upgrade. Say so; it is the difference between trusting the output and
  being confused by it.

### 1.3 Add the install steps the book omits entirely

Each of these is currently missing, and each produces a failure the reader
cannot diagnose from the book:

| Step | Omitted consequence |
|------|---------------------|
| `just setup-autoware-data` | `/opt/autoware/1.5.0/data` is root-owned, so every TensorRT `.engine` write fails, the engine is discarded, and the same models rebuild and fail on **every** launch |
| `just build-engines` | first launch spends 10–30 min compiling engines inside node constructors, with perception unavailable throughout; must run on the target board |
| Rust toolchain + `colcon-cargo-ros2` | `cuda_ndt_matcher` is Rust; without `--cargo-args --release` it builds unoptimised and `pose_source:=cuda_ndt` runs at ~80 ms/scan against the documented ~5 ms |
| `play_launch` | the launcher everything else in the book will now use |
| `direnv` / `.envrc` | the repo ships an `.envrc`; the book describes writing direnv config by hand |

### 1.4 New page — verifying the installation

`getting-started/installation/verify.md`: `./setup.sh --status`, `just build`,
and one smoke launch, with what a healthy result looks like. Currently the book
ends the install chapter at "you should see the system starting without critical
errors", which is not a check anyone can apply.

### 1.5 Docker page

`docker/` in this repo is a `Dockerfile` + `Makefile` + an L4T apt source. Either
verify it builds against Autoware 1.5.0 and correct the page, or mark the page
as unmaintained. Do not leave it asserting `2025.02` works.

**Success criteria**:
- [ ] No occurrence of `2025.02` remains in the book (EN and zh-TW)
- [ ] `overview.md` describes profiles, flags, and opt-in steps
- [ ] autoware-data, engines, Rust, play_launch, direnv each documented
- [ ] `verify.md` exists and its commands were run
- [ ] Docker page verified or marked unmaintained

---

## Phase 2 — Launch chapter

**Objective**: the reader learns to launch the system themselves, and to read
and write launch arguments. `just launch` becomes a convenience note, not the
lesson.

### 2.1 Rewrite `getting-started/usage.md` in teaching order

The order is the point. It is currently inverted.

1. **`play_launch launch <package> <launch_file> [arg:=value ...]`** as the
   primary form:

   ```bash
   source install/setup.bash
   play_launch launch autosdv_launch autosdv.launch.yaml
   ```

   Explain each position: the package, the launch file inside it, and the
   `name:=value` argument syntax — including that values are strings, that
   booleans are `true`/`false`, and that an empty value means "take the sensor
   suite's default".

2. **`play_launch` as the replacement for `ros2 launch`**, with the reason
   stated rather than asserted: multi-stage shutdown (SIGINT → SIGTERM →
   SIGKILL) instead of orphaned `component_container` processes; a web UI at
   `http://127.0.0.1:8080`; per-process resource and `/diagnostics` monitoring.
   Include `pip install play_launch` and `play_launch setcap`.

3. **`ros2 launch`** shown once as the plain-ROS equivalent, with the orphan
   hazard and the kill-by-PGID recipe, so a reader on a machine without
   `play_launch` is not stranded.

4. **`just launch`** last, as a side note — and stating what it silently injects,
   which is the reason it cannot be the teaching example:

   ```
   play_launch launch --web-addr 0.0.0.0:8081 autosdv_launch autosdv.launch.yaml [ARGS]
   # plus rviz:=false when $DISPLAY is unset
   ```

   Note the port differs from play_launch's own default, and that `ARGS` is one
   quoted string: `just launch ARGS="pose_source:=ndt"`.

### 2.2 Regenerate the argument table

`usage.md` documents 12 arguments. `autosdv.launch.yaml` declares roughly 45.
Derive the table from the file rather than editing the old one, and correct at
minimum:

| Argument | Book says | Truth |
|----------|-----------|-------|
| `pose_source` | default `ndt`, options `ndt, isaac` | default **`cuda_ndt`**; `cuda_ndt, ndt, mcl, isaac, visual` |
| `sensor_suite` | "robin_zed, vlp32c_zed_imu, etc." | `robin_zed, robin_zed_mpu, vlp32c_zed, vlp32c_zed_mpu, vlp32c_zed_imu, cube1_usb, custom` |

Entirely absent and needed: `data_path`, `map_path`, `occupancy_grid_file`,
`mcl_random_seed`, `pointcloud_backend`, `localization_pointcloud_backend`,
`pose_source_package`, `perception_preset`, `localization_preset`,
`launch_system_monitor`, `rviz_config`, and the `launch_*` module switches
(`launch_vehicle`, `launch_sensing`, `launch_localization`, `launch_planning`,
`launch_control`, `launch_perception`, `launch_map`, `launch_system`).

Point the reader at `play_launch resolve` for seeing what a set of arguments
actually expands to — it is the tool that answers "did my argument take effect".

### 2.3 Fix every renamed recipe

The hyphenated recipes are gone; the justfile is modules now
(`just tool rviz`, `just control basic`, `just bag play`). 25 call sites across
three pages:

| File | Occurrences |
|------|-------------|
| `guides/vehicle-control/tuning-and-testing.md` | 15 |
| `getting-started/usage.md` | 9 |
| `guides/sensor-integration/using-sensors.md` | 1 |

Plus each `.zh-TW.md`. Mention that both `just bag play` and `just bag::play`
work, and that `just <module>` lists that module.

**Success criteria**:
- [ ] `usage.md` leads with `play_launch launch` and explains `arg:=value`
- [ ] `just launch` documented as a wrapper, with its injected flags named
- [ ] argument table regenerated from `autosdv.launch.yaml`
- [ ] `grep -rn 'just \(tool\|bag\|control\|map\|sim\|demo\)-' src/` returns nothing
- [ ] every command in the chapter was executed before it was published

---

## Phase 3 — The unwritten chapters

Scope-expandable. Phases 1 and 2 are the release payload; this is what the book
still does not cover at all. Roadmap 5's content plan covers the simulation
subset and should be executed rather than rewritten.

### 3.1 Simulation (from roadmap 5)

`just sim planning`, `just sim logging`, `just sim coss-park`, `just bag
download`. Written against `play_launch launch autosdv_launch
logging_simulation.launch.yaml` with `just sim` as the note, per phase 2's rule.

### 3.2 Localization methods

One page covering `pose_source`: `cuda_ndt` (default), `ndt`, `mcl`, `isaac`,
`visual` — what each needs, what map artefact each consumes, and when to pick
it. The MCL scan contract and the measured accuracy table belong here.

### 3.3 Presets

The perception and localization preset mechanism, how to select one, how to
override an individual argument on top of one, and how to add a preset file.

### 3.4 Maps

`just map check`, `just map grid-from-pcd`, `just map grid-from-bag`, and the
per-method map requirements — PCD for NDT, occupancy grid for MCL, and the
frame check that is the expensive failure.

### 3.5 The CUDA pipeline

`pointcloud_backend`, `localization_pointcloud_backend`, the one-container
constraint, and which LiDARs qualify.

### 3.6 Rehome the orphans

Three pages are in neither the nav nor zh-TW:

| Page | Disposition |
|------|-------------|
| `replay_simulation.md` | fold into 3.1; it cites the dead `ros-launch-perf` repo and `dump_launch`, replaced by `play_launch dump launch` / `play_launch up` |
| `build_the_vehicle.md` | a one-line stub — write it or delete it |
| `reference/software/vehicle-interface.md` | add to nav, translate |

### 3.7 `guides/development.md`

Nine lines promising five sections; the nav ships three. Either write the two
missing ones or stop promising them.

**Success criteria**:
- [ ] Roadmap 5's page set exists, or roadmap 5 is re-scoped
- [ ] No page outside the nav
- [ ] Every EN page has a zh-TW sibling

---

## Phase 4 — Translate, verify, publish

### 4.1 zh-TW for every changed and new page

Then `just lint` and `just audit-translations`.

### 4.2 Add the new pages to `nav` and `nav_translations`

An untranslated nav entry shows an English label in the Chinese build.

### 4.3 Tag and deploy

```bash
git tag -a book-v0.3.0 -m "Documentation v0.3.0"
git push origin book-v0.3.0
```

The deploy workflow builds with `--strict`, publishes `gh-pages`, and updates
the `autosdv-book` submodule in `NEWSLabNTU.github.io`. It fires **only** on a
`book-v*` tag, so an untagged push publishes nothing.

**Success criteria**:
- [ ] `just lint` clean
- [ ] `mkdocs build --strict` clean in CI
- [ ] `book-v0.3.0` tagged and the site serving the new content
- [ ] the website submodule moved

---

## Phase summary

| Phase | Description | Payload | Status |
|-------|-------------|---------|--------|
| 0 | Release readiness gate | build/test verified, version bumped, paths corrected | **Done 2026-09-12** |
| 1 | Installation chapter | versions true, setup registry, the five omitted steps | Not started |
| 2 | Launch chapter | `play_launch` taught first, 45-argument table, 25 recipe fixes | Not started |
| 3 | The unwritten chapters | simulation, localization, presets, maps, CUDA, orphans | Not started |
| 4 | Translate and publish | zh-TW, lint, `book-v0.3.0` | Not started |

## Open decisions

1. **Scope**: phases 1+2 only (the stated ask), or 1+2+3.
2. **Docker**: verify the image against Autoware 1.5.0, or mark the page
   unmaintained.
3. **Roadmap 5**: correct its status to reflect that no page was written, and
   decide whether it is absorbed into phase 3 or kept as a separate roadmap.
