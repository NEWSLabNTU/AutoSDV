# Workshop: two hours, a laptop, and a working AutoSDV

**Goal**: a 2-hour classroom workshop in which each student installs AutoSDV on
their own laptop, learns the ROS 2 concepts the installation is made of while it
runs, and verifies the result by driving both simulations.

**Status**: Not started. Blocked on the book revision — see
[8-book-revision.md](8-book-revision.md).

**Source material**: `Lab_0__Autoware_Setup_and_Camera_Calibration.pptx`, 44
slides, the existing NTU lab deck. Camera calibration is out of scope and its
slides are dropped; the triage is below.

---

## Why this waits for the book

The workshop is the book's installation and launch chapters, performed live with
thirty people who cannot debug it themselves. Every defect catalogued in roadmap
7 becomes a room full of raised hands:

- the deck's slide 8 sends students to
  `newslabntu.github.io/autosdv-book/software_installation.html` — an mdBook URL
  that no longer exists under MkDocs — and promises "Autoware version 2025.02",
  which is not what installs
- the setup script the deck describes was replaced by the step registry
- `just tool-*` and `just control-*` no longer exist

So: roadmap 8 phases 0–2 first, then this. The workshop's handout should be
book pages, not a parallel set of instructions that drifts on its own.

---

## Shape of the two hours

The deck's own line — *"Install Autoware + AutoSDV on your laptop in the mean
time"* — is the correct structure and should be made explicit. The install is
long and mostly unattended; the ROS 2 teaching fills it.

| # | Block | ~min | Notes |
|---|-------|------|-------|
| 0 | Prerequisites checked at the door | 10 | see hardware section; students who fail this need a fallback |
| 1 | What AutoSDV is | 10 | deck slides 16, 2 |
| 2 | **Kick off the install** | 5 | `./setup.sh --run --profile dev --yes`, then leave it running |
| 3 | ROS 2 concepts, while it installs | 30 | deck slides 4–7; workspace, `src/`, rosdep, colcon, `source install/setup.bash` |
| 4 | Topics, nodes, and bags | 20 | deck slides 22–26 |
| 5 | Verify 1: planning simulation | 20 | no sensors, no GPU, no map download |
| 6 | Verify 2: logging simulation | 20 | the rosbag replay |
| 7 | Troubleshooting and next steps | 5 | |

Two hours has no slack, and the install is the risk. Decisions below address it.

---

## Hardware requirements — to be authored

The deck states none. This is the single largest gap and the most likely cause
of a failed session, because the room will contain laptops that cannot run what
is being taught.

What needs to be written and, more importantly, **measured on a real laptop**
before the workshop is announced:

| | Minimum | Suggested |
|---|---|---|
| OS | Ubuntu 22.04 (ROS 2 Humble; no substitute) | Ubuntu 22.04 native |
| CPU | to be measured | |
| RAM | to be measured — colcon linking is the peak | |
| Disk | to be measured; the Autoware Debian install plus the workspace plus a 2.8 GB rosbag | |
| GPU | **none required, with caveats below** | NVIDIA, for the default pose source |

### The GPU caveat is the part that must be got right

AutoSDV defaults to `pose_source:=cuda_ndt`, which needs CUDA, and perception
defaults to TensorRT models that need a GPU and a long first-run engine
compilation. Neither is acceptable in a 2-hour class on mixed laptops.

- **Planning simulation needs no GPU at all** — no sensors, no perception, no
  localization. It is the right first verification and should stay first.
- **Logging simulation on a GPU-less laptop** needs `pose_source:=ndt` (the
  OpenMP CPU matcher) and almost certainly `launch_perception:=false`. Both
  must be verified on such a machine before the workshop, not assumed.

This also makes a teaching point out of a constraint: the student types the
argument that selects a CPU matcher, and thereby learns what `pose_source`
means. That is the launch-argument lesson roadmap 8 phase 2 is built around.

### Fallback for laptops that do not qualify

Options, to be decided: a lab machine per pair of students, a prepared VM image,
or having them follow along without running. Whatever is chosen must be decided
before the announcement, not during the session.

---

## Slide triage

**Keep — 19 slides**, all needing correction:

| Slides | Content | Correction needed |
|--------|---------|-------------------|
| 1, 2 | Title, outline | retitle; drop the calibration line from the outline |
| 3 | Section: installation | — |
| 4–7 | Standard ROS build steps: `vcs import`, `rosdep`, `colcon build`, `source install/setup.bash` | keep nearly as-is; this is the best ROS-concept material in the deck. Retain the "do not run this manually" notice and use it to explain what `setup.sh` automates |
| 8 | Install AutoSDV | rewrite: dead URL, wrong Autoware version, and the current profile-based command |
| 16 | What AutoSDV is | update the book link |
| 9–11 | Planning simulation | replace the upstream `autoware_launch planning_simulator.launch.xml` with sample map/vehicle by AutoSDV's own: `autosdv_launch`, COSS map, `autosdv_vehicle`. Removes a separate sample-map download |
| 12–14 | RViz: 2D Pose Estimate → 2D Goal Pose → Auto | screenshots are still accurate in shape; retake against the current RViz config |
| 22–26 | `ros2 bag` record, info, play, `-r`, `--loop`, `--clock`, `--topics` | keep; `--clock` is exactly what the logging simulation needs, so this block now earns its place instead of serving the calibration lab |
| 44 | References | keep; add the AutoSDV book |

**Drop — 25 slides**: 15, 17–21 (USB and ZED camera drivers) and 27–43 (the
whole intrinsic calibration lab: checkerboard, camera matrix, distortion
coefficients, the calibrator tool, the submission instructions). None of it
serves setup, ROS concepts, or simulation.

The calibration material is good and should not be thrown away — it is a
candidate for a separate Lab, and `src/launcher/autosdv_launch/launch/camera_calibration.launch.xml`
already exists in this repo to support it.

**Author from nothing — not in the deck at all**:

- hardware requirements and the laptop triage above
- `play_launch` — the deck teaches `ros2 launch` only, while roadmap 8 phase 2
  makes `play_launch launch` the form students should learn
- the logging simulation (block 6); the deck stops at planning
- what a successful install looks like (`./setup.sh --status`)

---

## Content this repo already has

Do not write these twice. The workshop's slides should compress what the book
says, and link to it.

| Need | Source |
|------|--------|
| Install steps, profiles, flags | roadmap 8 phase 1 → book installation chapter |
| Launch syntax, `arg:=value` | roadmap 8 phase 2 → book launch chapter |
| Planning / logging simulation walkthroughs | roadmap 5's content plan, which was **never written** (see roadmap 8) — so this is real work, tracked in roadmap 8 phase 3.1 |
| Rosbag download | `just bag download`, ~2.8 GB |

---

## Open decisions

1. **Preserve the source deck.** The pptx currently exists only at
   `~/.claude/uploads/888e2568-.../79df663f-Lab_0__Autoware_Setup_and_Camera_Calibration.pptx`,
   which is not durable. It is 17 MB, so committing it needs a deliberate call:
   commit it, keep the images out and commit only the extracted text, or store
   it outside git.
2. **Network.** Autoware Debian packages plus a 2.8 GB rosbag, times thirty
   laptops, over classroom wifi, will not finish inside two hours. Pre-stage on
   USB sticks, a local mirror, or require both to be downloaded before class.
3. **Install profile.** `--profile dev` is the right base — `ci` omits Autoware
   entirely and cannot build. Whether the workshop wants a narrower `workshop`
   profile in `registry.py` (dropping PlotJuggler, gdown, the u-blox rules) is a
   real question, and adding one is a single entry in that file.
4. **Fallback for non-qualifying laptops** (see above).
5. **Where the slides live.** A new repo, a directory here, or generated from
   the book's own pages.
6. **Calibration lab.** Keep the dropped 25 slides as a separate Lab, or retire
   them.

---

## Before this can be scheduled

- [ ] Roadmap 8 phases 0–2 complete — the book is correct
- [ ] Requirements measured on a real non-NVIDIA laptop
- [ ] Logging simulation verified with `pose_source:=ndt` on that laptop
- [ ] A full install timed end to end, from a clean Ubuntu 22.04
- [ ] Decisions 1–6 settled
