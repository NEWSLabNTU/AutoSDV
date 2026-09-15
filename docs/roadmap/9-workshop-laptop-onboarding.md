# Workshop: two hours, a laptop, and a working AutoSDV

**Goal**: a 2-hour classroom workshop in which each student reaches a running
AutoSDV simulation on their own laptop, learns the ROS 2 concepts by inspecting
a live system, and drives both simulations.

**Status**: unblocked. The container ([roadmap 12](12-student-container.md))
removed the install, and the book page documenting it is written. What remains
is the deck itself and the decisions at the bottom.

**Source material**: `Lab_0__Autoware_Setup_and_Camera_Calibration.pptx`, 44
slides, the existing NTU lab deck. Camera calibration is out of scope and its
slides are dropped; the triage is below.

---

## What the container changed

This document was written around an install: students would run `setup.sh` on
their own Ubuntu machines and learn ROS 2 while apt worked. **That premise is
gone**, and with it most of this plan's risk.

| | before | now |
|---|---|---|
| OS | Ubuntu 22.04, no substitute | any: Windows, macOS, Linux |
| Apple Silicon | could not participate | native arm64 image |
| in-class install | 40+ minutes, unattended, the session's biggest risk | `docker run`, about a minute |
| what can fail | a hundred packages on thirty machines | Docker is installed, or it is not |
| laptop triage | required, unmeasured, with no fallback decided | memory setting, checked at the door |

**The 30-minute "ROS 2 concepts while it installs" block no longer has an
install to cover.** That block is not deleted — the material is the best in the
deck — but it stops being filler and becomes teaching in its own right, against
a system that is already running. That is a better lesson than it was: `ros2
node list` on a live 138-node stack beats a slide about what a node is.

The two hours are re-planned below rather than trimmed.

---

## Shape of the two hours

Nothing here waits for anything else, which is the point.

| # | Block | ~min | Notes |
|---|-------|------|-------|
| 0 | Containers up, checked at the door | 10 | pulled at home the night before; this is verification, not download |
| 1 | What AutoSDV is, and what we are about to run | 10 | deck slides 16, 2 |
| 2 | Planning simulation — run it before explaining it | 25 | RViz in the browser; 2D Pose Estimate → 2D Goal Pose → Auto |
| 3 | ROS 2 against the running stack | 25 | `ros2 node list`, `topic echo`, `topic hz`, the graph. Deck slides 4–7 supply the workspace vocabulary |
| 4 | Launch files and arguments | 15 | `play_launch`, `arg:=value`, and why `pose_source:=ndt` here |
| 5 | Logging simulation, two terminals | 25 | the rosbag replay; the second terminal is the launcher run again |
| 6 | `ros2 bag`, and where to go next | 10 | deck slides 22–26, compressed |

**Block 2 before block 3 is deliberate.** The old order explained ROS 2 and then
ran something; the container makes the reverse possible, and seeing the finished
thing work first is worth more — the same argument the book's *First Run* page
makes.

Slack: roughly 5 minutes, against 0 before. The install was the thing with no
slack at all.

---

## Prerequisites, and what to check at the door

The requirement is no longer a machine that can build Autoware. It is a machine
that can run Docker and spare it some memory.

| | Minimum | Notes |
|---|---|---|
| OS | Windows, macOS or Linux | Apple Silicon included, natively |
| Docker | Docker Desktop, or Docker Engine on Linux | the one thing that must be installed beforehand |
| Memory given to Docker | 8 GB; **12 GB recommended** | Docker Desktop's own setting, not the machine's RAM |
| Disk | 30 GB free | |
| Download | 5.4 GB Apple Silicon, 14.3 GB elsewhere | **before class** — see network, below |
| Repository clone | ~80 MB, carries the COSS map | the map is **not** in the image; the `data/` mount is what supplies it |
| GPU | none | and none is possible on macOS |

**The memory setting is the one to check at the door**, because its failure mode
teaches nothing: nodes are killed as they start, and the log shows scattered
`signal 6 (Aborted)` across unrelated nodes with no mention of memory anywhere.
A student hitting it mid-class looks like a student hitting a bug.

Two things follow for the session, both already documented on the book's
[container page](https://newslabntu.github.io/autosdv-book/):

- **`--container-mode observable`** wherever a student types `play_launch`
  directly. Measured on 8 cores and 8 GB: all nodes ready in 7 s against ~90 s,
  3.09 GiB against 5.71 GiB. On a memory-capped Docker VM this is the difference
  between working and not.
- **`pose_source:=ndt`**, not the `cuda_ndt` default, which needs a GPU at run
  time that no laptop in this room has. The student types the argument that
  selects a CPU matcher and thereby learns what `pose_source` means — the
  constraint is the lesson.

### Fallback for laptops that do not qualify

Much smaller than it was, and now mostly "their Docker has too little memory",
which is a setting rather than a machine. What is left:

- a machine with less than ~10 GB of RAM total, where 8 GB cannot be given to
  Docker
- a locked-down laptop where Docker Desktop cannot be installed

Pair those students with a neighbour. The container makes a single laptop a
perfectly good two-person station, which the old plan could not offer.

---

---

## Slide triage

**Keep — 19 slides**, all needing correction:

| Slides | Content | Correction needed |
|--------|---------|-------------------|
| 1, 2 | Title, outline | retitle; drop the calibration line from the outline |
| 3 | Section: installation | — |
| 4–7 | Standard ROS build steps: `vcs import`, `rosdep`, `colcon build`, `source install/setup.bash` | keep; still the best ROS-concept material in the deck. The framing changes: these are what the image already did, so the "do not run this manually" notice is now literally true — the container ships the workspace prebuilt, and the two `source` lines are done in every shell the launcher opens |
| 8 | Install AutoSDV | **replace with the container**: `docker pull` and one launcher command. The dead URL, the wrong Autoware version and the whole profile discussion go with it |
| 16 | What AutoSDV is | update the book link |
| 9–11 | Planning simulation | replace the upstream `autoware_launch planning_simulator.launch.xml` with sample map/vehicle by AutoSDV's own: `just sim planning`, COSS map, `autosdv_vehicle`. Removes a separate sample-map download |
| 12–14 | RViz: 2D Pose Estimate → 2D Goal Pose → Auto | screenshots still accurate in shape; retake **in the container's browser desktop**, which is what students will see, and note that the viewport is black for ~90 s while the map loads |
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

- getting the container up, and the memory setting that decides whether it works
- `play_launch` — the deck teaches `ros2 launch` only, while roadmap 8 phase 2
  makes `play_launch launch` the form students should learn, and
  `--container-mode observable` is what a laptop needs
- the logging simulation (block 5); the deck stops at planning
- the two-terminal pattern: running the launcher again attaches a second shell
  to the same container, which is the normal case and looks like an error
- what a working container looks like, before anything is launched

---

## Content this repo already has

Do not write these twice. The workshop's slides should compress what the book
says, and link to it.

| Need | Source |
|------|--------|
| Getting the container up, on any of the three platforms | book: *Run It in a Container* (EN + zh-TW) |
| The memory setting, and what its failure looks like | same page |
| `--container-mode observable`, with the measurements | same page, and `docker/desktop/README.md` |
| Launch syntax, `arg:=value` | roadmap 8 phase 2 → book launch chapter |
| Planning / logging simulation walkthroughs | book: *Tutorial* 1–3 |
| Handing the image out offline | `docker/desktop/export-images.sh`, `serve-images.sh` |

---

## Open decisions

1. **Preserve the source deck.** The pptx currently exists only at
   `~/.claude/uploads/888e2568-.../79df663f-Lab_0__Autoware_Setup_and_Camera_Calibration.pptx`,
   which is not durable. It is 17 MB, so committing it needs a deliberate call:
   commit it, keep the images out and commit only the extracted text, or store
   it outside git.
2. **Network — halved, not solved.** Three downloads, and they are not the same
   problem:

   | what | size | where it comes from |
   |---|---|---|
   | the image (Autoware, ROS 2, the built workspace) | 5.4 / 14.3 GB | Docker Hub, **or handed out offline** |
   | the COSS map, 78 MB PCD included | ~80 MB | the git clone; no separate download |
   | the rosbag, for the logging simulation only | **2.8 GB** | `just bag download` — **still per student, still unsolved** |

   The image is the one that used to be impossible and now is not:
   `export-images.sh` writes both architectures to files — named *"Apple Silicon
   Mac"* rather than by architecture — and `serve-images.sh` serves them on the
   local network with an address to put on the board. A USB stick works equally
   well. The map rides along with the repository, so it costs nothing.

   **The rosbag is the remaining network problem**, and block 5 is built on it.
   Thirty copies of 2.8 GB over classroom wifi will not happen inside two hours.
   Options, to be decided: require it before class alongside the image, serve it
   from the same local host as the image files, or put it on the sticks. Do not
   let this one look settled because the image is.
3. ~~**Install profile.**~~ **Moot.** Nobody runs `setup.sh` in class, so a
   narrower `workshop` profile buys the workshop nothing. (It may still be worth
   having for CI; that is a `registry.py` question, not this one.)
4. **Fallback for non-qualifying laptops** — much reduced, see above. What is
   left to decide is only whether to pair students or to keep one or two lab
   machines on hand.
5. **Where the slides live.** A new repo, a directory here, or generated from
   the book's own pages.
6. **Calibration lab.** Keep the dropped 25 slides as a separate Lab, or retire
   them.
7. **New: does the class use `just sim planning` or `play_launch` directly?**
   The recipe is one line and hides the container-mode flag; the raw command
   teaches what a launch file is and what arguments do. Block 2 wants the
   recipe, block 4 wants the raw form — probably both, in that order, but it
   should be a decision rather than an accident.

---

## Before this can be scheduled

- [x] The book is correct about installation — the container page is written,
      EN and zh-TW
- [x] A student-shaped machine verified: 8 cores, 8 GB, arm64, no GPU. Planning
      simulation reaches 34/34 nodes; RViz draws the map
- [x] The image can be handed out offline, so it no longer crosses the
      classroom network
- [ ] The **rosbag** (2.8 GB, block 5 only) staged somehow — the one download
      the container did not remove
- [ ] Logging simulation verified in the container with `pose_source:=ndt`
      (planning is verified; logging is not)
- [ ] A dry run on a **Windows** laptop and on an **Apple Silicon Mac**, by
      someone who has not seen this before
- [ ] Decisions 1, 4, 5, 6, 7 settled
