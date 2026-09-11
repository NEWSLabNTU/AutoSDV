# Book: a tutorial a newcomer can actually follow

**Goal**: turn the simulation chapters into a teaching sequence rather than a
set of verification checks, with pictures, and reorganize the book around the
path a newcomer walks rather than around the components the project has.

**Status**: proposal. Roadmap 7 phases 0-4 are done (English and zh-TW);
`book-v0.3.0` is deliberately **not** tagged, so this work lands before the
release.

**Premise from the campaign owner**: the planning simulation teaches the
*planning component*; the logging simulation is the one that resembles a real
drive, because it carries real sensor data. Both run on the COSS map and the
COSS rosbag. The demos must be verified to work, and shown in pictures.

---

## Reading the book as a newcomer

I re-read the book start to finish as someone who has used neither ROS 2 nor
Autoware. The frictions below are in the order they are hit. Several are in
pages this campaign just wrote, which is the point of doing the exercise.

### 1. Nothing ever says what ROS 2 is

The book opens on hardware and moves to installation. By the time a reader
reaches a command, they have met `source install/setup.bash` with no statement
of what sourcing does, that it must be repeated **in every new terminal**, or
what it puts on `PATH`.

The logging simulation then asks for **two terminals, both sourced**, and the
reason is never given. A reader who sources only the first gets
`ros2: command not found` in the second and has nothing to go on.

Unexplained on first use, in the pages as they now stand: node, topic,
publisher/subscriber, launch file, package, workspace, QoS, TF, composable node,
component container, bag.

### 2. Nothing says what Autoware *is made of*

The entire distinction the campaign owner wants to draw — planning simulation
exercises planning, logging simulation exercises sensing and localization —
presumes the reader knows Autoware is a pipeline of
sensing → localization → perception → planning → control.

The book never draws that pipeline. `usage.md` lists `launch_sensing`,
`launch_localization`, `launch_planning`, `launch_control` in a table of
switches, which is the first and only place the component names appear
together, as arguments rather than as an architecture.

This is the single largest gap. Every later choice — which pose source, which
preset, why a simulation is "partial" — is a statement about that pipeline.

### 3. `data/COSS-map-planning` is misleadingly named

It is not a planning map. It contains:

```
lanelet2_map.osm              589 KB   the road network (planning)
pointcloud_map.pcd             78 MB   the point cloud map (NDT localization)
occupancy_grid.pgm/.yaml      3.9 MB   the 2-D grid (MCL localization)
map_projector_info.yaml                geodetic projection
autosdv_map.yaml                       provenance
```

A newcomer reads "planning", uses it for the planning simulation, then meets the
same path in the logging simulation — which is about *localization* — and has no
way to know it is the same complete map serving three different consumers.

Worth a rename in the repository, and worth a short "what is in a map directory"
section in the book regardless.

### 4. The tutorial content is split across two pages that duplicate it

`installation/verify.md` step 4 **is** the planning simulation, with its own
launch command. `simulation/planning-simulation.md` is the planning simulation
again, at length. A reader who follows the install page in order does the
tutorial inside a verification checklist, then finds a fuller version of it
later and does not know whether they missed something.

Verification should *link* to the tutorial, not re-run it.

### 5. Three ways to run the same COSS scenario, with no guidance on which

The book now documents `just sim logging` plus a manual `ros2 bag play`,
`just sim coss-park`, and `just demo run` — and says `just demo run` is the good
one only in passing, on the third page. A newcomer meets the manual two-terminal
version first, which is the one with the timing trap.

For a tutorial the order should be inverted: **`just demo run` first**, because
it works; then take it apart to show what it did.

### 6. No pictures anywhere in the software chapters

The book has 40-odd figures, all of them hardware. The planning simulation page
says "click **2D Pose Estimate** in the toolbar, then click and drag" and
"in the AutowareStatePanel, click **AUTO**" — to a reader who has never opened
RViz, those are names without referents. The original NTU lab deck had exactly
these screenshots and they were the most useful slides in it.

### 7. The install is a long unrewarded slog

Clone, a 2-3 GB Autoware download, a build, a model tree fix, optionally 10-30
minutes of engine compilation — and only then does anything move on screen. The
page never shows the reader what they are working toward.

### 8. No glossary

NDT, PCD, lanelet2, TF, pose, twist, MRM, deskew, NVTL, ODD, rosbag. All used;
none defined.

### 9. Smaller frictions

- `play_launch`'s web UI is on 8080, `just launch`'s on 8081. Documented, but a
  reader who mixes the two commands will open the wrong one.
- "Wait for it to come up" before playing the bag is unquantified in
  `logging-simulation.md`. The real answer is "until `ndt_scan_matcher` exists,
  then ~25 s more for the map", which is what `just demo run` waits for.
- The COSS bag is 157 s but the vehicle is **parked for the first 115.7 s**. A
  reader watching the first two minutes sees nothing happen and concludes it is
  broken. This must be stated before they press play.

---

## What the two simulations should teach

The framing to write to, stated as the campaign owner put it:

| | Planning simulation | Logging simulation |
|---|---|---|
| **Teaches** | the planning component | what a real drive looks like |
| **Where the world comes from** | a kinematic model; you place the vehicle and the obstacles | recorded real sensor data |
| **Pose** | given — you set it, it is true by construction | **estimated** from LiDAR against the map |
| **Objects** | dummy cars you place | detected by perception, imperfectly |
| **What breaks** | routes, behaviors, trajectories | initialisation, scan matching, timing |
| **Needs** | lanelet2 map only | map, PCD, rosbag, and a GPU by default |
| **Runs on a laptop** | yes, no GPU | yes, with `pose_source:=ndt` |

The pedagogical arc between them is: *first learn what the vehicle decides, with
perception and localization removed; then put the real world back and watch
those two become the hard part.*

That arc should be stated explicitly at the top of the simulation chapter, and
each page should end by naming what the next one adds.

---

## Proposed organization

Six parts. The reordering is driven by one rule: **a reader with no vehicle and
no ROS 2 background should never have to scroll past something they cannot use.**

Every existing page is accounted for below. `N` = new, `R` = rewritten,
`M` = moved unchanged, `T` = trimmed, `—` = unchanged in place.

### Part 1 — Introduction

| | Page | Purpose |
|---|------|---------|
| R | `index.md` | What AutoSDV is. Two entry paths. Ends with a picture of a car driving itself in RViz, so the payoff is visible before the install. |
| — | `platform-models.md` | The hardware variants. |

### Part 1b — Working with ROS 2 in AutoSDV

**Not a ROS 2 tutorial.** There are many, they are better than anything this
book would write, and the page says so and links to them. What this section
covers is the subset AutoSDV actually leans on, plus the things that are
*different here* — which is where a newcomer following a generic tutorial gets
stranded.

| | Page | Purpose |
|---|------|---------|
| **N** | `concepts/environment.md` | **The source chain — the most important page in the book.** See below. |
| **N** | `concepts/launch-files.md` | Launch files, because everything in this project is one. XML/YAML/Python forms, `arg` vs `param`, `include`, substitutions (`$(var)`, `$(find-pkg-share)`, `$(env)`), and how an argument on the command line reaches a node's parameter. Preset files are launch files, and this is why that works. |
| **N** | `concepts/inspecting.md` | The daily debugging loop: `ros2 topic list`, `ros2 topic info -v` for **QoS on both sides**, `ros2 topic hz` for **frame rate**, `ros2 node info`, `ros2 param get`. Written around the two questions actually asked here — "is it publishing at the rate it should" and "why is my subscriber getting nothing". |
| **N** | `concepts/autoware-conventions.md` | **Autoware topic naming is the architecture.** The first path segment names the pipeline stage, so the namespace tree *is* the diagram. Plus the pipeline itself, as one Mermaid diagram. |
| **N** | `concepts/glossary.md` | NDT, MCL, PCD, lanelet2, TF, pose, twist, odometry, overlay/underlay, QoS, composable node, component container, rosbag, deskew, MRM, NVTL. One line each, linked from first use. |

#### Why `concepts/environment.md` leads

This is the page the current book is missing most, and the one whose absence the
`.envrc` actively conceals.

AutoSDV's installation **is not the official Autoware installation**, and a
newcomer who has read the Autoware docs arrives with wrong expectations:

- Official Autoware is built from source with `vcs import` and a long colcon
  build. **AutoSDV installs Autoware as Debian packages** into
  `/opt/autoware/1.5.0`, because a source build on an Orin costs hours that a
  student or a vehicle integrator should not pay.
- But **AutoSDV itself is still compiled** — `src/` is a colcon workspace and
  `just build` builds it. So the reader is in a hybrid: a binary Autoware
  underneath, a source workspace on top.

The consequence is a two-line chain that every terminal needs, and whose layers
provide different things:

```bash
source /opt/autoware/1.5.0/setup.bash   # ROS 2 Humble AND the Autoware packages
source install/setup.bash               # the AutoSDV workspace, built from src/
```

The first line is not just Autoware: `/opt/autoware/1.5.0/setup.bash` sources
`/opt/ros/humble/setup.bash` itself, then its own `local_setup.bash`. So it is
one command that establishes two layers, which is exactly the sort of thing a
reader cannot infer and will not guess.

**This is where dependency resolution comes from, and that is the stumble.**
A newcomer who sources only `install/setup.bash` gets a workspace whose packages
cannot find the Autoware messages and nodes they depend on, and the errors point
at AutoSDV rather than at the missing layer underneath. A newcomer who sources
only ROS 2 gets the same class of failure one layer up. The page must state
plainly: **the ROS 2 and Autoware dependencies your packages need come from that
chain, not from the build.**

The page then covers: what an overlay is, why ordering matters, why each new
terminal needs it again, and how to tell which layers are active
(`echo $AMENT_PREFIX_PATH`, `ros2 pkg prefix <pkg>`).

**Only then** does it introduce `direnv` and the `.envrc` — as a convenience that
*performs exactly those two lines for you*, with the file printed so the reader
sees it is nothing more. The same treatment applies to `just`: shown after the
raw command it wraps, never instead of it.

### Part 2 — Install

| | Page | Purpose |
|---|------|---------|
| R | `install/overview.md` | As rewritten in roadmap 7, **plus a new section up front: "what this installation is, and how it differs from Autoware's."** Debian Autoware vs a source build, what that buys (hours on an Orin) and what it costs (you take the pinned version), and the fact that AutoSDV itself still compiles. A "what you are working toward" picture at the top. |
| **T** | `install/verify.md` | Trimmed to checks 1–3. Check 4 links into the Tutorial instead of duplicating it. Gains an explicit environment check: source the chain in a **fresh** terminal and confirm `ros2 pkg prefix autoware_launch` resolves — the cheapest proof the underlay is really there. |
| — | `install/manual-environment.md` | Already explains the source-build alternative; now cross-links to `concepts/environment.md` rather than restating it. |
| — | `install/zed-sdk.md` | |
| — | `install/docker.md` | Unmaintained notice. |

### Part 3 — Tutorial: drive in simulation

**The heart of the book.** A numbered sequence, read in order, each page ending
by naming what the next one adds. Numbered filenames because the order *is* the
content.

| | Page | Purpose |
|---|------|---------|
| **N** | `tutorial/00-what-you-will-build.md` | The two simulations contrasted in one table, with the end-state pictures. What each teaches, what hardware each needs, how long each takes. Sets the arc: *learn what the vehicle decides, then put the real world back.* |
| **N** | `tutorial/01-first-run.md` | **`just demo check` then `just demo run`.** One command that works, end to end, including fetching data. Watch it drive. Read the metrics. No explanation yet — confidence first, understanding second. States plainly that this recipe is a wrapper and that step 4 opens it. |
| **R** | `tutorial/02-planning-simulation.md` | **Teaches the planning component.** No sensors, no localization, no perception: the pose is *given*, so nothing can be wrong except the decisions. Route vs trajectory. Behavior at stop lines. Place a dummy obstacle and watch the trajectory bend. Engage with AUTO. Screenshots 1–7. |
| **R** | `tutorial/03-logging-simulation.md` | **The closest thing to a real drive.** Real recorded sensor data. Pose is now *estimated*, and that is the whole difference — initialisation, convergence, and what a failing scan match looks like. Perception boxes appear on real clouds. States the 115.7 s parked prefix up front. Screenshots 8–10. |
| **N** | `tutorial/04-behind-the-recipe.md` | Takes `just demo run` apart, **downwards**: the `just` recipe → the `play_launch` command → the `ros2 launch` equivalent → the two `source` lines under both. The manual two-terminal version and its `--clock` requirement. How to see what an argument did with `play_launch resolve`. **This is where launching is taught**, which is what frees `usage.md` to be a reference. |
| **N** | `tutorial/06-play-launch.md` | **`play_launch`, honestly.** See below. |
| **N** | `tutorial/05-map-and-rosbag.md` | What is actually in `data/COSS-map-planning` — lanelet2 for planning, PCD for NDT, occupancy grid for MCL, one directory serving three consumers — and what is in the COSS bag: which sensors, how long, what the vehicle does. Closes the loop on why the same path appeared in steps 2 and 3. |

#### `tutorial/06-play-launch.md`, and not overselling our own tool

The current book tells the reader to use `play_launch` and gives the reasons it
is better. That is half the story, and the missing half matters more to a
newcomer than to us.

`play_launch` is **our own software**. `ros2 launch` is the reference
implementation that everything else in the ROS 2 world is written and tested
against. When a launch file misbehaves under `play_launch`, the first question
is not "what is wrong with the launch file" but "is this us".

The page therefore says all of:

- what it adds that matters here — process-group shutdown that does not orphan
  component containers, the web UI, per-process resource and `/diagnostics`
  monitoring, and `resolve`/`dump`/`up`
- that it is a drop-in for the same `<package> <file> arg:=value` line
- **that it can differ**, and that the project's own flags admit where:
  `--parser python` exists "for maximum compatibility" because the default Rust
  parser is a reimplementation of the launch-file language, and
  `--container-mode stock` exists to stop overriding composable-node containers
  at all
- the fallback ladder, in order:

  ```bash
  play_launch launch <pkg> <file> …                        # default
  play_launch launch <pkg> <file> --parser python          # if the file parses wrong
  play_launch launch <pkg> <file> --container-mode stock   # if composable nodes misbehave
  ros2 launch <pkg> <file> …                               # the reference; always available
  ```

- **how to tell which one is at fault**: if it works under `ros2 launch` and not
  under `play_launch`, that is a `play_launch` bug and belongs in its issue
  tracker, not in an AutoSDV one
- and, when falling back to `ros2 launch`, the kill-by-process-group recipe,
  because that is the thing `play_launch` was doing for you

A reader who knows the escape hatch will use our tool with more confidence, not
less. A reader who does not know it will conclude AutoSDV is broken.

### Part 4 — Operate

Reference and configuration. Read when you have a question, not in order.

| | Page | Purpose |
|---|------|---------|
| **M+T** | `operate/launching.md` | Was `getting-started/usage.md`. Keeps the full ~45-argument table and the everyday `just` commands; the teaching moves to tutorial 04 and 06, so this becomes the page you look things up in. Each `just` recipe is listed **with the command it wraps**, so the convenience layer never hides the real one. |
| M | `operate/localization-methods.md` | From `guides/`. |
| M | `operate/presets.md` | From `guides/`. |
| M | `operate/maps.md` | From `guides/`. |
| M | `operate/cuda-pipeline.md` | From `guides/`. |
| M | `operate/datasets.md` | From `simulation/`. Recording your own bags is an operating task, not a tutorial step. |
| **M+R** | `operate/scenarios.md` | Was `simulation/coss-park-scenario.md`. Reframed as what it is: the regression and benchmarking workflow — `demo compare`, `demo bench`, the run variants. Stops being a tutorial page competing with tutorial 01. |

### Part 5 — Build a vehicle

Everything hardware, after the software a reader can use without one.

| | Page | Purpose |
|---|------|---------|
| M | `vehicle/hardware-assembly.md` | From `getting-started/`. |
| — | `vehicle/sensor-integration/*` | The existing eight pages, unchanged. |
| — | `vehicle/vehicle-control/*` | The existing four pages, unchanged. |

### Part 6 — Reference and development

| | Page | Purpose |
|---|------|---------|
| — | `reference/overview.md` | |
| — | `reference/hardware/core-components.md`, `wiring-diagrams.md` | |
| — | `reference/networking/5g-deployment.md` | |
| — | `reference/software/vehicle-interface.md` | |
| R | `develop/index.md` | Was `guides/development.md`. Index of the development pages, with the two submodule habits. |
| — | `develop/source-code.md`, `develop/version-control.md` | From `guides/`. |

### The nav

```yaml
nav:
  - Introduction:
    - index.md
    - Vehicle Build Variants: platform-models.md
    - Concepts:
      - ROS 2 in Ten Minutes: concepts/ros2.md
      - The Autoware Pipeline: concepts/autoware.md
      - Glossary: concepts/glossary.md
  - Install:
    - Installation: install/overview.md
    - Verifying the Installation: install/verify.md
    - Manual Environment Setup: install/manual-environment.md
    - ZED SDK Installation: install/zed-sdk.md
    - Docker (Unmaintained): install/docker.md
  - Tutorial:
    - What You Will Build: tutorial/00-what-you-will-build.md
    - 1. First Run: tutorial/01-first-run.md
    - 2. Planning Simulation: tutorial/02-planning-simulation.md
    - 3. Logging Simulation: tutorial/03-logging-simulation.md
    - 4. Behind the Recipe: tutorial/04-behind-the-recipe.md
    - 5. The Map and the Rosbag: tutorial/05-map-and-rosbag.md
  - Operate:
    - Launching the System: operate/launching.md
    - Localization Methods: operate/localization-methods.md
    - Presets: operate/presets.md
    - Maps: operate/maps.md
    - CUDA Point Cloud Pipeline: operate/cuda-pipeline.md
    - Datasets & Rosbags: operate/datasets.md
    - Scenarios & Benchmarking: operate/scenarios.md
  - Build a Vehicle:
    - Hardware Assembly: vehicle/hardware-assembly.md
    - Sensor Integration: [ the existing eight pages ]
    - Vehicle Control: [ the existing four pages ]
  - Reference:
    - Overview: reference/overview.md
    - Hardware: [ core components, wiring ]
    - Software: [ vehicle interface ]
    - Networking: [ 5G ]
    - Development: [ index, source code, version control ]
```

### Counting the work

| | Pages |
|---|---|
| New | 12 |
| Rewritten | 4 |
| Trimmed | 2 |
| Moved, content unchanged | 14 |
| Unchanged in place | 12 |

The growth over the first draft is all in Part 1b and the `play_launch` page —
that is, in the material that explains what is *different* about this project,
which is the part no external tutorial supplies.

Each new or rewritten page is two files, plus a `nav_translations` entry. The
moves are cheap in effort and expensive in links: every relative link between
pages changes depth, and the zh-TW image paths carry their extra `../`.

### Do the moves now, or not at all

Moving pages changes their published URLs. The live site is `book-v0.2.1` and is
already months stale, and `book-v0.3.0` is not tagged — so **this is the cheapest
moment this restructure will ever have.** After a release that people link to,
it costs redirects.

If the moves are judged too disruptive, the tutorial can be built without them:
add `concepts/` and `tutorial/`, leave everything else where it is, and accept a
nav that reads well but whose paths do not match it. The Part 4–6 moves are the
optional half.

## Pictures needed

Minimum set for the tutorial to work. All are software screenshots; none exist
today.

**Planning simulation**

1. RViz on first launch — map loaded, no vehicle, toolbar visible
2. The toolbar, cropped, with **2D Pose Estimate** called out
3. Vehicle placed after setting the initial pose
4. Route drawn after setting a goal, trajectory visible
5. AutowareStatePanel, cropped, with **AUTO** called out
6. Vehicle driving the route
7. A dummy obstacle placed, and the trajectory bending around it

**Logging simulation**

8. The stack up, before the bag plays — map, no scan
9. Point cloud aligned on the map while localizing (the success picture)
10. A failing scan match — cloud beside the map (the failure picture, which is
    more useful than the success one)
11. `just demo run` terminal output with the metrics summary
12. The `play_launch` web UI node list

**Concepts**

13. The Autoware pipeline diagram — drawn, not a screenshot. Mermaid, so it
    stays editable and themes correctly.

### How to capture them

This is a real blocker and needs a decision:

**A TurboVNC server is already running on this machine**: display `:1`,
1240x900, depth 24 (`/opt/TurboVNC/bin/Xvnc :1`, rfbport 5901). ImageMagick
`import` is installed. So capture needs no new software:

```bash
DISPLAY=:1 play_launch launch ...          # render into the VNC session
DISPLAY=:1 import -window root shot.png    # capture the whole screen
```

`vncserver` is available to start a further display at a larger geometry if
1240x900 proves cramped for RViz — a 1920x1080 session would give the
screenshots room, and the ZED tooling needs VirtualGL in a VNC session anyway
(the `turbovnc-virtualgl` setup step).

Either way the machine has everything else: the workspace is built, the COSS map
is in `data/COSS-map-planning`, the rosbag is in
`data/rosbags/outdoor_20251226_153115`, and the GPU is an RTX 5090.

Store them under `src/figures/simulation/`, and remember the zh-TW pages need
one extra `../` in every image path.

---

## Verifying the demos actually work

Before writing a tutorial around them, run them and record what happens:

- [ ] `just demo check` on this machine
- [ ] `just demo run` end to end; keep the metrics output
- [ ] `just sim planning`, drive to a goal
- [ ] `just sim logging` with a manual bag play, to confirm the two-terminal
      sequence the book describes
- [ ] the CPU-only path: `pose_source:=ndt launch_perception:=false`, which the
      book claims works and which nothing has tested
- [ ] time each, so the tutorial can say how long to wait instead of "a while"

The last two matter most: the CPU-only claim is currently an assertion, and the
workshop in roadmap 8 depends on it being true.

---

## Roadmap

Six phases. **C is unblocked and can start today**; A gates D; the Part 4-6 moves
in E need a decision first.

### Phase A — Establish ground truth

Nothing in the tutorial should be written from memory. Run everything, record
what actually happens, and time it, so the pages can say "wait about 40 seconds"
instead of "wait a while".

| | Task | Why |
|---|------|-----|
| A1 | `just demo check`, then `just demo run` end to end | the tutorial's step 1; keep the metrics output verbatim |
| A2 | `just sim planning`, set a pose, set a goal, engage | time from launch to RViz usable |
| A3 | `just sim logging` + a manual `ros2 bag play --clock` in a second terminal | confirms the two-terminal sequence the book describes, and that `--clock` is the failure everyone hits |
| A4 | **`pose_source:=ndt launch_perception:=false`** on CPU | currently an **untested assertion** in the book, and the roadmap 8 workshop depends on it |
| A5 | Record every wait: map load, scan matcher up, engine compile | replaces every vague "wait for it to come up" |

A4 is the one that can invalidate other work. If the CPU-only path does not
hold, both the laptop workshop and the book's "a GPU is not needed to start"
claim need rewriting.

### Phase B — Pictures (needs A)

| | Task |
|---|------|
| B1 | Pick the display. TurboVNC `:1` is already running at 1240x900 — workable, but cramped for RViz; `vncserver` can start a 1920x1080 session that gives the screenshots room |
| B2 | Capture the 13 shots listed above with `DISPLAY=:N import -window root` |
| B3 | Place under `src/figures/simulation/`; remember zh-TW needs one extra `../` |

### Phase C — Part 1b, the concepts (unblocked — start now)

This is the half of the book no external tutorial supplies, and none of it needs
a running demo.

| | Page | Note |
|---|------|------|
| C1 | `concepts/environment.md` | **Write this first.** The deb-vs-source difference, the two-line chain, and the fact that dependency resolution comes from it |
| C2 | `concepts/launch-files.md` | |
| C3 | `concepts/inspecting.md` | QoS, `topic hz`, the daily loop |
| C4 | `concepts/autoware-conventions.md` | topic namespace as architecture, plus the pipeline diagram |
| C5 | `concepts/glossary.md` | |

### Phase D — The tutorial chapter (needs A and B)

Seven pages, `tutorial/00` through `tutorial/06`, written against the recorded
timings from A and the screenshots from B.

### Phase E — Restructure and navigation (needs a decision)

| | Task |
|---|------|
| E1 | **Decide the moves.** Parts 4-6 relocate pages and change published URLs |
| E2 | Move the pages; fix every relative link whose depth changed |
| E3 | Rewrite `nav` and `nav_translations` |

### Phase F — Translate, verify, release

| | Task |
|---|------|
| F1 | zh-TW for every new and changed page |
| F2 | `just lint`; `mkdocs build --strict` |
| F3 | Tag `book-v0.3.0`, which deploys and moves the website submodule |

---

## Open decisions

1. **The Part 4-6 moves.** They change published URLs. The live site is
   `book-v0.2.1`, already months stale, and `book-v0.3.0` is untagged — this is
   the cheapest moment the restructure will ever have. Declining them still
   leaves a working book: add `concepts/` and `tutorial/`, leave the rest in
   place, and accept paths that no longer match the nav.
2. **Screenshot display.** Reuse TurboVNC `:1` at 1240x900, or start a
   1920x1080 session.
3. **`data/COSS-map-planning` is misnamed** — it holds the PCD and the occupancy
   grid too. Rename in the repository, or explain it in the book and leave it.
4. Two items still open from roadmap 7: the project version on `develop`
   (`0.2.0` with a separate `prerelease: dev`, rather than a composed
   `0.2.0-dev`), and the deliberately empty amd64 CUDA/cuDNN/TensorRT triple in
   `versions.yaml`.
