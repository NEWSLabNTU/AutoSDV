# Book: Simulation-First Tutorials for Newcomers

**Goal**: Reorganize the AutoSDV book to put simulation first, making it
accessible to newcomers without physical vehicles. Add tutorial content
covering planning simulation, logging simulation, and full scenario replay.

**Status**: Done. The pages exist on `AutoSDV-book` `main` — this record was
briefly and wrongly marked "NOT DONE" on 2026-09-12; see the correction note
below.

!!! note "Correction, 2026-09-12"

    This file was edited to claim its work had never been done, on the strength
    of a `git log --all` that returned nothing for `src/simulation/*`. That
    search was run against a **clone that had never been fetched**, whose refs
    predated the work by months. `src/simulation/{planning-simulation,
    logging-simulation,coss-park-scenario,datasets}.md` and
    `installation/recommended.md` were all present on `origin/main` the whole
    time, added by `6ebde6f Add simulation tutorial` and
    `063f89d Extract the recommended installation step from overview.md`.

    The accusation is withdrawn. The lesson stands, but it lands on the
    accuser: **`git log --all` is not a search of the repository, it is a search
    of your refs.** Fetch first.

    Roadmap 9 has since rebuilt these pages as a numbered tutorial, and
    `planning-simulation.md`/`logging-simulation.md` are now superseded by
    `tutorial/02` and `tutorial/03`.

**Book Repository**: `~/repos/AutoSDV-book` (branch: `main` for develop)

---

## Summary

The AutoSDV book is currently organized around hardware deployment. Newcomers
who want to explore autonomous driving software without a vehicle have no
guided path. AutoSDV already has simulation infrastructure (`launch-sim-planning`,
`launch-sim-logging`, `sim-coss-park`) but the book doesn't document it.

This roadmap adds a simulation-first onboarding path following patterns from
F1TENTH (time-boxed tracks), Autoware (tiered simulation), and CARLA
(single-page first steps).

---

## Phase 5.1: Revise Landing Page

**Objective**: Revise the `index.md` landing page to direct newcomers to
simulation as a first experience, alongside the hardware path.

**Decision**: No separate "Start Here!" page. The existing `index.md` serves
as the entry page with concise links. Detailed tutorials will be added in
Phase 5.2–5.5.

### 5.1.1 Revise `index.md` landing page

Replace the old hardware-first getting started section with two clear paths:

- **"Try it in simulation"** — brief description, links to installation and
  simulation guides (no inline commands)
- **"Build and drive a vehicle"** — links to hardware setup, installation,
  and operation guides

### 5.1.2 Revise `index.zh-TW.md`

Chinese translation of the landing page changes.

**Success criteria**:
- [x] Landing page has simulation path and hardware path
- [x] No inline execution commands on the landing page
- [x] Chinese translation complete
- [x] Navigation unchanged (index.md remains the Get Started entry)

**Findings**: The landing page now serves as a clean index. Detailed
step-by-step content will go into Phase 5.2 (simulation tutorial) and the
installation pages. The `install-deps` recipe was merged into `download-data`
in the AutoSDV repo.

---

## Phase 5.2: Planning Simulation Guide

**Objective**: Create the Tier 1 simulation guide. This is the first hands-on
experience for newcomers — no sensors, no GPU, no hardware. Also serves as
the "Your First Simulation" entry point.

### 5.2.1 Create `src/simulation/planning-simulation.md`

Prerequisites: Completed [Recommended Installation](./installation/recommended.md).

Content outline:

1. **What it is**: Autoware planning simulator with AutoSDV vehicle model.
   Tests mission planning, behavior planning, motion planning, and control.
   No perception or localization involved.
2. **Launch**: `just sim planning`
3. **Open web UI**: Navigate to `http://localhost:8081`
4. **Set a goal**: Click "2D Goal Pose" → click on map → watch the vehicle
   plan and follow the route
5. **What you're seeing**: Brief explanation of the planning stack, the COSS
   Park map, the vehicle model, and control output
6. **Common scenarios**: Lane driving, U-turn, parking
7. **Adding dummy obstacles**: How to place static/dynamic obstacles
8. **Parameters**: Key planning parameters and how to tune them
9. **Troubleshooting**: Common issues (map not loading, vehicle not moving)
10. **Next steps**: Link to logging simulation, sensor guides

Include screenshots of the web UI with the planning simulator running.
Save to `src/figures/simulation/`.

### 5.2.2 Create `src/simulation/planning-simulation.zh-TW.md`

Chinese translation.

**Success criteria**:
- [x] Tutorial works end-to-end after recommended installation
- [x] All commands are copy-pasteable and tested
- [ ] Screenshots included (deferred — requires running system)
- [x] Common scenarios and parameters documented
- [x] Chinese translation complete

---

## Phase 5.3: Simulation Guide — Logging Simulation

**Objective**: Document rosbag replay simulation (Tier 2). Tests the full
localization and perception pipeline using recorded sensor data.

### 5.3.1 Create `src/simulation/logging-simulation.md`

Content:

- **What it is**: Replay recorded sensor data through the Autoware stack
- **What it tests**: Localization (NDT), perception (LiDAR detection),
  full pipeline minus hardware
- **Prerequisites**: Download test rosbag (`just bag download`)
- **How to launch**: `just sim logging` + rosbag playback
- **The outdoor test rosbag**: What sensors were recorded, map area,
  duration, expected behavior
- **Monitoring output**: PlotJuggler (`just tool plotjuggler`), RViz,
  web UI topics
- **Localization modes**: `pose_source:=ndt` vs others
- **Recording your own rosbags**: `just bag record`, topic selection
- **Troubleshooting**: NDT initialization, timing issues, missing topics

### 5.3.2 Create `src/simulation/logging-simulation.zh-TW.md`

Chinese translation.

### 5.3.3 Create `src/simulation/datasets.md`

Content:

- **Available datasets**: COSS Park outdoor rosbag, Leo Drive Bus-ODD
- **Downloading**: `just bag download` (auto-installs synology-dl if needed)
- **Rosbag management**: `just bag record`, `just bag play`
- **Dataset format**: ROS 2 bag format, topic list, sensor configuration
- **Using external datasets**: How to adapt Autoware-compatible bags

### 5.3.4 Create `src/simulation/datasets.zh-TW.md`

Chinese translation.

**Success criteria**:
- [x] Logging simulation walkthrough documented
- [x] Dataset download and playback documented
- [x] Chinese translations complete

---

## Phase 5.4: Simulation Guide — Full Scenario (COSS Park)

**Objective**: Document the full COSS Park simulation scenario (Tier 3),
which combines logging simulation with localization recording.

### 5.4.1 Create `src/simulation/coss-park-scenario.md`

Content:

- **What it is**: Full-stack simulation using COSS Park rosbag + map
- **What it tests**: Entire autonomy stack end-to-end
- **How to launch**: `just sim coss-park`
- **What happens**: Launch → bag playback → localization recording
  (all managed by GNU parallel)
- **Inspecting results**: PlotJuggler, recorded localization bags
- **Comparing runs**: Using `play_launch dump` for scope inspection
- **Using this for regression testing**: Before/after comparison workflow

### 5.4.2 Create `src/simulation/coss-park-scenario.zh-TW.md`

Chinese translation.

**Success criteria**:
- [x] Full scenario documented with expected output
- [x] Regression testing workflow explained
- [x] Chinese translation complete

---

## Phase 5.5: Navigation and Structural Updates

**Objective**: Update book navigation and structural elements to support the
simulation-first approach.

### 5.5.1 Split installation overview from recommended installation

Split `overview.md` into:

- `overview.md` — System requirements, installation methods table, OS
  preparation (Jetson/Ubuntu/Docker), ZED SDK note
- `recommended.md` — Clone → `./setup.sh` → direnv → `just build` →
  troubleshooting

"Software Installation" in the nav links to the overview page.

### 5.5.2 Merge `install-deps` into `download-data`

Remove the separate `just install-deps` recipe. The `download-test-rosbag.sh`
script now auto-installs `synology-dl` via `cargo install` if not found.

### 5.5.3 Update `mkdocs.yml` navigation

Add simulation guide section when Phase 5.3–5.5 pages are created:

```yaml
nav:
  - Get Started:
    - index.md
    - Vehicle Build Variants: platform-models.md
    - Hardware Setup: getting-started/hardware-assembly.md
    - Software Installation:
      - Overview: getting-started/installation/overview.md
      - Recommended Installation: getting-started/installation/recommended.md
      - Manual Environment Setup (Optional): getting-started/installation/manual-environment.md
      - Docker Setup (Optional): getting-started/installation/docker.md
      - ZED SDK Installation: getting-started/installation/zed-sdk.md
    - Operating the Vehicle: getting-started/usage.md
  - Simulation Guide:
    - Planning Simulation: simulation/planning-simulation.md
    - Logging Simulation: simulation/logging-simulation.md
    - COSS Park Scenario: simulation/coss-park-scenario.md
    - Datasets & Rosbags: simulation/datasets.md
  - Guides:
    - (existing content unchanged)
  - Reference:
    - (existing content unchanged)
```

### 5.5.4 Update nav_translations

Add Chinese translations for new navigation items.

**Success criteria**:
- [x] Installation split into overview + recommended (5.5.1)
- [x] `install-deps` merged into `download-data` (5.5.2)
- [x] Full navigation restructure with Simulation section (5.5.3)
- [x] nav_translations updated (5.5.4)

---

## Phase Summary

| Phase | Description                | Key Deliverable                          | Pages | Status |
|-------|----------------------------|------------------------------------------|-------|--------|
| 5.1   | Revise landing page        | Simulation path on index.md              | 2     | Done   |
| 5.2   | Planning Simulation guide  | Tier 1: first hands-on, no sensors       | 2     | Done   |
| 5.3   | Logging Simulation guide   | Tier 2: rosbag replay + datasets         | 4     | Done   |
| 5.4   | COSS Park Scenario guide   | Tier 3: full-stack simulation            | 2     | Done   |
| 5.5   | Navigation & structure     | Split install, nav restructure           | 2     | Done   |
