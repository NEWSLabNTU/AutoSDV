# Book: chapter architecture, and the usage guides nobody wrote

**Goal**: give the book an architecture a reader can walk — install once, choose
how to run, then follow the stack in the order it comes up — and fill the gaps
that flat list of "Guides" was hiding. Eight features exist in the tree and are
absent, wrong, or unfindable in the book.

**Status**: **phase 0 done** — the chapters exist, every page is in its new
place, the build is clean and no page is orphaned. Phases 1-3 (the new pages,
reconciliation, audit) are next. This closes
[roadmap 10](10-book-tutorial-restructure.md)'s **Phase E1**, "decide the
moves", which has been the open item holding that campaign's E2/E3.

**Book repository**: `~/repos/AutoSDV-book`, branch `main`. Separate repository,
not a submodule. Every English page has a `.zh-TW.md` sibling, so a new page is
two files plus a `nav` entry plus a `nav_translations` entry.

**Relationship to the other book roadmaps**:

| Roadmap | What it owned | What this one changes |
|---|---|---|
| [8](8-book-revision.md) | installation and launch chapters, corrected | nothing; its pages move but keep their content |
| [10](10-book-tutorial-restructure.md) | the tutorial as a teaching sequence | supersedes its Part 1-6 sketch, which predates the reorg already on `main`; closes E1 |
| [12](12-student-container.md) | the workshop container image | supplies the container page this roadmap writes |

---

## Why the flat "Guides" chapter had to go

Nineteen pages sat in one list: `Version Control` beside `Localization
Methods`, `Source Code Walkthrough` beside `LiDAR Sensors`. Two different
readers — someone changing the project, someone running it on a vehicle — were
being handed the same undifferentiated pile, in alphabetical-ish order.

Ordering the usage half along the pipeline is not tidiness. **Bring-up has a
dependency order**: sensors publish before localization converges, localization
holds before planning is worth tuning, and planning produces a trajectory before
control has anything to follow. A reader who meets the chapters in that order is
being walked through the same sequence they will live through on the vehicle.

It also makes two holes visible that the flat list concealed: **there is no
perception page and no planning page at all.**

---

## The architecture

```
1  Get Started                     install mainline only, nothing optional
     index · What Machine You Need
     Software Installation: Overview · Recommended · Verifying

2  Tutorial                        learn the system in simulation (unchanged)
     What You Will Build · 1 First Run · 2 Planning Sim · 3 Logging Sim
     4 Behind the Recipe · 5 The Map and the Rosbag · 6 play_launch

3  Running the System              NEW CHAPTER: "installed -- now what"
     In a Container          NEW, replaces "Docker Setup (Unmaintained)"
     On the Vehicle          MOVE getting-started/usage.md
     In Simulation           MOVE simulation/{coss-park-scenario,datasets}.md

4  Building the Stack              usage guides, in pipeline order
     Presets                 MOVE (cross-cutting config, read first)
     4.1 Sensing
         Using Sensors · LiDAR (EXPAND) · Camera · IMU · GNSS · Troubleshooting
         CUDA Point Cloud Pipeline   MOVE
     4.2 Localization
         Localization Methods (EXPAND) · NDT Tuning (NEW)
         Maps (EXPAND) · Localization Diagnostics (NEW)
     4.3 Perception          slot, no page exists today
     4.4 Planning            slot, no page exists today
     4.5 Control and Safety
         Vehicle Control x4 · Minimum Risk Manoeuvre (NEW)

5  Concepts                        unchanged
     The Environment · Launch Files · Inspecting (EXPAND)
     The Autoware Pipeline · Glossary

6  The Vehicle (hardware)
     Vehicle Build Variants · Hardware Setup

7  Developer Guide                 audience split
     Development Guide · Source Code Walkthrough · Version Control
     Adding a Sensor         MOVE from Sensor Integration
     Integration Walkthrough MOVE from Sensor Integration

8  Reference
     Overview · Command Reference (NEW)
     Hardware: Core Components · Sensor Capability Matrix (NEW) · Wiring Diagrams
     Software: Vehicle Interface (EXPAND) · Diagnostic Scripts (NEW)
     Networking: 5G/LTE Deployment

9  Appendix                        the detours
     ZED SDK Installation    MOVE from the install mainline
     Manual Environment Setup MOVE
```

All 49 current pages are accounted for: 47 in place or moved, the stale Docker
page replaced, 5 new pages, 2 chapters created.

### The three judgements worth recording

**The ZED SDK is an appendix, not an install step.** It is one camera's manual
download, and on the mainline it reads as something every reader owes. The
`zed-sdk` setup step already tells the few who need it, at the moment they need
it, in bright text at the end of a run.

**"Run in a container" comes after installation, not inside it.** You install
once and then choose how to run: container, vehicle, or simulation. Filed under
Software Installation it read as a competing way to install.

**Adding a Sensor and the Integration Walkthrough are developer pages.** They
are "write a new integration", not "use the ones that exist", and they were
sitting in the middle of the sensor usage pages a vehicle operator reads.

---

## What is missing, and where it comes from

Established by comparing the tree's feature surface against the book, 2026-09-21.
`2-D MCL` and the engine cache were checked and are **already covered** — not
every in-tree feature is a gap.

| # | Gap | Today | Source in the tree | Lands in |
|---|---|---|---|---|
| 1 | NDT tuning: process, pitfalls, parameter table, the `max_iterations` incident | no page; zero hits for `max_iterations` | `docs/guides/ndt-tuning.md` (298 lines) | 4.2 NDT Tuning |
| 2 | Solid-state LiDAR: what a 120 deg FOV does to scan matching; no usable ring for MCL; CUDA deskew refuses `cube1` | scattered, and the FOV number conflicts | `docs/research/robin_w_fov.md`, CLAUDE.md deskew matrix | 4.1 LiDAR + Reference matrix |
| 3 | MRM (minimum risk manoeuvre) | **zero mentions**, a safety feature with no coverage | `docs/guides/mrm_configuration.md`, `mrm_troubleshooting.md` | 4.5 MRM |
| 4 | Building a 3-D map | `maps.md` builds occupancy grids only; where the PCD comes from is answered nowhere | `docs/guides/lio_sam_mapping.md`; golf-cart's GLIM tutorials | 4.2 Maps |
| 5 | Localization diagnostics: 15 scripts | `inspecting.md` teaches generic `ros2` only | `scripts/testing/localization/` + its README | 4.2 + Reference |
| 6 | `just` recipes and `just tool` | zero hits for the tools; no recipe reference anywhere | `justfile`, `just/*.just` | Reference: Command Reference |
| 8 | Workshop container | the page says "Unmaintained" -- **actively wrong**; `docker/desktop/` is active and a multi-arch `jerry73204/autosdv:base` is published | `docker/desktop/`, [roadmap 12](12-student-container.md) | 3 In a Container |
| 9 | Steering status has no feedback: it republishes the command, so MPC closes the lateral loop on its own output | reference page silent | `docs/reports/steering-status-has-no-feedback.md` | Reference: Vehicle Interface |

Deliberately **not** ported: CARLA simulation and the Leo Drive dataset tooling
(owner's call), and the 20 `docs/reports/2dlidar-phase*` studies, which are
research and belong in the repository.

---

## Phase 0 — The move, with no content changes

One agent, alone, before anything else. Moving pages and writing pages in the
same pass makes a review unable to tell a relocation from a rewrite.

| | Task |
|---|------|
| 0.1 | Rewrite `nav` and `nav_translations` to the architecture above |
| 0.2 | Move page files whose directory changes; fix every relative link whose depth changed |
| 0.3 | `just build` clean: zero broken anchors, no new warnings |
| 0.4 | Record the URL changes in this document, for the release note |

**Done, 2026-09-21.** Four pages moved; everything else was regrouped in `nav`
without moving, which spared 26 inbound links for no gain in honesty — a page
under `guides/` filed in a Developer chapter reads no worse for it, and the
appendix pages are still installation topics.

| was | is now |
|---|---|
| `getting-started/usage.md` | `running/on-the-vehicle.md` |
| `simulation/coss-park-scenario.md` | `running/coss-park-scenario.md` |
| `simulation/datasets.md` | `running/datasets.md` |
| `getting-started/installation/container.md` | `running/container.md` |

Links were rewritten in two directions, which is the part worth remembering:
links *to* a moved page (26 files), and the moved pages' own outbound links,
whose depth changed under them (`getting-started/installation/` to `running/` is
two levels to one). The two simulation pages also link to each other and moved
together, so they needed the target mapping applied after the depth fix.

**W1 was overtaken while phase 0 ran.** `e42ef15 Document the container, which is
how most people should start` landed upstream mid-pass, writing the container
page this roadmap had queued as W1 — 259 lines plus its translation. Phase 0
therefore moves *that* page into the Running chapter, and the separate retired
`docker.md` (the Jetson Dockerfile that no longer builds) stays where it is,
filed under Appendix with its honest "(Unmaintained)" title. **Phase 1 is five
units, not six.**

The lesson is cheap to record and expensive to relearn: a move pass and a
content pass on the same page, in different sessions, conflict in a way git
resolves badly — it matched a renamed retired page against a new page of the
same name. Rebasing the move onto the new content was wrong; redoing the move
from the new base took two minutes and left no residue.

Verified: `just build` reports zero warnings and zero broken anchors,
`check-translations` reports 46 pages up to date with the three known
translated-diagram exceptions, and a nav-versus-files comparison finds 49 of
each with no orphan on either side.

The site is mike-versioned, so `0.1/` is frozen and keeps working whatever the
moves do. That is what makes this affordable.

---

## Phase 1 — The pages, in parallel

Six work units. **Each owns its files exclusively** — that is the whole
mechanism that lets them run at once, so the ownership column is not advisory.
Every unit writes the English page *and* its `.zh-TW.md` sibling, because a
missing sibling fails `just check` and blocks everyone's merge, not just its
author's.

| Unit | Writes | Owns exclusively | Source material |
|---|---|---|---|
| ~~W1~~ | ~~Container page~~ | — | **done upstream**, `e42ef15`; moved into the Running chapter by phase 0 |
| W2 | MRM guide | `src/guides/mrm.md` (+zh) | `docs/guides/mrm_configuration.md`, `mrm_troubleshooting.md` |
| W3 | NDT tuning + localization diagnostics | `src/guides/ndt-tuning.md`, `src/guides/localization-diagnostics.md` (+zh) | `docs/guides/ndt-tuning.md`, `scripts/testing/localization/README.md` |
| W4 | Solid-state LiDAR + capability matrix | `src/guides/sensor-integration/lidar.md`, `src/reference/hardware/sensor-capability-matrix.md` (+zh) | `docs/research/robin_w_fov.md`, CLAUDE.md, `pointcloud_preprocessor.launch.py` |
| W5 | Building a 3-D map | `src/guides/maps.md` (+zh) | `docs/guides/lio_sam_mapping.md`, golf-cart `docs/guides/glim/` |
| W6 | Command reference + vehicle interface | `src/reference/commands.md` (+zh), `src/reference/software/vehicle-interface.md` (+zh) | `justfile`, `just/*.just`, `docs/reports/steering-status-has-no-feedback.md` |

**Shared files nobody may touch in Phase 1**: `mkdocs.yml`, `src/index.md`,
`src/concepts/glossary.md`, `src/concepts/inspecting.md`,
`src/guides/localization-methods.md`. Every unit that wants a nav entry, a
glossary term, a cross-link or an `inspecting.md` section **files it as a note
in its own page's front matter comment**, and Phase 2 applies it. A parallel
edit to `mkdocs.yml` is the one conflict that cannot be auto-merged.

### Rules every unit follows

- **Claims come from the tree, not from memory.** Every command in a page is run
  before it ships, or marked as requiring hardware the author did not have.
- **Do not port research.** `robin_w_fov.md` proposes UWB beacons and AprilTag
  architectures this project never built; only what is true of the shipped stack
  goes in the book.
- **User-facing depth only.** Measured tables, methodology and incident reports
  stay in the repository and are linked, as
  `docs/reports/host-resource-measurements.md` already is.
- **CLI first, `just` as the shortcut** — the real command, then the recipe that
  wraps it.

---

## Phase 2 — Reconciliation

One agent, after all six units land. This is the pass that turns six pages into
one book.

| | Task |
|---|------|
| 2.1 | Apply every nav request: `nav` + `nav_translations` entries for the new pages |
| 2.2 | Apply every glossary and cross-link request the units filed |
| 2.3 | `inspecting.md`: the diagnostics and `just tool` sections, from W3's and W6's notes |
| 2.4 | `localization-methods.md`: cross-links to NDT Tuning, the sensor suitability note from W4 |
| 2.5 | De-duplicate. Six authors writing about one stack will have written the same paragraph more than once; the second copy becomes a link |
| 2.6 | One reading pass end to end, for tone and for order |

---

## Phase 3 — Audit

A different agent from the one that wrote or reconciled. Its job is to **fail
the work**, not to confirm it.

| | Check | How |
|---|---|---|
| 3.1 | Every command runs | execute them; a command needing a vehicle or a ZED is marked, not silently trusted |
| 3.2 | Every version, path and parameter name exists | grep the tree for each; `versions.yaml` is the authority for versions |
| 3.3 | No claim contradicts the tree | especially the deskew support matrix, `pose_source` options, and the `just` recipe names, which have all drifted in the book before |
| 3.4 | Links and anchors | `just build`, zero "does not contain an anchor" |
| 3.5 | Translation parity | `just check`: every page has a sibling, no structural drift beyond the three known translated diagrams |
| 3.6 | Deprecated spellings | `just sim planning` is deprecated for `just coss planning-sim`; a page that teaches the old one is a defect |
| 3.7 | Nothing user-facing carries lab-bench detail | measured tables belong in the repository |

---

## Open decisions

1. **The Robin-W vertical FOV conflicts**: the book's LiDAR page says
   120 x 25 deg, `docs/research/robin_w_fov.md` says 120 x 70 deg, and nothing
   in the tree settles it — no driver config or calibration file carries it.
   **W4 must not guess.** Measure it off a recorded cloud, or mark the row
   unverified.
2. **Perception and Planning are empty chapters.** Left as named slots rather
   than filled here: a perception page needs the preset matrix and the engine
   story, a planning page needs someone to say what this vehicle's planner is
   actually configured to do. Both are their own campaign.
3. **The hands-on walkthroughs** for NDT tuning and map building live inside
   4.2, not as Tutorial chapters 7-8. The Tutorial stays the
   simulation-learning path; the usage chapters carry their own walkthroughs.
   Revisit if 4.2 grows past what a reader will follow in one sitting.
4. **A release tag** is still held, per roadmap 8. This work lands before it.
