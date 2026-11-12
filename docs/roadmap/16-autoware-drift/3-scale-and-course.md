# Phase 3: Scale and course

**Status:** planned. **Needs:** phase 1 (submodule, regression gate, benchmark). Uses phase
2's specs as they arrive; car A's provisional spec until then.

[← Phase 2](2-car-facts.md) · [Overview](README.md) · [Phase 4 →](4-ros-nodes-in-simulation.md)

## Goal

The planner can plan for a 1/10 car on the COSS field: its full-size constants come from a
scale configuration, it honours a speed cap, and it reads a course built from AutoSDV's
lanelet2 map and GLIM scan. Proven offline: a plan around the COSS loop, plotted on the map.

## autoware-drift (generic; each change behind the regression gate)

- **3.1 Scale configuration.** One YAML, loaded by every module, in place of the ~40
  full-size literals (`axle.py` `SMOOTH` 50 N, `model.py` slip offsets 4.0 / 1.0 m/s,
  `car.py` wheel mass 20 kg, and the distances in `modes`, `multistart`, `live`). Defaults
  are today's values; `just regress check` passes unchanged.
- **3.2 `v_max`.** A planner parameter bounding vx in L1's window (`window.py`), the L0/QSS
  ceiling and `live.py`'s ceiling, and the tracker. Test: on a course where the
  unconstrained plan exceeds it, no planned or tracked speed does.
- **3.3 `Track.from_lanelet2(osm, lanelet_ids, grid=None, surfaces=None)`** and a course file
  reader/writer. Centreline resampled at 0.05 m, smoothed over ≈ 0.2 m, ψ and κ by central
  differences; each side's width the lesser of the lanelet bound and the grid's clearance,
  less a margin; z and grade from a ground surface; `mu` per lanelet `surface` class.
  Tests on a small synthetic lanelet2 map (a rounded rectangle) with a synthetic grid.
- **3.4 swb optional.** The planner imports and runs without swb; swb remains needed for
  the regression gate and the sim plant's full-size specs.
- **3.5 Differential.** If phase 2 found an open differential: grip-only planning accepts
  `diff: open` with the locked-axle approximation and a logged warning (drift styles refuse
  it); the open-differential model itself is phase 7's.

## AutoSDV

- **3.6 Small-car profile** for the scale configuration: Froude scaling from the wheelbase
  (0.319 / 2.886 ≈ 0.11) for lengths, × 0.33 for speeds and times, accelerations × 1; the
  horizon and re-plan period from phase 1's benchmark. Lives beside the per-car specs.
  The bundled drift_mustang data (`corner_library.npz`, `drift_map.json`,
  `drift_losses.json`) are switched off in it.
- **3.7 Surface classes** file: COSS paving's `mu`, `sliding` and peak slips from phase 2's
  skidpad (provisional literature values, tagged, until then).
- **3.8 The COSS course.** Choose the loop (the 495 m cycle or a shorter sub-loop; builder's
  call) and its start; `just race course` writes the course file from
  `data/COSS-map-planning` (lanelet2 map, the loop's ids, the occupancy grid, the PCD's
  ground fit); `just race check` checks it: closed, widths ≥ car width + margins, κ within
  the 20° lock, every station inside the grid's free space; it reports the narrow spots
  (COSS's p5 width is 0.86 m) and the 9 cm kerb step near (−12.6, −14.5).
- **3.9 Offline plan.** `just race plan` runs L0 and an L1 lap on the planner's own plant
  for car A's spec under a cap, and plots the line, speed and mode on the map.

## Exit

- autoware-drift's `main` has the scale configuration, `v_max`, `Track.from_lanelet2` and
  the course format; `just regress check` passes.
- The COSS course file passes `just race check`.
- An offline plan for car A at a cap of 1.5 m/s, plotted on the map, stays inside the
  course and under the cap; the plot is in the phase's report.
