# Running the simulations without a GPU, and what RViz costs

Measured 2026-09-13, to decide whether a classroom container can run the book's
tutorial on laptops that have no NVIDIA GPU. This is the gate for
[roadmap 11](../roadmap/12-student-container.md); nothing else in that plan is
worth building if the answer here is no.

**The answer is yes**, but only after fixing a defect in `just sim planning` and
changing which RViz config the workshop uses. Both are recorded below.

## What was measured on

| | |
|---|---|
| CPU | AMD Ryzen 9 9950X, 16 cores / 32 threads |
| RAM | 60 GB |
| Display | VNC `:1`, 1920x1080 |
| GL | `llvmpipe (LLVM 15.0.7)` — **software rendering, already** |
| GPU available for comparison | RTX 5090, reached with `vglrun -d egl` |

The VNC display was already on llvmpipe rather than VirtualGL, which makes this
machine an accurate stand-in for a GPU-less laptop *for rendering*. It is not a
stand-in for one in CPU terms: 32 threads is four to eight times a student
laptop, so **every number here is an upper bound**. A 4-core run is still owed.

## Defect: `just sim planning` loaded no map at all

`just/sim.just` computed the repository root by walking up from
`justfile_directory()`:

```just
repo := justfile_directory() / '..'      # -> /home/aeon/repos   (outside the repo)
```

In a just **module**, `justfile_directory()` already returns the directory of the
root justfile, so the extra `..` left the repository entirely. `map_path` then
pointed at a `data/` that does not exist.

The failure was quiet. `play_launch` reported "33/34 nodes running" and the
recipe appeared to start:

| | before | after |
|---|---|---|
| `map_projection_loader` | **exited without code** | running |
| `/map/vector_map` publishers | **0** (with 19 subscribers waiting) | 1 |
| `/map/pointcloud_map` publishers | **0** | 1 |
| startup summary | 33/34 nodes, 69/70 composables | **34/34, 70/70** |

With no lanelet2 map there is no route graph, so a student cannot set a goal or
plan — the planning simulation is the tutorial's first verification step and it
could not have worked in a classroom.

Fixed upstream in `9bed531`, which deletes the `repo` variable outright and uses
the shell's own working directory:

```just
map_path:="$PWD/data/COSS-map-planning"
```

That is sound because the module already declares `set working-directory := '..'`,
so a recipe runs from the repository root however it was invoked. Removing the
variable removes the class of bug rather than correcting one instance of it.

`demo/justfile` already used the correct idiom (`module_directory() / ".."`), and
was not affected. `just/sim.just` was the only occurrence.

## RViz is bounded by the point cloud map, not by software rendering

Same scene (planning simulator on the COSS map), same window, one RViz at a time,
measured from RViz's own frame-rate readout and `pidstat`:

| RViz config | Renderer | fps | CPU |
|---|---|---|---|
| stock `autoware.rviz` | llvmpipe | **2** | 443 % |
| stock `autoware.rviz` | RTX 5090 | 10 | 99 % |
| `PointCloudMap` display off | llvmpipe | **31** | 161 % |

The expected result was that software rendering is too slow. It is not the
binding constraint. Drawing the 78 MB `pointcloud_map.pcd` every frame is: it
holds a **5090** to 10 fps, and turning it off makes *software* rendering three
times faster than a GPU carrying it.

2 fps is not usable for teaching. 31 fps is comfortable, and it is reached on a
machine with no GPU in play.

### Consequence for the workshop

Ship a workshop RViz config with the map point cloud disabled rather than
shipping the stock Autoware one. This is a file, not an architectural change.

The planning simulation loses nothing pedagogically: it has no lidar, the
lanelet2 map, lane centre lines, vehicle pose and the Autoware state panel all
still render, and the discarded layer is the decorative grey PCD texture.

The logging simulation is a different case — the scan aligning against the map
*is* the thing being taught — so it likely wants the PCD decimated rather than
removed. That has not been measured.

## The logging simulation, and a four-core laptop — measured 2026-09-14

The two items this report owed, on a second machine: Intel Core Ultra 7 270K
Plus, 24 cores, 125 GB, VNC `:1` on the same `llvmpipe` software renderer. Run
GPU-less with `CUDA_VISIBLE_DEVICES=""`, `pose_source:=ndt
launch_perception:=false`, the full 157-second COSS replay at 1x, pose seeded
8 s in, RViz on with the launch file's own default layout. The four-core run is
the identical command under `taskset -c 0-3`, bag player included.

| | 24 cores | **4 cores** |
|---|---|---|
| NDT pose rate | 9.884 Hz | **9.892 Hz** |
| RViz frame rate | 3 fps | **1 fps** |
| Peak memory (RSS across the stack) | 6.3 GiB | 6.5 GiB |
| Peak CPU used by the stack | 10.7 cores | 4.0 (the cap) |

**The pipeline passes; the viewer does not.** NDT held the sensor's full 10 Hz
on four cores — 9.89 Hz against a 10 Hz recording, which is the same figure the
24-core run produced. Localization initialised, and the live scan sat on the map
where it should. Nothing about the *simulation* needs more than four cores.

RViz is the part that does not survive: 3 fps with 24 cores and **1 fps with
four**, both with the map point cloud displayed. That is not a localization
failure — the matcher does not care whether anything is drawn — but at 1 fps a
student cannot see the thing the logging simulation exists to show.

### What the point cloud layer costs, exactly

The same run with a layout whose map-PCD display is off, everything else
identical: **31 fps**, NDT 10.015 Hz. So the map point cloud is the whole
difference between 3 fps and 31, confirming this report's earlier finding on the
planning side and answering the question it left open.

But the logging simulation is where the map matters most: watching the live scan
settle onto the map *is* the lesson, and a layout with the map hidden shows a
scan floating in nothing. Removing the layer is the wrong trade here, unlike in
the planning simulation. The options are to decimate it rather than hide it, or
to display the localization module's own
`/localization/pose_estimator/debug/loaded_pointcloud_map` — the radius the
matcher actually loaded — instead of the entire 4.9 M points. Neither is
measured yet.

### What this settles for the hardware requirement

Four cores is enough to *run* both simulations. It is not enough to *watch* the
logging one without a graphics path, which makes the container's renderer
detection load-bearing rather than a nicety: on Linux and WSL2 it reaches a GPU,
and only macOS is stuck on llvmpipe.

## `just ... ARGS="..."` silently passes nothing, and the launch uses defaults

Found by running the logging simulation in the desktop container, which has no
GPU and therefore no `libcuda.so.1`:

    thread 'main' panicked at cudarc-0.17.8/src/lib.rs:159:
    Unable to dynamically load the "cuda" shared library

That is `cuda_ndt_matcher`, after a launch given `pose_source:=ndt`.

**The cause is the documented `ARGS=` spelling, not play_launch.** In just,
`ARGS="..."` after a recipe name is a positional value, not a variable
assignment, so the recipe's parameter receives the literal text:

    just sim logging ARGS="pose_source:=ndt"
      ->  logging_simulation.launch.yaml ARGS=pose_source:=ndt

play_launch is handed `ARGS=pose_source:=ndt`, which is not a launch argument
it can use, so `pose_source` keeps its default of `cuda_ndt`. The correct
spelling is positional:

    just sim logging "pose_source:=ndt launch_perception:=false"

play_launch was checked and is not at fault. Dumping the real launch file with
`play_launch dump ... pose_source:=ndt` resolves `package:
autoware_ndt_scan_matcher` -- the CPU matcher, correctly. Nested `$(eval ...)`,
boolean rendering (`True`/`False`) and `if`/`unless` on groups were each tested
in isolation and are all correct.

**Where this is documented wrongly:** CLAUDE.md (two places), the book's
`getting-started/usage` and several sensor-integration pages, and
`docs/tech/2d-mcl-localization.typ`. Every one of them silently launches
defaults.

**Why it matters more than a typo.** On a machine with an NVIDIA driver the
wrong default still runs -- `cuda_ndt` loads and works -- so the mistake is
invisible and the CPU path appears to have been tested when it never was. On a
GPU-less laptop the same command panics with no localization at all.

**It invalidates the four-core NDT rate in the section above.** That run used
`just sim logging ARGS=...`, so it measured `cuda_ndt` on a GPU, not CPU NDT on
four cores. The CPU matcher's throughput is still unmeasured: a container run
with the correct spelling gave 2.9 Hz against a 10 Hz sensor, but the host was
at load average 39 on 32 cores with the NDT process at 42% of one core --
waiting, not computing -- so that figure measures contention, not the matcher.

## Method notes

- RViz reports its own frame rate in the bottom-right corner; that readout was
  captured rather than inferred.
- `rviz2` cannot load `autoware.rviz` standalone — it aborts with
  `Statically typed parameter 'wheel_radius' must be initialized`. It needs the
  vehicle-info params that `play_launch` injects; reuse the params file from
  `play_log/<run>/node/rviz2/params_files/`.
- `play_launch` supervises and **restarts** a killed `rviz2`, so an A/B needs the
  stack launched with `rviz:=false` and RViz run separately.
- `vglrun` alone produced nothing on this host; `vglrun -d egl` reached the GPU.
