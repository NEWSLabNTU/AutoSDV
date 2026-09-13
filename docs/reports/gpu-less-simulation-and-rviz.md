# Running the simulations without a GPU, and what RViz costs

Measured 2026-09-13, to decide whether a classroom container can run the book's
tutorial on laptops that have no NVIDIA GPU. This is the gate for
[roadmap 11](../roadmap/11-student-container.md); nothing else in that plan is
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

## Not measured

Both belong in the same report once done:

- **The logging simulation.** Bag replay with `pose_source:=ndt
  launch_perception:=false` adds live point clouds and CPU-side NDT, and is the
  harder case. Untested.
- **A realistic core count.** Everything above is on 32 threads. The key runs
  should be repeated under `taskset -c 0-3` before any hardware requirement is
  published.

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
