# The desktop image

The workshop and tutorial environment: a student opens a browser and gets a
desktop with RViz and a terminal, with no ROS 2, no Autoware and no AutoSDV
build on their own laptop.

```bash
./docker/desktop/build.sh                        # amd64, on this machine
PLATFORM=linux/arm64 ./docker/desktop/build.sh   # arm64, ON THE ORIN
```

```bash
docker run -it --rm -p 6080:6080 \
  -v "$PWD/data:/opt/AutoSDV/data" \
  --shm-size=2gb --cap-add=NET_ADMIN \
  jerry73204/autosdv:desktop
# then open http://localhost:6080
```

**The `-v data` mount is not optional.** Maps and rosbags are deliberately kept
out of the image (`.dockerignore` excludes `data/`), so without it the planning
simulation comes up 33/34 with the map container half empty:

```
PCD load failed: /opt/AutoSDV/data/COSS-map-planning/pointcloud_map.pcd
Composable node '/map/lanelet2_map_loader' crashed: killed by signal 11
```

`lanelet2_map_loader` **segfaults** on a missing map rather than reporting one,
so the first line a student reads is a crash, not the cause. `compose.yaml`
carries the mount already; this plain `docker run` form is the one that needs it
spelled out.

## The two architectures are two platforms, not one image twice

This is the thing to understand before changing anything here.

| | amd64 | arm64 |
|---|---|---|
| Runs on | Windows, Intel Mac, Linux | **Apple Silicon** |
| Built on | any x86 machine | **a standard Ubuntu 22.04 arm64 server** |
| Base image | `nvidia/cuda:12.8.1-cudnn-devel-ubuntu22.04` | `ubuntu:22.04` |
| Autoware | `1.5.0-2ubuntu2204` | **`1.5.0-2jetpack62`** |
| CUDA / TensorRT | NVIDIA's x86 CUDA repository | **the Jetson repository** |

**The arm64 image is a Jetson-flavoured image that happens to run on Apple
Silicon.** It is not a generic-arm64 build, because there is no generic-arm64
Autoware: the only arm64 localrepo NVIDIA and this project publish is the
JetPack one. It works off a Jetson because JetPack 6.2 is Ubuntu 22.04
underneath and Apple Silicon is arm64.

Two consequences that are easy to get wrong:

- **Do not use `nvidia/cuda`'s arm64 tag as the BASE IMAGE.** It exists, and it
  will build. But it is the SBSA (server-ARM) platform, and pairing SBSA CUDA
  with JetPack Autoware packages mixes two platforms in one image. Plain Ubuntu
  plus the Jetson repository keeps it on one.

  **This is a rule about the base image, not about the build machine.** The
  arm64 image is built on a standard Ubuntu 22.04 SBSA server, and that is
  fine: SBSA is the host's platform, not the image's. Nothing from the host
  enters the image -- the base is `ubuntu:22.04` from Docker Hub and every
  Jetson-flavoured package is fetched from the Jetson repository inside the
  container. What is forbidden is putting SBSA *CUDA packages* in the image,
  and no step does that.
- **"Supported arm64 host" and "arm64 build machine" are different questions.**
  The only arm64 platform this project supports *running* AutoSDV on is the
  Orin. Building the image is unrelated: it takes nothing from the host except
  its instruction set, so an SBSA server produces the same image an Orin would,
  usually faster.

  A Jetson is arguably the *worse* builder. JetPack installations commonly set
  `nvidia` as Docker's default runtime, which injects the host's Tegra
  libraries into every container -- including the ones `docker build` runs. An
  image built that way can pick up JetPack libraries that are not in any of its
  layers, and it then fails on Apple Silicon, where they do not exist. A plain
  Ubuntu server has no such runtime and cannot leak anything. If you do build
  on a Jetson, check `docker info | grep -i runtime` first.

## Building the arm64 image

Built on **a standard Ubuntu 22.04 arm64 (SBSA) server** -- not on a Jetson,
and not on the x86 workstation. The reason is only speed: arm64 under qemu on
an x86 host compiles the entire ROS 2 workspace through emulation and takes
hours. `build.sh` warns when it detects that.

**The build host does not have to be, and here is not, a Jetson.** Everything
Jetson-flavoured about this image -- the `jetpack62` Autoware localrepo, the
Jetson apt repository for CUDA and TensorRT -- is fetched from the network
INSIDE a plain `ubuntu:22.04` container. The host contributes its instruction
set and nothing else, so an SBSA server and an Orin produce the same image.
Either way it targets Apple Silicon and is never run on a Jetson.

```bash
# on the arm64 server
cd ~/AutoSDV
git pull
PLATFORM=linux/arm64 ./docker/desktop/build.sh
./docker/desktop/publish.sh push
```

Being on SBSA does not make it an SBSA image: `container.desktop_base_arm64` is
`ubuntu:22.04`, and the rule against `nvidia/cuda`'s arm64 tag is about that
base-image choice, not about the machine doing the building.

## Publishing: one tag, every platform

A student types the same line on every machine they own:

```bash
docker pull jerry73204/autosdv:desktop
```

That tag is a **manifest list**, and the client resolves it by its own platform:

| Host | Docker asks for | Gets |
|---|---|---|
| Windows (Docker Desktop / WSL2) | `linux/amd64` | the amd64 image |
| Linux x86 | `linux/amd64` | the amd64 image |
| macOS on **Intel** | `linux/amd64` | the amd64 image |
| macOS on **Apple Silicon** | `linux/arm64` | the arm64 image |

Apple Intel needs nothing special -- it is an amd64 machine.

Each architecture is built and pushed natively on a machine of that
architecture, then the two are joined registry-side:

```bash
./docker/desktop/publish.sh push     # on the amd64 workstation
./docker/desktop/publish.sh push     # on the arm64 machine -- same command
./docker/desktop/publish.sh link     # on either, once both are pushed
./docker/desktop/publish.sh check    # what is published right now
```

`push` reads the architecture off the image itself and picks
`:desktop-amd64` or `:desktop-arm64` accordingly, so the operator types the
same thing in both places. Neither machine has to reach the other, or even be
up at the same time.

**Publish amd64 first and link it alone.** `link` writes a valid single-entry
index, so amd64 students can start straight away; re-running it when the arm64
build lands updates the same tag in place, and nobody changes what they type.

`publish.sh` refuses a flattened image. A single layer cannot download in
parallel or resume, so a pull that dies at 90% on shared wifi starts over --
see the flatten note in `build.sh`.

It uses `docker buildx imagetools create` rather than `docker manifest create`:
the former is GA where `docker manifest` is still experimental, works purely
registry-side by digest without pulling either image, and writes an OCI index
correctly.

The repository and tag live in `versions.yaml` (`container.desktop_repo`,
`container.desktop_tag`), not in the script.

## What is established, and how

Every row here was measured, not assumed. The design rests on them.

| Claim | Evidence |
|---|---|
| No NVIDIA **driver** is needed | `readelf -d` over every shipped library: **0 of 840** amd64 and **0 of 1053** arm64 link `libcuda.so.1` |
| Only the runtime is needed, and only by perception | 6 amd64 / 9 arm64 libraries link `libcudart`; all are perception |
| JetPack's arm64 CUDA installs on plain Ubuntu arm64 | probe against `repo.download.nvidia.com/jetson r36.4`: `cuda-cudart-12-6` pulls three config packages, `libnvinfer10` pulls **nothing** |
| TensorRT 10.3 matches what Autoware links | Autoware needs `libnvinfer.so.10`; the Jetson repo serves 10.3 |
| Only `zed_components` needs the ZED SDK | `zed_wrapper`, which the TF path xacros, builds without it |
| Software rendering is fast enough | 2 fps stock, **31 fps** with the map point cloud hidden; see [the report](../../docs/reports/gpu-less-simulation-and-rviz.md) |

## Design decisions worth not re-litigating

**It runs `setup.sh`, not a parallel install list.** A separate list would drift
from the one the book teaches, and the container's value is that it is a clean
machine running the documented procedure. Two steps are skipped because a
container cannot do them: `cyclonedds-sysctl` (`net.core.rmem_max` is not
namespaced) and `multicast-lo` (no systemd). The entrypoint checks the first
and sets the second.

**Nothing is patched around in the Dockerfile.** When the image fails because
a setup step is broken on a clean machine, the fix goes in the step, not here.
There were two `COLCON_IGNORE` workarounds during development and both were
removed once the underlying packages learned to skip themselves. A package
added here to get the build moving hides the same gap on a student's laptop,
which is the machine this image exists to stand in for.

**The workspace ships prebuilt.** A student reaches a running simulation in
minutes rather than watching colcon for forty, and a laptop that would run out
of memory while linking never has to.

**Graphics are decided at run time.** The container always runs TurboVNC and
serves it over noVNC, so the run command is identical everywhere; the
entrypoint picks the fastest renderer the host exposes (Mesa d3d12 on WSL2,
VirtualGL on a Linux GPU, software otherwise) and says which it chose. macOS
always lands on software: Hypervisor.framework exposes no vGPU to a container
and no flag changes that.

**`Dockerfile.dockerignore` excludes `setup/.markers`.** `setup.sh` imports
those as "already installed", so a build that copied a developer's markers
would skip ROS 2 and Autoware entirely and produce a broken image with no
error. The Dockerfile deletes them too, because a mistake there is silent.

## State of play, and what the amd64 build taught

The amd64 image has not completed a build yet. That is not a warning sign
about the design -- every failure so far has been a real defect in the
clean-install path, fixed at source, and the build reaches further each time:

| Attempt failed on | Fixed by |
|---|---|
| host `setup/.markers` copied into the image | `Dockerfile.dockerignore` + a defensive `rm` |
| `range-libc`: no Cython | the step installs `cython3` and `python3-dev` |
| `cyclonedds-sysctl` ran despite `--skip` | `--skip` accumulates (`action="extend"`) |
| `blickfeld_driver`: SDK hard-required | skips itself |
| `zed_components`: SDK and CUDA hard-required | skips itself |
| `autoware-debian`: no `libnvinfer10` | the `tensorrt` step |
| `rosdep update` fetch failure | three retries |

**Expect the same shape on arm64.** Every one of these was a package or step
assuming something the developer's machine happened to provide. The Orin has
JetPack, so it will provide things an Apple Silicon container will not -- which
is the same trap in a new place.

### Two mistakes worth not repeating

Both of mine, and both cost a build cycle.

**A skip that fails in its own right.** Twice, a package taught to skip itself
failed while skipping:

- `zed_components` called `ament_package()` at a point above where the file
  finds `ament_cmake`, giving `Unknown CMake command "ament_package"`
- `blickfeld_driver` used `find_package(... QUIET full)`, but a bare word is
  only a component after `REQUIRED` or `COMPONENTS`, giving
  `find_package called with invalid argument "full"`

In both cases the skip's warning still printed, so the log read as though the
skip had worked. Check the exit status, not the message.

**Testing the skip on a machine that has the dependency.** This workstation has
the ZED SDK, CUDA, Cython and TensorRT -- every dependency whose absence broke
something. A local `cmake` run took the normal path and reported success,
proving nothing. Verify a skip in a container that genuinely lacks the thing,
or in an isolated CMake snippet.

## Size

The first working image was **34.8 GB**, which is not a number a student pulls.
Measured from inside it, most of that was not the workspace:

| What | Size | Why it was there |
|---|---|---|
| Autoware localrepo pool | 1.9 GB | the 333 debs, still unpacked after being installed |
| the localrepo `.deb` | 1.9 GB | the download it was unpacked from -- a third copy |
| `libnvinfer_builder_resource_win.so` | 1.9 GB | TensorRT's **Windows** engine-builder resource |
| CUDA static archives (`*.a`) | 3.6 GB | from the `devel` base; nothing loads them at run time |
| Nsight Compute | 1.1 GB | a profiler, from the same base |
| `~/.cargo/registry` + rustdoc | 1.3 GB | crate sources and documentation |

All are now deleted by the Dockerfile, and the first three in the same `RUN` that
created them -- **a deletion in a later layer reclaims nothing**, it only writes
a whiteout while the bytes stay in the image. The last three arrived in the base
image, whose layers this build does not own, so they need `FLATTEN=1`.

The Autoware localrepo is `apt-get purge`d rather than `rm`ed, so dpkg and apt
stay consistent; it takes its `sources.list` and preferences pin with it, and
nothing cascades (verified: 0 packages removed, 333 Autoware debs still
installed, `ros2 pkg list` unchanged).

**`build/` and `src/` stay**, however tempting 1.5 GB is: `just build` passes
`--symlink-install`, so the 25 MB `install/` tree is 1071 symlinks pointing back
into them. Deleting either empties the workspace with no error at all.

### TensorRT was installed twice, and PyYAML is why

6.9 GB of the original image was two TensorRTs:

- **apt `libnvinfer10` 10.16.1.11+cuda13.2**, 2.5 GB, from `install-tensorrt.sh`.
- **`/opt/tensorrt/10.8.0`**, 4.4 GB, from the `tensorrt-runtime` step, to
  repair the engine-ABI mismatch the first install had just created.

The image's own apt history names the cause exactly:

```
Commandline: apt-get install -y --no-install-recommends \
             libnvinfer10 libnvinfer-plugin10 libnvonnxparsers10
```

**Bare package names.** `pinned_packages()` should have asked for
`libnvinfer10=10.8.0.43-1+cuda12.8`, and that version *is* in NVIDIA's
`ubuntu2204` repo -- all three packages offer it. The pin did not fail for want
of a candidate; it never ran.

`ENGINE_ABI` is read with `get-version.sh`, which parses `versions.yaml` using
**PyYAML**, and a clean machine has no PyYAML. In this image `python3-yaml`
arrived at **21:43**; the TensorRT step ran at **18:02**. So `get-version.sh`
failed, `2>/dev/null || true` swallowed it, `ENGINE_ABI` was empty, the pin
degraded silently to bare names, and apt took the newest.

The degradation was invisible: no warning, and an image that works, because the
second TensorRT that `tensorrt-runtime` installed shadows the wrong one through
`LD_LIBRARY_PATH`. The cost was 4.4 GB and a step whose only job was repairing
the previous step.

Both halves are now fixed:

- `install-tensorrt.sh` installs `python3-yaml` when it is missing rather than
  degrading past it, reads the version lazily so a machine that already has
  TensorRT still exits at tier 1 without an `apt-get update`, and prints a loud
  warning naming the consequence if it ends up unpinned anyway.
- `install-tensorrt-runtime.sh` skips when the **system** TensorRT is already the
  pinned version. It previously checked only whether `/opt/tensorrt/<version>`
  existed, so it would extract a second copy of libraries already installed.

Verified on the exact base image that failed: from a clean
`nvidia/cuda:12.8.1-cudnn-devel-ubuntu22.04` with no PyYAML, the pin now resolves
`10.8.0.43-1+cuda12.8` for all three packages.

Together these should remove the whole 6.9 GB duplication on the next build --
**a projection, since no image has been built with the fix yet.**

## The simulations use the stock `autoware.rviz`

Decided deliberately, against this repository's own measurements, so do not
"fix" it by pointing `just sim planning` at `workshop.rviz`.

`workshop.rviz` is the stock layout with the map point cloud display turned off.
It renders at 31 fps where the stock layout manages 2, and the gap is entirely
that one display -- it holds even an RTX 5090 to 10 fps. But hiding the map is
the wrong trade for teaching: watching the live scan settle onto the map is the
thing the logging simulation exists to show, and a layout with the map hidden
shows a scan floating in nothing.

The cost is real and lands on the student: on a laptop with no graphics path --
which on macOS is every laptop, since Hypervisor.framework exposes no vGPU --
the viewer runs at 2 fps. The pipeline itself is unaffected; NDT holds its full
10 Hz on four cores.

`workshop.rviz` stays in the tree for anyone who wants it, and the entrypoint
still mentions it when it falls back to software rendering. Nothing selects it
automatically.

## Known gaps

- **The arm64 image has never been built.** The arm64 branch of
  `install-tensorrt.sh` is written from a verified apt probe but has not run
  end to end. The Orin is the first real test.
- **The WSL `/dev/dxg` profile in `compose.yaml` is unverified** — written from
  Microsoft's documented requirements, never run on a Windows host.
- **No 4-core measurement.** Every performance number is from a 32-thread
  workstation, four to eight times a student laptop.
- **`libnvinfer10` is a 1.8 GB download** on a native install, 2.6 GB
  installed. An argument for the container over a native setup.
- **The post-cleanup size is a projection, not a measurement.** The cuts were
  measured by applying them inside the built image (33.1 GB -> 21.3 GB of disk),
  but no image has yet been built with the Dockerfile that performs them.
