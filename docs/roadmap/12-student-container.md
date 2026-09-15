# A container image students can run on any laptop

**Goal**: a class of 50 students, on whatever laptops they own, reaches a
running AutoSDV simulation in minutes — with no ROS 2, no Autoware and no
AutoSDV build on their own machine.

**Status**: the image ships. Both architectures are built, verified and
published as one `:desktop` tag, and a student on any of the three platforms
reaches a running planning simulation from one `docker pull`. What is left is
documentation and teaching material — phases 4, 8 and 9.

**Feeds**: [roadmap 9](9-workshop-laptop-onboarding.md), whose two-hour timetable
this changes — see *Consequences* below.

---

## Why a container rather than instructions

Roadmap 9 assumed every student installs AutoSDV natively during class, and
listed "a prepared VM image" as an undecided fallback. The room makes that
assumption unworkable: it contains Windows, macOS and several Linux flavours,
and Apple Silicon cannot run Autoware natively at all.

A container collapses that to one artefact and one command.

## What was established before building anything

Each of these was measured, not assumed. Together they are why the design looks
the way it does.

| Question | Answer | Evidence |
|---|---|---|
| Does AutoSDV need an NVIDIA driver? | **No.** 0 of 840 amd64 and 0 of 1053 arm64 Autoware libraries link `libcuda.so.1` | `readelf -d` over every shipped `.so` |
| Then what does it need? | CUDA *runtime* and TensorRT, and only for perception, which the workshop does not launch | 6 amd64 / 9 arm64 libs link `libcudart` |
| Can Apple Silicon run this natively? | **Yes.** JetPack's arm64 CUDA debs install on plain `ubuntu:22.04` arm64, pulling no Tegra dependency | probe against `repo.download.nvidia.com/jetson r36.4` |
| Is the ZED SDK required? | Only by `zed_components`. `zed_wrapper`, which the tutorial needs for TF, builds without it | `find_package(ZED REQUIRED)` in one CMakeLists |
| Is software rendering fast enough? | **Yes, with the right RViz config**: 31 fps, versus 2 fps with the stock one | [gpu-less-simulation-and-rviz.md](../reports/gpu-less-simulation-and-rviz.md) |

The last one is the load-bearing result. macOS can never be accelerated —
Hypervisor.framework exposes no vGPU — so the unaccelerated floor had to be
good enough on its own, and it is.

## Shape

One multi-arch manifest, so students never choose a variant:

```
docker.io/jerry73204/autosdv:desktop
   ├── linux/amd64   Windows, Intel Mac, Linux
   └── linux/arm64   Apple Silicon
```

**The two are different platforms sharing a Dockerfile, not one image built
twice.** The arm64 side is a *Jetson-flavoured* image: Autoware comes from the
`jetpack62` localrepo, because that is the only arm64 Autoware build that
exists, and its CUDA and TensorRT therefore come from the Jetson repository to
match. It runs off a Jetson because JetPack 6.2 is Ubuntu 22.04 underneath and
Apple Silicon is arm64.

| | amd64 | arm64 |
|---|---|---|
| Target | Windows, Intel Mac, Linux | Apple Silicon |
| Built on | any x86 machine | **any arm64 Linux machine** |
| Base | `nvidia/cuda:12.8.1-cudnn-devel-ubuntu22.04` | `ubuntu:22.04` |
| Autoware | `1.5.0-2ubuntu2204` | `1.5.0-2jetpack62` |
| CUDA / TensorRT | NVIDIA x86 CUDA repo | Jetson repo (`r36.5`) |

Not `nvidia/cuda`'s arm64 tag: that is SBSA, the server-ARM platform, and
pairing SBSA CUDA with JetPack Autoware packages mixes two platforms in one
image. **Running AutoSDV on an arm64 host other than the Orin is not
supported** -- this exists so an Apple Silicon laptop can run the Jetson
container natively instead of through emulation. *Building* it is a separate
question, and the answer is any arm64 Linux machine: nothing Jetson-flavoured
touches the host.

Graphics are decided by the entrypoint, not by the instructions: the container
always runs its own X server (TurboVNC) reached over noVNC in a browser, and
picks the fastest renderer the host actually exposes.

| Host | GPU reachable in a container? | Path |
|---|---|---|
| Linux + NVIDIA | yes | VirtualGL |
| Linux + Intel/AMD | yes | VirtualGL on `/dev/dri` |
| Windows (WSL2) | yes | Mesa **d3d12** via `/dev/dxg` |
| **macOS** | **never** | software (llvmpipe) |

The workspace ships **prebuilt**. A student reaches a running simulation in
minutes rather than watching colcon for forty, and a laptop that would run out
of memory while linking never has to.

## Phases

| # | Phase | State |
|---|---|---|
| 0 | Repo prep: split `docker/` into `jetson/` and `desktop/`; Autoware `1.5.0-2`; `workshop.rviz` | done |
| 1 | **Gate:** GPU-less verification and RViz measurement | **done — passed** |
| 2 | amd64 image, headless | **done -- built, verified, published** |
| 3 | Graphics layer: TurboVNC + noVNC + renderer detection | **done -- verified on both architectures** |
| 4 | Accelerated profiles (`nvidia`, `dri`, `wsl`) | pending |
| 5 | arm64, built natively on an arm64 host | **done -- built, verified, published; 5 of 5 unknowns settled** |
| 6 | Multi-arch manifest to Docker Hub; `docker save` tarball fallback | **done -- `:desktop` resolves both platforms** |
| 7 | Logging simulation + a 4-core laptop proxy, folded into the Phase 1 report | **done — NDT holds 9.89 Hz on 4 cores; RViz is the limit, 1 fps** |
| 8 | Book page, EN + zh-TW | **done -- `getting-started/installation/container.md` + zh-TW** |
| 9 | Rewrite roadmap 9's timetable | **done -- replanned around no install; see [roadmap 9](9-workshop-laptop-onboarding.md)** |

## Consequences for roadmap 9

Its timetable is built around a 30-minute "ROS 2 concepts while it installs"
block. **That block no longer has an install to cover.** A `docker pull` done at
home the night before, plus `docker run`, is about five minutes.

This is a gain — the install was the session's biggest risk — but the two hours
have to be re-planned rather than trimmed. The ROS 2 material is still worth
teaching; it just needs to stand on its own rather than fill dead time.

## Phase 5: building the arm64 image

The build host is the **builder**, not the target. The image it produces runs on
Apple Silicon and is never run on a Jetson. Build it on an arm64 machine because
arm64 under qemu on an x86 host compiles the whole workspace through emulation
and takes hours; `build.sh` warns when it detects that.

**It does not have to be a Jetson**, and an ordinary arm64 server is much
faster. This was first attempted on an Orin over six runs and finished on an
80-core server in one.

```bash
# on any arm64 Linux machine
cd ~/AutoSDV && git pull
PLATFORM=linux/arm64 TAG=jerry73204/autosdv:desktop-arm64 ./docker/desktop/build.sh
```

Everything the build needs is already wired:

- `versions.yaml` carries `container.desktop_base_arm64`, and `build.sh`
  selects it from `PLATFORM`
- `install-autoware-debian.sh` already picks the `jetpack62` deb on `aarch64`
- `install-tensorrt.sh` has an arm64 branch that installs from the Jetson
  repository (`JETSON_REPO` overrides the `r36.5` default)
- `blickfeld_driver` and `zed_components` now skip themselves when their SDKs
  are absent, so the build does not need `COLCON_IGNORE`

### Result: the image is built, verified and published

`jetpack62` Autoware in a plain `ubuntu:22.04` arm64 image, **11.2 GB** against
amd64's 26.7 GB -- the difference is the CUDA `devel` base, which this side does
not have.

Built on an 80-core arm64 server rather than the Orin. Nothing about the image
needs a Jetson: everything Jetson-flavoured is installed INSIDE the container,
so the build host supplies only its instruction set, and a server does in
around 70 minutes what the Orin was taking most of a day to attempt.

Verified in the published image, on the `:desktop` tag a student actually pulls:

| check | result |
|---|---|
| planning simulation | **34/34** nodes, 15/15 containers, 70/70 composables |
| a second shell sees the stack | **138 nodes, 564 topics** |
| noVNC | serves; RViz draws the lanelet network and the map point cloud |
| middleware | `rmw_cyclonedds_cpp` |

The prediction below was right -- (1)-(3) each surfaced a defect of exactly the
kind the amd64 build surfaced, and each was fixed in the step rather than
worked around in the Dockerfile.

| # | What the arm64 run was testing | Outcome |
|---|---|---|
| 1 | Jetson repo serves TensorRT into plain `ubuntu:22.04` arm64 | **Proven.** `setup.sh`'s own `tensorrt` step does it; tier 3's aarch64 branch adds the Jetson repo and installs cleanly |
| 2 | `jetpack62` localrepo installs outside a Jetson rootfs | **Proven.** Every step of `./setup.sh --run --profile dev --yes` reaches "Setup complete." -- 1872s on the Orin, 2669s in the image on the server |
| 3 | The workspace compiles, particularly `cuda_ndt_matcher` | **Fixed, then proven at the package level.** It did not compile; see below. `colcon build --packages-select cuda_ndt_matcher` now passes on this Orin in both CUDA and no-CUDA modes |
| 4 | TurboVNC and noVNC start | **Proven.** Xvnc and websockify come up, noVNC answers 200, and RViz renders through llvmpipe |
| 5 | Autoware's TensorRT nodes load at all | **Fixed.** They could not -- see the two defects below, neither of which any earlier attempt lived long enough to reach |

**The three defects, and where each was fixed:**

- **The Dockerfile's own early TensorRT install was arch-broken.** It
  `apt-get install`ed `libnvinfer10` directly, which works only because the
  amd64 `nvidia/cuda` base image pre-configures NVIDIA's repo. On arm64's plain
  `ubuntu:22.04` there is no such repo yet, so the build died on
  `E: Unable to locate package libnvinfer10`. Removed entirely: `setup.sh`'s
  `tensorrt` step already installs the same three packages correctly on both
  architectures, later in the same image.
- **A stale `zed-ros2-wrapper` submodule checkout.** `zed_components` was still
  hard-failing on `find_package(ZED REQUIRED)` despite the fix being pinned,
  because the working tree sat at `458c725` while the superproject pinned
  `24e978f` ("Make the skip actually work: find ament_cmake before calling
  ament_package"). `git submodule update` was the whole fix. Worth knowing: the
  symptom looks identical to the bug being unfixed.
- **`cuda_ffi` hard-panicked with no CUDA toolkit** -- "CUDA installation not
  found" -- and `cuda_ndt_matcher` took eleven other packages down with it.
  This was the last package in the workspace that had not learned to skip
  itself, and Cargo has no equivalent of the CMake early-return the other three
  use, so the fix was to make the crate **compile and link** with no CUDA
  present: `build.rs` emits a `cuda_ffi_stub` cfg instead of panicking, and
  every public item that reaches a compiled `.cu` kernel has a stub twin
  returning `CudaError::NoToolkit`. `NEWSLabNTU/cuda_ndt_matcher` `326c303`,
  verified both ways on this Orin (real mode unchanged, stub mode clean).

**A fourth fix, not a defect in the container but in how it downloads.** The
Autoware deb is ~2 GB and the release host throttles per connection, so the
single-stream `wget` fallback in `install-autoware-debian.sh` is an order of
magnitude slower than the `aria2c` path the script prefers -- measured here,
383 KB/s against 2905 KB/s. A clean machine has no `aria2c`, so that fast path
was one nobody ever took: one attempt crawled at 182 KB/s for an hour. `aria2`
is now in the image's base tools, the step installs it when missing, and every
fallback warns loudly instead of degrading in silence.

**Watch the submodule pin.** It was rewound once mid-session by an unrelated
commit made from a working tree still checked out at the older submodule
commit, which silently removed the `cuda_ffi` fix from `develop` and would have
reproduced defect (3) on the next build. `git submodule status --recursive |
grep '^+'` before committing is what catches this.

### Three more defects, found by getting further than any earlier attempt

Each was fixed in the step rather than the Dockerfile, and each was invisible
until the build reached it.

- **`zed_debug` hard-required the ZED SDK.** `zed_components` had been taught to
  skip itself; `zed_debug`, two packages on in the same repository, had not, so
  the identical `Could not find a package configuration file provided by "ZED"`
  came back one package later -- 1 failed and colcon aborted **19 more** queued
  behind it, the whole localization chain included. The rest of `src/` was
  grepped for the same shape while fixing it: `range_libc`'s CUDA find sits
  inside a `WITH_CUDA` branch and `cuda_pointcloud_filters` guards its own with
  `check_language(CUDA)`, so this was the last unguarded one.
- **`rosdep install` never refreshes the apt lists.** It resolves its keys to
  package names and shells out to `apt-get install`, so on a machine whose lists
  are empty it fails as `E: Unable to locate package python3-serial` -- which
  reads as a missing package and is a missing index. The image hits it every
  time, because the layer above deletes `/var/lib/apt/lists` to keep its own
  size down.
- **Autoware's TensorRT nodes could not load at all.** Two gaps, one behind the
  other, and the second is the interesting one -- see below.

### The DLA problem, which is structural rather than a bug

The arm64 image installed TensorRT from the Jetson repository and then could not
dlopen it. Found by running the planning simulation *in the built image*, where
it reached 33/34 nodes against amd64's 34/34.

First, nothing installed a **CUDA runtime** on this path: amd64 inherits one
from the `nvidia/cuda` base and a Jetson gets one from JetPack, so the plain
`ubuntu:22.04` image had neither and `shape_estimation` died on
`libcudart.so.12`. `cuda-cudart` is 780 KB from the same Jetson repo, now pinned
as `nvidia_arm64.cuda`.

That moved the error one library along, to the real one:

```
libnvdla_compiler.so => not found
libcudla.so.1        => not found
```

The Jetson build of `libnvinfer` links two **Tegra DLA** libraries that live in
the L4T board-support package. They exist only on a Jetson -- `nvidia-l4t-cuda`
has no candidate in the `jetson/common` pocket -- and they talk to the Tegra
driver, so obtaining them would buy nothing on Apple Silicon. The loader does
not care that DLA is unusable here: it refuses `libnvinfer` outright, and every
Autoware node linking TensorRT dies on dlopen.

**Empty stubs satisfy the loader**, and the node then loads and runs: 34/34,
verified. Nothing calls into them, because DLA is requested explicitly with
`setDeviceType(kDLA)` and Autoware builds GPU engines; if something ever did it
would abort on an undefined symbol rather than quietly compute the wrong thing.
Installed only where there is no `/etc/nv_tegra_release` and the real libraries
are absent, so a board keeps its own.

This is the one place the arm64 image is not simply "Autoware on another
architecture", and it is worth knowing before anyone tries to remove it.

### Iterating on this image used to cost 35 minutes a try

`COPY . ${AUTOSDV_HOME}` sat **above** `RUN ./setup.sh`, so editing one
CMakeLists invalidated ROS 2, Autoware and every apt package with it -- the
build then spent seventy minutes reinstalling all of it before reaching the line
that had changed. Six arm64 attempts paid that toll.

Only two setup steps read `src/` at all, so `ros-deps` and `range-libc` now run
after the full `COPY` via `--only`, and everything else keys on the installer
alone. The Autoware `.deb` moved into a **cache mount**: 1.9 GB the layer no
longer carries, and a rebuild reuses the download instead of spending 26 minutes
on it again.

Two details cost a build cycle each, both about the repo-root `setup.sh` being a
**symlink** into `setup/`. Naming it as an explicit `COPY` source dereferences
it, and the launcher's `readlink -f "$0"` then resolves to the repo root and
looks for `main.py` there. `COPY .` preserves it, so the installer layer calls
`./setup/setup.sh` directly and the image still ships the link.

Finally, publish so students never pick a variant -- the same command on each
build machine, then one to join them:

```bash
./docker/desktop/publish.sh push     # on each build machine; arch is detected
./docker/desktop/publish.sh link     # once, on either
```

amd64 can be linked on its own and the arm64 entry added later; the tag updates
in place. Details and the platform table: `docker/desktop/README.md`.

## Known gaps

- **The WSL `/dev/dxg` path is unverified.** Written from Microsoft's documented
  requirements; nobody here has a Windows machine to run it on. It ships as a
  documented starting point, not a tested path.
- **`net.core.rmem_max` is not namespaced.** It must be set on the host or
  passed as `--sysctl`; below ~10 MB no ROS 2 node starts at all, with an error
  that mentions nothing about buffers. The entrypoint checks and says so.
- **Every measurement so far is from a 32-thread workstation**, which is four to
  eight times a student laptop. No hardware requirement should be published
  until the 4-core proxy runs.
