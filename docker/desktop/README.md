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
  jerry73204/autosdv:desktop
# then open http://localhost:6080
```

## The two architectures are two platforms, not one image twice

This is the thing to understand before changing anything here.

| | amd64 | arm64 |
|---|---|---|
| Runs on | Windows, Intel Mac, Linux | **Apple Silicon** |
| Built on | any x86 machine | **the Orin** |
| Base image | `nvidia/cuda:12.8.1-cudnn-devel-ubuntu22.04` | `ubuntu:22.04` |
| Autoware | `1.5.0-2ubuntu2204` | **`1.5.0-2jetpack62`** |
| CUDA / TensorRT | NVIDIA's x86 CUDA repository | **the Jetson repository** |

**The arm64 image is a Jetson-flavoured image that happens to run on Apple
Silicon.** It is not a generic-arm64 build, because there is no generic-arm64
Autoware: the only arm64 localrepo NVIDIA and this project publish is the
JetPack one. It works off a Jetson because JetPack 6.2 is Ubuntu 22.04
underneath and Apple Silicon is arm64.

Two consequences that are easy to get wrong:

- **Do not use `nvidia/cuda`'s arm64 tag as the base.** It exists, and it will
  build. But it is the SBSA (server-ARM) platform, and pairing SBSA CUDA with
  JetPack Autoware packages mixes two platforms in one image. Plain Ubuntu plus
  the Jetson repository keeps it on one.
- **arm64 hosts other than the Orin are not a supported platform.** This image
  exists so an Apple Silicon laptop can run the Jetson container natively
  instead of through emulation, not to support arm64 servers.

## Building the arm64 image on the Orin

The Orin is only the **builder**. The image it produces targets Apple Silicon
and is never run on the Jetson itself.

Build it there because arm64 under qemu on an x86 host compiles the whole
workspace through emulation and takes hours. `build.sh` warns when it detects
that situation.

```bash
# on the Orin
cd ~/AutoSDV
git pull
PLATFORM=linux/arm64 TAG=jerry73204/autosdv:desktop-arm64 ./docker/desktop/build.sh
```

Then publish both architectures under one tag, so a student never chooses a
variant:

```bash
docker push jerry73204/autosdv:desktop-amd64      # from the x86 machine
docker push jerry73204/autosdv:desktop-arm64      # from the Orin
docker manifest create jerry73204/autosdv:desktop \
    jerry73204/autosdv:desktop-amd64 \
    jerry73204/autosdv:desktop-arm64
docker manifest push jerry73204/autosdv:desktop
```

## What is established, and how

Every row here was measured, not assumed. The design rests on them.

| Claim | Evidence |
|---|---|
| No NVIDIA **driver** is needed | `readelf -d` over every shipped library: **0 of 840** amd64 and **0 of 1053** arm64 link `libcuda.so.1` |
| Only the runtime is needed, and only by perception | 6 amd64 / 9 arm64 libraries link `libcudart`; all are perception |
| JetPack's arm64 CUDA installs on plain Ubuntu arm64 | probe against `repo.download.nvidia.com/jetson r36.4`: `cuda-cudart-12-6` pulls three config packages, `libnvinfer10` pulls **nothing** |
| TensorRT 10.3 matches what Autoware links | Autoware needs `libnvinfer.so.10`; the Jetson repo serves 10.3 |
| Only `zed_components` needs the ZED SDK | `zed_wrapper`, which the TF path xacros, builds without it |
| Software rendering is fast enough | 2 fps stock, **31 fps** with `rviz/workshop.rviz`; see [the report](../../docs/reports/gpu-less-simulation-and-rviz.md) |

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

### TensorRT is installed twice, and that is still true

6.9 GB of the original image was two TensorRTs:

- **apt `libnvinfer10` 10.16.1.11+cuda13.2**, 2.5 GB, pulled by
  `install-tensorrt.sh` tier 3. Its attempt to pin `10.8.0` found no candidate,
  because NVIDIA's `ubuntu2204` repo no longer carries that patch, so it fell
  back to newest as designed.
- **`/opt/tensorrt/10.8.0`**, 4.4 GB, installed by the `tensorrt-runtime` step
  to repair the engine-ABI mismatch the first install had just created.

Removing the Windows blob takes the second down to 2.4 GB. The duplication
itself is unresolved and is the largest remaining avoidable item.

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
