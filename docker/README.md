# Container images

Two images that share a name and nothing else. Read this before editing either.

| Directory | Image | Runs on | Purpose |
|---|---|---|---|
| [`desktop/`](desktop/) | `autosdv:desktop` | any laptop — Windows, macOS, Linux | the course environment: a browser desktop with RViz and every AutoSDV prerequisite; your checkout is mounted at `/workspace` and built there |
| [`jetson/`](jetson/) | `autosdv:<commit>` | Jetson / L4T only | a target-device image for the AGX Orin |

They are not variants of one another. `jetson/` builds *the vehicle's* software
on NVIDIA's L4T base and expects the NVIDIA container runtime to mount the Tegra
driver in from the host. `desktop/` builds a *teaching* environment on plain
Ubuntu and assumes no GPU at all.

## `desktop/` — the one students use

```bash
./docker/desktop/autosdv.sh        # Linux, macOS; autosdv.ps1 on Windows
# then open http://localhost:6080, and in the container's terminal:
just build
```

The image carries no AutoSDV build of its own. The launcher mounts your checkout
at `/workspace`, and that is the one you build and run: it survives `docker rm`,
and your own editor is already looking at it. (Until 2026-09-24 the image also
shipped a second, prebuilt AutoSDV at `/opt/AutoSDV`; two checkouts in one
container, one of them nobody's, was the most confusing thing about it.)

To run the simulation without building anything — the optional Lab 0 appendix,
the TA's live demo — use the frozen prebuilt image instead:

```bash
./docker/desktop/autosdv.sh --image jerry73204/autosdv:sim
```

`jerry73204/autosdv:sim` is the desktop image exactly as it was before that
date, `/opt/AutoSDV` built in. Nothing rebuilds it; see `desktop/publish.sh`.

Graphics are decided at start-up, not in the instructions: the container always
runs its own X server, and the entrypoint picks the fastest renderer the host
actually exposes — WSL2's d3d12, VirtualGL on a Linux GPU, or software. macOS
always lands on software, because Hypervisor.framework exposes no GPU to a
container and no flag can change that.

That floor is the reason `src/launcher/autosdv_launch/rviz/workshop.rviz`
exists. Measurements: [`docs/reports/gpu-less-simulation-and-rviz.md`](../docs/reports/gpu-less-simulation-and-rviz.md).

`compose.yaml` carries the accelerated profiles so nobody memorises device
flags.

## `jetson/` — not currently buildable

Moved here unchanged from `docker/`, where its generic path implied it was the
project's container image. It is stale in three independent ways and will not
build as it stands:

- `Dockerfile:50` runs `scripts/setup-dev-env/setup-dev-env.sh`, deleted when
  the setup step registry replaced it
- `nvidia-l4t-apt-source.list` pins **r36.3** (JetPack 6.0); the target is
  JetPack **6.2**, which is r36.4
- the base image is `l4t-tensorrt:r8.6.2`, but Autoware 1.5.0 links
  `libnvinfer.so.10` — TensorRT **10**

Repairing it is its own task and is not part of the workshop work.
