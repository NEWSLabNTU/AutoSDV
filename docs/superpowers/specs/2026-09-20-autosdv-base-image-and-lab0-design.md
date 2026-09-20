# AutoSDV base image, and Lab 0 (ROS 2 fundamentals)

Date: 2026-09-20
Status: approved, implementing

Two deliverables that share one mechanism. The **base image** is an AutoSDV
container built entirely by `setup.sh`, carrying prerequisites and no workspace,
with the host checkout bind-mounted and the host's UID/GID respected. **Lab 0**
is the first thing taught on it: ROS 2 fundamentals in week 3, one hour in class
and a week to finish, with real code the student writes.

The connecting idea: **student code lives in the mount, not in an image.** Lab 0
is therefore the last image handout of the semester — every later lab is source.

---

## 1. Image stack

```
jerry73204/autosdv:base      prereqs only, installed by setup.sh
                             UID/GID entrypoint, VNC desktop, GPU-optional
       │
       └── :desktop          base + the prebuilt workspace (workshop, sim labs)

Lab images: none.
```

`:desktop` keeps its current contract — it is what the workshop handed out and
what the simulation tutorials assume. It becomes a thin layer on `:base` rather
than a parallel build.

### 1.1 What is in the base

Everything `setup.sh` installs under the new `container` profile, which is `dev`
minus four steps:

| excluded step | why |
|---|---|
| `cyclonedds-sysctl` | configures a host kernel the container does not own (`net.core.rmem_max` is not namespaced) |
| `multicast-lo` | same; the entrypoint sets the `lo` MULTICAST flag at run time instead |
| `ros-deps` | reads `src/`, which the base does not contain |
| `range-libc` | same |

The last two are the price of a base with no workspace, and they are recoverable
on demand — building the mounted checkout starts with
`./setup.sh --run --only ros-deps range-libc --yes`. That command is documented,
not discovered.

`turbovnc-virtualgl` moves from a direct script call in the Dockerfile into the
profile. It is already a registry step with a `verify`; calling the script
directly means `--status` cannot see it.

### 1.2 Bootstrap layer stays

`setup.sh` cannot install what it needs to run: `ca-certificates`, `curl`,
`wget`, `aria2`, `gnupg`, `lsb-release`, `software-properties-common`, `git`,
`locales`, `tzdata`, `build-essential`, `python3`, `python3-pip`,
`python3-venv`. `aria2` earns its place for the reason the Dockerfile already
records — 383 KB/s against 2905 KB/s on the 2 GB Autoware deb.

`sudo` **stays** in that list — see §2.1. `gosu` joins it, for §3.

---

## 2. setup.sh changes

### 2.1 The runner stops asking for root when it already is root

**Corrected during implementation.** The original plan — "elide the `sudo`
prefix and drop the package from the image" — was written against an assumption
that did not survive contact with the code: `sudo` is called about **seventy
times inside the install scripts themselves**, across ten of them, not as a
prefix the runner controls. Removing the package would mean rewriting every one
of those call sites on the tested install path, days before a class, for a
cosmetic gain. `sudo` stays installed.

What was actually wrong is smaller and real: `Runner.ensure_sudo` runs
`sudo -n true` whenever any selected step declares `requires.sudo`, **even when
already root**. On a machine without the binary that is an uncaught
`FileNotFoundError` — a traceback before a single step runs. So:

- `needs_sudo` returns `False` when `os.geteuid() == 0`: no prompt, no
  keepalive thread, no `sudo` call.
- `ensure_sudo` catches `OSError` and says which of the two things to fix
  ("install sudo, or run as root") instead of failing once per step.

### 2.2 `--yes` implies a non-interactive frontend

`--yes` exports `DEBIAN_FRONTEND=noninteractive` itself rather than trusting the
caller to have done it. Today the Dockerfile sets it and a TA running the same
command in a terminal does not, so `tzdata` can still stop an "unattended" run.

### 2.3 The `container` profile

Derived, not restated: `container` = `dev` membership minus a named exclusion
set in `registry.py`. One line per exclusion with its reason, rather than
twenty edited step entries and a `--skip` list living in a Dockerfile.

The Dockerfile then reads:

```dockerfile
RUN ./setup/setup.sh --run --profile container --yes
```

### 2.4 Non-goal

`update-alternatives` is not touched. It would break a workstation with many
users, and that constraint stands.

---

## 3. The base entrypoint

Ordered: reconcile the user, then everything the current desktop entrypoint
already does (DDS buffer check, loopback multicast, renderer selection, VNC),
then drop privileges and hand over.

```
HOST_UID / HOST_GID  →  groupmod/usermod the built-in user to match
                     →  chown $HOME (small; never the mount)
                     →  exec gosu <user> <command>
```

`gosu` is 1.14-1ubuntu0.1 in Ubuntu 22.04.

**Not `--user $(id -u)`**, which leaves no `/etc/passwd` entry: the shell greets
the student with `I have no name!`, `$HOME` is unwritable and `~/.ros` logging
fails.

The mount is never chowned — matching the UID is what makes a chown
unnecessary, and chowning a student's git checkout from a container is a thing
we do not do.

Where it matters is Linux. On macOS (VirtioFS) and Windows (drvfs) ownership is
mapped or masked, so this is a no-op there. PowerShell has no `id -u`, so the
launcher defaults to 1000.

Symptoms it prevents on Linux: `build/`, `install/`, `log/` and recorded bags
landing root-owned inside the student's own checkout, and `git status` refusing
with *"detected dubious ownership"*.

---

## 4. GPU and non-GPU

A guarantee, not an accident:

- The amd64 base is `nvidia/cuda:…-cudnn-devel` and **runs without a driver**;
  the libraries sit unused until something makes a CUDA call.
- The entrypoint picks a renderer in order — `/dev/dxg` (WSL2) → `/dev/nvidia*`
  → `/dev/dri` → llvmpipe — and prints which it chose.
- `--gpu` stays opt-in, keeping the driver preflight against 570.124.06.
- The arm64 base is plain `ubuntu:22.04` with Jetson-flavoured Autoware. There
  is no GPU path on Apple Silicon and none is implied.
- **Lab 0 requires no GPU.** turtlesim under llvmpipe is a 2D window.

---

## 5. Workspace layout

```
host AutoSDV/        → /workspace      UID-matched; the student's own editor works on it
  labs/                                its own colcon workspace
    src/lab0_turtle/                   the student writes here
/opt/AutoSDV                           prebuilt in :desktop, absent from :base
```

Mounting over `/opt/AutoSDV` would shadow the prebuilt workspace and break the
entrypoint's `source install/setup.bash`, so the mount is `/workspace`.

`colcon build` runs from `/workspace/labs`, which sees the lab and nothing else —
plain `colcon build` cannot start an 852-package Autoware build by accident.

### 5.1 Launcher changes (`autosdv.sh`, `autosdv.ps1`)

- `--workspace DIR` / `AUTOSDV_WORKSPACE`, defaulting to the repository root.
- Passes `HOST_UID`/`HOST_GID` (1000 on PowerShell).
- **A mount cannot be added to a running container.** A student who already has
  one running gets a message naming `--stop`, not a silently missing directory.

---

## 6. Lab 0

One hour in class (lecture, live demo, briefing), one week to finish.

### 6.1 What the student builds

Package `lab0_turtle`, `ament_python`, in `labs/src/`. turtlesim is the vehicle.

| node | subscribes | publishes | who writes it |
|---|---|---|---|
| `waypoint_publisher` | `/lab0/goal_reached` | `/goal_pose` | student |
| `turtle_controller` | `/turtle1/pose`, `/goal_pose` | `/turtle1/cmd_vel`, `/lab0/goal_reached` | student |
| `trip_recorder` | `/turtle1/pose`, `/lab0/goal_reached` | `/lab0/waypoints_reached`, `/lab0/distance`, `/lab0/elapsed` | **provided, complete** |

Types: `/goal_pose` is `geometry_msgs/Point`, `/turtle1/pose` is
`turtlesim/Pose` (Humble — *not* `turtlesim_msgs`, which is Jazzy onwards),
`/turtle1/cmd_vel` is `geometry_msgs/Twist`, `/lab0/goal_reached` is
`std_msgs/Bool`, the three metrics are `std_msgs/Int32` and two
`std_msgs/Float32`.

Three separate metric topics rather than one packed message: each is readable
with `ros2 topic echo` by a student who has just met the CLI, and no custom
interface package is needed — which would otherwise drag `ament_cmake` into a
Python lab.

`waypoint_publisher` advances on `/lab0/goal_reached`, so each node both
publishes and subscribes. That is the point of the pairing.

Parameters:

| node | parameter | default |
|---|---|---|
| `waypoint_publisher` | `waypoints` (flat `[x0,y0,x1,y1,…]`, since ROS 2 parameters have no nested lists) | a four-point square |
| | `rate_hz` | 1.0 |
| `turtle_controller` | `k_linear` | 1.5 |
| | `k_angular` | 4.0 |
| | `goal_tolerance` | 0.15 |
| | `max_linear` | 2.0 |
| | `max_angular` | 2.0 |

### 6.2 The four stages

1. **Run a launch file.** `ros2 launch stage1_demo.launch.xml` — provided, works,
   no package and no build. Report what `ros2 node list`, `rqt_graph` and
   `ros2 topic hz` say. Everyone reaches a working baseline in ten minutes.
2. **`waypoint_publisher`.** Teaches `ros2 pkg create --build-type ament_python`,
   `declare_parameter`, timers, publishers, `setup.py` entry points, `colcon build`.
3. **`turtle_controller`.** Proportional control on range and bearing; advance
   when inside `goal_tolerance`. The lab's real work.
4. **`lab0.launch.xml` + `ros2 bag`.** One command brings up turtlesim and both
   nodes with `<param from="…/params.yaml"/>`. Record, kill turtlesim, replay the
   bag into `trip_recorder` alone — the logging simulation, at a size that fits
   in one head.

Launch files are **XML**, consistent with AutoSDV's own `.launch.xml`.

### 6.3 `turtle_stub`

turtlesim's kinematics without the window: integrates `/turtle1/cmd_vel` into
`/turtle1/pose` at a fixed step. Provided to students (inside `selfcheck.py`)
and used by the grader, so grading fifty submissions needs no display and is
deterministic.

Verified against the real thing: same topics, same `turtlesim/Pose` fields.

### 6.4 Grading — 100 points, automated

| | pts |
|---|---|
| builds; package, node, topic and executable names match spec | 15 |
| `waypoint_publisher` rate and list correct, parameters overridable | 15 |
| controller reaches waypoints **from a set the student was not given** | 40 |
| launch brings the stack up; `k_linear:=X` on the command line changes behaviour | 10 |
| bag contains the required topics; replay produces metrics | 10 |
| report: four questions whose answers are measured numbers | 10 |

`selfcheck.py` ships to students and runs the same checks, so nobody submits
blind. The controller's 40 points are per-waypoint partial credit, not all-or-nothing.

### 6.5 AI policy

Permitted, and stated plainly: *you may use AI, and you must be able to explain
any line you submit.* Three mechanisms make doing the work the cheaper path —
stubs with numbered TODOs, report questions whose answers are measured numbers
rather than explanations, and evidence-of-run deliverables (the bag, the `hz`
output, an `rqt_graph` screenshot).

---

## 7. Course repository layout

```
2026/labs/Workshop/        ← git mv from Lab0/ (the setup-and-simulation deck)
2026/labs/Lab0/
  build_deck.py            deck, same uv + python-pptx 1.0.2 pipeline
  README.md
  student/                 handed out as a zip
    lab0_turtle/           package skeleton: stubs with numbered TODOs
    launch/stage1_demo.launch.xml   provided, runs
    config/params.yaml
    selfcheck.py
    README.md              the written handout
  ta/
    solution/              complete reference implementation
    grade.py               automated grader
    waypoints_grading.yaml the withheld waypoint set
```

Mirrors `2025/labs/Lab2/solvepnp_practice`'s `student/` + `ta/` split, which is
the pattern this course already uses.

---

## 8. Distribution

One more full handout, and then none.

`:base` carries no prebuilt workspace, and the tarball is **flattened** —
correct for `docker save`, wrong for a Hub tag, and the thing that reclaims the
3.6 GB of CUDA static archives and 1.1 GB of Nsight Compute currently shipped
as whiteouts.

**Measured 2026-09-20**, and the estimate in the first draft of this section
("roughly half of the current 14.34 GB") was wrong:

| artifact | on disk | compressed |
|---|---|---|
| `:desktop`, as handed out | 26.7 GB | 14.19 GB |
| `:base`, unflattened | 24.9 GB | 13 GB |
| `:base`, flattened | 19.9 GB | **11 GB** |

A 22% cut, not 50%. Flattening does reclaim the expected 5 GB on disk — that
part held — but static archives compress extremely well, so they were worth
only about 2 GB of the tarball. This is why the section said measure rather
than promise.

11 GB × 50 students is 550 GB over the classroom switch, against 710 GB for the
desktop image. Worth having, and not the order-of-magnitude saving that a
`ros:humble-ros-base` image would have been had it been able to build AutoSDV.

`export-images.sh` generalizes past its two hardcoded `autosdv-desktop-<arch>`
names. The file server, DHCP configuration and `serve-images.sh` already ran end
to end and need nothing.

---

## 9. Verification — required before anything claims to work

1. **Done.** `setup.sh --run --profile container --yes` completed unattended as
   root inside `docker build`, 19 steps in 1316 s, ending `ok turbovnc-virtualgl`
   — which also confirms the step moved into the profile rather than being
   called as a script. (`sudo` is installed; see the correction in §2.1.)
2. **Done, and it corrected this document** — see the table in §8. 11 GB
   flattened, not the ~7 GB the estimate implied.
3. **Done on amd64, NOT on arm64.** Under `LIBGL_ALWAYS_SOFTWARE=1`: renderer
   `llvmpipe (LLVM 15.0.7)`, turtlesim drives from (5.544, 5.544, 0.000) to
   (5.033, 9.227, -2.878) under `cmd_vel`, and grabbed frames show both the
   turtle with its pen trace and `rqt_graph` with `/turtlesim` in it. arm64 is
   untested because testing it here would measure qemu rather than an Apple
   Silicon laptop, which is strictly faster. Full numbers:
   `docs/reports/gpu-less-simulation-and-rviz.md`.
4. **Done.** `grade.py ta/solution` → **100.0/100** in 70 s. A copy of that
   solution with one line sabotaged — `self._cmd_pub.publish(Twist())`, so the
   command is built and never filled — scores **55/100**: `0.0/40` on the
   controller and `5.0/10` on launch, because a stationary turtle cannot
   demonstrate a `k_linear:=` override. Partial credit is visible rather than
   all-or-nothing. (It keeps the bag and report marks because those artifacts
   were unchanged; the recorded evidence is graded separately from the code,
   which is correct.)
5. **Done.** Same image, same mount, two runs: `HOST_UID=0` leaves
   `made-as-root` owned `0:0`, and `HOST_UID=$(id -u)` leaves
   `made-with-hostuid` owned `1000005:1000001`. The container reports
   `user: autosdv (uid 1000005, gid 1000001)` and `id` inside agrees.
6. **Done, added during implementation** — the whole Lab 0 workflow in `:base`:
   `ros2 pkg create --build-type ament_python`, then `colcon build` from
   `/workspace/labs` (1 package, 0.70 s), then the node running with a
   command-line parameter override measured at 9.996 Hz for `rate_hz:=10.0`.
   `build/`, `install/`, `log/` and `src/` all land host-owned.

## 10. Open risk

The `:dev`-style layer sharing discussed earlier is **abandoned**: tarball
distribution never shares layers, so the base costs its full size on the wire
once. Accepted deliberately, because it buys a clean base and zero cost for
every later lab.
