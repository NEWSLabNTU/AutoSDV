"""Every setup step, in the order they must run.

A step is one object. If it is not in this list it does not run, and if it is in
this list the UI shows it. The previous design split that knowledge three ways --
a MENU_ITEMS table in setup.sh held labels and defaults, the justfile's `setup:`
recipe held the order, and a `_setup-*` wrapper per option held the condition --
so a step nobody wrote a wrapper for ran unconditionally and appeared in no menu.

Dropped from the previous system, and why:

* `pacmod` -- an AutonomouStuff apt source, added with `trusted=yes` so
  signatures are not checked. Nothing under `src/` references pacmod; this
  vehicle's interface is PCA9685 over I2C. Verified by grep before removal.
* `nebula-driver`, `ublox-driver` as apt steps of their own -- `rosdep install
  --from-paths src` already answers them. `ublox_gps` resolves to
  `ros-humble-ublox-gps`, which pulls `ublox-msgs` and `ublox-serialization`
  from one key, and Nebula arrives through `autoware-full-1-5-0`.

Kept, against the golf cart's registry which drops it:

* `gdown` -- still used here, by scripts/2dlidar/download-sample-rosbag.sh and
  cuda_ndt_matcher/scripts/download_sample_data.sh.
* `blickfeld`, `zed-sdk` -- AutoSDV has the hardware that needs them.

`just` is an ordinary step now. It used to be a prerequisite installed by hand
from a piped curl before setup would run at all; setup no longer needs it, but
the rest of the repo does.
"""

from __future__ import annotations

from .model import (
    DECLARED, FILES_DIR, HARDWARE_DIR, REPO_ROOT, SCRIPTS_DIR, Requires, Step,
)

_S = lambda name: str(SCRIPTS_DIR / name)          # noqa: E731
_BASH = lambda body: ["bash", "-euc", body]        # noqa: E731


# Preflight fragments. A step that needs something an EARLIER step installs
# must say so itself, because `after` is ordering only -- it never pulls the
# dependency into the selection. `--only tensorrt-engines` on a bare machine,
# or a run whose autoware-debian was unticked, both arrive here with nothing
# installed, and the failure then happens inside a third-party file:
#
#     /opt/ros/humble/setup.bash: No such file or directory
#
# which names neither the step nor the step that would fix it.
_REQUIRE_ROS = """
if [[ ! -f /opt/ros/humble/setup.sh ]]; then
    echo "ROS 2 Humble is not installed: /opt/ros/humble/setup.sh is missing." >&2
    echo "Run the ros2 step first, or let a full setup run reach it:" >&2
    echo "    ./setup.sh --only ros2" >&2
    exit 1
fi
"""

_REQUIRE_AUTOWARE = """
if [[ ! -f /opt/autoware/1.5.0/setup.bash ]]; then
    echo "Autoware 1.5.0 is not installed: /opt/autoware/1.5.0/setup.bash is missing." >&2
    echo "Run the autoware-debian step first, or let a full setup run reach it:" >&2
    echo "    ./setup.sh --only autoware-debian autoware-data" >&2
    exit 1
fi
"""

# The `just` step installs into ~/.local/bin, which Ubuntu's ~/.profile adds to
# PATH only at login and only when the directory already existed. On a first
# run the directory is created mid-pass, so a later step in the same pass
# inherits a PATH without it and `just` is not found. Prepend it rather than
# asking the user to log out and back in between two steps of one run.
_LOCAL_BIN_ON_PATH = """
export PATH="$HOME/.local/bin:$PATH"
if ! command -v just >/dev/null; then
    echo "just is not installed, and this step runs a just recipe." >&2
    echo "Run the just step first, or let a full setup run reach it:" >&2
    echo "    ./setup.sh --only just" >&2
    exit 1
fi
"""


def _ros_bash(body: str) -> list[str]:
    """Run `body` with ROS 2 Humble sourced.

    Two things this exists for, both learned the hard way.

    `set -u` has to come off around the sourcing. ROS's setup.sh and the ament
    shell hooks read variables that are deliberately unset, so under nounset
    the source aborts with

        /opt/ros/humble/setup.sh: line 124: AMENT_TRACE_SETUP_FILES: unbound
        variable

    which names a variable nobody set, says nothing about the step, and leaves
    `rosdep` looking like the thing that failed. It is restored immediately
    afterwards so the body itself still runs under nounset.

    And ROS may legitimately not be there yet -- see `_REQUIRE_ROS`.
    """
    return ["bash", "-euc", f"""
{_REQUIRE_ROS}
set +u
source /opt/ros/humble/setup.sh
set -u
{body}
"""]

# Only the three declared profiles appear here. `all` and `none` are answered
# by Step.default_for, so nothing has to remember to add a new step to them.
#
# The ladder is dev < vehicle: the vehicle is a development machine that also
# has the sensors and the bus wired to it, so every dev step is a vehicle step
# and the difference is exactly the "System config" group.
DEV = ("dev", "vehicle")
EVERY = ("dev", "vehicle", "ci")
VEHICLE = ("vehicle",)
OPT_IN: tuple[str, ...] = ()            # in no preset; tick it yourself


def _on(*profiles: str) -> dict[str, bool]:
    return {p: (p in profiles) for p in DECLARED}




STEPS: list[Step] = [
    # ---- Toolchain -------------------------------------------------------
    Step(
        id="just",
        label="just (command runner)",
        why="Every workflow in this repo is a just recipe: build, launch, "
            "record, map checks. Setup itself no longer needs it.",
        group="Toolchain",
        run=_BASH(
            "command -v just >/dev/null && { just --version; exit 0; }; "
            "curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh "
            "| bash -s -- --to \"$HOME/.local/bin\""
        ),
        profiles=_on(*EVERY),
    ),
    Step(
        id="ros2",
        label="ROS 2 Humble",
        why="The base distribution everything else builds against.",
        group="Toolchain",
        run=[_S("install-ros2.sh")],
        requires=Requires(sudo=True),
        profiles=_on(*EVERY),
    ),
    Step(
        id="ros2-dev-tools",
        label="ROS 2 development tools",
        why="colcon, rosdep, vcstool. Needed to build the workspace at all.",
        group="Toolchain",
        run=[_S("install-ros2-dev-tools.sh")],
        requires=Requires(sudo=True),
        after=("ros2",),
        profiles=_on(*EVERY),
    ),
    Step(
        id="rust",
        label="Rust toolchain (rustup)",
        why="cuda_ndt_matcher is Rust. Without a toolchain colcon skips it and "
            "pose_source:=cuda_ndt has nothing to launch.",
        group="Toolchain",
        run=_BASH(
            "command -v rustup >/dev/null && { rustc --version; exit 0; }; "
            "curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y"
        ),
        profiles=_on(*EVERY),
    ),
    Step(
        id="colcon-cargo-ros2",
        label="colcon-cargo-ros2 (Rust colcon support)",
        why="Without it colcon skips cuda_ndt_matcher silently and the build "
            "aborts later, on a missing package rather than on this.",
        group="Toolchain",
        run=[_S("install-colcon-cargo-ros2.sh")],
        after=("rust", "ros2-dev-tools"),
        profiles=_on(*EVERY),
        note="Version-floored (>= 0.5.1), so this re-checks on every run.",
    ),
    Step(
        id="play-launch",
        label="play_launch (launch orchestrator)",
        why="Every `just launch` runs through it. 0.9.0 brought the startup "
            "governor; below 0.10.0 the pose initializer's array parameters "
            "render as strings and the node dies at startup.",
        group="Toolchain",
        run=[_S("install-play-launch.sh")],
        profiles=_on(*EVERY),
        note="Version-floored (>= 0.10.0), so this re-checks on every run.",
    ),
    Step(
        id="gdown",
        label="gdown (Google Drive downloader)",
        why="Used by scripts/2dlidar/download-sample-rosbag.sh and the "
            "cuda_ndt_matcher sample-data script.",
        group="Toolchain",
        run=_BASH("pip3 install --user gdown"),
        profiles=_on(*DEV),
    ),
    Step(
        id="dev-tools",
        label="Developer tools (git-lfs, pre-commit, clang-format, PlotJuggler)",
        why="Formatting and the plotting tool `just tool plotjuggler` starts.",
        group="Toolchain",
        run=_BASH(
            "sudo apt-get update && sudo apt-get install -y git-lfs golang && "
            "pip3 install --user pre-commit 'clang-format==17.0.5' && "
            "if ! apt-mark showhold | grep -q ros-humble-plotjuggler-ros; then "
            "sudo apt-get install -y ros-humble-plotjuggler-ros || true; fi"
        ),
        requires=Requires(sudo=True),
        after=("ros2",),
        profiles=_on(*DEV),
    ),
    Step(
        id="python-deps",
        label="Python dependencies (vehicle control)",
        why="Adafruit-PCA9685 and simple-pid, which the vehicle interface "
            "imports at runtime.",
        group="Toolchain",
        run=_BASH("pip3 install --user Adafruit-PCA9685==1.0.1 simple-pid==2.0.1"),
        profiles=_on(*EVERY),
    ),
    Step(
        id="geographiclib",
        label="GeographicLib + geoid data",
        why="Autoware's map projection needs the egm2008-1 geoid to convert "
            "GNSS altitude.",
        group="Toolchain",
        run=_BASH(
            "sudo apt-get update && sudo apt-get install -y geographiclib-tools && "
            "if [[ ! -f /usr/share/GeographicLib/geoids/egm2008-1.pgm ]]; then "
            "sudo geographiclib-get-geoids egm2008-1; fi"
        ),
        requires=Requires(sudo=True),
        profiles=_on(*EVERY),
    ),

    # ---- Autoware --------------------------------------------------------
    Step(
        id="tensorrt",
        label="TensorRT runtime libraries",
        why="autoware-debian cannot resolve without libnvinfer10, and nothing "
            "else installs it: CUDA and TensorRT come from JetPack on the "
            "vehicle and from the host image on a workstation, so a clean "
            "Ubuntu machine has neither.",
        group="Autoware",
        run=[_S("install-tensorrt.sh")],
        requires=Requires(sudo=True),
        # Ordering is the point: autoware-debian fails without this.
        after=("ros2",),
        profiles=_on(*DEV),
        # A soname check, not a package check, so it is true however TensorRT
        # got here -- network repo, a downloaded local-repo deb, tar or
        # runfile. A machine that already has it is never touched, and the step
        # reads no apt configuration at all.
        # Written to survive `pipefail`, which this does not set but a future
        # caller might: `ldconfig -p | grep -q X` reports FAILURE on a match
        # under pipefail, because grep exits at the first hit and ldconfig then
        # takes SIGPIPE. Capturing once and matching a here-string has no
        # pipeline to misreport, and calls ldconfig once instead of three times.
        verify=["bash", "-c",
                "set -o pipefail; "
                "c=$(ldconfig -p 2>/dev/null) || exit 1; "
                "grep -qF libnvinfer.so.10 <<<\"$c\" && "
                "grep -qF libnvinfer_plugin.so.10 <<<\"$c\" && "
                "grep -qF libnvonnxparser.so.10 <<<\"$c\""],
        note="Installs three runtime libraries only: no CUDA toolkit, no "
             "driver, and /usr/local/cuda is not touched.",
    ),
    Step(
        id="autoware-debian",
        label="Autoware Debian packages",
        why="The 1.5.0 localrepo, about 2-3 GB. Everything in src/ builds "
            "against it.",
        group="Autoware",
        run=[_S("install-autoware-debian.sh")],
        requires=Requires(sudo=True),
        after=("ros2",),
        profiles=_on(*DEV),
    ),
    Step(
        id="autoware-data",
        label="Writable Autoware data directory",
        why="Autoware writes each compiled .engine next to the .onnx it built "
            "from, and the packaged tree is root-owned. Without this every "
            "model fails to write its engine and rebuilds, and fails, on every "
            "launch. Seconds, symlinks only.",
        group="Autoware",
        run=[str(REPO_ROOT / "scripts" / "setup_autoware_data.sh")],
        after=("autoware-debian",),
        profiles=_on(*DEV),
        # Re-run after an Autoware upgrade: the mirror points at the version
        # that was installed when it was built.
        verify=["test", "-d", str(REPO_ROOT / "data" / "autoware_data")],
    ),
    Step(
        id="tensorrt-runtime",
        label="TensorRT runtime matching Autoware's build (amd64)",
        why="Complements the 'tensorrt' step, which installs any TensorRT 10 so "
            "the Autoware packages resolve. This one is about engine REUSE: "
            "Autoware discards any cached .engine whose recorded TensorRT "
            "version differs from the one its own libraries were compiled "
            "against -- to the patch. A workstation whose TensorRT is a "
            "different 10.x patch therefore rebuilds all five perception "
            "engines on EVERY launch (~9 min on an RTX 3090) no matter how "
            "often the engine step has run. This extracts the matching "
            "runtime into /opt/tensorrt/<version>, which only shells that "
            "sourced scripts/env.sh see, so nothing else on the machine "
            "moves. No-op on a Jetson: JetPack already ships the one the "
            "arm64 Debians were built against.",
        group="Autoware",
        run=[_S("install-tensorrt-runtime.sh")],
        requires=Requires(sudo=True, arch=("x86_64",)),
        # After `tensorrt`, which makes the Autoware packages installable with
        # any TensorRT 10 and asks for this exact patch when a source offers it.
        # This step is what rescues the case it cannot fix: a workstation that
        # already had a different 10.x, which is left alone there by design.
        after=("autoware-debian", "tensorrt"),
        profiles=_on(*DEV),
        verify=["bash", "-c",
                'v=$("' + str(REPO_ROOT / "scripts" / "version" / "get-version.sh")
                + '" nvidia_amd64.tensorrt_engine_abi) '
                  '&& test -f "/opt/tensorrt/${v}/lib/libnvinfer.so.${v}"'],
    ),
    Step(
        id="tensorrt-engines",
        label="Pre-compile TensorRT engines",
        why="Downloads a prebuilt engine set matching this board's "
            "fingerprint from a NEWSLabNTU/AutoSDV release when one exists; "
            "builds locally (minutes per model on an Orin) otherwise, which "
            "is what would otherwise be paid inside each node's constructor "
            "on the first launch. Select 'tensorrt-engines-build' instead to "
            "always build locally and skip the cache lookup.",
        group="Autoware",
        run=_BASH(_LOCAL_BIN_ON_PATH + f'cd "{REPO_ROOT}" && just engines'),
        after=("autoware-data", "just"),
        profiles=_on(),                 # opt-in: slow (on a cache miss), and board-specific
    ),
    Step(
        id="tensorrt-engines-build",
        label="Build TensorRT engines from scratch (skip the cache)",
        why="Forces a local build even when a cached engine set exists for "
            "this board's fingerprint -- e.g. when developing something that "
            "changes a model, or when the cache shouldn't be trusted. "
            "Engines are tied to the TensorRT version AND the GPU, so this "
            "must run on the target board. Selecting this alongside "
            "'tensorrt-engines' just runs the build twice, harmlessly.",
        group="Autoware",
        run=_BASH(_LOCAL_BIN_ON_PATH + f'cd "{REPO_ROOT}" && just build-engines'),
        after=("autoware-data", "just"),
        profiles=_on(),                 # opt-in: slow, and board-specific
    ),
    Step(
        id="ros-deps",
        label="Workspace ROS dependencies (rosdep)",
        why="Resolves every key the packages under src/ declare, which is also "
            "why there are no per-driver apt steps here.",
        group="Autoware",
        # `rosdep update` is retried because it fetches the whole rosdistro
        # index from raw.githubusercontent.com, and one unreachable file fails
        # the lot -- including legacy ROS 1 entries this project never uses:
        #
        #   ERROR: unable to process source [.../releases/fuerte.yaml]
        #   ERROR: Not all sources were able to be updated.
        #
        # That is a transient network or rate-limit condition, not a broken
        # workspace, and it should not end an install that has already spent
        # twenty minutes downloading Autoware. Three attempts, then fail for
        # real.
        run=_ros_bash(
            "if [[ -f /opt/autoware/1.5.0/setup.sh ]]; then set +u; "
            "source /opt/autoware/1.5.0/setup.sh; set -u; fi\n"
            f'cd "{REPO_ROOT}"\n'
            "updated=0\n"
            "for attempt in 1 2 3; do\n"
            "  if rosdep update --rosdistro=humble; then updated=1; break; fi\n"
            "  echo \"rosdep update failed (attempt ${attempt}/3); retrying in 15s\" >&2\n"
            "  sleep 15\n"
            "done\n"
            "if [[ $updated -ne 1 ]]; then\n"
            "  echo 'rosdep update failed three times; check network access to "
            "raw.githubusercontent.com' >&2\n"
            "  exit 1\n"
            "fi\n"
            "rosdep install -y --from-paths src --ignore-src -r"
        ),
        requires=Requires(sudo=True),
        after=("ros2-dev-tools", "autoware-debian"),
        profiles=_on(*EVERY),
    ),

    # ---- Libraries and drivers ------------------------------------------
    Step(
        id="opencv",
        label="OpenCV consistency (headers, runtime, contrib)",
        why="JetPack leaves 4.8.0 headers over a 4.5.4 runtime, which compiles "
            "and then misbehaves. Also what makes aruco/contrib available.",
        group="Libraries",
        run=[_S("install-opencv.sh")],
        requires=Requires(sudo=True),
        profiles=_on(*DEV),
        verify=["bash", "-c",
                "pkg-config --modversion opencv4 2>/dev/null | grep -q ."],
    ),
    Step(
        id="range-libc",
        label="range_libc (the MCL raycaster)",
        why="pose_source:=mcl raycasts through this Cython extension, and "
            "particle_filter imports it at module scope -- without it that "
            "package cannot even be imported, let alone run.",
        group="Libraries",
        # Built from the submodule rather than pip: it is a Cython extension
        # over the vendored C++ in src/localization/external/range_libc, and
        # there is no wheel of our fork anywhere.
        #
        # cython3 and python3-dev are installed here because they are THIS
        # step's build dependencies and nothing else provides them. Leaving
        # them implicit worked on every machine this was developed on -- all of
        # which already had Cython -- and failed on a clean Ubuntu install with
        #
        #   ModuleNotFoundError: No module named 'Cython'
        #
        # which names the symptom and not the fix. python3-dev is the same
        # class of failure one step later: without Python.h the extension
        # cannot compile.
        run=_BASH(
            "sudo apt-get update && "
            "sudo apt-get install -y --no-install-recommends cython3 python3-dev && "
            f'cd "{REPO_ROOT}/src/localization/external/range_libc/pywrapper" && '
            "python3 setup.py install --user"
        ),
        requires=Requires(sudo=True),
        profiles=_on(*DEV),
        # range_libc imports nav_msgs.msg at module scope, so the check has to
        # source ROS. Importing it from a bare python3 fails on nav_msgs and
        # would report the extension as missing when it is present.
        verify=["bash", "-c",
                "source /opt/ros/humble/setup.bash >/dev/null 2>&1 && "
                "python3 -c 'import range_libc'"],
        note="Rebuild this after changing the range_libc submodule pin.",
    ),
    Step(
        id="blickfeld",
        label="Blickfeld Scanner Library",
        why="The Cube1 LiDAR driver. Selecting it accepts the library's "
            "licence terms.",
        group="Libraries",
        run=[_S("install-blickfeld.sh")],
        requires=Requires(sudo=True),
        profiles=_on(),                 # opt-in: one of three LiDAR options
    ),
    Step(
        id="zed-sdk",
        label="ZED SDK",
        why="The ZED X Mini, which every default sensor suite includes. Large "
            "download.",
        group="Libraries",
        run=[_S("install-zed-sdk.sh")],
        requires=Requires(sudo=True),
        profiles=_on(),                 # opt-in: large, and skippable for LiDAR-only work
    ),

    # ---- System configuration -------------------------------------------
    Step(
        id="cyclonedds-sysctl",
        label="Kernel socket buffers for CycloneDDS",
        why="net.core.rmem_max=2GB plus net.ipv4.ipfrag_*. Below about 10 MB "
            "no ros2 node can start at all.",
        group="System config",
        run=[_S("configure-cyclonedds-sysctl.sh")],
        requires=Requires(sudo=True),
        profiles=_on(*DEV),
        # 10 MB is the floor below which no node starts; the step asks for 2 GB.
        verify=["bash", "-c",
                "[[ $(sysctl -n net.core.rmem_max) -ge 10485760 ]]"],
    ),
    Step(
        id="multicast-lo",
        label="Multicast on loopback (persistent)",
        why="cyclonedds.xml pins lo, and lo loses its MULTICAST flag on every "
            "reboot. Installs multicast-lo.service so it survives one.",
        group="System config",
        run=[_S("configure-multicast-lo.sh")],
        requires=Requires(sudo=True),
        profiles=_on(*DEV),
        # lo drops MULTICAST on every reboot unless the unit restores it.
        verify=["bash", "-c", "ip link show lo | grep -q MULTICAST"],
    ),
    Step(
        id="ublox-udev",
        label="u-blox GNSS udev rules",
        why="Gives the receiver a stable /dev/ublox-gps name and adds you to "
            "dialout. Log out and back in for the group to take effect.",
        group="System config",
        run=_BASH(
            f'sudo cp "{FILES_DIR}/99-ublox-gps.rules" /etc/udev/rules.d/ && '
            "sudo chmod 644 /etc/udev/rules.d/99-ublox-gps.rules && "
            "sudo udevadm control --reload-rules && sudo udevadm trigger && "
            'sudo usermod -aG dialout "$USER" || true'
        ),
        requires=Requires(sudo=True, hardware="ublox-gnss"),
        verify=["test", "-f", "/etc/udev/rules.d/99-ublox-gps.rules"],
        profiles=_on(*VEHICLE),
    ),
    Step(
        id="turbovnc-virtualgl",
        label="TurboVNC + VirtualGL",
        why="GPU-accelerated rendering over VNC, which the ZED tools need to "
            "run in a VNC session at all.",
        group="System config",
        run=[_S("install-turbovnc-virtualgl.sh")],
        requires=Requires(sudo=True),
        verify=["bash", "-c", "command -v vglrun >/dev/null"],
        profiles=_on(),                 # opt-in: only for headless boards driven over VNC
    ),
]

BY_ID = {s.id: s for s in STEPS}


def ordered(selected: set[str]) -> list[Step]:
    """Selected steps in registry order, which already respects `after`.

    `after` is declared per step and asserted here rather than used to build a
    graph: the list is short, hand-ordered for readability, and a mismatch is a
    bug worth failing loudly on rather than silently reordering around.
    """
    seen: set[str] = set()
    out: list[Step] = []
    for step in STEPS:
        if step.id not in selected:
            continue
        for dep in step.after:
            if dep in selected and dep not in seen:
                raise AssertionError(
                    f"registry order is wrong: {step.id} runs before {dep}"
                )
        seen.add(step.id)
        out.append(step)
    return out
