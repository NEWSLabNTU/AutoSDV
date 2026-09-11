# Setup

```bash
./setup.sh                       # pick a preset, then the steps
./setup.sh --status              # what is installed
./setup.sh --list                # every step, and whether it applies here
./setup.sh --rerun opencv        # forget one step's state and run it again
./setup.sh --plain               # numbered menu, no full-screen drawing
```

Nothing to install first and no environment to build: the menu is Python's
stdlib `curses`, so `setup.sh` is a launcher and `./setup.sh --list` answers in
about 45 ms.

The state file is `setup/.state.json`. A machine set up by the old bash
`setup.sh` has its `.markers/` directory imported automatically on the first
run, so finished steps are not reinstalled.

### In the menu

The preset is asked **first**, on its own screen, and then the step list opens
seeded from it. `p` re-asks without leaving the list.

Two columns, two questions. `[x]` is what this run will install; the words on
the right are what the machine already has, so an unticked box beside
*installed* is the normal case -- a preset leaves finished steps alone. Tick one
anyway to run it again.

| key | |
|---|---|
| `↑` `↓`, `j` `k` | move |
| `space` | tick / untick |
| `p` | reload from a preset |
| `a` / `n` / `r` | all / none / back to this preset's defaults |
| `enter` | review the selection, then install |
| `q` | quit without installing |

Enter opens a review screen rather than installing straight away: it names every
step, flags any this machine does not look to need, says whether root is wanted,
and `escape` returns to the list with the selection intact.

`--plain` gives a numbered list read one line at a time — for a serial console,
a pipe, or a terminal `curses` cannot drive. It is also the automatic fallback.

### Unattended

```bash
./setup.sh --run --profile vehicle --yes        # the profile's steps, no prompts
./setup.sh --run --all --skip tensorrt-engines  # everything bar one step
./setup.sh --only ros2 ros2-dev-tools --yes     # exactly these
./setup.sh --dry-run --json --profile ci        # what would run, as JSON
./setup.sh --status --json                      # state, for a health check
./setup.sh --run --profile dev -y --keep-going  # do not stop at the first failure
```

| flag | |
|---|---|
| `--run` | run the resolved selection without opening the menu |
| `--profile P` | preset selection; detected when omitted |
| `--all` | every step, not just the profile's |
| `--only STEP...` | exactly these, done or not |
| `--skip STEP...` | subtract from whatever else was selected |
| `--force` | run selected steps even if already done |
| `--keep-going` | continue past a failure instead of stopping |
| `--dry-run` | resolve and print, install nothing |
| `--json` | machine-readable `--list`, `--status`, `--dry-run` |
| `--yes` / `-y` | no prompts |
| `--ignore-os-check` | proceed on a distribution Humble does not target |
| `--plain` | numbered menu instead of the full-screen one |

With no terminal and no flags, setup says which flag was wanted instead of
trying to draw a menu.

## How it fits together

`setup/setup.sh` finds `main.py` and execs it. Everything with logic in it is
Python, and all of it is standard library.

```
setup/setup.sh            launcher: exec python3 main.py
setup/main.py             CLI
setup/autosdv_setup/
    registry.py           every step, in run order
    model.py              what a Step is; hardware detection
    state.py              .state.json, and the .markers import
    runner.py             execution
    menu.py               the menu (stdlib curses, plus a plain fallback)
setup/scripts/            the install scripts steps call
```

Nothing here imports anything outside the standard library, so there is no
environment to build before setup can ask its first question — which matters
most on exactly the machine that has nothing installed yet.

What this replaced: 746 lines of bash, of which about 250 were a checkbox menu
re-forking `tput` and `cut` on every keystroke, plus a `MENU_ITEMS` table whose
rows had to agree by hand with a justfile `setup:` recipe chain and a `_setup-*`
wrapper per option. Steps nobody wrote a wrapper for ran unconditionally and
appeared in no menu. The justfile is gone with it; `registry.py` holds the order
that chain used to encode.

## Profiles

A profile is a set of defaults over the same step list, not a separate path.
Every step stays individually selectable in the menu.

| profile | for |
|---|---|
| `dev` | laptop, workstation, PC: dev tools, libraries, kernel socket buffers, loopback multicast |
| `vehicle` | the vehicle: everything in `dev`, plus the u-blox udev rules |
| `all` | every step, including the slow and opt-in ones |
| `none` | nothing preselected; tick what you want |
| `ci` | headless and minimal, build dependencies only |

`vehicle` is `dev` plus one group. The steps that touch devices or device naming
live under **System config**, and that group is the entire difference between
the two — so a workstation never writes a udev rule, and the vehicle never
misses one.

`all` and `none` are computed rather than declared, so a new step joins them
without anyone remembering to.

The suggested profile comes from detection — whether a u-blox receiver is
attached, whether the GPU is there — and is a starting point, never a
restriction. There is no per-board profile: a Jetson on a desk is a development
machine and a Jetson in the vehicle is the vehicle, and what separates them is
the hardware, not the SoC. A step whose hardware is absent is shown as such and
can still be selected, because machines get provisioned before hardware
arrives.

## State

`setup/.state.json`, one entry per step: status, timestamp, and a digest of what
the step would run.

The digest is why `stale` exists. A marker file could only say "ran once", so an
edited install script still read as done. When a step's script changes its digest
changes, and the menu shows it as stale rather than complete.

An existing `setup/.markers/` directory is imported automatically on first run,
so a working machine does not reinstall everything because the format changed.

To re-run one step, tick it in the menu or use `--rerun <step>`. There is no
marker file to find and delete.

## Adding a step

One entry in `setup/autosdv_setup/registry.py`:

```python
Step(
    id="my-thing",
    label="My thing",
    why="One sentence on what breaks without it.",
    run=[_S("install-my-thing.sh")],
    requires=Requires(sudo=True, hardware="can"),
    profiles=_on("vehicle"),
    group="Hardware",
)
```

There is no second place to register it and no wrapper to write. A step absent
from this list does not run; a step present in it is shown in the menu.

[uv]: https://docs.astral.sh/uv/

## The OS check

ROS 2 Humble targets Ubuntu 22.04 and every install script here writes its apt
package names, so setup looks at `/etc/os-release` before running anything:

| | |
|---|---|
| Ubuntu 22.04 | runs |
| another Ubuntu, or Debian, or anything `ID_LIKE` either | warns, then runs |
| anything else | stops, with `--ignore-os-check` named in the message |

The middle row is deliberate. A neighbouring Ubuntu can be made to work and
someone doing that is doing it on purpose; a distribution with no Humble
packages at all is not a near miss.
