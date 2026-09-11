#!/usr/bin/env python3
"""AutoSDV setup.

Reached through ./setup.sh, which bootstraps the environment first. Runnable
directly with a system python3 for everything except the TUI -- `--status`,
`--list` and `--dry-run` import nothing outside the standard library, on purpose:
those are what you need when the venv is the thing that is broken.
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from autosdv_setup.model import (  # noqa: E402
    OS_ERROR, OS_WARN, PROFILE_HELP, PROFILES, RETIRED_PROFILES, STATE_WORDS,
    Machine,
)
from autosdv_setup.registry import BY_ID, STEPS, ordered  # noqa: E402
from autosdv_setup.runner import Runner  # noqa: E402
from autosdv_setup.state import State  # noqa: E402

COLOUR = {"ok": "32", "stale": "33", "failed": "31", "skipped": "90", "pending": "90"}


def _paint(status: str, text: str) -> str:
    return f"\033[{COLOUR[status]}m{text}\033[0m"


def _state(status: str) -> str:
    """The machine's current state for a step, in words.

    Not a symbol: the listing already has a column for whether the profile
    selects the step, and two symbol columns side by side read as one
    contradictory statement -- an unticked box beside a tick. Colour is added
    after any padding, because escape sequences are not width.
    """
    return STATE_WORDS[status]


def _statuses(state: State) -> dict[str, str]:
    return {s.id: state.status_of(s.id, s.digest()) for s in STEPS}


def cmd_list(args) -> int:
    machine = Machine()
    state = State()
    status = _statuses(state)
    profile = args.profile or machine.suggested_profile()
    if args.json:
        import json
        print(json.dumps({
            "profile": profile,
            "arch": machine.arch,
            "jetson": machine.is_jetson,
            "steps": [
                {
                    "id": s.id, "label": s.label, "group": s.group,
                    "status": status[s.id],
                    "default": s.default_for(profile),
                    "applicable": machine.applicable(s)[0],
                    "reason": machine.applicable(s)[1],
                    "sudo": s.requires.sudo,
                }
                for s in STEPS
            ],
        }, indent=2))
        return 0
    group = None
    print(f"Profile: {profile}   host: {machine.arch}"
          f"{'  (jetson)' if machine.is_jetson else ''}\n")
    width = shutil.get_terminal_size((100, 24)).columns
    right = max(len(w) for w in STATE_WORDS.values())
    print("  run?  step".ljust(width - right - 2) + "already installed?\n")
    for step in STEPS:
        if step.group != group:
            group = step.group
            print(f"  {group}")
        ok, reason = machine.applicable(step)
        default = "[x]" if step.default_for(profile) else "[ ]"
        note = "" if ok else f"   ({reason})"
        left = f"   {default}  {step.id:<20} {step.label}{note}"
        word = _state(status[step.id])
        pad = max(1, width - right - 2 - len(left))
        print(left + " " * pad + _paint(status[step.id], word.rjust(right))
              if word else left)
    print(f"\n  [x] is what profile '{profile}' selects; the right-hand column "
          f"is what is already installed.")
    return 0


def cmd_status(args) -> int:
    state = State()
    status = _statuses(state)
    if args.json:
        import json
        print(json.dumps({
            s.id: {"status": status[s.id], **(state.record(s.id) or {})}
            for s in STEPS
        }, indent=2))
        return 0
    counts: dict[str, int] = {}
    for step in STEPS:
        if step.verify:
            continue          # reported from the machine below, not from state
        counts[status[step.id]] = counts.get(status[step.id], 0) + 1
    print("Setup status\n")
    live_checked = 0
    for step in STEPS:
        rec = state.record(step.id) or {}
        when = rec.get("at", "")
        extra = ""
        if status[step.id] == "failed":
            extra = f"  exit {rec.get('exit', '?')}"
        elif status[step.id] == "stale":
            extra = "  re-run to pick up changes"
        word = _state(status[step.id]) or "not run"
        # Where a step can say what the machine looks like NOW, that answer
        # wins: a recorded run says a step happened once, which a reboot or an
        # OTA can quietly undo.
        if step.verify:
            live_checked += 1
            present = subprocess.run(
                step.verify, capture_output=True, timeout=30
            ).returncode == 0
            word = "present on this machine" if present else "ABSENT on this machine"
            painted = _paint("ok" if present else "failed", word.ljust(23))
            print(f"  {step.id:<20} {painted} {when}{extra}")
            continue
        painted = _paint(status[step.id], word.ljust(23))
        print(f"  {step.id:<20} {painted} {when}{extra}")
    if counts:
        print("\n  " + "   ".join(
            f"{STATE_WORDS[k] or 'not run'}: {v}" for k, v in sorted(counts.items())))
    if live_checked:
        print(f"  {live_checked} row(s) read the machine rather than the state "
              f"file; the rest report what was recorded")
    print(f"  state file: {state.path}")
    return 0


def _known(ids, what: str) -> set:
    unknown = [i for i in ids if i not in BY_ID]
    if unknown:
        print(f"unknown step(s) for {what}: {', '.join(unknown)}", file=sys.stderr)
        print("Run --list to see them all.", file=sys.stderr)
        raise SystemExit(2)
    return set(ids)


def _select(args, machine: Machine, state: State) -> list:
    """Resolve the selection the way the menu would, without the menu.

    `--only` is exact and skips the done-check, because naming a step is already
    saying you want it. Everything else starts from a profile (or `--all`),
    drops what is already done unless `--force`, then subtracts `--skip`.
    """
    skip = _known(args.skip or (), "--skip")
    if args.only:
        return ordered(_known(args.only, "--only") - skip)

    profile = args.profile or machine.suggested_profile()
    status = _statuses(state)
    chosen = set()
    for step in STEPS:
        if not (args.all or step.default_for(profile)):
            continue
        if not args.force and status[step.id] == "ok":
            continue
        chosen.add(step.id)
    return ordered(chosen - skip)


def check_os(machine: Machine, args) -> int | None:
    """None to continue, or an exit code.

    Every install script here writes Ubuntu 22.04 apt package names, so this is
    not a style preference. A neighbouring Ubuntu or a Debian warns and
    proceeds -- someone doing that is doing it deliberately. Anything else
    stops, and says which flag reopens the door.
    """
    status, message = machine.os_check()
    if status == OS_ERROR and not args.ignore_os_check:
        print(f"error: {message}", file=sys.stderr)
        print("Pass --ignore-os-check to proceed anyway.", file=sys.stderr)
        return 2
    if status == OS_WARN or (status == OS_ERROR and args.ignore_os_check):
        print(f"warning: {message}\n")
    return None


def cmd_run(args) -> int:
    machine = Machine()
    state = State()
    refused = check_os(machine, args)
    if refused is not None:
        return refused
    imported = state.import_markers({s.id: s.digest() for s in STEPS})
    if imported:
        print(f"Imported {imported} completed steps from the old .markers/ "
              f"directory; those will not be reinstalled.\n")

    steps = _select(args, machine, state)
    if not steps:
        print("Nothing to do: the selection is empty -- already done, "
              "or subtracted by --skip.")
        print("Use --force to re-run, or --only <step> to pick one.")
        return 0

    if args.json:
        # Describing a run and performing one are different requests; making
        # --json imply "do not install" would be a silent no-op the day someone
        # adds it to a working command line.
        if not args.dry_run:
            print("--json describes a selection: pair it with --dry-run.",
                  file=sys.stderr)
            return 2
        import json
        print(json.dumps([s.id for s in steps]))
        return 0

    profile = "--only" if args.only else ("all" if args.all
                                          else args.profile or machine.suggested_profile())
    print(f"Profile: {profile}   {len(steps)} step(s) to run\n")
    for step in steps:
        ok, reason = machine.applicable(step)
        flag = "" if ok else f"   ({reason} -- selected anyway)"
        print(f"  · {step.label}{flag}")
    print()

    if args.dry_run:
        print("Dry run. Nothing installed.")
        Runner(state, machine, dry_run=True).run_all(steps)
        return 0

    if not args.yes and sys.stdin.isatty():
        if input("Continue? [Y/n] ").strip().lower() in {"n", "no"}:
            print("Cancelled.")
            return 0
        print()

    failures = Runner(state, machine).run_all(
        steps, stop_on_error=not args.keep_going)
    print()
    if failures:
        print(f"{failures} step(s) failed. Re-run to resume; "
              f"completed steps are skipped.")
        return 1
    print("Setup complete.")
    return 0


def cmd_rerun(args) -> int:
    state = State()
    for step_id in args.steps:
        if step_id not in BY_ID:
            print(f"unknown step: {step_id}", file=sys.stderr)
            return 2
        state.forget(step_id)
    state.save()
    print(f"Cleared state for: {', '.join(args.steps)}")
    args.only = args.steps
    args.force = True
    return cmd_run(args)


def cmd_menu(args) -> int:
    from autosdv_setup.menu import run_menu
    return run_menu(args)


def _accept_old_forms(argv: list[str]) -> list[str]:
    """Keep the previous command forms working.

    `./setup.sh status` and `./setup.sh <recipe>` were both documented, and are
    in CLAUDE.md and the README. Translating them costs a few lines and saves
    everyone relearning a tool they already know.
    """
    if not argv or argv[0].startswith("-"):
        return argv
    head, rest = argv[0], argv[1:]
    if head in {"status", "list"}:
        return [f"--{head}", *rest]
    if head in {"clean-markers", "clean-marker"}:
        print("Markers are gone; state lives in setup/.state.json.\n"
              "Use --rerun <step> for one step, or delete that file to reset all.",
              file=sys.stderr)
        raise SystemExit(2)
    if head in BY_ID:
        return ["--only", *argv]
    return argv


def _rename_profile(argv: list[str]) -> list[str]:
    """`--profile laptop` and `--profile orin` predate the current names."""
    out = list(argv)
    for i, token in enumerate(out):
        name = None
        if token == "--profile" and i + 1 < len(out):
            name, at = out[i + 1], i + 1
        elif token.startswith("--profile="):
            name, at = token.split("=", 1)[1], i
        if name in RETIRED_PROFILES:
            replacement = RETIRED_PROFILES[name]
            print(f"note: profile '{name}' is now '{replacement}'.", file=sys.stderr)
            out[at] = replacement if out[at] == name else f"--profile={replacement}"
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        prog="setup.sh",
        description="AutoSDV setup. With no arguments, opens the menu.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Profiles:\n" + "\n".join(
            f"  {name:<9} {PROFILE_HELP[name]}" for name in PROFILES
        ) + """

Unattended:
  setup.sh --run --profile vehicle --yes     the profile's steps, no prompts
  setup.sh --run --all --skip tensorrt-engines opencv
  setup.sh --only ros2 ros2-dev-tools --yes  exactly these
  setup.sh --dry-run --json --profile ci     what would run, as a JSON list
  setup.sh --run --profile ci -y --keep-going
""",
    )
    ap.add_argument("--profile", choices=PROFILES,
                    help="preset selection; asked for, or detected, if omitted")
    ap.add_argument("--only", nargs="+", metavar="STEP",
                    help="run exactly these steps")
    ap.add_argument("--rerun", nargs="+", metavar="STEP",
                    help="forget these steps' state, then run them")
    ap.add_argument("--all", action="store_true",
                    help="select every step, not just the profile's")
    ap.add_argument("--skip", nargs="+", metavar="STEP", default=[],
                    help="subtract these steps from the selection")
    ap.add_argument("--force", action="store_true",
                    help="run selected steps even if already done")
    ap.add_argument("--keep-going", action="store_true",
                    help="do not stop at the first failure")
    ap.add_argument("--run", action="store_true",
                    help="run the resolved selection without opening the menu")
    ap.add_argument("--json", action="store_true",
                    help="machine-readable output for --list, --status, --dry-run")
    ap.add_argument("--ignore-os-check", action="store_true",
                    help="run on a distribution ROS 2 Humble does not target")
    ap.add_argument("--plain", action="store_true",
                    help="numbered menu instead of the full-screen one")
    ap.add_argument("--yes", "-y", action="store_true", help="no prompts")
    ap.add_argument("--dry-run", action="store_true",
                    help="resolve and print, install nothing")
    ap.add_argument("--list", action="store_true",
                    help="every step, its status and whether it applies here")
    ap.add_argument("--status", action="store_true", help="what is installed")
    args = ap.parse_args(_rename_profile(_accept_old_forms(
        list(argv) if argv is not None else sys.argv[1:])))

    if args.list:
        return cmd_list(args)
    if args.status:
        return cmd_status(args)
    if args.rerun:
        args.steps = args.rerun
        return cmd_rerun(args)
    if (args.run or args.all or args.only
            or args.skip or args.yes or args.dry_run or args.force):
        return cmd_run(args)
    if args.plain:
        # The numbered menu reads lines, so it works on a pipe or a serial
        # console -- the two places the full-screen one cannot go.
        return cmd_menu(args)
    if not sys.stdin.isatty():
        # No terminal to draw a menu on. Say which flag was wanted rather than
        # failing somewhere less obvious.
        print("No terminal: use --run (optionally with --profile/--only) "
              "for an unattended install.", file=sys.stderr)
        return 2
    # A bare --profile still opens the menu: it answers the preset screen and
    # leaves the list editable, which is what picking a preset means here.
    return cmd_menu(args)


if __name__ == "__main__":
    sys.exit(main())
