"""The setup menu, on the standard library.

Textual was 5.8 MB of widgets inside a 15 MB venv, and the first run paid 54
seconds for uv to fetch a Python and build that venv before drawing anything.
`curses` is in the standard library, imports in 3 ms, and is why `setup.sh` no
longer bootstraps an environment at all.

What it kept from the Textual version, because those were the actual features:

* every step is shown, with its current state and the reason it exists;
* nothing installs until a review screen has listed what is about to run;
* a preset is a starting point that stays editable.

What changed is the shape. The preset is now asked **first**, on its own screen,
instead of sitting in a side pane competing for focus with the list -- that
split was what made the arrow keys ambiguous. Inside the list, `p` re-applies a
preset without leaving.
"""

from __future__ import annotations

import curses
import locale
import sys
from typing import Union

from .model import (
    OS_ERROR, OS_WARN, PROFILE_HELP, PROFILES, STATE_WORDS, Machine, Step,
)
from .registry import STEPS, ordered
from .state import State

locale.setlocale(locale.LC_ALL, "")

_UTF8 = "utf" in (locale.getpreferredencoding(False) or "").lower()

# Two columns, two questions, and they are not the same question: the box says
# what this run will do, the state says what the machine already has. Showing
# both as symbols side by side read as a contradiction -- an unticked box next
# to a tick -- so the state is spelled out in words.
STATE = STATE_WORDS
STATE_WIDTH = max(len(word) for word in STATE.values())
TICK, UNTICK = ("[x]", "[ ]")

C_DIM, C_OK, C_WARN, C_BAD, C_HEAD = 1, 2, 3, 4, 5

# `None` is "the user left", at every depth. A sentinel object read a little
# better and typed a lot worse.
Row = tuple[str, Union[str, Step]]


class Selection:
    """Which steps are ticked, and the preset that seeded them."""

    def __init__(self, state: State, preset: str) -> None:
        self.state = state
        self.status = {s.id: state.status_of(s.id, s.digest()) for s in STEPS}
        self.preset = preset
        self.ticked: dict[str, bool] = {}
        self.apply(preset)

    def apply(self, preset: str) -> None:
        """Seed from a preset.

        A step already done is left unticked, so the common case -- setup run
        again after adding one component -- does not reinstall the world. `all`
        is the exception: asking for everything is asking for the done ones
        too, and it is the only way to say that without ticking 25 boxes.
        """
        self.preset = preset
        for step in STEPS:
            want = step.default_for(preset)
            if want and preset != "all" and self.status[step.id] == "ok":
                want = False
            self.ticked[step.id] = want

    def toggle(self, step: Step) -> None:
        self.ticked[step.id] = not self.ticked[step.id]

    def set_all(self, value: bool) -> None:
        for step in STEPS:
            self.ticked[step.id] = value

    def chosen(self) -> list[Step]:
        return ordered({s.id for s in STEPS if self.ticked[s.id]})


def _colour(pair: int, bold: bool = False) -> int:
    if not curses.has_colors():
        return curses.A_BOLD if bold else curses.A_NORMAL
    attr = curses.color_pair(pair)
    return attr | curses.A_BOLD if bold else attr


def _put(win, y: int, x: int, text: str, attr: int = curses.A_NORMAL) -> None:
    """addstr that survives the bottom-right cell and a too-narrow window."""
    height, width = win.getmaxyx()
    if not 0 <= y < height or x >= width:
        return
    try:
        win.addnstr(y, x, text, max(0, width - x - 1), attr)
    except curses.error:
        pass


def _init_colours() -> None:
    if not curses.has_colors():
        return
    curses.start_color()
    try:
        curses.use_default_colors()
        back = -1
    except curses.error:
        back = curses.COLOR_BLACK
    for pair, colour in (
        (C_DIM, curses.COLOR_WHITE),
        (C_OK, curses.COLOR_GREEN),
        (C_WARN, curses.COLOR_YELLOW),
        (C_BAD, curses.COLOR_RED),
        (C_HEAD, curses.COLOR_CYAN),
    ):
        curses.init_pair(pair, colour, back)


# --------------------------------------------------------------------------
# screen 1: the preset
# --------------------------------------------------------------------------
def choose_preset(stdscr, machine: Machine, suggested: str,
                  current: str | None = None) -> str | None:
    """Pick a preset. Returns the name, or None if the user quit.

    Asked before the list rather than beside it: a preset decides most of the
    answer, and the previous layout made the user tab between two panes to find
    that out.
    """
    index = PROFILES.index(current or suggested)
    while True:
        stdscr.erase()
        _put(stdscr, 0, 1, "AutoSDV setup", _colour(C_HEAD, bold=True))
        _put(stdscr, 0, 20, _machine_line(machine), _colour(C_DIM))
        status, message = machine.os_check()
        row = 2
        if status != "ok":
            _put(stdscr, row, 1, message,
                 _colour(C_BAD if status == OS_ERROR else C_WARN))
            row += 2
        _put(stdscr, row, 1, "Start from which preset?", curses.A_BOLD)
        row += 2
        for i, name in enumerate(PROFILES):
            selected = i == index
            _put(stdscr, row, 1, ">" if selected else " ", _colour(C_HEAD, bold=True))
            _put(stdscr, row, 3, f"{name:<9}",
                 curses.A_REVERSE if selected else curses.A_BOLD)
            count = sum(1 for s in STEPS if s.default_for(name))
            _put(stdscr, row, 13, f"{count:2d} steps", _colour(C_DIM))
            row += 1
        row += 1
        for line in _wrap(PROFILE_HELP[PROFILES[index]], stdscr.getmaxyx()[1] - 4):
            _put(stdscr, row, 3, line, _colour(C_DIM))
            row += 1
        height = stdscr.getmaxyx()[0]
        _put(stdscr, height - 1, 1,
             "↑↓ choose   enter continue   q quit" if _UTF8 else
             "up/down choose   enter continue   q quit", _colour(C_DIM))
        stdscr.refresh()

        key = stdscr.getch()
        if key in (curses.KEY_UP, ord("k")):
            index = (index - 1) % len(PROFILES)
        elif key in (curses.KEY_DOWN, ord("j")):
            index = (index + 1) % len(PROFILES)
        elif key in (curses.KEY_ENTER, 10, 13):
            return PROFILES[index]
        elif key in (ord("q"), 27):
            return None


# --------------------------------------------------------------------------
# screen 2: the steps
# --------------------------------------------------------------------------
def choose_steps(stdscr, machine: Machine, sel: Selection) -> list[Step] | None:
    """Tick steps. Returns the chosen list, or None if the user quit."""
    rows: list[Row] = []
    group = None
    for step in STEPS:
        if step.group != group:
            group = step.group
            rows.append(("group", group))
        rows.append(("step", step))
    first = next(i for i, (kind, _) in enumerate(rows) if kind == "step")

    cursor, top = first, 0
    while True:
        height, width = stdscr.getmaxyx()
        body = max(1, height - 7)
        cursor = max(0, min(cursor, len(rows) - 1))
        top = max(min(top, cursor), cursor - body + 1, 0)

        stdscr.erase()
        ticked = sum(1 for v in sel.ticked.values() if v)
        _put(stdscr, 0, 1, "AutoSDV setup", _colour(C_HEAD, bold=True))
        _put(stdscr, 0, 20, f"preset: {sel.preset}   {ticked} of {len(STEPS)} ticked",
             _colour(C_DIM))
        heading = "already installed?"
        _put(stdscr, 1, 3, "run?  step", _colour(C_DIM))
        # Right-aligned against the widest state word, and placed by its own
        # length: _put truncates at the window edge rather than wrapping.
        _put(stdscr, 1, max(0, width - 2 - max(len(heading), STATE_WIDTH)),
             heading, _colour(C_DIM))
        _put(stdscr, 2, 1, "-" * (width - 2), _colour(C_DIM))

        for line, index in enumerate(range(top, min(top + body, len(rows)))):
            kind, value = rows[index]
            y = 3 + line
            if kind == "group":
                _put(stdscr, y, 1, str(value), _colour(C_HEAD, bold=True))
                continue
            assert isinstance(value, Step)
            step = value
            here = index == cursor
            _put(stdscr, y, 1, ">" if here else " ", _colour(C_HEAD, bold=True))
            _put(stdscr, y, 3, TICK if sel.ticked[step.id] else UNTICK,
                 curses.A_BOLD if sel.ticked[step.id] else _colour(C_DIM))
            _put(stdscr, y, 9, step.label,
                 curses.A_REVERSE if here else curses.A_NORMAL)
            fits, reason = machine.applicable(step)
            if not fits:
                _put(stdscr, y, 9 + len(step.label) + 2, f"({reason})", _colour(C_WARN))
            status = sel.status[step.id]
            word = STATE[status]
            if word:
                _put(stdscr, y, max(0, width - 2 - len(word)), word, _colour(
                    {"ok": C_OK, "stale": C_WARN, "failed": C_BAD}[status]))

        if top > 0:
            _put(stdscr, 3, width - 1, "^", _colour(C_DIM))
        if top + body < len(rows):
            _put(stdscr, 2 + body, width - 1, "v", _colour(C_DIM))

        focused = rows[cursor][1]
        if isinstance(focused, Step):
            _put(stdscr, height - 3, 3, _wrap(focused.why, width - 6)[0], _colour(C_DIM))
        _put(stdscr, height - 2, 1, "-" * (width - 2), _colour(C_DIM))
        _put(stdscr, height - 1, 1, _hints(), _colour(C_DIM))
        stdscr.refresh()

        key = stdscr.getch()
        if key == curses.KEY_RESIZE:
            continue
        if key in (curses.KEY_UP, ord("k")):
            cursor = _step_row(rows, cursor, -1)
        elif key in (curses.KEY_DOWN, ord("j")):
            cursor = _step_row(rows, cursor, +1)
        elif key == curses.KEY_PPAGE:
            cursor = _step_row(rows, max(first, cursor - body), 0)
        elif key == curses.KEY_NPAGE:
            cursor = _step_row(rows, min(len(rows) - 1, cursor + body), 0)
        elif key == curses.KEY_HOME:
            cursor = first
        elif key == curses.KEY_END:
            cursor = _step_row(rows, len(rows) - 1, 0)
        elif key == ord(" "):
            focused = rows[cursor][1]
            if isinstance(focused, Step):
                sel.toggle(focused)
        elif key == ord("a"):
            sel.set_all(True)
        elif key == ord("n"):
            sel.set_all(False)
        elif key == ord("r"):
            sel.apply(sel.preset)
        elif key == ord("p"):
            picked = choose_preset(stdscr, machine, sel.preset, sel.preset)
            if picked is not None:
                sel.apply(picked)
        elif key in (curses.KEY_ENTER, 10, 13):
            chosen = sel.chosen()
            if not chosen:
                _flash(stdscr, "Nothing ticked. Space ticks the step under the cursor.")
                continue
            if review(stdscr, machine, chosen):
                return chosen
        elif key in (ord("q"), 27):
            return None


def _step_row(rows: list[Row], index: int, delta: int) -> int:
    """Move to the next selectable row, skipping group headings."""
    index = max(0, min(index + delta, len(rows) - 1))
    while rows[index][0] != "step":
        index += delta or 1
        if not 0 <= index < len(rows):
            return _step_row(rows, max(0, min(index, len(rows) - 1)), -(delta or 1))
    return index


def _hints() -> str:
    if _UTF8:
        return ("↑↓ move   space tick   p preset   a all   n none   "
                "r reset   enter review   q quit")
    return ("up/down move   space tick   p preset   a all   n none   "
            "r reset   enter review   q quit")


# --------------------------------------------------------------------------
# screen 3: the review
# --------------------------------------------------------------------------
def review(stdscr, machine: Machine, steps: list[Step]) -> bool:
    """Name everything before it runs. Enter installs, escape goes back.

    Enter is one keypress from apt, sudo and kernel modules, and on most list
    widgets it is also the key that ticks a row, so it gets pressed by
    accident. This is the stop.
    """
    top = 0
    while True:
        height, width = stdscr.getmaxyx()
        body = max(1, height - 8)
        stdscr.erase()
        _put(stdscr, 0, 1, f"Install {len(steps)} step(s)?",
             _colour(C_HEAD, bold=True))
        _put(stdscr, 1, 1, "-" * (width - 2), _colour(C_DIM))
        for line, step in enumerate(steps[top:top + body]):
            fits, reason = machine.applicable(step)
            text = f"  {step.label}" + ("" if fits else f"   ({reason}, selected anyway)")
            _put(stdscr, 2 + line, 1, text,
                 curses.A_NORMAL if fits else _colour(C_WARN))
        if top + body < len(steps):
            _put(stdscr, 1 + body, 1, f"  ... {len(steps) - top - body} more",
                 _colour(C_DIM))

        notes = []
        if any(s.requires.sudo for s in steps):
            notes.append("Root is needed; sudo is asked for once, up front.")
        if any(s.requires.reboot for s in steps):
            notes.append("At least one step wants a reboot afterwards.")
        notes.append("The menu closes first, so each step prompts and prints "
                     "on the plain terminal.")
        for i, note in enumerate(notes[:3]):
            _put(stdscr, height - 4 + i, 1, note, _colour(C_DIM))
        _put(stdscr, height - 1, 1,
             "enter install   escape back   q quit", _colour(C_DIM))
        stdscr.refresh()

        key = stdscr.getch()
        if key in (curses.KEY_ENTER, 10, 13):
            return True
        if key in (27, ord("b")):
            return False
        if key == ord("q"):
            return False
        if key in (curses.KEY_DOWN, ord("j")):
            top = min(max(0, len(steps) - body), top + 1)
        elif key in (curses.KEY_UP, ord("k")):
            top = max(0, top - 1)


def _flash(stdscr, message: str) -> None:
    height, _ = stdscr.getmaxyx()
    _put(stdscr, height - 1, 1, message + "  (any key)", _colour(C_WARN, bold=True))
    stdscr.refresh()
    stdscr.getch()


def _machine_line(machine: Machine) -> str:
    caps = [name for name in ("cuda", "can", "ublox-gnss", "tier4-camera", "ptp-nic")
            if machine.has(name)]
    bits = [machine.os_name, machine.arch]
    if machine.is_jetson:
        bits.append("jetson")
    if machine.host_role:
        bits.append(f"host={machine.host_role}")
    bits.append("detected: " + (", ".join(caps) if caps else "no sensors"))
    return "   ".join(bits)


def _wrap(text: str, width: int) -> list[str]:
    import textwrap
    return textwrap.wrap(text, max(10, width)) or [""]


# --------------------------------------------------------------------------
# the plain fallback, for a terminal curses cannot drive
# --------------------------------------------------------------------------
def plain_flow(machine: Machine, sel: Selection) -> list[Step] | None:
    """Numbered list, one line of input at a time. No cursor, no redraw."""
    print(f"\n{_machine_line(machine)}\n")
    print("Presets: " + "  ".join(
        f"{i + 1}={name}" for i, name in enumerate(PROFILES)))
    answer = input(f"Preset [{sel.preset}]: ").strip()
    if answer:
        if answer.isdigit() and 1 <= int(answer) <= len(PROFILES):
            sel.apply(PROFILES[int(answer) - 1])
        elif answer in PROFILES:
            sel.apply(answer)
        else:
            print(f"Unknown preset: {answer}")
            return None

    while True:
        print()
        group = None
        for i, step in enumerate(STEPS, 1):
            if step.group != group:
                group = step.group
                print(f"  {group}")
            mark = TICK if sel.ticked[step.id] else UNTICK
            fits, reason = machine.applicable(step)
            note = "" if fits else f"   ({reason})"
            state = STATE[sel.status[step.id]]
            state = f"   [{state}]" if state else ""
            print(f"   {i:2d} {mark} {step.label}{note}{state}")
        print("\n[x] runs now; the bracket at the right is what the machine "
              "already has.")
        print("Toggle by number or range (3 7-9)   p preset   a all   n none   "
              "i install   q quit")
        line = input("> ").strip().lower()
        if line in {"q", "quit"}:
            return None
        if line == "a":
            sel.set_all(True)
        elif line == "n":
            sel.set_all(False)
        elif line == "p":
            answer = input(f"Preset [{sel.preset}]: ").strip()
            if answer in PROFILES:
                sel.apply(answer)
        elif line in {"i", "install", ""}:
            chosen = sel.chosen()
            if not chosen:
                print("Nothing ticked.")
                continue
            print(f"\nAbout to install {len(chosen)} step(s):")
            for step in chosen:
                print(f"  · {step.label}")
            if input("Continue? [y/N] ").strip().lower() in {"y", "yes"}:
                return chosen
        else:
            for token in line.replace(",", " ").split():
                start, _, end = token.partition("-")
                try:
                    lo, hi = int(start), int(end or start)
                except ValueError:
                    print(f"Not a number: {token}")
                    continue
                for number in range(lo, hi + 1):
                    if 1 <= number <= len(STEPS):
                        sel.toggle(STEPS[number - 1])


# --------------------------------------------------------------------------
def run_menu(args) -> int:
    """Pick steps, then run them on the plain terminal.

    Running the installs inside curses was tempting and is wrong: these steps
    are apt, sudo and kernel modules, they prompt, and their output is the
    thing you need when one fails. The menu ends first.
    """
    from .runner import Runner

    machine = Machine()
    state = State()
    imported = state.import_markers({s.id: s.digest() for s in STEPS})
    if imported:
        print(f"Imported {imported} completed steps from the old .markers/ "
              f"directory; those will not be reinstalled.")

    status, message = machine.os_check()
    if status == OS_ERROR and not args.ignore_os_check:
        print(f"error: {message}", file=sys.stderr)
        print("Pass --ignore-os-check to proceed anyway.", file=sys.stderr)
        return 2
    if status == OS_WARN or (status == OS_ERROR and args.ignore_os_check):
        print(f"warning: {message}\n")

    sel = Selection(state, args.profile or machine.suggested_profile())

    if args.plain or not sys.stdout.isatty():
        chosen = plain_flow(machine, sel)
    else:
        chosen = _curses_flow(machine, sel, args.profile)

    if not chosen:
        print("Nothing selected.")
        return 0

    print(f"\nInstalling {len(chosen)} step(s):")
    for step in chosen:
        print(f"  · {step.label}")
    print()
    failures = Runner(state, machine).run_all(
        chosen, stop_on_error=not args.keep_going)
    print()
    if failures:
        print(f"{failures} step(s) failed. Re-run to resume.")
        return 1
    print("Setup complete.")
    return 0


def _curses_flow(machine: Machine, sel: Selection,
                 explicit_profile: str | None) -> list[Step] | None:
    def body(stdscr) -> list[Step] | None:
        curses.curs_set(0)
        _init_colours()
        stdscr.keypad(True)
        # An explicit --profile has already answered the first screen.
        if explicit_profile is None:
            picked = choose_preset(stdscr, machine, sel.preset)
            if picked is None:
                return None
            sel.apply(picked)
        return choose_steps(stdscr, machine, sel)

    try:
        return curses.wrapper(body)
    except curses.error as exc:
        print(f"curses could not drive this terminal ({exc}); "
              f"falling back to the plain menu.\n")
        return plain_flow(machine, sel)
