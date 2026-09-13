"""Run selected steps, recording what happened.

Sudo is asked for once, up front, only when the selection actually needs it --
the old wrapper asked for every recipe except a hardcoded allowlist of four.
A background thread then refreshes the credential every 60s for the rest of
the run, well inside sudo's default 15-minute timestamp_timeout -- otherwise a
step with no sudo of its own (tensorrt-engines, ~1hr) lets the ticket expire,
and a later sudo step (ros-deps) blocks on a password prompt nobody is
watching for.
"""

from __future__ import annotations

import subprocess
import sys
import threading
import time

from .model import REPO_ROOT, Machine, Step
from .state import FAILED, OK, State


class Runner:
    def __init__(self, state: State, machine: Machine, dry_run: bool = False) -> None:
        self.state = state
        self.machine = machine
        self.dry_run = dry_run

    @staticmethod
    def needs_sudo(steps: list[Step]) -> bool:
        return any(s.requires.sudo for s in steps)

    def ensure_sudo(self, steps: list[Step]) -> bool:
        if self.dry_run or not self.needs_sudo(steps):
            return True
        if subprocess.run(["sudo", "-n", "true"], capture_output=True).returncode == 0:
            return True
        print("Some steps need root. Asking once now, up front.")
        return subprocess.run(["sudo", "-v"]).returncode == 0

    @staticmethod
    def _start_sudo_keepalive() -> tuple[threading.Thread, threading.Event]:
        stop = threading.Event()

        def refresh() -> None:
            while not stop.wait(60):
                subprocess.run(
                    ["sudo", "-n", "-v"], stdin=subprocess.DEVNULL, capture_output=True
                )

        thread = threading.Thread(target=refresh, daemon=True)
        thread.start()
        return thread, stop

    def run_one(self, step: Step, log=print) -> bool:
        digest = step.digest()
        if self.dry_run:
            log(f"  would run: {' '.join(step.run)}")
            return True
        started = time.monotonic()
        try:
            proc = subprocess.run(step.run, cwd=REPO_ROOT)
            rc = proc.returncode
        except OSError as exc:
            self.state.mark(step.id, FAILED, digest, error=str(exc))
            self.state.save()
            log(f"  could not start: {exc}")
            return False
        took = round(time.monotonic() - started, 1)
        if rc == 0:
            self.state.mark(step.id, OK, digest, seconds=took)
            self.state.save()
            if step.note:
                log(f"  note: {step.note}")
            return True
        self.state.mark(step.id, FAILED, digest, exit=rc, seconds=took)
        self.state.save()
        return False

    def run_all(self, steps: list[Step], log=print, stop_on_error: bool = True) -> int:
        """Returns the number of failures."""
        if not self.ensure_sudo(steps):
            log("Could not obtain sudo. Nothing was run.")
            return 1
        keepalive = None
        if not self.dry_run and self.needs_sudo(steps):
            keepalive = self._start_sudo_keepalive()
        try:
            failures = 0
            for i, step in enumerate(steps, 1):
                log(f"[{i}/{len(steps)}] {step.label}")
                if self.run_one(step, log=log):
                    if not self.dry_run:
                        log(f"  ok  {step.id}")
                else:
                    failures += 1
                    log(f"  FAILED  {step.id}")
                    if stop_on_error:
                        log("Stopping. Fix the failure and re-run; "
                            "completed steps will be skipped.")
                        break
            return failures
        finally:
            if keepalive is not None:
                thread, stop = keepalive
                stop.set()
                thread.join(timeout=2)
