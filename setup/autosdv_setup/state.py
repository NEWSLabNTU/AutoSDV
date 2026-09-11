"""Setup state: one JSON file, replacing one empty marker file per step.

A marker could only say "this ran once". It could not say when, whether it
succeeded, or whether the script has been edited since -- and the only way to
re-run a step was to know its internal id and delete a file from a dotted
directory the menu never mentioned.

This keeps the same resume behaviour and adds the two cases markers could not
express: `failed`, and `stale` (ran, but the step's digest has changed since).

Deliberately stdlib-only. `--status` has to work when the venv is broken, which
is exactly when someone needs to see what state the machine is in.
"""

from __future__ import annotations

import json
import os
import tempfile
from datetime import datetime, timezone
from pathlib import Path

from .model import SETUP_DIR

STATE_PATH = SETUP_DIR / ".state.json"
LEGACY_MARKERS = SETUP_DIR / ".markers"

OK, FAILED, SKIPPED = "ok", "failed", "skipped"


def _now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


class State:
    def __init__(self, path: Path = STATE_PATH) -> None:
        self.path = path
        self.data: dict = {"version": 1, "steps": {}}
        if path.is_file():
            try:
                self.data = json.loads(path.read_text())
            except (OSError, json.JSONDecodeError):
                # A corrupt state file must not block setup. Start clean and let
                # the digests re-detect what is stale; the worst case is that a
                # few idempotent steps run again.
                self.data = {"version": 1, "steps": {}, "recovered": _now()}
        self.data.setdefault("steps", {})

    # -- queries ---------------------------------------------------------
    def record(self, step_id: str) -> dict | None:
        return self.data["steps"].get(step_id)

    def status_of(self, step_id: str, digest: str) -> str:
        """One of: ok, stale, failed, skipped, pending."""
        rec = self.record(step_id)
        if not rec:
            return "pending"
        if rec.get("status") == OK:
            return "ok" if rec.get("digest") == digest else "stale"
        return rec.get("status", "pending")

    # -- mutations -------------------------------------------------------
    def mark(self, step_id: str, status: str, digest: str, **extra) -> None:
        self.data["steps"][step_id] = {
            "status": status, "at": _now(), "digest": digest, **extra
        }

    def forget(self, step_id: str) -> None:
        self.data["steps"].pop(step_id, None)

    def save(self) -> None:
        # Atomic: a setup interrupted mid-write must not leave unreadable state.
        self.path.parent.mkdir(parents=True, exist_ok=True)
        fd, tmp = tempfile.mkstemp(dir=self.path.parent, prefix=".state-")
        try:
            with os.fdopen(fd, "w") as fh:
                json.dump(self.data, fh, indent=2, sort_keys=True)
                fh.write("\n")
            os.replace(tmp, self.path)
        except BaseException:
            Path(tmp).unlink(missing_ok=True)
            raise

    # -- migration -------------------------------------------------------
    def import_markers(self, digests: dict[str, str]) -> int:
        """Adopt a pre-existing `.markers/` directory.

        Without this, everyone with a working machine reinstalls ROS 2 because
        the state format changed. Imported entries carry the marker's mtime and
        the step's *current* digest, so an imported step reads as done rather
        than immediately stale.
        """
        if not LEGACY_MARKERS.is_dir() or self.data["steps"]:
            return 0
        count = 0
        # Ids that were renamed when the registry was written.
        alias = {"network-dds": "cyclonedds-sysctl"}
        for marker in LEGACY_MARKERS.iterdir():
            if marker.name.startswith("."):
                continue
            step_id = alias.get(marker.name, marker.name)
            if step_id not in digests:
                continue                     # dropped step: iceoryx, pacmod, gdown
            self.data["steps"][step_id] = {
                "status": OK,
                "at": datetime.fromtimestamp(
                    marker.stat().st_mtime, timezone.utc
                ).isoformat(timespec="seconds"),
                "digest": digests[step_id],
                "imported_from": "markers",
            }
            count += 1
        if count:
            self.data["imported_at"] = _now()
            self.save()
        return count
