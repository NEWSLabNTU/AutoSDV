"""`just twin down` signals only the process groups `just twin up` started: a PGID recorded
before a reboot or a crash may since belong to anything of the user's."""

import os
import signal
import subprocess
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[4]


class _Group:
    """A `sleep` leading its own process group, orphaned to init as the twin's groups are
    (a child of this test would linger as a zombie that `kill -0` still finds)."""

    def __init__(self):
        out = subprocess.run(["bash", "-c", "setsid sleep 60 >/dev/null 2>&1 & echo $!"],
                             capture_output=True, text=True, check=True).stdout
        self.pid = int(out)
        time.sleep(0.3)

    def alive(self):
        try:
            return Path(f"/proc/{self.pid}/stat").read_text().rsplit(")", 1)[1].split()[0] != "Z"
        except FileNotFoundError:
            return False

    def kill(self):
        if self.alive():
            os.kill(self.pid, signal.SIGKILL)


def _group():
    return _Group()


def _starttime(pid):
    return Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()[19]


def _down(record):
    env = dict(os.environ, TWIN_PGIDS=str(record))
    return subprocess.run(["just", "twin", "down"], cwd=REPO, env=env, capture_output=True,
                          text=True, timeout=60)


def _boot():
    return Path("/proc/sys/kernel/random/boot_id").read_text().strip()


def test_a_reused_pgid_is_left_alone(tmp_path):
    p = _group()
    try:
        rec = tmp_path / "twin.pgids"
        rec.write_text(f"boot {_boot()}\n{p.pid} 1\n")   # start time not this process's
        r = _down(rec)
        assert p.alive(), r.stdout + r.stderr
        assert "skip" in r.stdout
        assert not rec.exists()
    finally:
        p.kill()


def test_a_record_from_another_boot_signals_nothing(tmp_path):
    p = _group()
    try:
        rec = tmp_path / "twin.pgids"
        rec.write_text(f"boot not-this-boot\n{p.pid} {_starttime(p.pid)}\n")
        r = _down(rec)
        assert p.alive(), r.stdout + r.stderr
        assert not rec.exists()
    finally:
        p.kill()


def test_the_twins_own_group_is_stopped(tmp_path):
    p = _group()
    rec = tmp_path / "twin.pgids"
    rec.write_text(f"boot {_boot()}\n{p.pid} {_starttime(p.pid)}\n")
    r = _down(rec)
    assert not p.alive(), r.stdout + r.stderr
    assert not rec.exists()
