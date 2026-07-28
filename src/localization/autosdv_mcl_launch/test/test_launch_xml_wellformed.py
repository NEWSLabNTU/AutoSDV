"""Every launch file in this package must be well-formed XML.

This exists because the same mistake was made four times while developing this
package: `--` inside an XML comment is illegal per the XML spec, and the failure
surfaces only at launch time as

    Error: Rust parser error: XML parsing error: comment at 63:3 contains '--'

which costs a full stack startup to discover. Prose written for humans naturally
reaches for an em-dash, so this is a trap worth automating rather than
remembering.
"""
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

LAUNCH_DIR = Path(__file__).resolve().parent.parent / "launch"
LAUNCH_FILES = sorted(LAUNCH_DIR.glob("*.launch.xml"))


def test_launch_directory_is_found():
    """Guard against the glob silently matching nothing."""
    assert LAUNCH_FILES, f"no launch XML found under {LAUNCH_DIR}"


@pytest.mark.parametrize("path", LAUNCH_FILES, ids=lambda p: p.name)
def test_launch_file_parses(path):
    ET.parse(path)


@pytest.mark.parametrize("path", LAUNCH_FILES, ids=lambda p: p.name)
def test_no_double_dash_inside_comments(path):
    """Catch the illegal sequence with a message naming the line.

    ElementTree already rejects these, but its error gives a byte offset; this
    reports the offending text so the fix is obvious.
    """
    text = path.read_text()
    offenders = []
    for match in re.finditer(r"<!--(.*?)-->", text, flags=re.S):
        body = match.group(1)
        if "--" in body:
            line = text[: match.start()].count("\n") + 1
            snippet = next((ln.strip() for ln in body.splitlines() if "--" in ln), "")
            offenders.append(f"{path.name}:{line}: {snippet[:80]}")
    assert not offenders, "'--' is illegal inside an XML comment:\n" + "\n".join(offenders)
