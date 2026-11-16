"""The Lab 3 map folder and its provenance file.

    team<N>_map/
        pointcloud_map.pcd        GLIM's map, exported and downsampled; x y z [intensity],
                                  in GLIM's frame: origin where the mapping bag began
        lanelet2_map.osm          the clockwise loop, local coordinates
        map_projector_info.yaml   projector_type: Local
        lab3_map.yaml             written by the Lab 3 tools: one section per step

The tools that write into the folder (lab3_downsample, lab3_check_lanelet) record their
step in lab3_map.yaml with `record_step`, at which AutoSDV commit and with which settings.
"""

import hashlib
import re
import os
import subprocess
import time
from typing import Optional

import yaml

MAP_META = 'lab3_map.yaml'
PCD = 'pointcloud_map.pcd'
LANELET = 'lanelet2_map.osm'
PROJECTOR = 'map_projector_info.yaml'


def sha256(path: str, chunk: int = 1 << 20) -> str:
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(chunk), b''):
            h.update(block)
    return h.hexdigest()


def git_rev(cwd: Optional[str] = None) -> str:
    """HEAD of the AutoSDV checkout, with +dirty when tracked files are modified."""
    cwd = cwd or os.environ.get('AUTOSDV_ROOT') or os.getcwd()
    try:
        def run(*a):
            return subprocess.run(['git', *a], cwd=cwd, capture_output=True, text=True,
                                  timeout=10).stdout.strip()
        rev = run('rev-parse', 'HEAD')
        if not rev:
            return 'unknown'
        return rev + ('+dirty' if run('status', '--porcelain', '--untracked-files=no') else '')
    except (OSError, subprocess.SubprocessError):
        return 'unknown'


def load_meta(map_dir: str) -> dict:
    path = os.path.join(map_dir, MAP_META)
    if not os.path.isfile(path):
        return {}
    with open(path) as f:
        return yaml.safe_load(f) or {}


def record_step(map_dir: str, step: str, data: dict) -> dict:
    """Write `data` as section `step` of lab3_map.yaml, stamped; returns the whole file."""
    meta = load_meta(map_dir)
    meta[step] = dict(data, time=time.strftime('%Y-%m-%dT%H:%M:%S%z'), autosdv=git_rev())
    os.makedirs(map_dir, exist_ok=True)
    with open(os.path.join(map_dir, MAP_META), 'w') as f:
        yaml.safe_dump(meta, f, sort_keys=False)
    return meta


_SPEED_TAG_RE = re.compile(r'[ \t]*<tag\s+k="speed_limit"\s+v="[^"]*"\s*/>[ \t]*\r?\n?')


def normalized_osm(text: str) -> str:
    """The lanelet2 file with every speed_limit tag removed: what the map hash covers,
    so a team may retune the segment speeds after hand-in (lanelet_speed)."""
    return _SPEED_TAG_RE.sub('', text)


def folder_hash(map_dir: str) -> str:
    """One hash over the files that define the map, for the week-2 bag's metadata. The
    lanelet file is hashed without its speed_limit tags, the one part a team may change."""
    h = hashlib.sha256()
    for name in (PCD, LANELET, PROJECTOR):
        path = os.path.join(map_dir, name)
        h.update(name.encode())
        if not os.path.isfile(path):
            h.update(b'missing')
        elif name == LANELET:
            with open(path, encoding='utf-8', errors='surrogateescape') as f:
                h.update(hashlib.sha256(normalized_osm(f.read()).encode(
                    'utf-8', errors='surrogateescape')).hexdigest().encode())
        else:
            h.update(sha256(path).encode())
    return h.hexdigest()
