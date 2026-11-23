"""The map a lap is graded against: `build` reads the team's map folder.

    {"lab": "lab3", "kind": "stack", "launch_time": ..., "map_dir": ..., "map_hash": ...,
     "speed_limits": {lanelet id: km/h}, "vehicle_id": ..., "autosdv": <git rev>,
     "lanelet2_map": <the lanelet2 file's text>}

lab3_grade --map MAP_DIR grades a lap bag against it (the loop and its lanelets from the
file, map_hash naming the folder). Bags recorded before the grader took --map carried the
same record on /lab3/run_meta, which the grade still reads when --map is not given.
"""

import os

from autosdv_lab3 import mapdir

TOPIC = '/lab3/run_meta'


def speed_limits(map_dir: str) -> dict:
    """{lanelet id: speed_limit tag} of the loaded lanelet file; {} when it is unreadable."""
    from autosdv_lab3.route import MapError, read_lanelets
    try:
        lanelets = read_lanelets(os.path.join(map_dir, mapdir.LANELET))
    except MapError:
        return {}
    return {str(lid): ll.tags.get('speed_limit') for lid, ll in sorted(lanelets.items())}


def lanelet_text(map_dir: str) -> str:
    try:
        with open(os.path.join(map_dir, mapdir.LANELET), encoding='utf-8') as f:
            return f.read()
    except OSError:
        return ''


def build(map_dir: str, vehicle_id: str, launch_time: float) -> dict:
    return {'lab': 'lab3', 'kind': 'stack', 'launch_time': launch_time,
            'map_dir': os.path.abspath(map_dir), 'map_hash': mapdir.folder_hash(map_dir),
            'speed_limits': speed_limits(map_dir),
            'vehicle_id': vehicle_id, 'autosdv': mapdir.git_rev(),
            'lanelet2_map': lanelet_text(map_dir)}
