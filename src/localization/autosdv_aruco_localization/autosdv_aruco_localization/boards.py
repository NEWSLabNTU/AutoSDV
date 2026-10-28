"""The lab's ArUco boards, read from config/boards.yaml (the single source). No ROS needed.

    from autosdv_aruco_localization import boards
    boards.MARKER_SIZE, boards.DICTIONARY, boards.PLATE_SIZE

These are DEFAULTS for things that create a map or a picture of one (a survey,
a generated course, the mock camera). A map's own course.yaml records the
marker_size it was surveyed with, and that recorded value is what the localizer
uses; nothing reads these at run time in its place.
"""
import os

import yaml

PACKAGE = 'autosdv_aruco_localization'


def config_path() -> str:
    """boards.yaml in the installed share directory, else beside this source file."""
    try:
        from ament_index_python.packages import get_package_share_directory
        p = os.path.join(get_package_share_directory(PACKAGE), 'config', 'boards.yaml')
        if os.path.exists(p):
            return p
    except Exception:
        pass
    here = os.path.dirname(os.path.realpath(__file__))
    return os.path.join(here, '..', 'config', 'boards.yaml')


def load(path: str = '') -> dict:
    path = path or config_path()
    try:
        with open(path) as f:
            d = yaml.safe_load(f) or {}
    except FileNotFoundError:
        raise FileNotFoundError(
            f'{PACKAGE} config/boards.yaml not found (tried the installed share directory and '
            f'{path}): build the package and source install/setup.bash') from None
    if not isinstance(d, dict):
        raise ValueError(f'{path}: expected a mapping')
    for k in ('dictionary', 'marker_size', 'plate_size'):
        if k not in d:
            raise ValueError(f'{path}: missing {k}')
    return d


_B = load()
DICTIONARY: str = str(_B['dictionary'])
MARKER_SIZE: float = float(_B['marker_size'])
PLATE_SIZE: float = float(_B['plate_size'])
