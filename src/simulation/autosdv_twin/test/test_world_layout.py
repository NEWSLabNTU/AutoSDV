"""The built COSS world (`just twin world`, data/coss-isaac/) laid out as aib and Autoware read it.

Skipped until the world is built: it is generated data, not in git.
"""

from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[4]
MAP = REPO / "data/coss-isaac/map"
WORLD = MAP / "isaac/world.yaml"

pytestmark = pytest.mark.skipif(not WORLD.exists(), reason="the COSS world is not built (just twin world)")


def test_the_projector_is_where_aib_looks():
    # aib's runner: the map dir is the one holding isaac/, two levels above world.yaml; without
    # its map_projector_info.yaml GNSS and the initial pose land ~34 km off (swb 133's lesson)
    map_dir = WORLD.resolve().parent.parent
    assert (map_dir / "map_projector_info.yaml").is_file()
    assert (map_dir / "lanelet2_map.osm").is_file()


def test_the_usd_is_relative_and_exists():
    w = yaml.safe_load(WORLD.read_text())
    assert not Path(w["usd"]).is_absolute()
    assert (WORLD.resolve().parent.parent / w["usd"]).is_file()


def test_the_world_is_the_map_frame():
    assert yaml.safe_load(WORLD.read_text())["map_origin"] == [0.0, 0.0, 0.0]


def test_autoware_localizes_against_the_cloud_the_world_was_meshed_from():
    # the map dir's PCD is the same file the colour transfer tiled (one source for both)
    src = REPO / "data/COSS-map-planning/pointcloud_map.pcd"
    assert (MAP / "pointcloud_map.pcd").resolve() == src.resolve()
