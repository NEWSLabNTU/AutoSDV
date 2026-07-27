import sys
from pathlib import Path

import numpy as np
import pytest
import yaml
from PIL import Image

sys.path.insert(0, str(Path(__file__).parent))
from check_map import (
    BBox,
    CheckError,
    bbox_from_points,
    overlap_pct,
    parse_grid_yaml,
    parse_lanelet2_osm,
    parse_projector_info,
    read_pgm_counts,
    run_check,
)


# ---------------------------------------------------------------------------
# bbox / overlap maths
# ---------------------------------------------------------------------------

def test_bbox_from_points():
    b = bbox_from_points([(1.0, 2.0), (5.0, -1.0), (3.0, 4.0)])
    assert (b.xmin, b.ymin, b.xmax, b.ymax) == (1.0, -1.0, 5.0, 4.0)


def test_bbox_from_points_empty_raises():
    with pytest.raises(ValueError):
        bbox_from_points([])


def test_overlap_fully_inside():
    inner = BBox(0, 0, 10, 10)
    outer = BBox(-5, -5, 15, 15)
    assert overlap_pct(inner, outer) == pytest.approx(100.0)


def test_overlap_partial():
    inner = BBox(0, 0, 10, 10)   # area 100
    outer = BBox(5, 5, 15, 15)   # intersection [5,10]x[5,10] = 25
    assert overlap_pct(inner, outer) == pytest.approx(25.0)


def test_overlap_disjoint():
    inner = BBox(0, 0, 10, 10)
    outer = BBox(100, 100, 110, 110)
    assert overlap_pct(inner, outer) == pytest.approx(0.0)


def test_overlap_touching_edge_is_zero_area_intersection():
    # boxes share only a boundary line -> zero-area intersection -> 0%
    inner = BBox(0, 0, 10, 10)
    outer = BBox(10, 0, 20, 10)
    assert overlap_pct(inner, outer) == pytest.approx(0.0)


def test_overlap_degenerate_point_inside():
    inner = BBox(5, 5, 5, 5)  # a single point
    outer = BBox(0, 0, 10, 10)
    assert overlap_pct(inner, outer) == pytest.approx(100.0)


def test_overlap_degenerate_point_outside():
    inner = BBox(50, 50, 50, 50)
    outer = BBox(0, 0, 10, 10)
    assert overlap_pct(inner, outer) == pytest.approx(0.0)


# ---------------------------------------------------------------------------
# grid yaml / pgm parsing
# ---------------------------------------------------------------------------

def write_grid(tmp_path, name="occupancy_grid", resolution=0.1, origin=(0.0, 0.0, 0.0),
               pixels=None):
    if pixels is None:
        pixels = np.full((5, 5), 205, np.uint8)
        pixels[0, 0] = 0
        pixels[1, 1] = 254
    yml = tmp_path / f"{name}.yaml"
    pgm = tmp_path / f"{name}.pgm"
    Image.fromarray(pixels).save(pgm)
    yml.write_text(
        f"image: {pgm.name}\nresolution: {resolution}\n"
        f"origin: [{origin[0]}, {origin[1]}, {origin[2]}]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n"
    )
    return yml, pgm


def test_parse_grid_yaml_ok(tmp_path):
    yml, _ = write_grid(tmp_path)
    meta = parse_grid_yaml(yml)
    assert meta["resolution"] == 0.1
    assert meta["origin"] == [0.0, 0.0, 0.0]
    assert meta["image"] == "occupancy_grid.pgm"


def test_parse_grid_yaml_missing_key(tmp_path):
    yml = tmp_path / "bad.yaml"
    yml.write_text("resolution: 0.1\norigin: [0, 0, 0]\n")  # no image key
    with pytest.raises(CheckError):
        parse_grid_yaml(yml)


def test_parse_grid_yaml_not_yaml(tmp_path):
    yml = tmp_path / "bad.yaml"
    yml.write_text("{not: [valid, yaml")
    with pytest.raises(CheckError):
        parse_grid_yaml(yml)


def test_read_pgm_counts(tmp_path):
    pixels = np.full((3, 4), 205, np.uint8)  # 12 cells unknown
    pixels[0, 0] = 0    # 1 occupied
    pixels[0, 1] = 254  # 1 free
    pixels[0, 2] = 254  # 2 free
    pgm = tmp_path / "grid.pgm"
    Image.fromarray(pixels).save(pgm)
    w, h, occ, free, unk, other = read_pgm_counts(pgm)
    assert (w, h) == (4, 3)
    assert occ == 1
    assert free == 2
    assert unk == 9
    assert other == 0


def test_read_pgm_counts_missing_file(tmp_path):
    with pytest.raises(CheckError):
        read_pgm_counts(tmp_path / "does_not_exist.pgm")


# ---------------------------------------------------------------------------
# lanelet2 osm / projector info parsing
# ---------------------------------------------------------------------------

OSM_TEMPLATE = """<?xml version="1.0" encoding="UTF-8"?>
<osm generator="test">
  <node id="1" lat="25.0201" lon="121.5423">
    <tag k="local_x" v="{lx0}"/>
    <tag k="local_y" v="{ly0}"/>
  </node>
  <node id="2" lat="25.0210" lon="121.5430">
    <tag k="local_x" v="{lx1}"/>
    <tag k="local_y" v="{ly1}"/>
  </node>
</osm>
"""


def test_parse_lanelet2_osm_ok(tmp_path):
    osm = tmp_path / "lanelet2_map.osm"
    osm.write_text(OSM_TEMPLATE.format(lx0=0, ly0=0, lx1=10, ly1=10))
    points, local_tags = parse_lanelet2_osm(osm)
    assert len(points) == 2
    assert points[0] == (25.0201, 121.5423)
    assert local_tags["1"] == (0.0, 0.0)
    assert local_tags["2"] == (10.0, 10.0)


def test_parse_lanelet2_osm_malformed_xml(tmp_path):
    osm = tmp_path / "lanelet2_map.osm"
    osm.write_text("<osm><node id=1 lat=25.0></osm>")  # unquoted attrs -> not well-formed
    with pytest.raises(CheckError):
        parse_lanelet2_osm(osm)


def test_parse_lanelet2_osm_no_nodes(tmp_path):
    osm = tmp_path / "lanelet2_map.osm"
    osm.write_text("<?xml version='1.0'?><osm></osm>")
    with pytest.raises(CheckError):
        parse_lanelet2_osm(osm)


def test_parse_projector_info_georeferenced(tmp_path):
    p = tmp_path / "map_projector_info.yaml"
    p.write_text(
        "projector_type: TransverseMercator\nvertical_datum: WGS84\n"
        "map_origin:\n  latitude: 25.0201\n  longitude: 121.5423\n  altitude: 25.0\n"
    )
    info = parse_projector_info(p)
    assert info["projector_type"] == "TransverseMercator"


def test_parse_projector_info_local(tmp_path):
    p = tmp_path / "map_projector_info.yaml"
    p.write_text("projector_type: Local\n")
    info = parse_projector_info(p)
    assert info["projector_type"] == "Local"


def test_parse_projector_info_invalid_type(tmp_path):
    p = tmp_path / "map_projector_info.yaml"
    p.write_text("projector_type: Klingon\n")
    with pytest.raises(CheckError):
        parse_projector_info(p)


def test_parse_projector_info_missing_key(tmp_path):
    p = tmp_path / "map_projector_info.yaml"
    p.write_text("vertical_datum: WGS84\n")
    with pytest.raises(CheckError):
        parse_projector_info(p)


# ---------------------------------------------------------------------------
# end-to-end run_check() using a Local-projector map (no Autoware env needed
# -- this is the one case where the honest fallback (local_x/local_y tags)
# applies directly), covering both a frame-correct and a wrong-frame grid.
# ---------------------------------------------------------------------------

def make_local_map(tmp_path, grid_origin):
    """A tiny 'Local' projector map: lanelet2 nodes span local_x/y
    [0,10]x[0,10], and a grid is placed at `grid_origin`."""
    (tmp_path / "lanelet2_map.osm").write_text(
        OSM_TEMPLATE.format(lx0=0, ly0=0, lx1=10, ly1=10)
    )
    (tmp_path / "map_projector_info.yaml").write_text("projector_type: Local\n")
    write_grid(tmp_path, resolution=1.0, origin=grid_origin,
               pixels=np.full((20, 20), 254, np.uint8))
    return tmp_path


def test_run_check_mcl_frame_correct_is_ready(tmp_path):
    make_local_map(tmp_path, grid_origin=(-5.0, -5.0, 0.0))  # grid covers [-5,15]x[-5,15]
    lines, ready = run_check(tmp_path, "mcl", "occupancy_grid.yaml", autoware_setup=None)
    assert ready is True
    extent = next(l for l in lines if l.name == "grid vs lanelet2 extent")
    assert extent.status == "ok"


def test_run_check_mcl_wrong_frame_grid_fails(tmp_path):
    # grid origin far from the lanelet2 bbox -> disjoint -> must FAIL, not pass
    make_local_map(tmp_path, grid_origin=(1000.0, 1000.0, 0.0))
    lines, ready = run_check(tmp_path, "mcl", "occupancy_grid.yaml", autoware_setup=None)
    assert ready is False
    extent = next(l for l in lines if l.name == "grid vs lanelet2 extent")
    assert extent.status == "FAIL"
    assert "not in the map frame" in extent.message


def test_run_check_local_missing_tags_cannot_verify(tmp_path):
    """projector_type: Local but lanelet2 lacks local_x/local_y -> must not
    silently pass; must report an explicit failure, not a guess."""
    (tmp_path / "lanelet2_map.osm").write_text(
        "<?xml version='1.0'?><osm><node id='1' lat='25.0' lon='121.5'/></osm>"
    )
    (tmp_path / "map_projector_info.yaml").write_text("projector_type: Local\n")
    write_grid(tmp_path, resolution=1.0, origin=(0.0, 0.0, 0.0),
               pixels=np.full((5, 5), 254, np.uint8))
    lines, ready = run_check(tmp_path, "mcl", "occupancy_grid.yaml", autoware_setup=None)
    assert ready is False
    extent = next(l for l in lines if l.name == "grid vs lanelet2 extent")
    assert extent.status == "FAIL"
    assert "local_x/local_y" in extent.message


def test_run_check_georeferenced_without_autoware_cannot_verify(tmp_path, monkeypatch):
    """Without an Autoware environment to reuse the real projection, a
    georeferenced map's frame check must report cannot-verify and fail --
    never approximate via local_x/local_y tags for a non-Local projector."""
    (tmp_path / "lanelet2_map.osm").write_text(
        OSM_TEMPLATE.format(lx0=0, ly0=0, lx1=10, ly1=10)
    )
    (tmp_path / "map_projector_info.yaml").write_text(
        "projector_type: TransverseMercator\nmap_origin:\n"
        "  latitude: 25.0201\n  longitude: 121.5423\n  altitude: 0.0\n"
    )
    write_grid(tmp_path, resolution=1.0, origin=(0.0, 0.0, 0.0),
               pixels=np.full((5, 5), 254, np.uint8))

    import check_map
    monkeypatch.setattr(check_map, "PROJECTOR_BIN", tmp_path / "no_such_binary")
    lines, ready = run_check(tmp_path, "mcl", "occupancy_grid.yaml", autoware_setup=None)
    assert ready is False
    extent = next(l for l in lines if l.name == "grid vs lanelet2 extent")
    assert extent.status == "FAIL"
    assert "cannot verify" in extent.message


def test_run_check_pcd_not_required_for_mcl(tmp_path):
    make_local_map(tmp_path, grid_origin=(-5.0, -5.0, 0.0))
    lines, _ready = run_check(tmp_path, "mcl", "occupancy_grid.yaml", autoware_setup=None)
    pcd_line = next(l for l in lines if l.name == "pointcloud_map.pcd")
    assert pcd_line.status == "-"
    assert "not required" in pcd_line.message


def test_run_check_pcd_required_for_cuda_ndt_and_missing_fails(tmp_path):
    (tmp_path / "lanelet2_map.osm").write_text(
        OSM_TEMPLATE.format(lx0=0, ly0=0, lx1=10, ly1=10)
    )
    (tmp_path / "map_projector_info.yaml").write_text("projector_type: Local\n")
    lines, ready = run_check(tmp_path, "cuda_ndt", "occupancy_grid.yaml", autoware_setup=None)
    assert ready is False
    pcd_line = next(l for l in lines if l.name == "pointcloud_map.pcd")
    assert pcd_line.status == "FAIL"


def test_run_check_grid_not_required_for_cuda_ndt_when_absent(tmp_path):
    (tmp_path / "lanelet2_map.osm").write_text(
        OSM_TEMPLATE.format(lx0=0, ly0=0, lx1=10, ly1=10)
    )
    (tmp_path / "map_projector_info.yaml").write_text("projector_type: Local\n")
    (tmp_path / "pointcloud_map.pcd").write_bytes(b"fake")
    (tmp_path / "pointcloud_map_metadata.yaml").write_text("x_resolution: 300.0\n")
    lines, ready = run_check(tmp_path, "cuda_ndt", "occupancy_grid.yaml", autoware_setup=None)
    grid_line = next(l for l in lines if l.name == "occupancy_grid.yaml")
    assert grid_line.status == "-"
    assert ready is True


def test_run_check_missing_lanelet2_always_fails(tmp_path):
    (tmp_path / "map_projector_info.yaml").write_text("projector_type: Local\n")
    lines, ready = run_check(tmp_path, "cuda_ndt", "occupancy_grid.yaml", autoware_setup=None)
    assert ready is False
    lanelet_line = next(l for l in lines if l.name == "lanelet2_map.osm")
    assert lanelet_line.status == "FAIL"


def test_run_check_custom_grid_yaml_name(tmp_path):
    """Regression for non-default grid filenames, e.g. sample-map-rosbag's
    occupancy_grid_scanaccum_mh1r05.yaml."""
    (tmp_path / "lanelet2_map.osm").write_text(
        OSM_TEMPLATE.format(lx0=0, ly0=0, lx1=10, ly1=10)
    )
    (tmp_path / "map_projector_info.yaml").write_text("projector_type: Local\n")
    write_grid(tmp_path, name="occupancy_grid_scanaccum_mh1r05",
               resolution=1.0, origin=(-5.0, -5.0, 0.0),
               pixels=np.full((20, 20), 254, np.uint8))
    lines, ready = run_check(
        tmp_path, "mcl", "occupancy_grid_scanaccum_mh1r05.yaml", autoware_setup=None)
    assert ready is True
    grid_line = next(l for l in lines if l.name == "occupancy_grid_scanaccum_mh1r05.yaml")
    assert grid_line.status == "ok"
