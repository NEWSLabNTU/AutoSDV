import sys
from pathlib import Path

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).parent))
import map_sidecar as ms


def test_load_missing_returns_none(tmp_path):
    assert ms.load(tmp_path) is None


def test_load_empty_file_returns_empty_dict(tmp_path):
    ms.sidecar_path(tmp_path).write_text("")
    assert ms.load(tmp_path) == {}


def test_load_rejects_non_mapping(tmp_path):
    ms.sidecar_path(tmp_path).write_text("- just\n- a list\n")
    with pytest.raises(ValueError, match="mapping"):
        ms.load(tmp_path)


def test_build_grid_provenance_pcd_slice():
    block = ms.build_grid_provenance(
        ms.METHOD_PCD_SLICE, "pointcloud_map.pcd", 0.05, z_band=(9.1, 9.4))
    assert block == {
        "method": "pcd_slice",
        "source": "pointcloud_map.pcd",
        "resolution": 0.05,
        "frame_consistent_with_lanelet2": True,
        "z_band": [9.1, 9.4],
    }


def test_build_grid_provenance_omits_z_band_when_absent():
    block = ms.build_grid_provenance(ms.METHOD_SLAM, "survey.bag", 0.05)
    assert "z_band" not in block


def test_build_grid_provenance_rejects_unknown_method():
    with pytest.raises(ValueError, match="unknown provenance method"):
        ms.build_grid_provenance("vibes", "x.pcd", 0.05)


def test_build_grid_provenance_rejects_inverted_band():
    with pytest.raises(ValueError, match="increasing"):
        ms.build_grid_provenance(ms.METHOD_PCD_SLICE, "x.pcd", 0.05,
                                 z_band=(9.4, 9.1))


def test_build_grid_provenance_coerces_numpy_like_scalars():
    """PyYAML emits unreadable tags for numpy scalars, so they must be cast."""
    class FloatLike(float):
        pass

    block = ms.build_grid_provenance(ms.METHOD_PCD_SLICE, "x.pcd",
                                     FloatLike(0.05),
                                     z_band=(FloatLike(1.0), FloatLike(2.0)))
    assert type(block["resolution"]) is float
    assert [type(v) for v in block["z_band"]] == [float, float]


def test_merge_preserves_unrelated_keys_and_geometry():
    existing = {
        "geometry": {"pointcloud": "pointcloud_map.pcd"},
        "site_notes": "surveyed 2026-03",
    }
    out = ms.merge(existing, geometry={"occupancy_grid": "occupancy_grid.yaml"})
    assert out["geometry"] == {
        "pointcloud": "pointcloud_map.pcd",
        "occupancy_grid": "occupancy_grid.yaml",
    }
    assert out["site_notes"] == "surveyed 2026-03"


def test_merge_drops_none_geometry_values():
    out = ms.merge(None, geometry={"pointcloud": None,
                                   "occupancy_grid": "g.yaml"})
    assert out["geometry"] == {"occupancy_grid": "g.yaml"}


def test_merge_replaces_provenance_wholesale():
    existing = {"occupancy_grid_provenance": {"method": "slam",
                                              "stale": "should go"}}
    block = ms.build_grid_provenance(ms.METHOD_PCD_SLICE, "x.pcd", 0.05,
                                     z_band=(1.0, 2.0))
    out = ms.merge(existing, geometry={}, grid_provenance=block)
    assert out["occupancy_grid_provenance"] == block
    assert "stale" not in out["occupancy_grid_provenance"]


def test_save_then_load_round_trips(tmp_path):
    block = ms.build_grid_provenance(ms.METHOD_PCD_SLICE, "pointcloud_map.pcd",
                                     0.05, z_band=(9.1, 9.4))
    path = ms.save(tmp_path,
                   geometry={"pointcloud": "pointcloud_map.pcd",
                             "occupancy_grid": "occupancy_grid.yaml"},
                   grid_provenance=block)
    assert path == ms.sidecar_path(tmp_path)
    reloaded = ms.load(tmp_path)
    assert reloaded["occupancy_grid_provenance"] == block
    assert reloaded["geometry"]["pointcloud"] == "pointcloud_map.pcd"


def test_save_is_idempotent_and_keeps_hand_written_keys(tmp_path):
    ms.sidecar_path(tmp_path).write_text(
        yaml.safe_dump({"site_notes": "keep me"}))
    block = ms.build_grid_provenance(ms.METHOD_SCAN_ACCUMULATION, "run.bag",
                                     0.05, z_band=(-0.15, 0.15))
    ms.save(tmp_path, geometry={"occupancy_grid": "g.yaml"},
            grid_provenance=block)
    first = ms.sidecar_path(tmp_path).read_text()
    ms.save(tmp_path, geometry={"occupancy_grid": "g.yaml"},
            grid_provenance=block)
    assert ms.sidecar_path(tmp_path).read_text() == first
    assert ms.load(tmp_path)["site_notes"] == "keep me"


def test_saved_file_is_plain_yaml_with_a_header(tmp_path):
    ms.save(tmp_path, geometry={"occupancy_grid": "g.yaml"})
    text = ms.sidecar_path(tmp_path).read_text()
    assert text.startswith("# autosdv_map.yaml")
    assert "!!python" not in text


def test_describe_absent_is_empty():
    assert ms.describe(None) == []
    assert ms.describe({}) == []


def test_describe_reports_method_band_and_resolution():
    block = ms.build_grid_provenance(ms.METHOD_PCD_SLICE, "pointcloud_map.pcd",
                                     0.05, z_band=(9.1, 9.4))
    lines = ms.describe({"occupancy_grid_provenance": block})
    assert "pcd_slice" in lines[0]
    assert "pointcloud_map.pcd" in lines[0]
    assert "[9.10, 9.40]" in lines[0]
    assert "0.050 m/px" in lines[0]


def test_describe_flags_unasserted_frame_consistency():
    block = ms.build_grid_provenance(ms.METHOD_SLAM, "s.bag", 0.05,
                                     frame_consistent_with_lanelet2=False)
    lines = ms.describe({"occupancy_grid_provenance": block})
    assert any("did NOT assert frame consistency" in l for l in lines)


def test_describe_tolerates_a_partial_hand_written_sidecar():
    lines = ms.describe({"occupancy_grid_provenance": {"method": "slam"}})
    assert lines and "slam" in lines[0]


def test_describe_tolerates_a_malformed_z_band():
    lines = ms.describe({"occupancy_grid_provenance": {
        "method": "pcd_slice", "source": "x.pcd", "z_band": "nonsense"}})
    assert lines and "nonsense" in lines[0]
