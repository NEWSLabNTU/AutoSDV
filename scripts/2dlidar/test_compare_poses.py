#!/usr/bin/env python3
"""Tests for compare_poses.py align_tracks/stats — plain-shell, no ROS deps.

Run: PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest scripts/2dlidar/test_compare_poses.py -v
"""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent))
from compare_poses import (  # noqa: E402
    align_tracks, stats, yaw_error, quat_to_yaw, _threshold_table, generate_report,
    read_pf_bag, PF_TYPE_TO_MSG_FIELD,
)


def test_align_tracks_nearest_timestamp_pairing():
    gt = [(0.0, 0.0, 0.0, 0.0), (1.0, 1.0, 0.0, 0.0), (2.0, 2.0, 0.0, 0.0)]
    pf = [(0.02, 0.1, 0.0, 0.0), (1.05, 1.2, 0.0, 0.0), (2.01, 2.3, 0.0, 0.0)]
    errors = align_tracks(gt, pf, max_dt=0.1)
    assert len(errors) == 3
    trans_errs = [e[0] for e in errors]
    assert trans_errs[0] == pytest.approx(0.1, abs=1e-9)
    assert trans_errs[1] == pytest.approx(0.2, abs=1e-9)
    assert trans_errs[2] == pytest.approx(0.3, abs=1e-9)


def test_align_tracks_rejects_pairs_beyond_max_dt():
    gt = [(0.0, 0.0, 0.0, 0.0), (5.0, 5.0, 0.0, 0.0)]
    pf = [(0.02, 0.1, 0.0, 0.0), (5.5, 5.5, 0.0, 0.0)]  # second pf pt dt=0.5 > max_dt
    errors = align_tracks(gt, pf, max_dt=0.1)
    assert len(errors) == 1


def test_align_tracks_empty_overlap_raises():
    gt = [(0.0, 0.0, 0.0, 0.0)]
    pf = [(100.0, 0.0, 0.0, 0.0)]
    with pytest.raises(ValueError):
        align_tracks(gt, pf, max_dt=0.1)


def test_align_tracks_empty_input_raises():
    with pytest.raises(ValueError):
        align_tracks([], [], max_dt=0.1)


def test_stats_exact_known_list():
    # translational errors: 1, 2, 3, 4 ; yaw errors: 0.1, -0.2, 0.3, -0.4
    errors = [(1.0, 0.1), (2.0, -0.2), (3.0, 0.3), (4.0, -0.4)]
    result = stats(errors)
    assert result["trans_mean"] == pytest.approx(2.5)
    assert result["trans_rms"] == pytest.approx(math.sqrt((1 + 4 + 9 + 16) / 4))
    assert result["trans_max"] == pytest.approx(4.0)
    # p95 of [1,2,3,4] via linear interpolation (numpy default)
    assert result["trans_p95"] == pytest.approx(3.85)
    assert result["yaw_mean_abs"] == pytest.approx((0.1 + 0.2 + 0.3 + 0.4) / 4)
    assert result["yaw_max_abs"] == pytest.approx(0.4)


def test_stats_empty_raises():
    with pytest.raises(ValueError):
        stats([])


def test_yaw_error_wraparound_near_pi():
    # gt yaw near +pi, pf yaw near -pi -> true difference should be small, not ~2pi
    err = yaw_error(math.pi - 0.05, -math.pi + 0.05)
    assert abs(err) == pytest.approx(0.1, abs=1e-9)


def test_yaw_error_normalized_to_pi_range():
    err = yaw_error(0.0, math.pi + 0.1)
    assert -math.pi <= err <= math.pi
    assert abs(err) == pytest.approx(math.pi - 0.1, abs=1e-9)


def test_quat_to_yaw_identity():
    assert quat_to_yaw(0.0, 0.0, 0.0, 1.0) == pytest.approx(0.0, abs=1e-9)


def test_quat_to_yaw_90deg():
    # 90 deg rotation about z: qz=sin(45deg), qw=cos(45deg)
    qz = math.sin(math.pi / 4)
    qw = math.cos(math.pi / 4)
    assert quat_to_yaw(0.0, 0.0, qz, qw) == pytest.approx(math.pi / 2, abs=1e-9)


def test_threshold_table_yaw_row_escapes_pipes():
    # The yaw row's own label must not contain unescaped "|" — those
    # break the 4-column markdown table (regression: previously
    # rendered as 6 cells instead of 4).
    fake_stats = {
        "trans_mean": 0.5,
        "trans_p95": 1.0,
        "yaw_mean_abs": 0.1,
    }
    table, _ = _threshold_table(fake_stats)
    yaw_row = [line for line in table.splitlines() if "yaw" in line.lower()][0]
    # Pipes inside the cell label must be escaped ("\|") so a markdown
    # renderer treats them as literal text, not column separators —
    # otherwise this row renders with 6 cells instead of 4.
    assert "\\|yaw\\|" in yaw_row


def test_generate_report_no_motion_window_omits_coss_window():
    # --no-motion-window (motion_window_cfg=None) must drop the COSS-tuned
    # motion-window section entirely -- for a bag that moves throughout
    # (e.g. the sample-site bag), the fixed 116s-156s split is meaningless
    # and must not appear in the report, regardless of default behavior.
    fake_stats = {
        "n": 4, "trans_mean": 0.5, "trans_rms": 0.5, "trans_max": 0.5,
        "trans_p95": 0.5, "yaw_mean_abs": 0.1, "yaw_max_abs": 0.1,
    }
    report = generate_report(
        "gt_bag", "pf_bag", fake_stats, motion_stats=None, motion_window_actual=None,
        all_pass=True, notes="n/a", motion_window_cfg=None,
    )
    assert "116" not in report
    assert "156" not in report
    assert "Motion-Window Statistics" in report
    assert "disabled" in report.lower()


def test_read_pf_bag_default_topic_and_type_unchanged(monkeypatch):
    # Default args must reach _read_bag exactly as before --pf-topic/--pf-type
    # existed (Phase 3c Lever 4 addition) -- particle_filter callers see no
    # behavior change.
    captured = {}

    def fake_read_bag(bag_path, topic, msg_field, time_source="stamp"):
        captured["bag_path"] = bag_path
        captured["topic"] = topic
        captured["msg_field"] = msg_field
        captured["time_source"] = time_source
        return []

    import compare_poses
    monkeypatch.setattr(compare_poses, "_read_bag", fake_read_bag)
    read_pf_bag("some_bag")
    assert captured == {
        "bag_path": "some_bag",
        "topic": "/pf/viz/inferred_pose",
        "msg_field": "pose_stamped",
        "time_source": "stamp",
    }


def test_read_pf_bag_amcl_pose_with_covariance(monkeypatch):
    # Phase 3c Lever 4: nav2_amcl publishes /amcl_pose as
    # geometry_msgs/PoseWithCovarianceStamped, not PoseStamped.
    captured = {}

    def fake_read_bag(bag_path, topic, msg_field, time_source="stamp"):
        captured["topic"] = topic
        captured["msg_field"] = msg_field
        return []

    import compare_poses
    monkeypatch.setattr(compare_poses, "_read_bag", fake_read_bag)
    read_pf_bag("some_bag", topic="/amcl_pose", pf_type="PoseWithCovarianceStamped")
    assert captured["topic"] == "/amcl_pose"
    assert captured["msg_field"] == "pose_with_covariance_stamped"


def test_read_pf_bag_unknown_type_raises():
    with pytest.raises(ValueError):
        read_pf_bag("some_bag", pf_type="NotAType")


def test_pf_type_to_msg_field_covers_odometry_too():
    # Odometry (nav_msgs/Odometry) was already a supported msg_field for GT
    # bags before this change; confirm the new PF_TYPE_TO_MSG_FIELD mapping
    # exposes it too, for parity/future flexibility.
    assert PF_TYPE_TO_MSG_FIELD["Odometry"] == "odometry"
    assert PF_TYPE_TO_MSG_FIELD["PoseStamped"] == "pose_stamped"


def test_generate_report_default_motion_window_unchanged():
    # Default (motion_window_cfg omitted / COSS bounds) behavior must be
    # byte-identical to before the --no-motion-window flag was added, so
    # the COSS run reproduces unchanged.
    fake_stats = {
        "n": 4, "trans_mean": 0.5, "trans_rms": 0.5, "trans_max": 0.5,
        "trans_p95": 0.5, "yaw_mean_abs": 0.1, "yaw_max_abs": 0.1,
    }
    report = generate_report(
        "gt_bag", "pf_bag", fake_stats, motion_stats=None, motion_window_actual=None,
        all_pass=True, notes="n/a",
    )
    assert "116s" in report
    assert "156s" in report
    assert "configured, source-bag sim time" in report
