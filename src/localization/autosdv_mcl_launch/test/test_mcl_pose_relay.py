"""Unit tests for mcl_pose_relay's pure, ROS-free conversion functions.

These are plain-Python/numpy functions with no rclpy/Node dependency, so
they are directly testable without a ROS graph:

    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_mcl_pose_relay.py -v

The covariance-remap tests are deliberately term-by-term: the defect being
fixed (docs/design/localization-method-switching.md Sec 5.2 gap 2) is
precisely a mis-indexed copy, so a test that only checks "some values ended
up nonzero somewhere" would not catch a regression back to the wrong
indices.
"""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from autosdv_mcl_launch.mcl_pose_relay import (  # noqa: E402
    PLANAR_SRC_TO_DEST_INDEX,
    UNUSED_AXIS_DIAG_INDICES,
    compose_poses,
    is_planar_quaternion,
    normalize_frame_id,
    quaternion_to_yaw,
    remap_planar_covariance,
    unknown_planar_covariance,
    yaw_to_quaternion,
)


# --- normalize_frame_id ----------------------------------------------------

def test_strips_single_leading_slash():
    assert normalize_frame_id('/map') == 'map'


def test_strips_leading_slash_from_laser_frame():
    assert normalize_frame_id('/laser') == 'laser'


def test_leaves_clean_frame_id_unchanged():
    assert normalize_frame_id('map') == 'map'


def test_strips_repeated_leading_slashes():
    assert normalize_frame_id('//map') == 'map'


def test_does_not_touch_internal_slashes():
    # Frame ids can legitimately contain '/', e.g. tf prefixes; only a
    # *leading* slash is the tf2-rejected form.
    assert normalize_frame_id('/robot/base_link') == 'robot/base_link'


def test_empty_frame_id_stays_empty():
    assert normalize_frame_id('') == ''


# --- remap_planar_covariance: term-by-term destination indices ------------

def _flat9(xx, xy, x_yaw, yx, yy, y_yaw, yaw_x, yaw_y, yaw_yaw):
    return [xx, xy, x_yaw, yx, yy, y_yaw, yaw_x, yaw_y, yaw_yaw]


def test_xx_lands_at_index_0():
    src = _flat9(1.1, 0, 0, 0, 0, 0, 0, 0, 0)
    dest = remap_planar_covariance(src, unused_axis_variance=0.0)
    assert dest[0] == pytest.approx(1.1)


def test_xy_lands_at_index_1():
    src = _flat9(0, 2.2, 0, 0, 0, 0, 0, 0, 0)
    dest = remap_planar_covariance(src, unused_axis_variance=0.0)
    assert dest[1] == pytest.approx(2.2)


def test_x_yaw_lands_at_index_5():
    src = _flat9(0, 0, 3.3, 0, 0, 0, 0, 0, 0)
    dest = remap_planar_covariance(src, unused_axis_variance=0.0)
    assert dest[5] == pytest.approx(3.3)


def test_yx_lands_at_index_6():
    src = _flat9(0, 0, 0, 4.4, 0, 0, 0, 0, 0)
    dest = remap_planar_covariance(src, unused_axis_variance=0.0)
    assert dest[6] == pytest.approx(4.4)


def test_yy_lands_at_index_7():
    src = _flat9(0, 0, 0, 0, 5.5, 0, 0, 0, 0)
    dest = remap_planar_covariance(src, unused_axis_variance=0.0)
    assert dest[7] == pytest.approx(5.5)


def test_y_yaw_lands_at_index_11():
    src = _flat9(0, 0, 0, 0, 0, 6.6, 0, 0, 0)
    dest = remap_planar_covariance(src, unused_axis_variance=0.0)
    assert dest[11] == pytest.approx(6.6)


def test_yaw_x_lands_at_index_30():
    src = _flat9(0, 0, 0, 0, 0, 0, 7.7, 0, 0)
    dest = remap_planar_covariance(src, unused_axis_variance=0.0)
    assert dest[30] == pytest.approx(7.7)


def test_yaw_y_lands_at_index_31():
    src = _flat9(0, 0, 0, 0, 0, 0, 0, 8.8, 0)
    dest = remap_planar_covariance(src, unused_axis_variance=0.0)
    assert dest[31] == pytest.approx(8.8)


def test_yaw_yaw_lands_at_index_35():
    src = _flat9(0, 0, 0, 0, 0, 0, 0, 0, 9.9)
    dest = remap_planar_covariance(src, unused_axis_variance=0.0)
    assert dest[35] == pytest.approx(9.9)


def test_full_matrix_all_nine_destinations_at_once():
    # A single call with all nine terms distinct catches any accidental
    # index collision between the nine mappings (e.g. two source terms
    # aliasing the same destination), which the one-term-at-a-time tests
    # above cannot catch on their own.
    src = _flat9(1, 2, 3, 4, 5, 6, 7, 8, 9)
    dest = remap_planar_covariance(src, unused_axis_variance=0.0)
    expected_at = {
        0: 1, 1: 2, 5: 3,
        6: 4, 7: 5, 11: 6,
        30: 7, 31: 8, 35: 9,
    }
    for idx, value in expected_at.items():
        assert dest[idx] == pytest.approx(value), f'destination index {idx}'
    # every other slot -- including the buggy destinations the original
    # particle_filter code wrote into (indices 2, 3, 4, 8) -- must be
    # exactly zero, not leftover source data.
    for idx in range(36):
        if idx not in expected_at:
            assert dest[idx] == 0.0, f'unexpected nonzero at index {idx}'


def test_original_bug_indices_are_not_reused():
    # The original (buggy) flatten-into-[0:9] placement would have put
    # x_yaw at flat index 2 (the x-z slot) and yx at index 3 (x-roll).
    # Confirm the fixed function does not write source data there.
    src = _flat9(0, 0, 111, 222, 0, 0, 0, 0, 0)
    dest = remap_planar_covariance(src, unused_axis_variance=0.0)
    assert dest[2] == 0.0
    assert dest[3] == 0.0


def test_unused_axis_diagonal_gets_configured_variance():
    src = _flat9(*range(9))
    dest = remap_planar_covariance(src, unused_axis_variance=1_000_000.0)
    for idx in UNUSED_AXIS_DIAG_INDICES:
        assert dest[idx] == pytest.approx(1_000_000.0)
    assert UNUSED_AXIS_DIAG_INDICES == (14, 21, 28)


def test_unused_axis_off_diagonal_terms_stay_zero():
    src = _flat9(*range(9))
    dest = remap_planar_covariance(src, unused_axis_variance=1_000_000.0)
    # z/roll/pitch off-diagonal cross terms (e.g. index 13, z-roll) are not
    # part of the mapping and must remain zero -- only the three diagonal
    # entries get the "unobserved" variance.
    untouched = set(range(36)) - set(PLANAR_SRC_TO_DEST_INDEX.values()) - set(UNUSED_AXIS_DIAG_INDICES)
    for idx in untouched:
        assert dest[idx] == 0.0, f'index {idx} should remain zero'


def test_wrong_length_source_raises():
    with pytest.raises(ValueError):
        remap_planar_covariance([1, 2, 3], unused_axis_variance=1.0)


def test_output_is_always_36_long():
    src = _flat9(*range(9))
    assert len(remap_planar_covariance(src, unused_axis_variance=1.0)) == 36


# --- unknown_planar_covariance ---------------------------------------------

def test_unknown_covariance_sets_all_six_diagonal_terms():
    dest = unknown_planar_covariance(unused_axis_variance=5.0)
    for idx in (0, 7, 35, 14, 21, 28):
        assert dest[idx] == pytest.approx(5.0)


def test_unknown_covariance_off_diagonal_stays_zero():
    dest = unknown_planar_covariance(unused_axis_variance=5.0)
    for idx in range(36):
        if idx not in (0, 7, 35, 14, 21, 28):
            assert dest[idx] == 0.0


# --- yaw / quaternion handling ----------------------------------------------

def test_zero_yaw_is_identity_quaternion():
    x, y, z, w = yaw_to_quaternion(0.0)
    assert (x, y, z) == pytest.approx((0.0, 0.0, 0.0))
    assert abs(w) == pytest.approx(1.0)


def test_yaw_quaternion_roundtrip():
    for yaw in (0.0, 0.3, -1.2, math.pi / 2, -math.pi / 2, 3.0):
        q = yaw_to_quaternion(yaw)
        recovered = quaternion_to_yaw(*q)
        assert recovered == pytest.approx(yaw, abs=1e-9)


def test_quarter_turn_quaternion_has_zero_x_y():
    x, y, z, w = yaw_to_quaternion(math.pi / 2)
    assert x == pytest.approx(0.0, abs=1e-12)
    assert y == pytest.approx(0.0, abs=1e-12)
    assert abs(z) == pytest.approx(math.sin(math.pi / 4))
    assert abs(w) == pytest.approx(math.cos(math.pi / 4))


def test_is_planar_quaternion_true_for_pure_yaw():
    q = yaw_to_quaternion(1.0)
    assert is_planar_quaternion(q)


def test_is_planar_quaternion_false_for_roll():
    # A pure 90-degree roll quaternion, in ROS xyzw order.
    q = (math.sin(math.pi / 4), 0.0, 0.0, math.cos(math.pi / 4))
    assert not is_planar_quaternion(q)


# --- compose_poses -----------------------------------------------------------

_IDENTITY_POSE = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))


def test_compose_with_identity_returns_other_pose():
    pose = ((1.0, 2.0, 0.0), yaw_to_quaternion(0.5))
    result = compose_poses(_IDENTITY_POSE, pose)
    t, q = result
    assert t == pytest.approx(pose[0])
    assert q == pytest.approx(pose[1])


def test_compose_pure_translation_adds_offsets():
    pose_a = ((1.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    pose_b = ((0.0, 2.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    t, q = compose_poses(pose_a, pose_b)
    assert t == pytest.approx((1.0, 2.0, 0.0))
    assert q == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_compose_rotates_child_translation_into_parent_frame():
    # pose_a: robot at origin, rotated 90 degrees (yaw) counter-clockwise.
    # pose_b: sensor offset +1m along the (rotated) local x-axis.
    # Expected: the sensor ends up at (0, 1, 0) in the parent frame, since
    # the local +x axis now points along the parent's +y axis.
    pose_a = ((0.0, 0.0, 0.0), yaw_to_quaternion(math.pi / 2))
    pose_b = ((1.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    t, q = compose_poses(pose_a, pose_b)
    assert t == pytest.approx((0.0, 1.0, 0.0), abs=1e-9)
    assert quaternion_to_yaw(*q) == pytest.approx(math.pi / 2)


def test_compose_yaws_add_for_two_yaw_only_poses():
    pose_a = ((0.0, 0.0, 0.0), yaw_to_quaternion(0.4))
    pose_b = ((0.0, 0.0, 0.0), yaw_to_quaternion(0.3))
    _t, q = compose_poses(pose_a, pose_b)
    assert quaternion_to_yaw(*q) == pytest.approx(0.7)


def test_compose_known_base_link_laser_offset():
    # A concrete map -> base_link recovery: MCL reports the *laser* pose at
    # (5, 5) facing +90deg yaw; the laser sits 0.2m ahead of base_link
    # along the vehicle's forward (local +x) axis. base_link should end up
    # 0.2m in -y from the laser position once rotated into the map frame.
    pose_map_laser = ((5.0, 5.0, 0.0), yaw_to_quaternion(math.pi / 2))
    laser_to_base = ((-0.2, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    t, q = compose_poses(pose_map_laser, laser_to_base)
    assert t == pytest.approx((5.0, 4.8, 0.0), abs=1e-9)
    assert quaternion_to_yaw(*q) == pytest.approx(math.pi / 2)


def test_compose_matches_manual_rotation_matrix(monkeypatch=None):
    rng = np.random.default_rng(0)
    yaw_a = float(rng.uniform(-math.pi, math.pi))
    yaw_b = float(rng.uniform(-math.pi, math.pi))
    t_a = tuple(rng.uniform(-3, 3, size=3))
    t_b = tuple(rng.uniform(-3, 3, size=3))

    pose_a = (t_a, yaw_to_quaternion(yaw_a))
    pose_b = (t_b, yaw_to_quaternion(yaw_b))
    t, q = compose_poses(pose_a, pose_b)

    c, s = math.cos(yaw_a), math.sin(yaw_a)
    expected_t = (
        t_a[0] + c * t_b[0] - s * t_b[1],
        t_a[1] + s * t_b[0] + c * t_b[1],
        t_a[2] + t_b[2],
    )
    assert t == pytest.approx(expected_t, abs=1e-9)
    assert quaternion_to_yaw(*q) == pytest.approx(
        math.atan2(math.sin(yaw_a + yaw_b), math.cos(yaw_a + yaw_b)), abs=1e-9)
