#!/usr/bin/env python3
"""Tests for scan_accumulate_grid.py's pure accumulate()/transform helpers.

Plain-shell, no ROS deps.
Run: PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest scripts/2dlidar/test_scan_accumulate_grid.py -v
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent))
from scan_accumulate_grid import (  # noqa: E402
    accumulate,
    transform_xy,
    quat_to_yaw,
    compose_static_chain,
    OCCUPIED,
    FREE,
)


def test_accumulate_two_scan_wall_occupied_where_hits_meet_threshold():
    # Both scans see the same three "wall" points -> count=2 at those cells.
    scan1 = np.array([[0.5, 0.5], [1.5, 0.5], [2.5, 0.5]])
    scan2 = np.array([[0.5, 0.5], [1.5, 0.5], [2.5, 0.5]])
    grid, origin = accumulate([scan1, scan2], resolution=1.0, min_hits=2)

    ox, oy = origin
    # origin = data_min - 5m margin = 0.5 - 5.0 = -4.5
    assert ox == pytest.approx(-4.5)
    assert oy == pytest.approx(-4.5)

    def cell(x, y):
        ci = int((x - ox) / 1.0)
        ri = int((y - oy) / 1.0)
        return ri, ci

    for x in (0.5, 1.5, 2.5):
        ri, ci = cell(x, 0.5)
        assert grid[ri, ci] == OCCUPIED

    # A far-away cell with zero hits must be free, not occupied/unknown.
    assert grid[0, 0] == FREE
    # No unknown (205) cells anywhere -- free everywhere except occupied hits.
    assert set(np.unique(grid).tolist()) <= {OCCUPIED, FREE}


def test_accumulate_min_hits_filtering():
    # Single scan -> every populated cell has count=1, below min_hits=2.
    scan1 = np.array([[0.5, 0.5], [1.5, 0.5], [2.5, 0.5]])
    grid, origin = accumulate([scan1], resolution=1.0, min_hits=2)
    assert not (grid == OCCUPIED).any()
    assert (grid == FREE).all()


def test_accumulate_min_hits_one_marks_any_hit_occupied():
    scan1 = np.array([[0.5, 0.5]])
    grid, origin = accumulate([scan1], resolution=1.0, min_hits=1)
    assert (grid == OCCUPIED).any()


def test_accumulate_empty_list_raises():
    with pytest.raises(ValueError):
        accumulate([], resolution=0.1, min_hits=3)


def test_accumulate_all_empty_arrays_raises():
    with pytest.raises(ValueError):
        accumulate([np.zeros((0, 2)), np.zeros((0, 2))], resolution=0.1, min_hits=3)


def test_transform_xy_pure_translation():
    pts = np.array([[1.0, 0.0], [0.0, 1.0]])
    out = transform_xy(pts, tx=10.0, ty=20.0, yaw=0.0)
    np.testing.assert_allclose(out, [[11.0, 20.0], [10.0, 21.0]])


def test_transform_xy_quarter_turn_rotation():
    pts = np.array([[1.0, 0.0]])
    out = transform_xy(pts, tx=0.0, ty=0.0, yaw=math.pi / 2)
    np.testing.assert_allclose(out, [[0.0, 1.0]], atol=1e-9)


def test_quat_to_yaw_identity_is_zero():
    assert quat_to_yaw(0.0, 0.0, 0.0, 1.0) == pytest.approx(0.0)


def test_quat_to_yaw_ninety_degrees():
    # q = (0, 0, sin(45deg), cos(45deg)) is a +90deg yaw rotation.
    q = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    assert quat_to_yaw(*q) == pytest.approx(math.pi / 2)


def test_compose_static_chain_two_hops():
    # frame C -> frame B: +90deg yaw, translate (1, 0, 0)
    # frame B -> frame A (root): translate (0, 5, 0), no rotation
    # Composed C -> A: rotate point by +90deg then translate by
    # R_b*(t_c) + t_b = (0,1,0)... let's just check numerically.
    chain = [
        ((1.0, 0.0, 0.0), (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))),
        ((0.0, 5.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
    ]
    tx, ty, yaw = compose_static_chain(chain)
    assert yaw == pytest.approx(math.pi / 2)
    # R_b(identity) * t_c=(1,0,0) + t_b=(0,5,0) = (1,5,0)
    assert tx == pytest.approx(1.0)
    assert ty == pytest.approx(5.0)
