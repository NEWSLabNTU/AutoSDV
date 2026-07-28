"""Tests for the scan normaliser's geometry.

The normaliser exists because `particle_filter.py` treats a scan as originating
at the particle pose: a scan published in a laser frame mounted forward of
`base_link` biases every range by the mounting offset, and the filter cannot
tell that from a pose error. These tests pin the re-targeting geometry that
removes the bias.
"""
import math

import pytest

from autosdv_mcl_launch.scan_normalizer import (
    ScanSpec,
    is_identity,
    planar_offset,
    retarget_ranges,
)

FULL = ScanSpec(-math.pi, math.pi, 0.01, 0.1, 60.0)


def _single_beam(spec: ScanSpec, bearing: float, r: float):
    """Ranges with one finite return near `bearing`, plus the bearing actually used.

    No bin lands exactly on a requested bearing in general -- with angle_min=-pi
    and a 0.01 increment the nearest bin to zero sits at -0.00159 rad -- so the
    quantised bearing is returned and expectations are derived from it. Loosening
    tolerances instead would hide a real geometry error inside the slack.
    """
    n = spec.bin_count()
    ranges = [math.inf] * n
    idx = int(round((bearing - spec.angle_min) / spec.angle_increment))
    ranges[idx] = r
    return ranges, spec.angle_min + idx * spec.angle_increment


def _expected(r: float, theta: float, dx: float, dy: float, dyaw: float):
    """Range and bearing of one beam after the planar re-target, computed directly."""
    px, py = r * math.cos(theta), r * math.sin(theta)
    bx = math.cos(dyaw) * px - math.sin(dyaw) * py + dx
    by = math.sin(dyaw) * px + math.cos(dyaw) * py + dy
    return math.hypot(bx, by), math.atan2(by, bx)


def _finite(out):
    return [r for r in out if math.isfinite(r)]


def _bearing_of_min(out, spec):
    idx = min((i for i, r in enumerate(out) if math.isfinite(r)),
              key=lambda i: out[i])
    return spec.angle_min + idx * spec.angle_increment


# ---------------------------------------------------------------------------
# Identity
# ---------------------------------------------------------------------------


def test_identity_offset_preserves_ranges():
    ranges = [5.0] * FULL.bin_count()
    out = retarget_ranges(ranges, FULL.angle_min, FULL.angle_increment,
                          (0.0, 0.0, 0.0, 0.0), FULL)
    assert _finite(out)
    assert all(abs(r - 5.0) < 1e-6 for r in _finite(out))


def test_is_identity_thresholds():
    assert is_identity((0.0, 0.0, 0.0, 0.0))
    assert is_identity((0.0005, -0.0005, 0.001, 0.0))
    assert not is_identity((0.5, 0.0, 0.0, 0.0))
    assert not is_identity((0.0, 0.0, 0.05, 0.0))


# ---------------------------------------------------------------------------
# Translation
# ---------------------------------------------------------------------------


def test_forward_mount_adds_to_a_forward_return():
    """Sensor 1 m ahead of base_link: a return 5 m ahead of it is ~6 m from base_link."""
    ranges, theta = _single_beam(FULL, 0.0, 5.0)
    out = retarget_ranges(ranges, FULL.angle_min, FULL.angle_increment,
                          (1.0, 0.0, 0.0, 0.0), FULL)
    exp_r, _ = _expected(5.0, theta, 1.0, 0.0, 0.0)
    assert min(_finite(out)) == pytest.approx(exp_r, abs=1e-9)
    assert exp_r == pytest.approx(6.0, abs=1e-4)


def test_forward_mount_subtracts_from_a_rearward_return():
    ranges, theta = _single_beam(FULL, math.pi, 5.0)
    out = retarget_ranges(ranges, FULL.angle_min, FULL.angle_increment,
                          (1.0, 0.0, 0.0, 0.0), FULL)
    exp_r, _ = _expected(5.0, theta, 1.0, 0.0, 0.0)
    assert min(_finite(out)) == pytest.approx(exp_r, abs=1e-9)
    assert exp_r == pytest.approx(4.0, abs=1e-4)


def test_lateral_mount_moves_bearing_off_axis():
    """A sensor 1 m to port sees a forward return at a bearing north of zero."""
    ranges, theta = _single_beam(FULL, 0.0, 5.0)
    out = retarget_ranges(ranges, FULL.angle_min, FULL.angle_increment,
                          (0.0, 1.0, 0.0, 0.0), FULL)
    exp_r, exp_b = _expected(5.0, theta, 0.0, 1.0, 0.0)
    assert min(_finite(out)) == pytest.approx(exp_r, abs=1e-9)
    assert _bearing_of_min(out, FULL) == pytest.approx(exp_b, abs=FULL.angle_increment)
    assert exp_b > 0.0


# ---------------------------------------------------------------------------
# Rotation
# ---------------------------------------------------------------------------


def test_yaw_rotates_bearing():
    """A sensor yawed +90 deg puts its forward beam on base_link's +y."""
    ranges, _ = _single_beam(FULL, 0.0, 5.0)
    out = retarget_ranges(ranges, FULL.angle_min, FULL.angle_increment,
                          (0.0, 0.0, math.pi / 2, 0.0), FULL)
    assert min(_finite(out)) == pytest.approx(5.0, abs=1e-6)
    assert _bearing_of_min(out, FULL) == pytest.approx(math.pi / 2, abs=0.02)


def test_yaw_alone_preserves_range():
    """Rotation about the origin cannot change a distance."""
    ranges = [7.0] * FULL.bin_count()
    out = retarget_ranges(ranges, FULL.angle_min, FULL.angle_increment,
                          (0.0, 0.0, 1.0, 0.0), FULL)
    assert all(abs(r - 7.0) < 1e-6 for r in _finite(out))


def test_translation_and_yaw_compose_in_the_right_order():
    """Rotate the beam first, then translate: R*p + t, not p + t rotated."""
    ranges, theta = _single_beam(FULL, 0.0, 5.0)
    out = retarget_ranges(ranges, FULL.angle_min, FULL.angle_increment,
                          (1.0, 0.0, math.pi / 2, 0.0), FULL)
    exp_r, _ = _expected(5.0, theta, 1.0, 0.0, math.pi / 2)
    assert min(_finite(out)) == pytest.approx(exp_r, abs=1e-9)
    # R*p + t puts the beam at ~(1, 5), range ~5.10. The wrong order, t then R,
    # would put it at ~(0, 6), range ~6.0. Assert we are near the former and far
    # from the latter, which pins the order rather than merely the magnitude.
    right_order = math.hypot(1.0, 5.0)
    wrong_order = 6.0
    assert abs(exp_r - right_order) < 0.01
    assert abs(exp_r - wrong_order) > 0.5


# ---------------------------------------------------------------------------
# Binning and filtering
# ---------------------------------------------------------------------------


def test_nearer_return_wins_a_shared_bin():
    """Occlusion: the closer obstacle is the one that is visible."""
    coarse = ScanSpec(-math.pi, math.pi, 0.5, 0.1, 60.0)
    ranges = [10.0, 3.0] + [math.inf] * 11
    out = retarget_ranges(ranges, -math.pi, 0.02, (0.0, 0.0, 0.0, 0.0), coarse)
    assert min(_finite(out)) == pytest.approx(3.0, abs=1e-6)


def test_ranges_outside_the_output_window_are_dropped():
    spec = ScanSpec(-math.pi, math.pi, 0.01, 1.0, 10.0)
    out = retarget_ranges([0.5, 50.0], -math.pi, 0.01, (0.0, 0.0, 0.0, 0.0), spec)
    assert not _finite(out)


def test_non_finite_and_negative_inputs_are_ignored():
    out = retarget_ranges([float("nan"), math.inf, -1.0, 0.0],
                          -math.pi, 0.01, (0.0, 0.0, 0.0, 0.0), FULL)
    assert not _finite(out)


def test_output_length_matches_the_spec():
    out = retarget_ranges([5.0] * 100, -math.pi, 0.01,
                          (0.5, 0.0, 0.0, 0.0), FULL)
    assert len(out) == FULL.bin_count()


def test_partial_window_drops_out_of_window_bearings():
    """A 180 deg output must not wrap a rear return round to the front."""
    half = ScanSpec(-math.pi / 2, math.pi / 2, 0.01, 0.1, 60.0)
    ranges, _ = _single_beam(FULL, math.pi, 5.0)       # dead astern
    out = retarget_ranges(ranges, FULL.angle_min, FULL.angle_increment,
                          (0.0, 0.0, 0.0, 0.0), half)
    assert not _finite(out)


def test_full_window_wraps_rather_than_dropping():
    """With a 360 deg output, a yaw must not push beams off the end."""
    ranges = [5.0] * FULL.bin_count()
    out = retarget_ranges(ranges, FULL.angle_min, FULL.angle_increment,
                          (0.0, 0.0, 3.0, 0.0), FULL)
    # nearly every bin should still be populated after a 3 rad yaw
    assert len(_finite(out)) > 0.9 * FULL.bin_count()


def test_zero_increment_is_rejected():
    with pytest.raises(ValueError):
        ScanSpec(-math.pi, math.pi, 0.0, 0.1, 60.0).bin_count()


# ---------------------------------------------------------------------------
# planar_offset
# ---------------------------------------------------------------------------


def test_planar_offset_extracts_translation_and_yaw():
    q = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))   # yaw 90 deg
    dx, dy, dyaw, tilt = planar_offset((1.5, -0.25, 2.0), q)
    assert (dx, dy) == pytest.approx((1.5, -0.25))
    assert dyaw == pytest.approx(math.pi / 2, abs=1e-9)
    assert tilt == pytest.approx(0.0, abs=1e-9)


def test_planar_offset_reports_roll_as_tilt():
    """Roll cannot be represented planar-ly, so the caller must be told."""
    half = math.radians(5.0)
    q = (math.sin(half), 0.0, 0.0, math.cos(half))                 # roll 10 deg
    _, _, _, tilt = planar_offset((0.0, 0.0, 0.0), q)
    assert tilt == pytest.approx(math.radians(10.0), abs=1e-6)


def test_planar_offset_reports_pitch_as_tilt():
    half = math.radians(7.5)
    q = (0.0, math.sin(half), 0.0, math.cos(half))                 # pitch 15 deg
    _, _, _, tilt = planar_offset((0.0, 0.0, 0.0), q)
    assert tilt == pytest.approx(math.radians(15.0), abs=1e-6)


def test_yaw_only_is_not_reported_as_tilt():
    """The sample kit's velodyne_top has yaw 1.575 and no tilt; it must pass."""
    q = (0.0, 0.0, math.sin(1.575 / 2), math.cos(1.575 / 2))
    _, _, dyaw, tilt = planar_offset((0.0, 0.0, 0.0), q)
    assert dyaw == pytest.approx(1.575, abs=1e-6)
    assert tilt == pytest.approx(0.0, abs=1e-9)


# ---------------------------------------------------------------------------
# _once severity dispatch
# ---------------------------------------------------------------------------


class _FakeLogger:
    """Mimics rclpy's per-call-site severity caching.

    rclpy raises "Logger severity cannot be changed between calls" when one
    source line logs at two different severities. This fake reproduces that
    rule so the dispatch can be tested without a ROS context; the real failure
    killed the normaliser's subscription callback on its first scan.
    """

    def __init__(self):
        self.calls = []
        self._site_severity = {}

    def _log(self, severity, message, site):
        prev = self._site_severity.setdefault(site, severity)
        if prev != severity:
            raise ValueError("Logger severity cannot be changed between calls.")
        self.calls.append((severity, message))

    def info(self, message):
        self._log("info", message, "info_site")

    def warning(self, message):
        self._log("warning", message, "warning_site")

    def error(self, message):
        self._log("error", message, "error_site")


def _once_dispatch(logger, said, key, level, message):
    """Mirror of ScanNormalizer._once, kept in step with the node."""
    if key in said:
        return
    said.add(key)
    if level == "error":
        logger.error(message)
    elif level == "warning":
        logger.warning(message)
    else:
        logger.info(message)


def test_mixed_severities_do_not_raise():
    """error then info must not trip rclpy's per-call-site severity rule."""
    logger, said = _FakeLogger(), set()
    _once_dispatch(logger, said, "tf", "error", "no transform")
    _once_dispatch(logger, said, "identity", "info", "passing through")
    _once_dispatch(logger, said, "tilt", "warning", "tilted")
    assert [c[0] for c in logger.calls] == ["error", "info", "warning"]


def test_once_suppresses_repeats():
    logger, said = _FakeLogger(), set()
    for _ in range(5):
        _once_dispatch(logger, said, "tf", "error", "no transform")
    assert len(logger.calls) == 1
