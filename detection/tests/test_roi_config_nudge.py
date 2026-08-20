# -*- coding: utf-8 -*-
"""델타 기반 조정. 콘솔이 키를, 데몬이 값을 들고 있어서 필요하다."""

import pytest

from roi_config import RoiConfig, nudge_band_by, nudge_target_by


def make_roi(near=0.14, far=0.20, target=0.0):
    return RoiConfig(x=0, y=0, w=10, h=10, near_m=near, far_m=far,
                     target_angle_deg=target)


def test_near_moves_by_the_delta():
    roi = make_roi()
    ok, _ = nudge_band_by(roi, "near", -0.005)
    assert ok and roi.near_m == pytest.approx(0.135)


def test_far_moves_by_the_delta():
    roi = make_roi()
    ok, _ = nudge_band_by(roi, "far", +0.005)
    assert ok and roi.far_m == pytest.approx(0.205)


def test_inverting_the_band_is_refused():
    """near >= far 가 되면 그 뒤 모든 판정이 조용히 0 이 된다."""
    roi = make_roi(near=0.14, far=0.145)
    ok, msg = nudge_band_by(roi, "near", +0.01)
    assert ok is False
    assert "뒤집" in msg
    assert roi.near_m == pytest.approx(0.14)     # 안 바뀌었다


def test_unknown_edge_is_refused():
    ok, msg = nudge_band_by(make_roi(), "middle", 0.005)
    assert ok is False and "middle" in msg


def test_target_wraps_past_ninety():
    """감지 않으면 validate() 가 막아 조용히 저장이 실패한다."""
    roi = make_roi(target=90.0)
    ok, _ = nudge_target_by(roi, +1.0)
    assert ok and roi.target_angle_deg == pytest.approx(-89.0)


def test_target_moves_both_ways():
    roi = make_roi(target=10.0)
    nudge_target_by(roi, -1.0)
    assert roi.target_angle_deg == pytest.approx(9.0)
