# -*- coding: utf-8 -*-
"""명령 디스패치. 소켓도 카메라도 모터도 없다."""

import numpy as np
import pytest

from grasp_commands import CommandHandler, FrameState
from link import PROTO
from roi_config import RoiConfig


class FakeMachine:
    def __init__(self):
        self.calls = []
        self.armed = False
        self.align_enabled = True
        self.state = "ARMED"

    def arm(self):
        self.armed = True
        self.calls.append("arm")

    def disarm(self):
        self.armed = False
        self.calls.append("disarm")

    def set_align_enabled(self, on):
        self.align_enabled = bool(on)
        self.calls.append(f"align={bool(on)}")

    def request_release(self):
        self.calls.append("release")
        return self.state == "HOLDING"

    def emergency_open(self):
        self.calls.append("emergency_open")


def make(roi=None, median=0.183, angle=8.2, saved=None):
    state = FrameState()
    state.roi = roi if roi is not None else RoiConfig(
        x=0, y=0, w=10, h=10, near_m=0.14, far_m=0.20)
    state.median = median
    state.angle = angle
    state.angle_reason = "" if angle is not None else "too few pixels"
    state.band_mask = np.ones((10, 10), dtype=bool)
    state.machine = FakeMachine()
    state.saves = [] if saved is None else saved
    state.on_roi_saved = state.saves.append
    return CommandHandler(state), state


def test_hello_accepts_the_matching_version():
    h, _ = make()
    assert h.handle({"cmd": "hello", "proto": PROTO})["ok"] is True


def test_hello_rejects_a_different_version():
    h, _ = make()
    ack = h.handle({"cmd": "hello", "proto": PROTO + 1})
    assert ack["ok"] is False and str(PROTO) in ack["msg"]


def test_unknown_command_is_refused_not_crashed():
    h, _ = make()
    ack = h.handle({"cmd": "self_destruct"})
    assert ack["ok"] is False and "self_destruct" in ack["msg"]


def test_arm_and_disarm():
    h, s = make()
    assert h.handle({"cmd": "arm"})["ok"] is True
    assert s.machine.armed is True
    h.handle({"cmd": "disarm"})
    assert s.machine.armed is False


def test_set_roi_replaces_the_box_and_keeps_the_band():
    h, s = make()
    ack = h.handle({"cmd": "set_roi", "x": 5, "y": 6, "w": 20, "h": 30})
    assert ack["ok"] is True
    assert (s.roi.x, s.roi.y, s.roi.w, s.roi.h) == (5, 6, 20, 30)
    assert s.roi.near_m == pytest.approx(0.14)      # 밴드는 유지된다
    assert len(s.saves) == 1


def test_set_roi_rejects_a_zero_sized_box():
    h, s = make()
    ack = h.handle({"cmd": "set_roi", "x": 5, "y": 6, "w": 0, "h": 30})
    assert ack["ok"] is False
    assert s.roi.w == 10                            # 안 바뀌었다


def test_calib_band_uses_the_daemons_own_median():
    """PC 가 보낸 값이 아니라 지금 이 프레임의 median 을 쓴다.

    PC 의 median 은 이미 100ms 낡았다. 그 값으로 캘리브레이션하면
    조용히 틀린다.
    """
    h, s = make(median=0.183)
    h.handle({"cmd": "calib_band", "edge": "near", "near_m": 999.0})
    assert s.roi.near_m == pytest.approx(0.183 - 0.01)   # MARGIN_M


def test_calib_band_far_adds_the_margin_outward():
    h, s = make(median=0.183)
    h.handle({"cmd": "calib_band", "edge": "far"})
    assert s.roi.far_m == pytest.approx(0.183 + 0.01)


def test_calib_band_refuses_when_there_is_no_depth():
    h, s = make(median=None)
    ack = h.handle({"cmd": "calib_band", "edge": "near"})
    assert ack["ok"] is False
    assert s.roi.near_m == pytest.approx(0.14)


def test_capture_target_uses_the_daemons_own_angle():
    h, s = make(angle=8.2)
    ack = h.handle({"cmd": "capture_target"})
    assert ack["ok"] is True
    assert s.roi.target_angle_deg == pytest.approx(8.2)


def test_capture_target_refuses_when_the_angle_is_unknown():
    h, s = make(angle=None)
    ack = h.handle({"cmd": "capture_target"})
    assert ack["ok"] is False and "too few pixels" in ack["msg"]


def test_nudge_target_moves_relative():
    h, s = make()
    s.roi.target_angle_deg = 10.0
    h.handle({"cmd": "nudge_target", "delta_deg": 1.0})
    assert s.roi.target_angle_deg == pytest.approx(11.0)


def test_nudge_band_refuses_to_invert():
    h, s = make()
    s.roi.near_m, s.roi.far_m = 0.14, 0.145
    ack = h.handle({"cmd": "nudge_band", "edge": "near", "delta_m": 0.01})
    assert ack["ok"] is False


def test_save_hand_mask_uses_the_current_band():
    h, s = make()
    written = []
    s.on_save_hand_mask = written.append
    ack = h.handle({"cmd": "save_hand_mask"})
    assert ack["ok"] is True
    assert written and written[0].shape == (10, 10)


def test_release_is_refused_when_not_holding():
    h, s = make()
    s.machine.state = "ARMED"
    assert h.handle({"cmd": "release"})["ok"] is False


def test_emergency_open_always_works():
    h, s = make()
    assert h.handle({"cmd": "emergency_open"})["ok"] is True
    assert "emergency_open" in s.machine.calls


def test_ping_is_acked_cheaply():
    h, _ = make()
    ack = h.handle({"cmd": "ping", "seq": 42})
    assert ack["ok"] is True and ack["seq"] == 42
