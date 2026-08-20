# -*- coding: utf-8 -*-
"""콘솔 명령 -> 데몬 동작. 소켓도 카메라도 모터도 모른다.

--- 핵심 규약: 콘솔은 "값"이 아니라 "동작"을 보낸다 ---
n 키는 원래 그 프레임의 median 으로 near 를 정한다. 콘솔이 값을 계산해
보내면 그 median 은 이미 100ms 낡았고, 캘리브레이션이 조용히 틀린다.
그래서 콘솔은 {"cmd":"calib_band","edge":"near"} 만 보내고, 값은 여기서
FrameState 의 최신 값으로 만든다. capture_target 과 save_hand_mask 도
같은 이유다.
"""

from dataclasses import dataclass, field

import orientation
import roi_config
from link import PROTO
from roi_config import MARGIN_M, RoiConfig, nudge_band_by, nudge_target_by


@dataclass
class FrameState:
    """데몬이 매 프레임 갱신하는 "지금 무엇을 보고 있나".

    디스패처는 여기 있는 값만 읽는다. 콘솔이 보낸 숫자는 안 믿는다.
    """

    roi: object = None
    median: float = None
    angle: float = None
    angle_reason: str = ""
    band_mask: object = None
    machine: object = None
    wrist: object = None
    aligner: object = None
    # 부수효과 훅. 데몬이 채운다. 테스트에서는 리스트에 모은다.
    on_roi_saved: object = field(default=lambda roi: None)
    on_save_hand_mask: object = field(default=lambda mask: None)
    on_jog_wrist: object = field(default=lambda direction: (False, "손목 없음"))


def _ok(cmd, msg="", **extra):
    return dict(t="ack", cmd=cmd, ok=True, msg=msg, **extra)


def _no(cmd, msg, **extra):
    return dict(t="ack", cmd=cmd, ok=False, msg=msg, **extra)


class CommandHandler:
    def __init__(self, state):
        self.s = state

    def handle(self, msg):
        """명령 하나 -> ack 하나. 예외를 밖으로 내보내지 않는다.

        여기서 죽으면 데몬 전체가 죽는다. 콘솔이 이상한 걸 보냈다고
        손이 물체를 쥔 채 방치되면 안 된다.
        """
        cmd = msg.get("cmd")
        fn = getattr(self, f"_do_{cmd}", None) if isinstance(cmd, str) else None
        if fn is None:
            return _no(cmd, f"모르는 명령이다: {cmd}")
        try:
            return fn(msg)
        except (KeyError, TypeError, ValueError) as e:
            return _no(cmd, f"명령을 처리하지 못했다: {e}")

    # --- 연결 ---

    def _do_hello(self, msg):
        got = msg.get("proto")
        if got != PROTO:
            return _no("hello",
                       f"프로토콜이 다르다 (데몬 {PROTO}, 콘솔 {got}). "
                       f"양쪽 코드를 맞춰야 한다.")
        return _ok("hello", f"proto {PROTO}", proto=PROTO)

    def _do_ping(self, msg):
        return _ok("ping", seq=msg.get("seq"))

    def _do_bye(self, msg):
        return _ok("bye", "잘 가")

    # --- 무장 ---

    def _do_arm(self, msg):
        self.s.machine.arm()
        return _ok("arm", "무장")

    def _do_disarm(self, msg):
        self.s.machine.disarm()
        return _ok("disarm", "무장 해제")

    # --- ROI ---

    def _do_set_roi(self, msg):
        old = self.s.roi
        near = old.near_m if old else roi_config.NEAR_M
        far = old.far_m if old else roi_config.FAR_M
        target = old.target_angle_deg if old else 0.0
        roi = RoiConfig(x=int(msg["x"]), y=int(msg["y"]),
                        w=int(msg["w"]), h=int(msg["h"]),
                        near_m=near, far_m=far, target_angle_deg=target)
        try:
            roi.validate()
        except ValueError as e:
            return _no("set_roi", str(e))
        self.s.roi = roi
        self.s.on_roi_saved(roi)
        return _ok("set_roi", f"ROI x={roi.x} y={roi.y} w={roi.w} h={roi.h}")

    def _do_calib_band(self, msg):
        """n/f 키. **데몬의 지금 median** 으로 정한다."""
        if self.s.roi is None:
            return _no("calib_band", "ROI 가 아직 없다.")
        if self.s.median is None:
            return _no("calib_band", "ROI 에 유효 깊이가 없어 캘리브레이션 불가")
        edge = msg.get("edge")
        roi, median = self.s.roi, self.s.median
        # 여유를 바깥으로 벌린다. 센서 노이즈가 ±1cm 다.
        if edge == "near":
            near, far = median - MARGIN_M, roi.far_m
        elif edge == "far":
            near, far = roi.near_m, median + MARGIN_M
        else:
            return _no("calib_band", f"밴드 경계 이름이 아니다: {edge}")
        if near >= far:
            return _no("calib_band",
                       f"밴드가 뒤집힌다 ({near:.3f} >= {far:.3f}).")
        roi.near_m, roi.far_m = near, far
        self.s.on_roi_saved(roi)
        return _ok("calib_band", f"밴드 {near:.3f} ~ {far:.3f} m")

    def _do_nudge_band(self, msg):
        if self.s.roi is None:
            return _no("nudge_band", "ROI 가 아직 없다.")
        ok, why = nudge_band_by(self.s.roi, msg.get("edge"),
                                float(msg["delta_m"]))
        if ok:
            self.s.on_roi_saved(self.s.roi)
        return (_ok if ok else _no)("nudge_band", why)

    # --- 목표 각도 ---

    def _do_capture_target(self, msg):
        """t 키. **데몬의 지금 angle** 로 정한다."""
        if self.s.roi is None:
            return _no("capture_target", "ROI 가 아직 없다.")
        if self.s.angle is None:
            return _no("capture_target",
                       f"각도를 못 재고 있어 목표를 정할 수 없다 "
                       f"({self.s.angle_reason}).")
        self.s.roi.target_angle_deg = orientation.wrap90(self.s.angle)
        self._push_target()
        self.s.on_roi_saved(self.s.roi)
        return _ok("capture_target",
                   f"목표 각도 {self.s.roi.target_angle_deg:+.1f}도")

    def _do_nudge_target(self, msg):
        if self.s.roi is None:
            return _no("nudge_target", "ROI 가 아직 없다.")
        ok, why = nudge_target_by(self.s.roi, float(msg["delta_deg"]))
        if ok:
            self._push_target()
            self.s.on_roi_saved(self.s.roi)
        return (_ok if ok else _no)("nudge_target", why)

    def _push_target(self):
        """정렬기에도 새 목표를 알린다. 안 하면 화면만 바뀌고 안 돈다."""
        if self.s.aligner is not None:
            self.s.aligner.target_deg = self.s.roi.target_angle_deg

    # --- 손 마스크 ---

    def _do_save_hand_mask(self, msg):
        """h 키. **데몬의 지금 밴드 마스크** 를 찍는다."""
        mask = self.s.band_mask
        if mask is None:
            return _no("save_hand_mask", "아직 밴드 마스크가 없다.")
        self.s.on_save_hand_mask(mask)
        return _ok("save_hand_mask",
                   f"손 마스크 저장 ({int(mask.sum())} 픽셀). "
                   f"물체를 치운 상태에서 찍었는지 확인할 것.")

    # --- 손목 / 손 ---

    def _do_set_align(self, msg):
        on = bool(msg.get("on"))
        self.s.machine.set_align_enabled(on)
        actual = self.s.machine.align_enabled
        if on and not actual:
            return _no("set_align", "정렬기가 없어 켤 수 없다 (손목 없음).")
        return _ok("set_align", "정렬 " + ("켬" if actual else "끔"))

    def _do_jog_wrist(self, msg):
        ok, why = self.s.on_jog_wrist(int(msg.get("dir", 1)))
        return (_ok if ok else _no)("jog_wrist", why)

    def _do_release(self, msg):
        if self.s.machine.request_release():
            return _ok("release", "놓는다")
        return _no("release", f"지금은 놓을 수 없다 (상태 {self.s.machine.state}).")

    def _do_emergency_open(self, msg):
        self.s.machine.emergency_open()
        return _ok("emergency_open", "비상 폄")
