# -*- coding: utf-8 -*-
"""telemetry 의 촉각 필드. 콘솔이 파지력을 그리려면 이게 실려야 한다.

데몬이 안 보내면 콘솔은 보여줄 방법이 없다 -- 2026-08-21 이전이 그
상태였고, 그래서 실제 파지 경로에서 힘이 화면에 한 번도 안 보였다.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import grasp_daemon


class FakeMachine:
    align_status = None
    align_enabled = True
    armed = True
    def confirm_remaining(self): return 0.0
    def rearm_remaining(self): return 0.0


class FakeWatchdog:
    def release_in(self): return None


def build(**over):
    base = dict(
        seq=1, state="HOLDING", roi=None, ratio=0.4, valid=1.0, median=0.1,
        angle=None, angle_reason="", axis=None, wrist_deg=None,
        machine=FakeMachine(), watchdog=FakeWatchdog(),
        has_hand_mask=False, bus_owner="hand", forces={},
    )
    base.update(over)
    return grasp_daemon.build_telemetry(**base)


class Test파지력_전송:
    def test_손가락별_힘이_실린다(self):
        tel = build(forces={"r_finger1": 1.02, "r_finger4": 1.52})
        assert tel["forces"] == {"r_finger1": 1.02, "r_finger4": 1.52}

    def test_센서가_없으면_빈_dict(self):
        # --simple-grasp / --no-hand 는 정상 경로다. 콘솔은 이걸 보고
        # "센서 없음"을 그려야 한다 -- 0N 으로 그리면 '안 눌림'과
        # '센서 없음'이 같아 보인다.
        assert build(forces={}) ["forces"] == {}

    def test_못_읽은_손가락은_None으로_실린다(self):
        # 채널이 끊긴 손가락. 0.0 으로 바꾸면 안 된다.
        tel = build(forces={"r_finger5": None})
        assert tel["forces"] == {"r_finger5": None}

    def test_접촉_임계값을_cfg로_같이_보낸다(self):
        # 접촉 개수는 콘솔이 센다. 임계값을 콘솔이 자기 hand_config 에서
        # 읽으면 파이와 PC 의 값이 조용히 어긋난다 -- cfg 를 싣는
        # 기존 이유(enter_ratio 등)와 같다.
        import hand_config
        assert build()["cfg"]["f_touch"] == hand_config.F_TOUCH
