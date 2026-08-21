# -*- coding: utf-8 -*-
"""콘솔 입력 -> 명령. cv2 도 소켓도 안 건드린다.

--- 왜 값이 아니라 동작을 보내나 ---
n/f/t/h 는 원래 "지금 이 프레임의 값"으로 정한다. 콘솔이 텔레메트리로
받은 값을 써서 계산하면 그 값은 이미 100ms 낡았고 캘리브레이션이
조용히 틀린다. 그래서 여기서는 동작 이름만 만든다.
"""

from roi_config import NUDGE_M, TARGET_NUDGE_DEG


def to_source(box, view_w, view_h, src_w, src_h):
    """미리보기 창 좌표 -> 원본 카메라 좌표.

    미리보기를 축소해 받으면 드래그한 좌표도 축소된 좌표다. 되돌리지
    않으면 화면에는 맞게 보이는데 데몬은 엉뚱한 픽셀을 잰다 -- 화면과
    판정이 어긋나서 원인을 찾기 어렵다.
    """
    x, y, w, h = box
    sx, sy = src_w / float(view_w), src_h / float(view_h)
    # 크기는 최소 1 이다. 0 이면 RoiConfig.validate() 가 거절한다.
    return (int(round(x * sx)), int(round(y * sy)),
            max(1, int(round(w * sx))), max(1, int(round(h * sy))))


# 키 -> 명령. a 는 여기 없다 -- 현재 정렬 상태를 알아야 뒤집을 수 있어서
# 콘솔 루프가 텔레메트리를 보고 만든다.
_KEYS = {
    "n": {"cmd": "calib_band", "edge": "near"},
    "f": {"cmd": "calib_band", "edge": "far"},
    "g": {"cmd": "grasp"},
    "t": {"cmd": "capture_target"},
    "h": {"cmd": "save_hand_mask"},
    "[": {"cmd": "nudge_band", "edge": "near", "delta_m": -NUDGE_M},
    "]": {"cmd": "nudge_band", "edge": "near", "delta_m": +NUDGE_M},
    "-": {"cmd": "nudge_band", "edge": "far", "delta_m": -NUDGE_M},
    "=": {"cmd": "nudge_band", "edge": "far", "delta_m": +NUDGE_M},
    ",": {"cmd": "nudge_target", "delta_deg": -TARGET_NUDGE_DEG},
    ".": {"cmd": "nudge_target", "delta_deg": +TARGET_NUDGE_DEG},
    "w": {"cmd": "jog_wrist", "dir": 1},
    "W": {"cmd": "jog_wrist", "dir": -1},
    "r": {"cmd": "release"},
    " ": {"cmd": "emergency_open"},
}


def key_to_command(key):
    """cv2.waitKey 값 -> 명령 dict. 매핑이 없으면 None."""
    if not 0 <= key < 0x110000:
        return None
    cmd = _KEYS.get(chr(key))
    return dict(cmd) if cmd is not None else None
