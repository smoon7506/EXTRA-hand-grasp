# -*- coding: utf-8 -*-
"""버튼 정의 -> 명령 dict. cv2 도 소켓도 모른다.

--- 키와 같은 명령을 만든다 ---
console_input.key_to_command() 가 이미 순수 함수로 키 -> 명령을 만든다.
버튼은 **같은 dict** 를 만들어 같은 경로로 보낸다. 그래서
grasp_commands.py(데몬 쪽 명령 처리)와 프로토콜은 안 바뀌고, 키 조작도
그대로 살아 있다. 두 경로가 갈라지면 버튼과 키가 다르게 동작한다.

--- 토글은 왜 텔레메트리를 받나 ---
m(무장)/a(정렬)는 "지금 값을 뒤집는다"라서 현재 상태를 알아야 한다.
그래서 _KEYS 에도 없고 콘솔 루프가 화면에 그린 tel 을 보고 만든다.
여기서도 같은 규약을 지킨다 -- 눈에 보이는 상태와 뒤집는 대상이
어긋나면 안 되므로, 호출부는 최신 텔레메트리가 아니라 **지금 그린
프레임의 tel** 을 넘겨야 한다.
"""

from dataclasses import dataclass, field

from roi_config import NUDGE_M, TARGET_NUDGE_DEG

# 버튼 사이/가장자리 여백(px).
GAP = 6
PAD = 8


@dataclass
class Button:
    id: str
    label: str
    # 그리기 힌트. "danger" 는 비상 정지처럼 눈에 띄어야 하는 것.
    kind: str = "normal"
    rect: tuple = field(default=None)


# 두 줄. 위는 상태를 바꾸는 것, 아래는 캘리브레이션.
#
# 비상 정지를 위 줄 끝에 둔 이유: 급할 때 손이 가는 자리는 한 곳이어야
# 한다. 다른 버튼과 섞어 두면 찾다가 늦는다.
ROWS = [
    [Button("arm", "ARM"), Button("align", "ALIGN"),
     Button("grasp", "GRASP"), Button("release", "RELEASE"),
     Button("emergency_open", "OPEN!", kind="danger")],
    [Button("calib_near", "near"), Button("calib_far", "far"),
     Button("near-", "n-"), Button("near+", "n+"),
     Button("far-", "f-"), Button("far+", "f+"),
     Button("capture_target", "target"),
     Button("target-", "t<"), Button("target+", "t>"),
     Button("wrist-", "<w"), Button("wrist+", "w>"),
     Button("save_hand_mask", "mask")],
]


# 토글이 아닌 것들. key_to_command 와 같은 dict 를 낸다.
_STATIC = {
    "grasp": {"cmd": "grasp"},
    "release": {"cmd": "release"},
    "emergency_open": {"cmd": "emergency_open"},
    "calib_near": {"cmd": "calib_band", "edge": "near"},
    "calib_far": {"cmd": "calib_band", "edge": "far"},
    "near-": {"cmd": "nudge_band", "edge": "near", "delta_m": -NUDGE_M},
    "near+": {"cmd": "nudge_band", "edge": "near", "delta_m": +NUDGE_M},
    "far-": {"cmd": "nudge_band", "edge": "far", "delta_m": -NUDGE_M},
    "far+": {"cmd": "nudge_band", "edge": "far", "delta_m": +NUDGE_M},
    "capture_target": {"cmd": "capture_target"},
    "target-": {"cmd": "nudge_target", "delta_deg": -TARGET_NUDGE_DEG},
    "target+": {"cmd": "nudge_target", "delta_deg": +TARGET_NUDGE_DEG},
    "wrist+": {"cmd": "jog_wrist", "dir": 1},
    "wrist-": {"cmd": "jog_wrist", "dir": -1},
    "save_hand_mask": {"cmd": "save_hand_mask"},
}


def place(bar_rect, rows=None):
    """버튼바 사각형 -> rect 가 채워진 Button 목록.

    한 줄 안에서 균등 분할한다. 겹치면 한 번 눌렀는데 두 명령이
    나갈 수 있으므로 여백을 뺀 뒤 나눈다.
    """
    rows = rows or ROWS
    bx, by, bw, bh = bar_rect
    row_h = (bh - PAD * 2 - GAP * (len(rows) - 1)) // len(rows)
    placed = []
    for r, row in enumerate(rows):
        if not row:
            continue
        y = by + PAD + r * (row_h + GAP)
        total = bw - PAD * 2 - GAP * (len(row) - 1)
        w = total // len(row)
        for c, proto in enumerate(row):
            x = bx + PAD + c * (w + GAP)
            placed.append(Button(proto.id, proto.label, proto.kind,
                                 (x, y, w, row_h)))
    return placed


def hit(placed, x, y):
    """좌표에 걸린 Button. 없으면 None."""
    for b in placed:
        bx, by, bw, bh = b.rect
        if bx <= x < bx + bw and by <= y < by + bh:
            return b
    return None


def command(button_id, tel):
    """버튼 id + 지금 그린 텔레메트리 -> 명령 dict. 없으면 None."""
    if button_id == "arm":
        # 데몬은 armed=False 로 뜬다. 텔레메트리가 아직 없으면 그 값을
        # 가정해야 첫 클릭이 '무장'이 된다.
        return {"cmd": "disarm" if tel.get("armed") else "arm"}
    if button_id == "align":
        # 정렬 기본값은 켬이다(GraspStateMachine.align_enabled).
        return {"cmd": "set_align", "on": not tel.get("align_on", True)}
    cmd = _STATIC.get(button_id)
    return dict(cmd) if cmd is not None else None
