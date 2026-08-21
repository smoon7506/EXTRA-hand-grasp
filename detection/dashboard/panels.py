# -*- coding: utf-8 -*-
"""촉각 패널. 손 전체 파지력을 그린다.

--- 왜 필요한가 ---
촉각 센서는 매 프레임 힘을 재는데, 2026-08-21 이전까지 실제 파지 경로
(데몬+콘솔)에서는 그 값이 화면 어디에도 안 나왔다. telemetry 에 아예
안 실려 있었다. 로그 감사로만 알 수 있던 것들(포화 62%, 엄지 접촉률 0%,
손가락별 힘이 4배까지 갈림)을 파지하는 동안 눈으로 보기 위한 패널이다.

--- 요약 계산을 따로 뺀 이유 ---
summarize/trend 는 cv2 를 안 쓴다. '센서 없음'과 '힘 0' 을 가르는 규칙이
이 패널의 유일한 실질 로직이라 창 없이 검증되어야 한다.
"""

from dataclasses import dataclass, field

import cv2
import numpy as np

# 손가락 막대를 꽉 채우는 힘(N). 실측 HOLD 힘 p90 이 1.91N, 최대 2.06N
# 이라 2.0 이면 정상 파지가 막대 대부분을 쓴다. 넘으면 색만 바뀐다.
FULL_N = 2.0

# 합계 스파크라인의 세로 눈금(N). 손가락 하나 기준인 FULL_N 을 합계에
# 쓰면 정상 파지(합계 실측 0.53~4.39N, 중앙값 2.07N)가 늘 천장에 붙어
# 잘린다 -- 그러면 오르내림이 안 보여서 스파크라인의 존재 이유가 없다.
FULL_TOTAL_N = 5.0

# 합계 추세 그래프 높이(px). 패널에 자리가 모자라면 줄여서 그린다.
SPARK_H = 90

# 추세를 보는 창(초). 슬립도 센서 드리프트도 순간값에는 안 보인다.
TREND_S = 8.0

# 추세로 인정하는 최소 변화(N). 이 안이면 "유지"다.
TREND_EPS = 0.05

_BG = (32, 32, 32)
_TEXT = (230, 230, 230)
_DIM = (120, 120, 120)
_BAR = (90, 200, 90)
_BAR_HOT = (60, 60, 235)
_WARN = (0, 210, 235)


@dataclass
class Summary:
    total: float | None          # 합계(N). 하나도 못 읽으면 None
    contact: int                 # f_touch 를 넘은 손가락 수
    n: int                       # 센서가 보고한 손가락 수
    missing: list = field(default_factory=list)   # 값을 못 읽은 손가락


def summarize(forces, f_touch):
    """{손가락: 힘|None} -> Summary.

    셋을 구분한다:
      값이 있다      -> 합계와 접촉 개수에 들어간다
      None 이다      -> 채널이 끊긴 것. 합계에서 빼고 missing 에 남긴다
                        (0 으로 세면 합계가 조용히 낮아진다)
      dict 가 비었다 -> 센서가 아예 없다(--simple-grasp/--no-hand).
                        total=None 으로 '안 눌림'과 갈라 준다
    """
    values = [(n, v) for n, v in forces.items() if v is not None]
    missing = sorted(n for n, v in forces.items() if v is None)
    total = sum(v for _, v in values) if values else None
    contact = sum(1 for _, v in values if v > f_touch)
    return Summary(total, contact, len(forces), missing)


def source(forces):
    """forces 필드가 어떤 상황인지 -> "stale_daemon" | "no_sensor" | "ok".

    telemetry 에 키가 아예 없는 것(None)과 빈 dict 는 전혀 다른 상황이다.

      None    데몬이 옛 버전이라 forces 를 안 보낸다. 파이에 배포가
              안 됐다는 뜻이지 센서 문제가 아니다.
      {}      데몬은 새것인데 센서가 없다(--simple-grasp / --no-hand).

    둘을 똑같이 그리면 배포 누락을 센서 고장으로 오해하고 엉뚱한 데를
    뒤진다 -- 2026-08-21 에 실제로 그랬다. 이 프로젝트가 ROI 에서
    "물체 없음"과 "센서가 못 봄"을 가르는 것과 같은 규칙이다.
    """
    if forces is None:
        return "stale_daemon"
    if not forces:
        return "no_sensor"
    return "ok"


def trend(history):
    """최근 합계 이력 -> "up" | "down" | "flat". 표본이 모자라면 None.

    양끝만 본다. 회귀를 쓰면 창 안의 한 번짜리 글리치에 기울기가
    휘둘리는데, 여기 목적은 "지금 빠지고 있나"를 눈으로 아는 것뿐이다.
    """
    if len(history) < 2:
        return None
    delta = history[-1] - history[0]
    if delta > TREND_EPS:
        return "up"
    if delta < -TREND_EPS:
        return "down"
    return "flat"


_ARROW = {"up": "^", "down": "v", "flat": "-"}

# 힘이 안 올 때 화면에 띄우는 말. 원인을 짚어 주지 않으면 "왜 안 나오지"
# 에서 멈춘다.
_NO_DATA = {
    "stale_daemon": ("no force in telemetry",
                     ("daemon is out of date -",
                      "redeploy grasp_daemon.py",
                      "to the Pi and restart it")),
    "no_sensor": ("sensor off",
                  ("daemon runs with",
                   "--simple-grasp or --no-hand")),
}


def draw(canvas, rect, forces, f_touch, history):
    """패널을 canvas 의 rect 자리에 그린다. 제자리에서 고친다."""
    x, y, w, h = rect
    cv2.rectangle(canvas, (x, y), (x + w, y + h), _BG, -1)
    font, small = cv2.FONT_HERSHEY_SIMPLEX, cv2.FONT_HERSHEY_PLAIN

    cv2.putText(canvas, "HAND FORCE", (x + 12, y + 26), font, 0.6, _TEXT, 1)

    kind = source(forces)
    if kind != "ok":
        head, hints = _NO_DATA[kind]
        cv2.putText(canvas, head, (x + 12, y + 56), font, 0.55, _WARN, 1)
        for i, line in enumerate(hints):
            cv2.putText(canvas, line, (x + 12, y + 78 + i * 16), small, 0.9,
                        _DIM, 1)
        return

    s = summarize(forces, f_touch)

    total_text = "--" if s.total is None else f"{s.total:5.2f} N"
    arrow = _ARROW.get(trend(history), " ")
    cv2.putText(canvas, f"total {total_text} {arrow}", (x + 12, y + 56),
                font, 0.6, _TEXT, 1)
    cv2.putText(canvas, f"contact {s.contact}/{s.n}", (x + 12, y + 80),
                font, 0.5, _TEXT if s.contact else _DIM, 1)

    # --- 손가락별 막대 ---
    top, row = y + 104, 26
    bar_x, bar_w = x + 44, w - 44 - 60
    for i, name in enumerate(sorted(forces)):
        cy = top + i * row
        value = forces[name]
        # 로그·분석에서 쓰는 f1~f5 표기에 맞춘다.
        cv2.putText(canvas, "f" + name[-1], (x + 12, cy + 12),
                    small, 1.0, _DIM, 1)
        cv2.rectangle(canvas, (bar_x, cy), (bar_x + bar_w, cy + 14),
                      (56, 56, 56), -1)
        if value is None:
            # 못 읽은 손가락은 막대를 그리지 않는다. 0 길이 막대로 두면
            # 안 눌린 것과 같아 보인다.
            cv2.putText(canvas, "n/a", (bar_x + 4, cy + 12), small, 1.0,
                        _WARN, 1)
            continue
        fill = int(bar_w * min(1.0, max(0.0, value / FULL_N)))
        color = _BAR_HOT if value >= FULL_N else _BAR
        if fill > 0:
            cv2.rectangle(canvas, (bar_x, cy), (bar_x + fill, cy + 14),
                          color, -1)
        cv2.putText(canvas, f"{value:4.2f}", (bar_x + bar_w + 8, cy + 12),
                    small, 1.0, _TEXT if value > f_touch else _DIM, 1)

    # --- 합계 추세 그래프 ---
    # 넉넉히 키운다. 34px 로 두면 실측 폭(합계 1.6~2.4N 정도의 변화)이
    # 5px 로 뭉개져서 빠지고 있는지 눈으로 못 본다.
    spark_y = top + len(forces) * row + 18
    spark_h = min(SPARK_H, (y + h) - spark_y - 22)
    if spark_h >= 30 and len(history) >= 2:
        _sparkline(canvas, (x + 12, spark_y, w - 24, spark_h), history)
        cv2.putText(canvas, f"last {TREND_S:.0f}s",
                    (x + 12, spark_y + spark_h + 16), small, 0.9, _DIM, 1)


def _sparkline(canvas, rect, history):
    """합계 이력을 꺾은선으로. 세로 눈금은 0~FULL_TOTAL_N 고정이다.

    자동 스케일로 두면 조용한 구간에서 노이즈가 산맥처럼 보여서,
    진짜 변화가 있을 때와 구분이 안 된다. 대신 눈금선을 1N 마다 그어
    고정 스케일에서도 절대값을 읽을 수 있게 한다.
    """
    x, y, w, h = rect
    cv2.rectangle(canvas, (x, y), (x + w, y + h), (24, 24, 24), -1)
    small = cv2.FONT_HERSHEY_PLAIN

    for n_line in range(1, int(FULL_TOTAL_N)):
        gy = int(y + h - (n_line / FULL_TOTAL_N) * h)
        cv2.line(canvas, (x, gy), (x + w, gy), (52, 52, 52), 1)
        cv2.putText(canvas, str(n_line), (x + 2, gy - 2), small, 0.7,
                    (80, 80, 80), 1)

    step = w / float(max(1, len(history) - 1))
    points = [(int(x + i * step),
               int(y + h - min(1.0, max(0.0, v / FULL_TOTAL_N)) * h))
              for i, v in enumerate(history)]
    cv2.polylines(canvas, [np.array(points, np.int32)], False, _BAR, 2)
