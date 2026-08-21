# -*- coding: utf-8 -*-
"""창 분할. 사각형만 계산한다 -- cv2 를 모른다.

--- 미리보기를 건드리지 않는 이유 ---
카메라 그림은 받은 크기 그대로 왼쪽 위에 둔다. 늘리거나 줄이면 ROI 를
드래그한 좌표를 원본으로 되돌리는 계산(console_input.to_source)이
어긋나서, 화면에는 맞게 보이는데 데몬은 엉뚱한 픽셀을 잰다. 대시보드는
옆과 아래에 '덧붙이기만' 한다.
"""

# 촉각 패널 너비(px). 손가락 5개 막대와 숫자가 들어갈 만큼.
PANEL_W = 260

# 버튼바 높이(px). 버튼 두 줄.
BAR_H = 76


def split(preview_w, preview_h, panel_w=PANEL_W, bar_h=BAR_H):
    """미리보기 크기 -> {"preview": rect, "panel": rect, "bar": rect}.

    rect 는 (x, y, w, h) 다.

        ┌──────────────┬────────┐
        │   preview    │ panel  │
        ├──────────────┴────────┤
        │         bar           │
        └───────────────────────┘
    """
    return {
        "preview": (0, 0, preview_w, preview_h),
        "panel": (preview_w, 0, panel_w, preview_h),
        "bar": (0, preview_h, preview_w + panel_w, bar_h),
    }


def window_size(preview_w, preview_h, panel_w=PANEL_W, bar_h=BAR_H):
    """세 영역을 다 덮는 창 크기 (w, h)."""
    return (preview_w + panel_w, preview_h + bar_h)
