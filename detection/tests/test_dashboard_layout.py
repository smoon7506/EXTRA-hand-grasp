# -*- coding: utf-8 -*-
"""창 분할. cv2 없이 사각형만 계산하는 순수 로직."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import pytest

from dashboard import layout


class Test창_분할:
    def test_카메라는_왼쪽_위에_원래_크기로_남는다(self):
        # 미리보기를 늘리거나 줄이면 ROI 드래그 좌표 환산이 어긋난다.
        # 대시보드는 옆과 아래에 '덧붙일' 뿐이다.
        r = layout.split(640, 480)
        assert r["preview"] == (0, 0, 640, 480)

    def test_패널은_카메라_오른쪽에_같은_높이로(self):
        r = layout.split(640, 480)
        px, py, pw, ph = r["panel"]
        assert (px, py) == (640, 0)
        assert ph == 480
        assert pw > 0

    def test_버튼바는_전체_너비로_아래에(self):
        r = layout.split(640, 480)
        bx, by, bw, bh = r["bar"]
        assert (bx, by) == (0, 480)
        assert bw == 640 + r["panel"][2]
        assert bh > 0

    def test_창_크기가_세_영역을_다_덮는다(self):
        r = layout.split(640, 480)
        w, h = layout.window_size(640, 480)
        assert w == r["preview"][2] + r["panel"][2]
        assert h == r["preview"][3] + r["bar"][3]

    def test_미리보기가_축소돼도_따라간다(self):
        # 데몬이 --preview-scale 로 줄여 보낼 수 있다.
        r = layout.split(320, 240)
        assert r["preview"] == (0, 0, 320, 240)
        assert r["panel"][3] == 240
        assert r["bar"][1] == 240
