# -*- coding: utf-8 -*-
"""마우스 라우팅. 한 창에 ROI 드래그와 버튼바가 같이 있다.

라우팅이 없으면 버튼을 누른 것이 ROI 드래그 시작으로도 잡혀서,
버튼 한 번에 명령이 나가면서 ROI 까지 바뀐다.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import cv2
import pytest

from dashboard import buttons as dash_buttons
from grasp_console import MouseRouter, RoiDragger

BAR = (0, 480, 900, 76)


def router():
    r = MouseRouter(RoiDragger())
    r.placed = dash_buttons.place(BAR)
    return r


def center(b):
    x, y, w, h = b.rect
    return (x + w // 2, y + h // 2)


def press(r, x, y):
    r.on_mouse(cv2.EVENT_LBUTTONDOWN, x, y, 0, None)


def release(r, x, y):
    r.on_mouse(cv2.EVENT_LBUTTONUP, x, y, 0, None)


class Test버튼_클릭:
    def test_누르고_떼면_발화한다(self):
        r = router()
        b = r.placed[0]
        press(r, *center(b))
        release(r, *center(b))
        assert r.take_click() is b

    def test_한_번만_돌려준다(self):
        # 루프가 매 프레임 읽으므로 안 지우면 같은 명령이 계속 나간다.
        r = router()
        b = r.placed[0]
        press(r, *center(b))
        release(r, *center(b))
        assert r.take_click() is b
        assert r.take_click() is None

    def test_누르기만_하면_아직_발화_안_한다(self):
        r = router()
        press(r, *center(r.placed[0]))
        assert r.take_click() is None

    def test_밖으로_끌어서_놓으면_취소된다(self):
        # 잘못 눌렀을 때 빠져나갈 길. 비상 정지 옆에 버튼이 있는
        # 배치라 이게 필요하다.
        r = router()
        press(r, *center(r.placed[0]))
        release(r, 5, 5)
        assert r.take_click() is None

    def test_다른_버튼_위에서_놓아도_취소된다(self):
        r = router()
        press(r, *center(r.placed[0]))
        release(r, *center(r.placed[1]))
        assert r.take_click() is None


class Test드래그와_섞이지_않는다:
    def test_버튼을_눌러도_ROI_드래그가_시작되지_않는다(self):
        r = router()
        press(r, *center(r.placed[0]))
        assert r.dragger.preview() is None

    def test_미리보기에서_누르면_드래그가_시작된다(self):
        r = router()
        press(r, 100, 100)
        r.on_mouse(cv2.EVENT_MOUSEMOVE, 200, 180, 0, None)
        assert r.dragger.preview() == (100, 100, 100, 80)

    def test_미리보기에서_끌어_버튼바에서_놓아도_ROI가_완성된다(self):
        # 누른 자리로 주인을 정하기 때문이다. 커서 위치로 매번 다시
        # 정하면 드래그가 중간에 사라진다.
        r = router()
        press(r, 100, 100)
        r.on_mouse(cv2.EVENT_MOUSEMOVE, 300, 500, 0, None)
        release(r, 300, 500)
        assert r.dragger.result == (100, 100, 200, 400)
        assert r.take_click() is None


class Test재배치돼도_클릭이_산다:
    """렌더 루프는 매 프레임 버튼을 새로 배치한다.

    2026-08-21 실물에서 버튼이 하나도 안 먹었다. 누른 시점과 뗀 시점
    사이에 렌더가 돌면서 placed 가 통째로 새 Button 객체로 갈리는데,
    발화 판정이 객체 동일성(is)이라 영원히 False 였다.

    앞의 테스트들이 이걸 놓친 이유: press 와 release 사이에 재배치를
    안 했다. 실제 루프는 반드시 그 사이에 돈다.
    """

    def test_누르고_뗄_사이에_재배치돼도_발화한다(self):
        r = router()
        target = r.placed[0]
        press(r, *center(target))
        # 렌더 한 바퀴. 같은 자리에 같은 버튼이지만 객체는 새것이다.
        r.placed = dash_buttons.place(BAR)
        assert r.placed[0] is not target
        release(r, *center(r.placed[0]))
        clicked = r.take_click()
        assert clicked is not None
        assert clicked.id == target.id

    def test_재배치_뒤_다른_버튼에서_떼면_여전히_취소된다(self):
        r = router()
        press(r, *center(r.placed[0]))
        r.placed = dash_buttons.place(BAR)
        release(r, *center(r.placed[1]))
        assert r.take_click() is None
