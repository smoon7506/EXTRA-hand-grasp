# -*- coding: utf-8 -*-
"""버튼 -> 명령. cv2 도 소켓도 모르는 순수 로직.

버튼은 키와 **같은 명령 dict** 를 만들어야 한다. 그래야 grasp_commands.py
(데몬 쪽 14개 명령)와 프로토콜이 안 바뀌고, 키 조작도 그대로 살아 있다.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import pytest

from console_input import key_to_command
from dashboard import buttons


class Test히트테스트:
    def test_버튼_안을_누르면_그_버튼(self):
        bar = (0, 480, 900, 76)
        placed = buttons.place(bar)
        x, y, w, h = placed[0].rect
        assert buttons.hit(placed, x + w // 2, y + h // 2) is placed[0]

    def test_버튼_밖은_None(self):
        placed = buttons.place((0, 480, 900, 76))
        assert buttons.hit(placed, 5, 5) is None

    def test_버튼끼리_겹치지_않는다(self):
        # 겹치면 한 번 눌렀는데 두 명령이 나갈 수 있다.
        placed = buttons.place((0, 480, 900, 76))
        for i, a in enumerate(placed):
            for b in placed[i + 1:]:
                ax, ay, aw, ah = a.rect
                bx, by, bw, bh = b.rect
                apart = (ax + aw <= bx or bx + bw <= ax
                         or ay + ah <= by or by + bh <= ay)
                assert apart, f"{a.id} 와 {b.id} 가 겹친다"

    def test_모든_버튼이_버튼바_안에_있다(self):
        bar = (0, 480, 900, 76)
        bx, by, bw, bh = bar
        for b in buttons.place(bar):
            x, y, w, h = b.rect
            assert bx <= x and x + w <= bx + bw
            assert by <= y and y + h <= by + bh


class Test명령_생성:
    def test_키와_같은_명령을_만든다(self):
        # 두 경로가 갈라지면 버튼과 키가 다르게 동작한다.
        for key, bid in [("r", "release"), (" ", "emergency_open"),
                         ("t", "capture_target"), ("h", "save_hand_mask"),
                         ("n", "calib_near"), ("f", "calib_far")]:
            assert buttons.command(bid, {}) == key_to_command(ord(key))

    def test_모르는_버튼은_None(self):
        assert buttons.command("없는버튼", {}) is None


class Test토글은_화면_상태를_따른다:
    """m(무장)/a(정렬)는 현재 상태를 알아야 뒤집을 수 있다.

    그래서 console_input._KEYS 에 없고 콘솔 루프가 텔레메트리를 보고
    만든다. 버튼도 같은 규약이어야 한다 -- 눈에 보이는 상태와 뒤집는
    대상이 어긋나면 안 된다.
    """

    def test_무장중이면_해제를_보낸다(self):
        assert buttons.command("arm", {"armed": True}) == {"cmd": "disarm"}

    def test_해제중이면_무장을_보낸다(self):
        assert buttons.command("arm", {"armed": False}) == {"cmd": "arm"}

    def test_정렬_토글도_같다(self):
        assert buttons.command("align", {"align_on": True}) == {
            "cmd": "set_align", "on": False}
        assert buttons.command("align", {"align_on": False}) == {
            "cmd": "set_align", "on": True}

    def test_텔레메트리가_아직_없으면_기본값을_쓴다(self):
        # 접속 직후. armed 기본은 False(데몬이 armed=False 로 뜬다),
        # align 기본은 True.
        assert buttons.command("arm", {}) == {"cmd": "arm"}
        assert buttons.command("align", {}) == {"cmd": "set_align",
                                                "on": False}


def test_수동_파지_버튼이_키와_같다():
    assert buttons.command("grasp", {}) == key_to_command(ord("g"))


def test_수동_파지_버튼이_배치된다():
    ids = [b.id for b in buttons.place((0, 480, 900, 76))]
    assert "grasp" in ids
