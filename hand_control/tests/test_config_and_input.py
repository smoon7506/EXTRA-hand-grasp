# -*- coding: utf-8 -*-
"""load_fingers(실제 r_hand.toml 읽기) + parse_input 테스트. 모터 불필요."""

import math

import pytest

import hand_config
import main


class TestLoadFingers:
    def test_toml에서_다섯_손가락을_읽는다(self):
        fingers = hand_config.load_fingers()
        assert len(fingers) == 5
        assert [f.name for f in fingers] == [
            "r_finger1", "r_finger2", "r_finger3", "r_finger4", "r_finger5",
        ]

    def test_finger1의_ID와_오프셋(self):
        # r_hand.toml:5-14 의 값. 오프셋은 각각 정확히 7도와 5도다.
        (f1,) = hand_config.load_fingers(active=["r_finger1"])
        assert (f1.id1, f1.id2) == (1, 2)
        assert math.degrees(f1.offset1) == pytest.approx(7.0)
        assert math.degrees(f1.offset2) == pytest.approx(5.0)

    def test_active_순서대로_돌려준다(self):
        fingers = hand_config.load_fingers(active=["r_finger5", "r_finger1"])
        assert [f.name for f in fingers] == ["r_finger5", "r_finger1"]

    def test_없는_이름은_에러(self):
        # 조용히 무시하면 "손가락이 안 움직이는데 에러도 없는" 상태가 된다.
        with pytest.raises(ValueError, match="없는 손가락"):
            hand_config.load_fingers(active=["r_finger9"])

    def test_가중치가_붙는다(self):
        # 값을 박아두지 않는다. FINGER_WEIGHTS 는 실물을 보며 튜닝하는
        # 표라서, 숫자를 고칠 때마다 이 테스트가 같이 깨지면 안 된다.
        # 검사할 것은 "표의 값이 그대로 실린다"는 것뿐이다.
        (thumb,) = hand_config.load_fingers(active=["r_finger5"])
        want_flex, want_spread = hand_config.FINGER_WEIGHTS["r_finger5"]
        assert thumb.flex_weight == pytest.approx(want_flex)
        assert thumb.spread_weight == pytest.approx(want_spread)

    def test_모터_ID가_전부_다르다(self):
        # 여기서 걸리면 toml 에 오타가 있는 것이다.
        fingers = hand_config.load_fingers()
        ids = [i for f in fingers for i in (f.id1, f.id2)]
        assert sorted(ids) == list(range(1, 11))


class TestParseInput:
    def test_a만_주면_s는_0(self):
        assert main.parse_input("0.3") == (0.3, 0.0)

    def test_a와_s를_둘_다(self):
        assert main.parse_input("0.3 -0.5") == (0.3, -0.5)

    def test_q는_None(self):
        assert main.parse_input("q") is None
        assert main.parse_input("  Q  ") is None

    def test_숫자가_아니면_에러(self):
        with pytest.raises(ValueError, match="숫자가 아닙니다"):
            main.parse_input("abc")

    def test_빈_입력은_에러(self):
        with pytest.raises(ValueError):
            main.parse_input("   ")

    def test_a가_범위_밖이면_거절한다(self):
        # kinematics 는 조용히 잘라주지만 사람 입력은 거절하는 게 낫다.
        # 0.5 를 치려다 5 를 쳤을 때 알려줘야 한다.
        with pytest.raises(ValueError, match="a 는"):
            main.parse_input("5")
        with pytest.raises(ValueError, match="a 는"):
            main.parse_input("-0.1")

    def test_s가_범위_밖이면_거절한다(self):
        with pytest.raises(ValueError, match="s 는"):
            main.parse_input("0.5 2")

    def test_값이_셋이면_에러(self):
        with pytest.raises(ValueError, match="최대 2개"):
            main.parse_input("0.1 0.2 0.3")
