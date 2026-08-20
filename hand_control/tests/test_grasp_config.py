# -*- coding: utf-8 -*-
"""파지 설정값의 불변식. 숫자를 잘못 적으면 import 시점에 죽어야 한다."""

import pytest

import hand_config


class Test기본값이_불변식을_지킨다:
    def test_현재_설정이_통과한다(self):
        hand_config.check_grasp_config()

    def test_목표힘이_상한보다_충분히_낮다(self):
        margin = hand_config.F_ABORT - hand_config.F_ABORT_HYST
        assert hand_config.F_TARGET_RIGID < margin
        assert hand_config.F_TARGET_SOFT < margin

    def test_접촉_임계값이_목표힘보다_낮다(self):
        assert hand_config.F_TOUCH < hand_config.F_TARGET_RIGID
        assert hand_config.F_TOUCH < hand_config.F_TARGET_SOFT

    def test_센서_매핑이_아는_손가락_이름만_쓴다(self):
        # 오타 하나면 검지 힘을 보고 새끼를 조인다. 증상만 보고는 못 찾는다.
        known = set(hand_config.FINGER_WEIGHTS)
        assert set(hand_config.SENSOR_CHANNEL_MAP.values()) <= known

    def test_센서_매핑에_중복_손가락이_없다(self):
        names = list(hand_config.SENSOR_CHANNEL_MAP.values())
        assert len(names) == len(set(names))

    def test_푸는_쪽이_조이는_쪽보다_빠르지_않다(self):
        # 놓치는 방향을 더 조심한다는 게 비대칭 제한의 목적이다.
        assert 0.0 < hand_config.DA_MAX_OPEN <= hand_config.DA_MAX

    def test_열림_제한이_a_max보다_작다(self):
        # 바닥이 0 이하가 되면 제한이 없는 것과 같다.
        assert 0.0 < hand_config.HOLD_OPEN_LIMIT < hand_config.A_MAX

    def test_위험을_접촉보다_늦게_인정하지_않는다(self):
        assert 1 <= hand_config.ABORT_CONFIRM_CYCLES
        assert (hand_config.ABORT_CONFIRM_CYCLES
                <= hand_config.TOUCH_CONFIRM_CYCLES)


class Test위반하면_거절한다:
    """monkeypatch 로 값을 망가뜨린 뒤 검사 함수가 잡아내는지 본다."""

    def test_목표힘이_상한보다_높으면_예외(self, monkeypatch):
        monkeypatch.setattr(hand_config, "F_TARGET_SOFT",
                            hand_config.F_ABORT + 1.0)
        with pytest.raises(ValueError, match="F_TARGET"):
            hand_config.check_grasp_config()

    def test_접촉_임계값이_목표힘보다_높으면_예외(self, monkeypatch):
        monkeypatch.setattr(hand_config, "F_TOUCH",
                            hand_config.F_TARGET_RIGID + 1.0)
        with pytest.raises(ValueError, match="F_TOUCH"):
            hand_config.check_grasp_config()

    def test_푸는_쪽이_더_빠르면_예외(self, monkeypatch):
        monkeypatch.setattr(hand_config, "DA_MAX_OPEN",
                            hand_config.DA_MAX * 2.0)
        with pytest.raises(ValueError, match="DA_MAX_OPEN"):
            hand_config.check_grasp_config()

    def test_열림_제한이_0이면_예외(self, monkeypatch):
        monkeypatch.setattr(hand_config, "HOLD_OPEN_LIMIT", 0.0)
        with pytest.raises(ValueError, match="HOLD_OPEN_LIMIT"):
            hand_config.check_grasp_config()

    def test_열림_제한이_a_max_이상이면_예외(self, monkeypatch):
        monkeypatch.setattr(hand_config, "HOLD_OPEN_LIMIT", hand_config.A_MAX)
        with pytest.raises(ValueError, match="HOLD_OPEN_LIMIT"):
            hand_config.check_grasp_config()

    def test_위험을_접촉보다_늦게_인정하면_예외(self, monkeypatch):
        monkeypatch.setattr(hand_config, "ABORT_CONFIRM_CYCLES",
                            hand_config.TOUCH_CONFIRM_CYCLES + 1)
        with pytest.raises(ValueError, match="ABORT_CONFIRM_CYCLES"):
            hand_config.check_grasp_config()

    def test_ABORT_CONFIRM_CYCLES가_0이면_예외(self, monkeypatch):
        monkeypatch.setattr(hand_config, "ABORT_CONFIRM_CYCLES", 0)
        with pytest.raises(ValueError, match="ABORT_CONFIRM_CYCLES"):
            hand_config.check_grasp_config()

    def test_DA_MAX_OPEN이_0이면_예외(self, monkeypatch):
        # 0 이면 힘이 목표를 넘어도 영원히 못 풀어 계속 조인다.
        monkeypatch.setattr(hand_config, "DA_MAX_OPEN", 0.0)
        with pytest.raises(ValueError, match="DA_MAX_OPEN"):
            hand_config.check_grasp_config()

    def test_K_MIN이_0이면_예외(self, monkeypatch):
        # k̂ 는 제어식의 분모다. 0 이 새어나가면 Δa 가 발산한다.
        monkeypatch.setattr(hand_config, "K_MIN", 0.0)
        with pytest.raises(ValueError, match="K_MIN"):
            hand_config.check_grasp_config()

    def test_K_FIXED가_범위_밖이면_예외(self, monkeypatch):
        # 제어식의 분모라 범위를 벗어나면 Δa 가 발산하거나 손가락이
        # 사실상 정지한다.
        monkeypatch.setattr(hand_config, "K_FIXED", 0.0)
        with pytest.raises(ValueError, match="K_FIXED"):
            hand_config.check_grasp_config()

    def test_K_FIXED가_K_MAX를_넘으면_예외(self, monkeypatch):
        monkeypatch.setattr(hand_config, "K_FIXED",
                            hand_config.K_MAX + 1.0)
        with pytest.raises(ValueError, match="K_FIXED"):
            hand_config.check_grasp_config()

    def test_K_FIXED가_None이면_통과한다(self, monkeypatch):
        # None 이 끄는 방법이다. 범위 검사에 걸리면 안 된다.
        monkeypatch.setattr(hand_config, "K_FIXED", None)
        hand_config.check_grasp_config()

    def test_K_FIXED가_범위_안이면_통과한다(self, monkeypatch):
        monkeypatch.setattr(hand_config, "K_FIXED", 3.9)
        hand_config.check_grasp_config()

    def test_LAMBDA가_1을_넘으면_예외(self, monkeypatch):
        # λ>1 은 매 사이클 목표를 지나쳐 가라는 뜻이다. 발진한다.
        monkeypatch.setattr(hand_config, "LAMBDA", 1.5)
        with pytest.raises(ValueError, match="LAMBDA"):
            hand_config.check_grasp_config()

    def test_A_MAX가_1을_넘으면_예외(self, monkeypatch):
        monkeypatch.setattr(hand_config, "A_MAX", 1.5)
        with pytest.raises(ValueError, match="A_MAX"):
            hand_config.check_grasp_config()

    def test_히스테리시스가_0이면_예외(self, monkeypatch):
        # 0 이면 상한 경계에서 HOLD ↔ BACKOFF 가 매 사이클 반복된다.
        monkeypatch.setattr(hand_config, "F_ABORT_HYST", 0.0)
        with pytest.raises(ValueError, match="F_ABORT_HYST"):
            hand_config.check_grasp_config()

    def test_프로빙_계단이_2회_미만이면_예외(self, monkeypatch):
        # 회귀로 기울기를 뽑으려면 점이 최소 2개 필요하다.
        monkeypatch.setattr(hand_config, "PROBE_STEPS", 1)
        with pytest.raises(ValueError, match="PROBE_STEPS"):
            hand_config.check_grasp_config()


class Test목표힘_순서:
    """무를수록 약하게 잡는다. 이게 분류의 존재 이유다.

    a 를 더 줬는데 F 가 안 늘어난다 -> 찌그러지는 중 -> 약하게.
    a 를 더 줬는데 F 가 늘어난다   -> 버티는 중     -> 세게 가능.

    한번 반대로 적혀 있던 자리라 검사로 못 박는다.
    """

    def test_연체_목표가_강체_목표보다_작거나_같다(self):
        assert hand_config.F_TARGET_SOFT <= hand_config.F_TARGET_RIGID

    def test_뒤집으면_예외(self, monkeypatch):
        monkeypatch.setattr(hand_config, "F_TARGET_SOFT",
                            hand_config.F_TARGET_RIGID + 1.0)
        with pytest.raises(ValueError, match="F_TARGET_SOFT"):
            hand_config.check_grasp_config()
