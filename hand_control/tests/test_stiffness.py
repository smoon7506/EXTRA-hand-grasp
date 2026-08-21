# -*- coding: utf-8 -*-
"""강성 추정. 순수 함수라 하드웨어가 필요 없다."""

import pytest

import stiffness

# 테스트 전역. hand_config 값과 무관하게 여기서 고정한다.
K_MIN, K_MAX, MIN_SPAN = 0.1, 500.0, 0.02


def line(slope, intercept, xs):
    """기울기가 알려진 인공 샘플."""
    return [(x, slope * x + intercept) for x in xs]


class Test기울기_추정:
    def test_직선이면_기울기를_정확히_찾는다(self):
        xs = [0.00, 0.05, 0.10, 0.15, 0.20]
        k, ok = stiffness.estimate_stiffness(
            line(30.0, 1.0, xs), K_MIN, K_MAX, MIN_SPAN)
        assert ok is True
        assert k == pytest.approx(30.0)

    def test_노이즈가_있어도_대략_맞는다(self):
        # 두 점 차분이 아니라 회귀를 쓰는 이유가 이것이다.
        xs = [0.00, 0.05, 0.10, 0.15, 0.20]
        noisy = [(x, 30.0 * x + 1.0 + n)
                 for x, n in zip(xs, [0.1, -0.1, 0.1, -0.1, 0.0])]
        k, ok = stiffness.estimate_stiffness(noisy, K_MIN, K_MAX, MIN_SPAN)
        assert ok is True
        assert k == pytest.approx(30.0, rel=0.15)

    def test_연체는_작은_k(self):
        xs = [0.0, 0.1, 0.2, 0.3, 0.4]
        k, ok = stiffness.estimate_stiffness(
            line(2.0, 0.3, xs), K_MIN, K_MAX, MIN_SPAN)
        assert ok is True
        assert k == pytest.approx(2.0)


class Test신뢰할_수_없는_경우:
    def test_변위가_거의_없으면_k_max_에_confident_False(self):
        # 아주 단단한 물체. 밀어도 관절이 안 움직여 기울기가 발산한다.
        xs = [0.000, 0.001, 0.002, 0.003, 0.004]   # span 0.004 < 0.02
        k, ok = stiffness.estimate_stiffness(
            line(1000.0, 0.0, xs), K_MIN, K_MAX, MIN_SPAN)
        assert ok is False
        assert k == K_MAX

    def test_기울기가_음수면_k_max_에_confident_False(self):
        # 밀었는데 힘이 줄었다 = 미끄러짐 또는 드리프트.
        # k_min 으로 클램프하면 Δa = λ·e/k̂ 가 폭발한다.
        xs = [0.0, 0.05, 0.10, 0.15, 0.20]
        k, ok = stiffness.estimate_stiffness(
            line(-10.0, 5.0, xs), K_MIN, K_MAX, MIN_SPAN)
        assert ok is False
        assert k == K_MAX

    def test_샘플이_1개면_k_max_에_confident_False(self):
        k, ok = stiffness.estimate_stiffness(
            [(0.1, 1.0)], K_MIN, K_MAX, MIN_SPAN)
        assert ok is False
        assert k == K_MAX

    def test_샘플이_비어도_죽지_않는다(self):
        k, ok = stiffness.estimate_stiffness([], K_MIN, K_MAX, MIN_SPAN)
        assert ok is False
        assert k == K_MAX

    def test_모든_x가_같아도_죽지_않는다(self):
        # 0으로 나누기가 나는 자리다.
        k, ok = stiffness.estimate_stiffness(
            [(0.1, 1.0), (0.1, 2.0), (0.1, 3.0)], K_MIN, K_MAX, MIN_SPAN)
        assert ok is False
        assert k == K_MAX


class Test클램프:
    def test_아주_큰_기울기는_k_max_로_잘린다(self):
        xs = [0.0, 0.05, 0.10, 0.15, 0.20]
        k, ok = stiffness.estimate_stiffness(
            line(9999.0, 0.0, xs), K_MIN, K_MAX, MIN_SPAN)
        assert k == K_MAX
        assert ok is True   # 변위는 충분했으므로 측정 자체는 신뢰할 수 있다

    def test_아주_작은_기울기는_k_min_으로_잘린다(self):
        xs = [0.0, 0.05, 0.10, 0.15, 0.20]
        k, ok = stiffness.estimate_stiffness(
            line(0.001, 0.0, xs), K_MIN, K_MAX, MIN_SPAN)
        assert k == K_MIN
        assert ok is True

    def test_결과는_항상_범위_안이다(self):
        for slope in (-100.0, 0.0, 0.001, 1.0, 50.0, 1e6):
            xs = [0.0, 0.05, 0.10, 0.15, 0.20]
            k, _ = stiffness.estimate_stiffness(
                line(slope, 0.0, xs), K_MIN, K_MAX, MIN_SPAN)
            assert K_MIN <= k <= K_MAX


class Test분류:
    def test_임계값보다_크면_강체(self):
        assert stiffness.classify(50.0, 10.0) == "rigid"

    def test_임계값보다_작으면_연체(self):
        assert stiffness.classify(2.0, 10.0) == "soft"

    def test_임계값과_같으면_강체(self):
        # 경계는 강체 쪽. 애매하면 약하게 잡는 F_TARGET_RIGID 로 간다.
        assert stiffness.classify(10.0, 10.0) == "rigid"

    def test_임계값이_None이면_연체로_본다(self):
        # 아직 실측 전. 규약상 무를수록 약하게 잡으므로, 모를 때는
        # 약한 쪽이 안전하다 -- 무른 걸 세게 잡으면 부수지만 단단한 걸
        # 약하게 잡으면 놓칠 뿐이다.
        assert stiffness.classify(0.5, None) == "soft"
        assert stiffness.classify(499.0, None) == "soft"


class Test손_전체_대표강성:
    """손가락 여러 개의 k_hat -> 손 하나의 대표값.

    물체는 하나인데 손가락마다 따로 분류하던 것을 손 단위로 올린다.
    집계가 평균이 아니라 최대인 이유: 접촉이 나쁜 손가락은 물체가
    아니라 자기 접촉 상태를 잰다(스치듯 닿으면 많이 들어가는데 힘은
    조금 -> 낮은 k). 강체의 증거는 "밀었는데 안 들어간다"이고 그건
    손가락 하나만으로 성립하는 증거다. 평균을 내면 못 닿은 손가락이
    그 증거를 희석시킨다.

    2026-08-18/19 로그 실측: 같은 물체를 같이 잡은 손가락 사이의
    k_hat 편차가 2.3배였다(한 런은 147배).
    """

    def test_confident한_것_중_최대를_고른다(self):
        assert stiffness.hand_k([(2.0, True), (9.0, True), (5.0, True)]) == 9.0

    def test_측정_실패는_무시한다(self):
        # 가장 중요한 함정이다. estimate_stiffness 는 flex 변화폭이
        # 모자라면 (K_MAX, False) 를 돌려준다. 그 값을 최대에 넣으면
        # 손가락 하나만 실패해도 손 전체가 항상 K_MAX = 항상 rigid 가
        # 된다. 게다가 실측 25건의 측정 실패가 전부 a=A_MAX 였다 --
        # 밀었는데 안 들어간 게 아니라 애초에 밀 여유가 없던 경우라
        # 강체 증거가 아니다.
        assert stiffness.hand_k([(2.0, True), (500.0, False)]) == 2.0

    def test_전부_실패면_모른다(self):
        assert stiffness.hand_k([(500.0, False), (500.0, False)]) is None

    def test_비어_있으면_모른다(self):
        assert stiffness.hand_k([]) is None

    def test_아직_안_잰_손가락은_건너뛴다(self):
        # CLASSIFY 전 손가락은 k_hat 이 None 이다. 손가락마다 PROBE 가
        # 끝나는 시점이 달라서 이 상태가 정상적으로 섞여 들어온다.
        assert stiffness.hand_k([(None, None), (3.0, True)]) == 3.0
