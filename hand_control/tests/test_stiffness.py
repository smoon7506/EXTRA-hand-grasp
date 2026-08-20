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
