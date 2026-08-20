# -*- coding: utf-8 -*-
"""손가락 힘 제어기. 순수 로직이라 하드웨어가 필요 없다."""

import pytest

from force_control import FingerForceController

FLEX_SPAN = 1.396      # 1.0 * radians(80) — finger1 의 a 1.0 당 flex(rad)
DA_MAX = 0.025
A_MAX = 0.9
DEADBAND = 0.2


def make(k_hat, f_target=3.0, lam=0.5, ki=0.02):
    return FingerForceController(
        f_target=f_target, k_hat=k_hat, flex_span=FLEX_SPAN,
        lam=lam, ki=ki, deadband=DEADBAND, da_max=DA_MAX, a_max=A_MAX,
    )


def spring(k_hat, a_contact=0.3):
    """가짜 물체. a 가 접촉점보다 얼마나 넘었는지에 비례해 힘을 낸다.

    force = k_hat * (a - a_contact) * FLEX_SPAN
    (k_hat 은 N/rad 이고 (a - a_contact) * FLEX_SPAN 이 파고든 rad)
    """
    def f(a):
        return max(0.0, k_hat * (a - a_contact) * FLEX_SPAN)
    return f


def settle(ctrl, obj, a0, steps=200):
    """제어기를 가짜 물체에 물려 수렴시킨다. -> (마지막 a, 마지막 force)"""
    a = a0
    for _ in range(steps):
        a = ctrl.update(a, obj(a))
    return a, obj(a)


class Test강성이_달라도_같은_게인으로_수렴한다:
    """이 설계의 존재 이유. 고정 게인 PI 였다면 여기서 깨진다.

    --- k 값을 고를 때 지켜야 하는 제약 ---
    가짜 물체가 낼 수 있는 최대 힘은 a 가 a_max 까지 갔을 때다:
        F_max = k_hat * (A_MAX - a_contact) * FLEX_SPAN
              = k_hat * (0.9 - 0.3) * 1.396
              = 0.8376 * k_hat
    즉 f_target=3.0 을 내려면 k_hat >= 3.2/0.8376 = 3.83 이어야 한다.
    k_hat=1.0 은 최대 0.84N 밖에 못 내므로 절대 수렴할 수 없다 --
    제어기 결함이 아니라 물리적 한계다.
    아래 5.0 / 500.0 은 이 조건을 만족하면서 100배 비율을 유지한다.
    """

    @pytest.mark.parametrize("k_hat", [5.0, 50.0, 500.0])
    def test_수렴한다(self, k_hat):
        ctrl = make(k_hat, f_target=3.0)
        _, force = settle(ctrl, spring(k_hat), a0=0.3)
        assert force == pytest.approx(3.0, abs=DEADBAND)

    def test_강성이_100배_달라도_같은_lam으로_된다(self):
        soft, rigid = make(5.0), make(500.0)
        _, f_soft = settle(soft, spring(5.0), a0=0.3)
        _, f_rigid = settle(rigid, spring(500.0), a0=0.3)
        assert f_soft == pytest.approx(3.0, abs=DEADBAND)
        assert f_rigid == pytest.approx(3.0, abs=DEADBAND)


class Test데드밴드:
    def test_오차가_작으면_안_움직인다(self):
        # 없으면 목표 근처에서 미세하게 계속 떨고 서보가 열난다.
        ctrl = make(10.0, f_target=3.0)
        a = ctrl.update(0.5, 3.0 - DEADBAND / 2)
        assert a == 0.5

    def test_데드밴드_안에서는_적분도_안_쌓인다(self):
        ctrl = make(10.0, f_target=3.0)
        for _ in range(100):
            ctrl.update(0.5, 3.05)
        assert ctrl.integral == 0.0


class Test안전_한계:
    def test_한_사이클_이동량이_da_max를_넘지_않는다(self):
        # 센서가 튀는 값을 한 번 뱉었을 때 손가락이 확 튀는 걸 막는다.
        ctrl = make(0.1, f_target=8.0)     # 아주 물렁 + 큰 오차 = 큰 Δa 요구
        a = 0.5
        for _ in range(50):
            new = ctrl.update(a, 0.0)
            assert abs(new - a) <= DA_MAX + 1e-12
            a = new

    def test_a가_음수로_안_간다(self):
        # kinematics.py:26-28 — 음수 a 는 스토퍼에 부딪힌 채 발열/스톨.
        ctrl = make(1.0, f_target=0.0)
        a = 0.01
        for _ in range(100):
            a = ctrl.update(a, 10.0)      # 목표보다 훨씬 세다 -> 계속 후퇴
        assert a >= 0.0

    def test_a가_a_max를_안_넘는다(self):
        ctrl = make(1.0, f_target=8.0)
        a = 0.8
        for _ in range(100):
            a = ctrl.update(a, 0.0)       # 영원히 힘이 안 잡힌다
        assert a <= A_MAX


class Test푸는_방향은_따로_제한된다:
    """2026-08-14 페트병 실측에서 나온 결함.

    힘이 목표를 넘자 제어기가 14사이클 연속 최대 속도로 열어 a 를 0.35
    (관절 22도) 풀었고 물체가 미끄러졌다. 촉각 센서 피드백이 0.5초 이상
    늦어서, 여는 동안 힘이 줄었는지 확인할 방법이 없는 채로 계속 열었다.

    조이는 쪽은 물체가 막아 주고 힘이 과하면 BACKOFF 가 잡지만, 푸는
    쪽은 저항이 없고 놓치면 되돌릴 수 없다 -- 위험이 대칭이 아니다.
    """

    def make_open(self, da_max_open):
        return FingerForceController(
            f_target=1.0, k_hat=2.0, flex_span=FLEX_SPAN,
            lam=0.5, ki=0.02, deadband=DEADBAND, da_max=DA_MAX,
            a_max=A_MAX, da_max_open=da_max_open,
        )

    def test_푸는_쪽만_느려진다(self):
        ctrl = self.make_open(0.006)
        opened = ctrl.update(0.5, 5.0)          # 목표보다 훨씬 세다
        assert 0.5 - opened == pytest.approx(0.006)

        ctrl = self.make_open(0.006)
        closed = ctrl.update(0.5, 0.0)          # 목표보다 훨씬 약하다
        assert closed - 0.5 == pytest.approx(DA_MAX)

    def test_여러_사이클_열어도_한계를_지킨다(self):
        ctrl = self.make_open(0.006)
        a = 0.8
        for _ in range(30):
            new = ctrl.update(a, 5.0)
            assert a - new <= 0.006 + 1e-12
            a = new

    def test_생략하면_예전처럼_대칭이다(self):
        # 기본값을 둔 이유. 값을 안 주는 호출부의 동작이 안 바뀌어야 한다.
        ctrl = make(2.0, f_target=1.0)
        assert 0.5 - ctrl.update(0.5, 5.0) == pytest.approx(DA_MAX)

    def test_0이면_거부한다(self):
        # 0 이면 힘이 목표를 넘어도 영원히 못 풀어 계속 조인다.
        with pytest.raises(ValueError):
            self.make_open(0.0)


class Test부호가_뒤집히면_적분을_버린다:
    """한 방향으로 쌓인 적분이 반대 방향 응답을 통째로 막는 걸 방지한다.

    a_max 에 붙어 목표에 못 미치는 동안 적분이 양수로 커진 뒤 물체가 세게
    눌러 오면, 비례항이 그 적분을 못 이겨 손가락이 아예 안 풀린다.
    """

    def test_반대_방향_요구에_곧바로_반응한다(self):
        # a_max 로 가는 도중에 적분이 쌓이고, 붙은 뒤에는 안티 와인드업이
        # 그 값을 그대로 얼려 둔다. ki 를 크게 잡아 적분이 비례항을 이기는
        # 상황을 만든다 -- 실물에서는 k_hat 이 클 때 이렇게 된다.
        ctrl = make(30.0, f_target=2.0, ki=0.2)
        a = 0.5
        for _ in range(200):                    # 목표에 못 미치는 상태 유지
            a = ctrl.update(a, 0.5)
        assert a == pytest.approx(A_MAX)        # 상한에 붙어 있다
        assert ctrl.integral > 0.0              # 적분이 양수로 쌓였다

        opened = ctrl.update(a, 5.0)            # 갑자기 목표보다 세다
        assert opened < a                       # 곧바로 풀려야 한다
        assert ctrl.integral == pytest.approx(
            ctrl.ki * (2.0 - 5.0) / 30.0)       # 버리고 새로 쌓았다

    def test_리셋이_없으면_안_풀린다(self):
        # 위 테스트가 무엇을 막고 있는지 보여준다. 리셋을 끄면 같은 상황
        # 에서 손가락이 아예 안 열린다.
        ctrl = make(30.0, f_target=2.0, ki=0.2)
        a = 0.5
        for _ in range(200):
            a = ctrl.update(a, 0.5)
        ctrl._prev_error = 0.0                  # 부호 반전 감지를 무력화
        assert ctrl.update(a, 5.0) == pytest.approx(a)

    def test_같은_부호면_적분을_유지한다(self):
        ctrl = make(30.0, f_target=2.0)
        ctrl.update(0.5, 0.0)
        first = ctrl.integral
        ctrl.update(0.5, 0.1)                   # 여전히 목표보다 약하다
        assert ctrl.integral > first

    def test_데드밴드를_지나_뒤집혀도_잡는다(self):
        # 오차가 데드밴드 안이면 update 가 일찍 반환해 부호 기록이 안 바뀐다.
        # 그 사이를 통과해 반대편으로 가도 리셋이 걸려야 한다.
        ctrl = make(30.0, f_target=2.0)
        for _ in range(50):
            ctrl.update(0.5, 0.0)
        assert ctrl.integral > 0.0
        ctrl.update(0.5, 2.0)                   # 데드밴드 안 -- 조기 반환
        ctrl.update(0.5, 5.0)                   # 반대편
        assert ctrl.integral < 0.0


class Test안티_와인드업:
    def test_상한에_붙어있으면_적분이_안_커진다(self):
        # 없으면 나중에 물체가 치워졌을 때 쌓인 적분 때문에 손가락이
        # 한참 동안 엉뚱하게 움직인다.
        ctrl = make(1.0, f_target=8.0)
        a = A_MAX
        for _ in range(5):
            a = ctrl.update(a, 0.0)
        saturated = ctrl.integral
        for _ in range(200):
            a = ctrl.update(a, 0.0)
        assert ctrl.integral == pytest.approx(saturated)

    def test_적분이_정상상태_오차를_없앤다(self):
        # 연체의 응력 완화로 힘이 조금씩 새는 상황.
        ctrl = make(5.0, f_target=3.0, ki=0.05)
        obj = spring(5.0)
        a = 0.3
        for _ in range(400):
            a = ctrl.update(a, obj(a) * 0.95)   # 항상 5% 모자라게 읽힘
        assert obj(a) * 0.95 == pytest.approx(3.0, abs=DEADBAND)
