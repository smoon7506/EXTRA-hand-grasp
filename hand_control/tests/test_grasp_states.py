# -*- coding: utf-8 -*-
"""손가락별 파지 상태기계. 순수 로직이라 하드웨어가 필요 없다."""

import pytest

import grasp
from grasp import FingerGrasp, GraspParams

FLEX_SPAN = 1.396      # radians(80)


def params(**over):
    base = dict(
        f_touch=0.3, f_abort=8.0, f_abort_hyst=1.0,
        f_target_rigid=2.0, f_target_soft=4.0, f_deadband=0.2,
        touch_confirm_cycles=1, contact_lost_cycles=5,
        k_threshold=10.0, k_min=0.1, k_max=500.0, probe_min_span=0.02,
        probe_step=0.02, probe_steps=5, probe_settle_s=0.15,
        a_rate=0.15, a_max=0.9, lam=0.5, ki=0.02,
        da_max=0.025, backoff_step=0.02, stall_timeout_s=2.0,
    )
    base.update(over)
    return GraspParams(**base)


def make(**over):
    return FingerGrasp("r_finger1", FLEX_SPAN, params(**over))


def run(finger, readings, dt=0.05):
    """[(force, flex), ...] 를 순서대로 먹인다. -> 상태 이력."""
    now, history = 0.0, []
    for force, flex in readings:
        finger.update(force, flex, now, dt)
        history.append(finger.state)
        now += dt
    return history


class TestApproach:
    def test_시작_상태는_APPROACH이고_a는_0이다(self):
        f = make()
        assert f.state == grasp.APPROACH
        assert f.a == 0.0

    def test_힘이_없으면_a_rate로_오므린다(self):
        f = make(a_rate=0.15)
        f.update(0.0, 0.0, now=0.0, dt=0.1)
        assert f.a == pytest.approx(0.015)

    def test_a_max에_닿으면_NO_CONTACT(self):
        # 허공을 잡았다. 계속 밀면 링키지가 스스로를 조인다.
        f = make(a_rate=1.0, a_max=0.5)
        run(f, [(0.0, 0.0)] * 20, dt=0.1)
        assert f.state == grasp.NO_CONTACT
        assert f.a == pytest.approx(0.5)

    def test_NO_CONTACT에서는_a가_안_변한다(self):
        f = make(a_rate=1.0, a_max=0.5)
        run(f, [(0.0, 0.0)] * 20, dt=0.1)
        frozen_a = f.a
        run(f, [(0.0, 0.0)] * 20, dt=0.1)
        assert f.a == pytest.approx(frozen_a)

    def test_f_touch를_넘으면_PROBE로_간다(self):
        f = make(f_touch=0.3)
        f.update(0.5, 0.1, now=0.0, dt=0.05)
        assert f.state == grasp.PROBE

    def test_dt가_커도_da_max를_넘지_않는다(self):
        # SERIAL_TIMEOUT_S(0.5초) 뒤의 사이클처럼 dt 가 크게 튀는 경우.
        # a_rate*dt(0.0825) 를 그대로 쓰면 da_max(0.025) 의 3배가 넘는
        # 한 스텝이 나가 접근 중이던 물체를 그대로 들이받는다.
        f = make(a_rate=0.15, da_max=0.025)
        f.update(0.0, 0.0, now=0.0, dt=0.55)
        assert f.a == pytest.approx(0.025)


class TestProbe:
    def _probe_to_end(self, f, k_true, a_contact=0.04):
        """가짜 물체를 물려 프로빙이 끝날 때까지 돌린다.

        a_contact 는 probe_steps(5) x probe_step(0.02) 안에서 실제로
        도달 가능해야 한다. PROBE 는 "샘플 -> (필요하면) 계단" 순서라
        마지막(5번째) 샘플 앞에는 계단이 없다 -- 즉 프로빙 동안 실제로
        벌어지는 a 이동량은 (probe_steps-1) x probe_step = 0.08 이 상한이다.
        0.1 로 두면 물체에 닿기도 전에 5개 샘플이 모두 flex=0 으로
        모여 확신도(confident)가 늘 False 로 떨어져 물렁한 물체조차
        강체로 오분류된다 -- 브리프 원안의 0.1 은 이 상한을 넘어서
        절대 닿지 않는 값이었다.
        """
        now, dt = 0.0, 0.05
        for _ in range(300):
            flex = max(0.0, (f.a - a_contact)) * FLEX_SPAN
            force = 0.5 + k_true * flex
            f.update(force, flex, now, dt)
            now += dt
            if f.state in (grasp.HOLD, grasp.BACKOFF):
                break
        return now

    def test_계단마다_안정화를_기다린다(self):
        # 서보가 도착하기 전에 재면 힘과 각도가 짝이 안 맞는다.
        f = make(probe_settle_s=0.5, probe_step=0.02)
        f.update(0.5, 0.0, now=0.0, dt=0.05)      # 접촉 -> PROBE
        f.update(0.5, 0.0, now=0.0, dt=0.05)      # 1번째 샘플 + 계단
        after_step = f.a
        f.update(0.6, 0.01, now=0.1, dt=0.05)     # 아직 대기 중
        assert f.a == pytest.approx(after_step)

    def test_probe_steps만큼_샘플을_모으면_분류한다(self):
        f = make(probe_steps=5)
        self._probe_to_end(f, k_true=30.0)
        assert f.state == grasp.HOLD
        assert f.k_hat is not None

    def test_딱딱한_물체는_rigid로_분류된다(self):
        f = make(k_threshold=10.0)
        self._probe_to_end(f, k_true=50.0)
        assert f.object_class == "rigid"
        assert f.f_target == pytest.approx(f.params.f_target_rigid)

    def test_물렁한_물체는_soft로_분류된다(self):
        f = make(k_threshold=10.0)
        self._probe_to_end(f, k_true=2.0)
        assert f.object_class == "soft"
        assert f.f_target == pytest.approx(f.params.f_target_soft)

    def test_임계값이_None이면_soft로_간다(self):
        # 아직 실측 전. 규약상 무를수록 약하게 잡으므로 모를 때는
        # 약한 쪽(soft)이 안전하다.
        f = make(k_threshold=None)
        self._probe_to_end(f, k_true=2.0)
        assert f.object_class == "soft"
        assert f.f_target == pytest.approx(f.params.f_target_soft)

    def test_관절이_안_움직이면_confident가_False(self):
        # 아주 단단한 물체. flex 가 늘 0 이다.
        f = make()
        now, dt = 0.0, 0.05
        for _ in range(300):
            f.update(5.0, 0.0, now, dt)
            now += dt
            if f.state in (grasp.HOLD, grasp.BACKOFF):
                break
        assert f.confident is False


class TestHold:
    def test_목표힘으로_수렴한다(self):
        f = make(k_threshold=10.0)
        a_contact, k_true = 0.1, 30.0
        now, dt = 0.0, 0.05
        for _ in range(800):
            flex = max(0.0, (f.a - a_contact)) * FLEX_SPAN
            force = 0.5 + k_true * flex
            f.update(force, flex, now, dt)
            now += dt
        flex = max(0.0, (f.a - a_contact)) * FLEX_SPAN
        assert 0.5 + k_true * flex == pytest.approx(
            f.f_target, abs=f.params.f_deadband)

    def test_정상_수렴하면_NO_CONTACT로_안_빠진다(self):
        # a 가 a_max 근처에 갈 일이 없는 정상 수렴 시나리오에서는
        # 스톨 타임아웃이 아무리 짧아도 NO_CONTACT 로 빠지면 안 된다.
        f = make(k_threshold=10.0, stall_timeout_s=0.1)
        a_contact, k_true = 0.1, 30.0
        now, dt = 0.0, 0.05
        for _ in range(800):
            flex = max(0.0, (f.a - a_contact)) * FLEX_SPAN
            force = 0.5 + k_true * flex
            f.update(force, flex, now, dt)
            now += dt
        assert f.state == grasp.HOLD

    def _to_hold_at_a_max(self, a_max=0.5, **over):
        """a_max 에 붙은 채 HOLD 인 손가락을 만든다. -> (finger, now, flex)"""
        f = make(k_threshold=10.0, a_max=a_max, **over)
        a_contact, k_true = 0.1, 30.0
        now, dt = 0.0, 0.05
        flex = 0.0
        for _ in range(400):
            flex = max(0.0, (f.a - a_contact)) * FLEX_SPAN
            f.update(0.5 + k_true * flex, flex, now, dt)
            now += dt
            if f.state == grasp.HOLD:
                break
        assert f.state == grasp.HOLD
        return f, now, flex

    def test_a_max에서_잡고_있으면_목표를_실측치로_낮춘다(self):
        # 손가락마다 낼 수 있는 힘이 접촉 지오메트리 때문에 크게 다르다
        # (2026-08-14 로그: finger2 0.40N vs finger4 1.72N). 손 전체에
        # 목표 하나를 주면 어떤 손가락에는 도달 불가능한 값이 된다.
        # 포기하는 대신 목표를 현실에 맞춘다.
        f, now, flex = self._to_hold_at_a_max(stall_timeout_s=0.3)
        dt = 0.05
        stuck_force = f.params.f_target_rigid - 1.0   # 목표(2.0)에 한참 미달
        assert stuck_force > f.params.f_touch         # 잡고는 있다

        for _ in range(200):
            f.update(stuck_force, flex, now, dt)
            now += dt
        assert f.state == grasp.HOLD                  # 안 놓는다
        assert f.f_target == pytest.approx(stuck_force)
        assert f._controller.f_target == pytest.approx(stuck_force)
        assert f.a == pytest.approx(0.5)

    def test_a_max에서_허공이면_예전처럼_NO_CONTACT(self):
        # 잡고 있는데 목표에 못 미치는 것과, 아무것도 없는데 조이는 것은
        # 다른 상황이다. 후자는 여전히 포기해야 한다.
        f, now, flex = self._to_hold_at_a_max(stall_timeout_s=0.3,
                                              contact_lost_cycles=1000)
        dt = 0.05
        empty = f.params.f_touch          # f_touch 초과가 아니다
        for _ in range(200):
            f.update(empty, flex, now, dt)
            now += dt
            if f.state == grasp.NO_CONTACT:
                break
        assert f.state == grasp.NO_CONTACT
        assert f.a == pytest.approx(0.5)

    def test_순간적인_골짜기에_목표가_고정되지_않는다(self):
        # 2026-08-14 13-01-13 로그의 finger1: 힘이 잠깐 0.67N 이던 순간에
        # 목표가 거기 고정됐는데 실제 중앙값은 1.12N 이었다. 창 안에서
        # 제일 잘 나온 힘을 써야 한다.
        f, now, flex = self._to_hold_at_a_max(stall_timeout_s=0.5)
        dt = 0.05
        low, high = 0.6, 1.2       # 둘 다 목표(2.0)에는 못 미친다
        for i in range(200):
            f.update(high if i % 2 else low, flex, now, dt)
            now += dt
        assert f.state == grasp.HOLD
        assert f.f_target == pytest.approx(high)

    def test_바닥에_붙었는데_힘이_남으면_목표를_올린다(self):
        # 올리는 쪽이 없으면 목표가 한 번 내려간 뒤 영영 못 돌아온다.
        f, now, flex = self._to_hold_at_a_max(stall_timeout_s=0.3,
                                              hold_open_limit=0.05)
        dt = 0.05
        # 먼저 목표를 낮게 고정시킨다
        for _ in range(200):
            f.update(0.5, flex, now, dt)
            now += dt
        assert f.f_target == pytest.approx(0.5)

        # 이제 목표보다 센 힘이 계속 들어온다 -> 바닥까지 열고 멈춘다
        for _ in range(200):
            f.update(1.5, flex, now, dt)
            now += dt
        assert f.state == grasp.HOLD
        assert f.f_target == pytest.approx(1.5)

    def test_목표는_설정값보다_높아지지_않는다(self):
        # 천장이 없으면 힘이 튈 때마다 목표가 따라 올라간다.
        f, now, flex = self._to_hold_at_a_max(stall_timeout_s=0.3,
                                              hold_open_limit=0.05)
        dt = 0.05
        ceiling = f._f_target_max
        for _ in range(40):
            f.update(0.5, flex, now, dt)
            now += dt
        for _ in range(300):
            f.update(ceiling * 3.0, flex, now, dt)
            now += dt
        assert f.f_target <= ceiling + 1e-9

    def test_목표가_f_touch_아래로는_안_내려간다(self):
        f, now, flex = self._to_hold_at_a_max(stall_timeout_s=0.3,
                                              contact_lost_cycles=1000)
        dt = 0.05
        weak = f.params.f_touch + 1e-9    # 겨우 접촉으로 인정되는 힘
        for _ in range(200):
            f.update(weak, flex, now, dt)
            now += dt
        assert f.f_target >= f.params.f_touch


class TestBackoff:
    def test_상한을_넘으면_어떤_상태에서든_후퇴한다(self):
        f = make(f_abort=8.0)
        f.update(20.0, 0.1, now=0.0, dt=0.05)
        assert f.state == grasp.BACKOFF

    def test_후퇴하면_a가_줄어든다(self):
        f = make(f_abort=8.0, backoff_step=0.02)
        f.a = 0.5
        f.update(20.0, 0.1, now=0.0, dt=0.05)
        assert f.a == pytest.approx(0.48)

    def test_히스테리시스_없이는_복귀하지_않는다(self):
        # F_ABORT 바로 아래로만 떨어져도 복귀하면 경계에서 매 사이클
        # HOLD <-> BACKOFF 가 반복된다.
        f = make(f_abort=8.0, f_abort_hyst=1.0)
        f.update(20.0, 0.1, now=0.0, dt=0.05)
        f.update(7.5, 0.1, now=0.05, dt=0.05)      # 8.0 - 1.0 = 7.0 보다 위
        assert f.state == grasp.BACKOFF

    def test_충분히_내려가면_복귀한다(self):
        f = make(f_abort=8.0, f_abort_hyst=1.0)
        f.update(20.0, 0.1, now=0.0, dt=0.05)
        f.update(6.5, 0.1, now=0.05, dt=0.05)      # 7.0 보다 아래
        assert f.state != grasp.BACKOFF

    def test_후퇴해도_a가_음수로_안_간다(self):
        f = make(f_abort=8.0, backoff_step=0.1)
        f.a = 0.05
        for i in range(10):
            f.update(20.0, 0.1, now=i * 0.05, dt=0.05)
        assert f.a >= 0.0


class TestProbeBackoffLivelock:
    def test_프로빙_중_상한을_넘어도_결국_분류로_이어진다(self):
        # 아주 강체인 물체를 프로빙하면 5계단 이동만으로 F_ABORT 를
        # 넘어 BACKOFF 로 간다. _controller 가 아직 None 이라 예전
        # 코드는 무조건 APPROACH 로 돌아가 _samples 를 지웠고, 힘이
        # 여전히 abort 근처라 즉시 재중단 -- 영원히 반복됐다(livelock).
        f = make(k_threshold=10.0, f_abort=8.0, f_abort_hyst=1.0)
        a_contact, k_true = 0.04, 400.0
        now, dt = 0.0, 0.05
        for _ in range(2000):
            flex = max(0.0, (f.a - a_contact)) * FLEX_SPAN
            force = 0.5 + k_true * flex
            f.update(force, flex, now, dt)
            now += dt
            if f.state in (grasp.HOLD, grasp.NO_CONTACT):
                break
        assert f.state == grasp.HOLD
        assert f.k_hat is not None


class TestFrozen:
    def test_힘이_None이면_동결된다(self):
        f = make()
        f.update(0.5, 0.1, now=0.0, dt=0.05)
        f.update(None, 0.1, now=0.05, dt=0.05)
        assert f.state == grasp.FROZEN

    def test_관절각이_None이어도_동결된다(self):
        # 힘과 위치 둘 다 있어야 제어식이 성립한다.
        f = make()
        f.update(0.5, 0.1, now=0.0, dt=0.05)
        f.update(0.5, None, now=0.05, dt=0.05)
        assert f.state == grasp.FROZEN

    def test_동결_중에는_a가_안_변한다(self):
        f = make()
        f.update(0.5, 0.1, now=0.0, dt=0.05)
        held = f.a
        for i in range(20):
            f.update(None, None, now=0.1 + i * 0.05, dt=0.05)
        assert f.a == pytest.approx(held)

    def test_복구되면_원래_상태로_돌아온다(self):
        f = make()
        f.update(0.5, 0.1, now=0.0, dt=0.05)
        before = f.state
        f.update(None, None, now=0.05, dt=0.05)
        assert f.state == grasp.FROZEN
        f.update(0.5, 0.1, now=0.1, dt=0.05)
        assert f.state == before


class Test손가락은_서로_독립이다:
    def test_한_손가락이_동결돼도_다른_손가락은_진행한다(self):
        a, b = make(), make()
        for i in range(10):
            now = i * 0.05
            a.update(None, None, now, 0.05)     # 센서 끊김
            b.update(0.0, 0.0, now, 0.05)       # 정상, 접근 중
        assert a.state == grasp.FROZEN
        assert b.state == grasp.APPROACH
        assert b.a > 0.0

    def test_진도가_서로_달라도_된다(self):
        # 물체가 동그랗지 않으면 검지는 닿았는데 새끼는 허공이다.
        early, late = make(), make()
        for i in range(5):
            now = i * 0.05
            early.update(0.5, 0.05, now, 0.05)   # 이미 접촉
            late.update(0.0, 0.0, now, 0.05)     # 아직 허공
        assert early.state == grasp.PROBE
        assert late.state == grasp.APPROACH


class Test접촉_확인과_상실:
    """2026-08-13 종이컵 실측에서 나온 두 결함.

    손가락이 컵 테두리를 치는 순간 4.8N 이 잡혔다가 컵이 좌굴하면서
    0 으로 꺼졌다. 코드는 그 한 번을 접촉으로 세고 프로빙에 들어갔고,
    이후 HOLD 에서 13초 동안 F=0.00 인 채로 a_max 까지 올라갔다.
    """

    def test_한_사이클_스파이크로는_프로빙에_안_들어간다(self):
        f = make(touch_confirm_cycles=3, f_touch=0.3)
        f.update(4.8, 0.05, now=0.0, dt=0.05)      # 튀었다
        assert f.state == grasp.APPROACH
        f.update(0.0, 0.05, now=0.05, dt=0.05)     # 곧바로 꺼짐
        assert f.state == grasp.APPROACH

    def test_연속으로_넘으면_프로빙에_들어간다(self):
        f = make(touch_confirm_cycles=3, f_touch=0.3)
        for i in range(3):
            f.update(0.5, 0.05, now=i * 0.05, dt=0.05)
        assert f.state == grasp.PROBE

    def test_중간에_끊기면_확인_카운트가_초기화된다(self):
        f = make(touch_confirm_cycles=3, f_touch=0.3)
        f.update(0.5, 0.05, now=0.00, dt=0.05)
        f.update(0.5, 0.05, now=0.05, dt=0.05)
        f.update(0.0, 0.05, now=0.10, dt=0.05)     # 끊김
        f.update(0.5, 0.05, now=0.15, dt=0.05)
        assert f.state == grasp.APPROACH

    def test_HOLD에서_물체가_빠지면_다시_접근한다(self):
        # 이게 없으면 목표 힘을 향해 허공을 조이며 a_max 까지 간다.
        f = make(touch_confirm_cycles=1, contact_lost_cycles=5,
                 k_threshold=10.0)
        now, dt = 0.0, 0.05
        for _ in range(400):
            flex = max(0.0, (f.a - 0.04)) * FLEX_SPAN
            f.update(0.5 + 30.0 * flex, flex, now, dt)
            now += dt
            if f.state == grasp.HOLD:
                break
        assert f.state == grasp.HOLD
        for i in range(5):
            f.update(0.0, 0.2, now + i * dt, dt)
        assert f.state == grasp.APPROACH
        # 마지막 4 사이클은 아직 포기 전이라 HOLD 로 계속 추종한다.
        # 그게 노이즈 한 번에 안 놓는 대가다.

    def test_한_사이클_0은_파지를_안_놓는다(self):
        # 센서 노이즈 한 번에 물체를 놓으면 안 된다.
        f = make(touch_confirm_cycles=1, contact_lost_cycles=5,
                 k_threshold=10.0)
        now, dt = 0.0, 0.05
        for _ in range(400):
            flex = max(0.0, (f.a - 0.04)) * FLEX_SPAN
            f.update(0.5 + 30.0 * flex, flex, now, dt)
            now += dt
            if f.state == grasp.HOLD:
                break
        f.update(0.0, 0.2, now, dt)
        assert f.state == grasp.HOLD


class Test뒤늦게_들어온_물체:
    """2026-08-14 페트병 실측에서 나온 결함.

    finger2/finger3 가 6.8초에 NO_CONTACT 로 떨어진 뒤 37초 동안 중앙값
    0.74~0.98N 으로 물체를 누르고 있었는데(사이클의 82~87% 가 f_touch
    초과), 상태기계는 허공이라고 믿고 a 를 a_max 에 고정한 채 방치했다.
    탈출구가 BACKOFF(8N) 하나뿐이었기 때문이다. 손가락 5개 중 3개가
    제어 밖에 있는 채로 파지하고 있었던 셈이다.
    """

    def _to_no_contact(self, f, dt=0.05):
        """허공을 쓸어 NO_CONTACT 까지 보낸다."""
        now = 0.0
        for _ in range(200):
            f.update(0.0, 0.0, now, dt)
            now += dt
            if f.state == grasp.NO_CONTACT:
                return now
        raise AssertionError("NO_CONTACT 에 도달하지 못했다")

    def test_힘이_연속으로_잡히면_힘_제어로_복귀한다(self):
        f = make(a_rate=1.0, a_max=0.5, touch_confirm_cycles=3)
        now = self._to_no_contact(f)
        for i in range(3):
            f.update(0.9, 0.5, now + i * 0.05, 0.05)
        assert f.state == grasp.HOLD

    def test_한_사이클_스파이크로는_안_깨어난다(self):
        # 허공에서 노이즈가 한 번 튈 때마다 되살아나면 진동한다.
        f = make(a_rate=1.0, a_max=0.5, touch_confirm_cycles=3)
        now = self._to_no_contact(f)
        f.update(0.9, 0.5, now, 0.05)
        f.update(0.0, 0.5, now + 0.05, 0.05)
        f.update(0.9, 0.5, now + 0.10, 0.05)
        assert f.state == grasp.NO_CONTACT

    def test_복귀할_때_프로빙을_건너뛴다(self):
        # 이미 a_max 라 계단을 밟을 여유가 없다. 표본 없이 CLASSIFY 로
        # 가므로 가장 보수적인 k_max 를 쓰고 confident 는 False 다.
        #
        # k_max 가 '보수적'인 것은 제어 게인 쪽 이야기다 -- Δa = λ·e/k_hat
        # 의 분모가 커져서 조금씩 움직인다. 파지력까지 세게 가라는 뜻이
        # 아니므로 분류는 soft 다(2026-08-21). 예전에는 여기가 rigid 였는데,
        # K_THRESHOLD 가 None 이라 어차피 목표힘이 안 갈렸을 때의 이야기다.
        # 임계값을 정한 뒤로는 "아무것도 못 쟀는데 더 세게 쥔다"가 되어
        # 규약("모르면 약한 쪽")과 정면으로 어긋난다.
        f = make(a_rate=1.0, a_max=0.5, touch_confirm_cycles=1,
                 k_max=500.0, k_threshold=10.0)
        now = self._to_no_contact(f)
        f.update(0.9, 0.5, now, 0.05)
        assert f.state == grasp.HOLD
        assert f.k_hat == pytest.approx(500.0)
        assert f.confident is False
        assert f.object_class == "soft"

    def test_복귀_직후에도_a는_a_max를_안_넘는다(self):
        f = make(a_rate=1.0, a_max=0.5, touch_confirm_cycles=1,
                 k_threshold=10.0)
        now = self._to_no_contact(f)
        for i in range(50):
            f.update(0.1, 0.5, now + i * 0.05, 0.05)   # 목표보다 훨씬 약함
        assert f.a <= 0.5 + 1e-12

    def test_힘이_안_잡히면_그대로_멈춰_있다(self):
        # 기존 동작을 깨면 안 된다. 허공에서는 여전히 a 가 고정이다.
        f = make(a_rate=1.0, a_max=0.5)
        now = self._to_no_contact(f)
        frozen_a = f.a
        for i in range(20):
            f.update(0.0, 0.0, now + i * 0.05, 0.05)
        assert f.state == grasp.NO_CONTACT
        assert f.a == pytest.approx(frozen_a)

    def test_복귀한_손가락은_NO_CONTACT로_되돌아가지_않는다(self):
        # 복귀한 손가락은 정의상 a_max 이고 프로빙을 못 해 k_max 를 쓰므로
        # 목표 힘에 못 미치는 게 당연하다. 예전에는 stall_timeout_s 만에
        # 무조건 NO_CONTACT 로 되돌아가 왕복만 했다(2026-08-14 로그:
        # 7.62s 복귀 -> 9.67s 되돌아감, 정확히 2.05초).
        # 이제는 목표가 실측치로 낮아져서 HOLD 에 머문다.
        f = make(a_rate=1.0, a_max=0.5, touch_confirm_cycles=1,
                 k_threshold=10.0, stall_timeout_s=0.3, f_target_rigid=2.0)
        now = self._to_no_contact(f)
        f.update(0.9, 0.5, now, 0.05)
        assert f.state == grasp.HOLD
        for i in range(1, 40):
            f.update(0.9, 0.5, now + i * 0.05, 0.05)
        assert f.state == grasp.HOLD
        assert f.f_target == pytest.approx(0.9)


class TestHold열림_제한:
    """2026-08-14 로그: 힘이 목표를 넘자 제어기가 14초에 걸쳐 a 를
    0.900 -> 0.602 로 풀었고 물체가 빠졌다. 방향은 제어식대로 맞았지만,
    이 물체는 펴도 힘이 잘 안 줄어서 접촉이 끊길 때까지 여는 것 말고
    종료 조건이 없었다.
    """

    def _to_hold(self, **over):
        """HOLD 까지 몰고 간다. -> (finger, now)

        접촉점을 a=0.5 로 잡는 이유: 진입 a 가 열림 한계보다 충분히 커야
        "얼마나 열리나"를 볼 수 있다. 접촉점이 0 근처면 진입 a 도 작아서
        바닥이 0 아래가 되고 테스트가 아무것도 검사하지 못한다.
        """
        f = make(k_threshold=10.0, **over)
        a_contact, k_true = 0.5, 30.0
        now, dt = 0.0, 0.05
        for _ in range(400):
            flex = max(0.0, (f.a - a_contact)) * FLEX_SPAN
            force = 0.0 if f.a <= a_contact else 0.5 + k_true * flex
            f.update(force, flex, now, dt)
            now += dt
            if f.state == grasp.HOLD:
                break
        assert f.state == grasp.HOLD
        assert f.a > 0.3, "열림 여유가 없으면 이 테스트는 의미가 없다"
        return f, now

    def test_진입_지점에서_한계_이상은_못_편다(self):
        f, now = self._to_hold(hold_open_limit=0.1)
        entry_a = f.a
        for i in range(300):
            # 목표보다 훨씬 센 힘을 계속 먹인다 -> 계속 풀려고 한다
            f.update(5.0, 0.5, now + i * 0.05, 0.05)
        assert f.a >= entry_a - 0.1 - 1e-9

    def test_제한이_없으면_예전처럼_끝까지_연다(self):
        # 기본값(None)을 둔 이유. 값을 안 주는 호출부는 동작이 안 바뀐다.
        f, now = self._to_hold()
        entry_a = f.a
        for i in range(300):
            f.update(5.0, 0.5, now + i * 0.05, 0.05)
        assert f.a < entry_a - 0.1

    def test_BACKOFF는_제한을_안_받는다(self):
        # 진짜 과부하 경로는 끝까지 물러설 수 있어야 한다.
        f, now = self._to_hold(hold_open_limit=0.1, f_abort=8.0,
                               abort_confirm_cycles=1, backoff_step=0.05)
        entry_a = f.a
        for i in range(6):
            f.update(50.0, 0.5, now + i * 0.05, 0.05)
        assert f.state == grasp.BACKOFF
        assert f.a < entry_a - 0.1

    def test_BACKOFF가_바닥도_같이_내린다(self):
        # 안 내리면 다음 HOLD 사이클이 a 를 바닥까지 되올려서 후퇴가
        # 무효가 된다.
        f, now = self._to_hold(hold_open_limit=0.1, f_abort=8.0,
                               abort_confirm_cycles=1, backoff_step=0.05)
        floor_before = f._hold_floor
        for i in range(5):
            f.update(50.0, 0.5, now + i * 0.05, 0.05)
        assert f.state == grasp.BACKOFF
        assert f._hold_floor < floor_before
        assert f._hold_floor <= f.a + 1e-9


class Test힘_상한은_연속_확인을_요구한다:
    """2026-08-14 로그: 앞뒤가 0.00N 인데 한 사이클만 18~49N 인 표본이
    세 번 나왔다. 이 손의 최대 파지력은 약 1.1N 이라 존재할 수 없는 값이다.

    센서 글리치든 물체가 미끄러지며 생긴 전단 과도응답이든 후퇴는 틀린
    대응이다 -- 미끄러지는 중에 손을 여는 것은 정확히 반대 행동이다.
    """

    def test_한_사이클_스파이크로는_후퇴하지_않는다(self):
        f = make(f_abort=8.0, abort_confirm_cycles=2)
        f.a = 0.5
        f.update(49.05, 0.1, now=0.00, dt=0.05)
        assert f.state != grasp.BACKOFF
        assert f.a == pytest.approx(0.5)      # 후퇴하지 않았다

    def test_연속으로_넘으면_후퇴한다(self):
        f = make(f_abort=8.0, abort_confirm_cycles=2, backoff_step=0.02)
        f.a = 0.5
        f.update(20.0, 0.1, now=0.00, dt=0.05)
        f.update(20.0, 0.1, now=0.05, dt=0.05)
        assert f.state == grasp.BACKOFF
        assert f.a == pytest.approx(0.48)

    def test_중간에_내려가면_확인_카운트가_초기화된다(self):
        f = make(f_abort=8.0, abort_confirm_cycles=2)
        f.update(20.0, 0.1, now=0.00, dt=0.05)
        f.update(0.0, 0.1, now=0.05, dt=0.05)     # 글리치가 꺼졌다
        f.update(20.0, 0.1, now=0.10, dt=0.05)
        assert f.state != grasp.BACKOFF

    def test_기본값_1이면_예전처럼_즉시_후퇴한다(self):
        # 값을 안 주는 기존 호출부의 동작이 안 바뀌어야 한다.
        f = make(f_abort=8.0)
        f.update(20.0, 0.1, now=0.0, dt=0.05)
        assert f.state == grasp.BACKOFF


class TestKFixed:
    """K_FIXED: 프로빙을 건너뛰고 강성을 상수로 쓴다.

    프로빙이 실제로 이득인지 재려고 넣은 비교용 스위치다. 파지력
    (f_target)은 건드리지 않는다 -- k_hat 은 '얼마나 빨리 조일지'의
    분모이지 '얼마나 세게 쥘지'가 아니다.
    """

    def _touch(self, f, n=2, force=0.9, flex=0.05):
        """접촉을 n 사이클 먹인다. -> 마지막 시각."""
        now = 0.0
        for _ in range(n):
            f.update(force, flex, now, 0.05)
            now += 0.05
        return now

    def test_프로빙을_건너뛰고_곧바로_HOLD로_간다(self):
        # 접촉 확인 사이클 + CLASSIFY 사이클 = 두 번이면 끝난다.
        # 프로빙이었으면 probe_steps 만큼 계단을 더 밟아야 한다.
        f = make(k_fixed=3.9, touch_confirm_cycles=1)
        self._touch(f, n=2)
        assert f.state == grasp.HOLD

    def test_PROBE를_한_번도_거치지_않는다(self):
        f = make(k_fixed=3.9, touch_confirm_cycles=1)
        history = run(f, [(0.9, 0.05)] * 6)
        assert grasp.PROBE not in history

    def test_k_hat이_설정값_그대로다(self):
        f = make(k_fixed=3.9, touch_confirm_cycles=1)
        self._touch(f, n=2)
        assert f.k_hat == pytest.approx(3.9)

    def test_confident는_True다(self):
        # 사람이 정한 값이라 '추정 실패'(False)를 붙이면 로그 해석이
        # 거꾸로 된다. confident=False 는 k_max 폴백 표시로 남겨 둔다.
        f = make(k_fixed=3.9, touch_confirm_cycles=1)
        self._touch(f, n=2)
        assert f.confident is True

    def test_제어기가_그_k_hat을_쓴다(self):
        f = make(k_fixed=3.9, touch_confirm_cycles=1)
        self._touch(f, n=2)
        assert f._controller.k_hat == pytest.approx(3.9)

    def test_파지력_목표는_안_바뀐다(self):
        # k_fixed 는 게인의 분모지 목표 힘이 아니다.
        f = make(k_fixed=3.9, touch_confirm_cycles=1,
                 k_threshold=10.0, f_target_rigid=2.0, f_target_soft=4.0)
        self._touch(f, n=2)
        assert f.f_target == pytest.approx(4.0)      # 3.9 < 10.0 -> soft

    def test_분류는_그대로_동작한다(self):
        f = make(k_fixed=20.0, touch_confirm_cycles=1,
                 k_threshold=10.0, f_target_rigid=2.0, f_target_soft=4.0)
        self._touch(f, n=2)
        assert f.object_class == "rigid"
        assert f.f_target == pytest.approx(2.0)

    def test_None이면_예전처럼_프로빙을_거친다(self):
        # 값을 안 주는 기존 호출부의 동작이 안 바뀌어야 한다.
        f = make(touch_confirm_cycles=1)
        history = run(f, [(0.9, 0.05)] * 4)
        assert grasp.PROBE in history

    def test_접촉이_확인되기_전에는_안_넘어간다(self):
        # 한 사이클 스파이크로 파지에 들어가면 안 된다.
        f = make(k_fixed=3.9, touch_confirm_cycles=3)
        f.update(0.9, 0.05, now=0.0, dt=0.05)
        assert f.state == grasp.APPROACH

    def test_NO_CONTACT_복귀도_같은_k를_쓴다(self):
        # 복귀 경로는 원래도 프로빙을 건너뛴다. k_fixed 가 있으면
        # k_max 폴백 대신 그 값을 써야 한다 -- 안 그러면 같은 물체를
        # 잡는데 진입 경로에 따라 게인이 128배 달라진다.
        f = make(k_fixed=3.9, a_rate=1.0, a_max=0.5,
                 touch_confirm_cycles=1, k_max=500.0)
        run(f, [(0.0, 0.0)] * 20, dt=0.1)
        assert f.state == grasp.NO_CONTACT
        f.update(0.9, 0.5, now=2.0, dt=0.05)
        assert f.state == grasp.HOLD
        assert f.k_hat == pytest.approx(3.9)


class Test손_전체_판정_받아들이기:
    """손 단위 분류 결과를 손가락에 밀어 넣는다.

    물체는 하나인데 지금까지는 손가락마다 따로 분류했다. 집계는 손
    전체를 보는 러너가 하고(grasp.py 는 손가락 하나짜리 순수 로직이라
    다른 손가락을 모른다), 그 결과를 이 통로로 받는다.
    """

    def _to_hold(self, f, k_true=2.0, a_contact=0.04):
        now, dt = 0.0, 0.05
        for _ in range(300):
            flex = max(0.0, (f.a - a_contact)) * FLEX_SPAN
            f.update(0.5 + k_true * flex, flex, now, dt)
            now += dt
            if f.state in (grasp.HOLD, grasp.BACKOFF):
                break

    def test_분류와_목표힘이_바뀐다(self):
        f = make(k_threshold=10.0, f_target_soft=1.2, f_target_rigid=1.8)
        self._to_hold(f, k_true=2.0)
        assert f.object_class == "soft"

        f.set_object("rigid", 1.8)

        assert f.object_class == "rigid"
        assert f.f_target == pytest.approx(1.8)

    def test_힘_제어기에도_반영된다(self):
        # 이게 없으면 f_target 만 바뀌고 제어기는 옛 목표를 계속 쫓는다.
        f = make(k_threshold=10.0, f_target_soft=1.2, f_target_rigid=1.8)
        self._to_hold(f, k_true=2.0)

        f.set_object("rigid", 1.8)

        assert f._controller.f_target == pytest.approx(1.8)

    def test_목표_천장도_같이_올라간다(self):
        # _f_target_max 는 stall 적응이 목표를 낮출 때의 상한이다.
        # 같이 올리지 않으면 승격 직후 stall 한 번에 다시 1.2 로
        # 깎여서 승격이 무효가 된다.
        f = make(k_threshold=10.0, f_target_soft=1.2, f_target_rigid=1.8)
        self._to_hold(f, k_true=2.0)

        f.set_object("rigid", 1.8)

        assert f._f_target_max == pytest.approx(1.8)

    def test_아직_제어기가_없어도_안_터진다(self):
        # APPROACH/PROBE 중인 손가락에도 손 전체 판정이 내려온다.
        # 다른 손가락이 먼저 CLASSIFY 를 끝냈을 때 정상적으로 생긴다.
        f = make(f_target_soft=1.2, f_target_rigid=1.8)
        assert f.state == grasp.APPROACH

        f.set_object("rigid", 1.8)

        assert f.object_class == "rigid"
        assert f.f_target == pytest.approx(1.8)


class Test측정_실패는_강체_증거가_아니다:
    """estimate_stiffness 는 실패 시 (k_max, False) 를 돌려준다.

    k_max 는 "가장 보수적인 분모"라는 뜻이지 "아주 단단하다"는 뜻이
    아니다. 그 값을 그대로 classify 에 넣으면 어떤 임계값이든 넘어서
    항상 rigid 가 된다.

    K_THRESHOLD 가 None 이던 동안에는 classify 가 무조건 soft 를
    돌려줘서 이 경로가 가려져 있었다. 임계값을 정하는 순간 드러난다.

    그리고 이 실패는 실제로 강체와 무관하다 -- 2026-08-18/19 로그의
    측정 실패 25건이 100% a=A_MAX 였다. PROBE 는 a 를 계단으로 올리며
    재는데 a 가 이미 천장이면 5샘플이 같은 자리에서 찍힌다. '밀었는데
    안 들어간' 게 아니라 '밀 여유가 없던' 것이다.
    """

    def test_확신이_없으면_강체로_분류하지_않는다(self):
        # 관절이 전혀 안 움직이는 물체. k_hat 은 k_max 로 떨어진다.
        f = make(k_threshold=50.0, f_target_soft=1.2, f_target_rigid=1.8)
        now, dt = 0.0, 0.05
        for _ in range(300):
            f.update(0.5, 0.0, now, dt)
            now += dt
            if f.state in (grasp.HOLD, grasp.BACKOFF):
                break

        assert f.confident is False
        assert f.k_hat == pytest.approx(f.params.k_max)
        assert f.object_class == "soft"
        assert f.f_target == pytest.approx(1.2)

    def test_확신이_있으면_평소대로_분류한다(self):
        # 위 보호장치가 정상 경로까지 막으면 안 된다.
        f = make(k_threshold=10.0, f_target_soft=1.2, f_target_rigid=1.8)
        now, dt = 0.0, 0.05
        for _ in range(300):
            flex = max(0.0, (f.a - 0.04)) * FLEX_SPAN
            f.update(0.5 + 50.0 * flex, flex, now, dt)
            now += dt
            if f.state in (grasp.HOLD, grasp.BACKOFF):
                break

        assert f.confident is True
        assert f.object_class == "rigid"
