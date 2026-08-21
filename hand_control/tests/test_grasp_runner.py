# -*- coding: utf-8 -*-
"""ForceGraspRunner. 촉각 센서도 모터도 없이 루프 골격만 검증한다."""

import grasp
import hand_config
import pytest
from grasp_runner import ForceGraspRunner


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


class FakeSensors:
    def __init__(self, force=0.0):
        self.force = force

    def read_forces(self):
        return {f: self.force for f in hand_config.ACTIVE_FINGERS}


class FakeHand:
    """Hand 대역. 관절은 명령을 그대로 따라간다고 본다."""

    def __init__(self, temps=(30.0,), flex=0.0):
        self.temps = list(temps)
        self.flex = flex
        self.poses = []
        self.spreads = []
        self.opened = []

    def read_flex(self):
        return {f: self.flex for f in hand_config.ACTIVE_FINGERS}

    def read_temperatures(self):
        return list(self.temps)

    def set_pose_map(self, targets, s=0.0):
        self.poses.append(dict(targets))
        self.spreads.append(dict(s) if isinstance(s, dict) else s)

    def set_pose(self, a, s=0.0, fingers=None):
        self.opened.append((a, s, [f.name for f in (fingers or [])]))


class StubGrasp:
    """FingerGrasp 대역. 러너가 실제로 쓰는 것만 갖는다: name/state/update.

    진짜 FingerGrasp 에 `state = HOLD` 를 박으면 안 된다. HOLD 는
    CLASSIFY 가 힘 제어기를 붙여 준 뒤에만 성립하는 상태라, 상태만
    바꿔치기하면 다음 update() 에서 `_controller` 가 None 이라 터진다.
    여기서 검증하려는 것은 러너의 안정 판정이지 grasp.py 의 전이가
    아니므로, 상태를 고정할 수 있는 대역을 쓴다.
    """

    def __init__(self, name, state):
        self.name = name
        self.state = state
        self.a = 0.0

    def update(self, force, flex, now, dt):
        return self.a


class FakeLogger:
    def __init__(self):
        self.path = "fake.csv"
        self.rows = []

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False

    def row(self, t, finger, force, flex, shear=None):
        self.rows.append((t, finger.name, finger.state, shear))


def make(fingers=None, sensors=None, hand=None, clock=None):
    fingers = fingers or hand_config.load_fingers(
        active=hand_config.ACTIVE_FINGERS)
    return ForceGraspRunner(
        fingers,
        sensors or FakeSensors(),
        hand or FakeHand(),
        clock=clock or FakeClock(),
        log_factory=FakeLogger,
    )


# --- 레이트리밋 ---------------------------------------------------------


def test_tick_does_not_advance_faster_than_loop_hz():
    """카메라는 30fps 인데 제어 루프는 10Hz 다.

    레이트리밋이 없으면 프레임마다 명령이 나가 오버슈트한다. LOOP_HZ 를
    20에서 10으로 내린 것과 같은 이유다.
    """
    clock = FakeClock()
    hand = FakeHand()
    runner = make(hand=hand, clock=clock)
    runner.start_grasp()
    runner.tick()
    sent = len(hand.poses)
    clock.advance(1.0 / hand_config.LOOP_HZ / 4.0)
    runner.tick()
    assert len(hand.poses) == sent          # 아직 이번 사이클이 아니다
    clock.advance(1.0 / hand_config.LOOP_HZ)
    runner.tick()
    assert len(hand.poses) == sent + 1


# --- 안정 판정 ----------------------------------------------------------


def test_settled_only_after_consecutive_stable_cycles():
    """한 사이클 HOLD 를 보고 '잡았다'로 넘어가면 안 된다."""
    clock = FakeClock()
    runner = make(clock=clock)
    runner.start_grasp()
    for name in list(runner.states):
        runner.states[name] = StubGrasp(name, grasp.HOLD)
    seen = []
    for _ in range(hand_config.GRASP_STABLE_CYCLES + 2):
        clock.advance(1.0 / hand_config.LOOP_HZ)
        seen.append(runner.tick())
    assert "settled" in seen
    assert seen.index("settled") >= hand_config.GRASP_STABLE_CYCLES - 1


def test_a_finger_still_approaching_keeps_it_busy():
    clock = FakeClock()
    runner = make(clock=clock)
    runner.start_grasp()
    names = list(runner.states)
    for name in names:
        runner.states[name] = StubGrasp(name, grasp.HOLD)
    runner.states[names[0]] = StubGrasp(names[0], grasp.APPROACH)
    for _ in range(hand_config.GRASP_STABLE_CYCLES + 3):
        clock.advance(1.0 / hand_config.LOOP_HZ)
        assert runner.tick() == "busy"


# --- 온도 감시가 따라와야 한다 ------------------------------------------


def test_overheating_aborts():
    clock = FakeClock()
    hand = FakeHand(temps=(hand_config.TEMP_LIMIT_C + 5.0,))
    runner = make(hand=hand, clock=clock)
    runner.start_grasp()
    status = "busy"
    for _ in range(int(hand_config.TEMP_CHECK_INTERVAL_S
                       * hand_config.LOOP_HZ) + 3):
        clock.advance(1.0 / hand_config.LOOP_HZ)
        status = runner.tick()
        if status == "abort":
            break
    assert status == "abort"


def test_repeated_temperature_read_failures_abort():
    """온도를 계속 못 읽으면 감시가 없는 채로 파지를 이어가는 셈이다.

    과열 가드가 마지막 안전장치인데 소리 없이 사라지면 안 된다.
    """
    clock = FakeClock()
    hand = FakeHand(temps=())          # 빈 리스트 = 하나도 못 읽음
    runner = make(hand=hand, clock=clock)
    runner.start_grasp()
    status = "busy"
    for _ in range(200):
        clock.advance(1.0 / hand_config.LOOP_HZ)
        status = runner.tick()
        if status == "abort":
            break
    assert status == "abort"


# --- 폄 ---------------------------------------------------------------


def test_open_finishes_and_reports_opened():
    clock = FakeClock()
    hand = FakeHand()
    runner = make(hand=hand, clock=clock)
    runner.start_open()
    status = "busy"
    for _ in range(200):
        clock.advance(1.0 / hand_config.LOOP_HZ)
        status = runner.tick()
        if status == "opened":
            break
    assert status == "opened"
    assert hand.opened                     # 실제로 편 자세를 보냈다


def test_open_sends_the_thumb_first():
    """주먹에서 전 손가락을 한 번에 펴면 엄지와 검지가 부딪힌다.

    hand_config.OPEN_DELAY_S 의 순서를 그대로 따라야 한다.
    """
    clock = FakeClock()
    hand = FakeHand()
    runner = make(hand=hand, clock=clock)
    runner.start_open()
    for _ in range(200):
        clock.advance(1.0 / hand_config.LOOP_HZ)
        if runner.tick() == "opened":
            break
    first_group = hand.opened[0][2]
    assert "r_finger5" in first_group


# --- 손 전체 강성 분류 ---------------------------------------------------


class ClassifiableStub(StubGrasp):
    """k_hat 을 들고 손 전체 판정을 받을 수 있는 대역."""

    def __init__(self, name, state, k_hat=None, confident=None):
        super().__init__(name, state)
        self.k_hat = k_hat
        self.confident = confident
        self.object_class = None
        self.f_target = None
        self.applied = []

    def set_object(self, object_class, f_target):
        self.object_class = object_class
        self.f_target = f_target
        self.applied.append((object_class, f_target))


def classifying_runner(ks, clock=None, **over):
    """ks: {손가락이름: (k_hat, confident)} 로 러너를 세운다."""
    from grasp import GraspParams

    base = dict(GraspParams.from_config().__dict__)
    base.update(k_threshold=10.0, f_target_soft=1.2, f_target_rigid=1.8)
    base.update(over)
    clock = clock or FakeClock()
    runner = ForceGraspRunner(
        hand_config.load_fingers(active=list(ks)),
        FakeSensors(), FakeHand(), clock=clock,
        params=GraspParams(**base), log_factory=FakeLogger)
    runner.start_grasp()
    for name, (k, ok) in ks.items():
        runner.states[name] = ClassifiableStub(name, grasp.HOLD, k, ok)
    return runner, clock


def test_한_손가락이라도_임계값을_넘으면_손_전체가_rigid():
    """물체는 하나다. 손가락마다 rigid/soft 가 갈리는 건 말이 안 된다.

    접촉이 나쁜 손가락은 물체가 아니라 자기 접촉을 재서 낮은 k 를
    준다. 강체의 증거는 제대로 닿은 손가락 하나로 성립한다.
    """
    runner, clock = classifying_runner({
        "r_finger1": (2.0, True),      # 스치듯 닿음
        "r_finger2": (50.0, True),     # 제대로 닿음 -> 강체 증거
        "r_finger3": (3.0, True),
    })
    clock.advance(1.0 / hand_config.LOOP_HZ)
    runner.tick()

    assert runner.object_class == "rigid"
    for state in runner.states.values():
        assert state.object_class == "rigid"
        assert state.f_target == pytest.approx(1.8)


def test_측정_실패한_손가락은_rigid로_오판하지_않는다():
    """estimate_stiffness 는 실패 시 (K_MAX, False) 를 돌려준다.

    그걸 최대에 넣으면 손가락 하나만 실패해도 항상 rigid 가 된다.
    게다가 실측 25건의 측정 실패가 100% a=A_MAX 였다 -- '밀었는데 안
    들어간' 게 아니라 '밀 여유가 없던' 경우라 강체 증거가 아니다.
    """
    runner, clock = classifying_runner({
        "r_finger1": (2.0, True),
        "r_finger2": (500.0, False),   # 측정 실패
    })
    clock.advance(1.0 / hand_config.LOOP_HZ)
    runner.tick()

    assert runner.object_class == "soft"


def test_한번_rigid면_강등되지_않는다():
    """승격 전용. "하나라도 hard 면 hard" 규약이 시간축으로도 성립해야 한다.

    손가락마다 CLASSIFY 시점이 다르고, HOLD 에서 접촉이 끊긴 손가락은
    APPROACH 로 돌아가 다시 프로빙하며 더 낮은 k_hat 을 쓸 수 있다.
    그때 손 전체가 soft 로 되돌아가면 목표힘이 내려가 물체를 놓친다.
    """
    runner, clock = classifying_runner({"r_finger1": (50.0, True)})
    clock.advance(1.0 / hand_config.LOOP_HZ)
    runner.tick()
    assert runner.object_class == "rigid"

    runner.states["r_finger1"].k_hat = 2.0      # 재측정이 낮게 나왔다
    clock.advance(1.0 / hand_config.LOOP_HZ)
    runner.tick()

    assert runner.object_class == "rigid"


def test_늦게_분류된_손가락도_손_전체_판정을_받는다():
    """손가락마다 PROBE 가 끝나는 시점이 다르다.

    먼저 끝난 손가락이 rigid 를 확정한 뒤 합류한 손가락이 자기
    CLASSIFY 값(soft)을 그대로 쓰면, 같은 물체를 손가락마다 다른
    힘으로 잡는다.
    """
    runner, clock = classifying_runner({
        "r_finger1": (50.0, True),
        "r_finger2": (None, None),      # 아직 프로빙 중
    })
    clock.advance(1.0 / hand_config.LOOP_HZ)
    runner.tick()
    late = runner.states["r_finger2"]
    assert late.object_class == "rigid"

    late.k_hat, late.confident = 2.0, True      # 이제 자기 값이 나왔다
    clock.advance(1.0 / hand_config.LOOP_HZ)
    runner.tick()

    assert late.object_class == "rigid"


def test_아무도_못_쟀으면_분류하지_않는다():
    runner, clock = classifying_runner({"r_finger1": (500.0, False)})
    clock.advance(1.0 / hand_config.LOOP_HZ)
    runner.tick()

    assert runner.k_hand is None
    assert runner.object_class is None


# --- 전단력 로깅 --------------------------------------------------------


class ShearSensors(FakeSensors):
    """전단력까지 주는 센서 대역."""

    def __init__(self, force=0.0, shear=0.0):
        super().__init__(force)
        self.shear = shear
        self.shear_calls = 0

    def read_shear(self):
        self.shear_calls += 1
        return {f: self.shear for f in hand_config.ACTIVE_FINGERS}


def test_전단력을_매_사이클_로그에_남긴다():
    """슬립은 수직력에 안 보인다. tf 를 안 쌓으면 임계값을 못 정한다."""
    clock = FakeClock()
    sensors = ShearSensors(force=0.5, shear=1.25)
    runner = make(sensors=sensors, clock=clock)
    runner.start_grasp()
    clock.advance(1.0 / hand_config.LOOP_HZ)
    runner.tick()

    assert sensors.shear_calls == 1
    assert runner._log.rows, "로그가 비어 있다"
    assert all(r[-1] == pytest.approx(1.25) for r in runner._log.rows)


def test_전단력을_못_주는_센서에도_안_터진다():
    # read_shear 가 없는 옛 대역/구현이 남아 있어도 파지는 계속돼야
    # 한다. 로깅은 부가기능이고 제어를 막으면 안 된다.
    clock = FakeClock()
    runner = make(sensors=FakeSensors(force=0.5), clock=clock)
    runner.start_grasp()
    clock.advance(1.0 / hand_config.LOOP_HZ)
    assert runner.tick() in ("busy", "settled")
    assert all(r[-1] is None for r in runner._log.rows)


# --- 벌림 탐색 ----------------------------------------------------------


def seeking_runner(states, clock=None, **over):
    """states: {손가락: FingerGrasp 상태}. 대역으로 세운다."""
    clock = clock or FakeClock()
    hand = FakeHand()
    runner = make(sensors=ShearSensors(force=0.0), hand=hand, clock=clock)
    runner.start_grasp()
    for name, st in states.items():
        runner.states[name] = StubGrasp(name, st)
    for k, v in over.items():
        setattr(runner, k, v)
    return runner, clock, hand


def spin(runner, clock, n):
    out = []
    for _ in range(n):
        clock.advance(1.0 / hand_config.LOOP_HZ)
        out.append(runner.tick())
    return out


def test_접촉이_부족하면_탐색에_들어간다():
    """허공을 쥔 손가락이 있으면 그냥 포기하지 않고 옆을 훑어본다."""
    names = hand_config.ACTIVE_FINGERS
    states = {n: grasp.NO_CONTACT for n in names}
    states[names[0]] = grasp.HOLD
    runner, clock, _ = seeking_runner(states)
    spin(runner, clock, hand_config.GRASP_STABLE_CYCLES + 2)
    assert runner.seeking is True


def test_접촉이_충분하면_탐색을_건너뛴다():
    names = hand_config.ACTIVE_FINGERS
    runner, clock, _ = seeking_runner({n: grasp.HOLD for n in names})
    out = spin(runner, clock, hand_config.GRASP_STABLE_CYCLES + 2)
    assert runner.seeking is False
    assert "settled" in out


def test_탐색_중에는_접촉한_손가락의_벌림이_0이다():
    """같이 움직이면 마찰이 줄어 잡고 있던 물체를 놓는다."""
    names = hand_config.ACTIVE_FINGERS
    states = {n: grasp.NO_CONTACT for n in names}
    states[names[0]] = grasp.HOLD
    runner, clock, hand = seeking_runner(states)
    spin(runner, clock, hand_config.GRASP_STABLE_CYCLES + 4)
    assert runner.seeking is True
    assert isinstance(hand.spreads[-1], dict)
    assert hand.spreads[-1][names[0]] == 0.0


def test_탐색이_끝나면_settled로_간다():
    names = hand_config.ACTIVE_FINGERS
    states = {n: grasp.NO_CONTACT for n in names}
    states[names[0]] = grasp.HOLD
    runner, clock, _ = seeking_runner(states)
    out = spin(runner, clock, 400)
    assert runner.seeking is False
    assert "settled" in out


def test_탐색은_파지당_한_번만_한다():
    # 매번 다시 훑으면 손이 영원히 좌우로 떨린다.
    names = hand_config.ACTIVE_FINGERS
    states = {n: grasp.NO_CONTACT for n in names}
    states[names[0]] = grasp.HOLD
    runner, clock, _ = seeking_runner(states)
    spin(runner, clock, 400)
    assert runner.seek_done is True
    spin(runner, clock, 40)
    assert runner.seeking is False


def test_탐색이_끝나면_다시_안정될_때까지_기다린다():
    """최적점을 찾자마자 settled 로 가면 안 된다.

    NO_CONTACT 손가락은 힘이 touch_confirm_cycles 연속으로 잡혀야
    되살아난다. 벌림을 옮긴 바로 그 사이클에 '다 됐다'고 알리면,
    애써 찾은 자리에서 실제로 물리기도 전에 상태기계가 HOLDING 으로
    가버린다 -- 탐색이 아무것도 안 바꾼 것과 같아진다.
    """
    names = hand_config.ACTIVE_FINGERS
    states = {n: grasp.NO_CONTACT for n in names}
    states[names[0]] = grasp.HOLD
    runner, clock, _ = seeking_runner(states)

    out = []
    for _ in range(400):
        clock.advance(1.0 / hand_config.LOOP_HZ)
        out.append(runner.tick())
        if runner.seek_done and not runner.seeking:
            break
    # 탐색이 끝난 바로 그 사이클은 아직 settled 가 아니다.
    assert out[-1] == "busy"
    assert runner._stable == 0
