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
        self.opened = []

    def read_flex(self):
        return {f: self.flex for f in hand_config.ACTIVE_FINGERS}

    def read_temperatures(self):
        return list(self.temps)

    def set_pose_map(self, targets, s=0.0):
        self.poses.append(dict(targets))

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

    def row(self, t, finger, force, flex):
        self.rows.append((t, finger.name, finger.state))


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
