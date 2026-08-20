# -*- coding: utf-8 -*-
"""링크가 끊겼을 때의 조치. 소켓은 없다 -- 시각만 다룬다."""

from grasp_state import (ALIGNING, ARMED, CONFIRMING, GRASPING, HOLDING,
                         RELEASING)
from link_watchdog import LinkWatchdog


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


class FakeMachine:
    """상태기계 흉내. 워치독이 무엇을 부르는지만 본다."""

    def __init__(self, state=ARMED):
        self.state = state
        self.armed = True
        self.calls = []

    def disarm(self):
        self.armed = False
        self.calls.append("disarm")

    def abort_to_armed(self):
        self.state = ARMED
        self.calls.append("abort_to_armed")

    def emergency_open(self):
        self.state = RELEASING
        self.calls.append("emergency_open")


def make(state=ARMED):
    clock = FakeClock()
    machine = FakeMachine(state)
    dog = LinkWatchdog(machine, timeout_s=2.0, hold_release_s=30.0,
                       clock=clock)
    dog.feed()                      # 붙은 상태에서 시작한다
    return dog, machine, clock


def test_connected_while_fed():
    dog, machine, clock = make()
    for _ in range(10):
        clock.advance(0.5)
        dog.feed()
        dog.tick()
    assert dog.connected is True
    assert machine.calls == []


def test_timeout_disarms():
    dog, machine, clock = make()
    clock.advance(2.1)
    dog.tick()
    assert dog.connected is False
    assert "disarm" in machine.calls


def test_aligning_aborts_on_loss():
    dog, machine, clock = make(ALIGNING)
    clock.advance(2.1)
    dog.tick()
    assert "abort_to_armed" in machine.calls


def test_confirming_aborts_on_loss():
    dog, machine, clock = make(CONFIRMING)
    clock.advance(2.1)
    dog.tick()
    assert "abort_to_armed" in machine.calls


def test_grasping_is_not_interrupted():
    """반쯤 닫힌 손이 멈춘 손보다 나쁘다."""
    dog, machine, clock = make(GRASPING)
    clock.advance(2.1)
    dog.tick()
    assert machine.calls == ["disarm"]


def test_holding_opens_after_the_delay():
    dog, machine, clock = make(HOLDING)
    clock.advance(2.1)
    dog.tick()
    assert "emergency_open" not in machine.calls
    clock.advance(29.0)
    dog.tick()
    assert "emergency_open" not in machine.calls     # 아직 30초 전
    clock.advance(1.1)
    dog.tick()
    assert "emergency_open" in machine.calls


def test_reconnect_cancels_the_release_timer():
    """링크 복구 = 사람이 돌아옴이다."""
    dog, machine, clock = make(HOLDING)
    clock.advance(2.1)
    dog.tick()
    assert dog.release_in() is not None
    clock.advance(5.0)
    dog.feed()
    dog.tick()
    assert dog.connected is True
    assert dog.release_in() is None
    clock.advance(60.0)
    dog.feed()
    dog.tick()
    assert "emergency_open" not in machine.calls


def test_grasping_that_finishes_while_down_starts_the_timer():
    """GRASPING 은 끝까지 가고, 도착한 HOLDING 에서 타이머가 선다."""
    dog, machine, clock = make(GRASPING)
    clock.advance(2.1)
    dog.tick()
    assert dog.release_in() is None
    machine.state = HOLDING              # 실행기가 도착을 알렸다
    dog.tick()
    assert dog.release_in() is not None


def test_drop_acts_immediately():
    """bye 는 워치독이 눈치채기를 기다릴 이유가 없다."""
    dog, machine, clock = make(ALIGNING)
    dog.drop()
    dog.tick()
    assert dog.connected is False
    assert "abort_to_armed" in machine.calls


def test_release_in_counts_down():
    dog, machine, clock = make(HOLDING)
    clock.advance(2.1)
    dog.tick()
    first = dog.release_in()
    clock.advance(10.0)
    dog.tick()
    assert dog.release_in() < first
