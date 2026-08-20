# -*- coding: utf-8 -*-
"""무장/해제. 사람이 안 보고 있으면 새 파지를 시작하지 않는다."""

from grasp_state import (ARMED, CONFIRMING, GRASPING, GraspStateMachine,
                         SequenceExecutor)
from roi_judge import RatioTrigger

# tests/ 는 패키지가 아니라서 test_roi_grasp 에서 import 할 수 없다.
# 같은 대역을 여기 다시 둔다 (test_roi_grasp.py:163-194 와 같은 것).
HIT = 0.9


class FakeSeq:
    def __init__(self, ticks_to_finish=2):
        self.ticks_to_finish = ticks_to_finish
        self.starts = []
        self._left = 0

    def start(self, a, s=0.0):
        self.starts.append((a, s))
        self._left = self.ticks_to_finish

    def tick(self):
        if self._left <= 0:
            return False
        self._left -= 1
        return self._left > 0


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


def make_machine(armed=True):
    """test_roi_grasp.py:196-206 의 make_machine 과 같은 값이다.

    ticks=2 / settle=1.0 을 그대로 쓰는 이유: 1/0.0 으로 두면 파지를
    시작한 다음 프레임에 실행기가 곧바로 "settled" 를 내서 HOLDING 으로
    넘어간다. GRASPING 에 머무는 것을 보려면 기존 값이어야 한다.
    """
    seq, clock = FakeSeq(ticks_to_finish=2), FakeClock()
    trigger = RatioTrigger(enter_ratio=0.3, exit_ratio=0.15, enter_frames=1)
    executor = SequenceExecutor(seq, grasp_a=0.8, settle_s=1.0, clock=clock)
    machine = GraspStateMachine(executor, trigger, aligner=None, clock=clock,
                                rearm_s=0.0, confirm_hold_s=0.0, armed=armed)
    return machine, seq, clock


def test_default_is_armed():
    """기존 테스트 107개가 이 기본값에 기대고 있다."""
    machine, _, _ = make_machine()
    assert machine.armed is True


def test_disarmed_never_grasps():
    machine, seq, _ = make_machine(armed=False)
    for _ in range(20):
        assert machine.update(HIT) == ARMED
    assert seq.starts == []


def test_arming_lets_it_grasp_again():
    machine, seq, _ = make_machine(armed=False)
    machine.update(HIT)
    assert seq.starts == []
    machine.arm()
    # 정렬기가 없어도 CONFIRMING 을 한 프레임 거친다. 창 길이가 0 이라
    # 다음 프레임에 GRASPING 이 된다 (test_roi_grasp.py:208-215 와 같다).
    assert machine.update(HIT) == CONFIRMING
    assert machine.update(HIT) == GRASPING
    assert seq.starts == [(0.8, 0.0)]


def test_disarm_does_not_freeze_a_running_grasp():
    """진행 중인 동작은 얼리지 않는다. 반쯤 닫힌 손이 더 나쁘다."""
    machine, _, _ = make_machine()
    machine.update(HIT)                          # -> CONFIRMING
    assert machine.update(HIT) == GRASPING
    machine.disarm()
    assert machine.update(None) == GRASPING


def test_abort_to_armed_clears_the_streak():
    machine, seq, _ = make_machine()
    machine.update(HIT)
    machine.abort_to_armed()
    assert machine.state == ARMED
    assert machine.trigger.active is False
