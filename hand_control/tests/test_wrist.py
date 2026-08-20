# -*- coding: utf-8 -*-
"""Sts3215Wrist. 실물 서보 없이 호출 순서와 안전 조건만 검증한다."""

import math

import pytest

from wrist import Sts3215Wrist


class FakeSts:
    """STS3215 컨트롤러 대역.

    rustypot 의 성질을 그대로 흉내낸다:
      - read_* 는 단일 ID 를 넣어도 **리스트**를 돌려준다
      - ping 은 없는 ID 에서 False 를 주기도 하고 예외를 던지기도 한다
    """

    def __init__(self, mode=0, present=0.25, alive=True):
        self.mode = mode
        self.present = present
        self.alive = alive
        self.calls = []

    def ping(self, sid):
        self.calls.append(("ping", sid))
        return self.alive

    def read_mode(self, sid):
        self.calls.append(("read_mode", sid))
        return [self.mode]

    def read_present_position(self, sid):
        self.calls.append(("read_present_position", sid))
        return [self.present]

    def write_goal_position(self, sid, rad):
        self.calls.append(("write_goal_position", sid, rad))

    def write_goal_speed(self, sid, speed):
        self.calls.append(("write_goal_speed", sid, speed))

    def write_torque_enable(self, sid, on):
        self.calls.append(("write_torque_enable", sid, on))

    def names(self):
        return [c[0] for c in self.calls]


def make(fake, **over):
    kwargs = dict(servo_id=11, port="COM11", baudrate=1000000,
                  timeout_s=0.5, speed=3,
                  min_rad=math.radians(-30.0), max_rad=math.radians(30.0),
                  factory=lambda **kw: fake)
    kwargs.update(over)
    return Sts3215Wrist(**kwargs)


# --- 연결 순서 ----------------------------------------------------------


def test_connect_reads_position_before_enabling_torque():
    """토크를 먼저 켜면 레지스터에 남은 옛 goal 로 손목이 튄다.

    손목 위에는 카메라가 실려 있다. 손 연결 순서와 같은 규약이다.
    """
    fake = FakeSts(present=0.25)
    make(fake).connect()
    names = fake.names()
    assert names.index("read_present_position") < names.index("write_goal_position")
    assert names.index("write_goal_position") < names.index("write_torque_enable")
    assert names.index("write_goal_speed") < names.index("write_torque_enable")


def test_connect_writes_the_present_position_as_the_goal():
    """'제자리 유지'로 켜야 한다. 다른 값을 쓰면 그리로 튄다."""
    fake = FakeSts(present=0.25)
    make(fake).connect()
    goal = [c for c in fake.calls if c[0] == "write_goal_position"][0]
    assert goal[2] == pytest.approx(0.25)


def test_connect_uses_a_bool_for_torque_enable():
    """STS3215 의 write_torque_enable 은 bool 만 받는다.

    SCS0009 는 0/1 을 받는다. 섞으면 손목에서만 조용히 실패한다.
    """
    fake = FakeSts()
    make(fake).connect()
    call = [c for c in fake.calls if c[0] == "write_torque_enable"][0]
    assert call[2] is True


# --- 안전 조건 ----------------------------------------------------------


def test_wheel_mode_is_refused_and_torque_stays_off():
    """mode=1 은 연속 회전 모드다.

    그 상태에서 goal 을 쓰면 **속도 명령**으로 해석돼 손목이 그냥
    돌아간다. EEPROM 이라 전원을 꺼도 남는다.
    """
    fake = FakeSts(mode=1)
    with pytest.raises(RuntimeError, match="mode"):
        make(fake).connect()
    assert "write_torque_enable" not in fake.names()


def test_a_servo_that_does_not_answer_is_refused():
    fake = FakeSts(alive=False)
    with pytest.raises(RuntimeError):
        make(fake).connect()
    assert "write_torque_enable" not in fake.names()


def test_a_ping_that_raises_is_treated_as_absent():
    """ping 은 두 가지로 실패한다: False 반환, 그리고 예외.

    예외를 안 잡으면 '서보가 없다'가 스택 트레이스로 튀어나온다.
    """
    class Raising(FakeSts):
        def ping(self, sid):
            raise RuntimeError("Timeout error")

    with pytest.raises(RuntimeError):
        make(Raising()).connect()


# --- 목표 쓰기 ----------------------------------------------------------


def test_write_goal_clamps_to_the_range():
    fake = FakeSts()
    wrist = make(fake)
    wrist.connect()
    sent = wrist.write_goal(math.radians(80.0))
    assert sent == pytest.approx(math.radians(30.0))
    last = [c for c in fake.calls if c[0] == "write_goal_position"][-1]
    assert last[2] == pytest.approx(math.radians(30.0))


def test_read_position_unwraps_the_list():
    fake = FakeSts(present=0.4)
    wrist = make(fake)
    wrist.connect()
    assert wrist.read_position() == pytest.approx(0.4)


def test_read_position_returns_none_on_a_failure():
    """읽기 실패로 루프가 죽으면 손목이 중간 자세에 방치된다."""
    class Broken(FakeSts):
        def read_present_position(self, sid):
            if len(self.calls) > 3:
                raise RuntimeError("Parsing error")
            self.calls.append(("read_present_position", sid))
            return [self.present]

    wrist = make(Broken())
    wrist.connect()
    assert wrist.read_position() is None


def test_write_goal_before_connect_raises():
    """포트가 없는데 조용히 성공하면 '왜 안 도는지'를 못 찾는다."""
    with pytest.raises(RuntimeError):
        make(FakeSts()).write_goal(0.0)


# --- 반납 ---------------------------------------------------------------


def test_disconnect_keeps_torque_by_default():
    """리스를 비켜 줄 뿐이다. 손목이 카메라를 든 채 늘어지면 안 된다."""
    fake = FakeSts()
    wrist = make(fake)
    wrist.connect()
    fake.calls.clear()
    wrist.disconnect()
    assert "write_torque_enable" not in fake.names()


def test_disconnect_can_turn_torque_off_for_shutdown():
    fake = FakeSts()
    wrist = make(fake)
    wrist.connect()
    fake.calls.clear()
    wrist.disconnect(torque_off=True)
    call = [c for c in fake.calls if c[0] == "write_torque_enable"][0]
    assert call[2] is False
