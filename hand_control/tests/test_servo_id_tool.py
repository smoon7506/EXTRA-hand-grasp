# -*- coding: utf-8 -*-
"""servo_id_tool 의 순수 로직. 실물 서보 없이 안전 조건만 검증한다."""

import pytest

import servo_id_tool
from servo_id_tool import scan, set_id


class FakeSts:
    """STS 컨트롤러 대역. read_* 는 rustypot 처럼 리스트를 돌려준다."""

    def __init__(self, present=(11,), model=servo_id_tool.STS3215_MODEL,
                 mode=0):
        self.present = list(present)
        self.model = model
        self.mode = mode
        self.calls = []

    def ping(self, sid):
        return sid in self.present

    def read_model(self, sid):
        return [self.model]

    def read_mode(self, sid):
        return [self.mode]

    def read_present_position(self, sid):
        return [0.25]

    def write_lock(self, sid, value):
        self.calls.append(("write_lock", sid, value))

    def write_id(self, sid, new_id):
        self.calls.append(("write_id", sid, new_id))
        self.present = [new_id if s == sid else s for s in self.present]


# --- 스캔 ---------------------------------------------------------------


def test_scan_finds_only_the_servos_that_answer():
    fake = FakeSts(present=[3, 11])
    found = scan(fake, range(1, 13))
    assert [e["id"] for e in found] == [3, 11]
    assert found[0]["mode"] == 0


def test_scan_survives_a_read_that_raises():
    """읽기 하나가 터져서 스캔 전체가 죽으면 ID 를 영영 못 찾는다."""
    class Broken(FakeSts):
        def read_model(self, sid):
            raise RuntimeError("Parsing error")

    found = scan(Broken(present=[11]), [11])
    assert found[0]["id"] == 11
    assert found[0]["model"] is None


def test_a_ping_that_raises_is_treated_as_absent():
    class Raising(FakeSts):
        def ping(self, sid):
            raise RuntimeError("Timeout error")

    assert scan(Raising(), [11]) == []


# --- ID 변경 ------------------------------------------------------------


def test_set_id_refuses_when_more_than_one_servo_is_on_the_bus():
    """어느 개체가 바뀌었는지 알 수 없는 상태를 만들면 안 된다."""
    fake = FakeSts(present=[1, 11])
    found = scan(fake, [1, 11])
    with pytest.raises(RuntimeError):
        set_id(fake, found, 12)
    assert fake.calls == []          # EEPROM 을 건드리지 않았다


def test_set_id_refuses_the_hand_id_range():
    """손이 1~10 을 전부 쓴다. 겹치면 그 순간 둘 다 응답하지 않는다."""
    fake = FakeSts(present=[11])
    found = scan(fake, [11])
    with pytest.raises(ValueError):
        set_id(fake, found, 5)
    assert fake.calls == []


def test_set_id_unlocks_writes_then_locks_again():
    """lock 을 안 잠그면 이후 평범한 쓰기가 EEPROM 을 갉아먹는다."""
    fake = FakeSts(present=[11])
    found = scan(fake, [11])
    assert set_id(fake, found, 12) == 12
    assert fake.calls == [("write_lock", 11, 0),
                          ("write_id", 11, 12),
                          ("write_lock", 12, 1)]


def test_set_id_is_a_noop_when_it_is_already_right():
    fake = FakeSts(present=[11])
    found = scan(fake, [11])
    assert set_id(fake, found, 11) == 11
    assert fake.calls == []


# --- 표시 ---------------------------------------------------------------


def test_describe_warns_about_wheel_mode():
    fake = FakeSts(present=[11], mode=1)
    line = servo_id_tool.describe(scan(fake, [11])[0])
    assert "wheel" in line


def test_describe_warns_when_the_id_clashes_with_the_hand():
    fake = FakeSts(present=[5])
    line = servo_id_tool.describe(scan(fake, [5])[0])
    assert "1~10" in line
