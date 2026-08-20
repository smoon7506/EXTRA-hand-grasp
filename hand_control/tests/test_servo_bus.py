# -*- coding: utf-8 -*-
"""ServoLease. 실물 시리얼 포트 없이 배타성만 검증한다."""

import pytest

from servo_bus import ServoLease


class FakeBus:
    """COM11 흉내. 이미 열려 있는데 또 열면 실물처럼 OSError 를 던진다.

    실물 STS 버스에서 실측된 동작이다. 이걸 흉내내지 않으면 테스트가
    통과해도 실물에서 그대로 죽는다.
    """

    def __init__(self):
        self.open_name = None
        self.history = []

    def open(self, name):
        if self.open_name is not None:
            raise OSError(f"액세스가 거부되었습니다 ({self.open_name} 이 "
                          f"이미 열고 있음)")
        self.open_name = name
        self.history.append(("open", name))
        return f"handle:{name}"

    def close(self, handle):
        self.history.append(("close", self.open_name))
        self.open_name = None


def make(bus, name):
    return (lambda: bus.open(name), lambda handle: bus.close(handle))


# --- 배타성 -------------------------------------------------------------


def test_acquiring_a_second_owner_releases_the_first():
    """이게 이 파일의 존재 이유다.

    손 컨트롤러가 열린 채 손목 컨트롤러를 열면 실물에서 OSError 가 난다.
    """
    bus = ServoLease()
    fake = FakeBus()
    open_hand, close_hand = make(fake, "hand")
    open_wrist, close_wrist = make(fake, "wrist")

    bus.acquire("hand", open_hand, close_hand)
    assert fake.open_name == "hand"
    bus.acquire("wrist", open_wrist, close_wrist)      # 여기서 안 죽어야 한다
    assert fake.open_name == "wrist"
    assert fake.history == [("open", "hand"), ("close", "hand"),
                            ("open", "wrist")]


def test_reacquiring_the_same_owner_does_not_reopen():
    """매 프레임 acquire 를 불러도 포트를 다시 열지 않는다.

    다시 열면 재연결 비용(10개 서보 x 4번 왕복)을 프레임마다 낸다.
    """
    bus = ServoLease()
    fake = FakeBus()
    open_hand, close_hand = make(fake, "hand")

    first = bus.acquire("hand", open_hand, close_hand)
    second = bus.acquire("hand", open_hand, close_hand)
    assert first is second
    assert fake.history == [("open", "hand")]


def test_release_frees_the_port():
    bus = ServoLease()
    fake = FakeBus()
    bus.acquire("hand", *make(fake, "hand"))
    bus.release()
    assert fake.open_name is None
    assert bus.owner() is None


def test_release_is_idempotent():
    # 종료 경로에서 두 번 불릴 수 있다. 죽으면 안 된다.
    bus = ServoLease()
    bus.release()
    bus.release()
    assert bus.owner() is None


def test_owner_reports_who_holds_the_port():
    bus = ServoLease()
    fake = FakeBus()
    assert bus.owner() is None
    bus.acquire("wrist", *make(fake, "wrist"))
    assert bus.owner() == "wrist"


# --- 실패해도 포트를 놔야 한다 -------------------------------------------


def test_a_failing_close_still_frees_the_lease():
    """close 가 터져도 리스는 비워야 한다.

    안 비우면 그 뒤로 아무도 포트를 못 잡아, 비상 폄까지 막힌다.
    """
    bus = ServoLease()

    def bad_close(handle):
        raise RuntimeError("닫다가 터짐")

    bus.acquire("hand", lambda: "handle", bad_close)
    with pytest.raises(RuntimeError):
        bus.release()
    assert bus.owner() is None


def test_a_failing_open_leaves_the_lease_empty():
    """열다 실패하면 아무도 안 쥔 상태여야 한다.

    주인만 기록해 두면 다음 acquire 가 '이미 갖고 있다'며 없는 자원을
    돌려준다.
    """
    bus = ServoLease()
    fake = FakeBus()
    bus.acquire("hand", *make(fake, "hand"))

    def bad_open():
        raise OSError("포트 없음")

    with pytest.raises(OSError):
        bus.acquire("wrist", bad_open, lambda h: None)
    assert bus.owner() is None
    assert fake.open_name is None      # 손은 이미 반납됐다
