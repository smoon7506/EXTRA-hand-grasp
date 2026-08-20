# -*- coding: utf-8 -*-
"""보내다 막히면 버린다. 제어 루프는 절대 안 멈춘다."""

import socket

from link_sender import DropSender


class FakeSock:
    def __init__(self, behavior="ok", incoming=b""):
        self.behavior = behavior
        self.incoming = incoming
        self.sent = []

    def send(self, blob):
        if self.behavior == "block":
            raise BlockingIOError()
        if self.behavior == "reset":
            raise ConnectionResetError()
        if self.behavior == "partial":
            self.sent.append(blob[:1])
            return 1
        self.sent.append(blob)
        return len(blob)

    def recv(self, n):
        if self.behavior == "block":
            raise BlockingIOError()
        if self.behavior == "reset":
            raise ConnectionResetError()
        out, self.incoming = self.incoming[:n], self.incoming[n:]
        return out

    def close(self):
        self.behavior = "closed"


def test_sends_when_the_socket_is_willing():
    sock = FakeSock()
    s = DropSender(sock)
    assert s.send(b"hello") is True
    assert sock.sent == [b"hello"]
    assert s.dropped == 0


def test_drops_instead_of_blocking():
    """이것이 이 파일의 존재 이유다."""
    s = DropSender(FakeSock("block"))
    assert s.send(b"hello") is False
    assert s.dropped == 1
    assert s.connected is True          # 끊긴 건 아니다


def test_a_reset_detaches_the_socket():
    s = DropSender(FakeSock("reset"))
    assert s.send(b"hello") is False
    assert s.connected is False


class ChokedSock:
    """한 번에 limit 바이트만 받는 소켓. 논블로킹 소켓의 실제 모습이다.

    보낸 것을 순서대로 이어 붙여 두므로, 받는 쪽이 본 바이트열을 그대로
    재현할 수 있다 -- 프레임이 잘렸는지 확인하는 데 이게 필요하다.
    """

    def __init__(self, limit=4):
        self.limit = limit
        self.wire = b""

    def send(self, blob):
        n = min(self.limit, len(blob))
        self.wire += blob[:n]
        return n

    def close(self):
        pass


def test_partial_send_keeps_the_tail():
    """이미 나간 바이트는 되돌릴 수 없다. 나머지를 반드시 마저 보낸다.

    "절반 보내고 나머지는 버린다"로 두면 받는 쪽은 잘린 프레임을 받고,
    다음 프레임의 길이 필드를 JPEG 한가운데에서 읽는다. 실제로
    "미리보기 프레임이 3537097941 바이트다"로 터졌다.
    """
    sock = ChokedSock(limit=4)
    s = DropSender(sock)
    assert s.send(b"HELLOWORLD") is False      # 4 바이트만 나갔다
    assert s.backlogged is True
    assert s.dropped == 0                      # 버린 게 아니라 밀린 것이다


def test_flush_drains_the_tail_in_order():
    sock = ChokedSock(limit=4)
    s = DropSender(sock)
    s.send(b"HELLOWORLD")
    for _ in range(5):
        s.flush()
    assert sock.wire == b"HELLOWORLD"          # 잘리지 않고 순서대로
    assert s.backlogged is False


def test_a_new_frame_is_dropped_whole_while_the_tail_is_pending():
    """버리는 자리는 항상 프레임 경계다. 중간에서 버리면 안 된다."""
    sock = ChokedSock(limit=4)
    s = DropSender(sock)
    s.send(b"AAAAAAAAAA")                      # 4 나가고 6 밀림
    assert s.send(b"BBBBBBBBBB") is False      # 새 프레임은 통째로 버린다
    assert s.dropped == 1
    while s.backlogged:
        s.flush()
    assert sock.wire == b"AAAAAAAAAA"          # B 가 한 바이트도 안 섞였다


def test_a_blocked_send_leaves_no_tail():
    """한 바이트도 안 나갔으면 통째로 버려도 스트림이 안 어긋난다."""
    s = DropSender(FakeSock("block"))
    assert s.send(b"hello") is False
    assert s.dropped == 1
    assert s.backlogged is False


def test_detach_discards_the_tail():
    """옛 꼬리를 들고 있으면 다음 연결의 첫 프레임 앞에 붙는다."""
    s = DropSender(ChokedSock(limit=2))
    s.send(b"HELLO")
    assert s.backlogged is True
    s.detach()
    assert s.backlogged is False


def test_flush_without_a_socket_is_false():
    assert DropSender().flush() is False


def test_flush_with_nothing_pending_is_true():
    assert DropSender(FakeSock()).flush() is True


def test_no_socket_is_not_an_error():
    s = DropSender()
    assert s.send(b"hello") is False
    assert s.connected is False


def test_detach_closes_and_forgets():
    sock = FakeSock()
    s = DropSender(sock)
    s.detach()
    assert s.connected is False
    assert s.send(b"x") is False


def test_recv_returns_what_arrived():
    s = DropSender(FakeSock(incoming=b'{"cmd":"ping"}\n'))
    assert s.recv() == b'{"cmd":"ping"}\n'


def test_recv_returns_empty_when_nothing_is_ready():
    """읽을 게 없는 것과 끊긴 것은 다르다."""
    s = DropSender(FakeSock("block"))
    assert s.recv() == b""
    assert s.connected is True


def test_recv_returns_none_and_detaches_on_close():
    """상대가 닫으면 recv 가 b'' 를 준다. 그건 끊김이다."""
    s = DropSender(FakeSock(incoming=b""))
    assert s.recv() is None
    assert s.connected is False


def test_recv_without_a_socket_is_none():
    assert DropSender().recv() is None
