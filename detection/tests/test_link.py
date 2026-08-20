# -*- coding: utf-8 -*-
"""link.py 프레이밍. 소켓도 네트워크도 필요 없다."""

import struct

import pytest

from link import LineReader, LinkError, MAX_LINE, encode_msg


def test_round_trip():
    msg = {"cmd": "set_roi", "x": 220, "y": 150, "w": 150, "h": 150}
    reader = LineReader()
    assert reader.feed(encode_msg(msg)) == [msg]


def test_two_messages_in_one_chunk():
    a, b = {"cmd": "ping", "seq": 1}, {"cmd": "ping", "seq": 2}
    reader = LineReader()
    assert reader.feed(encode_msg(a) + encode_msg(b)) == [a, b]


def test_message_split_across_chunks():
    """TCP 는 메시지 경계를 안 지킨다. 여기서 실제 버그가 난다."""
    msg = {"cmd": "capture_target"}
    blob = encode_msg(msg)
    reader = LineReader()
    out = []
    for i in range(len(blob)):          # 한 바이트씩 먹인다
        out += reader.feed(blob[i:i + 1])
    assert out == [msg]


def test_partial_message_yields_nothing_yet():
    reader = LineReader()
    assert reader.feed(b'{"cmd": "pi') == []


def test_korean_survives_the_round_trip():
    msg = {"t": "ack", "ok": False, "msg": "각도를 못 재고 있습니다"}
    reader = LineReader()
    assert reader.feed(encode_msg(msg)) == [msg]


def test_broken_json_raises():
    reader = LineReader()
    with pytest.raises(LinkError):
        reader.feed(b"not json at all\n")


def test_oversized_line_raises():
    """줄바꿈 없이 무한정 쌓이면 메모리가 터진다. 상대가 고장났을 때다."""
    reader = LineReader()
    with pytest.raises(LinkError):
        reader.feed(b"x" * (MAX_LINE + 1))


from link import MAX_FRAME, PreviewReader, encode_preview


def make_frame(seq=1):
    return encode_preview(seq=seq, src_w=640, src_h=480,
                          jpeg=b"\xff\xd8fakejpeg", mask_png=b"\x89PNGfake",
                          roi=[220, 150, 150, 150])


def test_preview_round_trip():
    [f] = PreviewReader().feed(make_frame(seq=7))
    assert f.seq == 7
    assert (f.src_w, f.src_h) == (640, 480)
    assert f.roi == [220, 150, 150, 150]
    assert f.jpeg == b"\xff\xd8fakejpeg"
    assert f.mask_png == b"\x89PNGfake"


def test_preview_split_across_chunks():
    """JPEG 는 크니까 반드시 쪼개져서 온다."""
    blob = make_frame(seq=3)
    reader = PreviewReader()
    out = []
    for i in range(len(blob)):
        out += reader.feed(blob[i:i + 1])
    assert len(out) == 1 and out[0].seq == 3


def test_two_preview_frames_in_one_chunk():
    out = PreviewReader().feed(make_frame(1) + make_frame(2))
    assert [f.seq for f in out] == [1, 2]


def test_preview_without_a_mask():
    """ROI 가 아직 없으면 마스크도 없다."""
    blob = encode_preview(seq=1, src_w=640, src_h=480,
                          jpeg=b"\xff\xd8x", mask_png=b"", roi=None)
    [f] = PreviewReader().feed(blob)
    assert f.mask_png == b"" and f.roi is None


def test_oversized_preview_frame_raises():
    reader = PreviewReader()
    with pytest.raises(LinkError):
        reader.feed((MAX_FRAME + 1).to_bytes(4, "big"))


def test_preview_header_missing_a_field_raises_link_error():
    """헤더가 유효한 JSON 이어도 필드가 빠지면 LinkError 여야 한다.

    KeyError 가 그대로 새어 나가면 콘솔이 죽는다. 프로토콜이 깨졌을 때는
    한 종류의 예외로 모여야 호출부가 "연결을 끊는다" 하나로 대응한다.
    """
    body = encode_msg({"seq": 1, "jpeg_len": 0, "mask_len": 0}) # src_w/src_h 없음
    blob = struct.pack(">I", len(body)) + body
    with pytest.raises(LinkError):
        PreviewReader().feed(blob)


def test_preview_header_with_a_non_numeric_field_raises_link_error():
    body = encode_msg({"seq": "일", "src_w": 640, "src_h": 480,
                       "jpeg_len": 0, "mask_len": 0})
    blob = struct.pack(">I", len(body)) + body
    with pytest.raises(LinkError):
        PreviewReader().feed(blob)
