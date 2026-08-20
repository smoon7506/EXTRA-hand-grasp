# -*- coding: utf-8 -*-
"""PC(콘솔) <-> 파이(데몬) 와이어 프로토콜. 양쪽이 이 파일을 그대로 쓴다.

--- 왜 프레이밍이 따로 필요한가 ---
TCP 는 바이트 스트림이라 메시지 경계를 안 지킨다. send 한 번이 recv 한
번으로 오지 않는다. 그래서 받는 쪽이 경계를 스스로 찾아야 한다.

--- 왜 소켓을 안 다루나 ---
이 파일은 bytes 만 다룬다. 그래야 소켓 없이 전부 테스트된다.
"""

import json
import struct
from dataclasses import dataclass

PROTO = 1

# 한 줄의 상한(바이트). 넘으면 상대가 고장난 것으로 본다 -- 줄바꿈 없이
# 계속 보내면 버퍼가 무한정 자란다.
MAX_LINE = 1 << 20


class LinkError(Exception):
    """프로토콜이 깨졌다. 연결을 끊어야 하는 상황이다."""


def encode_msg(obj):
    """dict -> 한 줄 bytes.

    ensure_ascii=False 로 한글을 그대로 싣는다. 거절 사유가 한글이라
    이스케이프되면 로그에서 못 읽는다.
    """
    line = json.dumps(obj, ensure_ascii=False, separators=(",", ":"))
    return line.encode("utf-8") + b"\n"


class LineReader:
    """바이트 조각 -> 완성된 메시지 목록.

    완성되지 않은 꼬리는 다음 feed 까지 들고 있는다.
    """

    def __init__(self):
        self._buf = bytearray()

    def feed(self, chunk):
        self._buf += chunk
        out = []
        while True:
            i = self._buf.find(b"\n")
            if i < 0:
                if len(self._buf) > MAX_LINE:
                    raise LinkError(
                        f"줄바꿈 없이 {len(self._buf)} 바이트가 쌓였다 "
                        f"(상한 {MAX_LINE}). 상대가 고장났다.")
                return out
            line = bytes(self._buf[:i])
            del self._buf[:i + 1]
            if not line.strip():
                continue                # 빈 줄은 무시한다
            try:
                out.append(json.loads(line.decode("utf-8")))
            except (UnicodeDecodeError, json.JSONDecodeError) as e:
                raise LinkError(f"메시지를 못 읽었다: {e}") from e


# 미리보기 프레임 한 장의 상한(바이트). 640x480 JPEG q70 이 30KB 남짓이라
# 4MB 면 한참 여유다. 넘으면 스트림이 어긋난 것이다.
MAX_FRAME = 4 << 20


@dataclass
class PreviewFrame:
    """화면에 그릴 한 장. 깊이 배열은 여기 없다 -- 보내지 않는다.

    seq 가 텔레메트리의 seq 와 같다. 콘솔은 이걸로 그림과 숫자의 짝을
    맞춘다. 짝을 안 맞추면 화면의 각도 숫자와 그림이 서로 다른 프레임이
    된다.
    """

    seq: int
    src_w: int
    src_h: int
    roi: list
    jpeg: bytes
    mask_png: bytes


def encode_preview(seq, src_w, src_h, jpeg, mask_png, roi):
    """한 장 -> bytes.

    [4B 전체길이 BE][헤더 JSON + \\n][JPEG][마스크 PNG]

    src_w/src_h 는 **원본** 해상도다. 미리보기를 축소해 보낼 때 콘솔이
    드래그 좌표를 원본으로 되돌려야 하는데, 이게 빠지면 ROI 가 조용히
    엉뚱한 곳에 잡힌다.
    """
    header = encode_msg({
        "seq": int(seq), "src_w": int(src_w), "src_h": int(src_h),
        "jpeg_len": len(jpeg), "mask_len": len(mask_png),
        "roi": list(roi) if roi is not None else None,
    })
    body = header + bytes(jpeg) + bytes(mask_png)
    return struct.pack(">I", len(body)) + body


class PreviewReader:
    """바이트 조각 -> 완성된 PreviewFrame 목록."""

    def __init__(self):
        self._buf = bytearray()

    def feed(self, chunk):
        self._buf += chunk
        out = []
        while True:
            if len(self._buf) < 4:
                return out
            (size,) = struct.unpack(">I", bytes(self._buf[:4]))
            if size > MAX_FRAME:
                raise LinkError(
                    f"미리보기 프레임이 {size} 바이트다 (상한 {MAX_FRAME}). "
                    f"스트림이 어긋났다.")
            if len(self._buf) < 4 + size:
                return out
            body = bytes(self._buf[4:4 + size])
            del self._buf[:4 + size]
            out.append(self._decode(body))

    @staticmethod
    def _decode(body):
        i = body.find(b"\n")
        if i < 0:
            raise LinkError("미리보기 헤더에 줄바꿈이 없다.")
        try:
            head = json.loads(body[:i].decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as e:
            raise LinkError(f"미리보기 헤더를 못 읽었다: {e}") from e
        rest = body[i + 1:]
        # 헤더가 문법적으로 유효한 JSON 이어도 필드가 빠졌거나 타입이
        # 안 맞을 수 있다. 그대로 새어 나가면 KeyError/TypeError/ValueError
        # 가 콘솔까지 올라가 죽는다. 프로토콜이 깨진 상황이니 LinkError
        # 하나로 모아 호출부가 "연결을 끊는다" 하나로 대응하게 한다.
        try:
            seq = int(head["seq"])
            src_w = int(head["src_w"])
            src_h = int(head["src_h"])
            jl = int(head["jpeg_len"])
            ml = int(head["mask_len"])
            roi = head["roi"]
        except (KeyError, TypeError, ValueError) as e:
            raise LinkError(f"미리보기 헤더 필드가 이상하다: {e}") from e
        if len(rest) != jl + ml:
            raise LinkError(
                f"미리보기 본문 길이가 안 맞는다 "
                f"(헤더 {jl}+{ml}, 실제 {len(rest)}).")
        return PreviewFrame(seq=seq, src_w=src_w, src_h=src_h, roi=roi,
                            jpeg=rest[:jl], mask_png=rest[jl:])
