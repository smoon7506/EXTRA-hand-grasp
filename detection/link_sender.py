# -*- coding: utf-8 -*-
"""막히면 버리는 송신기.

--- 왜 필요한가 ---
WiFi 가 한 번 딸꾹질할 때 소켓 send 가 블록되면 30Hz 판정 루프가 같이
멈춘다. 그러면 손목이 정렬 도중에 굳거나 파지가 늦는다. 화면 한 장을
보내려고 손을 멈출 수는 없다.

--- 버리는 자리는 프레임 경계여야 한다 ---
논블로킹 소켓의 send() 는 커널 버퍼에 들어가는 만큼만 받고 그 수를
돌려준다. 5.9KB 짜리 미리보기 프레임은 한 번에 다 안 들어가는 일이
흔하다.

**이미 나간 바이트는 되돌릴 수 없다.** 그래서 "절반 보내고 나머지는
버린다"는 성립하지 않는다 -- 받는 쪽은 잘린 프레임을 받고, 다음
프레임의 길이 필드를 JPEG 데이터 한가운데에서 읽는다. 그때부터
스트림이 영영 어긋난다 (실제로 "미리보기 프레임이 3537097941 바이트다"
로 터졌다).

그래서 규칙은 이렇다:
  - 한 바이트도 못 나갔으면(BlockingIOError) 통째로 버린다. 안전하다
  - 일부라도 나갔으면 나머지를 _pending 에 들고 있다가 마저 보낸다
  - _pending 이 남아 있는 동안 들어온 새 프레임은 버린다

즉 버리는 것은 항상 '아직 시작 안 한 프레임'이다. 프레임 중간에서
버리는 일이 없으므로 수신 측 경계가 유지된다. 제어 루프는 여전히
블록되지 않는다 -- 매 프레임 한 번씩만 밀어 넣기 때문이다.
"""


class DropSender:
    def __init__(self, sock=None):
        self._sock = sock
        self._pending = b""      # 나가다 만 프레임의 꼬리
        self.dropped = 0

    @property
    def connected(self):
        return self._sock is not None

    @property
    def backlogged(self):
        """아직 안 나간 꼬리가 있나. 진단·표시용."""
        return bool(self._pending)

    def attach(self, sock):
        self.detach()
        self._sock = sock

    def detach(self):
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
        self._sock = None
        # 소켓이 바뀌면 옛 꼬리는 의미가 없다. 들고 있으면 다음 연결의
        # 첫 프레임 앞에 붙어서 스트림을 처음부터 어긋나게 만든다.
        self._pending = b""

    def flush(self):
        """밀린 꼬리를 더 내보낸다. 남은 게 없으면 True.

        제어 루프가 매 프레임 한 번 부른다. 안 부르면 꼬리가 안 나가고
        그동안 새 프레임이 전부 버려진다.
        """
        if self._sock is None:
            return False
        if not self._pending:
            return True
        try:
            n = self._sock.send(self._pending)
        except (BlockingIOError, InterruptedError):
            return False
        except OSError:
            self.detach()
            return False
        self._pending = self._pending[n:]
        return not self._pending

    def send(self, blob):
        """다 보냈으면 True. 버렸거나 꼬리가 남았으면 False."""
        if self._sock is None:
            return False
        if self._pending:
            # 앞 프레임이 아직 다 안 나갔다. 이번 것은 시작도 안 했으니
            # 통째로 버린다 -- 프레임 경계에서 버리는 것이라 안전하다.
            self.dropped += 1
            self.flush()
            return False
        try:
            n = self._sock.send(blob)
        except (BlockingIOError, InterruptedError):
            # 한 바이트도 안 나갔다. 버려도 스트림이 안 어긋난다.
            self.dropped += 1
            return False
        except OSError:
            # ConnectionReset / BrokenPipe 등. 상대가 사라졌다.
            self.detach()
            return False
        if n < len(blob):
            # 앞부분이 이미 나갔다. 되돌릴 수 없으므로 반드시 마저 보낸다.
            self._pending = blob[n:]
            return False
        return True

    def recv(self, n=65536):
        """b"" = 지금은 없다. None = 끊겼다. 그 외 = 데이터.

        이 셋을 섞으면 안 된다. 콘솔이 잠깐 조용한 것과 사라진 것은
        전혀 다른 사건인데, 둘을 같이 다루면 딸꾹질마다 연결이 끊긴다.
        """
        if self._sock is None:
            return None
        try:
            chunk = self._sock.recv(n)
        except (BlockingIOError, InterruptedError):
            return b""
        except OSError:
            self.detach()
            return None
        if chunk == b"":
            # 상대가 정상적으로 닫았다.
            self.detach()
            return None
        return chunk
