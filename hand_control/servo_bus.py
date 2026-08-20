# -*- coding: utf-8 -*-
"""COM11 을 한 번에 한 컨트롤러만 열게 하는 배타 리스.

--- 왜 필요한가 ---
URT-1 의 서보 커넥터는 병렬로 묶인 한 가닥이라, 손(SCS0009 x10)과
손목(STS3215)이 같은 COM11 을 쓴다. 그런데 같은 포트를 두 컨트롤러가
동시에 열 수 없다 -- Scs0009PyController 로 COM11 을 연 상태에서
Sts3215PyController 로 또 열면 OSError("액세스가 거부되었습니다") 가 난다.

--- 왜 gc.collect() 인가 ---
파이썬 객체를 놓기만 해서는 Rust 쪽 드롭이 안 돌아 포트가 안 풀린다.

--- 반납은 토크를 끄지 않는다 ---
토크는 서보 안의 레지스터다. 포트를 닫아도 서보는 마지막 goal position 을
계속 유지한다. 그래서 손을 편 자세로 세워 둔 채 포트만 놓을 수 있다.
토크까지 끄는 것은 종료 경로(Hand.release)의 일이다.
"""

import gc


class ServoLease:
    def __init__(self):
        self._owner = None
        self._resource = None
        self._close = None

    def owner(self):
        """지금 포트를 쥔 이름. 아무도 안 쥐고 있으면 None."""
        return self._owner

    def acquire(self, name, open_fn, close_fn):
        """포트를 name 에게 넘긴다. -> open_fn() 이 만든 자원.

        이미 name 이 쥐고 있으면 아무것도 하지 않고 기존 자원을 돌려준다.
        매 프레임 불려도 되게 하려는 것이다 -- 다시 열면 재연결 비용을
        프레임마다 낸다.
        """
        if self._owner == name:
            return self._resource
        # 반드시 먼저 놓는다. 새 것을 먼저 열면 그 시점에 옛 것이 아직
        # 포트를 쥐고 있어서 OSError 가 난다.
        self.release()
        # 열다 실패하면 주인을 기록하지 않는다. 기록하면 다음 acquire 가
        # "이미 갖고 있다"며 없는 자원을 돌려준다.
        resource = open_fn()
        self._resource = resource
        self._close = close_fn
        self._owner = name
        return resource

    def release(self):
        """포트를 놓는다. 아무도 안 쥐고 있으면 아무것도 안 한다.

        close 가 터져도 리스는 반드시 비운다. 안 비우면 그 뒤로 아무도
        포트를 못 잡아 비상 폄까지 막힌다.
        """
        if self._owner is None:
            return
        close, resource = self._close, self._resource
        self._owner = None
        self._resource = None
        self._close = None
        try:
            close(resource)
        finally:
            del resource
            gc.collect()
