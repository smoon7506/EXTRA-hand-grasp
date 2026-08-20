# -*- coding: utf-8 -*-
"""PC 가 사라졌을 때 파이가 하는 일.

--- 왜 TCP 만으로 부족한가 ---
WiFi 가 조용히 죽으면 TCP 는 몇 분간 눈치를 못 챈다. 그래서 애플리케이션
레벨 하트비트를 쓴다. 콘솔이 매 프레임 ping 을 보내고, 여기서 마지막
수신 시각을 본다.

--- 원칙 ---
사람이 안 보고 있으면 새로 시작하지 않는다. 하지만 진행 중인 동작을
중간에 얼리지도 않는다.
"""

import time

from grasp_state import ALIGNING, CONFIRMING, HOLDING

# 이 시간 동안 아무것도 안 오면 끊긴 것으로 본다(초).
# 30Hz 기준 60번 연속 유실이라 오탐 여지가 거의 없다.
LINK_TIMEOUT_S = 2.0

# 끊긴 채 물체를 쥐고 있을 때, 이만큼 지나면 놓는다(초).
# WiFi 재연결은 DHCP 까지 포함해 보통 5~15초에 끝난다. 30초면 일시적
# 딸꾹질은 전부 살아남고 진짜 끊긴 경우만 놓는다.
HOLD_RELEASE_S = 30.0


class LinkWatchdog:
    def __init__(self, machine, timeout_s=LINK_TIMEOUT_S,
                 hold_release_s=HOLD_RELEASE_S, clock=time.perf_counter):
        self.machine = machine
        self.timeout_s = timeout_s
        self.hold_release_s = hold_release_s
        self._clock = clock
        self._last_seen = clock()
        self._dropped = False
        self._was_up = True
        self._release_at = None

    @property
    def connected(self):
        if self._dropped:
            return False
        return (self._clock() - self._last_seen) < self.timeout_s

    def feed(self):
        """콘솔에서 뭔가 왔다. ping 이든 명령이든 상관없다."""
        self._last_seen = self._clock()
        self._dropped = False

    def drop(self):
        """bye 를 받았거나 소켓이 닫혔다. 2초를 기다릴 이유가 없다."""
        self._dropped = True

    def release_in(self):
        """자동 폄까지 남은 초. 타이머가 없으면 None."""
        if self._release_at is None:
            return None
        return max(0.0, self._release_at - self._clock())

    def tick(self):
        """매 프레임 한 번. 상태기계보다 나중에 부른다.

        HOLDING 판정을 이 안에서 매번 다시 하는 이유: 끊긴 시점에
        GRASPING 이었다면 그건 끝까지 가고, 도착해서 HOLDING 이 된 뒤에야
        타이머가 서야 한다. 끊기는 순간에만 보면 그 경우를 놓친다.
        """
        up = self.connected
        if up:
            if not self._was_up:
                # 복구. 사람이 돌아왔으니 자동 폄을 취소한다.
                self._release_at = None
            self._was_up = True
            return

        if self._was_up:
            self._was_up = False
            self._on_lost()

        if self.machine.state == HOLDING and self._release_at is None:
            self._release_at = self._clock() + self.hold_release_s
        elif self.machine.state != HOLDING and self._release_at is not None:
            # 그 사이에 다른 경로로 풀렸다(과열 abort 등).
            self._release_at = None

        if self._release_at is not None and self._clock() >= self._release_at:
            self._release_at = None
            self.machine.emergency_open()

    def _on_lost(self):
        """끊긴 첫 프레임에 한 번."""
        self.machine.disarm()
        if self.machine.state in (ALIGNING, CONFIRMING):
            # 손목이 도는 중이다. 감시자가 없으면 멈춘다.
            self.machine.abort_to_armed()
