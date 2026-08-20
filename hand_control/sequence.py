# -*- coding: utf-8 -*-
"""손가락별 출발 지연을 두고 목표를 내보내는 스케줄러.

하드웨어를 모른다. kinematics.py 와 같은 성격이라 가짜 시계만으로
전부 테스트된다.

--- 왜 논블로킹인가 ---
run_sim 의 메인 루프는 물리를 계속 전진시켜야 화면이 살아 있다
(main.py:191-205). 여기서 time.sleep 으로 막으면 시뮬레이션이 얼어붙는다.
그래서 tick() 은 "지금 나갈 것만 내보내고 곧바로 돌아온다". 기다리는
쪽(run_interactive)이 자기 사정에 맞게 sleep 한다.

--- 아직 출발 안 한 손가락 ---
아무것도 보내지 않는다. 서보는 마지막 goal position 을 스스로 유지하고,
sim 도 _target 을 안 건드리면 그대로다. '붙잡아두는' 코드가 없는 게
버그가 아니라 설계다.
"""

import time

import hand_config


class PoseSequencer:
    def __init__(self, fingers, senders, clock=time.perf_counter, last_a=0.0):
        """fingers: Finger 목록. senders: f(a, s, fingers) 꼴 콜러블 목록.

        senders 를 목록으로 받는 이유: 실물만이면 [hand.set_pose],
        --sim 이면 [hand.set_pose, sim.set_target] 이다. 시퀀서는 어느
        쪽인지 알 필요가 없다.

        clock 을 주입받는 이유: 테스트에서 가짜 시계를 넣어 time.sleep
        없이 순서를 검증하기 위해서다.

        last_a: 방향 판정의 기준이 되는 '직전 a'. 기본 0.0 은 편 손이다.
        connect() 직후는 보통 편 손이므로 이 가정이 맞다.
        """
        self.fingers = list(fingers)
        self.senders = list(senders)
        self._clock = clock
        self._last_a = last_a
        self._pending = []      # [(지연초, Finger), ...] 아직 안 보낸 것
        self._a = 0.0
        self._s = 0.0
        self._t0 = 0.0

    def start(self, a, s=0.0):
        """새 목표의 출발 계획을 짠다. 이 시점에는 아무것도 보내지 않는다.

        시퀀스가 진행 중이어도 그냥 갈아엎는다. 사람이 새 값을 쳤다면
        그게 최신 의도이므로, 옛 계획을 마저 끝내는 것은 오히려 이상하다.
        """
        closing = a >= self._last_a
        self._a, self._s = a, s
        self._last_a = a
        self._t0 = self._clock()
        self._pending = [
            (hand_config.start_delay(f.name, closing), f) for f in self.fingers
        ]

    def tick(self):
        """출발할 때가 된 손가락에게만 목표를 보낸다.

        돌려주는 값은 '아직 안 보낸 손가락이 남았나'다. 모터가 목표에
        도착했는지가 아니다 -- 서보는 이 함수가 False 를 돌려준 뒤에도
        GOAL_SPEED 로 계속 움직인다. set_pose 가 즉시 반환하는 것과 같은
        규약이다.
        """
        if not self._pending:
            return False

        elapsed = self._clock() - self._t0
        due = [f for delay, f in self._pending if elapsed >= delay]
        self._pending = [
            (delay, f) for delay, f in self._pending if elapsed < delay
        ]

        if due:
            for send in self.senders:
                send(self._a, self._s, due)
        return bool(self._pending)


def format_schedule(fingers, closing):
    """출발 스케줄을 사람이 읽는 문자열로. --dry-run 에서 쓴다.

    모터를 돌리기 전에 순서를 눈으로 확인하기 위한 것이다. 실물에서
    순서를 뒤집어볼 때 지연 표를 제대로 고쳤는지 여기서 먼저 본다.
    """
    groups = {}
    for finger in fingers:
        delay = hand_config.start_delay(finger.name, closing)
        groups.setdefault(delay, []).append(finger.name)
    return "\n".join(
        f"  {delay:.2f}초  {', '.join(groups[delay])}"
        for delay in sorted(groups)
    )
