# -*- coding: utf-8 -*-
"""촉각 힘 제어 루프를 논블로킹으로 돌리는 러너.

grasp_main.run_live() 의 알맹이를 여기로 옮겼다. 그쪽은 블로킹
`while True` 라 카메라 루프 안에 못 들어간다.

--- tick() 의 규약 ---
sequence.PoseSequencer.tick() 과 같다. "지금 나갈 것만 내보내고 곧바로
돌아온다". 기다리는 쪽이 자기 사정에 맞게 sleep 한다.

--- 레이트리밋을 러너가 갖는 이유 ---
카메라는 30fps 인데 제어 루프는 LOOP_HZ(10Hz)다. 호출부가 "10Hz 로만
불러라"를 지키는 구조로 두면, 카메라가 느려진 날 조용히 어긋난다.
"""

import time

import grasp
import hand_config
import sequence
from grasp import FingerGrasp, GraspParams
from grasp_log import GraspLogger


def flex_span(finger):
    """이 손가락의 a 1.0 당 flex 라디안."""
    return finger.flex_weight * hand_config.FLEX_LIMIT_RAD


class ForceGraspRunner:
    def __init__(self, fingers, sensors, hand, *, params=None,
                 clock=time.monotonic, log_factory=GraspLogger):
        self.fingers = list(fingers)
        self.sensors = sensors
        self.hand = hand
        self.params = params or GraspParams.from_config()
        self._clock = clock
        self._log_factory = log_factory

        self.states = {}
        self.status = "busy"
        self.log_path = None
        self._log = None
        self._seq = None             # start_open() 에서 만든다
        self._mode = None            # "grasp" | "open" | None
        self._dt = 1.0 / hand_config.LOOP_HZ
        self._t0 = 0.0
        self._last = 0.0
        self._next_temp = 0.0
        self._temp_fail = 0
        self._stable = 0
        self._delays = {}
        self._prev = {}

    # --- 시작 -----------------------------------------------------------

    def start_grasp(self):
        """새 파지를 시작한다. 상태와 로그를 새로 연다."""
        self.close()
        self.states = {f.name: FingerGrasp(f.name, flex_span(f), self.params)
                       for f in self.fingers}
        self._prev = {f.name: self.states[f.name].state for f in self.fingers}
        # 손가락별 출발 지연. grasp.py 는 순수 로직이라 스케줄링을
        # 모르므로 여기서 dt=0.0 을 먹여 APPROACH 를 제자리에 세워 둔다.
        # update() 자체는 매 사이클 계속 부르므로 결측(FROZEN)이나
        # 과다힘(BACKOFF) 감지는 지연 중에도 그대로 살아 있다.
        self._delays = {f.name: hand_config.start_delay(f.name, closing=True)
                        for f in self.fingers}
        now = self._clock()
        self._t0 = now
        self._last = now
        self._next_temp = now + hand_config.TEMP_CHECK_INTERVAL_S
        self._temp_fail = 0
        self._stable = 0
        self._log = self._log_factory()
        self._log.__enter__()
        self.log_path = getattr(self._log, "path", None)
        self._mode = "grasp"
        self.status = "busy"

    def start_open(self):
        """편 자세로 되돌리기 시작한다.

        시퀀서를 여기서 새로 만드는 이유가 두 가지다:

        1) clock 을 러너의 것과 같게 맞춘다. 시퀀서가 자기 시계를 쓰면
           러너는 레이트리밋 걸린 10Hz 로 도는데 시퀀서는 실시간으로
           지연을 재서, 카메라 루프가 느린 날 순서가 어긋난다.
        2) last_a=1.0 으로 "닫힌 손에서 출발한다"를 알려 준다. 시퀀서는
           a 가 줄어드는지로 폄/굽힘을 판정하는데(closing = a >= last_a),
           기본값 0.0 인 채로 start(0.0) 을 부르면 굽힘으로 오판해서
           CLOSE_DELAY_S 를 쓴다 -- 그러면 덮고 있던 엄지가 마지막에
           나와 검지와 부딪힌다. 러너는 파지 뒤에만 펴므로 손이 닫혀
           있다는 가정이 항상 맞다.
        """
        self._seq = sequence.PoseSequencer(
            self.fingers, [self.hand.set_pose],
            clock=self._clock, last_a=1.0)
        self._seq.start(0.0, 0.0)
        self._last = self._clock()
        self._mode = "open"
        self.status = "busy"

    def close(self):
        """로거를 닫는다. 여러 번 불려도 된다."""
        if self._log is not None:
            self._log.__exit__(None, None, None)
            self._log = None

    # --- 진행 -----------------------------------------------------------

    def tick(self):
        """한 사이클. -> "busy" | "settled" | "opened" | "abort"."""
        if self._mode is None:
            return self.status
        now = self._clock()
        if now - self._last < self._dt:
            return self.status          # 아직 이번 사이클이 아니다
        elapsed = now - self._last
        self._last = now

        if self._mode == "open":
            busy = self._seq.tick()
            self.status = "busy" if busy else "opened"
            return self.status

        forces = self.sensors.read_forces()
        flexes = self.hand.read_flex()
        t = now - self._t0

        targets = {}
        for finger in self.fingers:
            state = self.states[finger.name]
            finger_dt = 0.0 if t < self._delays[finger.name] else elapsed
            targets[finger.name] = state.update(
                forces.get(finger.name), flexes.get(finger.name),
                now, finger_dt)
            self._log.row(t, state, forces.get(finger.name),
                          flexes.get(finger.name))
            if state.state != self._prev[finger.name]:
                if state.state == grasp.FROZEN:
                    print(f"\n[WARN] {finger.name} 이(가) FROZEN 상태로 "
                          f"전환됐습니다 -- 센서 또는 관절 읽기를 "
                          f"놓쳤습니다.")
                elif self._prev[finger.name] == grasp.FROZEN:
                    print(f"\n[INFO] {finger.name} 이(가) FROZEN 에서 "
                          f"복구됐습니다.")
                self._prev[finger.name] = state.state

        self.hand.set_pose_map(targets, s=hand_config.GRASP_SPREAD)

        # 온도는 1초에 한 번. 파지를 유지하면 서보가 계속 토크를 쓰므로
        # 실제로 발생한다. 이건 마지막 안전장치라 반드시 따라와야 한다.
        if now >= self._next_temp:
            self._next_temp = now + hand_config.TEMP_CHECK_INTERVAL_S
            temps = self.hand.read_temperatures()
            if temps:
                self._temp_fail = 0
                hottest = max(temps)
                if hottest > hand_config.TEMP_LIMIT_C:
                    print(f"\n[ERROR] 모터 온도 {hottest:.0f}도 — 한계 "
                          f"{hand_config.TEMP_LIMIT_C}도 초과. 손을 풉니다.")
                    self.status = "abort"
                    return self.status
            else:
                self._temp_fail += 1
                print(f"[WARN] 온도를 한 개도 못 읽었습니다 "
                      f"({self._temp_fail}/{hand_config.TEMP_FAIL_LIMIT})")
                if self._temp_fail >= hand_config.TEMP_FAIL_LIMIT:
                    print(f"\n[ERROR] 온도 읽기가 {self._temp_fail}회 연속 "
                          f"실패했습니다 -- 더 이상 과열을 감시할 수 없어 "
                          f"파지를 중단합니다.")
                    self.status = "abort"
                    return self.status

        # 자세가 잡혔나. FingerGrasp 에는 종착 상태가 없으므로 밖에서
        # 정한다 -- HOLD 는 목표 힘을 계속 추종하는 정상 상태다.
        if all(s.state in (grasp.HOLD, grasp.NO_CONTACT)
               for s in self.states.values()):
            self._stable += 1
        else:
            self._stable = 0
        self.status = ("settled"
                       if self._stable >= hand_config.GRASP_STABLE_CYCLES
                       else "busy")
        return self.status
