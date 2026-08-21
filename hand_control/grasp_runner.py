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
import spread_seek
import stiffness
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
        # 손 전체의 대표 강성과 분류. 물체는 하나이므로 손가락마다 따로
        # 갖지 않는다. k_hand 는 '지금까지 본 것 중 최대'라서 단조
        # 증가하고, 그 결과 분류는 soft -> rigid 승격만 일어난다.
        self.k_hand = None
        self.object_class = None
        # 벌림 탐색. seek_done 은 파지당 한 번만 훑기 위한 것이다 --
        # 매번 다시 훑으면 손이 영원히 좌우로 떨린다.
        self._seeker = None
        self.seek_done = False
        self.spread = hand_config.GRASP_SPREAD
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
        self.k_hand = None
        self.object_class = None
        self._seeker = None
        self.seek_done = False
        self.spread = hand_config.GRASP_SPREAD
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

    def _classify_hand(self):
        """손가락들의 k_hat 을 모아 손 하나의 분류를 정하고 되돌려 준다.

        --- 왜 손가락이 아니라 손 단위인가 ---
        물체는 하나다. 손가락마다 rigid/soft 가 갈리면 같은 물체를
        손가락마다 다른 힘으로 잡게 된다. 실측(2026-08-18/19)에서 같은
        물체를 같이 잡은 손가락 사이 k_hat 편차가 2.3배, 한 런은
        147배였다 -- 그 편차는 물체가 아니라 접촉 상태를 잰 것이다.

        집계가 최대인 이유와 confident=False 를 빼는 이유는
        stiffness.hand_k 주석에 있다.

        --- 승격 전용 ---
        k_hand 를 누적 최대로 들고 있으므로 분류는 soft -> rigid 로만
        간다. 강등이 없어야 하는 이유: HOLD 에서 접촉이 끊긴 손가락은
        APPROACH 로 돌아가 다시 프로빙하며 더 낮은 k_hat 을 낼 수
        있는데, 그때 목표힘이 내려가면 잡고 있던 물체를 놓는다.
        """
        k = stiffness.hand_k(
            (getattr(s, "k_hat", None), getattr(s, "confident", None))
            for s in self.states.values())
        if k is not None and (self.k_hand is None or k > self.k_hand):
            self.k_hand = k
        if self.k_hand is None:
            return                      # 아직 아무도 못 쟀다

        self.object_class = stiffness.classify(self.k_hand,
                                               self.params.k_threshold)
        f_target = (self.params.f_target_rigid
                    if self.object_class == "rigid"
                    else self.params.f_target_soft)
        for state in self.states.values():
            # 이미 같은 판정이면 건드리지 않는다. 매 사이클 덮어쓰면
            # HOLD 의 stall 적응이 손가락별로 낮춰 둔 목표를 계속
            # 되돌려서, 도달 불가능한 목표를 영영 쫓게 된다.
            if getattr(state, "object_class", None) != self.object_class:
                state.set_object(self.object_class, f_target)

    @property
    def seeking(self):
        """지금 벌림을 훑는 중인가."""
        return self._seeker is not None and not self._seeker.done

    def _contacting(self):
        """지금 물체를 쥐고 있는 손가락 이름들.

        힘이 아니라 **상태**로 센다. HOLD 는 접촉을 touch_confirm_cycles
        만큼 확인하고 들어간 자리이고, 순간적으로 힘이 꺼져도
        contact_lost_cycles 동안은 버틴다. 힘으로 세면 그 노이즈가
        그대로 들어와서, 안정 판정(HOLD/NO_CONTACT)과 접촉 판정이
        같은 사이클에 서로 다른 말을 한다.

        훑는 중의 점수는 반대로 힘으로 낸다(SpreadSeeker). 거기서는
        "방금 옮긴 자리에서 닿았나"를 빨리 알아야 하는데 상태가 따라
        오려면 몇 사이클이 걸린다.
        """
        return [n for n, s in self.states.items() if s.state == grasp.HOLD]

    def _seek_spread(self, forces):
        """벌림 탐색을 진행하고 이번에 명령할 벌림을 돌려준다.

        --- 언제 시작하나 ---
        손가락들이 자리를 잡았는데(HOLD/NO_CONTACT 로 안정) 접촉한
        손가락이 SEEK_MIN_CONTACT 미만일 때다. 허공을 쥔 채로 끝내는
        대신 옆을 훑어본다.

        --- 왜 이게 슬립 대책인가 ---
        2026-08-21 감사에서 슬립의 성격이 드러났다. HOLD 중 a 도 안 열리고
        수직력도 안 주는데 물체가 빠진다 -- 마찰 부족이고, 62% 가 이미
        a_max 포화라 힘으로는 못 올린다. 남은 레버가 접촉점 수다.

        --- 왜 파지당 한 번인가 ---
        끝난 뒤에도 계속 훑으면 손이 영원히 좌우로 떨린다.
        """
        if self._seeker is not None:
            self._seeker.update(forces, self.params.f_touch)
            if self._seeker.done:
                # 최적점에 고정한다. 못 찾았으면 best_u 가 0 이라
                # 제자리로 돌아간다 -- 헛되이 벌린 채로 두면 다음
                # 파지가 그 자세에서 시작한다.
                self.spread = self._seeker.best_spread()
                if self._seeker.aborted:
                    print("\n[WARN] 벌림 탐색 중 힘 상한을 넘어 멈췄습니다.")
                self._seeker = None
                self.seek_done = True
                # 찾은 자리에서 실제로 물릴 시간을 준다. NO_CONTACT 는
                # 힘이 touch_confirm_cycles 연속으로 잡혀야 되살아나는데,
                # 여기서 바로 settled 로 넘기면 그 전에 상태기계가
                # HOLDING 으로 가버려서 탐색이 아무것도 안 바꾼 셈이 된다.
                self._stable = 0
            else:
                self.spread = self._seeker.spread
            return

        contacting = self._contacting()
        if len(contacting) >= hand_config.SEEK_MIN_CONTACT:
            self.seek_done = True       # 훑을 이유가 없다
            return
        seeking = [f.name for f in self.fingers if f.name not in contacting
                   and f.spread_weight != 0.0]
        if not seeking:
            # 엄지는 spread_weight 가 0 이라 s 로는 안 움직인다. 훑을
            # 손가락이 하나도 없으면 탐색이 의미가 없다.
            self.seek_done = True
            return
        self._seeker = spread_seek.SpreadSeeker(
            seeking, contacting, f_abort=self.params.f_abort)
        self.spread = self._seeker.spread

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
        # 전단력은 로그에만 쓴다. 슬립이 수직력에 안 보인다는 걸
        # 2026-08-21 감사로 확인했는데, 임계값을 정할 실측이 한 줄도
        # 없어서 먼저 쌓는다. read_shear 가 없는 구현이 남아 있어도
        # 파지는 계속돼야 하므로 없으면 조용히 건너뛴다 -- 로깅이
        # 제어를 막으면 안 된다.
        read_shear = getattr(self.sensors, "read_shear", None)
        shears = read_shear() if read_shear is not None else {}
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
                          flexes.get(finger.name),
                          shear=shears.get(finger.name))
            if state.state != self._prev[finger.name]:
                if state.state == grasp.FROZEN:
                    print(f"\n[WARN] {finger.name} 이(가) FROZEN 상태로 "
                          f"전환됐습니다 -- 센서 또는 관절 읽기를 "
                          f"놓쳤습니다.")
                elif self._prev[finger.name] == grasp.FROZEN:
                    print(f"\n[INFO] {finger.name} 이(가) FROZEN 에서 "
                          f"복구됐습니다.")
                self._prev[finger.name] = state.state

        self._classify_hand()

        self.hand.set_pose_map(targets, s=self.spread)

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

        # 자리를 잡았는데 접촉이 모자라면 옆을 훑어본다. 훑는 동안은
        # settled 로 넘어가면 안 된다 -- 상태기계가 HOLDING 으로 가면
        # 손이 움직이는 중에 '다 됐다'고 알리는 셈이다.
        if (hand_config.SEEK_ENABLED and not self.seek_done
                and self._stable >= hand_config.GRASP_STABLE_CYCLES):
            self._seek_spread(forces)
        if self.seeking:
            self.status = "busy"
            return self.status

        self.status = ("settled"
                       if self._stable >= hand_config.GRASP_STABLE_CYCLES
                       else "busy")
        return self.status
