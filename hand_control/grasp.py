# -*- coding: utf-8 -*-
"""손가락 하나의 파지 상태기계. 하드웨어를 모르는 순수 로직.

손가락마다 독립된 상태와 독립된 시계를 갖는다. 물체가 동그랗지 않으면
검지는 이미 접촉해서 프로빙 중인데 새끼는 아직 허공일 수 있다. 손 전체를
하나로 묶어 "다 같이 다음 단계"로 가면 그런 물체를 못 잡는다.

--- 상태 ---
    APPROACH   힘이 잡힐 때까지 천천히 오므린다
    PROBE      조금씩 밀며 (flex, force) 샘플을 모은다
    CLASSIFY   k 를 계산하고 강체/연체를 정한다 (한 사이클만 머문다)
    HOLD       목표 힘을 추종한다 (도달 불가면 목표를 실측치로 낮춘다)
    BACKOFF    힘 상한을 넘어 후퇴하는 중
    FROZEN     센서나 위치를 못 읽어 a 를 고정한 상태
    NO_CONTACT a_max 까지 갔는데 아무것도 안 잡혔다

--- 우선순위 (위가 아래를 덮는다) ---
    1. force 나 flex 가 None      -> FROZEN
    2. force > f_abort            -> BACKOFF (상태 무관)
    3. 상태별 동작

NO_CONTACT 도 "상태 무관"의 예외가 아니다. 허공을 쓸고 지나간 뒤에도
물체가 뒤늦게 손 안으로 들어와 힘이 상한을 넘을 수 있는데, 그때도 a 를
a_max 에 고정한 채 방치하면 서보가 과부하 상태로 눌린 채 남는다. 그래서
NO_CONTACT 정지 판정은 BACKOFF 우선순위 검사 다음에 온다.

그리고 NO_CONTACT 는 종착역이 아니다. f_touch 를 연속으로 넘으면 뒤늦게
들어온 물체로 보고 힘 제어로 복귀한다 -- 근거는 update() 안 주석 참고.
"""

from dataclasses import dataclass

import force_control
import stiffness

APPROACH = "APPROACH"
PROBE = "PROBE"
CLASSIFY = "CLASSIFY"
HOLD = "HOLD"
BACKOFF = "BACKOFF"
FROZEN = "FROZEN"
NO_CONTACT = "NO_CONTACT"


@dataclass(frozen=True)
class GraspParams:
    """파지 파라미터 묶음.

    hand_config 를 직접 읽지 않고 값을 받는다. 그래야 테스트가 전역
    설정을 건드리지 않고 극단값을 넣어볼 수 있다. 실제 실행에서는
    from_config() 가 hand_config 에서 채운다.
    """
    f_touch: float
    f_abort: float
    f_abort_hyst: float
    f_target_rigid: float
    f_target_soft: float
    f_deadband: float
    touch_confirm_cycles: int
    contact_lost_cycles: int
    k_threshold: float | None
    k_min: float
    k_max: float
    probe_min_span: float
    probe_step: float
    probe_steps: int
    probe_settle_s: float
    a_rate: float
    a_max: float
    lam: float
    ki: float
    da_max: float
    backoff_step: float
    stall_timeout_s: float
    # 아래 둘은 나중에 추가된 필드라 기본값이 있다. 값을 안 주는 기존
    # 호출부가 예전 동작을 그대로 유지하게 하기 위해서다.
    # from_config() 는 항상 채운다.
    #
    # da_max_open         : None 이면 da_max 와 대칭
    # abort_confirm_cycles: 1 이면 한 사이클만 넘어도 즉시 후퇴
    # hold_open_limit     : None 이면 HOLD 에서 여는 양에 제한이 없다
    # k_fixed             : None 이면 프로빙으로 k_hat 을 측정한다
    da_max_open: float | None = None
    abort_confirm_cycles: int = 1
    hold_open_limit: float | None = None
    k_fixed: float | None = None

    @classmethod
    def from_config(cls):
        """hand_config 에서 값을 읽어 온다. 여기가 유일한 접점이다."""
        import hand_config

        return cls(
            f_touch=hand_config.F_TOUCH,
            f_abort=hand_config.F_ABORT,
            f_abort_hyst=hand_config.F_ABORT_HYST,
            f_target_rigid=hand_config.F_TARGET_RIGID,
            f_target_soft=hand_config.F_TARGET_SOFT,
            f_deadband=hand_config.F_DEADBAND,
            touch_confirm_cycles=hand_config.TOUCH_CONFIRM_CYCLES,
            contact_lost_cycles=hand_config.CONTACT_LOST_CYCLES,
            k_threshold=hand_config.K_THRESHOLD,
            k_min=hand_config.K_MIN,
            k_max=hand_config.K_MAX,
            probe_min_span=hand_config.PROBE_MIN_SPAN,
            probe_step=hand_config.PROBE_STEP,
            probe_steps=hand_config.PROBE_STEPS,
            probe_settle_s=hand_config.PROBE_SETTLE_S,
            a_rate=hand_config.A_RATE,
            a_max=hand_config.A_MAX,
            lam=hand_config.LAMBDA,
            ki=hand_config.KI,
            da_max=hand_config.DA_MAX,
            backoff_step=hand_config.BACKOFF_STEP,
            stall_timeout_s=hand_config.STALL_TIMEOUT_S,
            da_max_open=hand_config.DA_MAX_OPEN,
            abort_confirm_cycles=hand_config.ABORT_CONFIRM_CYCLES,
            hold_open_limit=hand_config.HOLD_OPEN_LIMIT,
            k_fixed=hand_config.K_FIXED,
        )


class FingerGrasp:
    """손가락 하나의 파지 진행 상태."""

    def __init__(self, name, flex_span, params):
        """
        name      : 손가락 이름 (로그용)
        flex_span : a 1.0 당 flex 라디안 (= flex_weight * FLEX_LIMIT_RAD)
        params    : GraspParams
        """
        self.name = name
        self.flex_span = flex_span
        self.params = params

        self.state = APPROACH
        self.a = 0.0
        self.k_hat = None
        self.confident = None
        self.object_class = None
        self.f_target = None

        self._samples = []
        self._touch_cycles = 0     # 연속으로 f_touch 를 넘은 사이클 수
        self._lost_cycles = 0      # 연속으로 f_touch 아래인 사이클 수
        self._wait_until = 0.0
        self._controller = None
        self._resume_state = None
        self._stall_since = None   # 제어기가 한쪽 끝에 붙기 시작한 시각
        self._stall_force = 0.0    # 그 창 동안의 대표 힘
        self._f_target_max = None  # 목표 천장 (CLASSIFY 에서 정한 값)
        self._abort_cycles = 0     # 연속으로 f_abort를 넘은 사이클 수
        self._hold_floor = None    # HOLD에서 여기보다 더 못 편다

    def _contact_lost(self, force, p):
        """물체가 빠졌나. 연속으로 f_touch 아래여야 인정한다.

        한 사이클만 보고 판단하면 센서 노이즈 한 번에 파지를 놓는다.
        반대로 아예 안 보면 허공을 향해 목표 힘을 쫓는다.
        """
        if force > p.f_touch:
            self._lost_cycles = 0
            return False
        self._lost_cycles += 1
        return self._lost_cycles >= p.contact_lost_cycles

    def update(self, force, flex, now, dt):
        """한 사이클 진행. -> 새 a.

        force : 이 손가락의 힘(N) 또는 None(센서 끊김)
        flex  : 실측 관절각(rad) 또는 None(읽기 실패)
        now   : monotonic 초. 프로빙 대기 판단에 쓴다
        dt    : 지난 사이클로부터의 경과 시간(초). 접근 속도에 쓴다
        """
        p = self.params

        # --- 1. 결측: 힘과 위치 둘 다 있어야 제어식이 성립한다 ---
        if force is None or flex is None:
            if self.state not in (FROZEN, NO_CONTACT):
                self._resume_state = self.state
                self.state = FROZEN
            return self.a
        if self.state == FROZEN:
            self.state = self._resume_state or APPROACH
            self._resume_state = None

        # --- 2. 절대 힘 상한: 어떤 상태보다 우선한다 (NO_CONTACT 포함) ---
        #
        # 한 사이클만 넘는 것으로는 후퇴하지 않는다. 2026-08-14 로그에서
        # 앞뒤가 0.00N 인데 한 사이클만 18.42 / 27.20 / 49.05N 인 표본이
        # 세 번 나왔다. 이 손이 낼 수 있는 힘은 약 1.1N 이므로 그런
        # 크기의 '파지력'은 존재할 수 없다.
        #
        # 원인은 둘 중 하나인데 어느 쪽이든 후퇴는 틀린 대응이다.
        #   - 센서 글리치: tactile.MEDIAN_WINDOW=3 은 설계상 두 장 연속
        #     글리치를 통과시킨다(그 주석 참고). 가짜 입력이다.
        #   - 물체가 미끄러지며 생긴 전단 과도응답: 진짜 신호이긴 하나
        #     수직 파지력이 아니다. 게다가 미끄러지는 중에 손을 여는 것은
        #     정반대 대응이다 -- 놓으라고 부추기는 셈이 된다.
        #
        # 중앙값 창을 넓히는 방법도 있지만 그건 힘 신호 전체에 지연을
        # 더한다. 실측 지연이 이미 0.55초라 더 늘리면 안 된다. 여기서만
        # 연속 확인을 요구하면 정상 경로의 지연은 그대로다.
        if force > p.f_abort:
            self._abort_cycles += 1
            if self._abort_cycles >= p.abort_confirm_cycles:
                self.state = BACKOFF
        else:
            self._abort_cycles = 0
        if self.state == BACKOFF:
            if force < p.f_abort - p.f_abort_hyst:
                # 히스테리시스가 없으면 경계에서 매 사이클 왕복한다.
                if self._controller is not None:
                    self.state = HOLD
                elif len(self._samples) >= 2:
                    # 프로빙 중 상한을 넘어 여기로 왔다. APPROACH로 돌려
                    # 보내면 force가 여전히 f_touch를 넘어 있으므로 즉시
                    # PROBE로 재진입하면서 _samples가 지워지고, 힘도 아직
                    # abort 근처라 곧바로 다시 BACKOFF -- 영원히 반복된다
                    # (강체는 5계단 이동만으로 8N을 넘기 쉬워 실제로 자주
                    # 일어난다). 이미 2개 이상 모았으면 그걸로 분류를 끝낸다.
                    self.state = CLASSIFY
                else:
                    self.state = APPROACH
            else:
                self.a = max(0.0, self.a - p.backoff_step)
                # 후퇴는 HOLD 의 열림 제한을 받지 않는다. 여긴 진짜
                # 과부하 경로라 끝까지 물러설 수 있어야 한다. 대신 바닥을
                # 같이 내린다 -- 안 그러면 BACKOFF 로 내려간 a 를 다음
                # HOLD 사이클이 곧바로 바닥까지 되올려 후퇴를 무효로 만든다.
                if self._hold_floor is not None:
                    self._hold_floor = min(self._hold_floor, self.a)
                return self.a

        if self.state == NO_CONTACT:
            # 허공 판정 뒤에 물체가 손 안으로 들어오는 일이 실제로 있다.
            # 2026-08-14 로그에서 finger2/finger3 가 6.8초에 NO_CONTACT 로
            # 떨어진 뒤 37초 동안 중앙값 0.74~0.98N 으로 물체를 누르고
            # 있었는데(사이클의 82~87% 가 f_touch 초과), 상태기계는 허공
            # 이라고 믿고 a 를 a_max 에 고정한 채 방치했다. 탈출구가
            # BACKOFF(f_abort=8N) 하나뿐이라 정상 파지력 범위(0.5~2N)로
            # 닿는 경우를 전혀 못 잡았기 때문이다. 손가락 5개 중 3개가
            # 제어 밖에 있는 채로 파지하고 있었던 셈이다.
            #
            # 복귀 기준은 APPROACH 와 같다 -- 연속 touch_confirm_cycles.
            # 한 사이클 노이즈로 되살아나면 허공에서 진동한다.
            if force <= p.f_touch:
                self._touch_cycles = 0
                return self.a
            self._touch_cycles += 1
            if self._touch_cycles < p.touch_confirm_cycles:
                return self.a

            # PROBE 를 거치지 않고 CLASSIFY 로 간다. 이미 a 가 a_max 라
            # 계단을 밟을 여유가 없어서, PROBE 로 보내면 5샘플이 전부
            # 같은 a 에서 찍혀 flex 변화폭이 0 이 된다 -- 어차피
            # estimate_stiffness 가 (k_max, False) 를 돌려주는 자리다.
            # 표본 없이 CLASSIFY 로 보내면 같은 결과를 얻으면서 물체를
            # 놓을 위험이 있는 프로빙 동작을 생략할 수 있다. 가장 보수적인
            # k_max 로 힘 제어를 시작하고, 그 사실은 로그에 confident=0
            # 으로 남는다.
            self._touch_cycles = 0
            self._lost_cycles = 0
            self._samples = []
            self.state = CLASSIFY

        # --- 3. 상태별 ---
        if self.state == APPROACH:
            if force > p.f_touch:
                # 한 사이클만 넘어서는 접촉으로 안 친다. 2026-08-13
                # 종이컵 실측에서 손가락이 테두리를 치는 순간 4.8N 이
                # 잡혔다가 컵이 좌굴하면서 0 으로 꺼졌다. 그걸 접촉으로
                # 세면 곧바로 프로빙에 들어가 힘이 사라지는 동안 표본을
                # 모으고, k 는 confident=0 짜리 쓰레기가 된다.
                self._touch_cycles += 1
                if self._touch_cycles >= p.touch_confirm_cycles:
                    # k_fixed 가 있으면 잴 게 없다. 계단을 밟는 동안
                    # (probe_steps × probe_settle_s) 물체를 밀어 놓칠
                    # 위험만 남으므로 곧바로 CLASSIFY 로 간다.
                    self.state = CLASSIFY if p.k_fixed is not None else PROBE
                    self._samples = []
                    self._lost_cycles = 0
                    self._wait_until = now      # 첫 샘플은 즉시
                return self.a
            else:
                self._touch_cycles = 0
                # dt 는 실측 사이클 시간이라 시리얼 타임아웃(0.5초) 뒤
                # 한 번 부풀 수 있다. 그대로 a_rate*dt 를 쓰면 da_max 의
                # 몇 배가 한 사이클에 튀어 나가 접근 중이던 물체를 그대로
                # 들이받는다. da_max 로 한 번 더 막는다.
                step = min(p.a_rate * dt, p.da_max)
                self.a = min(p.a_max, self.a + step)
                if self.a >= p.a_max:
                    # 허공을 잡았다. 더 밀면 링키지가 스스로를 조인다.
                    self.state = NO_CONTACT
            return self.a

        if self.state == PROBE:
            # 물체가 도망갔는지 먼저 본다. 종이컵은 좌굴하면 훅 꺼지고,
            # 그러면 남은 계단은 전부 허공에서 재는 셈이다.
            if self._contact_lost(force, p):
                self.state = APPROACH
                self._samples = []
                return self.a
            if now < self._wait_until:
                # 서보가 도착하기 전에 재면 힘과 각도의 짝이 안 맞는다.
                return self.a
            self._samples.append((flex, force))
            if len(self._samples) >= p.probe_steps:
                self.state = CLASSIFY
            else:
                self.a = min(p.a_max, self.a + p.probe_step)
                self._wait_until = now + p.probe_settle_s
                return self.a

        if self.state == CLASSIFY:
            if p.k_fixed is not None:
                # 사람이 정한 값이라 confident=True 다. False 는
                # "추정이 실패해서 k_max 를 쓴다"는 표시로 남겨 둔다 --
                # 여기에 붙이면 로그에서 두 상황이 구분되지 않는다.
                self.k_hat, self.confident = p.k_fixed, True
            else:
                self.k_hat, self.confident = stiffness.estimate_stiffness(
                    self._samples, p.k_min, p.k_max, p.probe_min_span)
            self.object_class = stiffness.classify(self.k_hat, p.k_threshold)
            self.f_target = (p.f_target_rigid
                             if self.object_class == "rigid"
                             else p.f_target_soft)
            # 이 값이 목표의 천장이다. HOLD 가 손가락별로 아래로 적응할
            # 수는 있어도, 설정보다 세게 잡는 일은 없어야 한다.
            self._f_target_max = self.f_target
            self._controller = force_control.FingerForceController(
                f_target=self.f_target, k_hat=self.k_hat,
                flex_span=self.flex_span, lam=p.lam, ki=p.ki,
                deadband=p.f_deadband, da_max=p.da_max, a_max=p.a_max,
                da_max_open=p.da_max_open,
            )
            # HOLD 에서 여기보다 더 펴지 않는다. "힘을 낮추려다 물체를
            # 놓는" 경로를 원천 차단하는 바닥이다.
            #
            # 2026-08-14 로그: 힘이 목표를 넘자 제어기가 14초에 걸쳐 a 를
            # 0.900 -> 0.602 로 풀었고 물체가 빠졌다. 방향 자체는 제어식대로
            # 맞았지만, 이 물체는 펴도 힘이 잘 안 줄어서 접촉이 끊길 때까지
            # 여는 것 말고 종료 조건이 없었다.
            self._hold_floor = (
                None if p.hold_open_limit is None
                else max(0.0, self.a - p.hold_open_limit)
            )
            self.state = HOLD

        if self.state == HOLD:
            # 잡고 있던 물체가 빠졌으면 다시 찾으러 간다. 이게 없으면
            # 목표 힘을 향해 허공을 조이며 a_max 까지 올라간다 -- 실측
            # 로그에서 13초 동안 F=0.00 인 채로 그러고 있었다. 스톨
            # 가드가 잡긴 하지만 a_max 에 닿은 뒤에야 작동한다.
            if self._contact_lost(force, p):
                self.state = APPROACH
                self._controller = None
                self._stall_since = None
                return self.a
            new_a = self._controller.update(self.a, force)
            if self._hold_floor is not None:
                new_a = max(new_a, self._hold_floor)
                # 더 깊이 잡았으면 바닥도 따라 올라간다. 규약은 "HOLD 진입
                # 지점"이 아니라 "지금까지 잡은 것 중 가장 깊은 지점에서
                # hold_open_limit 이상은 못 내준다"이다. 진입 지점에
                # 고정하면, HOLD 에 들어온 뒤 더 조인 만큼 여유가 그대로
                # 늘어나 제한이 무의미해진다.
                self._hold_floor = max(self._hold_floor,
                                       new_a - p.hold_open_limit)
            self.a = new_a

            # --- 목표가 도달 불가하면 목표를 옮긴다 (양방향) ---
            #
            # 손가락마다 낼 수 있는 힘이 접촉 지오메트리 때문에 크게 다르다
            # (2026-08-14 로그의 HOLD 중앙값: finger2 0.40N, finger4 1.72N
            # -- 같은 물체를 같이 잡는데 4배). 손 전체에 목표 하나를 주면
            # 어떤 손가락에는 영영 도달 불가능한 값이 된다.
            #
            # 판정 기준은 "제어기가 한쪽 끝에 붙어 있는데도 목표 쪽으로
            # 못 간다"이고, 양쪽 끝을 대칭으로 본다:
            #
            #   a_max 에 붙었는데 힘이 모자란다  -> 더 조일 수 없다 -> 목표를 내린다
            #   바닥에 붙었는데 힘이 남는다      -> 더 풀 수 없다   -> 목표를 올린다
            #
            # 올리는 쪽이 없으면 목표가 한 번 내려간 뒤 영영 못 돌아온다.
            # 2026-08-14 13-01-13 로그의 finger1 이 그랬다 -- 힘이 잠깐
            # 0.67N 이던 순간에 목표가 거기 고정됐는데 실제 힘 중앙값은
            # 1.12N 이라, 제어기가 내내 열려고 했고 열림 바닥에만 의지해
            # 버텼다(낙폭이 정확히 HOLD_OPEN_LIMIT 였다). 여유가 0 인
            # 상태라 물체가 조금만 움직였으면 그대로 놓쳤을 것이다.
            #
            # 목표는 CLASSIFY 에서 설정한 값을 천장으로 삼는다. 즉 손가락은
            # 그 아래로 적응할 뿐, 설정보다 세게 잡는 일은 없다.
            pinned_closed = self.a >= p.a_max and force < self.f_target
            pinned_open = (self._hold_floor is not None
                           and self.a <= self._hold_floor + 1e-9
                           and force > self.f_target)
            if pinned_closed or pinned_open:
                if self._stall_since is None:
                    self._stall_since = now
                    self._stall_force = force
                elif pinned_closed:
                    # 창 안에서 제일 잘 나온 힘을 쓴다. 순간적인 골짜기에
                    # 목표가 고정되는 걸 막는다 -- 위 finger1 사례다.
                    self._stall_force = max(self._stall_force, force)
                else:
                    self._stall_force = force

                if now - self._stall_since >= p.stall_timeout_s:
                    if pinned_closed and force <= p.f_touch:
                        # 잡고 있는데 목표에 못 미치는 것과, 아무것도 없는데
                        # 조이는 것은 다른 상황이다. 후자만 포기한다.
                        self.state = NO_CONTACT
                    else:
                        self.f_target = min(
                            max(self._stall_force, p.f_touch),
                            self._f_target_max)
                        self._controller.f_target = self.f_target
                    self._stall_since = None
            else:
                self._stall_since = None

        return self.a
