# -*- coding: utf-8 -*-
"""파지 상태기계. 하드웨어도 카메라도 네트워크도 모른다.

roi_grasp.py 에서 갈라져 나왔다. executor 규약(start_grasp / start_open /
tick() -> str)만 지키면 고정 자세든 촉각 힘 제어든 이 파일은 안 바뀐다.
"""

import time

import wrist_align

# --- 손 ---
GRASP_A = 0.8        # 파지 시 hand_control 의 a 값
SETTLE_S = 1.0       # 시퀀스 종료 후 모터가 실제로 도착하기를 기다리는 시간
# 폄이 끝난 뒤 다시 판정을 시작하기까지의 대기(초).
# seq.tick() 이 False 인 시점은 마지막 목표를 '보낸' 때지 서보가 도착한
# 때가 아니다. 손이 실제로 벌어지고, 사람이 물체를 치울 시간을 준다.
# 짧으면 놓자마자 같은 물체를 다시 잡는다.
REARM_S = 3.0

# --- 정렬 ---
# 아래는 전부 **미검증 시작값**이다. 실물에서 화면을 보고 정한다.
#
# 오차가 이 안이면 맞은 것으로 본다(도).
ALIGN_TOL_DEG = 1.5
# 일단 맞은 뒤에는 여기까지 봐준다(도). ALIGN_TOL_DEG 이상이어야 한다.
#
# ratio 가 ENTER 0.30 / EXIT 0.15 로 나뉜 것과 같은 구조다. 하나로 쓰면
# CONFIRM_HOLD_S 동안 각도 노이즈가 한 번만 튀어도 ALIGNED 가 풀려
# 타이머를 버리고 ALIGNING 으로 되돌아간다 -- 정렬은 되는데 확인 창을
# 못 넘기는 상태가 된다.
#
# 정렬 정밀도는 ALIGN_TOL_DEG 가 그대로 결정한다. 이 값은 "이미 맞춘
# 것을 언제 깨진 것으로 볼까"만 정한다.
ALIGN_HOLD_TOL_DEG = 3.0
# 오차 1도당 손목을 몇 도 돌릴지. 1.0 이면 한 번에 다 가라는 뜻이라
# 오버슈트하고, 너무 작으면 타임아웃에 걸린다.
ALIGN_GAIN = 0.3
# 허용치 안에 연속 몇 프레임이면 정렬 완료. RatioTrigger 의 ENTER_FRAMES
# 와 같은 이유다 -- 한 프레임 맞은 값으로 파지에 들어가면 안 된다.
ALIGN_STABLE_FRAMES = 5
# 손목이 가동범위 한계에 붙은 채 연속 몇 프레임이면 '정렬 불가'.
ALIGN_PINNED_FRAMES = 15
# 이 시간을 넘으면 정렬 포기(초).
ALIGN_TIMEOUT_S = 20.0

# --- 3초 확인 창 ---
# 정렬이 끝난 뒤 이 시간 동안 ROI 존재와 각도를 **동시에** 유지해야
# 파지한다. 하나라도 깨지면 타이머를 리셋한다.
#
# 이름을 SETTLE 이 아니라 CONFIRM 으로 쓴다. 위의 SETTLE_S 는 "모터가
# 목표에 도착하기를 기다리는 시간"이라 뜻이 전혀 다르다.
CONFIRM_HOLD_S = 1.0


ARMED = "ARMED"
ALIGNING = "ALIGNING"
CONFIRMING = "CONFIRMING"
GRASPING = "GRASPING"
HOLDING = "HOLDING"
RELEASING = "RELEASING"


class SequenceExecutor:
    """PoseSequencer 를 실행기 규약으로 감싼다.

    파지 실행을 "고정 자세를 한 번 보낸다"로 하는 구현이다. 촉각 센서가
    없어도 돌기 때문에 상태기계만 확인할 때 쓴다.

    --- settle_s 가 여기 있는 이유 ---
    seq.tick() 이 False 인 시점은 마지막 목표를 '보낸' 때지 서보가 도착한
    때가 아니다. 도착 여부는 실행 방식에 딸린 사정이므로 상태기계가 아니라
    실행기가 안다.
    """

    def __init__(self, seq, grasp_a=GRASP_A, settle_s=SETTLE_S,
                 clock=time.perf_counter):
        self.seq = seq
        self.grasp_a = grasp_a
        self.settle_s = settle_s
        self._clock = clock
        self._settled_at = None
        self._opening = False

    def start_grasp(self):
        self.seq.start(self.grasp_a, 0.0)
        self._settled_at = None
        self._opening = False

    def start_open(self):
        self.seq.start(0.0, 0.0)
        self._settled_at = None
        self._opening = True

    def tick(self):
        busy = self.seq.tick()
        if busy:
            return "busy"
        if self._opening:
            return "opened"
        if self._settled_at is None:
            self._settled_at = self._clock() + self.settle_s
        return "settled" if self._clock() >= self._settled_at else "busy"


class GraspStateMachine:
    """ROI 판정 -> 손 구동. 하드웨어를 모른다.

    executor 는 start_grasp() / start_open() / tick() -> str 규약을 가진
    객체면 무엇이든 된다. tick() 은 "busy" / "settled" / "opened" /
    "abort" 중 하나를 돌려준다. 고정 자세 시퀀스(SequenceExecutor)든
    촉각 힘 제어(ForceGraspRunner)든 상태기계는 어느 쪽인지 모른다 --
    그래서 파지 방식을 바꿔도 이 클래스는 안 바뀐다.

    --- 왜 ARMED 에서만 판정하나 ---
    파지가 시작되면 손가락이 ROI 안으로 들어오고 물체도 손에 가려진다.
    그 상태에서 판정을 계속하면 "물체 사라짐 -> 폄 -> 다시 보임 -> 잡음"
    이 무한 반복된다.

    --- 왜 트리거를 이 클래스가 들고 있나 ---
    호출부가 "ARMED 일 때만 trigger.update() 를 불러라"를 지키는 구조였는데,
    그러면 다른 상태에 있는 동안 트리거가 옛 상태(active=True)로 얼어붙는다.
    ARMED 로 돌아온 첫 프레임에 enter_frames 조건이 통째로 건너뛰어져서
    물체가 남아 있으면 곧바로 재파지됐다. 트리거의 수명을 상태 전이와 같이
    관리해야 이 종류의 어긋남이 원천적으로 안 생긴다.
    """

    def __init__(self, executor, trigger, aligner=None,
                 clock=time.perf_counter, rearm_s=REARM_S,
                 confirm_hold_s=CONFIRM_HOLD_S, align_enabled=True,
                 armed=True):
        self.executor = executor
        self.trigger = trigger
        self.aligner = aligner
        self._clock = clock
        self.rearm_s = rearm_s
        self.confirm_hold_s = confirm_hold_s
        self.align_enabled = align_enabled and aligner is not None
        # 무장하지 않으면 판정을 아예 안 한다. 기본값이 True 인 이유는
        # 이 클래스를 단독으로 쓰던 코드와 테스트를 안 깨기 위해서다 --
        # 데몬은 armed=False 로 만들어 "사람이 걸어야 한다"를 지킨다.
        self.armed = bool(armed)
        self.state = ARMED
        self.align_status = None       # 화면에 띄운다
        self.wrist_goal_rad = None     # 이번 프레임에 손목으로 보낼 값
        self._rearm_at = 0.0
        self._confirm_until = None

    def _to_armed(self):
        """어떤 실패에서든 여기로 돌아온다. 쿨다운을 건다.

        트리거를 지워야 다음 파지가 enter_frames 를 처음부터 다시 채운다.
        """
        self.trigger.reset()
        self._rearm_at = self._clock() + self.rearm_s
        self._confirm_until = None
        self.wrist_goal_rad = None
        self.state = ARMED

    def update(self, ratio, angle_deg=None):
        """매 프레임 한 번. 이번 프레임의 밴드 비율과 각도를 준다.

        ratio 와 angle_deg 는 ARMED/ALIGNING/CONFIRMING 에서만 쓰인다.
        파지가 시작되면 손가락이 ROI 안으로 들어오고 물체도 손에 가려져서,
        그 값으로 판정을 계속하면 "사라짐 -> 폄 -> 다시 보임 -> 잡음"이
        무한 반복된다.
        """
        # tick() 은 어느 상태에서든 부른다. 보낼 게 없으면 실행기가 스스로
        # 판단해 돌려주므로 상태별로 가르지 않는다.
        status = self.executor.tick()
        self.wrist_goal_rad = None

        # ratio 가 exit 아래로 떨어졌나. 정렬/확인 중에 물체가 나가면
        # 곧바로 물러난다.
        gone = (ratio is None or ratio < self.trigger.exit_ratio)

        if self.state == ARMED:
            if not self.armed:
                # 무장 해제. 물체가 보여도 연속 프레임을 못 쌓게 지운다.
                # 쿨다운과 같은 자리에서 같은 일을 한다.
                self.trigger.reset()
            elif self._clock() < self._rearm_at:
                # 쿨다운 중. 물체가 보여도 연속 프레임을 못 쌓게 계속 지운다.
                self.trigger.reset()
            elif self.trigger.update(ratio):
                if not self.align_enabled:
                    # 정렬을 끄면 ALIGNING 을 건너뛴다. 각도 조건도 안 본다.
                    self._confirm_until = self._clock() + self.confirm_hold_s
                    self.state = CONFIRMING
                elif angle_deg is None:
                    # 모르면 잡지 않는다. 트리거를 지워 다시 쌓게 한다.
                    self.trigger.reset()
                else:
                    # 지금 손목 자리에서 시작한다. 0 으로 되돌리면 첫 명령이
                    # 손목을 중립까지 한 번에 보내 카메라가 크게 흔들린다.
                    self.aligner.start(self.aligner.goal_rad)
                    self.state = ALIGNING

        elif self.state == ALIGNING:
            if gone:
                self._to_armed()
            else:
                goal, align_status = self.aligner.update(angle_deg)
                self.align_status = align_status
                self.wrist_goal_rad = goal
                if align_status == wrist_align.ALIGNED:
                    self._confirm_until = (self._clock()
                                           + self.confirm_hold_s)
                    self.state = CONFIRMING
                elif align_status in (wrist_align.UNREACHABLE,
                                      wrist_align.TIMEOUT):
                    print(f"[INFO] 정렬 실패({align_status}). "
                          f"파지하지 않습니다.")
                    self._to_armed()

        elif self.state == CONFIRMING:
            if gone:
                self._to_armed()
            elif self.align_enabled:
                goal, align_status = self.aligner.update(angle_deg)
                self.align_status = align_status
                self.wrist_goal_rad = goal
                if align_status != wrist_align.ALIGNED:
                    # 각도가 틀어졌다. 타이머만 리셋하고 제자리에 두면
                    # 손목이 안 움직여서 영영 안 잡힌다. 다시 맞춘다.
                    self._confirm_until = None
                    self.state = ALIGNING
                elif self._clock() >= self._confirm_until:
                    self._confirm_until = None
                    self.executor.start_grasp()
                    self.state = GRASPING
            elif self._clock() >= self._confirm_until:
                self._confirm_until = None
                self.executor.start_grasp()
                self.state = GRASPING

        elif self.state == GRASPING:
            if status == "abort":
                # 과열이나 온도 감시 불능이다. 손을 풀어야 한다.
                self.executor.start_open()
                self.state = RELEASING
            elif status == "settled":
                # 실행기가 '도착했다'고 알린 시점이다. 고정 자세면 서보가
                # 목표에 닿은 때고, 힘 제어면 모든 손가락이 안정된 때다.
                self.state = HOLDING

        elif self.state == HOLDING:
            if status == "abort":
                self.executor.start_open()
                self.state = RELEASING

        elif self.state == RELEASING:
            if status == "opened":
                self._to_armed()

        return self.state

    def set_align_enabled(self, enabled):
        """a 키. 정렬기가 없으면 켜지지 않는다.

        손목이 없는데(--no-wrist) 켜면 ALIGNING 에서 아무도 각도를 안
        지워서 영영 CONFIRMING 에 못 간다. 켤 수 없다는 걸 그 자리에서
        알려야 한다.
        """
        self.align_enabled = bool(enabled) and self.aligner is not None
        if not self.align_enabled and self.state == ALIGNING:
            # 정렬이 안 되는 걸 보고 그 자리에서 껐다. 3초 창부터 다시.
            self._confirm_until = self._clock() + self.confirm_hold_s
            self.wrist_goal_rad = None
            self.state = CONFIRMING

    def confirm_remaining(self):
        """3초 창이 끝나기까지 남은 초. 창 밖이면 0.0."""
        if self._confirm_until is None:
            return 0.0
        return max(0.0, self._confirm_until - self._clock())

    def rearm_remaining(self):
        """재무장까지 남은 초. 쿨다운 중이 아니면 0.0.

        화면에 띄우기 위한 것이다. 쿨다운 중에도 상태는 ARMED 라서,
        표시가 없으면 "물체를 넣었는데 왜 반응이 없지"로 보인다.
        """
        return max(0.0, self._rearm_at - self._clock())

    def request_grasp(self):
        """g 키 / GRASP 버튼. 지금 곧바로 파지한다. -> 받았나.

        ROI 트리거가 걸려야만 파지가 시작되면 반복 시험이 번거롭다.
        강성 분류나 벌림 탐색처럼 파지 안쪽 로직을 고칠 때, 카메라
        조건을 매번 맞추지 않고 바로 걸 수 있어야 한다.

        --- 무장 해제와 쿨다운을 무시하는 이유 ---
        둘 다 "자동으로 새 파지를 시작하지 않는다"를 위한 장치다.
        disarm 은 사람이 화면을 안 볼 때를, 쿨다운은 놓자마자 같은
        물체를 다시 잡는 것을 막는다. 사람이 직접 누른 것은 그 두
        상황 어느 쪽도 아니다 -- emergency_open 과 같은 성질이다.

        --- 진행 중인 파지는 거절한다 ---
        GRASPING/HOLDING/RELEASING 에서 다시 시작하면 손가락이 반쯤
        닫힌 채로 상태가 초기화되어 물체에 끼인다. disarm() 이 진행 중인
        동작을 안 건드리는 것과 같은 이유다.
        """
        if self.state not in (ARMED, ALIGNING, CONFIRMING):
            return False
        # 안 지우면 파지가 끝나고 ARMED 로 돌아온 첫 프레임에 옛 연속
        # 프레임이 남아 곧바로 자동 재파지된다.
        self.trigger.reset()
        self._confirm_until = None
        self.wrist_goal_rad = None
        self.executor.start_grasp()
        self.state = GRASPING
        return True

    def request_release(self):
        """r 키. 잡고 있을 때만 받아들인다.

        '모터가 도착했는지'는 실행기가 settled 로 이미 알려 줬다.
        """
        if self.state != HOLDING:
            return False
        self.executor.start_open()
        self.state = RELEASING
        return True

    def emergency_open(self):
        """space 키. 어느 상태에서든 즉시 편다.

        ALIGNING/CONFIRMING 중에는 손 포트가 안 열려 있다. 호출부가
        리스를 손으로 스왑한 뒤에 불러야 한다 -- 그러지 않으면 비상
        정지가 가장 필요한 순간에 조용히 아무것도 안 한다.

        이미 펴는 중이면 아무것도 하지 않는다. 키를 누르고 있으면 이
        함수가 매 프레임 불리는데, 그때마다 start_open() 을 다시 부르면
        PoseSequencer 의 경과 시간이 리셋되어 지연을 받은 손가락이 영영
        출발하지 못한다 -- 엄지만 펴지고 나머지 네 개는 키를 뗄 때까지
        그대로 있는다.
        """
        if self.state == RELEASING:
            return
        self._confirm_until = None
        self.wrist_goal_rad = None
        self.executor.start_open()
        self.state = RELEASING

    def arm(self):
        """사람이 화면을 보고 있다는 확인. 여기서부터 판정한다."""
        self.armed = True

    def disarm(self):
        """새 파지를 시작하지 않는다.

        진행 중인 동작은 건드리지 않는다 -- GRASPING 을 중간에 멈추면
        손가락이 반쯤 닫힌 채 물체에 끼인다. 상태별 조치는 부르는 쪽이
        정한다.
        """
        self.armed = False

    def abort_to_armed(self):
        """정렬/확인을 접고 물러난다. 쿨다운도 같이 건다."""
        self._to_armed()
