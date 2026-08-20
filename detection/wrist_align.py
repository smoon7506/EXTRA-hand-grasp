# -*- coding: utf-8 -*-
"""각도 오차 -> 손목 goal. 하드웨어를 모르는 순수 로직.

--- 왜 비례 제어만 쓰나 ---
정렬은 정지한 물체를 향한 한 번의 수렴이지, 외란을 계속 밀어내는 추종이
아니다. 적분항은 데드밴드 안에서 조용히 쌓였다가 물체가 살짝 움직일 때
튀어나오고, 미분항은 각도 측정 노이즈를 그대로 증폭한다. 둘 다 손해다.

--- 실패를 두 가지로 구분하는 이유 ---
UNREACHABLE 은 손목 가동범위 밖이라 영영 못 맞추는 것이고, TIMEOUT 은
각도가 계속 흔들리거나 게인이 낮은 것이다. 고치는 방법이 달라서, 화면에
'정렬 실패' 하나로만 뜨면 무엇을 만져야 할지 알 수 없다.
"""

import math
import time

from orientation import wrap90

ALIGNING = "ALIGNING"          # 맞추는 중
ALIGNED = "ALIGNED"            # 허용치 안에서 안정
UNREACHABLE = "UNREACHABLE"    # 가동범위 밖이라 못 맞춘다
TIMEOUT = "TIMEOUT"            # 시간 안에 못 맞췄다
NO_ANGLE = "NO_ANGLE"          # 각도를 못 재고 있다


class WristAligner:
    def __init__(self, target_deg, *, tol_deg, gain, da_max_rad,
                 min_rad, max_rad, direction_sign, stable_frames,
                 pinned_frames, timeout_s, hold_tol_deg=None,
                 clock=time.perf_counter):
        """
        target_deg      : 목표 상대각. roi.json 의 target_angle_deg
        tol_deg         : 이 안이면 맞은 것으로 본다 (데드밴드)
        hold_tol_deg    : 일단 ALIGNED 가 된 뒤의 허용치. None 이면
                          tol_deg 와 같다(히스테리시스 없음)
        gain            : 오차 1도당 손목 몇 도를 갈지 (0~1)
        da_max_rad      : 한 사이클 최대 이동
        min_rad/max_rad : 손목 가동범위
        direction_sign  : +1 또는 -1. 실물에서 w/W 조그로 확인한다
        stable_frames   : 허용치 안에 연속 몇 프레임이면 ALIGNED
        pinned_frames   : 한계에 붙은 채 연속 몇 프레임이면 UNREACHABLE
        timeout_s       : 이 시간을 넘으면 TIMEOUT
        """
        if direction_sign not in (1, -1):
            raise ValueError(
                f"direction_sign 은 +1 또는 -1 이어야 한다 "
                f"(받은 값 {direction_sign}). 0 이면 손목이 영영 안 움직인다."
            )
        if min_rad >= max_rad:
            raise ValueError(
                f"min_rad({min_rad}) 가 max_rad({max_rad}) 이상이다. "
                f"가동범위가 없으면 모든 정렬이 UNREACHABLE 이 된다."
            )
        if hold_tol_deg is None:
            hold_tol_deg = tol_deg
        if hold_tol_deg < tol_deg:
            raise ValueError(
                f"hold_tol_deg({hold_tol_deg}) 가 tol_deg({tol_deg}) 보다 "
                f"작으면 히스테리시스가 뒤집혀 ALIGNED 가 되자마자 풀린다. "
                f"RatioTrigger 의 exit_ratio 검사와 같은 이유다."
            )
        self.target_deg = wrap90(target_deg)
        self.tol_deg = tol_deg
        self.hold_tol_deg = hold_tol_deg
        self.gain = gain
        self.da_max_rad = da_max_rad
        self.min_rad = min_rad
        self.max_rad = max_rad
        self.direction_sign = direction_sign
        self.stable_frames = max(1, stable_frames)
        self.pinned_frames = max(1, pinned_frames)
        self.timeout_s = timeout_s
        self._clock = clock

        self.goal_rad = 0.0
        self.status = ALIGNING
        self.error_deg = None
        self._stable = 0
        self._pinned = 0
        self._t0 = None

    def start(self, goal_rad):
        """현재 손목 위치에서 정렬을 시작한다.

        goal 을 0 이 아니라 실측 위치로 잡는 이유: 0 으로 시작하면 첫
        명령이 손목을 중립까지 한 번에 되돌린다 -- 그 사이 카메라가 크게
        흔들려 각도 측정이 통째로 날아간다.
        """
        self.goal_rad = max(self.min_rad, min(self.max_rad, float(goal_rad)))
        self.status = ALIGNING
        self.error_deg = None
        self._stable = 0
        self._pinned = 0
        # 타임아웃은 '정렬을 시작한 때'부터 잰다. 첫 update() 까지 미루면
        # 각도를 못 재는 동안 흐른 시간이 통째로 빠져서, 물체를 못 보는
        # 채로 손목이 한없이 기다린다.
        self._t0 = self._clock()

    def update(self, measured_deg):
        """이번 프레임의 측정 각도(없으면 None) -> (goal_rad, status)."""
        now = self._clock()
        if self._t0 is None:
            self._t0 = now

        if measured_deg is None:
            # 모르면 움직이지 않는다. 연속 판정도 끊는다 -- 안 끊으면
            # 못 보는 사이에 ALIGNED 로 넘어가 그대로 파지에 들어간다.
            self._stable = 0
            self._pinned = 0
            self.error_deg = None
            self.status = NO_ANGLE
            return (self.goal_rad, self.status)

        error = wrap90(measured_deg - self.target_deg)
        self.error_deg = error

        # 맞추는 중에는 좁은 쪽(tol_deg), 이미 맞은 뒤에는 넓은 쪽
        # (hold_tol_deg). 하나로 쓰면 확인 창 동안 노이즈가 한 번만 튀어도
        # ALIGNED 가 풀려 GraspStateMachine 이 타이머를 버리고 ALIGNING
        # 으로 되돌아간다 -- 그러면 영영 파지에 못 들어간다.
        tol = (self.hold_tol_deg if self.status == ALIGNED
               else self.tol_deg)
        if abs(error) <= tol:
            self._stable += 1
            self._pinned = 0
            self.status = (ALIGNED if self._stable >= self.stable_frames
                           else ALIGNING)
            return (self.goal_rad, self.status)

        self._stable = 0
        step = math.radians(self.gain * error) * self.direction_sign
        step = max(-self.da_max_rad, min(self.da_max_rad, step))
        wanted = self.goal_rad + step
        clamped = max(self.min_rad, min(self.max_rad, wanted))
        # 한계에 붙어 더 못 가는 상태를 센다. 부호가 반대일 때도 여기로
        # 떨어진다 -- 오차를 키우며 한계까지 갔다가 멈춘다.
        if abs(clamped - wanted) > 1e-9:
            self._pinned += 1
        else:
            self._pinned = 0
        self.goal_rad = clamped

        if self._pinned >= self.pinned_frames:
            self.status = UNREACHABLE
        elif now - self._t0 >= self.timeout_s:
            self.status = TIMEOUT
        else:
            self.status = ALIGNING
        return (self.goal_rad, self.status)
