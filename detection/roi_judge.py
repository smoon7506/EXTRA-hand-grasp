# -*- coding: utf-8 -*-
"""깊이 ROI -> "물체가 있는가". 하드웨어도 화면도 모른다.

roi_grasp.py 에서 갈라져 나왔다. 여기 있는 것들은 전부 파이(데몬)에서
돈다 -- 깊이 배열이 있어야 계산되기 때문이다. 깊이 배열은 네트워크로
보내지 않는다.
"""

import numpy as np

# --- 판정 ---
# 밴드 안 픽셀이 이 비율을 넘으면 물체가 있다고 본다.
ENTER_RATIO = 0.30
# 트리거가 풀리는 비율. ENTER 보다 작아야 경계에서 안 깜빡인다.
EXIT_RATIO = 0.15
# 트리거가 서기까지 필요한 연속 프레임 수. 30fps 에서 약 0.17초.
ENTER_FRAMES = 5
# ROI 유효 픽셀이 이보다 적으면 화면에 경고한다. 200mm 는 Min-Z 경계라
# 카메라를 조금만 당겨도 ROI 가 통째로 0 이 된다. 그때 '물체 없음'과
# '센서가 못 봄'이 화면에서 구분돼야 한다.
MIN_VALID_RATIO = 0.30


def band_ratio(depth_roi, near_m, far_m):
    """ROI 깊이 배열 -> (밴드 안 비율, 유효 픽셀 비율).

    유효 픽셀이 하나도 없으면 (None, 0.0). None 은 '물체 없음'이 아니라
    '모름'이고, 호출부는 모르면 잡지 않는 쪽으로 실패해야 한다.

    depth 0 은 RealSense 가 반사·경계·최소거리 미만에서 내는 무효값이다.
    분모에서 뺀다 -- 넣으면 물체가 ROI 를 꽉 채워도 비율이 안 올라간다.
    """
    depth = np.asarray(depth_roi)
    if depth.size == 0:
        return (None, 0.0)
    valid = depth > 0
    n_valid = int(np.count_nonzero(valid))
    if n_valid == 0:
        return (None, 0.0)
    # 경계는 양끝 포함. 규칙이 애매하면 캘리브레이션 값이 경계에 걸렸을 때
    # 원인 모를 히스테리시스가 생긴다.
    in_band = valid & (depth >= near_m) & (depth <= far_m)
    return (int(np.count_nonzero(in_band)) / n_valid, n_valid / depth.size)


def roi_median(depth_roi):
    """ROI 유효 깊이의 중앙값(m). 없으면 None.

    캘리브레이션용이다. 화면에 띄워서 사람이 밴드 값을 눈으로 고른다.
    평균이 아니라 중앙값인 이유: 무효 경계에 붙은 튀는 값 몇 개에
    끌려가지 않기 위해서다.
    """
    depth = np.asarray(depth_roi)
    if depth.size == 0:
        return None
    valid = depth[depth > 0]
    if valid.size == 0:
        return None
    return float(np.median(valid))


class RatioTrigger:
    """밴드 안 비율 -> 물체 있음(bool). 히스테리시스 + 연속 프레임 확인.

    detection/grasp_signal.py 의 GraspSignal 과 같은 구조지만 방향이
    반대다(저쪽은 '거리가 작을수록 참', 여기는 '비율이 클수록 참').
    -ratio 를 거리로 먹여 재사용하면 수식은 맞아떨어지지만 나중에 읽는
    사람이 -0.83 이 뭔지 알 수 없어서, 비율 전용으로 따로 둔다.
    """

    def __init__(self, enter_ratio=ENTER_RATIO, exit_ratio=EXIT_RATIO,
                 enter_frames=ENTER_FRAMES):
        if exit_ratio > enter_ratio:
            raise ValueError(
                f"exit_ratio({exit_ratio}) 가 enter_ratio({enter_ratio}) 보다 "
                f"크면 히스테리시스가 뒤집혀 오히려 더 깜빡인다."
            )
        self.enter_ratio = enter_ratio
        self.exit_ratio = exit_ratio
        self.enter_frames = max(1, enter_frames)
        self.active = False
        self._hits = 0

    def reset(self):
        """연속 프레임 카운트와 상태를 지운다.

        재무장할 때 반드시 불러야 한다. active 가 True 로 남아 있으면
        다음 update() 가 히스테리시스 분기로 들어가서 exit_ratio 만
        넘겨도 즉시 True 를 돌려준다 -- enter_frames 조건이 통째로
        건너뛰어진다.
        """
        self.active = False
        self._hits = 0

    def update(self, ratio):
        """이번 프레임의 밴드 비율(없으면 None) -> 트리거 상태(bool).

        ratio 가 None 이면 즉시 내린다. None 은 '물체 없음'이 아니라
        '센서가 못 봄'이고, 모르는 채로 손을 닫아서는 안 된다.
        """
        if ratio is None:
            self._hits = 0
            self.active = False
            return self.active

        if self.active:
            if ratio < self.exit_ratio:
                self.active = False
                self._hits = 0
            return self.active

        self._hits = self._hits + 1 if ratio >= self.enter_ratio else 0
        if self._hits >= self.enter_frames:
            self.active = True
        return self.active


def axis_segment(mask, angle_deg, half_len=None):
    """마스크 중심을 지나는 주축 선분 ((x0,y0), (x1,y1)). ROI 안 좌표계.

    화면에 그려서 "무엇을 물체로 보고 있는가"를 눈으로 확인하기 위한
    것이다. 각도 숫자만으로는 손을 보고 있는지 병을 보고 있는지 모른다.
    """
    ys, xs = np.nonzero(mask)
    if xs.size == 0:
        return None
    cx, cy = float(xs.mean()), float(ys.mean())
    if half_len is None:
        half_len = 0.4 * max(mask.shape)
    rad = np.radians(angle_deg)
    dx, dy = np.cos(rad) * half_len, -np.sin(rad) * half_len
    return ((cx - dx, cy - dy), (cx + dx, cy + dy))
