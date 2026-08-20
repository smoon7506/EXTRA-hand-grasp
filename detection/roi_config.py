# -*- coding: utf-8 -*-
"""ROI 박스·깊이 밴드·목표 각도. 이 값들의 진실의 출처다.

roi_grasp.py 에서 갈라져 나왔다. 판정하는 쪽(파이 데몬)이 이 파일을
소유한다 -- ROI 를 두 곳에서 들고 있으면 조용히 어긋난다.
"""

import json
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

import orientation

# --- 깊이 밴드(m) ---
# 밴드의 의미는 '손이 감쌀 수 있는 부피'다. near = 손끝 평면,
# far = 손바닥 평면. far 보다 멀면 손 뒤 배경이고, near 보다 가까우면
# 아직 손 밖이다.
# 아래는 카메라~손끝 200mm 기준의 추정값이다. 실행 중 n/f 키로 실측해
# roi.json 에 저장하면 그 값이 우선한다.
NEAR_M, FAR_M = 0.6, 0.14

# --- 캘리브레이션 ---
# n/f 로 찍은 값을 바깥으로 벌리는 여유(m). 센서 노이즈가 ±1cm 정도다.
MARGIN_M = 0.01
# [ ] - = 키로 경계를 한 번에 움직이는 폭(m).
# n/f 는 현재 중앙값으로 덮어쓰기라 "지금 값에서 조금만" 이 안 된다.
NUDGE_M = 0.005

# , / . 키로 목표 각도를 미세조정하는 폭(도).
# t 는 덮어쓰기라 "지금 값에서 조금만"이 안 된다. [ ] - = 가 있는 것과
# 같은 이유다.
TARGET_NUDGE_DEG = 1.0

# 드래그와 n/f 캘리브레이션 결과. 다음 실행에서 자동으로 읽는다.
ROI_JSON = Path(__file__).with_name("roi.json")


@dataclass
class RoiConfig:
    """ROI 박스(픽셀)와 깊이 밴드(m). 실행 중 갱신되고 파일로 남는다.

    frozen 이 아닌 이유: n/f 키 캘리브레이션이 near_m/far_m 을 그 자리에서
    바꾼다. 박스와 밴드는 같이 맞춰야 의미가 있어서 한 파일에 같이 둔다.
    """

    x: int
    y: int
    w: int
    h: int
    near_m: float
    far_m: float
    # 물체 장축의 목표 상대각(도). t 키로 찍는다.
    #
    # 손목 위치가 아니라 목표 상대각을 저장하는 이유: 병이 매번 다른
    # 각도로 놓이면 손목의 절대 위치는 매번 달라지지만, 결과로 만들어지는
    # 손-물체 상대 자세는 항상 같아야 한다.
    #
    # 기본값 0.0 은 "아직 안 찍었다"에 가깝다. 실제 값은 카메라의 롤
    # 장착 각도에 달려 있어 계산으로 못 정한다.
    target_angle_deg: float = 0.0

    def validate(self):
        """어긋난 값이면 ValueError. 저장 직전과 로드 직후에 부른다."""
        if self.w <= 0 or self.h <= 0:
            raise ValueError(
                f"ROI 크기가 0 이하다 (w={self.w}, h={self.h}). "
                f"드래그가 한 점에서 끝나면 이렇게 된다."
            )
        if self.near_m >= self.far_m:
            raise ValueError(
                f"near_m({self.near_m}) 가 far_m({self.far_m}) 이상이다. "
                f"n 은 손끝(가까움), f 는 손바닥(멂)에서 찍어야 한다."
            )
        if not -90.0 < self.target_angle_deg <= 90.0:
            raise ValueError(
                f"target_angle_deg({self.target_angle_deg})가 (-90, 90] "
                f"밖이다. orientation.wrap90 의 규약이라, 밖의 값이 파일에 "
                f"들어가면 정렬 오차 계산이 조용히 어긋난다."
            )

    def slice(self, depth):
        """depth 이미지에서 ROI 부분만 잘라낸다.

        numpy 는 [행, 열] = [y, x] 순서다. 뒤집으면 ROI 가 엉뚱한 데를
        보는데, 화면에는 박스가 제자리에 그려져서 찾기 어렵다.
        """
        return np.asarray(depth)[self.y:self.y + self.h,
                                 self.x:self.x + self.w]

    def save(self, path=ROI_JSON):
        self.validate()
        Path(path).write_text(
            json.dumps(asdict(self), indent=2), encoding="utf-8")

    @classmethod
    def load(cls, path=ROI_JSON):
        """파일이 없으면 None(첫 실행). 내용이 깨졌으면 ValueError.

        깨진 파일을 조용히 기본값으로 대체하면 "어제 맞춰둔 ROI 가 왜
        다르지"를 영영 못 찾는다. 시끄럽게 실패한다.
        """
        path = Path(path)
        if not path.exists():
            return None
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
            roi = cls(x=int(data["x"]), y=int(data["y"]),
                      w=int(data["w"]), h=int(data["h"]),
                      near_m=float(data["near_m"]),
                      far_m=float(data["far_m"]),
                      target_angle_deg=float(
                          data.get("target_angle_deg", 0.0)))
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as e:
            raise ValueError(f"{path} 를 읽지 못했다: {e}") from e
        roi.validate()
        return roi


def nudge_band_by(roi, edge, delta_m):
    """밴드 경계를 delta_m 만큼 민다. -> (처리했나, 사유).

    키가 아니라 델타를 받는 이유: 콘솔이 키를, 데몬이 값을 들고 있다.
    사유를 문자열로 돌려주는 이유: 데몬은 print 가 아니라 ack 로 답한다.
    """
    if edge == "near":
        near, far = roi.near_m + delta_m, roi.far_m
    elif edge == "far":
        near, far = roi.near_m, roi.far_m + delta_m
    else:
        return False, f"밴드 경계 이름이 아니다: {edge}"
    if near >= far:
        return False, (f"밴드가 뒤집혀서 무시했다 ({near:.3f} >= {far:.3f}).")
    roi.near_m, roi.far_m = near, far
    return True, f"밴드 {roi.near_m:.3f} ~ {roi.far_m:.3f} m"


def nudge_target_by(roi, delta_deg):
    """목표 각도를 delta_deg 만큼 민다. -> (처리했나, 사유).

    (-90, 90] 로 감는다. 감지 않으면 90도 근처에서 validate() 가 막아
    조용히 저장이 실패한다.
    """
    roi.target_angle_deg = orientation.wrap90(roi.target_angle_deg + delta_deg)
    return True, f"목표 각도 {roi.target_angle_deg:+.1f}도"


def nudge_band(roi, key):
    """[ ] - = 키 -> 밴드 경계를 NUDGE_M 만큼 민다. 처리했으면 True.

    n/f 는 현재 중앙값으로 덮어쓰는 방식이라 "지금 값에서 1cm 만 넓히자"
    가 안 된다. 초록 픽셀을 보면서 실시간으로 조이는 용도다.

    경계를 뒤집는 조작은 무시한다. near >= far 가 되면 그 뒤 모든 판정이
    조용히 0 이 되어, 원인을 화면에서 찾을 수 없게 된다.
    """
    if key == ord("["):
        near, far = roi.near_m - NUDGE_M, roi.far_m
    elif key == ord("]"):
        near, far = roi.near_m + NUDGE_M, roi.far_m
    elif key == ord("-"):
        near, far = roi.near_m, roi.far_m - NUDGE_M
    elif key == ord("="):
        near, far = roi.near_m, roi.far_m + NUDGE_M
    else:
        return False
    if near >= far:
        print("[WARN] 밴드가 뒤집혀서 무시했습니다 "
              f"({near:.3f} >= {far:.3f}).")
        return True
    roi.near_m, roi.far_m = near, far
    return True


def nudge_target(roi, key):
    """, / . 키 -> 목표 각도를 TARGET_NUDGE_DEG 만큼 민다. 처리했으면 True.

    t 는 현재 각도로 덮어쓰기라 "지금 값에서 1도만" 이 안 된다.
    nudge_band 가 있는 것과 같은 이유다.

    (-90, 90] 로 감는다. 감지 않으면 90도 근처에서 . 을 한 번 눌렀을 때
    validate() 가 막아 조용히 저장이 실패한다.
    """
    if key == ord(","):
        delta = -TARGET_NUDGE_DEG
    elif key == ord("."):
        delta = +TARGET_NUDGE_DEG
    else:
        return False
    roi.target_angle_deg = orientation.wrap90(roi.target_angle_deg + delta)
    return True
