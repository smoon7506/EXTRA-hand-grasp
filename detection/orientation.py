# -*- coding: utf-8 -*-
"""밴드 마스크 -> 물체 장축 각도. 하드웨어를 모르는 순수 로직.

이미 매 프레임 계산하는 band mask 를 재사용하므로 새 의존성이 없다.
YOLO 를 쓰지 않는 이유는 스펙의 '이번에 하지 않는 것' 참고 -- GPU 추론
지연이 30fps 루프에 들어오는 대가가 크고, 각도만 필요하다.
"""

import math

import cv2
import numpy as np

# 각도를 믿기 위한 최소 조건. 둘 다 미검증 시작값이다.
MIN_ANGLE_PIXELS = 200      # 성분 픽셀 수
MIN_ELONGATION = 2.0        # 이심률. 병 기준


def wrap90(deg):
    """각도를 (-90, 90] 로 접는다.

    장축은 **방향이 없다** -- 병을 180도 돌려도 같은 자세다. 그래서 모든
    각도 비교는 180도 주기로 접어야 하고, 그 결과 정렬 오차는 항상 90도
    이하가 되어 손목이 언제나 짧은 쪽으로 돈다.

    경계를 +90 으로 정한 이유: 정해 두지 않으면 같은 자세가 프레임마다
    +90 과 -90 을 오가며 오차 계산이 180도씩 튄다.
    """
    folded = (float(deg) + 90.0) % 180.0 - 90.0
    # % 는 -90.0 을 주고 90.0 을 못 준다. 규약대로 뒤집는다.
    return 90.0 if folded == -90.0 else folded


def principal_axis(mask, min_pixels=MIN_ANGLE_PIXELS,
                   min_elongation=MIN_ELONGATION):
    """불리언 마스크 -> (각도_deg, 이심률) 또는 (None, 이유).

    실패할 때 이유 문자열을 같이 돌려주는 이유: 화면에 띄우기 위해서다.
    "각도 없음"만 보이면 물체가 없는 건지 너무 둥근 건지 픽셀이 모자란
    건지 알 수 없어서 무엇을 고칠지 못 정한다.

    각도 규약은 사람이 읽는 쪽(반시계 +)이다. 화면 y 는 아래로 증가하므로
    부호를 한 번 뒤집는다.
    """
    m = np.asarray(mask, dtype=bool)
    if m.size == 0 or not m.any():
        return (None, "empty")

    # 밴드에는 물체 말고도 노이즈 조각이 걸린다. 전체 픽셀로 주축을 뽑으면
    # 떨어진 조각 두 개를 잇는 방향이 나온다.
    n_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
        m.astype(np.uint8), connectivity=8)
    if n_labels <= 1:
        return (None, "empty")
    areas = stats[1:, cv2.CC_STAT_AREA]      # 0 번은 배경이다
    biggest = 1 + int(np.argmax(areas))
    ys, xs = np.nonzero(labels == biggest)

    if xs.size < min_pixels:
        return (None, f"small({xs.size})")

    x = xs.astype(np.float64)
    y = ys.astype(np.float64)
    x -= x.mean()
    y -= y.mean()
    # 2차 모멘트. minAreaRect 가 아니라 이걸 쓰는 이유: minAreaRect 는
    # 볼록 껍질에 맞춘 사각형이라 튀어나온 픽셀 몇 개에 각도가 휘둘린다.
    # 게다가 게이트로 쓸 이심률이 이 계산에서 덤으로 나온다.
    cov = np.array([[(x * x).mean(), (x * y).mean()],
                    [(x * y).mean(), (y * y).mean()]])
    values, vectors = np.linalg.eigh(cov)    # eigh 는 오름차순으로 준다
    lam_minor, lam_major = float(values[0]), float(values[1])
    if lam_major <= 0.0:
        return (None, "degenerate")

    elongation = (math.sqrt(lam_major / lam_minor)
                  if lam_minor > 1e-12 else float("inf"))
    if elongation < min_elongation:
        return (None, f"round({elongation:.1f})")

    vx, vy = float(vectors[0, 1]), float(vectors[1, 1])
    angle = math.degrees(math.atan2(-vy, vx))
    return (wrap90(angle), elongation)


from pathlib import Path

# 손 정적 마스크. 비트맵이라 roi.json 에 못 넣어서 옆에 따로 둔다.
HAND_MASK_NPY = Path(__file__).with_name("hand_mask.npy")


def save_hand_mask(mask, path=HAND_MASK_NPY):
    """지금 밴드 마스크를 '손'으로 저장한다. 물체를 치운 채로 불러야 한다."""
    np.save(str(path), np.asarray(mask, dtype=bool))


def load_hand_mask(shape, path=HAND_MASK_NPY):
    """저장된 손 마스크. 없거나 모양이 다르면 None.

    모양이 다른 것은 ROI 를 다시 드래그했다는 뜻이다. 조용히 쓰면
    broadcasting 으로 엉뚱한 데가 지워지거나 예외로 죽는다. 안 쓰고
    경고만 한다 -- 사람이 h 를 다시 눌러야 한다는 걸 알아야 한다.
    """
    p = Path(path)
    if not p.exists():
        return None
    stored = np.load(str(p))
    if stored.shape != tuple(shape):
        print(f"[WARN] 손 마스크 모양이 다릅니다 {stored.shape} != "
              f"{tuple(shape)}. ROI 를 바꿨다면 h 로 다시 찍으세요. "
              f"이번 실행에서는 쓰지 않습니다.")
        return None
    return stored.astype(bool)


def subtract_hand(mask, hand_mask):
    """마스크에서 손 픽셀을 뺀 **새 배열**.

    제자리에서 지우지 않는다. 호출부가 같은 mask 를 화면에도 그리는데,
    제자리에서 지우면 초록 픽셀 표시에서 손이 사라져 무엇이 빠졌는지
    눈으로 확인할 수 없게 된다.
    """
    m = np.asarray(mask, dtype=bool)
    if hand_mask is None:
        return m
    hand = np.asarray(hand_mask, dtype=bool)
    if hand.shape != m.shape:
        return m
    return m & ~hand
