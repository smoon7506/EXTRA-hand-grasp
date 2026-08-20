# -*- coding: utf-8 -*-
"""orientation 의 순수 로직. 카메라도 모터도 필요 없다."""

import numpy as np
import pytest

from orientation import principal_axis, wrap90, load_hand_mask, save_hand_mask, subtract_hand


# --- wrap90 ------------------------------------------------------------


def test_wrap90_keeps_values_already_in_range():
    assert wrap90(0.0) == pytest.approx(0.0)
    assert wrap90(45.0) == pytest.approx(45.0)
    assert wrap90(-45.0) == pytest.approx(-45.0)


def test_wrap90_folds_the_180_degree_period():
    """장축은 방향이 없다. 병을 180도 돌려도 같은 자세다."""
    assert wrap90(180.0) == pytest.approx(0.0)
    assert wrap90(100.0) == pytest.approx(-80.0)
    assert wrap90(-100.0) == pytest.approx(80.0)


def test_wrap90_puts_the_boundary_at_plus_90():
    """규약을 (-90, 90] 하나로 고정한다.

    ±90 은 같은 방향이라 어느 쪽으로 정해도 되지만, 정해 두지 않으면
    같은 자세가 프레임마다 +90 과 -90 을 오가며 오차 계산이 180도씩 튄다.
    """
    assert wrap90(90.0) == pytest.approx(90.0)
    assert wrap90(-90.0) == pytest.approx(90.0)
    assert wrap90(270.0) == pytest.approx(90.0)


# --- principal_axis ----------------------------------------------------


def _bar(shape, x0, x1, y0, y1):
    """직사각형 하나짜리 마스크."""
    mask = np.zeros(shape, dtype=bool)
    mask[y0:y1, x0:x1] = True
    return mask


def test_horizontal_bar_is_zero_degrees():
    mask = _bar((60, 60), x0=5, x1=55, y0=28, y1=32)
    angle, elong = principal_axis(mask, min_pixels=10, min_elongation=2.0)
    assert angle == pytest.approx(0.0, abs=1.0)
    assert elong > 2.0


def test_vertical_bar_is_ninety_degrees():
    mask = _bar((60, 60), x0=28, x1=32, y0=5, y1=55)
    angle, _ = principal_axis(mask, min_pixels=10, min_elongation=2.0)
    assert angle == pytest.approx(90.0, abs=1.0)


def test_a_backslash_diagonal_is_negative_forty_five():
    """화면 y 는 아래로 증가한다.

    x 와 y 가 같이 커지는 선은 화면에서 '\\' 모양이고, 사람이 읽는
    각도(반시계 +)로는 -45도다. 여기서 부호를 뒤집으면 손목이 정확히
    반대로 돈다.
    """
    mask = np.zeros((60, 60), dtype=bool)
    for i in range(5, 55):
        mask[i, i] = True
        mask[i, min(i + 1, 59)] = True
    angle, _ = principal_axis(mask, min_pixels=10, min_elongation=2.0)
    assert angle == pytest.approx(-45.0, abs=3.0)


def test_round_object_is_rejected():
    """둥근 물체는 장축이 없다.

    각도가 프레임마다 아무 값이나 나오는데, 그걸 믿고 손목을 돌리면
    영원히 흔들린다.
    """
    yy, xx = np.mgrid[0:60, 0:60]
    mask = (yy - 30) ** 2 + (xx - 30) ** 2 <= 15 ** 2
    angle, reason = principal_axis(mask, min_pixels=10, min_elongation=2.0)
    assert angle is None
    assert "round" in reason


def test_empty_mask_is_unknown():
    angle, reason = principal_axis(np.zeros((10, 10), dtype=bool),
                                  min_pixels=1, min_elongation=1.0)
    assert angle is None
    assert reason == "empty"


def test_zero_size_mask_is_unknown():
    # 드래그가 한 점에서 끝나면 0 크기 ROI 가 나온다. 죽으면 안 된다.
    angle, _ = principal_axis(np.zeros((0, 0), dtype=bool))
    assert angle is None


def test_too_few_pixels_is_unknown():
    """픽셀이 듬성듬성하면 주축이 통계적으로 흔들린다.

    현재 밴드가 D435i 의 Min-Z 아래라 실제로 자주 일어난다 (스펙 §2).
    """
    mask = _bar((60, 60), x0=5, x1=25, y0=30, y1=31)   # 20 픽셀
    angle, reason = principal_axis(mask, min_pixels=200, min_elongation=2.0)
    assert angle is None
    assert "small" in reason


def test_only_the_largest_component_is_used():
    """전체 픽셀로 주축을 뽑으면 떨어진 조각 두 개를 잇는 방향이 나온다."""
    mask = _bar((80, 80), x0=5, x1=70, y0=40, y1=44)    # 큰 가로 막대
    mask |= _bar((80, 80), x0=2, x1=5, y0=2, y1=20)     # 구석의 작은 세로 조각
    angle, _ = principal_axis(mask, min_pixels=10, min_elongation=2.0)
    assert angle == pytest.approx(0.0, abs=2.0)


# --- 손 마스크 --------------------------------------------------------


def test_hand_mask_round_trips(tmp_path):
    path = tmp_path / "hand_mask.npy"
    mask = np.zeros((6, 8), dtype=bool)
    mask[0:2, :] = True
    save_hand_mask(mask, path)
    loaded = load_hand_mask((6, 8), path)
    assert loaded is not None
    assert loaded.dtype == bool
    assert np.array_equal(loaded, mask)


def test_hand_mask_missing_file_is_none(tmp_path):
    # 아직 h 로 안 찍었다. 빼지 않고 그냥 간다 (기존 동작).
    assert load_hand_mask((6, 8), tmp_path / "nope.npy") is None


def test_hand_mask_shape_mismatch_is_rejected(tmp_path, capsys):
    """ROI 를 다시 드래그하면 마스크 모양이 달라진다.

    조용히 쓰면 broadcasting 으로 엉뚱한 데가 지워지거나 예외로 죽는다.
    안 쓰고 경고만 한다 -- 사람이 h 를 다시 눌러야 한다는 걸 알아야 한다.
    """
    path = tmp_path / "hand_mask.npy"
    save_hand_mask(np.ones((6, 8), dtype=bool), path)
    assert load_hand_mask((10, 10), path) is None
    out = capsys.readouterr().out
    assert "손 마스크" in out


def test_subtract_hand_removes_the_hand_pixels():
    mask = np.ones((4, 4), dtype=bool)
    hand = np.zeros((4, 4), dtype=bool)
    hand[0, :] = True
    result = subtract_hand(mask, hand)
    assert not result[0].any()
    assert result[1:].all()


def test_subtract_hand_is_a_noop_without_a_mask():
    mask = np.ones((4, 4), dtype=bool)
    assert np.array_equal(subtract_hand(mask, None), mask)


def test_subtract_hand_does_not_mutate_the_input():
    """호출부는 같은 프레임의 mask 를 화면에도 그린다.

    제자리에서 지우면 초록 픽셀 표시에서 손이 사라져, 무엇이 빠졌는지
    화면으로 확인할 수 없게 된다.
    """
    mask = np.ones((4, 4), dtype=bool)
    hand = np.zeros((4, 4), dtype=bool)
    hand[0, :] = True
    subtract_hand(mask, hand)
    assert mask.all()
