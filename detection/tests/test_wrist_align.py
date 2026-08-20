# -*- coding: utf-8 -*-
"""WristAligner 의 순수 로직. STS3215 도 카메라도 필요 없다."""

import math

import pytest

from wrist_align import (ALIGNED, ALIGNING, NO_ANGLE, TIMEOUT, UNREACHABLE,
                         WristAligner)


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


def make(target=0.0, clock=None, **over):
    kwargs = dict(
        tol_deg=5.0,
        gain=0.5,
        da_max_rad=math.radians(3.0),
        min_rad=math.radians(-30.0),
        max_rad=math.radians(30.0),
        direction_sign=+1,
        stable_frames=3,
        pinned_frames=4,
        timeout_s=10.0,
    )
    kwargs.update(over)
    aligner = WristAligner(target, clock=clock or FakeClock(), **kwargs)
    aligner.start(0.0)
    return aligner


# --- 데드밴드와 안정 판정 ----------------------------------------------


def test_inside_the_deadband_the_wrist_does_not_move():
    a = make(target=0.0)
    goal, status = a.update(3.0)       # tol 5도 안
    assert goal == pytest.approx(0.0)
    assert status == ALIGNING          # 아직 stable_frames 를 못 채웠다


def test_aligned_needs_consecutive_frames():
    """한 프레임 맞은 값으로 파지에 들어가면 안 된다.

    RatioTrigger 가 연속 프레임을 요구하는 것과 같은 이유다.
    """
    a = make(target=0.0, stable_frames=3)
    assert a.update(1.0)[1] == ALIGNING
    assert a.update(1.0)[1] == ALIGNING
    assert a.update(1.0)[1] == ALIGNED


def test_a_miss_resets_the_streak():
    a = make(target=0.0, stable_frames=3)
    a.update(1.0)
    a.update(1.0)
    a.update(40.0)                     # 크게 틀어졌다
    assert a.update(1.0)[1] == ALIGNING
    assert a.update(1.0)[1] == ALIGNING
    assert a.update(1.0)[1] == ALIGNED


def test_a_flipped_object_counts_as_aligned():
    """장축은 방향이 없다.

    목표 0도일 때 178도는 -2도와 같은 자세다. wrap90 이 없으면 손목이
    178도를 지우려고 가동범위 끝까지 돈다.
    """
    a = make(target=0.0, stable_frames=1)
    assert a.update(178.0)[1] == ALIGNED


# --- 이동 ---------------------------------------------------------------


def test_the_step_is_capped_per_cycle():
    """한 사이클에 크게 움직이면 손목 위 카메라가 흔들려 다음 측정이 깨진다."""
    a = make(target=0.0, da_max_rad=math.radians(3.0))
    goal, _ = a.update(80.0)           # gain 0.5 -> 40도를 원한다
    assert goal == pytest.approx(math.radians(3.0))


def test_it_converges_on_a_fake_plant():
    """eye-in-hand: 손목을 +Δ 돌리면 화면 속 각도가 -Δ 만큼 바뀐다."""
    a = make(target=0.0, stable_frames=1, timeout_s=1e9)
    object_world_deg = 20.0
    status = ALIGNING
    for _ in range(200):
        measured = object_world_deg - math.degrees(a.goal_rad)
        _, status = a.update(measured)
        if status == ALIGNED:
            break
    assert status == ALIGNED
    assert math.degrees(a.goal_rad) == pytest.approx(20.0, abs=5.0)


# --- 실패 두 가지를 구분한다 --------------------------------------------


def test_out_of_range_is_unreachable_not_timeout():
    """병이 크게 틀어져 있으면 손목 가동범위로는 못 맞춘다.

    타임아웃과 원인이 달라서 화면에서 구분돼야 한다. 이쪽은 카메라를
    돌려 놓거나 병을 돌려 놔야 하고, 타임아웃은 게인이나 노이즈 문제다.
    """
    a = make(target=0.0, max_rad=math.radians(10.0), pinned_frames=3,
             timeout_s=1e9)
    status = ALIGNING
    for _ in range(50):
        _, status = a.update(80.0)     # 영영 못 지우는 오차
        if status == UNREACHABLE:
            break
    assert status == UNREACHABLE
    assert a.goal_rad == pytest.approx(math.radians(10.0))


def test_a_wrong_direction_sign_stops_at_the_limit():
    """부호가 반대면 손목이 오차를 키우며 한계까지 간다.

    발산해서 영원히 도는 게 아니라 UNREACHABLE 로 끝나야 한다. 손목 위에
    카메라가 실려 있다.
    """
    a = make(target=0.0, direction_sign=-1, pinned_frames=3, timeout_s=1e9)
    status = ALIGNING
    for _ in range(200):
        measured = 20.0 - math.degrees(a.goal_rad)
        _, status = a.update(measured)
        if status == UNREACHABLE:
            break
    assert status == UNREACHABLE


def test_timeout_when_it_never_settles():
    clock = FakeClock()
    a = make(target=0.0, clock=clock, timeout_s=1.0, pinned_frames=1000)
    a.update(30.0)
    clock.advance(1.5)
    assert a.update(30.0)[1] == TIMEOUT


# --- 각도를 모를 때 -----------------------------------------------------


def test_unknown_angle_freezes_the_wrist():
    """모르면 움직이지 않는다.

    band_ratio 가 None 일 때 트리거를 내리는 것과 같은 방침이다.
    """
    a = make(target=0.0, stable_frames=2)
    a.update(1.0)
    goal, status = a.update(None)
    assert goal == pytest.approx(0.0)
    assert status == NO_ANGLE
    # 연속 카운트도 끊겨야 한다. 안 끊으면 못 보는 사이에 ALIGNED 가 된다.
    assert a.update(1.0)[1] == ALIGNING


def test_start_resets_the_counters():
    clock = FakeClock()
    a = make(target=0.0, clock=clock, timeout_s=1.0, pinned_frames=1000)
    a.update(30.0)
    clock.advance(1.5)
    a.start(0.0)
    assert a.update(30.0)[1] == ALIGNING     # 타이머가 다시 0 부터


# --- 유지 허용치(히스테리시스) ------------------------------------------
#
# 정렬은 빡세게(tol_deg), 유지는 느슨하게(hold_tol_deg). RatioTrigger 의
# ENTER/EXIT 와 같은 구조다 -- 하나로 쓰면 확인 창 동안 노이즈 한 번에
# ALIGNED 가 풀려서 타이머가 계속 리셋된다.


def test_hold_tol_은_기본적으로_tol_과_같다():
    # 값을 안 주는 기존 호출부의 동작이 안 바뀌어야 한다.
    a = make(tol_deg=5.0)
    assert a.hold_tol_deg == pytest.approx(5.0)


def test_hold_tol_이_tol_보다_작으면_예외():
    # 뒤집히면 유지가 정렬보다 빡세져서 ALIGNED 가 되자마자 풀린다.
    with pytest.raises(ValueError, match="hold_tol_deg"):
        make(tol_deg=5.0, hold_tol_deg=1.0)


def test_ALIGNED_가_되려면_좁은_쪽을_통과해야_한다():
    # tol 과 hold_tol 사이 값으로는 ALIGNED 가 되면 안 된다.
    a = make(target=0.0, tol_deg=1.5, hold_tol_deg=3.0, stable_frames=3)
    for _ in range(10):
        _, status = a.update(2.0)          # 1.5 < 2.0 < 3.0
    assert status == ALIGNING


def test_ALIGNED_뒤에는_넓은_쪽까지_버틴다():
    a = make(target=0.0, tol_deg=1.5, hold_tol_deg=3.0, stable_frames=3)
    for _ in range(3):
        _, status = a.update(0.5)
    assert status == ALIGNED
    _, status = a.update(2.5)              # tol 밖, hold_tol 안
    assert status == ALIGNED


def test_넓은_쪽도_넘으면_다시_정렬한다():
    a = make(target=0.0, tol_deg=1.5, hold_tol_deg=3.0, stable_frames=3)
    for _ in range(3):
        a.update(0.5)
    _, status = a.update(4.0)              # hold_tol 밖
    assert status == ALIGNING


def test_유지_구간에서는_손목이_안_움직인다():
    # 데드밴드라 goal 이 그대로여야 한다. 움직이면 카메라가 흔들려
    # 다음 프레임 각도 측정이 깨진다.
    a = make(target=0.0, tol_deg=1.5, hold_tol_deg=3.0, stable_frames=3)
    for _ in range(3):
        a.update(0.5)
    goal_before = a.goal_rad
    goal, status = a.update(2.5)
    assert status == ALIGNED
    assert goal == pytest.approx(goal_before)


def test_다시_정렬로_떨어지면_좁은_쪽이_적용된다():
    a = make(target=0.0, tol_deg=1.5, hold_tol_deg=3.0, stable_frames=3)
    for _ in range(3):
        a.update(0.5)
    a.update(4.0)                          # ALIGNING 으로 떨어짐
    for _ in range(10):
        _, status = a.update(2.0)          # 사이 값으로는 못 돌아온다
    assert status == ALIGNING


def test_start_하면_다시_좁은_쪽부터다():
    a = make(target=0.0, tol_deg=1.5, hold_tol_deg=3.0, stable_frames=3)
    for _ in range(3):
        a.update(0.5)
    assert a.status == ALIGNED
    a.start(0.0)
    for _ in range(10):
        _, status = a.update(2.0)
    assert status == ALIGNING


def test_각도를_놓치면_유지_구간을_잃는다():
    # NO_ANGLE 은 _stable 을 지운다. 못 보는 사이에 ALIGNED 로 남아
    # 있으면 그대로 파지에 들어간다.
    a = make(target=0.0, tol_deg=1.5, hold_tol_deg=3.0, stable_frames=3)
    for _ in range(3):
        a.update(0.5)
    a.update(None)
    _, status = a.update(2.5)
    assert status == ALIGNING
