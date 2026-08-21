# -*- coding: utf-8 -*-
"""roi_config / roi_judge / grasp_state 의 순수 로직. 카메라도 모터도 필요 없다."""

import numpy as np
import pytest

from roi_judge import band_ratio, roi_median


# --- band_ratio --------------------------------------------------------


def test_band_ratio_counts_only_pixels_inside_the_band():
    # 0.18/0.19 는 밴드 안, 0.42 는 손 뒤 배경.
    depth = np.array([[0.18, 0.19, 0.42, 0.42]], dtype=np.float32)
    ratio, valid = band_ratio(depth, 0.15, 0.25)
    assert ratio == pytest.approx(0.5)
    assert valid == pytest.approx(1.0)


def test_band_ratio_excludes_invalid_pixels_from_the_denominator():
    """0 은 '가까움'이 아니라 '모름'이다.

    분모에 넣으면 물체가 ROI 를 꽉 채워도 비율이 안 올라가서, 센서가
    안 보이는 상황이 '물체 없음'으로 둔갑한다.
    """
    depth = np.array([[0.18, 0.18, 0.0, 0.0]], dtype=np.float32)
    ratio, valid = band_ratio(depth, 0.15, 0.25)
    assert ratio == pytest.approx(1.0)      # 유효 2개 중 2개
    assert valid == pytest.approx(0.5)      # 전체 4개 중 2개만 유효


def test_band_ratio_none_when_nothing_is_measurable():
    # 유효 픽셀이 하나도 없으면 '모른다'. 모르면 잡지 않는다.
    depth = np.zeros((3, 3), dtype=np.float32)
    assert band_ratio(depth, 0.15, 0.25) == (None, 0.0)


def test_band_ratio_none_for_an_empty_roi():
    # 드래그가 한 점에서 끝나면 0 크기 ROI 가 나온다. 죽으면 안 된다.
    assert band_ratio(np.empty((0, 0), dtype=np.float32), 0.15, 0.25) == (None, 0.0)


def test_band_ratio_includes_both_boundaries():
    # 경계를 여닫는 규칙이 애매하면 캘리브레이션 값이 경계에 걸렸을 때
    # 원인 모를 히스테리시스가 생긴다. 양끝 포함으로 고정한다.
    depth = np.array([[0.15, 0.25]], dtype=np.float32)
    ratio, _ = band_ratio(depth, 0.15, 0.25)
    assert ratio == pytest.approx(1.0)


# --- roi_median --------------------------------------------------------


def test_roi_median_ignores_invalid_pixels():
    depth = np.array([[0.0, 0.20, 0.22, 0.24]], dtype=np.float32)
    assert roi_median(depth) == pytest.approx(0.22)


def test_roi_median_none_when_all_invalid():
    assert roi_median(np.zeros((2, 2), dtype=np.float32)) is None


from roi_judge import RatioTrigger


# --- RatioTrigger ------------------------------------------------------


def test_trigger_needs_consecutive_frames():
    """한 프레임 튄 값으로 손이 닫히면 안 된다."""
    t = RatioTrigger(enter_ratio=0.3, exit_ratio=0.15, enter_frames=3)
    assert t.update(0.5) is False
    assert t.update(0.5) is False
    assert t.update(0.5) is True


def test_trigger_resets_the_streak_on_a_miss():
    t = RatioTrigger(enter_ratio=0.3, exit_ratio=0.15, enter_frames=3)
    t.update(0.5)
    t.update(0.5)
    t.update(0.1)          # 끊겼다
    assert t.update(0.5) is False
    assert t.update(0.5) is False
    assert t.update(0.5) is True


def test_trigger_holds_between_exit_and_enter():
    """히스테리시스. 경계에서 깜빡이면 손이 열었다 닫았다 한다."""
    t = RatioTrigger(enter_ratio=0.3, exit_ratio=0.15, enter_frames=1)
    assert t.update(0.5) is True
    assert t.update(0.2) is True       # enter 아래지만 exit 위 -> 유지
    assert t.update(0.1) is False      # exit 아래 -> 내려간다


def test_trigger_drops_immediately_when_ratio_is_unknown():
    # None 은 센서가 못 본다는 뜻이다. 모르면 잡지 않는다.
    t = RatioTrigger(enter_ratio=0.3, exit_ratio=0.15, enter_frames=1)
    assert t.update(0.5) is True
    assert t.update(None) is False


def test_trigger_rejects_inverted_thresholds():
    # exit > enter 면 히스테리시스가 뒤집혀서 오히려 더 깜빡인다.
    # 입구에서 막는다.
    with pytest.raises(ValueError):
        RatioTrigger(enter_ratio=0.3, exit_ratio=0.5, enter_frames=1)


import json

from roi_config import RoiConfig


# --- RoiConfig ---------------------------------------------------------


def test_roi_slices_the_depth_image():
    depth = np.arange(100, dtype=np.float32).reshape(10, 10)
    roi = RoiConfig(x=2, y=1, w=3, h=2, near_m=0.20, far_m=0.26)
    # y 가 행, x 가 열이다. 뒤집히면 ROI 가 엉뚱한 데를 본다.
    assert roi.slice(depth).tolist() == [[12.0, 13.0, 14.0],
                                         [22.0, 23.0, 24.0]]


def test_roi_round_trips_through_json(tmp_path):
    path = tmp_path / "roi.json"
    RoiConfig(x=10, y=20, w=30, h=40, near_m=0.19, far_m=0.27).save(path)
    loaded = RoiConfig.load(path)
    assert (loaded.x, loaded.y, loaded.w, loaded.h) == (10, 20, 30, 40)
    assert loaded.near_m == pytest.approx(0.19)
    assert loaded.far_m == pytest.approx(0.27)


def test_roi_load_returns_none_when_missing(tmp_path):
    # 첫 실행에는 파일이 없다. 그때는 기본값으로 시작해야 한다.
    assert RoiConfig.load(tmp_path / "nope.json") is None


def test_roi_load_raises_on_a_broken_file(tmp_path):
    """조용히 기본값으로 넘어가면 '왜 어제 맞춘 ROI 가 다르지'를 못 찾는다."""
    path = tmp_path / "roi.json"
    path.write_text(json.dumps({"x": 1, "y": 2}), encoding="utf-8")
    with pytest.raises(ValueError):
        RoiConfig.load(path)


def test_roi_rejects_zero_size():
    # 드래그가 한 점에서 끝나면 이렇게 된다. 저장 전에 막는다.
    with pytest.raises(ValueError):
        RoiConfig(x=0, y=0, w=0, h=10, near_m=0.20, far_m=0.26).validate()


def test_roi_rejects_inverted_band():
    with pytest.raises(ValueError):
        RoiConfig(x=0, y=0, w=10, h=10, near_m=0.30, far_m=0.26).validate()


from grasp_state import (ARMED, GRASPING, HOLDING, RELEASING, GraspStateMachine,
                        SequenceExecutor)


class FakeSeq:
    """PoseSequencer 대역. 몇 번 tick 해야 끝나는지를 흉내낸다."""

    def __init__(self, ticks_to_finish=2):
        self.ticks_to_finish = ticks_to_finish
        self.starts = []          # [(a, s), ...]
        self._left = 0

    def start(self, a, s=0.0):
        self.starts.append((a, s))
        self._left = self.ticks_to_finish

    def tick(self):
        if self._left <= 0:
            return False
        self._left -= 1
        return self._left > 0


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


HIT = 0.9      # 물체 있음으로 판정되는 비율
MISS = 0.0     # 물체 없음


def make_machine(ticks=2, settle=1.0, rearm=0.0, enter_frames=1):
    seq, clock = FakeSeq(ticks), FakeClock()
    trigger = RatioTrigger(enter_ratio=0.3, exit_ratio=0.15,
                           enter_frames=enter_frames)
    executor = SequenceExecutor(seq, grasp_a=0.8, settle_s=settle,
                               clock=clock)
    # 정렬기가 없는 기계다(--no-wrist 경로). 정렬은 건너뛰지만 3초 확인
    # 창은 그대로 있으므로, 창 길이를 0 으로 두어 상태 전이만 본다.
    machine = GraspStateMachine(executor, trigger, aligner=None, clock=clock,
                                rearm_s=rearm, confirm_hold_s=0.0)
    return machine, seq, clock


def grasp_from_armed(machine, ratio=HIT):
    """ARMED 에서 GRASPING 까지 민다. -> 마지막 상태.

    정렬기가 없어도 트리거는 CONFIRMING 을 한 번 거친다. 창 길이가 0 이라
    다음 프레임에 곧바로 GRASPING 이 된다.
    """
    machine.update(ratio)               # -> CONFIRMING
    return machine.update(ratio)        # -> GRASPING


# --- 상태 전이 ---------------------------------------------------------


def test_machine_starts_armed():
    m, _, _ = make_machine()
    assert m.state == ARMED


def test_object_starts_a_grasp():
    m, seq, _ = make_machine()
    assert m.update(HIT) == CONFIRMING   # 트리거는 확인 창부터 연다
    assert seq.starts == []              # 창이 끝나기 전에는 안 움직인다
    assert m.update(HIT) == GRASPING
    assert seq.starts == [(0.8, 0.0)]


def test_no_object_stays_armed():
    m, seq, _ = make_machine()
    assert m.update(MISS) == ARMED
    assert seq.starts == []


def test_unknown_ratio_does_not_grasp():
    # None 은 센서가 못 본다는 뜻이다. 모르면 잡지 않는다.
    m, seq, _ = make_machine()
    assert m.update(None) == ARMED
    assert seq.starts == []


def test_grasping_ignores_the_object_signal():
    """파지가 시작되면 손가락이 ROI 를 침범하고 물체도 가려진다.

    여기서 판정을 계속하면 '사라짐 -> 폄 -> 보임 -> 잡음'이 무한 반복된다.
    """
    m, seq, _ = make_machine(ticks=3)
    grasp_from_armed(m)
    m.update(MISS)                      # 손가락에 가려 안 보임
    assert m.state == GRASPING
    assert len(seq.starts) == 1         # 다시 start 하지 않았다


def test_grasping_becomes_holding_when_the_sequence_ends():
    # settle=0.0 은 '도착 대기 없음'이다. 대기가 있으면 시퀀스가 끝나도
    # 실행기가 아직 busy 를 주므로(아래 테스트) 여기서는 0 으로 둔다.
    m, seq, _ = make_machine(ticks=2, settle=0.0)
    grasp_from_armed(m)                 # ARMED -> CONFIRMING -> GRASPING
    m.update(HIT)                       # tick 1
    assert m.update(HIT) == HOLDING     # tick 2 에서 끝


def test_release_is_refused_until_the_motors_settle():
    """tick() 이 False 인 시점은 '마지막 목표를 보낸 때'지 도착한 때가 아니다.

    도착 판정이 상태기계에서 실행기로 옮겨 갔다. 이동 중에는 상태가
    GRASPING 에 머무르므로, r 키가 거부되는 결과는 예전과 같다.
    """
    m, _, clock = make_machine(ticks=1, settle=1.0)
    grasp_from_armed(m)
    m.update(HIT)
    assert m.state == GRASPING               # 아직 이동 중
    assert m.request_release() is False
    clock.advance(1.0)
    assert m.update(HIT) == HOLDING
    assert m.request_release() is True


def test_release_returns_to_armed():
    m, seq, _ = make_machine(ticks=1, settle=0.0)
    grasp_from_armed(m)
    m.update(HIT)
    assert m.state == HOLDING
    m.request_release()
    assert m.state == RELEASING
    assert seq.starts[-1] == (0.0, 0.0)
    m.update(MISS)
    assert m.state == ARMED


def test_full_cycle():
    m, seq, _ = make_machine(ticks=1, settle=0.0)
    for _ in range(2):
        grasp_from_armed(m)
        m.update(HIT)
        assert m.state == HOLDING
        assert m.request_release() is True
        m.update(MISS)
        assert m.state == ARMED
    assert seq.starts == [(0.8, 0.0), (0.0, 0.0), (0.8, 0.0), (0.0, 0.0)]


def test_emergency_open_works_from_any_state():
    m, seq, _ = make_machine(ticks=5)
    grasp_from_armed(m)
    assert m.state == GRASPING
    m.emergency_open()
    assert m.state == RELEASING
    assert seq.starts[-1] == (0.0, 0.0)


def test_emergency_open_does_not_restart_an_open_in_progress():
    """space 를 누르고 있으면 매 프레임 호출된다.

    그때마다 seq.start() 를 다시 부르면 PoseSequencer 의 경과 시간이
    리셋되어, 지연을 받은 손가락(폄에서는 finger1~4)이 영영 출발하지
    못한다. 엄지만 펴지고 나머지는 키를 뗄 때까지 그대로다.
    """
    m, seq, _ = make_machine(ticks=5)
    m.update(HIT)
    m.emergency_open()
    m.emergency_open()
    m.emergency_open()
    assert seq.starts.count((0.0, 0.0)) == 1


def test_release_is_refused_when_not_holding():
    m, _, _ = make_machine()
    assert m.request_release() is False      # ARMED 에서는 놓을 게 없다


# --- 재무장 (놓은 직후) -------------------------------------------------


def test_rearm_requires_a_fresh_streak():
    """놓고 나서 다시 잡으려면 연속 프레임 조건을 처음부터 다시 채워야 한다.

    트리거를 초기화하지 않으면 active 가 True 로 남아, ARMED 로 돌아온
    첫 프레임에 exit_ratio(0.15)만 넘겨도 즉시 재파지된다. 사람이 물체를
    치울 틈이 없다.
    """
    m, seq, _ = make_machine(ticks=1, settle=0.0, rearm=0.0, enter_frames=3)
    for _ in range(3):
        m.update(HIT)                 # 3프레임째에 트리거 -> CONFIRMING
    assert m.state == CONFIRMING
    m.update(HIT)                     # -> GRASPING
    assert m.state == GRASPING
    m.update(HIT)                     # -> HOLDING
    assert m.state == HOLDING
    m.request_release()
    m.update(HIT)                     # -> ARMED (물체는 아직 그대로)
    assert m.state == ARMED

    # 여기서 즉시 잡히면 안 된다. 3프레임을 다시 채워야 한다.
    assert m.update(HIT) == ARMED
    assert m.update(HIT) == ARMED
    assert m.update(HIT) == CONFIRMING


def test_rearm_waits_out_the_cooldown():
    """폄이 끝나도 rearm_s 동안은 판정하지 않는다.

    seq.tick() 이 False 인 시점은 마지막 목표를 '보낸' 때지 서보가 도착한
    때가 아니다. 손이 실제로 벌어지고 사람이 물체를 치울 시간을 준다.
    """
    m, _, clock = make_machine(ticks=1, settle=0.0, rearm=2.0, enter_frames=1)
    grasp_from_armed(m)
    m.update(HIT)                     # -> HOLDING
    m.request_release()
    m.update(HIT)
    assert m.state == ARMED

    for _ in range(10):               # 쿨다운 중에는 물체가 있어도 무시
        assert m.update(HIT) == ARMED
    clock.advance(2.0)
    assert grasp_from_armed(m) == GRASPING


def test_rearm_remaining_counts_down():
    # 화면 표시용. 쿨다운 중에도 상태는 ARMED 라 이게 없으면 멈춘 것처럼 보인다.
    m, _, clock = make_machine(ticks=1, settle=0.0, rearm=2.0)
    assert m.rearm_remaining() == pytest.approx(0.0)
    grasp_from_armed(m)
    m.update(HIT)                     # -> HOLDING
    m.request_release()
    m.update(HIT)                     # -> ARMED, 쿨다운 시작
    assert m.rearm_remaining() == pytest.approx(2.0)
    clock.advance(1.5)
    assert m.rearm_remaining() == pytest.approx(0.5)
    clock.advance(1.0)                # 지나도 음수가 되면 안 된다
    assert m.rearm_remaining() == pytest.approx(0.0)


from roi_config import NUDGE_M, nudge_band


# --- nudge_band --------------------------------------------------------


def make_roi(near=0.20, far=0.30):
    return RoiConfig(x=0, y=0, w=10, h=10, near_m=near, far_m=far)


def test_nudge_moves_near_both_ways():
    roi = make_roi()
    assert nudge_band(roi, ord("[")) is True
    assert roi.near_m == pytest.approx(0.20 - NUDGE_M)
    assert nudge_band(roi, ord("]")) is True
    assert roi.near_m == pytest.approx(0.20)


def test_nudge_moves_far_both_ways():
    roi = make_roi()
    assert nudge_band(roi, ord("-")) is True
    assert roi.far_m == pytest.approx(0.30 - NUDGE_M)
    assert nudge_band(roi, ord("=")) is True
    assert roi.far_m == pytest.approx(0.30)


def test_nudge_ignores_other_keys():
    roi = make_roi()
    assert nudge_band(roi, ord("q")) is False
    assert (roi.near_m, roi.far_m) == (0.20, 0.30)


def test_nudge_refuses_to_invert_the_band():
    """near >= far 가 되면 그 뒤 판정이 조용히 0 이 된다. 화면에서 원인을
    찾을 수 없으므로 조작 자체를 막는다."""
    roi = make_roi(near=0.20, far=0.20 + NUDGE_M)
    assert nudge_band(roi, ord("]")) is True     # 처리는 했다
    assert roi.near_m == pytest.approx(0.20)     # 값은 안 변했다
    assert roi.far_m == pytest.approx(0.20 + NUDGE_M)


# --- RoiConfig.target_angle_deg -----------------------------------------------


def test_roi_defaults_the_target_angle():
    roi = RoiConfig(x=0, y=0, w=10, h=10, near_m=0.20, far_m=0.26)
    assert roi.target_angle_deg == pytest.approx(0.0)


def test_roi_round_trips_the_target_angle(tmp_path):
    path = tmp_path / "roi.json"
    RoiConfig(x=1, y=2, w=3, h=4, near_m=0.19, far_m=0.27,
              target_angle_deg=-37.5).save(path)
    assert RoiConfig.load(path).target_angle_deg == pytest.approx(-37.5)


def test_roi_loads_old_files_without_the_key(tmp_path):
    """어제 만든 roi.json 이 그대로 읽혀야 한다.

    조용히 실패하면 밴드 캘리브레이션까지 통째로 다시 해야 한다.
    """
    path = tmp_path / "roi.json"
    path.write_text(json.dumps({"x": 1, "y": 2, "w": 3, "h": 4,
                                "near_m": 0.19, "far_m": 0.27}),
                    encoding="utf-8")
    roi = RoiConfig.load(path)
    assert roi.target_angle_deg == pytest.approx(0.0)


def test_roi_rejects_a_target_angle_outside_the_wrap90_range():
    """wrap90 규약은 (-90, 90] 이다.

    밖의 값이 파일에 들어가면 오차 계산이 조용히 어긋난다.
    """
    with pytest.raises(ValueError):
        RoiConfig(x=0, y=0, w=10, h=10, near_m=0.20, far_m=0.26,
                  target_angle_deg=120.0).validate()
    with pytest.raises(ValueError):
        RoiConfig(x=0, y=0, w=10, h=10, near_m=0.20, far_m=0.26,
                  target_angle_deg=-90.0).validate()


def test_roi_accepts_exactly_plus_90():
    RoiConfig(x=0, y=0, w=10, h=10, near_m=0.20, far_m=0.26,
              target_angle_deg=90.0).validate()


# --- SequenceExecutor --------------------------------------------------


def test_sequence_executor_reports_settled_after_the_motors_arrive():
    """tick() 이 False 인 건 '마지막 목표를 보낸' 시점이지 서보가 도착한
    때가 아니다. 도착까지 기다렸다가 settled 를 알린다.
    """
    seq, clock = FakeSeq(ticks_to_finish=2), FakeClock()
    ex = SequenceExecutor(seq, grasp_a=0.8, settle_s=1.0, clock=clock)
    ex.start_grasp()
    assert seq.starts == [(0.8, 0.0)]
    assert ex.tick() == "busy"
    assert ex.tick() == "busy"       # 시퀀스는 끝났지만 서보는 가는 중
    clock.advance(1.5)
    assert ex.tick() == "settled"


def test_sequence_executor_opens_to_zero():
    seq, clock = FakeSeq(ticks_to_finish=1), FakeClock()
    ex = SequenceExecutor(seq, grasp_a=0.8, settle_s=0.0, clock=clock)
    ex.start_open()
    assert seq.starts == [(0.0, 0.0)]
    assert ex.tick() == "opened"


def test_sequence_executor_never_aborts():
    """고정 자세 시퀀스에는 힘도 온도도 없다. abort 를 낼 근거가 없다."""
    seq, clock = FakeSeq(ticks_to_finish=1), FakeClock()
    ex = SequenceExecutor(seq, grasp_a=0.8, settle_s=0.0, clock=clock)
    ex.start_grasp()
    for _ in range(10):
        assert ex.tick() != "abort"


import math

from grasp_state import ALIGNING, CONFIRMING
from wrist_align import WristAligner


def make_aligner(clock, target=0.0, **over):
    kwargs = dict(
        tol_deg=5.0, gain=0.5, da_max_rad=math.radians(3.0),
        min_rad=math.radians(-30.0), max_rad=math.radians(30.0),
        direction_sign=+1, stable_frames=1, pinned_frames=3,
        timeout_s=10.0,
    )
    kwargs.update(over)
    a = WristAligner(target, clock=clock, **kwargs)
    a.start(0.0)
    return a


def make_aligning_machine(rearm=0.0, confirm=3.0, **align):
    seq, clock = FakeSeq(1), FakeClock()
    trigger = RatioTrigger(enter_ratio=0.3, exit_ratio=0.15, enter_frames=1)
    executor = SequenceExecutor(seq, grasp_a=0.8, settle_s=0.0, clock=clock)
    machine = GraspStateMachine(
        executor, trigger, aligner=make_aligner(clock, **align),
        clock=clock, rearm_s=rearm, confirm_hold_s=confirm)
    return machine, seq, clock


# --- ALIGNING ----------------------------------------------------------


def test_trigger_goes_to_aligning_not_grasping():
    machine, seq, _ = make_aligning_machine()
    assert machine.update(HIT, angle_deg=40.0) == ALIGNING
    assert seq.starts == []           # 아직 손은 안 움직인다


def test_an_unknown_angle_does_not_trigger():
    """모르면 잡지 않는다. band_ratio 가 None 일 때와 같은 방침이다."""
    machine, _, _ = make_aligning_machine()
    assert machine.update(HIT, angle_deg=None) == ARMED


def test_aligning_publishes_a_wrist_goal():
    machine, _, _ = make_aligning_machine()
    machine.update(HIT, angle_deg=40.0)
    machine.update(HIT, angle_deg=40.0)
    assert machine.wrist_goal_rad is not None
    assert machine.wrist_goal_rad > 0.0


def test_aligned_moves_to_confirming():
    machine, _, _ = make_aligning_machine()
    machine.update(HIT, angle_deg=40.0)
    assert machine.update(HIT, angle_deg=0.0) == CONFIRMING


def test_losing_the_object_during_aligning_returns_to_armed():
    machine, _, _ = make_aligning_machine()
    machine.update(HIT, angle_deg=40.0)
    assert machine.update(MISS, angle_deg=40.0) == ARMED


def test_an_unreachable_alignment_returns_to_armed():
    """가동범위 밖이면 못 맞춘다. 못 맞춘 채 잡는 경로는 없다."""
    machine, seq, _ = make_aligning_machine(
        max_rad=math.radians(2.0), pinned_frames=2)
    state = ALIGNING
    for _ in range(30):
        state = machine.update(HIT, angle_deg=80.0)
        if state == ARMED:
            break
    assert state == ARMED
    assert seq.starts == []


def test_an_alignment_timeout_returns_to_armed():
    machine, seq, clock = make_aligning_machine(timeout_s=1.0,
                                                pinned_frames=1000)
    machine.update(HIT, angle_deg=40.0)
    clock.advance(1.5)
    assert machine.update(HIT, angle_deg=40.0) == ARMED
    assert seq.starts == []


# --- CONFIRMING (3초 창) ------------------------------------------------


def test_confirming_needs_the_full_window():
    machine, seq, clock = make_aligning_machine(confirm=3.0)
    machine.update(HIT, angle_deg=40.0)
    machine.update(HIT, angle_deg=0.0)          # -> CONFIRMING
    clock.advance(2.9)
    assert machine.update(HIT, angle_deg=0.0) == CONFIRMING
    assert seq.starts == []
    clock.advance(0.2)
    assert machine.update(HIT, angle_deg=0.0) == GRASPING
    assert seq.starts == [(0.8, 0.0)]


def test_losing_the_object_resets_the_window():
    machine, _, clock = make_aligning_machine(confirm=3.0)
    machine.update(HIT, angle_deg=40.0)
    machine.update(HIT, angle_deg=0.0)
    clock.advance(2.5)
    assert machine.update(MISS, angle_deg=0.0) == ARMED


def test_a_drifting_angle_goes_back_to_aligning():
    """3초 안에 각도가 크게 틀어지면 다시 맞춘다.

    타이머만 리셋하고 제자리에 두면 손목이 안 움직여서 영영 안 잡힌다.
    """
    machine, _, clock = make_aligning_machine(confirm=3.0)
    machine.update(HIT, angle_deg=40.0)
    machine.update(HIT, angle_deg=0.0)
    clock.advance(2.0)
    assert machine.update(HIT, angle_deg=40.0) == ALIGNING


def test_the_window_restarts_after_realigning():
    """되돌아갔다 오면 3초를 처음부터 다시 센다."""
    machine, seq, clock = make_aligning_machine(confirm=3.0)
    machine.update(HIT, angle_deg=40.0)
    machine.update(HIT, angle_deg=0.0)
    clock.advance(2.9)
    machine.update(HIT, angle_deg=40.0)         # -> ALIGNING
    machine.update(HIT, angle_deg=0.0)          # -> CONFIRMING (다시)
    clock.advance(2.9)
    assert machine.update(HIT, angle_deg=0.0) == CONFIRMING
    assert seq.starts == []


def test_confirm_remaining_counts_down():
    machine, _, clock = make_aligning_machine(confirm=3.0)
    machine.update(HIT, angle_deg=40.0)
    machine.update(HIT, angle_deg=0.0)
    clock.advance(1.0)
    machine.update(HIT, angle_deg=0.0)
    assert machine.confirm_remaining() == pytest.approx(2.0, abs=0.01)


def test_confirm_remaining_is_zero_outside_the_window():
    machine, _, _ = make_aligning_machine()
    assert machine.confirm_remaining() == pytest.approx(0.0)


# --- 정렬 끄기 ----------------------------------------------------------


def test_align_off_skips_aligning():
    """손목이 아직 없거나 부호를 모를 때 나머지를 확인하는 경로다."""
    machine, seq, clock = make_aligning_machine(confirm=3.0)
    machine.set_align_enabled(False)
    assert machine.update(HIT, angle_deg=None) == CONFIRMING
    clock.advance(3.1)
    assert machine.update(HIT, angle_deg=None) == GRASPING
    assert seq.starts == [(0.8, 0.0)]


def test_align_off_does_not_publish_a_wrist_goal():
    machine, _, clock = make_aligning_machine()
    machine.set_align_enabled(False)
    machine.update(HIT, angle_deg=None)
    assert machine.wrist_goal_rad is None


def test_align_off_still_requires_the_full_window():
    """정렬만 끄는 것이지 3초 게이트까지 끄는 게 아니다."""
    machine, seq, clock = make_aligning_machine(confirm=3.0)
    machine.set_align_enabled(False)
    machine.update(HIT, angle_deg=None)
    clock.advance(2.9)
    assert machine.update(HIT, angle_deg=None) == CONFIRMING
    assert seq.starts == []


def test_turning_alignment_off_mid_align_falls_back_to_confirming():
    """정렬이 안 되는 걸 보고 그 자리에서 끌 수 있어야 한다."""
    machine, _, _ = make_aligning_machine()
    machine.update(HIT, angle_deg=40.0)         # -> ALIGNING
    machine.set_align_enabled(False)
    assert machine.update(HIT, angle_deg=40.0) == CONFIRMING


def test_alignment_cannot_be_enabled_without_an_aligner():
    """손목이 없는데(--no-wrist) 켜면 영영 CONFIRMING 에 못 간다."""
    seq, clock = FakeSeq(1), FakeClock()
    trigger = RatioTrigger(enter_ratio=0.3, exit_ratio=0.15, enter_frames=1)
    executor = SequenceExecutor(seq, grasp_a=0.8, settle_s=0.0, clock=clock)
    machine = GraspStateMachine(executor, trigger, aligner=None, clock=clock,
                                rearm_s=0.0, confirm_hold_s=0.0)
    machine.set_align_enabled(True)
    assert machine.align_enabled is False


# --- 비상 폄 ------------------------------------------------------------


def test_emergency_open_during_aligning_actually_opens():
    """비상 정지가 가장 필요한 순간에 조용히 아무것도 안 하면 안 된다."""
    machine, seq, _ = make_aligning_machine()
    machine.update(HIT, angle_deg=40.0)         # -> ALIGNING
    machine.emergency_open()
    assert machine.state == RELEASING
    assert seq.starts == [(0.0, 0.0)]


def test_emergency_open_during_confirming_actually_opens():
    machine, seq, _ = make_aligning_machine()
    machine.update(HIT, angle_deg=40.0)
    machine.update(HIT, angle_deg=0.0)          # -> CONFIRMING
    machine.emergency_open()
    assert machine.state == RELEASING
    assert seq.starts == [(0.0, 0.0)]


def test_emergency_open_clears_the_confirm_window():
    machine, _, _ = make_aligning_machine()
    machine.update(HIT, angle_deg=40.0)
    machine.update(HIT, angle_deg=0.0)
    machine.emergency_open()
    assert machine.confirm_remaining() == pytest.approx(0.0)


def test_emergency_open_is_idempotent_while_releasing():
    """키를 누르고 있으면 매 프레임 불린다.

    그때마다 start_open() 을 다시 부르면 경과 시간이 리셋되어 지연을 받은
    손가락이 영영 출발하지 못한다 -- 엄지만 펴지고 나머지 넷은 그대로다.
    """
    machine, seq, _ = make_aligning_machine()
    machine.update(HIT, angle_deg=40.0)
    machine.emergency_open()
    machine.emergency_open()
    machine.emergency_open()
    assert seq.starts == [(0.0, 0.0)]


from roi_config import nudge_target


# --- 목표 각도 미세조정 -------------------------------------------------


def test_nudge_target_moves_by_one_step():
    roi = RoiConfig(x=0, y=0, w=10, h=10, near_m=0.20, far_m=0.26,
                    target_angle_deg=10.0)
    assert nudge_target(roi, ord(".")) is True
    assert roi.target_angle_deg == pytest.approx(11.0)
    assert nudge_target(roi, ord(",")) is True
    assert roi.target_angle_deg == pytest.approx(10.0)


def test_nudge_target_wraps_at_the_boundary():
    """(-90, 90] 밖으로 나가면 validate() 가 저장을 막는다.

    감아 주지 않으면 90도 근처에서 . 을 한 번 눌렀을 때 조용히 저장이
    실패한다.
    """
    roi = RoiConfig(x=0, y=0, w=10, h=10, near_m=0.20, far_m=0.26,
                    target_angle_deg=90.0)
    nudge_target(roi, ord("."))
    roi.validate()                      # 여기서 안 터져야 한다
    assert -90.0 < roi.target_angle_deg <= 90.0


def test_nudge_target_ignores_other_keys():
    roi = RoiConfig(x=0, y=0, w=10, h=10, near_m=0.20, far_m=0.26)
    assert nudge_target(roi, ord("q")) is False



# --- 수동 파지 ---------------------------------------------------------
#
# ROI 트리거가 걸려야만 파지가 시작되면 반복 시험이 번거롭다. 사람이
# 직접 걸 수 있어야 파지 로직(강성 분류, 벌림 탐색)을 카메라 조건과
# 무관하게 시험할 수 있다.


def test_수동_파지는_ARMED에서_곧바로_GRASPING(self=None):
    m, seq, _ = make_machine()
    assert m.request_grasp() is True
    assert m.state == GRASPING
    assert seq.starts == [(0.8, 0.0)]


def test_수동_파지는_쿨다운을_무시한다():
    # 사람이 명시적으로 누른 것이다. 쿨다운은 "물체가 남아 있어서 자동
    # 재파지되는 것"을 막는 장치이고, 사람의 의도를 막을 이유는 없다.
    m, _, clock = make_machine(rearm=3.0)
    m.abort_to_armed()
    assert m.rearm_remaining() > 0
    assert m.request_grasp() is True
    assert m.state == GRASPING


def test_수동_파지는_무장_해제_중에도_된다():
    # disarm 은 "새 파지를 자동으로 시작하지 않는다"이다. 사람이 직접
    # 누르는 것은 화면을 보고 있다는 증거 자체다.
    m, _, _ = make_machine()
    m.disarm()
    assert m.request_grasp() is True
    assert m.state == GRASPING


def test_수동_파지는_정렬_중에도_가로챈다():
    # "정렬 됐든 안 됐든 지금 잡아라". 브링업에서 자주 쓰는 경로다.
    m, _, _ = make_machine()
    m.update(HIT)
    assert m.state == CONFIRMING
    assert m.request_grasp() is True
    assert m.state == GRASPING


def test_잡고_있을_때는_수동_파지를_거절한다():
    # 진행 중인 파지를 다시 시작하면 손가락이 반쯤 닫힌 채로 상태가
    # 초기화되어 물체에 끼인다.
    m, seq, _ = make_machine()
    grasp_from_armed(m)
    before = len(seq.starts)
    assert m.request_grasp() is False
    assert len(seq.starts) == before


def test_수동_파지_뒤_트리거가_비어_있다():
    # 안 지우면 파지가 끝나고 ARMED 로 돌아온 첫 프레임에 옛 연속
    # 프레임이 남아 곧바로 자동 재파지된다.
    m, _, _ = make_machine(enter_frames=2)
    m.update(HIT)                 # 연속 프레임 1 쌓임
    m.request_grasp()
    assert m.trigger.active is False
