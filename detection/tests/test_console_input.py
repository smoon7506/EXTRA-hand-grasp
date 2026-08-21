# -*- coding: utf-8 -*-
"""콘솔의 입력 변환. cv2 창도 소켓도 필요 없다."""

from console_input import key_to_command, to_source


def test_unscaled_view_passes_through():
    assert to_source((10, 20, 30, 40), 640, 480, 640, 480) == (10, 20, 30, 40)


def test_half_scale_view_doubles_the_box():
    """축소해 보낸 미리보기 위에서 드래그하면 ROI 가 절반 자리에 잡힌다.

    이 환산이 빠지면 화면에는 맞게 보이는데 데몬은 엉뚱한 데를 잰다.
    """
    assert to_source((10, 20, 30, 40), 320, 240, 640, 480) == (20, 40, 60, 80)


def test_rounding_never_produces_a_zero_size():
    """0 크기 ROI 는 validate() 에서 거절된다. 여기서 막는다."""
    x, y, w, h = to_source((0, 0, 1, 1), 1000, 1000, 100, 100)
    assert w >= 1 and h >= 1


def test_align_toggle_is_not_here():
    """a 는 현재 상태를 알아야 뒤집을 수 있어 콘솔 루프가 처리한다."""
    assert key_to_command(ord("a")) is None


def test_calibration_keys_send_actions_not_values():
    assert key_to_command(ord("n")) == {"cmd": "calib_band", "edge": "near"}
    assert key_to_command(ord("f")) == {"cmd": "calib_band", "edge": "far"}
    assert key_to_command(ord("t")) == {"cmd": "capture_target"}
    assert key_to_command(ord("h")) == {"cmd": "save_hand_mask"}


def test_nudge_keys_send_deltas():
    from roi_config import NUDGE_M, TARGET_NUDGE_DEG
    assert key_to_command(ord("[")) == {
        "cmd": "nudge_band", "edge": "near", "delta_m": -NUDGE_M}
    assert key_to_command(ord("]")) == {
        "cmd": "nudge_band", "edge": "near", "delta_m": +NUDGE_M}
    assert key_to_command(ord("-")) == {
        "cmd": "nudge_band", "edge": "far", "delta_m": -NUDGE_M}
    assert key_to_command(ord("=")) == {
        "cmd": "nudge_band", "edge": "far", "delta_m": +NUDGE_M}
    assert key_to_command(ord(",")) == {
        "cmd": "nudge_target", "delta_deg": -TARGET_NUDGE_DEG}
    assert key_to_command(ord(".")) == {
        "cmd": "nudge_target", "delta_deg": +TARGET_NUDGE_DEG}


def test_wrist_jog_carries_a_direction():
    assert key_to_command(ord("w")) == {"cmd": "jog_wrist", "dir": 1}
    assert key_to_command(ord("W")) == {"cmd": "jog_wrist", "dir": -1}


def test_release_and_emergency():
    assert key_to_command(ord("r")) == {"cmd": "release"}
    assert key_to_command(ord(" ")) == {"cmd": "emergency_open"}


def test_unmapped_key_is_none():
    assert key_to_command(ord("z")) is None
    assert key_to_command(255) is None


def test_g는_수동_파지다():
    # ROI 트리거를 안 기다리고 지금 잡는다. 파지 안쪽 로직을 고칠 때
    # 카메라 조건을 매번 맞추지 않으려면 이 경로가 필요하다.
    assert key_to_command(ord("g")) == {"cmd": "grasp"}
