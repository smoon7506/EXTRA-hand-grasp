# -*- coding: utf-8 -*-
"""D405 ROI 안에 물체가 들어오면 EXTRA Hand 로 파지한다.

카메라와 손은 서로 고정되어 있다는 전제다. 그래서 YOLO 로 손을 매 프레임
찾을 필요가 없다 -- 손 위치는 픽셀 좌표로 한 번 정해두면 끝이다.
(detection/hand_center.py 계열은 손이 움직일 때를 위한 것이다.)

  python roi_grasp.py --no-hand   카메라·판정만. 모터에 연결하지 않는다
  python roi_grasp.py             전체

--- 하드웨어를 아는 코드는 main() 안에만 있다 ---
그래야 판정 로직 전체가 카메라 없이 테스트된다. real_sense.py 는 import
만 해도 파이프라인이 열리므로 그 파일을 복사 베이스로 삼지 말 것.
"""

import argparse
import json
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path

import cv2
import numpy as np

import orientation
import wrist_align
from grasp_state import (ALIGN_GAIN, ALIGN_HOLD_TOL_DEG, ALIGN_PINNED_FRAMES,
                         ALIGN_STABLE_FRAMES, ALIGN_TIMEOUT_S, ALIGN_TOL_DEG,
                         ALIGNING, ARMED, CONFIRM_HOLD_S, CONFIRMING, GRASP_A,
                         GRASPING, HOLDING, REARM_S, RELEASING, SETTLE_S,
                         GraspStateMachine, SequenceExecutor)
from roi_config import (FAR_M, MARGIN_M, NEAR_M, NUDGE_M, ROI_JSON,
                        TARGET_NUDGE_DEG, RoiConfig, nudge_band,
                        nudge_target)
from roi_judge import (ENTER_FRAMES, ENTER_RATIO, EXIT_RATIO,
                       MIN_VALID_RATIO, RatioTrigger, axis_segment,
                       band_ratio, roi_median)

# hand_control 은 옆 폴더다. 복사하지 않고 경로만 더해서 원본을 쓴다
# -- 복사본을 두면 파지 순서를 고쳤을 때 조용히 어긋난다.
_HAND_CONTROL = Path(__file__).resolve().parent.parent / "hand_control"
if str(_HAND_CONTROL) not in sys.path:
    sys.path.insert(0, str(_HAND_CONTROL))

import grasp_runner                     # noqa: E402
import hand_config                      # noqa: E402
import sequence                         # noqa: E402
import servo_bus                        # noqa: E402
import wrist as wrist_mod               # noqa: E402
from hand import Hand                   # noqa: E402
from tactile import TactileHand         # noqa: E402

# 이 파일은 한글로 출력한다. 콘솔 코드페이지가 UTF-8 이 아니면
# UnicodeEncodeError 로 죽는다 (hand_control/main.py:28-31 과 같은 패턴).
try:
    sys.stdout.reconfigure(encoding="utf-8")
except (AttributeError, ValueError):
    pass


# --- 카메라 ---
# depth 해상도는 함부로 올리면 안 된다. D435i 의 최소 측정 거리(Min-Z)는
# 해상도에 비례해서, 카메라~손끝 200mm 배치 기준으로
#   1280x720 -> 약 280mm (측정 불가)
#   848x480  -> 약 195mm (아슬아슬)
#   640x480  -> 약 175mm (여유 있음)
# 이다. 올리는 순간 ROI 가 통째로 무효 픽셀이 되면서 조용히 깨진다.
#
# 위 숫자는 D435i 로 잰 것이다. 지금 쓰는 카메라는 D405 라 Min-Z 가
# 훨씬 짧으므로, 해상도를 올릴 일이 생기면 D405 로 다시 재야 한다.
WIDTH, HEIGHT, FPS = 640, 480, 30


class RoiDragger:
    """마우스 드래그로 ROI 박스를 그린다.

    OpenCV 콜백은 상태를 밖에 둘 수밖에 없어서 작은 클래스로 감싼다.
    """

    def __init__(self):
        self.start = None       # 드래그 시작점 (x, y)
        self.current = None     # 지금 커서 (x, y)
        self.result = None      # 완성된 (x, y, w, h)

    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.start = (x, y)
            self.current = (x, y)
            self.result = None
        elif event == cv2.EVENT_MOUSEMOVE and self.start is not None:
            self.current = (x, y)
        elif event == cv2.EVENT_LBUTTONUP and self.start is not None:
            self.current = (x, y)
            x0, y0 = self.start
            # 어느 방향으로 끌어도 되게 정규화한다.
            box = (min(x0, x), min(y0, y), abs(x - x0), abs(y - y0))
            self.start = None
            # 한 점 클릭은 무시한다. w/h 가 0 이면 ROI 가 빈 배열이 된다.
            self.result = box if box[2] > 0 and box[3] > 0 else None

    def preview(self):
        """드래그 중인 박스 (x, y, w, h). 드래그 중이 아니면 None."""
        if self.start is None or self.current is None:
            return None
        x0, y0 = self.start
        x1, y1 = self.current
        return (min(x0, x1), min(y0, y1), abs(x1 - x0), abs(y1 - y0))


# 화면 색 (BGR)
_GREEN = (0, 255, 0)
_YELLOW = (0, 255, 255)
_RED = (0, 0, 255)
_WHITE = (255, 255, 255)

_STATE_COLOR = {
    ARMED: _GREEN,
    ALIGNING: _YELLOW,
    CONFIRMING: _YELLOW,
    GRASPING: _YELLOW,
    HOLDING: _RED,
    RELEASING: _YELLOW,
}


def draw_overlay(image, roi, depth_m, ratio, valid, median, state, hint,
                 cooldown=0.0, angle=None, angle_reason="", wrist_deg=None,
                 align_status=None, confirm_left=0.0, has_hand_mask=False,
                 align_on=True, axis=None):
    """화면에 ROI, 밴드 안 픽셀, 주축, 숫자, 상태를 그린다.

    튜닝이 이 프로젝트 작업량의 대부분이라 여기에 투자한다. 숫자만
    보고는 "왜 안 잡히지"를 못 찾는다 -- 무엇이 밴드에 걸렸는지가
    눈에 보여야 한다.

    axis: ROI 안 좌표계의 ((x0, y0), (x1, y1)) 선분. 주축을 눈으로 보기
    위한 것이다 -- 각도 숫자만으로는 무엇을 물체로 보고 있는지 알 수 없다.

    문구를 영문으로 쓰는 이유: OpenCV 기본 폰트는 한글을 ??? 로 그린다.
    폰트를 붙이는 건 이 작업의 목적이 아니다.
    """
    if roi is not None:
        # 밴드 안 픽셀을 초록으로 칠한다.
        sub = roi.slice(depth_m)
        if sub.size:
            mask = (sub > 0) & (sub >= roi.near_m) & (sub <= roi.far_m)
            patch = image[roi.y:roi.y + roi.h, roi.x:roi.x + roi.w]
            if patch.shape[:2] == mask.shape:
                patch[mask] = (0.5 * patch[mask] +
                               0.5 * np.array(_GREEN)).astype(np.uint8)
        cv2.rectangle(image, (roi.x, roi.y),
                      (roi.x + roi.w, roi.y + roi.h), _WHITE, 1)

    if roi is not None and axis is not None:
        (x0, y0), (x1, y1) = axis
        cv2.line(image, (roi.x + int(x0), roi.y + int(y0)),
                 (roi.x + int(x1), roi.y + int(y1)), _RED, 2)

    # 쿨다운 중에도 상태는 ARMED 다. 남은 시간을 안 보여주면 "물체를
    # 넣었는데 왜 반응이 없지"가 된다.
    lines = [f"state: {state}"
             + (f"  (rearm in {cooldown:.1f}s)" if cooldown > 0 else "")]
    if roi is None:
        lines.append("drag to set ROI")
    else:
        lines.append(f"band: {roi.near_m:.3f} ~ {roi.far_m:.3f} m")
        lines.append(
            "ratio: --" if ratio is None else
            f"ratio: {ratio:5.2f}  (enter {ENTER_RATIO:.2f} / exit {EXIT_RATIO:.2f})")
        lines.append(
            "median: --" if median is None else f"median: {median:.3f} m")
        if valid < MIN_VALID_RATIO:
            lines.append(f"! valid {valid*100:.0f}% - sensor cannot see")

    if roi is not None:
        # 각도는 정렬을 껐을 때도 보여준다. --no-hand 는 손목이 없어 항상
        # align OFF 인데, 브링업 2단계(각도가 물체를 따라가나)는 바로 그
        # 상태에서 이 숫자를 보고 판단한다.
        lines.append(
            "angle: --  (%s)" % angle_reason if angle is None else
            f"angle: {angle:+6.1f}  target: {roi.target_angle_deg:+6.1f}"
            f"  err: {orientation.wrap90(angle - roi.target_angle_deg):+6.1f}"
        )
        if not align_on:
            lines.append("align: OFF")
        else:
            lines.append(
                "wrist: --" if wrist_deg is None else
                f"wrist: {wrist_deg:+6.1f}  {align_status or ''}")
        if confirm_left > 0.0:
            lines.append(f"confirm in {confirm_left:.1f}s")
        if not has_hand_mask:
            lines.append("! no hand mask - press h with the object removed")

    color = _STATE_COLOR.get(state, _WHITE)
    for i, text in enumerate(lines):
        cv2.putText(image, text, (10, 25 + i * 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    cv2.putText(image, hint, (10, image.shape[0] - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, _WHITE, 1)
    return image


_HINT = ("drag=ROI  n/f=band  []-= nudge  t=target ,.=tweak  "
         "w/W=wrist  h=hand mask  a=align  r=release  space=open  q=quit")


def _save_band(roi):
    """밴드를 파일에 남긴다. 뒤집힌 값이면 저장하지 않고 알린다."""
    try:
        roi.save()
        print(f"[INFO] 밴드 저장: {roi.near_m:.3f} ~ {roi.far_m:.3f} m")
    except ValueError as e:
        print(f"[WARN] {e}")


def _wrist_lease_fns(wrist):
    """ServoLease 에 넘길 (열기, 닫기) 짝.

    닫을 때 토크를 끄지 않는다. 손목 위에는 카메라가 실려 있어서, 리스를
    비켜 줄 때마다 늘어지면 그 자체로 각도 측정이 깨진다.
    """
    return (lambda: (wrist.connect(), wrist)[1],
            lambda _w: wrist.disconnect(torque_off=False))


def _hand_lease_fns(hand):
    """손 리스. 반납은 토크를 끄지 않는다 -- 편 자세를 유지해야 한다."""
    return (lambda: (hand.connect(), hand)[1],
            lambda _h: hand.disconnect())


def main():
    import pyrealsense2 as rs      # 카메라 없는 환경에서도 import 가능해야 한다

    p = argparse.ArgumentParser(
        prog="roi_grasp.py",
        description="ROI 안에 물체가 들어오면 EXTRA Hand 로 파지")
    p.add_argument("--no-hand", action="store_true",
                   help="모터에 연결하지 않고 카메라·판정만")
    p.add_argument("--no-wrist", action="store_true",
                   help="손목을 쓰지 않는다 (정렬 없이 3초 게이트만)")
    p.add_argument("--simple-grasp", action="store_true",
                   help="촉각 힘 제어 대신 고정 자세 시퀀스로 파지한다. "
                        "촉각 센서가 죽었을 때 데모용")
    args = p.parse_args()

    try:
        roi = RoiConfig.load()
    except ValueError as e:
        print(f"[WARN] {e}")
        print("       ROI 를 새로 드래그하세요.")
        roi = None
    if roi is None:
        print(f"[INFO] {ROI_JSON.name} 이 없습니다. 드래그로 ROI 를 지정하세요.")

    fingers = hand_config.load_fingers(active=hand_config.ACTIVE_FINGERS)

    # 손과 손목은 같은 COM11 을 쓴다. 동시에 열 수 없어서 리스로 시분할한다.
    bus = servo_bus.ServoLease()
    hand = None if args.no_hand else Hand(fingers)
    hand_open, hand_close = ((None, None) if hand is None
                             else _hand_lease_fns(hand))

    wrist = None
    if not args.no_wrist and not args.no_hand:
        wrist = wrist_mod.Sts3215Wrist(
            hand_config.WRIST_ID, hand_config.SERIAL_PORT,
            hand_config.BAUDRATE, hand_config.SERIAL_TIMEOUT_S,
            speed=hand_config.WRIST_GOAL_SPEED,
            min_rad=hand_config.WRIST_MIN_RAD,
            max_rad=hand_config.WRIST_MAX_RAD)
    wrist_open, wrist_close = ((None, None) if wrist is None
                               else _wrist_lease_fns(wrist))

    # 촉각 센서는 CH341(별도 USB)이라 시리얼 버스와 무관하다. 내내 켜 둔다.
    sensors = None
    if hand is not None and not args.simple_grasp:
        sensors = TactileHand()
        sensors.start()
        if not sensors.wait_ready(hand_config.SENSOR_TIMEOUT_S):
            print("[ERROR] 촉각 센서 데이터가 오지 않습니다. "
                  "--simple-grasp 로 고정 자세 파지를 쓸 수 있습니다.")
            sensors.stop()
            return 1
        print("[INFO] baseline 을 잡습니다. 손에 아무것도 닿지 않게 하세요...")
        sensors.calibrate()

    if hand is None or args.simple_grasp:
        senders = [] if hand is None else [hand.set_pose]
        executor = SequenceExecutor(sequence.PoseSequencer(fingers, senders))
    else:
        executor = grasp_runner.ForceGraspRunner(fingers, sensors, hand)

    aligner = None
    if wrist is not None:
        aligner = wrist_align.WristAligner(
            (roi.target_angle_deg if roi else 0.0),
            tol_deg=ALIGN_TOL_DEG, hold_tol_deg=ALIGN_HOLD_TOL_DEG,
            gain=ALIGN_GAIN,
            da_max_rad=hand_config.WRIST_DA_MAX_RAD,
            min_rad=hand_config.WRIST_MIN_RAD,
            max_rad=hand_config.WRIST_MAX_RAD,
            direction_sign=hand_config.WRIST_DIRECTION_SIGN,
            stable_frames=ALIGN_STABLE_FRAMES,
            pinned_frames=ALIGN_PINNED_FRAMES,
            timeout_s=ALIGN_TIMEOUT_S)
        aligner.start(hand_config.WRIST_CENTER_RAD)

    machine = GraspStateMachine(executor, RatioTrigger(), aligner=aligner)
    dragger = RoiDragger()
    hand_mask = None

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
    config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)
    # depth 를 컬러 시점에 맞춰야 화면에 그린 ROI 와 실제로 재는 픽셀이
    # 같은 곳을 가리킨다.
    align = rs.align(rs.stream.color)

    window = "ROI grasp"
    print("[INFO] D405 연결 대기 중...")
    pipeline.start(config)
    cv2.namedWindow(window)
    cv2.setMouseCallback(window, dragger.on_mouse)
    print(f"[INFO] 시작. {_HINT}")

    try:
        while True:
            frames = align.process(pipeline.wait_for_frames())
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            image = np.asanyarray(color_frame.get_data()).copy()
            # z16 raw -> m. get_distance 를 픽셀마다 부르면 너무 느리다.
            depth_m = (np.asanyarray(depth_frame.get_data()).astype(np.float32)
                       * depth_frame.get_units())

            if dragger.result is not None:
                x, y, w, h = dragger.result
                dragger.result = None
                near, far = (roi.near_m, roi.far_m) if roi else (NEAR_M, FAR_M)
                roi = RoiConfig(x=x, y=y, w=w, h=h, near_m=near, far_m=far)
                roi.save()
                print(f"[INFO] ROI 저장: x={x} y={y} w={w} h={h}")

            ratio, valid, median = None, 0.0, None
            if roi is not None:
                sub = roi.slice(depth_m)
                ratio, valid = band_ratio(sub, roi.near_m, roi.far_m)
                median = roi_median(sub)

            angle, angle_reason, axis = None, "no roi", None
            if roi is not None:
                sub = roi.slice(depth_m)
                band = (sub > 0) & (sub >= roi.near_m) & (sub <= roi.far_m)
                if hand_mask is None:
                    hand_mask = orientation.load_hand_mask(band.shape)
                clean = orientation.subtract_hand(band, hand_mask)
                angle, info = orientation.principal_axis(clean)
                if angle is None:
                    angle_reason = str(info)
                else:
                    axis = axis_segment(clean, angle)

            # 트리거는 machine 이 들고 있다. 어느 상태에서 판정할지,
            # 언제 초기화할지를 상태 전이와 같이 관리해야 어긋나지 않는다.
            state = machine.update(ratio, angle_deg=angle)

            # --- 포트 시분할 ---
            # 손과 손목은 같은 COM11 을 쓴다. 동시에 열 수 없다.
            if hand is not None:
                if state in (ALIGNING, CONFIRMING) and wrist is not None \
                        and machine.align_enabled:
                    bus.acquire("wrist", wrist_open, wrist_close)
                    if machine.wrist_goal_rad is not None:
                        wrist.write_goal(machine.wrist_goal_rad)
                elif state in (GRASPING, HOLDING, RELEASING):
                    bus.acquire("hand", hand_open, hand_close)
                elif state == ARMED and bus.owner() is not None:
                    # ARMED 에서는 아무 모터도 안 움직인다. 놓아 두면
                    # 다른 도구를 같이 띄울 수 있다.
                    bus.release()

            wrist_deg = None
            if wrist is not None and bus.owner() == "wrist":
                current = wrist.read_position()
                if current is not None:
                    wrist_deg = np.degrees(current)

            draw_overlay(image, roi, depth_m, ratio, valid, median,
                         machine.state, _HINT, machine.rearm_remaining(),
                         angle=angle, angle_reason=angle_reason,
                         wrist_deg=wrist_deg,
                         align_status=machine.align_status,
                         confirm_left=machine.confirm_remaining(),
                         has_hand_mask=hand_mask is not None,
                         align_on=machine.align_enabled, axis=axis)
            box = dragger.preview()
            if box is not None:
                cv2.rectangle(image, (box[0], box[1]),
                              (box[0] + box[2], box[1] + box[3]), _YELLOW, 1)
            cv2.imshow(window, image)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("r"):
                if not machine.request_release():
                    print("[INFO] 지금은 놓을 수 없습니다 "
                          f"(상태 {machine.state}).")
            elif key == ord(" "):
                # 비상 폄. 정렬 중이면 손 포트가 안 열려 있다.
                if hand is not None:
                    bus.acquire("hand", hand_open, hand_close)
                machine.emergency_open()
                print("[INFO] 비상 폄")
            elif roi is not None and key == ord("t"):
                if angle is None:
                    print(f"[WARN] 각도를 못 재고 있어 목표를 정할 수 "
                          f"없습니다 ({angle_reason}).")
                else:
                    roi.target_angle_deg = angle
                    if aligner is not None:
                        aligner.target_deg = orientation.wrap90(angle)
                    _save_band(roi)
                    print(f"[INFO] 목표 각도 저장: {angle:+.1f}도")
            elif roi is not None and key in (ord(","), ord(".")):
                if nudge_target(roi, key):
                    if aligner is not None:
                        aligner.target_deg = roi.target_angle_deg
                    _save_band(roi)
            elif roi is not None and key == ord("h"):
                sub = roi.slice(depth_m)
                band = (sub > 0) & (sub >= roi.near_m) & (sub <= roi.far_m)
                orientation.save_hand_mask(band)
                hand_mask = band.copy()
                print(f"[INFO] 손 마스크 저장 ({int(band.sum())} 픽셀). "
                      f"물체를 치운 상태에서 찍었는지 확인하세요.")
            elif key == ord("a"):
                machine.set_align_enabled(not machine.align_enabled)
                print(f"[INFO] 자동 정렬 "
                      f"{'켬' if machine.align_enabled else '끔'}")
            elif key in (ord("w"), ord("W")) and wrist is not None:
                # 수동 조그. 부호와 가동범위를 눈으로 확인하는 용도다.
                bus.acquire("wrist", wrist_open, wrist_close)
                step = (hand_config.WRIST_DA_MAX_RAD
                        * (1 if key == ord("w") else -1))
                current = wrist.read_position()
                if current is None:
                    print("[WARN] 손목 위치를 못 읽었습니다.")
                else:
                    sent = wrist.write_goal(current + step)
                    print(f"[INFO] 손목 {np.degrees(sent):+.1f}도")
            elif roi is not None and key in (ord("n"), ord("f")):
                if median is None:
                    print("[WARN] ROI 에 유효 깊이가 없어 캘리브레이션 불가")
                else:
                    # 여유를 바깥으로 벌린다. 센서 노이즈가 ±1cm 다.
                    if key == ord("n"):
                        roi.near_m = median - MARGIN_M
                    else:
                        roi.far_m = median + MARGIN_M
                    _save_band(roi)
            elif roi is not None and nudge_band(roi, key):
                _save_band(roi)
    except KeyboardInterrupt:
        print("\n[INFO] 중단됨")
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        # 리스를 번갈아 잡아야 하므로 순서가 정해져 있다. 각 단계는 앞
        # 단계가 실패해도 실행돼야 한다 -- 그러지 않으면 모터가 토크
        # 걸린 채 방치된다. KeyboardInterrupt 는 Exception 의 하위가
        # 아니므로 BaseException 을 잡는다.
        if wrist is not None:
            try:
                bus.acquire("wrist", wrist_open, wrist_close)
                wrist.disconnect(torque_off=True)
            except BaseException as e:
                print(f"[WARN] 손목 토크 해제 실패: {e}")
        try:
            bus.release()
        except BaseException as e:
            print(f"[WARN] 리스 반납 실패: {e}")
        if hand is not None:
            try:
                hand.connect()
                hand.release()
                print("[INFO] 편 자세로 되돌리고 토크를 껐습니다.")
            except BaseException as e:
                print(f"[WARN] 손 정리 실패: {e}")
        if sensors is not None:
            sensors.stop()
        if isinstance(executor, grasp_runner.ForceGraspRunner):
            executor.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
