# -*- coding: utf-8 -*-
"""ROI 파지 데몬. 화면이 없다. 판정과 구동이 전부 여기 있다.

  python grasp_daemon.py --no-hand    카메라·판정만
  python grasp_daemon.py              전체

--- 화면이 없다는 것의 의미 ---
사람이 볼 것은 콘솔(grasp_console.py)이 그린다. 이 파일은 숫자와
JPEG 만 올려보낸다. 깊이 배열은 보내지 않는다 -- 오버레이가 필요로
하는 것은 ROI 크기 이진 마스크뿐이다(설계 문서 참고).

--- 무장은 사람이 건다 ---
GraspStateMachine 을 armed=False 로 만든다. 부팅하자마자 손이 알아서
잡기 시작하면 안 된다.
"""

import argparse
import signal
import socket
import sys
import time
from pathlib import Path

import cv2
import numpy as np

import grasp_state
import orientation
import roi_config
import wrist_align
from grasp_commands import CommandHandler, FrameState
from link import LineReader, LinkError, PreviewFrame, encode_msg, encode_preview
from link_sender import DropSender
from link_watchdog import LinkWatchdog
from roi_config import RoiConfig
from roi_judge import (ENTER_RATIO, EXIT_RATIO, MIN_VALID_RATIO, RatioTrigger,
                       axis_segment, band_ratio, roi_median)

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

try:
    sys.stdout.reconfigure(encoding="utf-8")
except (AttributeError, ValueError):
    pass

WIDTH, HEIGHT, FPS = 640, 480, 30
CONTROL_PORT = 5001
PREVIEW_PORT = 5002
JPEG_QUALITY = 70


class Listener:
    """논블로킹 리슨 소켓 하나. 최신 연결만 들고 있는다.

    새 연결이 오면 기존 것을 끊는다. 거절하면 콘솔이 비정상 종료했을 때
    좀비 연결 때문에 다시 못 붙는다.
    """

    def __init__(self, host, port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((host, port))
        self.sock.listen(1)
        self.sock.setblocking(False)

    def poll(self):
        """새 연결이 있으면 그 소켓, 없으면 None."""
        try:
            conn, addr = self.sock.accept()
        except (BlockingIOError, OSError):
            return None
        conn.setblocking(False)
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        print(f"[INFO] 접속: {addr}")
        return conn

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


def build_telemetry(seq, state, roi, ratio, valid, median, angle,
                    angle_reason, axis, wrist_deg, machine, watchdog,
                    has_hand_mask, bus_owner):
    """매 프레임 한 장. roi 와 cfg 를 통째로 실어 콘솔을 스테이트리스로 둔다.

    cfg 를 싣는 이유: 지금 draw_overlay 가 ENTER_RATIO 상수를 직접 읽어
    두 곳의 값이 어긋날 수 있다. 데몬이 실제로 쓰는 값을 보내면 그 여지가
    없어진다.
    """
    return {
        "t": "tel", "seq": seq, "state": state,
        "ratio": ratio, "valid": valid, "median": median,
        "angle": angle, "angle_reason": angle_reason,
        "axis": ([[axis[0][0], axis[0][1]], [axis[1][0], axis[1][1]]]
                 if axis else None),
        "wrist_deg": wrist_deg,
        "align_status": machine.align_status,
        "align_on": machine.align_enabled,
        "armed": machine.armed,
        "confirm_left": machine.confirm_remaining(),
        "rearm_left": machine.rearm_remaining(),
        "release_in": watchdog.release_in(),
        "has_hand_mask": has_hand_mask,
        "bus_owner": bus_owner,
        "roi": (None if roi is None else {
            "x": roi.x, "y": roi.y, "w": roi.w, "h": roi.h,
            "near_m": roi.near_m, "far_m": roi.far_m,
            "target_angle_deg": roi.target_angle_deg}),
        "cfg": {"enter_ratio": ENTER_RATIO, "exit_ratio": EXIT_RATIO,
                "min_valid_ratio": MIN_VALID_RATIO},
    }


def encode_frame(seq, image, band, roi, scale):
    """컬러 프레임 + ROI 밴드 마스크 -> 보낼 bytes.

    깊이 배열 대신 이 마스크를 보낸다. 640x480 float32 는 1.2MB 지만
    150x150 이진 PNG 는 500 바이트다.
    """
    src_h, src_w = image.shape[:2]
    view = image
    if scale != 1.0:
        view = cv2.resize(image, (int(src_w * scale), int(src_h * scale)),
                          interpolation=cv2.INTER_AREA)
    ok, jpg = cv2.imencode(".jpg", view,
                           [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
    if not ok:
        return None
    mask_png = b""
    if band is not None and band.size:
        ok2, png = cv2.imencode(".png", (band.astype(np.uint8) * 255))
        if ok2:
            mask_png = png.tobytes()
    return encode_preview(
        seq=seq, src_w=src_w, src_h=src_h, jpeg=jpg.tobytes(),
        mask_png=mask_png,
        roi=None if roi is None else [roi.x, roi.y, roi.w, roi.h])


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
        prog="grasp_daemon.py",
        description="화면 없는 ROI 파지 데몬. 소켓으로 콘솔과 통신한다.")
    p.add_argument("--no-hand", action="store_true",
                   help="모터에 연결하지 않고 카메라·판정만")
    p.add_argument("--no-wrist", action="store_true",
                   help="손목을 쓰지 않는다 (정렬 없이 3초 게이트만)")
    p.add_argument("--simple-grasp", action="store_true",
                   help="촉각 힘 제어 대신 고정 자세 시퀀스로 파지한다. "
                        "촉각 센서가 죽었을 때 데모용")
    p.add_argument("--host", default="0.0.0.0",
                   help="리슨 주소 (기본 0.0.0.0)")
    p.add_argument("--preview-fps", type=float, default=15.0,
                   help="미리보기 전송 프레임률 (기본 15)")
    p.add_argument("--preview-scale", type=float, default=1.0,
                   help="미리보기 축소 비율 (기본 1.0 = 원본 그대로)")
    args = p.parse_args()

    try:
        roi = RoiConfig.load()
    except ValueError as e:
        print(f"[WARN] {e}")
        print("       ROI 를 콘솔에서 새로 지정하세요.")
        roi = None
    if roi is None:
        print(f"[INFO] {roi_config.ROI_JSON.name} 이 없습니다. "
              f"콘솔에서 ROI 를 지정하세요.")

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
        executor = grasp_state.SequenceExecutor(
            sequence.PoseSequencer(fingers, senders))
    else:
        executor = grasp_runner.ForceGraspRunner(fingers, sensors, hand)

    aligner = None
    if wrist is not None:
        aligner = wrist_align.WristAligner(
            (roi.target_angle_deg if roi else 0.0),
            tol_deg=grasp_state.ALIGN_TOL_DEG,
            hold_tol_deg=grasp_state.ALIGN_HOLD_TOL_DEG,
            gain=grasp_state.ALIGN_GAIN,
            da_max_rad=hand_config.WRIST_DA_MAX_RAD,
            min_rad=hand_config.WRIST_MIN_RAD,
            max_rad=hand_config.WRIST_MAX_RAD,
            direction_sign=hand_config.WRIST_DIRECTION_SIGN,
            stable_frames=grasp_state.ALIGN_STABLE_FRAMES,
            pinned_frames=grasp_state.ALIGN_PINNED_FRAMES,
            timeout_s=grasp_state.ALIGN_TIMEOUT_S)
        aligner.start(hand_config.WRIST_CENTER_RAD)

    # 무장은 사람이 콘솔에서 건다. 부팅하자마자, 혹은 systemd 가 재시작
    # 하자마자 손이 알아서 잡기 시작하면 안 된다.
    machine = grasp_state.GraspStateMachine(
        executor, RatioTrigger(), aligner=aligner, armed=False)

    fs = FrameState(machine=machine, wrist=wrist, aligner=aligner)
    handler = CommandHandler(fs)
    watchdog = LinkWatchdog(machine)

    box = {"hand_mask": None}       # 루프와 훅이 같이 보는 한 칸.
    # hand_mask 를 이 안에 담는 이유: 루프는 읽고 훅(on_save_hand_mask)은
    # 쓰는데, 파이썬 클로저는 바깥 지역변수에 대입을 못 한다 -- 그냥
    # hand_mask = ... 라고 쓰면 훅 안의 새 지역변수가 되고 루프는 영영
    # 옛 값을 본다.

    def _save_roi(r):
        try:
            r.save()
            print(f"[INFO] roi.json 저장: 밴드 {r.near_m:.3f} ~ {r.far_m:.3f} m")
        except ValueError as e:
            print(f"[WARN] 저장하지 못했다: {e}")

    def _save_mask(mask):
        orientation.save_hand_mask(mask)
        box["hand_mask"] = mask.copy()
        print(f"[INFO] 손 마스크 저장 ({int(mask.sum())} 픽셀)")

    def _jog(direction):
        if wrist is None:
            return False, "손목이 없다."
        bus.acquire("wrist", wrist_open, wrist_close)
        step = hand_config.WRIST_DA_MAX_RAD * (1 if direction > 0 else -1)
        current = wrist.read_position()
        if current is None:
            return False, "손목 위치를 못 읽었다."
        sent = wrist.write_goal(current + step)
        return True, f"손목 {np.degrees(sent):+.1f}도"

    fs.on_roi_saved = _save_roi
    fs.on_save_hand_mask = _save_mask
    fs.on_jog_wrist = _jog

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
    config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)
    # depth 를 컬러 시점에 맞춰야 콘솔이 그린 ROI 와 실제로 재는 픽셀이
    # 같은 곳을 가리킨다.
    align = rs.align(rs.stream.color)

    print("[INFO] D405 연결 대기 중...")
    pipeline.start(config)

    ctl_listener = Listener(args.host, CONTROL_PORT)
    pv_listener = Listener(args.host, PREVIEW_PORT)
    ctl_sender = DropSender()
    pv_sender = DropSender()
    ctl_reader = LineReader()
    # FPS 를 정수로 안 나눠떨어지면 round 가 preview_fps 를 살짝 어긋나게
    # 하지만, 미리보기는 눈으로 보는 용도라 그 정도 오차는 무해하다.
    preview_every = max(1, round(FPS / args.preview_fps))

    stopping = {"now": False}

    def _stop(signum, frame):
        # 헤드리스라 Ctrl+C 대신 systemd 가 SIGTERM 을 보낼 수 있다.
        # 이게 없으면 모터가 토크 걸린 채 방치된다.
        print(f"\n[INFO] 종료 신호({signum})")
        stopping["now"] = True

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    print(f"[INFO] 시작. control={CONTROL_PORT} preview={PREVIEW_PORT}")

    seq = 0
    try:
        while not stopping["now"]:
            frames = align.process(pipeline.wait_for_frames())
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            image = np.asanyarray(color_frame.get_data()).copy()
            # z16 raw -> m. get_distance 를 픽셀마다 부르면 너무 느리다.
            depth_m = (np.asanyarray(depth_frame.get_data()).astype(np.float32)
                       * depth_frame.get_units())

            # ROI 가 없는 프레임에서도 미리보기가 band 를 참조하므로 루프
            # 머리에서 초기화한다. 원본은 if roi is not None: 안에서만
            # 만든다.
            band = None

            ratio, valid, median = None, 0.0, None
            if roi is not None:
                sub = roi.slice(depth_m)
                ratio, valid = band_ratio(sub, roi.near_m, roi.far_m)
                median = roi_median(sub)

            angle, angle_reason, axis = None, "no roi", None
            if roi is not None:
                sub = roi.slice(depth_m)
                band = (sub > 0) & (sub >= roi.near_m) & (sub <= roi.far_m)
                if box["hand_mask"] is None:
                    box["hand_mask"] = orientation.load_hand_mask(band.shape)
                clean = orientation.subtract_hand(band, box["hand_mask"])
                angle, info = orientation.principal_axis(clean)
                if angle is None:
                    angle_reason = str(info)
                else:
                    axis = axis_segment(clean, angle)

            fs.roi = roi
            fs.median = median
            fs.angle = angle
            fs.angle_reason = angle_reason
            fs.band_mask = band

            # --- 새 접속 받기 ---
            conn = ctl_listener.poll()
            if conn is not None:
                ctl_sender.attach(conn)
                ctl_reader = LineReader()
                watchdog.feed()
            pv = pv_listener.poll()
            if pv is not None:
                pv_sender.attach(pv)

            # --- 명령 읽기 ---
            chunk = ctl_sender.recv()
            if chunk is None:
                # 끊겼다. recv 가 detach 까지 했다.
                watchdog.drop()
            elif chunk:
                watchdog.feed()
                try:
                    for msg in ctl_reader.feed(chunk):
                        ack = handler.handle(msg)
                        ctl_sender.send(encode_msg(ack))
                        if msg.get("cmd") == "bye":
                            ctl_sender.detach()
                            watchdog.drop()
                except LinkError as e:
                    print(f"[WARN] 프로토콜이 깨졌다: {e}")
                    ctl_sender.detach()
                    watchdog.drop()

            # 트리거는 machine 이 들고 있다. 어느 상태에서 판정할지,
            # 언제 초기화할지를 상태 전이와 같이 관리해야 어긋나지 않는다.
            # --- 상태기계 다음에 워치독 ---
            state = machine.update(ratio, angle_deg=angle)
            watchdog.tick()

            # --- 포트 시분할 ---
            # 손과 손목은 같은 COM11 을 쓴다. 동시에 열 수 없다.
            if hand is not None:
                if state in (grasp_state.ALIGNING, grasp_state.CONFIRMING) \
                        and wrist is not None and machine.align_enabled:
                    bus.acquire("wrist", wrist_open, wrist_close)
                    if machine.wrist_goal_rad is not None:
                        wrist.write_goal(machine.wrist_goal_rad)
                elif state in (grasp_state.GRASPING, grasp_state.HOLDING,
                              grasp_state.RELEASING):
                    bus.acquire("hand", hand_open, hand_close)
                elif state == grasp_state.ARMED and bus.owner() is not None:
                    # ARMED 에서는 아무 모터도 안 움직인다. 놓아 두면
                    # 다른 도구를 같이 띄울 수 있다.
                    bus.release()

            wrist_deg = None
            if wrist is not None and bus.owner() == "wrist":
                current = wrist.read_position()
                if current is not None:
                    wrist_deg = np.degrees(current)

            # --- 올려보내기 (막히면 버린다) ---
            tel = build_telemetry(
                seq=seq, state=state, roi=roi, ratio=ratio, valid=valid,
                median=median, angle=angle, angle_reason=angle_reason,
                axis=axis, wrist_deg=wrist_deg, machine=machine,
                watchdog=watchdog, has_hand_mask=box["hand_mask"] is not None,
                bus_owner=bus.owner())
            # 나가다 만 꼬리부터 밀어낸다. 이걸 안 부르면 꼬리가 영영
            # 안 나가고 그동안 새 프레임이 전부 버려진다.
            ctl_sender.flush()
            pv_sender.flush()

            ctl_sender.send(encode_msg(tel))
            if seq % preview_every == 0:
                blob = encode_frame(seq, image, band, roi, args.preview_scale)
                if blob is not None:
                    pv_sender.send(blob)
            seq += 1
    except KeyboardInterrupt:
        print("\n[INFO] 중단됨")
    finally:
        pipeline.stop()
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
        ctl_listener.close()
        pv_listener.close()
        ctl_sender.detach()
        pv_sender.detach()
    return 0


if __name__ == "__main__":
    sys.exit(main())
