# -*- coding: utf-8 -*-
"""파지 데몬 콘솔. PC 에서 돈다 -- 화면을 그리고 키 입력을 명령으로 보낸다.

  python grasp_console.py [--host 127.0.0.1]

--- roi_grasp.py 와의 관계 ---
draw_overlay/RoiDragger/main() 의 키 처리를 그 파일에서 옮겨 왔다.
roi_grasp.py 자체는 기준선이라 건드리지 않는다 -- 카메라와 모터를 직접
쥔 옛 단일 프로세스판이 그대로 남아, 이 콘솔+데몬 쌍의 동작을 비교할
잣대가 된다.

--- 판정은 여기 없다 ---
ROI/밴드/각도/상태기계는 전부 grasp_daemon.py(파이) 쪽에 있다. 이
파일은 소켓으로 받은 텔레메트리(dict)와 JPEG/PNG 만 갖고 그리고, 키를
눌렀을 때 명령만 보낸다. 그래서 카메라도 모터도 import 하지 않는다.
"""

import argparse
import json
import socket
import sys
from pathlib import Path

import cv2
import numpy as np

import orientation
from console_input import key_to_command, to_source
from link import PROTO, LineReader, LinkError, PreviewReader, encode_msg

try:
    sys.stdout.reconfigure(encoding="utf-8")
except (AttributeError, ValueError):
    pass


CONTROL_PORT = 5001
PREVIEW_PORT = 5002

# pending 에 쌓아 두는 텔레메트리 상한. 넘으면 오래된 것부터 버린다 --
# 미리보기가 한동안 안 오면(연결 끊김 등) pending 이 무한정 자란다.
MAX_PENDING = 32

# 화면 색 (BGR). roi_grasp.py:112-124 를 그대로 옮겼다.
_GREEN = (0, 255, 0)
_YELLOW = (0, 255, 255)
_RED = (0, 0, 255)
_WHITE = (255, 255, 255)

_STATE_COLOR = {
    "ARMED": _GREEN,
    "ALIGNING": _YELLOW,
    "CONFIRMING": _YELLOW,
    "GRASPING": _YELLOW,
    "HOLDING": _RED,
    "RELEASING": _YELLOW,
}

_HINT = ("drag=ROI  n/f=band  []-= nudge  t=target ,.=tweak  "
         "w/W=wrist  h=hand mask  a=align  m=arm  r=release  "
         "space=open  q=quit")


class RoiDragger:
    """마우스 드래그로 ROI 박스를 그린다.

    roi_grasp.py:75-108 을 한 글자도 안 바꾸고 옮겼다. OpenCV 콜백은
    상태를 밖에 둘 수밖에 없어서 작은 클래스로 감싼다.
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


def draw_overlay(image, tel, mask, hint, scale=1.0):
    """텔레메트리 한 장 + ROI 마스크 -> 화면.

    --- 원본(roi_grasp.py:127-202)과 무엇이 다른가 ---
    depth_m 을 안 받는다. 원본은 초록 틴트를 그리려고 깊이 배열을 다시
    임계했지만(roi_grasp.py:145-147), 그 마스크는 데몬이 이미 만들었고
    PNG 로 왔다. 깊이 배열 1.2MB 를 매 프레임 보낼 이유가 없다.

    ENTER_RATIO 등을 상수에서 안 읽는다. tel["cfg"] 가 데몬이 실제로
    쓰는 값이라 콘솔과 데몬 두 곳의 임계값이 어긋날 수 없다.

    데몬이 미리보기를 축소해 보낼 수 있는데 ROI 좌표는 원본 기준으로
    온다. 드래그 좌표를 원본으로 되돌려 보내야 해서 그게 맞고, 대신
    그릴 때 여기서 줄인다.
    """
    roi = tel.get("roi")
    cfg = tel.get("cfg") or {}

    if roi is not None:
        rx, ry = int(round(roi["x"] * scale)), int(round(roi["y"] * scale))
        rw, rh = int(round(roi["w"] * scale)), int(round(roi["h"] * scale))

        if mask is not None and rw > 0 and rh > 0:
            patch = image[ry:ry + rh, rx:rx + rw]
            m = mask
            if m.shape != patch.shape[:2]:
                m = cv2.resize(m.astype(np.uint8), (patch.shape[1], patch.shape[0]),
                               interpolation=cv2.INTER_NEAREST).astype(bool)
            if m.shape == patch.shape[:2]:
                patch[m] = (0.5 * patch[m]
                            + 0.5 * np.array(_GREEN)).astype(np.uint8)
        cv2.rectangle(image, (rx, ry), (rx + rw, ry + rh), _WHITE, 1)

    axis = tel.get("axis")
    if roi is not None and axis is not None:
        (x0, y0), (x1, y1) = axis
        cv2.line(image, (rx + int(x0 * scale), ry + int(y0 * scale)),
                 (rx + int(x1 * scale), ry + int(y1 * scale)), _RED, 2)

    state = tel.get("state")
    cooldown = tel.get("rearm_left") or 0.0
    lines = [f"state: {state}"
             + (f"  (rearm in {cooldown:.1f}s)" if cooldown > 0 else "")]
    if roi is None:
        lines.append("drag to set ROI")
    else:
        lines.append(f"band: {roi['near_m']:.3f} ~ {roi['far_m']:.3f} m")
        ratio = tel.get("ratio")
        lines.append(
            "ratio: --" if ratio is None else
            f"ratio: {ratio:5.2f}  (enter {cfg.get('enter_ratio', 0.0):.2f}"
            f" / exit {cfg.get('exit_ratio', 0.0):.2f})")
        median = tel.get("median")
        lines.append(
            "median: --" if median is None else f"median: {median:.3f} m")
        valid = tel.get("valid") or 0.0
        min_valid = cfg.get("min_valid_ratio", 0.0)
        if valid < min_valid:
            lines.append(f"! valid {valid*100:.0f}% - sensor cannot see")

    if roi is not None:
        # 각도는 정렬을 껐을 때도 보여준다. --no-wrist 는 손목이 없어
        # 항상 align OFF 인데, 브링업 2단계(각도가 물체를 따라가나)는
        # 바로 그 상태에서 이 숫자를 보고 판단한다.
        angle = tel.get("angle")
        angle_reason = tel.get("angle_reason", "")
        lines.append(
            "angle: --  (%s)" % angle_reason if angle is None else
            f"angle: {angle:+6.1f}  target: {roi['target_angle_deg']:+6.1f}"
            f"  err: "
            f"{orientation.wrap90(angle - roi['target_angle_deg']):+6.1f}")
        align_on = tel.get("align_on", True)
        if not align_on:
            lines.append("align: OFF")
        else:
            wrist_deg = tel.get("wrist_deg")
            align_status = tel.get("align_status")
            lines.append(
                "wrist: --" if wrist_deg is None else
                f"wrist: {wrist_deg:+6.1f}  {align_status or ''}")
        confirm_left = tel.get("confirm_left") or 0.0
        if confirm_left > 0.0:
            lines.append(f"confirm in {confirm_left:.1f}s")
        if not tel.get("has_hand_mask", False):
            lines.append("! no hand mask - press h with the object removed")

    # 아래 둘은 원본에 없다 -- 링크 감시와 무장은 콘솔/데몬 분리로 새로
    # 생긴 개념이다.
    if tel.get("release_in") is not None:
        lines.append(f"! LINK LOST - auto open in {tel['release_in']:.0f}s")
    if not tel.get("armed", True):
        lines.append("DISARMED - press 'm' to arm")

    color = _STATE_COLOR.get(state, _WHITE)
    for i, text in enumerate(lines):
        cv2.putText(image, text, (10, 25 + i * 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    cv2.putText(image, hint, (10, image.shape[0] - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, _WHITE, 1)
    return image


_BACKUP = Path(__file__).with_name("backup") / "roi.json"


def backup_roi(roi, last):
    """텔레메트리의 roi 가 바뀌었으면 PC 에 떨군다. -> 새 last.

    파이가 소유하고 여기는 사본만 둔다. 이 파일을 읽어서 쓰지 않는다 --
    두 곳에서 읽으면 어느 쪽이 진짜인지 모르게 된다. 목적은 파이 SD
    카드가 죽어도 캘리브레이션을 잃지 않는 것뿐이다.
    """
    if roi is None or roi == last:
        return last
    try:
        _BACKUP.parent.mkdir(parents=True, exist_ok=True)
        _BACKUP.write_text(json.dumps(roi, indent=2), encoding="utf-8")
    except OSError as e:
        print(f"[WARN] ROI 백업 실패: {e}")
        return last
    return roi


def _recv_nonblocking(sock, n=65536):
    """넌블로킹 소켓에서 한 번 읽는다. b"" = 지금은 없음, None = 끊김.

    link_sender.DropSender.recv() 와 규약이 같다. 그 모듈은 데몬이
    송신을 막힘 없이 하려고 만든 것이라(콘솔의 Interfaces 에는 없다)
    여기서는 수신 한 방향만 필요한 만큼 얇게 다시 만든다.
    """
    try:
        chunk = sock.recv(n)
    except (BlockingIOError, InterruptedError):
        return b""
    except OSError:
        return None
    return None if chunk == b"" else chunk


def _decode_preview(pf):
    """PreviewFrame -> (컬러 이미지, 불리언 마스크). 디코드 실패시 (None, None)."""
    image = cv2.imdecode(np.frombuffer(pf.jpeg, dtype=np.uint8),
                         cv2.IMREAD_COLOR)
    if image is None:
        return None, None
    mask = None
    if pf.mask_png:
        gray = cv2.imdecode(np.frombuffer(pf.mask_png, dtype=np.uint8),
                            cv2.IMREAD_GRAYSCALE)
        if gray is not None:
            mask = gray > 127
    return image, mask


def _print_ack(msg):
    """ack 를 콘솔에 보여준다. ping 은 매 프레임 오므로 조용히 넘긴다."""
    cmd = msg.get("cmd")
    if cmd == "ping":
        return
    ok = msg.get("ok", False)
    tag = "INFO" if ok else "WARN"
    print(f"[{tag}] {cmd}: {msg.get('msg', '')}")


def main():
    p = argparse.ArgumentParser(
        prog="grasp_console.py",
        description="파지 데몬 콘솔 -- 화면과 키")
    p.add_argument("--host", default="127.0.0.1",
                   help="데몬 주소 (기본 127.0.0.1)")
    p.add_argument("--control-port", type=int, default=CONTROL_PORT)
    p.add_argument("--preview-port", type=int, default=PREVIEW_PORT)
    args = p.parse_args()

    print(f"[INFO] 데몬 접속 시도: {args.host}:{args.control_port}"
          f"/{args.preview_port}")
    ctl = socket.create_connection((args.host, args.control_port), timeout=5)
    pv = socket.create_connection((args.host, args.preview_port), timeout=5)
    ctl.setblocking(False)
    pv.setblocking(False)
    ctl_reader, pv_reader = LineReader(), PreviewReader()
    dragger = RoiDragger()

    # tel/image/mask 는 "지금 화면에 보이는" 세 짝이다. seq 로 맞춘
    # 미리보기가 올 때만 셋이 같이 바뀐다 -- 안 그러면 그림과 숫자가
    # 서로 다른 프레임이 된다.
    tel, image, mask = None, None, None
    pending = {}                    # seq -> 텔레메트리. 그림을 기다린다
    last_roi = None
    last_src = None                 # (src_w, src_h). 드래그 환산에 쓴다

    try:
        ctl.sendall(encode_msg({"cmd": "hello", "proto": PROTO}))
    except OSError as e:
        print(f"[ERROR] 데몬에 인사를 못 보냈다: {e}")
        ctl.close()
        pv.close()
        return 1

    window = "ROI grasp (console)"
    cv2.namedWindow(window)
    cv2.setMouseCallback(window, dragger.on_mouse)
    print(f"[INFO] 접속됨. {_HINT}")

    try:
        while True:
            # --- 1. ping: 데몬 워치독의 먹이다. 2초 안 보내면 무장 해제된다 ---
            try:
                ctl.sendall(encode_msg({"cmd": "ping"}))
            except OSError as e:
                print(f"[WARN] 제어 연결이 끊겼다: {e}")
                break

            # --- 2. 제어 채널 수신: 텔레메트리는 pending 에, ack 는 출력 ---
            chunk = _recv_nonblocking(ctl)
            if chunk is None:
                print("[WARN] 데몬이 제어 연결을 닫았다.")
                break
            if chunk:
                try:
                    for msg in ctl_reader.feed(chunk):
                        if msg.get("t") == "tel":
                            seq = msg.get("seq")
                            if seq is not None:
                                pending[seq] = msg
                                while len(pending) > MAX_PENDING:
                                    del pending[min(pending)]
                            last_roi = backup_roi(msg.get("roi"), last_roi)
                        elif msg.get("t") == "ack":
                            _print_ack(msg)
                except LinkError as e:
                    print(f"[WARN] 제어 프로토콜이 깨졌다: {e}")
                    break

            # --- 3. 미리보기 채널 수신: seq 가 맞는 것만 화면에 올린다 ---
            pchunk = _recv_nonblocking(pv)
            if pchunk is None:
                print("[WARN] 데몬이 미리보기 연결을 닫았다.")
                break
            if pchunk:
                try:
                    for pf in pv_reader.feed(pchunk):
                        last_src = (pf.src_w, pf.src_h)
                        matched = pending.pop(pf.seq, None)
                        if matched is None:
                            # 텔레메트리가 아직 안 왔다. 짝이 안 맞는 그림을
                            # 보여주느니 이전 화면을 그대로 유지한다.
                            continue
                        new_image, new_mask = _decode_preview(pf)
                        if new_image is None:
                            continue
                        tel, image, mask = matched, new_image, new_mask
                except LinkError as e:
                    print(f"[WARN] 미리보기 프로토콜이 깨졌다: {e}")
                    break

            # --- 4. 드래그가 끝났으면 원본 좌표로 환산해 보낸다 ---
            if dragger.result is not None:
                box = dragger.result
                dragger.result = None
                if image is None or last_src is None:
                    print("[WARN] 아직 화면이 없어 ROI 를 보낼 수 없다.")
                else:
                    view_h, view_w = image.shape[:2]
                    src_w, src_h = last_src
                    x, y, w, h = to_source(box, view_w, view_h, src_w, src_h)
                    ctl.sendall(encode_msg(
                        {"cmd": "set_roi", "x": x, "y": y, "w": w, "h": h}))

            # --- 5. 화면 갱신 ---
            if image is not None:
                display = image.copy()
                # 데몬이 미리보기를 축소해 보낼 수 있다(--preview-scale).
                # ROI/축은 원본(src_w) 기준으로 오므로 그릴 때만 줄인다.
                scale = (image.shape[1] / float(last_src[0])
                         if last_src and last_src[0] else 1.0)
                draw_overlay(display, tel or {}, mask, _HINT, scale=scale)
                box = dragger.preview()
                if box is not None:
                    cv2.rectangle(display, (box[0], box[1]),
                                  (box[0] + box[2], box[1] + box[3]),
                                  _YELLOW, 1)
                cv2.imshow(window, display)

            # --- 6. 키 처리 ---
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                try:
                    ctl.sendall(encode_msg({"cmd": "bye"}))
                except OSError:
                    pass
                break
            if key == ord("m"):
                # 무장 토글. 원본에 없던 개념이다. 지금 화면에 보이는
                # armed 값을 뒤집는다 -- 눈에 보이는 상태와 어긋나면
                # 안 되므로 latest 가 아니라 화면과 짝인 tel 을 쓴다.
                armed_now = bool((tel or {}).get("armed"))
                ctl.sendall(encode_msg(
                    {"cmd": "disarm" if armed_now else "arm"}))
            elif key == ord("a"):
                align_now = (tel or {}).get("align_on", True)
                ctl.sendall(encode_msg(
                    {"cmd": "set_align", "on": not align_now}))
            else:
                cmd = key_to_command(key)
                if cmd is not None:
                    ctl.sendall(encode_msg(cmd))
    except KeyboardInterrupt:
        print("\n[INFO] 중단됨")
        try:
            ctl.sendall(encode_msg({"cmd": "bye"}))
        except OSError:
            pass
    finally:
        cv2.destroyAllWindows()
        ctl.close()
        pv.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
