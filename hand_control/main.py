# -*- coding: utf-8 -*-
"""a(굽힘) / s(벌림) 값으로 EXTRA Hand 를 움직인다.

  python main.py --dry-run     모터 없이 목표 각도만 출력
  python main.py               실물 손

입력 형식은 두 모드가 같다:  'a' 또는 'a s'

MuJoCo 시뮬레이션 모드는 이 저장소에 없다. 씬 파일이 AmazingHand 데모
트리에 있고 파지 시스템에는 필요 없어서 뺐다.

--- 첫 실행 순서 (하드웨어를 쓸 때) ---
  1) --dry-run 으로 숫자 확인
  2) hand_config.ACTIVE_FINGERS 를 ["r_finger1"] 로 줄이고 a=0 부터
  3) 방향/한계 확인 후 손가락을 늘린다
  손 안팎에 아무것도 없는 상태에서만 할 것.
"""

import argparse
import sys
import threading
import time

import hand_config
import sequence
from hand import Hand, preview_pose

# 이 파일은 한글로 출력한다. 콘솔 코드페이지가 UTF-8 이 아니면
# UnicodeEncodeError 로 죽는다. tactile_motor_test/main.py:21-24 와 같은 패턴.
try:
    sys.stdout.reconfigure(encoding="utf-8")
except (AttributeError, ValueError):
    pass


def parse_input(text):
    """입력 문자열 -> (a, s) 또는 None(종료).

    허용 형식:
        "0.3"       -> (0.3, 0.0)   s 를 생략하면 0
        "0.3 -0.5"  -> (0.3, -0.5)
        "q"         -> None

    범위를 벗어나면 ValueError 를 낸다. kinematics 는 범위 밖 값을 조용히
    잘라주지만, 여기서는 일부러 거절한다. 사람이 0.5 를 치려다 5 를 쳤을 때
    "잘렸습니다"보다 "잘못 입력했습니다"가 훨씬 도움이 되기 때문이다.
    """
    text = text.strip()
    if not text:
        raise ValueError("값을 입력하세요.")
    if text.lower() == "q":
        return None

    parts = text.split()
    if len(parts) > 2:
        raise ValueError("값은 최대 2개입니다 (a 또는 'a s').")
    try:
        nums = [float(p) for p in parts]
    except ValueError:
        raise ValueError(f"숫자가 아닙니다: {text!r}")

    a = nums[0]
    s = nums[1] if len(nums) == 2 else 0.0
    if not 0.0 <= a <= 1.0:
        raise ValueError(f"a 는 0 ~ 1 범위입니다 (받은 값 {a}). "
                         f"손가락은 뒤로 젖혀지지 않습니다.")
    if not -1.0 <= s <= 1.0:
        raise ValueError(f"s 는 -1 ~ 1 범위입니다 (받은 값 {s}).")
    return (a, s)


def _load():
    """활성 손가락을 읽는다. 실패하면 메시지를 찍고 None."""
    try:
        return hand_config.load_fingers(active=hand_config.ACTIVE_FINGERS)
    except (OSError, ValueError, KeyError) as e:
        print(f"[ERROR] 손가락 설정을 읽지 못했습니다: {e}")
        print(f"        toml 경로: {hand_config.HAND_TOML}")
        return None


def _print_header(fingers):
    print(f"[INFO] 활성 손가락 {len(fingers)}개: "
          f"{', '.join(f.name for f in fingers)}")
    print(f"[INFO] FLEX_LIMIT={hand_config.FLEX_LIMIT_DEG}도  "
          f"SPREAD_LIMIT={hand_config.SPREAD_LIMIT_DEG}도  "
          f"모터 범위 {hand_config.MOTOR_MIN_DEG}~{hand_config.MOTOR_MAX_DEG}도")


def _print_usage():
    print("\n값을 입력하세요.  'a' 또는 'a s'  (a: 0~1 굽힘, s: -1~1 벌림)")
    print("예)  0.2      0.5 -1      끝내려면  q\n")


def _connect_hand(fingers):
    """실물 손에 연결한다. 실패하면 None (메시지는 여기서 출력)."""
    hand = Hand(fingers)
    try:
        hand.connect()
    except BaseException as e:
        # connect() 는 시리얼 쓰기를 여러 번 한다. 그 중간에 Ctrl+C 가
        # 들어오면 KeyboardInterrupt 는 Exception 의 하위가 아니므로
        # except Exception 으로는 못 잡고 release() 를 건너뛴 채 죽는다
        # -- 모터가 토크 걸린 채 방치된다. 그래서 BaseException 을 잡는다.
        print(f"[ERROR] 모터 연결 실패 (포트 {hand_config.SERIAL_PORT}): {e}")
        print("        서보 전원이 켜져 있는지 먼저 확인하세요. 전원이 없으면")
        print("        COM 포트는 정상으로 보이면서 통신만 실패합니다.")
        # 일부 모터만 토크가 켜진 상태로 실패했을 수 있다.
        # release() 는 self._c 가 None 이면 스스로 no-op 이라 항상 안전하다.
        hand.release()
        return None
    return hand


# ==============================================================
#  모드 1: 숫자만
# ==============================================================

def run_dry():
    """모터 없이 목표 각도만 출력한다."""
    fingers = _load()
    if fingers is None:
        return 1

    _print_header(fingers)
    print("\n[INFO] 모터에 아무것도 보내지 않습니다. 숫자만 확인하세요.\n")

    print("=== 굽힘 (s=0) ===")
    for a in (0.0, 0.25, 0.5, 0.75, 1.0):
        print(f"a = {a}")
        print(preview_pose(a, 0.0, fingers))

    print("\n=== 벌림 (a=0) ===")
    for s in (-1.0, 0.0, 1.0):
        print(f"s = {s:+}")
        print(preview_pose(0.0, s, fingers))

    print("\n=== 굽힘+벌림 동시 (합쳐서 한계를 넘는지 확인) ===")
    for a, s in ((1.0, 1.0), (1.0, -1.0)):
        print(f"a = {a}, s = {s:+}")
        print(preview_pose(a, s, fingers))

    print("\n=== 파지 순서 (a 증가) ===")
    print(sequence.format_schedule(fingers, closing=True))
    print("=== 폄 순서 (a 감소) ===")
    print(sequence.format_schedule(fingers, closing=False))
    print("[HINT] 순서를 뒤집으려면 hand_config 의 CLOSE_DELAY_S 와")
    print("       OPEN_DELAY_S 를 통째로 맞바꾸면 된다.")

    print("\n[HINT] a=0 일 때 나오는 값이 '편 손' 자세입니다. 0도가 아니라")
    print("       오프셋 값인 게 정상입니다 (r_hand.toml 의 캘리브레이션 영점).")
    return 0


def main():
    p = argparse.ArgumentParser(
        prog="main.py",
        description="a(굽힘)/s(벌림) 값으로 EXTRA Hand 손가락 구동",
    )
    g = p.add_mutually_exclusive_group()
    g.add_argument("--dry-run", action="store_true",
                   help="모터 없이 목표 각도만 출력")
    args = p.parse_args()

    if args.dry_run:
        return run_dry()
    return run_interactive()


if __name__ == "__main__":
    sys.exit(main())