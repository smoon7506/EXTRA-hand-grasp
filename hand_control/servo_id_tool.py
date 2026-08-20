# -*- coding: utf-8 -*-
"""COM11 버스에 붙은 STS 서보를 스캔하고, 필요하면 ID 를 바꾼다.

--- 왜 필요한가 ---
SCS0009 손이 ID 1~10 을 전부 쓴다. 손목 STS3215 가 그 안의 ID 를 갖고
있으면 같은 주소로 둘이 동시에 응답해서 **둘 다** 못 쓴다. 버스가
반이중 단선이라 충돌을 감지할 방법도 없다 -- 그냥 조용히 안 된다.
그래서 손목을 붙이기 전에 실제 ID 를 눈으로 확인해야 한다.

    .venv/Scripts/python.exe hand_control/servo_id_tool.py
    .venv/Scripts/python.exe hand_control/servo_id_tool.py --set-id 11

--- ID 를 바꾸는 것은 EEPROM 쓰기다 ---
전원을 꺼도 남는다. 그래서 --set-id 는 **버스에 서보가 딱 하나만
붙어 있을 때만** 동작한다. 여러 개가 붙은 채로 바꾸면 어느 개체가
바뀌었는지 알 수 없고, 이미 쓰고 있는 ID 와 겹치면 그 순간 둘 다 죽는다.

--- mode 를 같이 찍는 이유 ---
mode=1 은 연속 회전(wheel) 모드다. 그 상태에서 goal position 을 쓰면
속도 명령으로 해석돼 손목이 멈추지 않고 돈다. EEPROM 이라 예전에 wheel
모드로 쓴 개체면 전원을 껐다 켜도 그대로다.
"""

import argparse
import sys

# 손이 쓰는 ID 범위. 손목은 반드시 이 밖이어야 한다.
HAND_ID_RANGE = range(1, 11)

# STS3215 의 model 번호. 다른 값이면 손(SCS0009)이거나 다른 개체다.
STS3215_MODEL = 777


def _one(value):
    """rustypot 의 리스트 반환을 벗긴다 (wrist._one 과 같은 규약)."""
    if isinstance(value, (list, tuple)):
        return value[0]
    return value


def _try(fn, *args):
    """읽기 하나가 실패해도 스캔 전체가 죽지 않게 한다. -> 값 또는 None."""
    try:
        return _one(fn(*args))
    except Exception:
        return None


def alive(controller, servo_id):
    """이 ID 가 응답하나. ping 은 False 를 주기도 하고 예외도 던진다."""
    try:
        return bool(controller.ping(servo_id))
    except Exception:
        return False


def scan(controller, ids):
    """살아 있는 서보 목록. [{id, model, mode, position}, ...]"""
    found = []
    for servo_id in ids:
        if not alive(controller, servo_id):
            continue
        found.append({
            "id": servo_id,
            "model": _try(controller.read_model, servo_id),
            "mode": _try(controller.read_mode, servo_id),
            "position": _try(controller.read_present_position, servo_id),
        })
    return found


def describe(entry):
    """스캔 한 줄을 사람이 읽는 문자열로."""
    model, mode = entry["model"], entry["mode"]
    kind = "STS3215" if model == STS3215_MODEL else f"model={model}"
    warn = ""
    if mode not in (0, None):
        warn = "  <-- !! wheel 모드. 위치 명령이 속도로 해석된다"
    elif entry["id"] in HAND_ID_RANGE and model == STS3215_MODEL:
        warn = "  <-- !! 손이 쓰는 ID 범위(1~10)와 겹친다"
    pos = entry["position"]
    pos_text = "--" if pos is None else f"{pos:+.3f} rad"
    return (f"  ID {entry['id']:>3}  {kind:<12} mode={mode}  "
            f"pos={pos_text}{warn}")


def set_id(controller, found, new_id):
    """버스에 하나만 붙어 있을 때 그 서보의 ID 를 바꾼다. -> 바뀐 ID.

    여러 개가 붙어 있으면 아무것도 하지 않는다. 어느 개체가 바뀌었는지
    알 수 없는 상태를 만드는 것보다 거절하는 쪽이 낫다.
    """
    if len(found) != 1:
        raise RuntimeError(
            f"버스에 서보가 {len(found)}개 붙어 있습니다. ID 변경은 손목 "
            f"하나만 남기고(다른 커넥터를 뽑고) 실행하세요 -- 여러 개가 "
            f"붙은 채로 바꾸면 어느 개체가 바뀌었는지 알 수 없고, 이미 "
            f"쓰는 ID 와 겹치면 그 순간 둘 다 응답하지 않습니다."
        )
    old_id = found[0]["id"]
    if new_id in HAND_ID_RANGE:
        raise ValueError(
            f"새 ID {new_id} 는 손이 쓰는 범위(1~10)입니다. 손목은 11 "
            f"이상이어야 합니다."
        )
    if old_id == new_id:
        return new_id

    # EEPROM 은 lock 을 풀어야 써진다. 쓰고 나면 반드시 다시 잠근다 --
    # 잠그지 않으면 이후의 평범한 쓰기가 EEPROM 을 갉아먹는다.
    controller.write_lock(old_id, 0)
    try:
        controller.write_id(old_id, new_id)
    finally:
        try:
            controller.write_lock(new_id, 1)
        except Exception:
            controller.write_lock(old_id, 1)
    return new_id


def _make_controller(port, baudrate, timeout_s):
    from rustypot import Sts3215PyController
    return Sts3215PyController(serial_port=port, baudrate=baudrate,
                               timeout=timeout_s)


def main(argv=None):
    import hand_config

    p = argparse.ArgumentParser(
        prog="servo_id_tool.py",
        description="STS 버스 스캔 / 손목 서보 ID 변경")
    p.add_argument("--first", type=int, default=1, help="스캔 시작 ID")
    p.add_argument("--last", type=int, default=20, help="스캔 끝 ID")
    p.add_argument("--set-id", type=int, default=None,
                   help="찾은 서보의 ID 를 이 값으로 바꾼다 (EEPROM 쓰기). "
                        "버스에 서보가 하나만 붙어 있어야 한다")
    args = p.parse_args(argv)

    print(f"[INFO] {hand_config.SERIAL_PORT} 스캔 "
          f"(ID {args.first}~{args.last})...")
    controller = _make_controller(hand_config.SERIAL_PORT,
                                  hand_config.BAUDRATE,
                                  hand_config.SERIAL_TIMEOUT_S)
    found = scan(controller, range(args.first, args.last + 1))
    if not found:
        print("[ERROR] 응답하는 서보가 없습니다. 서보 전원부터 확인하세요 "
              "-- 전원이 없으면 COM 포트는 정상으로 보이면서 통신만 "
              "실패합니다.")
        return 1

    print(f"[INFO] 서보 {len(found)}개:")
    for entry in found:
        print(describe(entry))

    if args.set_id is None:
        return 0

    try:
        new_id = set_id(controller, found, args.set_id)
    except (RuntimeError, ValueError) as e:
        print(f"[ERROR] {e}")
        return 1
    print(f"[INFO] ID 를 {new_id} 로 바꿨습니다. 확인합니다...")
    again = scan(controller, [new_id])
    if not again:
        print(f"[WARN] 새 ID {new_id} 가 응답하지 않습니다. 전원을 껐다 "
              f"켜고 다시 스캔해 보세요.")
        return 1
    print(describe(again[0]))
    print(f"[INFO] hand_config.WRIST_ID 를 {new_id} 로 적으세요.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
