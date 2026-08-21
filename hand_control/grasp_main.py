# -*- coding: utf-8 -*-
"""강성 기반 적응 파지.

물체를 잡으면서 얼마나 딱딱한지 스스로 판단하고, 그 판단에 맞는
파지력을 손가락별로 유지한다.

  python grasp_main.py --sensor-only   센서 값만 (모터 안 켬)
  python grasp_main.py --sensor-diag   전단력/taxel 분포 (슬립 신호 확인)
  python grasp_main.py --dry-run       가짜 물체로 전체 사이클 시뮬레이션
  python grasp_main.py                 실물 파지

--- 첫 실행 순서 (반드시 이 순서로) ---
  1) --sensor-only 로 손가락을 하나씩 눌러 SENSOR_CHANNEL_MAP 확인
     이게 틀리면 검지 힘을 보고 새끼를 조인다
  2) --dry-run 으로 상태 전이 확인
  3) hand_config.ACTIVE_FINGERS 를 ["r_finger1"] 로 줄이고 실물
  4) 로그를 보고 K_THRESHOLD 결정
  5) 손가락을 늘린다
"""

import argparse
import math
import sys
import time

import hand_config
import grasp
from grasp import FingerGrasp, GraspParams
from grasp_log import GraspLogger
from grasp_runner import ForceGraspRunner, flex_span as _flex_span
from hand import Hand
from tactile import TactileHand

# 콘솔 코드페이지가 UTF-8 이 아니면 한글 출력이 UnicodeEncodeError 로
# 죽는다. main.py:29-32 와 같은 패턴.
try:
    sys.stdout.reconfigure(encoding="utf-8")
except (AttributeError, ValueError):
    pass


def _load():
    try:
        return hand_config.load_fingers(active=hand_config.ACTIVE_FINGERS)
    except (OSError, ValueError, KeyError) as e:
        print(f"[ERROR] 손가락 설정을 읽지 못했습니다: {e}")
        print(f"        toml 경로: {hand_config.HAND_TOML}")
        return None


def _make_states(fingers):
    params = GraspParams.from_config()
    return {f.name: FingerGrasp(f.name, _flex_span(f), params)
            for f in fingers}


def _print_status(t, states, forces, flex):
    parts = []
    for name, finger in states.items():
        force = forces.get(name)
        parts.append(
            f"{name[-7:]}:{finger.state[:4]} "
            f"a={finger.a:.3f} "
            f"F={'--' if force is None else f'{force:5.2f}'}"
        )
    print(f"[{t:6.2f}s] " + " | ".join(parts))


# ==============================================================
#  모드 1: 센서만  (채널 매핑 검증용)
# ==============================================================

def run_sensor_only():
    """모터를 켜지 않고 센서 5개 값만 출력한다.

    이 모드의 목적은 힘을 보는 게 아니라 SENSOR_CHANNEL_MAP 을
    검증하는 것이다. 손가락을 하나씩 눌러 보면서 눌린 손가락의 값만
    올라가는지 확인한다.
    """
    sensors = TactileHand()
    print("[INFO] 센서 드라이버를 시작합니다...")
    sensors.start()

    if not sensors.wait_ready(hand_config.SENSOR_TIMEOUT_S):
        print(f"[ERROR] {hand_config.SENSOR_TIMEOUT_S}초 안에 센서 데이터가 "
              f"오지 않았습니다.")
        print("        CH341 USB 어댑터와 센서 전원을 확인하세요.")
        sensors.stop()
        return 1

    found = sensors.discover(hand_config.DISCOVER_S)
    print(f"[INFO] 붙어 있는 센서 {len(found)}개: {', '.join(sorted(found))}")
    missing = sorted(set(hand_config.SENSOR_CHANNEL_MAP.values()) - set(found))
    if missing:
        print(f"[WARN] 데이터가 안 오는 손가락: {missing}")

    # baseline 을 먼저 잡고 나서 "눌러 보라"고 안내한다. 순서가 바뀌면
    # 안내를 그대로 따른 사용자가 baseline 을 잡는 도중에 센서를 눌러
    # 그 채널의 무부하 평균이 오염된다. 오염된 채널은 세션 내내 0으로
    # 읽혀 채널 매핑 검증(이 모드의 목적)이 통째로 무의미해진다.
    print("[INFO] baseline 을 잡습니다. 센서를 만지지 마세요...")
    if not sensors.calibrate():
        print("[WARN] baseline 을 못 잡았습니다. 0 으로 진행합니다.")

    print("\n[INFO] 손가락을 하나씩 눌러 보세요.")
    print("       눌린 손가락의 값만 올라가야 합니다. 다른 손가락이 "
          "올라가면")
    print("       hand_config.SENSOR_CHANNEL_MAP 을 고쳐야 합니다.")
    print("       Ctrl+C 로 종료.\n")

    names = [hand_config.SENSOR_CHANNEL_MAP[c]
             for c in sorted(hand_config.SENSOR_CHANNEL_MAP)]
    print("\n" + "  ".join(f"{n:>10}" for n in names))
    try:
        while True:
            forces = sensors.read_forces()
            cells = []
            for name in names:
                value = forces.get(name)
                cells.append("      ----" if value is None
                             else f"{value:10.3f}")
            print("  ".join(cells), end="\r", flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n[INFO] 중단됨")
    finally:
        sensors.stop()
    return 0


# ==============================================================
#  모드 1.5: 센서 진단  (전단력 / taxel 분포가 살아 있는지)
# ==============================================================

_BARS = " ▁▂▃▄▅▆▇█"


def _bar(value, top):
    """0~top 을 블록 문자 하나로. top 이 0 이면 공백."""
    if top <= 0.0:
        return _BARS[0]
    level = int(round(min(1.0, max(0.0, value / top)) * (len(_BARS) - 1)))
    return _BARS[level]


def _centroid(deltas, coords):
    """taxel 압력 중심 (col, row). 눌린 데가 없으면 None.

    물체가 미끄러지면 접촉 패치가 손가락 위에서 이동한다. 그 이동이
    슬립 신호가 될 수 있는지 보려고 계산한다.
    """
    total = sum(deltas)
    if total <= 0.0:
        return None
    col = sum(d * c for d, (c, _) in zip(deltas, coords)) / total
    row = sum(d * r for d, (_, r) in zip(deltas, coords)) / total
    return (col, row)


def _taxel_coords(n):
    """taxel 좌표 -> (좌표 목록, 출처 문자열).

    드라이버의 tactile_geometry 를 쓴다. 거기에 실제 PCB 배치
    (TIP13_LAYOUT: 팁 1점 + 3x4 격자)가 들어 있어서, 우리가 다시 적으면
    조용히 어긋난다.

    출처를 같이 돌려주는 이유: import 가 실패하면 격자로 대체되는데,
    그러면 중심 좌표가 실제 손가락 위 위치와 다른 뜻이 된다. 그걸 모른 채
    숫자를 읽으면 안 된다. import 는 CAPREAD_DIR 이 sys.path 에 들어간
    뒤에만 되므로(tactile._import_driver), start() 뒤에 불러야 한다.
    """
    if n <= 0:
        return ([], "없음")
    try:
        import tactile_geometry
        return (tactile_geometry.taxel_layout(n), "PCB 실제 배치")
    except Exception as e:
        rows = math.ceil(n / 3)
        grid = [(i % 3, rows - 1 - i // 3) for i in range(n)]
        return (grid, f"3열 격자로 대체 ({e})")


def run_sensor_diag(finger_name=None):
    """한 손가락의 tf(전단력) / tfDir / taxel 분포를 자세히 본다.

    목적은 힘을 보는 게 아니라 **슬립 신호로 쓸 수 있는 채널이 실물에서
    살아 있는지** 확인하는 것이다. 드라이버는 매 프레임 tf 와 taxel 별
    정전용량을 보내지만(cap_read.pack_from_finger), 개체에 따라 tf 가
    0 으로만 나올 수 있어서 실측 없이는 설계를 못 정한다.

    --- 보는 법 ---
    손가락 위에 물체를 올리고 **옆으로 밀어 본다**.
      nf 는 그대로인데 tf 가 오르면      -> 전단력이 살아 있다
      tf 가 0 만 나오면                  -> taxel 중심 이동을 대신 쓴다
      둘 다 안 움직이면                  -> 힘 하강(load drop)으로 후퇴
    """
    sensors = TactileHand()
    print("[INFO] 센서 드라이버를 시작합니다...")
    sensors.start()
    if not sensors.wait_ready(hand_config.SENSOR_TIMEOUT_S):
        print(f"[ERROR] {hand_config.SENSOR_TIMEOUT_S}초 안에 센서 데이터가 "
              f"오지 않았습니다.")
        sensors.stop()
        return 1

    found = sensors.discover(hand_config.DISCOVER_S)
    if not found:
        print("[ERROR] 붙어 있는 센서가 없습니다.")
        sensors.stop()
        return 1

    if finger_name is None:
        finger_name = (hand_config.ACTIVE_FINGERS[0]
                       if hand_config.ACTIVE_FINGERS[0] in found
                       else sorted(found)[0])
    if finger_name not in found:
        print(f"[ERROR] {finger_name} 에서 데이터가 오지 않습니다.")
        print(f"        붙어 있는 손가락: {', '.join(sorted(found))}")
        sensors.stop()
        return 1

    # --- 정적 정보. nf/tf 배열 길이와 스케일이 여기 달려 있다 ---
    info = sensors.read_raw().get(finger_name, {})
    touch_num = info.get("touchNum")
    channels = info.get("channelCapData") or []
    print(f"\n[INFO] {finger_name}")
    print(f"       프로젝트   : {info.get('prjName')}")
    print(f"       채널 수    : sensorNum={info.get('sensorNum')} "
          f"touchNum={touch_num} (실제 수신 {len(channels)}개)")
    print(f"       nf 배열    : {len(info.get('nf') or [])}개  "
          f"tf 배열: {len(info.get('tf') or [])}개")
    if not info.get("tf"):
        print("       [WARN] tf 배열이 비어 있습니다. 이 개체는 전단력을 "
              "안 보내는 것으로 보입니다.")

    print("\n[INFO] baseline 을 잡습니다. 센서를 만지지 마세요...")
    if not sensors.calibrate():
        print("[WARN] nf baseline 을 못 잡았습니다. 0 으로 진행합니다.")

    # taxel 원시값은 DC 오프셋이 커서 따로 baseline 을 잡아야 변화가 보인다.
    sums, counts = {}, 0
    deadline = time.monotonic() + 1.5
    while time.monotonic() < deadline:
        entry = sensors.read_raw().get(finger_name)
        if entry and entry.get("channelCapData"):
            for i, value in enumerate(entry["channelCapData"]):
                sums[i] = sums.get(i, 0.0) + float(value)
            counts += 1
        time.sleep(0.02)
    ch_base = ({i: s / counts for i, s in sums.items()} if counts else {})

    n_taxel = touch_num if touch_num else len(ch_base)
    coords, layout_src = _taxel_coords(n_taxel)
    print(f"[INFO] taxel {n_taxel}개, 배치: {layout_src}")

    print("\n[INFO] 손가락 위에 물체를 올리고 **옆으로 밀어** 보세요.")
    print("       nf 는 그대로인데 tf 가 오르면 전단력이 살아 있는 것입니다.")
    print("       Ctrl+C 로 종료.\n")
    print(f"{'nf(N)':>7}{'tf':>8}{'tf/nf':>7}{'tfDir':>7}  "
          f"taxel Δ{'':<{max(0, n_taxel - 7)}}  중심(col,row)   "
          f"peak: tf / tf∙nf⁻¹")

    peak_tf = 0.0
    peak_ratio = 0.0
    try:
        while True:
            entry = sensors.read_raw().get(finger_name)
            if entry is None:
                time.sleep(0.1)
                continue
            forces = sensors.read_forces()
            nf = forces.get(finger_name)
            tf_list = entry.get("tf") or []
            tf = float(tf_list[0]) if tf_list else 0.0
            dirs = entry.get("tfDir") or []
            tf_dir = dirs[0] if dirs else "--"

            ratio = (tf / nf) if (nf and nf > 0.05) else 0.0
            peak_tf = max(peak_tf, tf)
            peak_ratio = max(peak_ratio, ratio)

            raw_ch = entry.get("channelCapData") or []
            deltas = [max(0.0, float(v) - ch_base.get(i, 0.0))
                      for i, v in enumerate(raw_ch[:n_taxel])]
            top = max(deltas) if deltas else 0.0
            bars = "".join(_bar(d, top) for d in deltas)
            center = _centroid(deltas, coords) if coords else None
            center_text = ("   --  ,  --  " if center is None
                           else f" {center[0]:5.2f},{center[1]:5.2f} ")

            nf_text = "--" if nf is None else f"{nf:.2f}"
            print(f"{nf_text:>7}"
                  f"{tf:8.2f}{ratio:7.2f}{str(tf_dir):>7}  "
                  f"{bars}  {center_text}  "
                  f"{peak_tf:5.2f} / {peak_ratio:5.2f}",
                  end="\r", flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n[INFO] 중단됨")
    finally:
        sensors.stop()

    print(f"\n--- 요약 ---")
    print(f"  tf 최대       : {peak_tf:.3f}")
    print(f"  tf/nf 최대    : {peak_ratio:.3f}")
    if peak_tf <= 0.0:
        print("  => tf 가 한 번도 안 움직였습니다. 전단력 채널은 못 씁니다.")
        print("     taxel 중심 이동이 보였다면 그쪽으로, 아니면 힘 하강")
        print("     (위치 고정인데 힘이 빠짐)으로 슬립을 잡아야 합니다.")
    else:
        print("  => tf 가 살아 있습니다. tf/nf 가 마찰계수에 근접하는")
        print("     지점이 슬립 임계값 후보입니다.")
    return 0


# ==============================================================
#  모드 2: dry-run  (하드웨어 없이 전체 사이클)
# ==============================================================

class _FakeObject:
    """가짜 물체. a 가 접촉점을 넘은 만큼 힘을 낸다.

    force = k_true * (a - a_contact) * flex_span
    flex  = (a - a_contact) * flex_span     (접촉 후에는 물체가 막는다)
    """

    def __init__(self, k_true, a_contact, flex_span):
        self.k_true = k_true
        self.a_contact = a_contact
        self.flex_span = flex_span

    def sense(self, a):
        if a <= self.a_contact:
            return (0.0, a * self.flex_span)
        # 접촉 후에는 명령 a 가 늘어도 관절은 물체 강성만큼만 들어간다.
        # 강체일수록 flex 가 거의 안 늘어난다 -- 이걸 재현해야 dry-run 이
        # 의미가 있다.
        over = a - self.a_contact
        penetration = over / (1.0 + self.k_true / 5.0)
        flex = (self.a_contact + penetration) * self.flex_span
        force = self.k_true * penetration * self.flex_span
        return (force, flex)


def run_dry():
    """하드웨어 없이 상태기계 전체를 돌린다. 통합 테스트 역할."""
    fingers = _load()
    if fingers is None:
        return 1

    print(f"[INFO] 활성 손가락 {len(fingers)}개: "
          f"{', '.join(f.name for f in fingers)}")
    print(f"[INFO] K_THRESHOLD = {hand_config.K_THRESHOLD}")
    if hand_config.K_THRESHOLD is None:
        # stiffness.classify 는 임계값이 없으면 무조건 "soft" 를 준다.
        # 모르는 상태에서는 약하게 잡는 쪽이 안전하기 때문이다.
        print("       (아직 실측 전이라 전부 soft 로 분류됩니다)")

    for label, k_true in (("스펀지 (k=2)", 2.0),
                          ("종이컵 (k=15)", 15.0),
                          ("나무토막 (k=200)", 200.0)):
        print(f"\n{'=' * 62}\n  가짜 물체: {label}\n{'=' * 62}")
        states = _make_states(fingers)
        objects = {f.name: _FakeObject(k_true, 0.25, _flex_span(f))
                   for f in fingers}

        dt = 1.0 / hand_config.LOOP_HZ
        t = 0.0
        for step in range(int(20.0 / dt)):
            forces, flexes = {}, {}
            for f in fingers:
                force, flex = objects[f.name].sense(states[f.name].a)
                forces[f.name], flexes[f.name] = force, flex
            for f in fingers:
                states[f.name].update(forces[f.name], flexes[f.name], t, dt)
            if step % 20 == 0:
                _print_status(t, states, forces, flexes)
            t += dt

        print("  --- 결과 ---")
        for f in fingers:
            s = states[f.name]
            k_text = "--" if s.k_hat is None else f"{s.k_hat:.2f}"
            print(f"    {f.name:<10} {s.state:<10} "
                  f"k_hat={k_text:>8} N/rad  "
                  f"confident={s.confident}  "
                  f"class={s.object_class}  a={s.a:.3f}")

    print(f"\n[HINT] 위 세 물체의 k_hat 이 확실히 갈라지면 그 사이 값을")
    print(f"       K_THRESHOLD 후보로 삼을 수 있습니다. 다만 이건 가짜")
    print(f"       물체라 실물 값과 다릅니다. 반드시 실물 로그로 정하세요.")
    return 0


# ==============================================================
#  모드 3: 실물
# ==============================================================

def run_live():
    fingers = _load()
    if fingers is None:
        return 1
    print(f"[INFO] 활성 손가락 {len(fingers)}개: "
          f"{', '.join(f.name for f in fingers)}")

    # --- 센서 먼저. 모터를 켜기 전에 센서가 살아 있는지 확인한다 ---
    sensors = TactileHand()
    print("[INFO] 센서 드라이버를 시작합니다...")
    sensors.start()
    if not sensors.wait_ready(hand_config.SENSOR_TIMEOUT_S):
        print("[ERROR] 센서 데이터가 오지 않습니다. 모터는 켜지 않습니다.")
        sensors.stop()
        return 1
    found = sensors.discover(hand_config.DISCOVER_S)
    print(f"[INFO] 센서 {len(found)}개: {', '.join(sorted(found))}")

    need = {f.name for f in fingers}
    missing = sorted(need - set(found))
    if missing:
        print(f"[ERROR] 활성 손가락 중 센서가 없는 것: {missing}")
        print("        센서 없이 힘 제어를 할 수 없습니다. "
              "hand_config.ACTIVE_FINGERS 를 줄이거나 배선을 확인하세요.")
        sensors.stop()
        return 1

    print("[INFO] baseline 을 잡습니다. 손에 아무것도 닿지 않게 하세요...")
    if not sensors.calibrate():
        print("[ERROR] baseline 을 못 잡았습니다.")
        sensors.stop()
        return 1

    # --- 모터 ---
    hand = Hand(fingers)
    try:
        hand.connect()
    except BaseException as e:
        # connect() 중간에 Ctrl+C 가 들어오면 KeyboardInterrupt 는
        # Exception 의 하위가 아니라 except Exception 으로 못 잡고
        # release() 를 건너뛴 채 죽는다 -- 모터가 토크 걸린 채 방치된다.
        print(f"[ERROR] 모터 연결 실패 (포트 {hand_config.SERIAL_PORT}): {e}")
        print("        서보 전원부터 확인하세요. 전원이 없으면 COM 포트는")
        print("        정상으로 보이면서 통신만 실패합니다.")
        hand.release()
        sensors.stop()
        return 1

    # connect() 가 성공한 순간부터 10개 서보에 토크가 걸린다. 여기부터
    # 루프 진입 전까지(GraspParams.from_config() 의 AttributeError,
    # 콘솔 코드페이지 문제로 인한 UnicodeEncodeError, 도중의 Ctrl+C 등)
    # 뭐가 터지든 finally 의 release() 가 반드시 돌게 try 를 여기서
    # 바로 연다 -- 그 사이에 죽으면 손이 토크 걸린 채 방치된다.
    runner = None
    try:
        runner = ForceGraspRunner(fingers, sensors, hand)

        print(f"\n[INFO] 파지를 시작합니다. Ctrl+C 로 중단.")
        print(f"[INFO] 힘 상한 {hand_config.F_ABORT}N, "
              f"온도 한계 {hand_config.TEMP_LIMIT_C}도\n")

        runner.start_grasp()
        print(f"[INFO] 로그: {runner.log_path}\n")
        dt = 1.0 / hand_config.LOOP_HZ
        start = time.monotonic()

        while True:
            cycle = time.monotonic()
            status = runner.tick()
            if status == "abort":
                break

            t = cycle - start
            if int(t * hand_config.LOOP_HZ) % 10 == 0:
                _print_status(t, runner.states,
                              sensors.read_forces(), hand.read_flex())

            # 러너가 스스로 레이트리밋을 하므로 여기서는 CPU 만 아낀다.
            time.sleep(max(0.0, dt - (time.monotonic() - cycle)))
    except KeyboardInterrupt:
        print("\n[INFO] 중단됨")
    finally:
        # runner 가 만들어지기 전에 터질 수 있다 (ForceGraspRunner 생성자의
        # GraspParams.from_config() 가 설정 오류로 죽는 경우). None 검사를
        # 빼면 그 상황에서 NameError 가 나면서 hand.release() 까지 건너뛴다..
        if runner is not None:
            runner.close()
        hand.release()
        sensors.stop()
        print("[INFO] 편 자세로 되돌리고 토크를 껐습니다.")
    return 0


def main():
    p = argparse.ArgumentParser(
        prog="grasp_main.py",
        description="강성 기반 적응 파지 (촉각 센서 + 손가락별 힘 제어)",
    )
    g = p.add_mutually_exclusive_group()
    g.add_argument("--sensor-only", action="store_true",
                   help="모터 없이 센서 값만 출력 (채널 매핑 검증용)")
    g.add_argument("--sensor-diag", nargs="?", const="", metavar="FINGER",
                   help="한 손가락의 전단력(tf)과 taxel 분포까지 자세히. "
                        "슬립 신호로 쓸 채널이 살아 있는지 확인용. "
                        "이름을 생략하면 ACTIVE_FINGERS 의 첫 손가락")
    g.add_argument("--dry-run", action="store_true",
                   help="하드웨어 없이 가짜 물체로 전체 사이클 시뮬레이션")
    args = p.parse_args()

    if args.sensor_only:
        return run_sensor_only()
    if args.sensor_diag is not None:
        return run_sensor_diag(args.sensor_diag or None)
    if args.dry_run:
        return run_dry()
    return run_live()


if __name__ == "__main__":
    sys.exit(main())
