# -*- coding: utf-8 -*-
"""hand_control 설정값 + r_hand.toml 로더.

조정할 값은 전부 여기에만 둔다. tactile_motor_test/config.py 와 같은 규약.

오프셋은 여기에 적지 않는다. r_hand.toml 에서 읽는다.
tactile_motor_test/config.py 에서 ID 는 finger1 로 바꿨는데 오프셋은
finger4/5 값이 남아 있던 버그가 있었다. 손가락이 10개로 늘면 그 실수가
5배로 늘어나므로, 진실의 출처를 toml 하나로 고정한다.
"""

import math
import os
import sys
import tomllib
from dataclasses import dataclass
from pathlib import Path

# --- 하드웨어 ---
# PC(윈도우)와 라즈베리파이(리눅스)가 같은 파일을 쓴다. 파일을 옮길 때마다
# 고쳐 넣으면 scp 한 번에 다시 덮여서 조용히 옛 값으로 돌아간다.
#
# 포트 번호가 다르면(URT-1 을 다른 자리에 꽂았거나 USB 장치가 여럿) 아래
# 환경변수로 덮는다:
#   리눅스: export HAND_SERIAL_PORT=/dev/ttyUSB1
#   윈도우: $env:HAND_SERIAL_PORT = "COM12"
#
# 파이에서 이름이 바뀌는 게 싫으면 /dev/serial/by-id/ 아래 고정 이름을 쓴다.
SERIAL_PORT = os.environ.get(
    "HAND_SERIAL_PORT",
    "/dev/ttyUSB0" if sys.platform.startswith("linux") else "COM11")
BAUDRATE = 1000000
SERIAL_TIMEOUT_S = 0.5
GOAL_SPEED = 3  # robot hand speed

# 모터 ID 와 오프셋. **이 저장소가 이 파일의 주인이다.**
#
# 원래는 다른 저장소의 데모 트리 안에 있는 r_hand.toml 을 절대경로로 가리켰다.
# 그 트리를 여기 같이 두지 않으므로 config/ 로 옮겼다.
#
# 사본을 두 곳에 두지 말 것 -- 예전에 오프셋 사본이 원본과 어긋나 손가락이
# 엉뚱한 각도로 간 적이 있다. 손을 새로 조립했거나 서보를 바꿨으면
# **config/r_hand.toml 만** 고친다.
#
# 다른 손(예: 왼손)을 쓰려면 환경변수로 바꾼다:
#   export HAND_TOML=/path/to/l_hand.toml
HAND_TOML = Path(os.environ.get(
    "HAND_TOML",
    Path(__file__).resolve().parent.parent / "config" / "r_hand.toml"))


# --- 가동 범위 (도) ---
# 2026-07-31 실물 확인 완료. finger1 하나로 방향/한계를 잡은 뒤 5개로 늘려
# 아래 값에서 무리 없이 동작하는 것을 확인했다.
# 바꿀 일이 생기면 --dry-run 으로 숫자를 먼저 보고, 손가락 1개부터 다시 올릴 것.
FLEX_LIMIT_DEG = 80.0     # a=1.0 일 때의 굽힘 각도
SPREAD_LIMIT_DEG = 30.0   # s=±1.0 일 때의 벌림 각도

# 두 성분을 합친 뒤의 모터 각도 상한/하한(도).
# flex 와 spread 를 각각 제한해도 합치면 한계를 넘을 수 있으므로
# (예: flex=30, spread=10 -> m1=40) 최종 각도를 여기서 한 번 더 막는다.
MOTOR_MIN_DEG = -80.0
MOTOR_MAX_DEG = 90.0

FLEX_LIMIT_RAD = math.radians(FLEX_LIMIT_DEG)
SPREAD_LIMIT_RAD = math.radians(SPREAD_LIMIT_DEG)
MOTOR_MIN_RAD = math.radians(MOTOR_MIN_DEG)
MOTOR_MAX_RAD = math.radians(MOTOR_MAX_DEG)

# --- 손가락별 가중치 ---
# (flex_weight, spread_weight)
#
# flex_weight: 같은 a 에 대해 이 손가락이 얼마나 접히나. 지금은 엄지를
#   포함해 전부 1.0 이다. 원래 엄지만 0.8 이었지만(마주보는 방향이라 같은
#   각도로 접으면 다른 손가락과 부딪혀서), 작은 물체에 엄지가 안 닿아
#   1.0 까지 올렸다 -- 자세한 경위는 아래 r_finger5 주석 참고.
#
# spread_weight: 부호가 핵심이다. 손을 벌리면 검지는 엄지 쪽, 새끼는 반대쪽으로
#   간다. 즉 중지(finger2)를 기준으로 좌우 부호가 뒤집혀야 한다.
#   전부 양수로 두면 손가락이 다 같은 쪽으로 쏠린다.
#   아래 부호는 2026-07-31 5손가락 동시 구동으로 실물 확인했다.
#
#   다듬을 여지 하나: 약지와 새끼가 둘 다 -1.0 이라 정확히 같은 각도로
#   움직인다. 실제 손은 부채처럼 점점 벌어지므로 약지를 -0.5 로 두면
#   더 자연스럽다. 동작에는 지장 없으니 급하지 않다.
FINGER_WEIGHTS = {
    "r_finger1": (1.0, +1.5),   # 검지 — 촉각 센서가 달려 있는 손가락
    "r_finger2": (1.0,  1.0),   # 중지 — 검지와 같은 쪽, 크기만 작다
    "r_finger3": (1.0, -1.0),   # 약지
    "r_finger4": (1.0, -1.5),   # 새끼
    # 엄지 flex_weight — 작은 물체를 잡으려면 이 값을 올린다.
    #
    # 0.8 은 sim_bridge/grasp_control.py:5 의 GRASP_POSE (다른 손가락
    # 1.5 / 엄지 1.2, 즉 1.2/1.5 = 0.8) 에서 온 값이고, 큰 물체를
    # 감싸는 파워 그립 기준이다. 엄지가 마주보는 방향이라 같은 각도로
    # 접으면 다른 손가락과 부딪히기 때문에 일부러 덜 접게 했다.
    #
    # 그런데 작은 물체는 반대다. 2026-08-13 실측에서 a=0.9 일 때
    #     검지·중지·약지·새끼  flex = 1.22 ~ 1.25 rad
    #     엄지                 flex = 0.98 rad
    # 이었고 엄지 힘은 두 번 다 0.00N — 물체에 아예 안 닿았다.
    #
    # 그래서 0.8 -> 1.0 으로 올렸고, 지금 값이 그것이다. **굽힘으로 낼 수
    # 있는 여유는 여기서 끝이다** -- 더 올리면 다른 손가락과 부딪힌다.
    #
    # 그런데 2026-08-14 로그에서도 엄지 힘은 여전히 0.00N 이다. 즉 이
    # 축으로는 해결이 안 된다. 엄지는 다른 손가락을 마주보는 방향이라
    # 아무리 접어도 옆으로는 안 오기 때문이다. 남은 축은 옆으로 트는
    # FINGER_SPREAD_BIAS_DEG 하나뿐이다(아직 0.0, 부호 미확인).
    "r_finger5": (1.0,  0.0),   # 엄지
}

# 이번 실행에서 실제로 움직일 손가락.
# 5개 전부 동작 확인 완료(2026-07-31).
# 배선/가중치/한계값을 바꾼 뒤에는 다시 ["r_finger1"] 하나로 줄여서 시작할 것.
# 10개 모터를 한꺼번에 돌리면 무엇이 잘못됐는지 알 수 없고 링키지끼리 부딪힌다.
ACTIVE_FINGERS = ["r_finger1","r_finger2","r_finger3","r_finger4","r_finger5"]

# --- 파지 순서 (손가락별 출발 지연, 초) ---
# 엄지(finger5)와 검지(finger1)는 최종 자세에서는 안 부딪히는데 '가는 도중'
# 스윕 경로가 겹친다. 그래서 목표를 바꾸는 게 아니라 출발 시각을 어긋나게 한다.
#
# 하드 배리어("1단계 도착 확인 후 2단계")가 아니라 지연 출발이다. 도착을
# read_present_position 으로 확인하는 방식은 물체를 잡는 순간 목표에 영영
# 도달하지 못해 영원히 안 끝난다 -- 파지가 목적인 손에서는 못 쓴다.
#
# 표에 없는 이름은 0.0(지연 없음). 전부 0.0 으로 두면 이 기능이 생기기
# 전과 완전히 같은 동작이 된다. 되돌릴 때 그렇게 하면 된다.
#
# 2026-08-04 실물 확인 완료. 아래 순서에서 엄지-검지 걸림이 없어졌다.
# 엄지가 마지막에 접혀 이미 닫힌 손가락 '위를 덮는' 형태다. 반대로
# 엄지를 먼저 접으면 엄지 최종 위치가 검지 경로 위에 놓여서 부딪힌다.
#
# 두 표는 서로 역순이어야 한다. 닫을 때 늦게 가는 손가락이 펼 때 먼저
# 나와야 경로가 안 겹친다 (tests/test_sequence.py 가 이걸 검사한다).
#
# 0.6 은 눈으로 맞춘 값이다. GOAL_SPEED=3 이 실제 몇 rad/s 인지는
# 어디에도 없어서 계산으로는 못 뽑는다.
CLOSE_DELAY_S = {
    "r_finger1": 0.0,   # 손가락 먼저 접히고
    "r_finger2": 0.0,
    "r_finger3": 0.0,
    "r_finger4": 0.0,
    "r_finger5": 0.6,   # 엄지가 그 위를 덮는다
}
OPEN_DELAY_S = {
    "r_finger5": 0.0,   # 덮고 있던 엄지를 먼저 치우고
    "r_finger1": 0.6,   # 손가락이 뒤따라 펴진다
    "r_finger2": 0.6,
    "r_finger3": 0.6,
    "r_finger4": 0.6,
}

# release() 가 마지막 그룹을 보낸 뒤 토크를 끄기 전에 기다리는 시간.
# 이동이 끝나기 전에 토크를 끊으면 손이 중간 자세에서 툭 떨어진다.
RELEASE_SETTLE_S = 0.5


def start_delay(finger_name, closing):
    """이 손가락이 몇 초 뒤에 출발해야 하나. 표에 없으면 0.0.

    closing=True 는 굽히는 방향(a 증가), False 는 펴는 방향(a 감소)이다.
    """
    table = CLOSE_DELAY_S if closing else OPEN_DELAY_S
    return float(table.get(finger_name, 0.0))


@dataclass(frozen=True)
class Finger:
    """r_hand.toml 한 항목. 모터 2개가 한 손가락을 이룬다."""
    name: str
    id1: int
    offset1: float
    id2: int
    offset2: float
    flex_weight: float
    spread_weight: float
    spread_bias: float = 0.0    # 라디안. FINGER_SPREAD_BIAS_DEG 참고


def load_fingers(toml_path=HAND_TOML, weights=None, active=None):
    """r_hand.toml 을 읽어 Finger 목록을 만든다.

    toml 구조는 이렇다 (r_hand.toml:5-14 참고):
        [[motors]]
        finger_name = "r_finger1"
        motor1.id = 1
        motor1.offset = 0.12217304763960307
        motor2.id = 2
        motor2.offset = 0.08726646259971647

    [[motors]] 는 [Fingers] 아래에 있는 것처럼 보이지만 toml 의 테이블
    헤더는 항상 절대경로라서 실제로는 최상위다. 즉 data["motors"] 로 접근한다.

    active 에 이름 목록을 주면 그 손가락만, active 순서대로 돌려준다.
    weights 에 없는 손가락은 (1.0, 0.0) 으로 둔다.
    """
    if weights is None:
        weights = FINGER_WEIGHTS

    with open(toml_path, "rb") as f:
        data = tomllib.load(f)

    by_name = {}
    for entry in data["motors"]:
        name = entry["finger_name"]
        flex_w, spread_w = weights.get(name, (1.0, 0.0))
        bias_deg = FINGER_SPREAD_BIAS_DEG.get(name, 0.0)
        by_name[name] = Finger(
            name=name,
            id1=int(entry["motor1"]["id"]),
            offset1=float(entry["motor1"]["offset"]),
            id2=int(entry["motor2"]["id"]),
            offset2=float(entry["motor2"]["offset"]),
            flex_weight=flex_w,
            spread_weight=spread_w,
            spread_bias=math.radians(bias_deg),
        )

    if active is None:
        return list(by_name.values())

    # active 에 있는데 toml 에 없는 이름을 조용히 무시하면, 오타 하나로
    # "손가락이 안 움직이는데 에러도 없는" 상태가 된다. 그게 제일 찾기 어렵다.
    missing = [n for n in active if n not in by_name]
    if missing:
        raise ValueError(
            f"{toml_path.name} 에 없는 손가락 이름: {missing}. "
            f"사용 가능한 이름: {sorted(by_name)}"
        )
    return [by_name[n] for n in active]


# ==============================================================
#  촉각 센서
# ==============================================================

# 센서 드라이버(Tashan capRead) 위치. 원본을 수정하지 않고 sys.path 로만
# 참조한다 -- 벤더 SDK 라 이 저장소에 포함하지 않았다. 센서와 같이 받은
# 것을 아무 데나 풀고 그 경로를 알려준다:
#
#   export CAPREAD_DIR=~/capRead_Python-win\&Linux-64bit
#
# 기본값은 이 저장소 옆의 vendor/capRead 다. 없으면 촉각 센서만 안 열리고
# 나머지는 정상이다 -- 그때는 --simple-grasp 로 돌린다.
#
# 리눅스에서는 라이브러리 파일만으로는 부족하고 CH341 커널 모듈을 직접
# 빌드해야 한다. docs/pi-setup.md 3장 참고.
CAPREAD_DIR = Path(os.environ.get(
    "CAPREAD_DIR",
    Path(__file__).resolve().parent.parent / "vendor" / "capRead"))

# PCA9548 채널 -> 손가락 이름.
# hand_tactile_view.py:28-34 의 규약을 그대로 가져왔다. 그 파일 주석이
# "실제 배선에 맞춰 고친다"고 명시하고 있으므로 이 값은 아직 미검증이다.
# grasp_main.py --sensor-only 로 손가락을 하나씩 눌러 반드시 확인할 것.
# 이게 틀리면 검지 힘을 보고 새끼를 조인다.
SENSOR_CHANNEL_MAP = {
    2: "r_finger1",
    3: "r_finger2",
    4: "r_finger3",
    5: "r_finger4",
    6: "r_finger5",
}

BASELINE_FRAMES = 30       # 무부하 평균에 쓸 프레임 수 (채널별)
BASELINE_TIMEOUT_S = 10.0
SENSOR_TIMEOUT_S = 10.0    # 첫 프레임 대기
DISCOVER_S = 3.0           # 어떤 채널이 붙었는지 모으는 시간
# 최신 프레임이 이보다 오래되면 그 채널을 "데이터 없음"으로 본다.
# CH341 이 뽑히면 드라이버가 마지막 값을 계속 물고 있으므로, 이게 없으면
# 손가락이 조인 채로 영원히 방치된다.
STALE_TIMEOUT_S = 1.0

# ==============================================================
#  파지 제어
# ==============================================================
#
# 아래 값 대부분은 눈으로 정한 시작값이다. Task 8(실물 브링업)에서
# 로그를 보고 조정한다. 조정 순서는 계획서 Task 8 참고.

F_TOUCH = 0.3          # 이만큼 힘이 잡히면 "닿았다" (N)
F_ABORT = 8.0          # 절대 상한. 넘으면 즉시 후퇴 (N)
F_ABORT_HYST = 1.0     # BACKOFF -> HOLD 복귀 여유 (N). 경계 채터링 방지
# --- 분류별 목표 파지력 ---
#
# 규약: 무를수록 약하게 잡는다.
#
#   a 를 더 줬는데 F 가 안 늘어난다  -> 찌그러지고 있다  -> soft -> 약하게
#   a 를 더 줬는데 F 가 늘어난다    -> 버티고 있다      -> rigid -> 세게 가능
#
# 판별 결과를 쓰는 목적이 "약한 물체를 안 부수는 것"이므로
# 반드시 F_TARGET_SOFT <= F_TARGET_RIGID 여야 한다. check_grasp_config()
# 가 이 순서를 강제한다 -- 한번 뒤집혀 있었던 자리다.
#
# 크기는 2026-08-13 실측 기준이다. 손가락이 a_max 까지 가도 지속 가능한
# 힘이 약 1.1N 였다(SCS0009 정지토크 0.15N.m 를 링키지로 환산한 값과
# 일치). 그보다 큰 목표를 주면 도달하지 못해 STALL_TIMEOUT_S 로 빠진다.
#
# 2026-08-14 현재 둘이 같은 값이다. 즉 분류 결과가 파지력을 안 바꾼다.
# 일부러 그렇게 뒀다 -- 가용 범위가 0.8~1.1N 뿐이라 두 목표를 벌려 봐야
# 차이가 F_DEADBAND 보다 작아 무의미하고, 지금은 슬립 원인을 걷어내는
# 게 먼저다. k_hat 은 계속 로그에 남으므로 K_THRESHOLD 자료 수집에는
# 지장이 없다. 액추에이터 여유가 생기면 다시 벌린다.
#
# 이전 값(SOFT 0.8 / RIGID 1.2)의 문제:
#   - SOFT 0.8 은 접촉만 해도 나오는 힘(0.81~1.24N)보다 낮아서, 제어기가
#     HOLD 내내 "너무 세다"고 판단해 손가락을 폈다 -> 미끄러짐
#   - RIGID 1.2 는 위 주석이 못 간다고 적어 둔 1.1N 을 넘는 값이었다.
#     실제로 접촉한 세 손가락 중 둘이 1.06/1.07N 에서 멈췄다
F_TARGET_SOFT = 1.8    # 연체로 판정 
F_TARGET_RIGID = 1.8   # 강체로 판정 -- 실측 도달 가능 상한 근처

F_DEADBAND = 0.3       # 오차가 이 안이면 아예 안 움직인다 (N)

# 접촉으로 인정하려면 F_TOUCH 를 연속으로 몇 사이클 넘어야 하나.
# 2026-08-13 종이컵 실측: 손가락이 테두리를 치는 순간 4.8N 이 잡혔다가
# 컵이 좌굴하면서 0 으로 꺼졌다. 한 사이클만 보고 프로빙에 들어가면
# 힘이 사라지는 동안 표본을 모아 k 가 쓰레기가 된다.
TOUCH_CONFIRM_CYCLES = 3

# 잡고 있던 물체가 빠졌다고 인정하려면 F_TOUCH 아래로 몇 사이클 연속
# 떨어져야 하나. 한 사이클만 보면 센서 노이즈 한 번에 파지를 놓고,
# 아예 안 보면 허공을 향해 목표 힘을 쫓으며 a_max 까지 올라간다.
CONTACT_LOST_CYCLES = 5

# F_ABORT 를 연속으로 몇 사이클 넘어야 후퇴(BACKOFF)하나.
#
# 2026-08-14 로그에서 앞뒤가 0.00N 인데 한 사이클만 18.42 / 27.20 /
# 49.05N 인 표본이 세 번 나왔다. 이 손이 낼 수 있는 힘은 약 1.1N 이라
# 그런 크기의 파지력은 존재할 수 없다. 원인이 센서 글리치든 물체가
# 미끄러지며 생긴 전단 과도응답이든, 후퇴는 틀린 대응이다 -- 특히
# 미끄러지는 중이라면 손을 여는 게 정확히 반대 행동이다.
#
# 대안이었던 tactile.MEDIAN_WINDOW 확대(3->5)는 쓰지 않았다. 그건 힘
# 신호 전체에 지연을 더하는데 실측 지연이 이미 0.55초라 더 늘릴 수
# 없다. 여기서만 연속 확인을 요구하면 정상 경로의 지연은 그대로다.
#
# 대가: 진짜 과부하에 대한 반응이 이 사이클 수만큼 늦는다. LOOP_HZ=10
# 기준 2사이클 = 0.2초이고, 그 사이 조이는 양은 DA_MAX*2 = 0.05 다.
ABORT_CONFIRM_CYCLES = 2

# 강체/연체 경계 (N/rad).
#
# None 인 것은 누락이 아니라 의도다. 이 값의 크기는 센서 감도, 링키지
# 지오메트리, 물체에 모두 의존해서 실측 없이는 근거 있는 숫자를 적을 수
# 없다. None 이면 stiffness.classify 가 분류를 건너뛰고 무조건 "soft" 를
# 돌려주므로 전부 F_TARGET_SOFT(둘 중 약한 쪽)로 동작하며, 그 사실이
# 로그의 class 열에 남는다 -- 조용히 아무 값이나 쓰는 것보다 "아직 안
# 정했다"가 드러나는 편이 낫다.
#
# 정하려면 스펀지/종이컵/나무토막의 k_hat 로그가 필요한데, 2026-08-14
# 시점에는 아직 못 정한다. 표본이 1.9~4.3 한 덩어리에 몰려 있고(종이컵
# 4.28 이 페트병 3.25 보다 크게 나왔다) 강체 쪽 표본이 없다. 게다가
# PROBE_SETTLE_S 를 고치기 전 로그라 k_hat 자체를 믿을 수 없다.
K_THRESHOLD = None

# k_hat 클램프. k_hat 은 제어식의 분모이므로 0/음수/무한대가 새어나가면
# Δa 가 발산한다. 여기서 반드시 막는다.
K_MIN = 0.1
K_MAX = 500.0

# 프로빙을 건너뛰고 이 강성을 그대로 쓴다. None 이면 예전처럼 측정한다.
#
# --- 왜 있는가 ---
# 프로빙이 실제로 이득인지 재기 위한 비교용 스위치다. 08-14/18 로그
# 74건에서 측정 성공률이 49% 였고, 실패한 51% 는 전부 a 가 A_MAX 에
# 붙어 flex 변화가 안 나온 경우였다(= 물체에 충분히 안 닿았다). 그
# 손가락들은 게인과 무관하게 더 조이지 못하므로, 남은 질문은 "성공한
# 49% 에서 실측 k_hat 이 상수 하나보다 나은가" 하나다.
#
# --- 파지력이 아니다 ---
# k_hat 은 제어식 Δa = λ·e/k_hat 의 분모, 즉 "얼마나 빨리 조일지"다.
# "얼마나 세게 쥘지"는 F_TARGET_* 이고 이 값과 무관하다.
#
# 3.9 는 위 로그에서 측정에 성공한 36건의 중앙값이다(범위 0.10~15.15).
# 켜면 파지가 PROBE_STEPS × PROBE_SETTLE_S = 3.75초 빨라진다.
K_FIXED = None

# 프로빙
PROBE_MIN_SPAN = 0.02   # 신뢰 가능한 최소 flex 변화폭 (rad).
                        # SCS0009 분해능 약 0.005 rad 의 4배.
PROBE_STEP = 0.02       # 계단 크기 (a 단위). flex 약 1.6도
PROBE_STEPS = 5         # 계단 횟수 = 샘플 개수
PROBE_SETTLE_S = 0.75   # 계단 후 안정화 대기.
                        # 서보 도착 시간만이 아니라 촉각 센서의 중앙값
                        # 창(tactile.MEDIAN_WINDOW=3)이 새 값으로 갈리는
                        # 시간까지 기다려야 한다. 이보다 짧게 잡으면
                        # 계단을 밟기 전 힘을 기록해 k 가 작게 나온다.
                        #
                        # 0.40 -> 0.75 (2026-08-14). 앞선 추정치 0.18초는
                        # 너무 낙관적이었다. 실측 로그에서 a 를 바꾼 뒤
                        # 힘이 반응하기까지 0.55초가 걸렸고(11-21-05.csv
                        # finger4 6.62s -> 7.17s), 그 결과 (flex, force)
                        # 짝이 한 계단씩 어긋난 채 회귀가 돌아갔다.
                        # 같은 페트병을 잡은 같은 시행에서 k_hat 이
                        # finger1 7.11 / finger4 1.29 로 5배 갈렸고, 직전
                        # 시행의 finger1 은 2.57 이었다 -- 사실상 난수다.

# 접근 / 유지
# 파지할 때 기본으로 주는 벌림(s). 0 이면 손가락이 부채 모양 그대로
# 닫힌다.
#
# 왜 필요한가 (2026-08-13 실측): 물체가 작으면 엄지가 아예 안 닿는다.
# 그때는 엄지 flex_weight 가 0.8 이라 덜 접히는 게 원인이라고 봤는데,
# 1.0 으로 올린 뒤(2026-08-14)에도 엄지 힘은 0.00N 그대로였다. 굽힘
# 축으로는 안 된다는 뜻이다.
#
# 부호: FINGER_WEIGHTS 의 spread_weight 가 검지 +1.5 / 중지 +1.0 /
# 약지 -1.0 / 새끼 -1.5 / 엄지 0 이고, 주석 규약상 s>0 이 검지를 엄지
# 쪽으로 보낸다. 즉 **양수가 검지·중지를 엄지 쪽으로 모은다.**
# 새끼는 반대로 벌어지지만 작은 물체에서는 어차피 안 쓴다.
#
# 기본값 0.0 = 끔. 이건 '다른 손가락을 엄지 쪽으로' 모으는 방식인데,
# 엄지 자신은 spread_weight 가 0.0 이라 이 값으로는 꿈쩍도 안 한다.
# 엄지 자체를 옆으로 틀려면 아래 FINGER_SPREAD_BIAS_DEG 를 쓴다.
# 필요하면 켜되(SPREAD_LIMIT_DEG=30도 기준 0.3 은 약 9도) 손가락
# 경로가 달라지므로 손 주변을 비우고 먼저 확인할 것.
GRASP_SPREAD = 0.0

# --- 손가락별 고정 벌림 바이어스 (도) ---
# a(굽힘) 와 s(벌림) 와 무관하게 이 손가락에 '항상' 더해지는 각도다.
# 편 자세(a=0)에도 적용되므로 말 그대로 기본 자세를 바꾼다.
#
# 왜 필요한가 (2026-08-13): 작은 물체를 엄지가 못 잡는다. 굽힘량
# (flex_weight)을 올려도 안 된다 -- 엄지는 다른 손가락을 마주보는
# 방향이라 아무리 접어도 옆으로는 안 오기 때문이다. 옆으로 트는 축은
# 벌림뿐인데, 엄지의 spread_weight 는 0.0 이라 s 로도 안 움직인다.
# 그래서 s 와 별개인 고정 바이어스를 둔다.
#
# 부호는 실물로 확인되지 않았다. 작은 값(5도)부터 넣고 어느 쪽으로
# 도는지 눈으로 보고 정할 것. 반대로 가면 부호를 뒤집는다.
#
# 안전: 이 값은 flex/spread 를 합친 뒤 MOTOR_MIN/MAX 로 잘리기 전에
# 더해진다. 즉 최종 각도는 여전히 한계 안이다. 다만 바이어스를 크게
# 주면 그만큼 굽힘 여유가 줄어든다.
#
# 처음 바꿀 때는 ACTIVE_FINGERS 를 ["r_finger1", "r_finger5"] 로 줄이고
# 손 주변을 비운 채 --dry-run 으로 숫자부터 확인할 것.
FINGER_SPREAD_BIAS_DEG = {
    "r_finger5": 0.0,   # 엄지 — 여기를 5, 10, 15 로 올리며 찾는다
}

A_RATE = 0.15          # 접근 속도 (a/s)
A_MAX = 0.9            # a 상한. 1.0 이 아닌 이유는 여유를 두기 위함
# hold 게인. 계산된 필요 변위의 몇 %를 갈지 (0~1]
#
# 0.5 -> 0.2 (2026-08-14). 촉각 센서 피드백이 0.5초 이상 늦어서, 한
# 사이클에 크게 움직이면 그 결과를 확인하기 전에 몇 번을 더 움직인다.
# 실측 로그에서 힘이 목표를 넘자 12사이클 연속 최대 속도로 열렸다.
LAMBDA = 0.2
KI = 0.02              # 적분 게인. 연체의 응력 완화를 잡는다. 작게 시작
DA_MAX = 0.025         # 조이는 방향 사이클당 최대 Δa. flex 약 2도

# 푸는 방향(a 감소) 사이클당 최대 |Δa|. 조이는 쪽보다 훨씬 작다.
#
# 두 방향의 위험이 대칭이 아니기 때문이다 -- 조여서 힘이 과해지면
# BACKOFF 가 잡지만, 풀어서 물체를 떨어뜨리면 되돌릴 방법이 없다.
# 게다가 조일 때는 물체가 막아 줘서 명령한 Δa 만큼 관절이 실제로
# 들어가지 않는 반면, 풀 때는 저항이 없어 그대로 다 나간다.
#
# 크기 근거: 힘 피드백 지연이 약 0.55초 = LOOP_HZ 10 기준 6사이클이다.
# 0.006 이면 그 6사이클 동안 최대 0.036 만 열린다. 직전 로그에서는
# (LOOP_HZ 20, 대칭 0.025) 같은 시간에 0.3 이 열려 물체가 미끄러졌다.
DA_MAX_OPEN = 0.006

BACKOFF_STEP = 0.02    # 후퇴 계단 (a 단위)

# HOLD 에서 진입 시점의 a 로부터 얼마까지 펼 수 있나.
#
# "힘을 낮추려다 물체를 놓는" 경로를 막는 바닥이다. 2026-08-14 로그에서
# finger4 가 14초에 걸쳐 a 를 0.900 -> 0.602 로 풀다가 물체를 놓쳤다.
# 방향은 제어식대로 맞았지만, 이 물체는 펴도 힘이 잘 안 줄어서 접촉이
# 끊길 때까지 여는 것 말고 종료 조건이 없었다.
#
# 아래 STALL_TIMEOUT_S 의 목표 자동 하향은 a_max 에 붙어 있을 때만
# 발동한다. 즉 한번 열리기 시작한 손가락은 그쪽이 못 잡는다 -- 그
# 경로를 이 바닥이 막는다.
#
# 크기 근거: 실효 강성이 약 2.4 N/rad 였으므로 a 0.15 = flex 0.21 rad 는
# 약 0.5N 을 낮출 수 있다. F_DEADBAND(0.3N)보다 넉넉해서 정상적인 힘
# 조절은 그대로 되면서, 놓칠 만큼 크게 여는 것만 막힌다.
#
# BACKOFF 는 이 제한을 안 받는다 -- 진짜 과부하 경로는 끝까지 물러설 수
# 있어야 한다 (grasp.py 의 BACKOFF 분기 참고).
HOLD_OPEN_LIMIT = 0.15

# HOLD 가 a_max 에 붙은 채 목표 힘에 못 미치는 상태로 이 시간(초) 이상
# 머물면 "이 접촉에서는 도달 불가"로 판정한다. 그 뒤 동작이 두 가지다:
#
#   힘이 F_TOUCH 초과  -> 잡고는 있다. 목표를 그 실측치로 낮추고 계속 간다
#   힘이 F_TOUCH 이하  -> 진짜 허공이다. NO_CONTACT
#
# 앞의 경우가 손가락별 편차를 흡수한다. 2026-08-14 로그의 HOLD 중앙값이
# finger2 0.40N / finger4 1.72N 으로 4배 갈렸다 -- 같은 물체를 같이
# 잡는데도 접촉 지오메트리 때문에 이만큼 다르다. 목표를 넉넉히 걸어두면
# 각 손가락이 이 타임아웃 뒤에 자기가 낼 수 있는 값으로 정착한다.
#
# 그래서 이 값은 "얼마나 기다렸다가 목표를 포기하나"다. 너무 짧으면
# 아직 조이는 중인데 목표를 낮추고, 너무 길면 그동안 손가락이 도달
# 불가능한 목표를 쫓는다.
STALL_TIMEOUT_S = 2.0

TEMP_LIMIT_C = 55.0    # 이 온도를 넘으면 손 전체 release
TEMP_CHECK_INTERVAL_S = 1.0   # 모터 온도를 몇 초마다 확인할지.
                              # 과열 abort 는 이 간격만큼 자주만 발동 가능하고,
                              # 온도 읽기는 시리얼 왕복이라 실제 트레이드오프
# 온도 읽기가 이 횟수만큼 연속으로 실패하면 더 이상 과열을 감시할 수
# 없다는 뜻이므로 경고만 찍고 계속하지 않고 파지를 중단한다. 과열
# 가드가 마지막 안전장치인데, 그게 소리 없이 사라지면 안 된다.
TEMP_FAIL_LIMIT = 3

# 제어 루프 주기(Hz).
#
# 20 -> 10 (2026-08-14). 루프를 센서보다 빠르게 돌려 봐야 같은 힘 값을
# 두 번 읽을 뿐이고, 그 사이에 명령만 계속 나가서 오버슈트가 된다.
# 촉각 드라이버는 66Hz 로 도는데 제어 루프와 시리얼을 경합하면 실효
# 11Hz 까지 떨어지고, 거기에 tactile.MEDIAN_WINDOW(3프레임)가 더해져
# 실측 지연이 0.55초였다. 10Hz 로 낮추면 위치/온도 읽기 왕복이 절반이
# 되어 센서 쪽 지연 자체도 줄어든다.
#
# 주의: TOUCH_CONFIRM_CYCLES / CONTACT_LOST_CYCLES 는 '사이클' 단위라
# 이 값을 바꾸면 실제 시간이 같이 바뀐다 (3사이클 = 0.15초 -> 0.3초).
LOOP_HZ = 10


# ==============================================================
#  손목 (STS3215)
# ==============================================================
#
# 손과 같은 COM11 을 쓴다. URT-1 의 서보 커넥터는 병렬로 묶인 한 가닥이라
# 어디에 꽂아도 같은 버스다. 두 컨트롤러를 동시에 열 수 없으므로
# servo_bus.ServoLease 로 시간분할한다.

# SCS0009 손이 ID 1~10 을 전부 쓴다. 손목은 반드시 11 이상이어야 한다.
# **미확인** -- servo_id_tool.py 로 스캔해서 실제 값을 여기 적을 것.
WRIST_ID = 11

# 손목 중립 자세와 가동범위(라디안).
# **미확인** -- 좁게 시작해서 넓힌다. 손목 위에 카메라가 실려 있어서
# 크게 잡았다가 한계를 넘으면 케이블이 먼저 끊어진다.
WRIST_CENTER_RAD = 0.0
WRIST_MIN_RAD = math.radians(-180.0)
WRIST_MAX_RAD = math.radians(180.0)

# 각도 오차를 지우는 방향. **미확인** -- w/W 조그로 눈으로 확인한다.
# 반대면 손목이 오차를 키우며 한계까지 갔다가 UNREACHABLE 로 끝난다
# (발산하지는 않는다. wrist_align 의 클램프가 막는다).
WRIST_DIRECTION_SIGN = +1

# 서보 내부 속도. 손과 같은 값으로 시작한다. 미검증.
WRIST_GOAL_SPEED = GOAL_SPEED

# 한 사이클 최대 이동. 크게 잡으면 손목 위 카메라가 흔들려 다음 프레임의
# 각도 측정이 깨진다 -- 제어 루프가 자기 센서를 흔드는 셈이다.
WRIST_DA_MAX_RAD = math.radians(3.0)

# GRASPING -> HOLDING 판정. 활성 손가락 전부가 HOLD/NO_CONTACT 인 상태가
# 이만큼 연속되면 '자세가 잡혔다'로 본다. LOOP_HZ=10 기준 0.5초.
#
# grasp.FingerGrasp 에는 종착 상태가 없다 -- HOLD 는 목표 힘을 계속
# 추종하는 정상 상태고 NO_CONTACT 도 되살아난다. 그래서 '끝'을 밖에서
# 정해야 한다.
GRASP_STABLE_CYCLES = 5


def check_grasp_config():
    """파지 설정값의 불변식을 검사한다. 어기면 ValueError.

    import 시점(파일 맨 끝)에서 한 번 부른다. 값 하나 잘못 적어서
    손가락이 실물에서 영원히 떠는 것보다 여기서 죽는 게 낫다.
    """
    safe = F_ABORT - F_ABORT_HYST
    if F_ABORT_HYST <= 0.0:
        raise ValueError(
            f"F_ABORT_HYST({F_ABORT_HYST})는 0보다 커야 한다. "
            f"0이면 힘 상한 경계에서 HOLD와 BACKOFF가 매 사이클 반복된다."
        )
    for name, value in (("F_TARGET_RIGID", F_TARGET_RIGID),
                        ("F_TARGET_SOFT", F_TARGET_SOFT)):
        if value >= safe:
            raise ValueError(
                f"{name}({value})는 F_ABORT - F_ABORT_HYST({safe})보다 "
                f"작아야 한다. 그러지 않으면 HOLD가 목표를 향해 조이다 "
                f"상한에 걸려 BACKOFF로 가고, 후퇴하면 다시 조이는 "
                f"무한 왕복이 된다."
            )
    if F_TARGET_SOFT > F_TARGET_RIGID:
        raise ValueError(
            f"F_TARGET_SOFT({F_TARGET_SOFT})가 F_TARGET_RIGID"
            f"({F_TARGET_RIGID})보다 크다. 무른 물체를 더 세게 잡는다는 "
            f"뜻이라 분류의 목적(약한 물체를 안 부수는 것)이 뒤집힌다."
        )
    if F_TOUCH >= min(F_TARGET_RIGID, F_TARGET_SOFT):
        raise ValueError(
            f"F_TOUCH({F_TOUCH})는 목표힘보다 작아야 한다. "
            f"접촉을 인식한 순간 이미 목표를 넘어 있으면 프로빙이 무의미하다."
        )
    if K_MIN <= 0.0:
        raise ValueError(
            f"K_MIN({K_MIN})은 0보다 커야 한다. k_hat은 제어식의 "
            f"분모이므로 0이 새어나가면 Δa가 발산한다."
        )
    if K_MAX <= K_MIN:
        raise ValueError(f"K_MAX({K_MAX})는 K_MIN({K_MIN})보다 커야 한다.")
    if not 0.0 < LAMBDA <= 1.0:
        raise ValueError(
            f"LAMBDA({LAMBDA})는 (0, 1] 범위여야 한다. 1을 넘으면 매 "
            f"사이클 목표를 지나쳐 가라는 뜻이라 발진한다."
        )
    if KI < 0.0:
        raise ValueError(f"KI({KI})는 음수일 수 없다.")
    if not 0.0 < A_MAX <= 1.0:
        raise ValueError(
            f"A_MAX({A_MAX})는 (0, 1] 범위여야 한다. kinematics의 a 규약이다."
        )
    if DA_MAX <= 0.0:
        raise ValueError(f"DA_MAX({DA_MAX})는 0보다 커야 한다.")
    if DA_MAX_OPEN <= 0.0:
        raise ValueError(
            f"DA_MAX_OPEN({DA_MAX_OPEN})은 0보다 커야 한다. 0 이면 힘이 "
            f"목표를 넘어도 영원히 못 풀어서 물체를 계속 조인다."
        )
    if DA_MAX_OPEN > DA_MAX:
        raise ValueError(
            f"DA_MAX_OPEN({DA_MAX_OPEN})이 DA_MAX({DA_MAX})보다 크다. "
            f"푸는 쪽을 더 빠르게 두면 비대칭 제한의 목적(놓치는 방향을 "
            f"더 조심하는 것)이 뒤집힌다."
        )
    if BACKOFF_STEP <= 0.0:
        raise ValueError(f"BACKOFF_STEP({BACKOFF_STEP})은 0보다 커야 한다.")
    if HOLD_OPEN_LIMIT <= 0.0:
        raise ValueError(
            f"HOLD_OPEN_LIMIT({HOLD_OPEN_LIMIT})은 0보다 커야 한다. "
            f"0 이면 HOLD 가 힘을 낮출 방법이 아예 없어진다."
        )
    if HOLD_OPEN_LIMIT >= A_MAX:
        raise ValueError(
            f"HOLD_OPEN_LIMIT({HOLD_OPEN_LIMIT})이 A_MAX({A_MAX}) 이상이다. "
            f"그러면 바닥이 0 이하가 되어 제한이 없는 것과 같다."
        )
    unknown_bias = sorted(set(FINGER_SPREAD_BIAS_DEG) - set(FINGER_WEIGHTS))
    if unknown_bias:
        raise ValueError(
            f"FINGER_SPREAD_BIAS_DEG 에 모르는 손가락 이름: {unknown_bias}. "
            f"사용 가능한 이름: {sorted(FINGER_WEIGHTS)}"
        )
    for _name, _deg in FINGER_SPREAD_BIAS_DEG.items():
        if abs(_deg) > SPREAD_LIMIT_DEG:
            raise ValueError(
                f"{_name} 의 벌림 바이어스({_deg}도)가 "
                f"SPREAD_LIMIT_DEG({SPREAD_LIMIT_DEG}도)를 넘는다. "
                f"이만큼 틀면 굽힘 여유가 그만큼 사라진다."
            )
    if not -1.0 <= GRASP_SPREAD <= 1.0:
        raise ValueError(
            f"GRASP_SPREAD({GRASP_SPREAD})는 -1 ~ 1 범위여야 한다. "
            f"kinematics 의 s 규약이다."
        )
    if A_RATE <= 0.0:
        raise ValueError(f"A_RATE({A_RATE})는 0보다 커야 한다.")
    if STALL_TIMEOUT_S <= 0.0:
        raise ValueError(
            f"STALL_TIMEOUT_S({STALL_TIMEOUT_S})는 0보다 커야 한다. "
            f"0 이하면 HOLD가 a_max에 닿자마자 곧바로 NO_CONTACT로 떨어져 "
            f"목표 힘에 도달할 시간조차 없다."
        )
    if TEMP_CHECK_INTERVAL_S <= 0.0:
        raise ValueError(
            f"TEMP_CHECK_INTERVAL_S({TEMP_CHECK_INTERVAL_S})는 0보다 커야 한다. "
            f"0 이하면 매 사이클 온도를 읽어 시리얼 대역폭을 다 쓴다."
        )
    if TEMP_FAIL_LIMIT < 1:
        raise ValueError(
            f"TEMP_FAIL_LIMIT({TEMP_FAIL_LIMIT})은 1 이상이어야 한다. "
            f"0 이하면 온도 읽기가 한 번도 실패하지 않아도 즉시 감시 "
            f"불능으로 판정돼 매번 파지를 중단하게 된다."
        )
    if TOUCH_CONFIRM_CYCLES < 1:
        raise ValueError(
            f"TOUCH_CONFIRM_CYCLES({TOUCH_CONFIRM_CYCLES})는 1 이상이어야 "
            f"한다. 0 이면 접촉을 한 번도 확인하지 않고 프로빙에 들어간다."
        )
    if CONTACT_LOST_CYCLES < 1:
        raise ValueError(
            f"CONTACT_LOST_CYCLES({CONTACT_LOST_CYCLES})는 1 이상이어야 "
            f"한다. 0 이면 매 사이클 파지를 놓는다."
        )
    if ABORT_CONFIRM_CYCLES < 1:
        raise ValueError(
            f"ABORT_CONFIRM_CYCLES({ABORT_CONFIRM_CYCLES})는 1 이상이어야 "
            f"한다. 0 이하면 힘이 상한을 넘지 않아도 후퇴한다."
        )
    if ABORT_CONFIRM_CYCLES > TOUCH_CONFIRM_CYCLES:
        raise ValueError(
            f"ABORT_CONFIRM_CYCLES({ABORT_CONFIRM_CYCLES})가 "
            f"TOUCH_CONFIRM_CYCLES({TOUCH_CONFIRM_CYCLES})보다 크다. "
            f"접촉을 인정하는 것보다 위험을 인정하는 데 더 오래 걸린다는 "
            f"뜻이라, 과부하가 접촉보다 늦게 감지된다."
        )
    if PROBE_STEPS < 2:
        raise ValueError(
            f"PROBE_STEPS({PROBE_STEPS})는 2 이상이어야 한다. 회귀로 "
            f"기울기를 뽑으려면 점이 최소 2개 필요하다."
        )
    if PROBE_STEP <= 0.0:
        raise ValueError(f"PROBE_STEP({PROBE_STEP})은 0보다 커야 한다.")
    if PROBE_MIN_SPAN <= 0.0:
        raise ValueError(f"PROBE_MIN_SPAN({PROBE_MIN_SPAN})은 0보다 커야 한다.")
    if K_THRESHOLD is not None and not K_MIN < K_THRESHOLD < K_MAX:
        raise ValueError(
            f"K_THRESHOLD({K_THRESHOLD})는 K_MIN({K_MIN})과 "
            f"K_MAX({K_MAX}) 사이여야 한다. 클램프 밖에 두면 모든 물체가 "
            f"한쪽으로만 분류된다."
        )

    bad_weights = sorted(name for name, (flex_w, _) in FINGER_WEIGHTS.items()
                         if flex_w <= 0.0)
    if bad_weights:
        raise ValueError(
            f"FINGER_WEIGHTS의 flex_weight는 0보다 커야 한다: {bad_weights}. "
            f"0이면 flex_span이 0이 되어 FingerForceController가 CLASSIFY "
            f"단계에서 ValueError를 낸다 -- 이미 물체를 잡은 채로."
        )

    if WRIST_ID in range(1, 11):
        raise ValueError(
            f"WRIST_ID({WRIST_ID})가 SCS0009 손이 쓰는 범위(1~10)와 겹친다. "
            f"STS 버스는 반이중 단선이라 ID 가 겹치면 둘이 동시에 응답해서 "
            f"**둘 다** 죽는다. servo_id_tool.py --set-id 로 11 이상으로 "
            f"바꿀 것."
        )
    if WRIST_MIN_RAD >= WRIST_MAX_RAD:
        raise ValueError(
            f"WRIST_MIN_RAD({WRIST_MIN_RAD})가 WRIST_MAX_RAD"
            f"({WRIST_MAX_RAD}) 이상이다. 가동범위가 없으면 모든 정렬이 "
            f"UNREACHABLE 이 된다."
        )
    if not WRIST_MIN_RAD <= WRIST_CENTER_RAD <= WRIST_MAX_RAD:
        raise ValueError(
            f"WRIST_CENTER_RAD({WRIST_CENTER_RAD})가 가동범위 밖이다. "
            f"중립으로 되돌리는 명령이 곧바로 한계로 잘린다."
        )
    if WRIST_DIRECTION_SIGN not in (1, -1):
        raise ValueError(
            f"WRIST_DIRECTION_SIGN({WRIST_DIRECTION_SIGN})은 +1 또는 -1 "
            f"이어야 한다. 0 이면 손목이 영영 안 움직인다."
        )
    if WRIST_DA_MAX_RAD <= 0.0:
        raise ValueError(
            f"WRIST_DA_MAX_RAD({WRIST_DA_MAX_RAD})는 0보다 커야 한다. "
            f"0 이면 손목이 영영 안 움직인다."
        )
    if GRASP_STABLE_CYCLES < 1:
        raise ValueError(
            f"GRASP_STABLE_CYCLES({GRASP_STABLE_CYCLES})는 1 이상이어야 "
            f"한다. 0 이하면 파지를 시작한 첫 사이클에 곧바로 '잡았다'가 "
            f"된다."
        )

    if K_FIXED is not None and not K_MIN <= K_FIXED <= K_MAX:
        raise ValueError(
            f"K_FIXED({K_FIXED})가 [K_MIN({K_MIN}), K_MAX({K_MAX})] 밖이다. "
            f"k_hat 은 제어식의 분모라 범위를 벗어나면 Δa 가 발산하거나 "
            f"손가락이 사실상 정지한다. 끄려면 None 으로 둘 것."
        )

    names = list(SENSOR_CHANNEL_MAP.values())
    if len(names) != len(set(names)):
        raise ValueError(
            f"SENSOR_CHANNEL_MAP에 같은 손가락이 두 번 나온다: {names}"
        )
    unknown = sorted(set(names) - set(FINGER_WEIGHTS))
    if unknown:
        raise ValueError(
            f"SENSOR_CHANNEL_MAP에 모르는 손가락 이름: {unknown}. "
            f"사용 가능한 이름: {sorted(FINGER_WEIGHTS)}"
        )


check_grasp_config()
