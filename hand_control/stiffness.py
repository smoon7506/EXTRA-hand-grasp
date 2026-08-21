# -*- coding: utf-8 -*-
"""(flex, force) 샘플 -> 강성 k. 하드웨어를 모르는 순수 계산.

--- 왜 시간이 아니라 변위인가 ---
"파지를 계속하면 힘이 계속 오른다 = 강체"는 시간 기준(dF/dt)으로 보면
틀린다. 강체를 잡아도 서보가 목표에 도달해 멈추면 힘 증가도 멈추고,
연체는 손가락이 계속 파고들며 힘이 천천히라도 계속 오른다. 두 경우가
뒤집혀 보인다.

물리적으로 맞는 판별량은 변위당 힘 증가율 k = ΔF/Δx 다.
    벽    : 조금 밀었는데 힘이 확 오름   -> k 큼   -> 강체
    스펀지: 많이 밀었는데 힘이 조금 오름 -> k 작음 -> 연체

x 는 '명령한 각도'가 아니라 '실제로 굽은 각도'여야 한다. 강체를 잡으면
명령은 계속 나가는데 실제 관절은 멈춰 있어서, 명령값을 쓰면 강체가
연체로 분류된다. hand.read_flex() 가 실측값을 준다.

단위: k 는 N/rad (flex 라디안당 뉴턴).
"""


def estimate_stiffness(samples, k_min, k_max, min_span):
    """[(flex_rad, force_N), ...] -> (k_hat, confident).

    최소제곱 1차 회귀의 기울기. 두 점 차분을 쓰지 않는 이유는 센서
    노이즈와 서보 분해능(약 0.005 rad) 때문에 두 점만으로는 기울기가
    크게 흔들리기 때문이다.

    confident=False 인 경우 k_hat 은 k_max 다. 가장 보수적인 값이라
    Δa = λ·e/k_hat 이 작아져 조심스럽게 움직이게 된다. 다만 호출부는
    이 플래그를 반드시 로그에 남겨야 한다 -- 조용히 큰 수를 쓰면 나중에
    왜 그렇게 잡았는지 추적할 수 없다.
    """
    if len(samples) < 2:
        return (k_max, False)

    xs = [float(x) for x, _ in samples]
    ys = [float(y) for _, y in samples]

    # 변위가 거의 없으면 기울기가 발산한다. 아주 단단한 물체에서 실제로
    # 일어난다 -- 밀어도 관절이 서보 분해능만큼도 안 움직인다.
    if max(xs) - min(xs) < min_span:
        return (k_max, False)

    n = len(xs)
    mean_x = sum(xs) / n
    mean_y = sum(ys) / n
    sxx = sum((x - mean_x) ** 2 for x in xs)
    if sxx == 0.0:
        # min_span 검사를 통과했는데 여기 걸리는 건 부동소수점 극단값뿐이다.
        # 그래도 0으로 나누기는 막는다.
        return (k_max, False)
    sxy = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys))
    slope = sxy / sxx

    # 밀었는데 힘이 줄었다 = 미끄러졌거나 센서가 드리프트했다.
    # 음수를 k_min 으로 클램프하면 아주 작은 양수가 되어 Δa 가 폭발한다.
    # 가장 보수적인 k_max 로 보낸다.
    if slope <= 0.0:
        return (k_max, False)

    return (max(k_min, min(slope, k_max)), True)


def classify(k_hat, k_threshold, confident=True):
    """k_hat -> "rigid" | "soft".

    confident=False 면 k_hat 을 보지 않고 "soft" 다. estimate_stiffness 가
    추정에 실패하면 k_hat 자리에 k_max 를 넣어 주는데, 그건 "가장 보수적인
    분모"라는 뜻이지 "아주 단단하다"는 뜻이 아니다. 그대로 임계값과
    비교하면 어떤 임계값이든 넘어서 실패가 곧 rigid 가 된다.

    K_THRESHOLD 가 None 이던 동안에는 아래 분기가 무조건 soft 를 돌려줘서
    이 경로가 가려져 있었다. 임계값을 정하는 순간 드러난다.

    실패를 강체 증거로 못 쓰는 이유는 hand_k 주석에 있다 -- 실측 25건의
    측정 실패가 100% a=A_MAX 였다.

    k_threshold 가 None 이면(아직 실측 전) "soft" 를 돌려준다.
    규약상 무를수록 약하게 잡으므로(F_TARGET_SOFT <= F_TARGET_RIGID),
    모르는 상태에서는 약한 쪽이 안전하다 -- 무른 물체를 세게 잡으면
    부수지만, 단단한 물체를 약하게 잡으면 놓칠 뿐이다.

    경계값(k_hat == k_threshold)은 "rigid" 다. 임계값을 실측으로 정한
    뒤라면 경계는 이미 판단 가능한 영역이고, 여기서 soft 로 떨어지면
    임계값 바로 위/아래가 불연속으로 갈린다.
    """
    if k_threshold is None or not confident:
        return "soft"
    return "rigid" if k_hat >= k_threshold else "soft"


def hand_k(samples):
    """[(k_hat, confident), ...] -> 손 하나의 대표 강성. 모르면 None.

    --- 왜 평균이 아니라 최대인가 ---
    물체는 하나인데 손가락마다 접촉 상태가 다르다. 스치듯 닿은 손가락은
    많이 들어가는데 힘은 조금 나오므로 낮은 k 를 준다 -- 물체가 아니라
    자기 접촉을 잰 값이다. 반면 강체의 증거인 "밀었는데 안 들어간다"는
    제대로 닿은 손가락 하나만으로 성립한다. 평균은 그 증거를 못 닿은
    손가락들로 희석시킨다.

    2026-08-18/19 로그: 같은 물체를 같이 잡은 손가락 사이의 k_hat 편차가
    2.3배였고, 한 런은 147배였다(f1=14.72 f3=0.10).

    --- confident=False 를 반드시 빼는 이유 ---
    estimate_stiffness 는 flex 변화폭이 모자라면 (k_max, False) 를
    돌려준다. 그 값을 최대에 넣으면 손가락 하나만 실패해도 손 전체가
    항상 k_max, 즉 항상 rigid 가 된다.

    게다가 그 실패는 강체의 증거가 아니다. 실측 25건의 측정 실패가
    100% a=A_MAX 였다. PROBE 는 a 를 계단으로 올리며 재는데 a 가 이미
    천장이면 5샘플이 전부 같은 자리에서 찍힌다 -- '밀었는데 안 들어간
    것'이 아니라 '밀 여유가 없던 것'이다. 두 상황은 로그에서 구분되지
    않으므로 강체 판정에 쓸 수 없다.
    """
    usable = [float(k) for k, ok in samples if ok and k is not None]
    return max(usable) if usable else None
