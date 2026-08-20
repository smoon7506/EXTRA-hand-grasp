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


def classify(k_hat, k_threshold):
    """k_hat -> "rigid" | "soft".

    k_threshold 가 None 이면(아직 실측 전) "soft" 를 돌려준다.
    규약상 무를수록 약하게 잡으므로(F_TARGET_SOFT <= F_TARGET_RIGID),
    모르는 상태에서는 약한 쪽이 안전하다 -- 무른 물체를 세게 잡으면
    부수지만, 단단한 물체를 약하게 잡으면 놓칠 뿐이다.

    경계값(k_hat == k_threshold)은 "rigid" 다. 임계값을 실측으로 정한
    뒤라면 경계는 이미 판단 가능한 영역이고, 여기서 soft 로 떨어지면
    임계값 바로 위/아래가 불연속으로 갈린다.
    """
    if k_threshold is None:
        return "soft"
    return "rigid" if k_hat >= k_threshold else "soft"
