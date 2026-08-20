# -*- coding: utf-8 -*-
"""a(굽힘) / s(벌림) -> 모터 각도. 하드웨어를 모르는 순수 계산.

AmazingHand 는 손가락 하나에 모터가 2개인 차동(differential) 구조다.
두 모터를 어떻게 섞느냐로 두 가지 동작이 나온다.

    부호가 반대 (m1 = +x, m2 = -x)  ->  굽힘/폄   (flexion)
    부호가 같음 (m1 = +y, m2 = +y)  ->  벌림/모음 (spread)

그래서 두 성분을 그냥 더하면 된다:

    m1 =  flex + spread + offset1
    m2 = -flex + spread + offset2

역변환도 성립한다: flex = (m1-m2)/2, spread = (m1+m2)/2.
(오프셋을 뺀 뒤 기준)

기존 tactile_motor_test/motor.py:9 의 goal_positions(bend, o1, o2) 는
위 식에서 spread=0 인 특수한 경우다. 즉 이 모듈은 그 함수를 버리는 게
아니라 일반화한 것이다.

--- 범위 규약 (중요) ---
    a (flexion) : [0, 1]   0 = 편 손, 1 = 최대 굽힘
    s (spread)  : [-1, 1]  좌/우 양방향

a 가 [-1, 1] 이 아닌 이유: 손가락은 굽는 방향으로만 움직이고 뒤로는
젖혀지지 않는다. 음수 a 를 주면 링키지가 기계적 스토퍼에 부딪힌 채로
서보가 계속 힘을 써서 발열/스톨한다.
"""


def clamp(value, low, high):
    """value 를 [low, high] 안으로 자른다."""
    return max(low, min(value, high))


def motor_angles(flex_rad, spread_rad, offset1, offset2,
                 motor_min_rad, motor_max_rad):
    """굽힘/벌림(라디안) -> 모터 2개의 목표 각도(라디안) 튜플.

    자르는 것은 반드시 '합친 뒤'다. flex 와 spread 를 각각 제한해도
    합계는 한계를 넘을 수 있기 때문이다
    (예: flex=30도, spread=10도 -> m1 = 40도).

    오프셋도 자르기 전에 더한다. 오프셋은 캘리브레이션 영점이라
    실제로 서보에 나가는 값의 일부이고, 그것까지 포함한 최종값이
    안전 범위 안이어야 의미가 있다.
    """
    m1 = flex_rad + spread_rad + offset1
    m2 = -flex_rad + spread_rad + offset2
    return (
        clamp(m1, motor_min_rad, motor_max_rad),
        clamp(m2, motor_min_rad, motor_max_rad),
    )


def finger_command(a, s, finger, flex_limit_rad, spread_limit_rad,
                   motor_min_rad, motor_max_rad):
    """정규화 입력 (a, s) + Finger -> 모터 2개의 목표 각도.

    a, s 는 손가락과 무관한 '손 전체의 의도'이고,
    손가락별 차이는 finger.flex_weight / finger.spread_weight 가 만든다.

    finger.spread_bias 는 a, s 와 무관하게 항상 더해지는 고정 벌림
    각도다. 굽힘량으로는 만들 수 없는 '기본 자세'를 잡는 축이다 --
    엄지는 마주보는 방향이라 아무리 접어도 옆으로 안 오는데, 작은
    물체는 엄지를 살짝 틀어야 닿는다. a=0(편 손) 에서도 적용된다.
    """
    # 호출부를 믿지 않는다. 범위 밖 값이 조용히 통과하면 모터가 이상한
    # 데로 가는데, 최종 각도는 어차피 잘리므로 증상만 보고는 원인을
    # 찾기 어렵다. 들어오는 자리에서 막는다.
    a = clamp(a, 0.0, 1.0)
    s = clamp(s, -1.0, 1.0)

    flex_rad = a * finger.flex_weight * flex_limit_rad
    spread_rad = (s * finger.spread_weight * spread_limit_rad
                  + finger.spread_bias)
    return motor_angles(
        flex_rad, spread_rad, finger.offset1, finger.offset2,
        motor_min_rad, motor_max_rad,
    )


def hand_pose(a, s, fingers, flex_limit_rad, spread_limit_rad,
              motor_min_rad, motor_max_rad):
    """손 전체 자세 -> {모터 ID: 목표 각도(rad)} 딕셔너리.

    길이 10 벡터가 아니라 딕셔너리로 돌려주는 이유: 이번 실행에서
    일부 손가락만 활성화(ACTIVE_FINGERS)할 수 있어야 하고, 그때
    "몇 번째 칸이 몇 번 모터였지"를 세지 않아도 되게 하기 위해서다.
    """
    pose = {}
    for finger in fingers:
        m1, m2 = finger_command(
            a, s, finger, flex_limit_rad, spread_limit_rad,
            motor_min_rad, motor_max_rad,
        )
        for motor_id, angle in ((finger.id1, m1), (finger.id2, m2)):
            # 같은 ID 가 두 번 나오면 dict 라서 조용히 덮어써진다. 그러면
            # 손가락 하나가 통째로 안 움직이는데 에러는 없는 상태가 된다.
            # r_hand.toml 의 오타로 충분히 생길 수 있으므로 여기서 막는다.
            if motor_id in pose:
                raise ValueError(
                    f"모터 ID {motor_id} 가 중복된다 (손가락 {finger.name}). "
                    f"r_hand.toml 의 id 를 확인할 것."
                )
            pose[motor_id] = angle
    return pose
