# -*- coding: utf-8 -*-
"""목표 힘 추종. 하드웨어를 모르는 순수 로직.

--- 왜 고정 게인 PI 가 아닌가 ---
강체와 연체는 강성이 100배까지 차이 난다. 고정 Kp 하나로는 둘을 동시에
맞출 수 없다. 연체에 맞춘 Kp 는 강체에서 발진하고, 강체에 맞춘 Kp 는
연체에서 한없이 느리다.

여기서는 프로빙에서 이미 측정한 k_hat 을 게인의 분모로 쓴다.

    e / k_hat = 목표 힘까지 flex 를 몇 rad 더 굽혀야 하는가

물리적으로 필요한 변위를 직접 계산하는 것이라, 강체는 k_hat 이 커서
Δa 가 저절로 작아지고 연체는 k_hat 이 작아서 저절로 커진다. 물체가
바뀌어도 lam 을 다시 튜닝할 필요가 없다.
"""


def clamp(value, low, high):
    return max(low, min(value, high))


class FingerForceController:
    """손가락 하나의 힘 추종 제어기. 손가락마다 하나씩 만든다.

    k_hat 은 이 손가락이 지금 잡고 있는 물체의 강성이다. 물체가 바뀌면
    (= 다시 프로빙하면) 새 제어기를 만든다. 중간에 k_hat 을 갈아끼우면
    적분 상태가 옛 강성 기준이라 맞지 않는다.
    """

    def __init__(self, f_target, k_hat, flex_span,
                 lam, ki, deadband, da_max, a_max, da_max_open=None):
        """
        f_target    : 유지할 힘 (N)
        k_hat       : 이 물체의 강성 (N/rad). 0 이나 음수면 안 된다
        flex_span   : a 1.0 당 flex 라디안 (= flex_weight * FLEX_LIMIT_RAD)
        lam         : 계산된 필요 변위의 몇 %를 한 사이클에 갈지 (0, 1]
        ki          : 적분 게인
        deadband    : 이 안의 오차는 무시 (N)
        da_max      : 조이는 방향(a 증가) 한 사이클 최대 Δa
        a_max       : a 상한
        da_max_open : 푸는 방향(a 감소) 한 사이클 최대 |Δa|.
                      None 이면 da_max 와 같다(대칭).

        --- 왜 푸는 쪽을 따로 제한하는가 ---
        두 방향의 위험이 대칭이 아니다.

        조일 때는 물체가 막아 주므로 명령 Δa 만큼 관절이 실제로 들어가지
        않는다. 게다가 힘이 과해지면 BACKOFF 가 잡는다.

        푸는 방향은 저항이 없어 명령 Δa 가 그대로 실제 변위가 되고,
        놓쳐서 물체를 떨어뜨리면 되돌릴 방법이 없다. 2026-08-14 로그에서
        힘이 목표를 넘자 제어기가 14사이클 연속 최대 속도로 열어 a 를
        0.35 (관절 22도) 풀었고 물체가 미끄러졌다. 촉각 센서 피드백이
        0.5초 이상 늦어서, 여는 동안 힘이 줄었는지 확인할 방법이 없는
        채로 계속 열었기 때문이다.
        """
        if k_hat <= 0.0:
            raise ValueError(
                f"k_hat({k_hat})은 0보다 커야 한다. 제어식의 분모라서 "
                f"0이나 음수가 들어오면 Δa가 발산하거나 방향이 뒤집힌다. "
                f"stiffness.estimate_stiffness 가 [k_min, k_max]로 "
                f"클램프한 값을 넘겨야 한다."
            )
        if flex_span <= 0.0:
            raise ValueError(f"flex_span({flex_span})은 0보다 커야 한다.")
        if da_max_open is not None and da_max_open <= 0.0:
            raise ValueError(
                f"da_max_open({da_max_open})은 0보다 커야 한다. 0 이면 힘이 "
                f"목표를 넘어도 영원히 못 풀어서 물체를 계속 조인다."
            )
        self.f_target = f_target
        self.k_hat = k_hat
        self.flex_span = flex_span
        self.lam = lam
        self.ki = ki
        self.deadband = deadband
        self.da_max = da_max
        self.da_max_open = da_max if da_max_open is None else da_max_open
        self.a_max = a_max
        self.integral = 0.0     # 누적 보정량 (rad)
        self._prev_error = 0.0  # 부호 반전 감지용. update() 주석 참고

    def update(self, a, force):
        """현재 a 와 측정 힘 -> 새 a."""
        error = self.f_target - force

        # 데드밴드. 이게 없으면 목표 근처에서 미세 진동이 계속되고
        # 서보가 쉬지 못해 열난다. control.py 의 히스테리시스와 같은 목적.
        # 적분도 여기서는 쌓지 않는다 -- 쌓으면 데드밴드가 무의미해진다.
        if abs(error) < self.deadband:
            return a

        # 오차 부호가 뒤집히면 적분을 버린다.
        #
        # 기존 안티 와인드업은 각도 한계에 걸렸을 때만 롤백하는데, 그것만
        # 으로는 한 방향으로 쌓인 적분이 반대 방향 응답을 통째로 막는다.
        # a_max 에 붙어 목표에 못 미치는 동안 적분이 양수로 커진 뒤 물체가
        # 세게 눌러 오면, 비례항(lam*error/k_hat)이 그 적분을 못 이겨서
        # 손가락이 아예 안 풀린다. 반대 경우도 같다 -- 2026-08-14
        # 11-49-05 로그의 28초 구간에서 힘이 목표보다 낮은데도 a 가 계속
        # 줄어든 게 이 현상이다.
        #
        # 부호가 바뀌었다는 건 상황이 반대로 뒤집혔다는 뜻이라, 예전 오차의
        # 누적은 더 이상 쓸모가 없다.
        if error * self._prev_error < 0.0:
            self.integral = 0.0
        self._prev_error = error

        previous_integral = self.integral
        self.integral += self.ki * error / self.k_hat

        delta_rad = self.lam * error / self.k_hat + self.integral
        # 조이는 쪽과 푸는 쪽의 한계가 다르다 (__init__ 주석 참고).
        delta_a = clamp(delta_rad / self.flex_span,
                        -self.da_max_open, self.da_max)

        target = a + delta_a
        new_a = clamp(target, 0.0, self.a_max)

        # 안티 와인드업. 각도 한계에 걸려 실제로 못 움직였는데 적분만
        # 계속 쌓이면, 나중에 물체가 치워졌을 때 쌓인 값 때문에 손가락이
        # 한참 동안 엉뚱하게 움직인다.
        if new_a != target:
            self.integral = previous_integral

        return new_a
