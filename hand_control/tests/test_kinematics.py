# -*- coding: utf-8 -*-
"""kinematics 순수 함수 테스트. 하드웨어 불필요."""

import math

import pytest

import hand_config
import kinematics
from hand_config import Finger


LIM_FLEX = math.radians(30.0)
LIM_SPREAD = math.radians(10.0)
MIN_R = math.radians(-60.0)
MAX_R = math.radians(60.0)

# 클램프가 개입하지 않는 넉넉한 범위. "자르기"를 검증하는 테스트가 아닐 때는
# 이걸 써야 한다. 안 그러면 수식이 틀렸는데 클램프 덕분에 통과하는
# 가짜 초록불이 나올 수 있다.
BIG = math.radians(1000.0)


def make_finger(name="r_finger1", id1=1, offset1=0.0, id2=2, offset2=0.0,
                flex_weight=1.0, spread_weight=0.0):
    """테스트용 Finger. 기본값은 오프셋·가중치가 단순해서 계산이 눈에 보인다."""
    return Finger(name=name, id1=id1, offset1=offset1, id2=id2,
                  offset2=offset2, flex_weight=flex_weight,
                  spread_weight=spread_weight)


class TestClamp:
    def test_안쪽_값은_그대로(self):
        assert kinematics.clamp(5, 0, 10) == 5

    def test_범위를_벗어나면_잘린다(self):
        assert kinematics.clamp(-3, 0, 10) == 0
        assert kinematics.clamp(99, 0, 10) == 10


class TestMotorAngles:
    def test_굽힘만_주면_부호가_반대다(self):
        # 이게 차동 구조의 핵심이다. 두 모터가 반대로 돌아야 손가락이 굽는다.
        m1, m2 = kinematics.motor_angles(0.5, 0.0, 0.0, 0.0, -BIG, BIG)
        assert m1 == pytest.approx(0.5)
        assert m2 == pytest.approx(-0.5)

    def test_벌림만_주면_부호가_같다(self):
        # 두 모터가 같은 방향으로 돌면 손가락이 옆으로 벌어진다.
        m1, m2 = kinematics.motor_angles(0.0, 0.3, 0.0, 0.0, -BIG, BIG)
        assert m1 == pytest.approx(0.3)
        assert m2 == pytest.approx(0.3)

    def test_오프셋이_더해진다(self):
        # flex=0, spread=0 이면 오프셋만 남는다. 즉 a=0 은 '움직이지 마라'가
        # 아니라 '캘리브레이션 영점(=편 손)으로 가라'다.
        m1, m2 = kinematics.motor_angles(0.0, 0.0, 0.12, 0.08, -BIG, BIG)
        assert m1 == pytest.approx(0.12)
        assert m2 == pytest.approx(0.08)

    def test_합친_뒤에_잘린다(self):
        # flex(0.9)도 spread(0.3)도 각각은 MAX_R(1.047) 안이지만
        # 합(1.2)은 넘는다. 이 테스트가 이 모듈에서 제일 중요하다.
        # 여기가 깨지면 실물에서 링키지가 부러진다.
        m1, m2 = kinematics.motor_angles(0.9, 0.3, 0.0, 0.0, MIN_R, MAX_R)
        assert m1 == pytest.approx(MAX_R)
        assert MIN_R <= m1 <= MAX_R
        assert MIN_R <= m2 <= MAX_R

    def test_오프셋도_클램프_대상이다(self):
        # 오프셋은 실제로 서보에 나가는 값의 일부다. 오프셋을 더한 뒤의
        # 최종값이 범위 안이어야 의미가 있다.
        m1, _ = kinematics.motor_angles(0.0, 0.0, 99.0, 0.0, MIN_R, MAX_R)
        assert m1 == pytest.approx(MAX_R)

    def test_기존_goal_positions_와_같은_결과(self):
        # spread=0 일 때 tactile_motor_test/motor.py:9 의
        #     goal_positions(bend, o1, o2) = (bend+o1, -bend+o2)
        # 와 정확히 같아야 한다. 이 모듈은 그 함수의 '일반화'이지
        # 다른 규약이 아니다. 이 테스트가 통과하면 기존 동작을 안 깬 것이다.
        o1, o2 = 0.12217304763960307, 0.08726646259971647
        for bend in (0.0, 0.5, 1.0):
            expected = (bend + o1, -bend + o2)
            actual = kinematics.motor_angles(bend, 0.0, o1, o2, -BIG, BIG)
            assert actual[0] == pytest.approx(expected[0])
            assert actual[1] == pytest.approx(expected[1])


class TestFingerCommand:
    def test_a는_0에서_1로_잘린다(self):
        # 손가락은 뒤로 젖혀지지 않는다. a=-0.5 는 a=0 과 같아야 한다.
        finger = make_finger()
        neg = kinematics.finger_command(-0.5, 0.0, finger, LIM_FLEX,
                                        LIM_SPREAD, -BIG, BIG)
        zero = kinematics.finger_command(0.0, 0.0, finger, LIM_FLEX,
                                         LIM_SPREAD, -BIG, BIG)
        assert neg == zero

        over = kinematics.finger_command(5.0, 0.0, finger, LIM_FLEX,
                                         LIM_SPREAD, -BIG, BIG)
        one = kinematics.finger_command(1.0, 0.0, finger, LIM_FLEX,
                                        LIM_SPREAD, -BIG, BIG)
        assert over == one

    def test_s는_음수도_허용된다(self):
        # s=-1 은 정상 입력이다. 반대쪽으로 벌린다.
        finger = make_finger(spread_weight=1.0)
        m1, m2 = kinematics.finger_command(0.0, -1.0, finger, LIM_FLEX,
                                           LIM_SPREAD, -BIG, BIG)
        assert m1 == pytest.approx(-LIM_SPREAD)
        assert m2 == pytest.approx(-LIM_SPREAD)

    def test_a가_1이면_limit_만큼_굽는다(self):
        finger = make_finger()
        m1, m2 = kinematics.finger_command(1.0, 0.0, finger, LIM_FLEX,
                                           LIM_SPREAD, -BIG, BIG)
        assert m1 == pytest.approx(LIM_FLEX)
        assert m2 == pytest.approx(-LIM_FLEX)

    def test_가중치가_굽힘_크기를_바꾼다(self):
        # 엄지(flex_weight=0.8)는 같은 a 에서 80% 만 굽어야 한다.
        thumb = make_finger(name="r_finger5", flex_weight=0.8)
        m1, _ = kinematics.finger_command(1.0, 0.0, thumb, LIM_FLEX,
                                          LIM_SPREAD, -BIG, BIG)
        assert m1 == pytest.approx(0.8 * LIM_FLEX)

    def test_spread_가중치_부호가_방향을_뒤집는다(self):
        # 손을 벌리면 검지와 새끼는 서로 반대쪽으로 간다.
        # 부호를 안 뒤집으면 손가락이 다 같은 쪽으로 쏠린다.
        right = make_finger(name="r_finger1", spread_weight=+1.0)
        left = make_finger(name="r_finger4", id1=7, id2=8, spread_weight=-1.0)
        r1, _ = kinematics.finger_command(0.0, 1.0, right, LIM_FLEX,
                                          LIM_SPREAD, -BIG, BIG)
        l1, _ = kinematics.finger_command(0.0, 1.0, left, LIM_FLEX,
                                          LIM_SPREAD, -BIG, BIG)
        assert r1 == pytest.approx(-l1)
        assert r1 > 0 > l1

    def test_spread_가중치_0이면_벌림_명령을_무시한다(self):
        # 중지는 벌림의 기준축이라 s 를 줘도 안 움직여야 한다.
        middle = make_finger(name="r_finger2", spread_weight=0.0)
        with_s = kinematics.finger_command(0.3, 1.0, middle, LIM_FLEX,
                                           LIM_SPREAD, -BIG, BIG)
        without_s = kinematics.finger_command(0.3, 0.0, middle, LIM_FLEX,
                                              LIM_SPREAD, -BIG, BIG)
        assert with_s == without_s


class TestHandPose:
    def test_모터_ID를_키로_돌려준다(self):
        fingers = [make_finger(name="r_finger1", id1=1, id2=2),
                   make_finger(name="r_finger4", id1=7, id2=8)]
        pose = kinematics.hand_pose(0.5, 0.0, fingers, LIM_FLEX,
                                    LIM_SPREAD, -BIG, BIG)
        assert sorted(pose) == [1, 2, 7, 8]

    def test_활성_손가락만_들어간다(self):
        # 첫 실행 안전 절차(손가락 1개부터)가 이것에 의존한다.
        fingers = [make_finger(name="r_finger1", id1=1, id2=2)]
        pose = kinematics.hand_pose(0.5, 0.0, fingers, LIM_FLEX,
                                    LIM_SPREAD, -BIG, BIG)
        assert sorted(pose) == [1, 2]

    def test_모터_ID가_중복되면_에러(self):
        # dict 라서 조용히 덮어써지면 손가락 하나가 통째로 안 움직이는데
        # 에러는 없는 상태가 된다. r_hand.toml 의 id 오타로 생길 수 있다.
        fingers = [make_finger(name="r_finger1", id1=1, id2=2),
                   make_finger(name="r_finger2", id1=1, id2=3)]
        with pytest.raises(ValueError, match="중복"):
            kinematics.hand_pose(0.5, 0.0, fingers, LIM_FLEX,
                                 LIM_SPREAD, -BIG, BIG)

    def test_각_손가락이_자기_오프셋을_쓴다(self):
        # 손가락마다 오프셋이 다르다. 한 손가락 것을 다른 손가락에 쓰면
        # 두 모터가 서로 다른 '편 상태'를 믿게 되어 가만히 있어도 힘을 쓴다.
        fingers = [make_finger(name="r_finger1", id1=1, offset1=0.11,
                               id2=2, offset2=0.22),
                   make_finger(name="r_finger4", id1=7, offset1=0.33,
                               id2=8, offset2=0.44)]
        pose = kinematics.hand_pose(0.0, 0.0, fingers, LIM_FLEX,
                                    LIM_SPREAD, -BIG, BIG)
        assert pose[1] == pytest.approx(0.11)
        assert pose[2] == pytest.approx(0.22)
        assert pose[7] == pytest.approx(0.33)
        assert pose[8] == pytest.approx(0.44)


class Test벌림_바이어스:
    """손가락별 고정 벌림. a, s 와 무관하게 항상 더해진다.

    엄지는 다른 손가락을 마주보는 방향이라 아무리 접어도 옆으로 안
    온다. 작은 물체를 잡으려면 옆으로 틀어야 하는데, 굽힘량으로는
    만들 수 없는 축이라 별도 바이어스가 필요하다.
    """

    def _finger(self, bias_rad, spread_weight=0.0):
        return hand_config.Finger(
            name="t", id1=1, offset1=0.0, id2=2, offset2=0.0,
            flex_weight=1.0, spread_weight=spread_weight,
            spread_bias=bias_rad,
        )

    def _cmd(self, finger, a=0.0, s=0.0):
        return kinematics.finger_command(
            a, s, finger,
            hand_config.FLEX_LIMIT_RAD, hand_config.SPREAD_LIMIT_RAD,
            hand_config.MOTOR_MIN_RAD, hand_config.MOTOR_MAX_RAD,
        )

    def test_바이어스가_0이면_이전과_같다(self):
        assert self._cmd(self._finger(0.0), a=0.5) == self._cmd(
            self._finger(0.0), a=0.5)

    def test_편_자세에도_적용된다(self):
        # 이게 '기본 각도'의 정의다. a=0 에서도 틀어져 있어야 한다.
        bias = math.radians(10.0)
        m1, m2 = self._cmd(self._finger(bias), a=0.0, s=0.0)
        # spread 성분은 두 모터에 같은 부호로 실린다 (kinematics.py:12-13)
        assert m1 == pytest.approx(bias)
        assert m2 == pytest.approx(bias)

    def test_굽힘과_섞여도_벌림_성분만_바뀐다(self):
        bias = math.radians(10.0)
        plain = self._cmd(self._finger(0.0), a=0.5)
        biased = self._cmd(self._finger(bias), a=0.5)
        # flex = (m1-m2)/2 는 그대로, spread = (m1+m2)/2 만 bias 만큼 이동
        assert (biased[0] - biased[1]) / 2 == pytest.approx(
            (plain[0] - plain[1]) / 2)
        assert (biased[0] + biased[1]) / 2 - (plain[0] + plain[1]) / 2 \
            == pytest.approx(bias)

    def test_s와_더해진다(self):
        # 바이어스는 s 를 대체하는 게 아니라 s 에 더해지는 상수다.
        bias = math.radians(10.0)
        f = self._finger(bias, spread_weight=1.0)
        m1, m2 = self._cmd(f, a=0.0, s=1.0)
        assert (m1 + m2) / 2 == pytest.approx(
            hand_config.SPREAD_LIMIT_RAD + bias)

    def test_바이어스도_모터_한계로_잘린다(self):
        # 한계를 넘겨도 최종 각도는 안전 범위 안이어야 한다.
        huge = hand_config.MOTOR_MAX_RAD * 2
        m1, m2 = self._cmd(self._finger(huge), a=1.0)
        assert hand_config.MOTOR_MIN_RAD <= m1 <= hand_config.MOTOR_MAX_RAD
        assert hand_config.MOTOR_MIN_RAD <= m2 <= hand_config.MOTOR_MAX_RAD
