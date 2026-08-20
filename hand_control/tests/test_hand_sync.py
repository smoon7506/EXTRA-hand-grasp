# -*- coding: utf-8 -*-
"""Hand 의 sync 경로. 가짜 컨트롤러라 모터가 필요 없다."""

import pytest

import hand_config
import kinematics
from hand import Hand


class FakeSyncController:
    """rustypot Scs0009PyController 의 sync API 대역.

    실측 시그니처(2026-08-13):
        sync_write_goal_position(ids, values) -> None
        read_present_position(id) -> list[float]

    읽기에 sync 대역이 없는 것은 의도다. SCS0009 가 SYNC READ 에
    응답하지 않아서 hand.py 가 개별 읽기만 쓴다.
    """

    def __init__(self, positions=None):
        self.writes = []              # [(ids, values), ...] 호출 단위로
        self.positions = positions or {}   # {motor_id: rad}
        self.read_calls = 0
        self.temp_calls = 0
        self.fail_read = False

    def sync_write_goal_position(self, ids, values):
        assert len(ids) == len(values), "ids 와 values 길이가 다르다"
        self.writes.append((list(ids), list(values)))

    def read_present_position(self, motor_id):
        # rustypot 의 read_* 는 단일 ID 를 넣어도 리스트를 돌려준다.
        self.read_calls += 1
        if self.fail_read:
            raise RuntimeError("시리얼 끊김")
        return [self.positions.get(motor_id, 0.0)]

    def read_present_temperature(self, motor_id):
        self.temp_calls += 1
        return [30.0]


@pytest.fixture
def fingers():
    return hand_config.load_fingers()


@pytest.fixture
def hand(fingers):
    h = Hand(fingers)
    h._c = FakeSyncController()
    return h


class TestSetPoseMap:
    def test_맵에_있는_손가락만_보낸다(self, hand, fingers):
        f1 = fingers[0]
        hand.set_pose_map({f1.name: 0.5})
        ids, _ = hand._c.writes[0]
        assert sorted(ids) == sorted([f1.id1, f1.id2])

    def test_버스_왕복은_한_번이다(self, hand, fingers):
        # 20Hz 루프에서 손가락마다 따로 쓰면 시리얼이 막힌다.
        hand.set_pose_map({f.name: 0.4 for f in fingers})
        assert len(hand._c.writes) == 1
        ids, values = hand._c.writes[0]
        assert len(ids) == 10 and len(values) == 10

    def test_손가락별로_다른_a를_보낼_수_있다(self, hand, fingers):
        # 이게 이 메서드의 존재 이유다. set_pose 는 a 하나만 받는다.
        a_map = {f.name: 0.1 * (i + 1) for i, f in enumerate(fingers)}
        pose = hand.set_pose_map(a_map)
        for f in fingers:
            expected = hand.set_pose_map({f.name: a_map[f.name]})
            assert pose[f.id1] == pytest.approx(expected[f.id1])
            assert pose[f.id2] == pytest.approx(expected[f.id2])

    def test_기존_set_pose와_같은_각도가_나온다(self, hand, fingers):
        # 두 경로가 갈리면 손가락마다 자세가 어긋난다.
        hand._c = FakeSyncController()
        via_map = hand.set_pose_map({f.name: 0.6 for f in fingers}, s=0.3)
        expected = {}
        for f in fingers:
            m1, m2 = kinematics.finger_command(
                0.6, 0.3, f,
                hand_config.FLEX_LIMIT_RAD, hand_config.SPREAD_LIMIT_RAD,
                hand_config.MOTOR_MIN_RAD, hand_config.MOTOR_MAX_RAD)
            expected[f.id1], expected[f.id2] = m1, m2
        for mid, angle in via_map.items():
            assert angle == pytest.approx(expected[mid])

    def test_빈_맵이면_아무것도_안_보낸다(self, hand):
        pose = hand.set_pose_map({})
        assert hand._c.writes == []
        assert pose == {}

    def test_모르는_손가락_이름은_예외(self, hand):
        # 오타 하나로 "손가락이 안 움직이는데 에러도 없는" 상태가 되면
        # 제일 찾기 어렵다. load_fingers 와 같은 방침이다.
        with pytest.raises(ValueError, match="r_finger9"):
            hand.set_pose_map({"r_finger9": 0.5})

    def test_모터_ID가_중복되면_예외(self):
        # kinematics.hand_pose 가 하는 검사(kinematics.py:96-100)를
        # set_pose_map 도 똑같이 해야 한다. set_pose_map 은 hand_pose 를
        # 거치지 않고 finger_command 를 직접 부르므로 검사를 물려받지
        # 못한다 -- r_hand.toml 오타로 두 손가락이 같은 모터 ID 를 쓰면,
        # 검사가 없으면 dict(zip(...)) 에서 한쪽 각도가 조용히 덮어써져
        # "손가락 하나가 안 움직이는데 에러도 없는" 상태가 된다.
        # r_hand.toml 을 건드리지 않고, 여기서 직접 중복 ID 를 만든다.
        dup_a = hand_config.Finger(
            name="fake_a", id1=1, offset1=0.0, id2=2, offset2=0.0,
            flex_weight=1.0, spread_weight=0.0)
        dup_b = hand_config.Finger(
            name="fake_b", id1=1, offset1=0.0, id2=3, offset2=0.0,
            flex_weight=1.0, spread_weight=0.0)
        h = Hand([dup_a, dup_b])
        h._c = FakeSyncController()
        with pytest.raises(ValueError, match="중복된다"):
            h.set_pose_map({"fake_a": 0.5, "fake_b": 0.5})


class TestReadFlex:
    def test_역변환이_kinematics_규약과_맞는다(self, hand, fingers):
        # kinematics.py:15 — flex = (m1 - m2) / 2  (오프셋 제거 후)
        f = fingers[0]
        want = 0.35
        hand._c.positions = {
            f.id1: want + f.offset1,
            f.id2: -want + f.offset2,
        }
        flex = hand.read_flex()
        assert flex[f.name] == pytest.approx(want)

    def test_벌림이_섞여도_flex만_뽑는다(self, hand, fingers):
        # m1 = flex + spread + off1,  m2 = -flex + spread + off2
        # 차분을 쓰므로 spread 는 상쇄돼야 한다.
        f = fingers[0]
        want, spread = 0.35, 0.2
        hand._c.positions = {
            f.id1: want + spread + f.offset1,
            f.id2: -want + spread + f.offset2,
        }
        assert hand.read_flex()[f.name] == pytest.approx(want)

    def test_모터마다_한_번씩_읽는다(self, hand):
        # 읽기는 개별이다. 쓰기(set_pose_map)만 왕복 한 번이다.
        hand.read_flex()
        assert hand._c.read_calls == len(hand.motor_ids())

    def test_모든_활성_손가락이_키로_나온다(self, hand, fingers):
        flex = hand.read_flex()
        assert set(flex) == {f.name for f in fingers}

    def test_읽기가_실패하면_전부_None(self, hand, fingers):
        # 예외를 밖으로 던지면 20Hz 루프가 통째로 죽는다.
        # None 을 주면 grasp 상태기계가 그 손가락을 동결한다.
        hand._c.fail_read = True
        flex = hand.read_flex()
        assert set(flex) == {f.name for f in fingers}
        assert all(v is None for v in flex.values())

    def test_실제_각도는_명령값과_다를_수_있다(self, hand, fingers):
        # 강체에 막히면 명령은 나가도 관절은 안 움직인다.
        # 이 차이를 읽어내는 게 read_flex 의 존재 이유다.
        f = fingers[0]
        hand.set_pose_map({f.name: 0.9})
        blocked = 0.1
        hand._c.positions = {
            f.id1: blocked + f.offset1,
            f.id2: -blocked + f.offset2,
        }
        assert hand.read_flex()[f.name] == pytest.approx(blocked)
        assert hand.read_flex()[f.name] < 0.9 * hand_config.FLEX_LIMIT_RAD

    def test_한_손가락_값만_깨져도_그_손가락만_None(self, hand, fingers):
        # 읽기 호출 자체는 성공해도 흔들리는 시리얼 버스에서는 값
        # 하나만 깨져서 돌아올 수 있다 (호출 실패와는 다른 고장
        # 모드). float() 변환이
        # 실패해도 예외가 밖으로 새면 안 되고, 깨진 손가락만 None 이
        # 되고 나머지 손가락은 정상 값을 내야 한다.
        broken = fingers[0]
        healthy = fingers[1]
        hand._c.positions = {
            broken.id1: "corrupted",   # float() 이 못 바꾸는 값
            broken.id2: 0.0,
            healthy.id1: 0.2 + healthy.offset1,
            healthy.id2: -0.2 + healthy.offset2,
        }
        flex = hand.read_flex()
        assert flex[broken.name] is None
        assert flex[healthy.name] == pytest.approx(0.2)


class Test개별_읽기_실패_격리:
    """모터 하나가 응답을 놓쳐도 그 손가락만 None 이어야 한다.

    읽기가 개별이라 한 모터의 실패가 나머지에 안 번진다. 손 전체가
    FROZEN 이 되면 물체를 문 채로 제어가 멈춘다.
    """

    def test_실패한_모터의_손가락만_None(self, fingers):
        h = Hand(fingers)
        h._c = FakeSyncController()
        dead = fingers[0]
        original = h._c.read_present_position

        def flaky(motor_id):
            if motor_id == dead.id1:
                raise RuntimeError("응답 없음")
            return original(motor_id)

        h._c.read_present_position = flaky
        flex = h.read_flex()
        assert flex[dead.name] is None
        assert all(flex[f.name] is not None for f in fingers if f is not dead)


class Test온도_읽기:
    """온도 감시는 개별 읽기다. sync_read 는 SCS0009 에서 안 된다.

    감시가 실패했다고 제어 루프가 죽으면 안 된다 -- 2026-08-13 실물에서
    온도 sync_read 실패가 TEMP_FAIL_LIMIT(3회, 3초)에 걸려 손가락이
    물체에 닿기도 전에 손이 풀렸다.
    """

    def _hand(self, fingers):
        h = Hand(fingers)
        h._c = FakeSyncController()
        return h

    def test_모터마다_한_번씩_읽는다(self, fingers):
        h = self._hand(fingers)
        temps = h.read_temperatures()
        assert len(temps) == len(h.motor_ids())
        assert h._c.temp_calls == len(h.motor_ids())
        assert all(t == pytest.approx(30.0) for t in temps)

    def test_전부_실패하면_빈_리스트(self, fingers):
        # 호출부는 이 빈 리스트를 "감시 불가"로 세고, 연속 3회면 중단한다.
        h = self._hand(fingers)

        def boom(motor_id):
            raise RuntimeError("응답 없음")

        h._c.read_present_temperature = boom
        assert h.read_temperatures() == []

    def test_일부만_실패하면_나머지는_감시된다(self, fingers):
        h = self._hand(fingers)
        dead = h.motor_ids()[0]
        original = h._c.read_present_temperature

        def flaky(motor_id):
            if motor_id == dead:
                raise RuntimeError("응답 없음")
            return original(motor_id)

        h._c.read_present_temperature = flaky
        assert len(h.read_temperatures()) == len(h.motor_ids()) - 1

    def test_예외를_밖으로_던지지_않는다(self, fingers):
        # 감시가 실패했다고 제어 루프가 죽으면 손이 물체를 문 채 방치된다.
        h = self._hand(fingers)
        h._c.read_present_temperature = lambda mid: (_ for _ in ()).throw(
            RuntimeError("boom"))
        h.read_temperatures()   # 예외가 나오면 테스트 실패
