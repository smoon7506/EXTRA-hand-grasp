# -*- coding: utf-8 -*-
"""Hand 의 부분 자세 전송과 release 순서. 가짜 컨트롤러라 모터가 필요 없다."""

import pytest

import hand_config
from hand import Hand


class FakeController:
    """rustypot Scs0009PyController 대역. 어떤 순서로 뭘 썼는지 기록만 한다."""

    def __init__(self):
        self.goals = []          # [(motor_id, angle), ...] 쓴 순서대로
        self.torque = []         # [(motor_id, value), ...]

    def write_goal_position(self, mid, angle):
        self.goals.append((mid, angle))

    def write_goal_speed(self, mid, speed):
        pass

    def write_torque_enable(self, mid, value):
        self.torque.append((mid, value))


@pytest.fixture
def fingers():
    return hand_config.load_fingers()


@pytest.fixture
def hand(fingers):
    """connect() 를 부르지 않고 _c 만 가짜로 꽂는다. 시리얼이 없어도 된다."""
    h = Hand(fingers)
    h._c = FakeController()
    return h


class TestPartialSetPose:
    def test_fingers를_안_주면_전체를_보낸다(self, hand):
        hand.set_pose(0.5, 0.0)
        assert len(hand._c.goals) == 10

    def test_fingers를_주면_그_손가락만_보낸다(self, hand, fingers):
        thumb = [f for f in fingers if f.name == "r_finger5"]
        pose = hand.set_pose(0.5, 0.0, thumb)
        sent_ids = [mid for mid, _ in hand._c.goals]
        assert sorted(sent_ids) == sorted([thumb[0].id1, thumb[0].id2])
        assert set(pose) == set(sent_ids)

    def test_부분_전송의_각도는_전체_전송과_같다(self, hand, fingers):
        # 부분으로 보낸다고 값이 달라지면 손가락마다 자세가 어긋난다.
        thumb = [f for f in fingers if f.name == "r_finger5"]
        full = hand.set_pose(0.7, 0.3)
        part = hand.set_pose(0.7, 0.3, thumb)
        for mid, angle in part.items():
            assert angle == pytest.approx(full[mid])


class TestReleaseOrder:
    def _timeline(self, hand, monkeypatch):
        """모터 쓰기와 sleep 을 한 줄에 섞어 기록한다.

        모터 ID 순서만 보면 안 된다. r_finger5 는 ACTIVE_FINGERS 의
        마지막이라, 전부 한 번에 보내도 엄지 모터가 dict 순서상 뒤에
        나온다 -- 순서가 지켜져서가 아니라 우연이다. 그래서 두 묶음
        '사이에 대기가 끼어 있는지'를 본다. 그게 순차 폄의 정의다.
        """
        events = []
        monkeypatch.setattr("hand.time.sleep",
                            lambda s: events.append(("sleep", s)))
        original = hand._c.write_goal_position

        def record(mid, angle):
            events.append(("goal", mid))
            original(mid, angle)

        hand._c.write_goal_position = record
        return events

    def test_폄_지연_표대로_묶여서_나간다(self, hand, fingers, monkeypatch):
        """어느 손가락이 먼저인지가 아니라 '표대로 나뉘는가'를 본다.

        순서 방향은 hand_config 에서 튜닝하는 값이다. 여기에 방향을
        박아두면 표를 고칠 때마다 이 테스트가 같이 깨진다.
        """
        events = self._timeline(hand, monkeypatch)
        early, late = set(), set()
        for f in fingers:
            group = late if hand_config.start_delay(
                f.name, closing=False) > 0.0 else early
            group.update((f.id1, f.id2))

        hand.release()

        goals = [(i, mid) for i, (kind, mid) in enumerate(events)
                 if kind == "goal"]
        assert {mid for _, mid in goals} == early | late, "빠진 모터가 있다"

        if not late:
            # 표를 비운 되돌리기 상태. 한 번에 다 나가는 게 정상이다.
            return

        last_early = max(i for i, mid in goals if mid in early)
        first_late = min(i for i, mid in goals if mid in late)
        assert last_early < first_late, "지연 그룹이 먼저 나갔다"
        # 두 묶음 사이에 실제로 기다려야 한다. 안 그러면 동시 전송과 같다.
        between = [s for i, (kind, s) in enumerate(events)
                   if kind == "sleep" and last_early < i < first_late]
        assert between, "두 묶음 사이에 대기가 없다 (동시 전송과 동일)"
        assert sum(between) == pytest.approx(max(
            hand_config.start_delay(f.name, closing=False) for f in fingers))

    def test_모든_모터의_토크를_끈다(self, hand, monkeypatch):
        monkeypatch.setattr("hand.time.sleep", lambda s: None)
        hand.release()
        assert sorted(m for m, _ in hand._c.torque) == list(range(1, 11))
        assert all(v == 0 for _, v in hand._c.torque)

    def test_펴기가_실패해도_토크는_끈다(self, hand, monkeypatch):
        # 종료 경로의 핵심 규약. 여기가 깨지면 서보가 가열된 채 방치된다.
        monkeypatch.setattr("hand.time.sleep", lambda s: None)

        def boom(mid, angle):
            raise RuntimeError("시리얼 끊김")

        hand._c.write_goal_position = boom
        hand.release()
        assert sorted(m for m, _ in hand._c.torque) == list(range(1, 11))


def test_disconnect_는_토크를_끄지_않고_포트만_놓는다(hand):
    """리스를 반납해도 손은 편 자세를 유지해야 한다.

    토크는 서보 안의 레지스터라 포트를 닫아도 안 꺼진다. 여기서
    write_torque_enable(0) 을 부르면 손이 툭 늘어진다 -- release() 와
    disconnect() 를 가르는 이유가 정확히 이것이다.
    """
    fake = hand._c
    hand.disconnect()
    assert hand._c is None
    assert fake.torque == []
    assert fake.goals == []


def test_disconnect_는_두_번_불려도_죽지_않는다(hand):
    # 종료 경로와 리스 반납 경로에서 둘 다 불릴 수 있다.
    hand.disconnect()
    hand.disconnect()
    assert hand._c is None
