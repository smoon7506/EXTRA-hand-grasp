# -*- coding: utf-8 -*-
"""PoseSequencer 순수 로직 테스트. 모터도 MuJoCo 도 필요 없다.

--- 이 파일의 두 종류 테스트 ---
TestStartDelay 만 hand_config 의 '진짜' 표를 읽는다. 실물에서 확인된
순서를 기록해 두는 것이 목적이라, 순서를 바꾸면 여기가 같이 바뀌어야 한다.

나머지는 전부 합성 표(staggered fixture)를 쓴다. 검증 대상이 '표대로
나가는가'이지 '어느 손가락이 먼저인가'가 아니기 때문이다. 지연값을
튜닝할 때마다 스케줄러 테스트가 깨지면 안 된다.
"""

import pytest

import hand_config
import sequence
from hand_config import Finger


def make_finger(name, id1, id2):
    """테스트용 Finger. 오프셋과 가중치는 스케줄러가 안 보는 값이라 0/1 로 둔다."""
    return Finger(name=name, id1=id1, offset1=0.0, id2=id2, offset2=0.0,
                  flex_weight=1.0, spread_weight=0.0)


class TestStartDelay:
    """실물에서 확인된 순서(2026-08-04). 순서를 바꾸면 여기도 바꾼다."""

    def test_닫힐_때_손가락이_먼저_엄지가_나중이다(self):
        # 엄지가 이미 닫힌 손가락 위를 덮는 형태라 경로가 안 겹친다.
        assert hand_config.start_delay("r_finger1", closing=True) == 0.0
        assert hand_config.start_delay("r_finger5", closing=True) > 0.0

    def test_펴질_때_엄지가_먼저_빠진다(self):
        # 닫을 때의 역순. 엄지가 위에 얹혀 있으니 먼저 치워야 한다.
        assert hand_config.start_delay("r_finger5", closing=False) == 0.0
        assert hand_config.start_delay("r_finger1", closing=False) > 0.0

    def test_표에_없는_이름은_0(self):
        # 왼손이나 새 손가락 이름이 들어와도 예외 대신 '지연 없음'으로
        # 동작해야 한다. 표를 못 찾았다고 손이 안 움직이면 안 된다.
        assert hand_config.start_delay("l_finger9", closing=True) == 0.0
        assert hand_config.start_delay("l_finger9", closing=False) == 0.0

    def test_두_방향이_서로_역순이다(self):
        # 닫을 때 늦게 가는 손가락이 펼 때는 먼저 나와야 한다. 이게
        # 안 지켜지면 어느 한 방향에서 반드시 경로가 겹친다.
        for name in hand_config.ACTIVE_FINGERS:
            close = hand_config.start_delay(name, closing=True)
            open_ = hand_config.start_delay(name, closing=False)
            assert (close > 0.0) != (open_ > 0.0), (
                f"{name}: 닫힘 {close}, 폄 {open_} -- 한쪽만 지연이어야 한다")


class FakeClock:
    """수동으로 굴리는 시계. time.sleep 없이 순서를 검증하려고 쓴다."""

    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


class Recorder:
    """sender 대역. 호출마다 (a, s, 손가락이름목록) 을 기록한다."""

    def __init__(self):
        self.calls = []

    def __call__(self, a, s, fingers):
        self.calls.append((a, s, [f.name for f in fingers]))

    def names_at(self, index):
        return sorted(self.calls[index][2])


LEAD_CLOSE = "r_finger1"        # 합성 표에서 닫을 때 먼저 가는 손가락
LEAD_OPEN = "r_finger5"         # 합성 표에서 펼 때 먼저 가는 손가락
FOLLOW_DELAY = 0.5


@pytest.fixture
def staggered(monkeypatch):
    """합성 지연 표. 실제 값을 튜닝해도 스케줄러 테스트가 안 깨지게 한다.

    어느 손가락이 선발인지는 임의로 정한 것이고, 여기서 확인하는 것은
    '표에 적힌 대로 시각을 맞춰 나가는가' 하나다.
    """
    monkeypatch.setattr(hand_config, "CLOSE_DELAY_S", {
        "r_finger1": 0.0,
        "r_finger2": FOLLOW_DELAY,
        "r_finger3": FOLLOW_DELAY,
        "r_finger4": FOLLOW_DELAY,
        "r_finger5": FOLLOW_DELAY,
    })
    monkeypatch.setattr(hand_config, "OPEN_DELAY_S", {
        "r_finger1": FOLLOW_DELAY,
        "r_finger2": FOLLOW_DELAY,
        "r_finger3": FOLLOW_DELAY,
        "r_finger4": FOLLOW_DELAY,
        "r_finger5": 0.0,
    })


@pytest.fixture
def five():
    """실제 이름을 쓴다 -- 지연 표가 이름으로 조회되기 때문이다."""
    return [
        make_finger("r_finger1", 1, 2),
        make_finger("r_finger2", 3, 4),
        make_finger("r_finger3", 5, 6),
        make_finger("r_finger4", 7, 8),
        make_finger("r_finger5", 9, 10),
    ]


def followers(exclude):
    return sorted(f"r_finger{i}" for i in range(1, 6)
                  if f"r_finger{i}" != exclude)


class TestSequencerClosing:
    def test_첫_tick에는_선발_손가락만_나간다(self, five, staggered):
        clock, rec = FakeClock(), Recorder()
        seq = sequence.PoseSequencer(five, [rec], clock=clock)
        seq.start(1.0)
        assert seq.tick() is True          # 아직 남았다
        assert rec.names_at(0) == [LEAD_CLOSE]

    def test_지연이_지나면_후발이_나간다(self, five, staggered):
        clock, rec = FakeClock(), Recorder()
        seq = sequence.PoseSequencer(five, [rec], clock=clock)
        seq.start(1.0)
        seq.tick()
        clock.advance(FOLLOW_DELAY)
        assert seq.tick() is False         # 마지막 묶음을 보냈다
        assert rec.names_at(1) == followers(LEAD_CLOSE)

    def test_같은_손가락에_두_번_보내지_않는다(self, five, staggered):
        clock, rec = FakeClock(), Recorder()
        seq = sequence.PoseSequencer(five, [rec], clock=clock)
        seq.start(1.0)
        for _ in range(5):
            seq.tick()
        clock.advance(FOLLOW_DELAY * 2)
        for _ in range(5):
            seq.tick()
        sent = [n for call in rec.calls for n in call[2]]
        assert sorted(sent) == sorted(f.name for f in five)

    def test_보낼_게_없으면_tick은_False(self, five, staggered):
        clock, rec = FakeClock(), Recorder()
        seq = sequence.PoseSequencer(five, [rec], clock=clock)
        assert seq.tick() is False         # start 를 안 불렀다
        assert rec.calls == []

    def test_a와_s가_그대로_실린다(self, five, staggered):
        clock, rec = FakeClock(), Recorder()
        seq = sequence.PoseSequencer(five, [rec], clock=clock)
        seq.start(0.8, -0.4)
        seq.tick()
        a, s, _ = rec.calls[0]
        assert (a, s) == (0.8, -0.4)


class TestSequencerOpening:
    def test_펼_때는_폄_표를_쓴다(self, five, staggered):
        clock, rec = FakeClock(), Recorder()
        # last_a=1.0 (주먹 쥔 상태)에서 0 으로 가면 '펴는' 방향이다.
        seq = sequence.PoseSequencer(five, [rec], clock=clock, last_a=1.0)
        seq.start(0.0)
        seq.tick()
        assert rec.names_at(0) == [LEAD_OPEN]
        clock.advance(FOLLOW_DELAY)
        seq.tick()
        assert rec.names_at(1) == followers(LEAD_OPEN)

    def test_같은_값을_다시_주면_닫는_방향으로_친다(self, five, staggered):
        # a 가 그대로면(>=) 닫힘으로 본다. 방향을 정할 근거가 없을 때
        # 어느 쪽이든 하나로 고정돼 있어야 동작이 예측 가능하다.
        clock, rec = FakeClock(), Recorder()
        seq = sequence.PoseSequencer(five, [rec], clock=clock, last_a=0.5)
        seq.start(0.5)
        seq.tick()
        assert rec.names_at(0) == [LEAD_CLOSE]


class TestSequencerMisc:
    def test_sender가_여럿이면_모두_같은_묶음을_받는다(self, five, staggered):
        # 실물 + 시뮬 동시 구동. 한쪽만 받으면 두 손이 어긋난다.
        clock, r1, r2 = FakeClock(), Recorder(), Recorder()
        seq = sequence.PoseSequencer(five, [r1, r2], clock=clock)
        seq.start(1.0)
        seq.tick()
        assert r1.calls == r2.calls

    def test_시퀀스_도중_새_start는_계획을_갈아엎는다(self, five, staggered):
        clock, rec = FakeClock(), Recorder()
        seq = sequence.PoseSequencer(five, [rec], clock=clock)
        seq.start(1.0)
        seq.tick()                         # 선발만 나감
        seq.start(0.0)                     # 아직 후발이 안 나갔는데 폄 명령
        seq.tick()
        # 새 목표(0.0)로 다시 계획이 짜였고, 이번엔 폄 표를 쓴다.
        assert rec.calls[1][0] == 0.0
        assert rec.names_at(1) == [LEAD_OPEN]

    def test_지연이_전부_0이면_한_번에_다_나간다(self, five, monkeypatch):
        # 되돌리기 경로. 표를 비우면 이 기능이 생기기 전과 같아야 한다.
        monkeypatch.setattr(hand_config, "CLOSE_DELAY_S", {})
        monkeypatch.setattr(hand_config, "OPEN_DELAY_S", {})
        clock, rec = FakeClock(), Recorder()
        seq = sequence.PoseSequencer(five, [rec], clock=clock)
        seq.start(1.0)
        assert seq.tick() is False
        assert len(rec.calls) == 1
        assert rec.names_at(0) == sorted(f.name for f in five)


class TestFormatSchedule:
    def test_출발_시각순으로_묶어서_보여준다(self, five, staggered):
        text = sequence.format_schedule(five, closing=True)
        assert "0.00초" in text
        assert f"{FOLLOW_DELAY:.2f}초" in text
        # 선발 손가락이 첫 줄에 혼자 있어야 한다.
        first_line = text.splitlines()[0]
        assert LEAD_CLOSE in first_line
        assert followers(LEAD_CLOSE)[0] not in first_line
