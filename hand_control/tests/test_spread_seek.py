# -*- coding: utf-8 -*-
"""벌림 탐색. 하드웨어를 모르는 순수 로직.

--- 무엇을 푸는가 ---
2026-08-21 로그 감사: HOLD 중 a 는 안 열리고(Δa 중앙값 0) 수직력도
시간에 따라 안 줄었는데(dF/dt 중앙값 0, 감소:증가 46:54 대칭) 물체는
빠진다. 손가락이 같은 힘으로 누르는 채로 물체가 흘러내린다는 뜻이고,
수직력은 62% 가 이미 a_max 포화라 더 못 올린다.

그래서 남은 레버는 접촉 지오메트리다. 접촉점을 늘리면 마찰점이 는다.

--- 왜 접촉한 손가락을 안 건드리나 ---
같이 움직이면 마찰이 줄어 잡고 있던 물체를 놓는다. 탐색이 파지를
깨뜨리는 셈이다. 못 찾은 손가락은 어차피 허공이라 움직여도 물체를
긁지 않는다.
"""

import pytest

from spread_seek import SpreadSeeker

F_TOUCH = 0.3


def forces(**over):
    base = {"f1": 0.0, "f2": 0.0, "f3": 0.0, "f4": 0.0}
    base.update(over)
    return base


def run_to_end(seeker, score_by_u, cycles=200):
    """u 에 따라 힘이 달라지는 가짜 손으로 탐색을 끝까지 돌린다.

    score_by_u: {u 반올림값: forces dict}
    """
    for _ in range(cycles):
        if seeker.done:
            break
        u = round(seeker.u, 3)
        seeker.update(score_by_u.get(u, forces()), F_TOUCH)
    return seeker


class Test동결:
    def test_잡고_있는_손가락은_안_움직인다(self):
        s = SpreadSeeker(["f1", "f2"], frozen=["f3", "f4"])
        s.update(forces(), F_TOUCH)
        assert s.spread["f3"] == 0.0
        assert s.spread["f4"] == 0.0

    def test_훑는_손가락은_같은_u를_공유한다(self):
        # 손가락별로 따로 훑으면 조합이 폭발하고, 부채 관계가 깨져
        # 인접 손가락과 부딪히는 새 경로가 생긴다.
        s = SpreadSeeker(["f1", "f2"], frozen=[])
        s.update(forces(), F_TOUCH)
        assert s.spread["f1"] == s.spread["f2"]


class Test훑기:
    def test_범위를_안_넘는다(self):
        s = SpreadSeeker(["f1"], frozen=[], max_s=0.3, step=0.1)
        seen = set()
        for _ in range(200):
            if s.done:
                break
            seen.add(round(s.u, 3))
            s.update(forces(), F_TOUCH)
        assert min(seen) >= -0.3
        assert max(seen) <= 0.3

    def test_양쪽을_다_본다(self):
        # 어느 방향이 물체 쪽인지 모른다. 한쪽만 보면 반은 못 찾는다.
        s = SpreadSeeker(["f1"], frozen=[], max_s=0.2, step=0.1)
        seen = set()
        for _ in range(200):
            if s.done:
                break
            seen.add(round(s.u, 3))
            s.update(forces(), F_TOUCH)
        assert min(seen) < 0 < max(seen)

    def test_각_지점에서_안정화를_기다린다(self):
        # 서보가 도착하기 전에 재면 힘과 자세의 짝이 안 맞는다.
        s = SpreadSeeker(["f1"], frozen=[], settle_cycles=3)
        first = s.u
        s.update(forces(), F_TOUCH)
        s.update(forces(), F_TOUCH)
        assert s.u == first          # 아직 같은 지점
        s.update(forces(), F_TOUCH)
        assert s.u != first


class Test최적점_고르기:
    def test_접촉이_가장_많은_지점을_고른다(self):
        s = SpreadSeeker(["f1", "f2"], frozen=[], max_s=0.2, step=0.1,
                         settle_cycles=1)
        run_to_end(s, {0.1: forces(f1=0.9, f2=0.9)})
        assert s.done
        assert s.best_u == pytest.approx(0.1)

    def test_접촉_수가_같으면_합계_힘으로_가른다(self):
        s = SpreadSeeker(["f1"], frozen=[], max_s=0.2, step=0.1,
                         settle_cycles=1)
        run_to_end(s, {0.1: forces(f1=0.5), 0.2: forces(f1=1.5)})
        assert s.best_u == pytest.approx(0.2)

    def test_아무_데서도_못_찾으면_제자리로(self):
        # 헛되이 벌린 채로 두면 다음 파지가 그 자세에서 시작한다.
        s = SpreadSeeker(["f1"], frozen=[], max_s=0.2, step=0.1,
                         settle_cycles=1)
        run_to_end(s, {})
        assert s.best_u == pytest.approx(0.0)

    def test_최적_벌림도_동결_손가락은_0이다(self):
        s = SpreadSeeker(["f1"], frozen=["f2"], max_s=0.1, step=0.1,
                         settle_cycles=1)
        run_to_end(s, {0.1: forces(f1=0.9)})
        assert s.best_spread() == {"f1": pytest.approx(0.1), "f2": 0.0}


class Test안전:
    def test_힘이_상한을_넘으면_즉시_멈춘다(self):
        # 옆으로 훑다가 물체나 다른 손가락에 끼었다. 계속 밀면 안 된다.
        s = SpreadSeeker(["f1"], frozen=[], settle_cycles=1, f_abort=8.0)
        s.update(forces(f1=9.0), F_TOUCH)
        assert s.done
        assert s.aborted is True

    def test_멈추면_그때까지_최고점을_쓴다(self):
        s = SpreadSeeker(["f1"], frozen=[], max_s=0.3, step=0.1,
                         settle_cycles=1, f_abort=8.0)
        s.update(forces(f1=0.9), F_TOUCH)      # 첫 지점에서 접촉
        good = s.best_u
        s.update(forces(f1=9.0), F_TOUCH)      # 다음 지점에서 끼임
        assert s.done
        assert s.best_u == pytest.approx(good)
