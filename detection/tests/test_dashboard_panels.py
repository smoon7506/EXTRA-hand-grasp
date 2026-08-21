# -*- coding: utf-8 -*-
"""촉각 패널이 요약하는 값. 그리기 전 계산만 순수하게 뽑아 검증한다."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import pytest

from dashboard import panels


class Test힘_요약:
    def test_합계와_접촉_개수(self):
        s = panels.summarize({"f1": 1.0, "f2": 0.1, "f3": 0.5}, f_touch=0.3)
        assert s.total == pytest.approx(1.6)
        assert s.contact == 2          # 0.1 은 f_touch 미만
        assert s.n == 3

    def test_센서가_없으면_합계가_None(self):
        # 0.0 이 아니라 None 이다. '안 눌림'과 '센서 없음'을 화면에서
        # 갈라야 한다 -- 0N 으로 그리면 손이 멀쩡한데 왜 반응이 없나로
        # 원인을 못 찾는다.
        s = panels.summarize({}, f_touch=0.3)
        assert s.total is None
        assert s.n == 0

    def test_못_읽은_손가락은_합계에서_빠지고_개수에는_남는다(self):
        # 채널이 끊긴 손가락. 0 으로 세면 합계가 조용히 낮아진다.
        s = panels.summarize({"f1": 1.0, "f2": None}, f_touch=0.3)
        assert s.total == pytest.approx(1.0)
        assert s.contact == 1
        assert s.n == 2
        assert s.missing == ["f2"]

    def test_전부_못_읽으면_합계가_None(self):
        s = panels.summarize({"f1": None}, f_touch=0.3)
        assert s.total is None


class Test추세:
    def test_오르면_상승(self):
        assert panels.trend([1.0, 1.2, 1.5, 1.9]) == "up"

    def test_내리면_하강(self):
        assert panels.trend([1.9, 1.5, 1.2, 1.0]) == "down"

    def test_평평하면_유지(self):
        assert panels.trend([1.0, 1.01, 0.99, 1.0]) == "flat"

    def test_표본이_모자라면_모른다(self):
        # 붙자마자 화살표가 튀면 안 된다.
        assert panels.trend([]) is None
        assert panels.trend([1.0]) is None


class Test옛_데몬을_구분한다:
    """telemetry 에 forces 키가 아예 없는 것과 빈 dict 는 다른 상황이다.

    키 없음  = 데몬이 옛 버전이다(파이에 배포가 안 됐다)
    빈 dict  = 데몬은 새것인데 센서가 없다(--simple-grasp / --no-hand)

    둘을 똑같이 "sensor off" 로 그리면, 배포를 안 한 것을 센서 문제로
    오해하고 엉뚱한 데를 뒤진다 -- 2026-08-21 에 실제로 그랬다.
    """

    def test_None이면_옛_데몬(self):
        assert panels.source(None) == "stale_daemon"

    def test_빈_dict면_센서_없음(self):
        assert panels.source({}) == "no_sensor"

    def test_값이_있으면_정상(self):
        assert panels.source({"f1": 1.0}) == "ok"

    def test_전부_None이어도_정상_경로다(self):
        # 채널이 다 끊긴 것. 데몬은 살아 있고 센서도 붙어 있다.
        assert panels.source({"f1": None}) == "ok"
