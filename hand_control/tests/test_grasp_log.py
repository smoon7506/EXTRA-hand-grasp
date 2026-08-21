# -*- coding: utf-8 -*-
"""파지 로그. 임계값을 정하려면 실물 데이터가 있어야 한다.

K_THRESHOLD 도 슬립 임계값도 계산으로는 못 뽑는다. 이 파일이 남기는
컬럼이 그대로 나중에 정할 수 있는 값의 범위를 정한다 -- 안 남긴 것은
영영 못 정한다.
"""

import csv

from grasp_log import HEADER, GraspLogger


class FakeFinger:
    def __init__(self, **over):
        self.name = "r_finger1"
        self.state = "HOLD"
        self.a = 0.85
        self.k_hat = 3.9
        self.confident = True
        self.object_class = "soft"
        self.f_target = 1.2
        for k, v in over.items():
            setattr(self, k, v)


def read(path):
    with open(path, encoding="utf-8") as fh:
        return list(csv.DictReader(fh))


class Test전단력_컬럼:
    """슬립은 수직력에 안 보인다(2026-08-21 감사). tf 를 안 남기면
    "미끄러질 때 tf 가 얼마인가"를 영영 못 재고, 슬립 대응도 못 만든다.
    """

    def test_헤더에_tf가_있다(self):
        assert "tf" in HEADER

    def test_tf를_기록한다(self, tmp_path):
        with GraspLogger(root=tmp_path) as log:
            log.row(1.0, FakeFinger(), force=0.9, flex=0.5, shear=1.25)
            path = log.path
        assert read(path)[0]["tf"] == "1.2500"

    def test_tf가_None이면_빈_칸(self, tmp_path):
        # 0 으로 적으면 '전단력 없음'과 '측정 안 됨'이 같아 보인다.
        # force_N 이 이미 같은 규약이다.
        with GraspLogger(root=tmp_path) as log:
            log.row(1.0, FakeFinger(), force=0.9, flex=0.5, shear=None)
            path = log.path
        assert read(path)[0]["tf"] == ""

    def test_shear를_안_주는_옛_호출부도_돈다(self, tmp_path):
        # grasp_main 등 tf 를 아직 안 넘기는 자리가 남아 있다.
        with GraspLogger(root=tmp_path) as log:
            log.row(1.0, FakeFinger(), force=0.9, flex=0.5)
            path = log.path
        assert read(path)[0]["tf"] == ""


class Test기존_컬럼은_그대로다:
    def test_한_줄이_전부_들어간다(self, tmp_path):
        with GraspLogger(root=tmp_path) as log:
            log.row(2.5, FakeFinger(), force=0.9, flex=0.5, shear=0.1)
            path = log.path
        row = read(path)[0]
        assert row["t"] == "2.500"
        assert row["finger"] == "r_finger1"
        assert row["state"] == "HOLD"
        assert row["force_N"] == "0.9000"
        assert row["class"] == "soft"
