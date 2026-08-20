# -*- coding: utf-8 -*-
"""파지 로그. 임계값을 정하려면 실물 데이터가 있어야 한다.

K_THRESHOLD 는 센서 감도, 링키지 지오메트리, 물체에 모두 의존해서
계산으로는 못 뽑는다. 스펀지/종이컵/나무토막을 잡아 보고 k_hat 분포를
눈으로 봐야 정할 수 있다. 그래서 이 파일은 부가기능이 아니다.
"""

import csv
import time
from pathlib import Path

HEADER = [
    "t", "finger", "state", "a", "flex_rad", "force_N",
    "k_hat", "confident", "class", "f_target",
]


class GraspLogger:
    """hand_control/logs/YYYY-MM-DD/HH-MM-SS.csv 에 한 줄씩 쓴다.

    기존 hand_control/logs/ 디렉터리 규약을 따른다.
    with 문으로 쓰면 예외가 나도 파일이 닫힌다.
    """

    def __init__(self, root=None):
        if root is None:
            root = Path(__file__).parent / "logs"
        now = time.localtime()
        directory = Path(root) / time.strftime("%Y-%m-%d", now)
        directory.mkdir(parents=True, exist_ok=True)
        self.path = directory / (time.strftime("%H-%M-%S", now) + ".csv")
        self._file = None
        self._writer = None

    def __enter__(self):
        self._file = open(self.path, "w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        self._writer.writerow(HEADER)
        return self

    def __exit__(self, exc_type, exc, tb):
        if self._file is not None:
            self._file.close()
        return False

    def row(self, t, finger, force, flex):
        """FingerGrasp 하나의 현재 상태를 한 줄로."""
        self._writer.writerow([
            f"{t:.3f}",
            finger.name,
            finger.state,
            f"{finger.a:.4f}",
            "" if flex is None else f"{flex:.5f}",
            "" if force is None else f"{force:.4f}",
            "" if finger.k_hat is None else f"{finger.k_hat:.3f}",
            "" if finger.confident is None else int(finger.confident),
            finger.object_class or "",
            "" if finger.f_target is None else f"{finger.f_target:.3f}",
        ])
