# -*- coding: utf-8 -*-
"""벌림 탐색. 접촉을 못 찾은 손가락을 옆으로 훑어 파지 자리를 찾는다.

하드웨어를 모르는 순수 로직이다. 힘을 먹이면 다음에 명령할 벌림을
돌려준다.

--- 무엇을 푸는가 ---
2026-08-21 로그 감사에서 슬립의 성격이 드러났다. HOLD 중 a 는 안 열리고
(Δa 중앙값 0) 수직력도 시간에 따라 안 주는데(a 고정 구간 68개에서
dF/dt 중앙값 0, 감소:증가 46:54 로 대칭) 물체는 빠진다. 즉 손가락이
같은 힘으로 누르는 채로 물체가 그 사이로 흘러내린다 -- 마찰 부족이다.

수직력을 올리면 될 것 같지만 안 된다. 안정 HOLD 손가락의 62% 가 이미
a=A_MAX 포화라 더 조일 여지가 없다.

그래서 남은 레버가 접촉 지오메트리다. 닿는 손가락을 늘리면 마찰점이
는다. 이 파일이 그걸 찾는다.

--- 왜 접촉한 손가락을 건드리지 않나 ---
같이 움직이면 그 손가락의 마찰이 줄어 잡고 있던 물체를 놓는다. 탐색이
파지를 깨뜨리는 셈이 된다. 반대로 못 찾은 손가락은 어차피 허공에
있으므로 옆으로 움직여도 물체를 긁지 않는다. 그래서 이 탐색은 손가락을
풀지 않고도 안전하다 -- kinematics.hand_pose 가 손가락별 s 를 받게
고친 것이 이걸 위해서다.

--- 왜 손가락마다 따로 안 훑나 ---
조합이 폭발하고(N개 지점^k개 손가락), 부채 관계가 깨져서 인접 손가락과
부딪히는 새 경로가 생긴다. 훑는 손가락 전부가 같은 u 를 쓰면 기존
s 의 기하가 그대로 유지된다 -- spread_weight 부호가 이미 서로 벌어지는
방향으로 잡혀 있다.

--- 왜 양방향인가 ---
어느 쪽이 물체 쪽인지 알 방법이 없다. 한쪽만 보면 절반은 못 찾는다.
"""

# 한 계단 크기. SPREAD_LIMIT_DEG=30도 기준 0.1 은 약 3도다.
STEP = 0.1

# 훑는 범위. ±0.3 이면 약 ±9도. 더 벌리면 손가락이 물체를 벗어나거나
# 인접 손가락에 닿기 시작한다.
MAX_S = 0.3

# 한 지점에서 몇 사이클 기다렸다 재나. 서보가 도착하기 전에 재면 힘과
# 자세의 짝이 안 맞는다. PROBE_SETTLE_S 와 같은 이유다.
SETTLE_CYCLES = 3


def _sweep(max_s, step):
    """훑을 u 목록. 0 에서 시작해 양쪽으로 번갈아 벌어진다.

    0 부터 보는 이유: 지금 자리가 이미 최선이면 거기서 끝내고 싶다.
    번갈아 가는 이유: 한쪽을 끝까지 간 뒤 반대로 가면 손가락이 전체
    폭을 가로지르며 물체를 쓸고 지나간다.
    """
    points = [0.0]
    n = int(round(max_s / step))
    for i in range(1, n + 1):
        points.append(round(+i * step, 6))
        points.append(round(-i * step, 6))
    return points


class SpreadSeeker:
    """훑고, 점수를 매기고, 최적점을 고른다.

    seeking : 접촉을 못 찾아 훑을 손가락 이름들
    frozen  : 이미 잡고 있어 건드리면 안 되는 손가락 이름들
    """

    def __init__(self, seeking, frozen, *, step=STEP, max_s=MAX_S,
                 settle_cycles=SETTLE_CYCLES, f_abort=None):
        self.seeking = list(seeking)
        self.frozen = list(frozen)
        self.settle_cycles = max(1, int(settle_cycles))
        self.f_abort = f_abort

        self._points = _sweep(max_s, step)
        self._at = 0
        self._waited = 0
        # (접촉 수, 합계 힘, u). 사전식으로 비교한다 -- 접촉 수가
        # 먼저다. 힘이 큰 한 점보다 닿는 손가락이 많은 쪽이 안 미끄러진다.
        self._best = (0, 0.0, 0.0)
        self.done = False
        self.aborted = False

    # --- 지금 명령할 것 ---------------------------------------------

    @property
    def u(self):
        """지금 시험 중인 벌림 값."""
        if self._at >= len(self._points):
            return self._points[-1]
        return self._points[self._at]

    @property
    def spread(self):
        """지금 명령할 {손가락: s}. 동결된 손가락은 0 이다."""
        return self._map(self.u)

    def _map(self, u):
        out = {name: u for name in self.seeking}
        out.update({name: 0.0 for name in self.frozen})
        return out

    # --- 진행 -------------------------------------------------------

    def update(self, forces, f_touch):
        """한 사이클. 지금 지점의 힘을 먹인다."""
        if self.done:
            return

        values = [v for v in forces.values() if v is not None]

        # 안전이 먼저다. 훑다가 물체나 이웃 손가락에 끼면 계속 밀면
        # 안 된다. 그때까지 찾은 최고점을 들고 그대로 끝낸다.
        if self.f_abort is not None and any(v > self.f_abort for v in values):
            self.aborted = True
            self.done = True
            return

        self._waited += 1
        if self._waited < self.settle_cycles:
            return
        self._waited = 0

        contact = sum(1 for v in values if v > f_touch)
        total = sum(values)
        score = (contact, total, self.u)
        if score[:2] > self._best[:2]:
            self._best = score

        self._at += 1
        if self._at >= len(self._points):
            self.done = True

    # --- 결과 -------------------------------------------------------

    @property
    def best_u(self):
        """가장 좋았던 벌림. 아무 데서도 못 찾았으면 0.0 (제자리)."""
        return self._best[2]

    def best_spread(self):
        """최적 {손가락: s}. 동결된 손가락은 여전히 0 이다."""
        return self._map(self.best_u)
