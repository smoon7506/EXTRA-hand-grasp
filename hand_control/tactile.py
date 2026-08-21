# -*- coding: utf-8 -*-
"""촉각 센서 5개 래퍼. 원본 드라이버를 수정하지 않고 감싼다.

--- tactile_motor_test/sensor.py 와 무엇이 다른가 ---
그쪽 _poll 은 큐를 끝까지 비우고 '마지막 프레임 하나'만 들고 있었다.
센서가 1개일 때는 맞았지만 5개가 되면 매 사이클 4개를 버리고 임의의 한
손가락만 남는다 -- read_force() 가 부를 때마다 다른 손가락 값을 준다.

드라이버(cap_read.py:305)는 센서 5개의 프레임을 하나의 큐에 섞어서
넣고 각 프레임에 sensorIndex(= PCA 채널)를 붙인다. 그래서 여기서는
채널별로 최신 프레임을 따로 보관한다. baseline 과 STALE 판정도
채널별로 독립이다.
"""

import sys
import threading
import time

import hand_config

# 스파이크 제거 창 길이(서로 다른 프레임 장수). 홀수여야 중앙값이 하나로
# 정해진다. 3 이면 한 장짜리 글리치는 확실히 지워지고, 두 장 연속으로
# 같은 방향으로 튀는 경우만 통과한다. 지연은 프레임 1장(약 30~90ms)이다.
MEDIAN_WINDOW = 3

# 진단용으로 보존하는 프레임 필드. 제어 루프는 nf 만 쓰지만, 드라이버는
# 매 프레임 이만큼을 실어 보낸다 (cap_read.pack_from_finger 참고).
#
#   tf / tfDir        전단력 크기와 방향. 슬립은 접촉면에서 전단력이
#                     마찰 한계를 넘는 현상이라 이게 직접 신호다.
#   channelCapData    taxel 별 원시 정전용량. prg=44 기준 14채널 중
#                     앞 13개가 힘 채널(touchNum)이다. 접촉 패치의
#                     중심이 움직이는 것도 슬립 신호다.
#   prjName 등        어느 손가락 프로젝트인지. nf/tf 배열 길이(ydds_num)
#                     와 스케일(/100 여부)이 여기 달려 있다.
#
# 여기 있는 값들은 아직 제어에 안 쓴다. 먼저 --sensor-diag 로 실물에서
# 살아 있는지 확인해야 한다 -- 개체에 따라 0 만 나올 수 있다.
_DIAG_FIELDS = (
    "tf", "tfDir", "channelCapData", "sProxCapData", "mProxCapData",
    "sensorNum", "touchNum", "prjName",
)


def _snapshot_frame(frame):
    """진단 필드를 사본으로 뜬다. 없는 필드는 None 이다.

    테스트의 가짜 프레임처럼 nf 만 있는 객체도 그대로 통과해야 한다.
    """
    snapshot = {}
    for field in _DIAG_FIELDS:
        value = getattr(frame, field, None)
        snapshot[field] = list(value) if isinstance(value, list) else value
    return snapshot


def compute_force(nf, baseline=0.0):
    """taxel 별 수직항력 리스트 -> 총 힘(N).

    데이터가 없으면 None. baseline 을 뺀 뒤 0 으로 clamp 한다.
    tactile_motor_test/sensor.py:11-21 과 같은 계산이다.
    """
    if nf is None:
        return None
    if len(nf) == 0:
        return None
    total = float(sum(abs(float(v)) for v in nf))
    return max(0.0, total - baseline)


def _import_driver():
    """센서 드라이버를 import 한다.

    class_ch341.py:53-54 는 Windows 에서 DLL 경로를
        os.path.dirname(sys.argv[0]) + '/lib/ch341/windows/CH341DLLA64.DLL'
    로 만든다. 즉 모듈 위치가 아니라 '실행 스크립트' 위치 기준이다.
    패치하지 않으면 DLL 을 못 찾고, 드라이버는 예외 없이 조용히
    재연결만 반복해서 데이터가 영원히 오지 않는다.
    """
    capread = str(hand_config.CAPREAD_DIR)
    sys.argv[0] = str(hand_config.CAPREAD_DIR / "__entry__.py")
    if capread not in sys.path:
        sys.path.insert(0, capread)
    import cap_read

    return cap_read


class TactileHand:
    """센서 5개 -> {손가락이름: 힘(N) 또는 None}.

    driver 를 주면 그것을 쓰고, None 이면 실제 드라이버를 import 한다.
    테스트는 가짜 드라이버를 주입한다.
    """

    def __init__(self, driver=None):
        self._cap = driver
        # {sensorIndex: (nf 사본, monotonic ts, seq)}
        #
        # seq 가 왜 따로 필요한가: calibrate 는 "이 채널에 새 프레임이
        # 왔나"를 판단해야 하는데, Windows 의 time.monotonic() 해상도는
        # 약 15.6ms 이고 센서는 66Hz(약 15ms 간격)다. 연속한 두 프레임이
        # 같은 ts 를 받는 일이 흔하다. ts 로 새 프레임을 판별하면 진짜
        # 새 프레임을 조용히 버린다. seq 는 저장할 때마다 1 씩 느는
        # 정수라 그런 일이 없다.
        #
        # ts 는 그대로 남긴다. STALE 판정(1초 단위)에는 15.6ms 해상도가
        # 충분하고, 거기에는 실제 경과 시간이 필요하다.
        self._frames = {}
        self._baseline = {}        # {sensorIndex: float}
        self._seq = 0              # 저장한 프레임 수. 위 주석 참고
        self._recent = {}          # {sensorIndex: [최근 프레임의 raw 힘]}
        # {sensorIndex: 진단 필드 사본}. 제어 경로는 안 쓴다 -- read_raw
        # 로만 읽는다. _DIAG_FIELDS 주석 참고.
        self._raw = {}
        self._lock = threading.Lock()
        self._stop = threading.Event()

    # ---------- 수명주기 ----------

    def start(self):
        """드라이버와 폴링 스레드를 띄운다."""
        if self._cap is None:
            self._cap = _import_driver()
        self._cap.threadFingerRead.start()
        # daemon: _poll 은 블로킹 큐에서 깨울 방법이 없다. 프로세스
        # 종료에 맡긴다. sensor.py:54-55 와 같은 이유.
        threading.Thread(target=self._poll, daemon=True).start()

    def _poll(self):
        while not self._stop.is_set():
            frame = self._cap.getFingerData(False)
            if frame is None:
                continue
            self._store(frame)
            self._drain_once()

    def _drain_once(self):
        """큐에 쌓인 프레임을 전부 가져와 채널별 최신값으로 만든다.

        센서는 약 66Hz, 제어 루프는 20Hz 다. 비우지 않으면 지연이
        누적된다. 단일 최신값이 아니라 '채널별' 최신값을 남기는 것이
        sensor.py 와의 결정적 차이다.
        """
        while True:
            frame = self._cap.getFingerData(True)
            if frame is None:
                break
            self._store(frame)

    def _store(self, frame):
        index = getattr(frame, "sensorIndex", None)
        nf = getattr(frame, "nf", None)
        if index is None or nf is None:
            return
        # nf 는 드라이버 스레드가 제자리에서 계속 덮어쓰는 리스트다.
        # 사본을 떠 두지 않으면 한 합산 안에 서로 다른 두 프레임의
        # taxel 이 섞인다 (sensor.py:77-79 와 같은 이유).
        with self._lock:
            self._seq += 1
            index = int(index)
            self._frames[index] = (list(nf), time.monotonic(), self._seq)
            # 스파이크 제거용 창. 프레임 '한 장' 단위로 쌓는다.
            #
            # 왜 필요한가 (2026-08-13 실물 로그):
            #   t=9.031 F=1.21
            #   t=9.078 F=38.63   <- 한 프레임짜리 글리치
            #   t=9.187 F=1.21
            # 683 샘플 중 2개가 튀었고 그게 F_ABORT(8N)를 넘겨 손가락을
            # 후퇴시켰다. 안전장치는 제대로 돌았지만 입력이 가짜였다.
            #
            # 루프 샘플이 아니라 프레임 단위로 모으는 이유: 제어 루프가
            # 센서보다 빠를 때 같은 프레임을 두 번 읽는다. 위 로그에서
            # 38.63 이 두 번 연속 나온 게 그것이다. 루프 샘플로 창을
            # 만들면 한 장짜리 글리치가 중앙값을 차지할 수 있다.
            window = self._recent.setdefault(index, [])
            window.append(compute_force(nf, 0.0))
            if len(window) > MEDIAN_WINDOW:
                del window[0]
            # 진단 필드도 같은 락 안에서 같이 갈아 끼운다. 따로 두면
            # read_raw 가 nf 와 짝이 안 맞는 tf 를 볼 수 있다.
            self._raw[index] = _snapshot_frame(frame)

    def stop(self):
        """드라이버에 종료를 알린다. 주입된 드라이버든 직접 import 한
        드라이버든 똑같이 대한다 -- 둘을 나눠 봤자 하는 일이 같다.

        _poll 스레드는 기다리지 않는다. getFingerData(False) 에 블로킹돼
        있으면 밖에서 깨울 방법이 없기 때문이다. daemon 스레드라
        프로세스 종료에 맡긴다.
        """
        self._stop.set()
        if self._cap is None:
            return
        try:
            self._cap.fingerExit()
        except Exception as e:
            # 한 손이 종료에 실패해도 호출부의 정리 경로는 계속돼야 한다.
            print(f"[WARN] 센서 종료 중 오류: {e}")

    # ---------- 조회 ----------

    def _snapshot(self):
        with self._lock:
            return dict(self._frames)

    def discovered(self):
        """지금까지 프레임이 한 번이라도 온 손가락 이름들."""
        names = []
        for index in self._snapshot():
            name = hand_config.SENSOR_CHANNEL_MAP.get(index)
            if name is not None:
                names.append(name)
        return names

    def wait_ready(self, timeout):
        """아무 채널이든 첫 프레임이 올 때까지 기다린다."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._snapshot():
                return True
            time.sleep(0.05)
        return False

    def discover(self, seconds):
        """어떤 채널이 붙었는지 모은다. -> 손가락 이름 목록.

        스캔 순서가 채널마다 다르므로 첫 프레임만 보고 판단하면 실제로
        붙어 있는 센서를 놓친다. 잠깐 모아서 본다.
        """
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            time.sleep(0.05)
        found = self.discovered()
        unmapped = sorted(set(self._snapshot())
                          - set(hand_config.SENSOR_CHANNEL_MAP))
        if unmapped:
            # 조용히 자동 배정하지 않는다. 배선 실수가 숨어버린다.
            print(f"[WARN] SENSOR_CHANNEL_MAP 에 없는 PCA 채널 {unmapped} "
                  f"에서 데이터가 온다. 무시한다. hand_config 를 확인할 것.")
        return found

    def calibrate(self):
        """채널별 무부하 baseline. 호출 전 센서를 만지지 말 것.

        센서마다 무부하 값이 다르므로 반드시 채널별로 잡는다. 하나로
        뭉치면 어떤 손가락은 항상 눌린 것으로, 어떤 손가락은 항상
        0 으로 보인다.

        --- 같은 프레임을 두 번 세지 않는다 ---
        이 루프는 20ms 마다 도는데 _frames 는 채널별 '최신 한 장'만
        들고 있다. 새 프레임인지 안 보면 아직 갱신되지 않은 채널에서
        캐시에 남은 값을 반복해서 더하게 된다. 그러면 결과는
        'BASELINE_FRAMES 번 측정의 평균'이 아니라 '한 측정값'이 된다 --
        평균을 내는 것이 이 함수의 유일한 목적인데 그게 사라진다.

        판별에 ts 가 아니라 seq 를 쓴다. Windows 의 time.monotonic()
        해상도는 약 15.6ms 인데 센서는 66Hz(약 15ms) 라서 연속한 두
        프레임이 같은 ts 를 받는 일이 흔하다. ts 로 판별하면 진짜 새
        프레임을 버려서 샘플이 조용히 절반으로 줄고, 최악의 경우
        BASELINE_TIMEOUT_S 까지 돌다가 한 장으로 평균을 낸다.
        """
        sums = {}
        counts = {}
        last_seq = {}
        deadline = time.monotonic() + hand_config.BASELINE_TIMEOUT_S
        while time.monotonic() < deadline:
            for index, (nf, _ts, seq) in self._snapshot().items():
                if last_seq.get(index) == seq:
                    continue        # 아직 새 프레임이 안 왔다
                value = compute_force(nf, 0.0)
                if value is None:
                    continue
                last_seq[index] = seq
                sums[index] = sums.get(index, 0.0) + value
                counts[index] = counts.get(index, 0) + 1
            if counts and min(counts.values()) >= hand_config.BASELINE_FRAMES:
                break
            time.sleep(0.02)

        if not counts:
            return False
        self._baseline = {i: sums[i] / counts[i] for i in counts}
        return True

    def read_forces(self):
        """-> {손가락이름: 힘(N) 또는 None}.

        SENSOR_CHANNEL_MAP 의 모든 손가락이 키로 나온다. 프레임이 없거나
        STALE 이면 그 손가락만 None 이다. 호출부(grasp)는 None 을 받은
        손가락만 동결하고 나머지는 계속 진행한다.
        """
        # 프레임 스냅샷과 중앙값 창을 같은 락 안에서 같이 떠 온다.
        # 따로 뜨면 그 사이에 폴링 스레드가 창을 밀어서, 어떤 채널은
        # 새 프레임인데 창은 옛날 것인 상태가 섞일 수 있다.
        with self._lock:
            frames = dict(self._frames)
            recent = {i: list(w) for i, w in self._recent.items()}
        now = time.monotonic()
        forces = {}
        for index, name in hand_config.SENSOR_CHANNEL_MAP.items():
            entry = frames.get(index)
            if entry is None:
                forces[name] = None
                continue
            nf, ts, _seq = entry
            if now - ts > hand_config.STALE_TIMEOUT_S:
                # 오래된 값을 계속 주면 센서가 끊겨도 마지막 값이
                # 영원히 유효해 보인다. 손가락이 조인 채로 방치된다.
                forces[name] = None
                continue
            # 중앙값을 쓴다. 원시 프레임 하나가 튀어도(위 _store 주석)
            # 그 값이 F_ABORT 를 넘겨 손가락을 헛되이 후퇴시키지 않는다.
            # baseline 은 중앙값을 낸 뒤에 뺀다 -- 순서를 바꾸면 clamp(0)
            # 때문에 창 안의 값들이 0 으로 뭉개져 중앙값이 왜곡된다.
            window = recent.get(index)
            if window:
                raw = sorted(window)[len(window) // 2]
            else:
                raw = compute_force(nf, 0.0)
            forces[name] = max(0.0, raw - self._baseline.get(index, 0.0))
        return forces

    def read_shear(self):
        """-> {손가락이름: 전단력 또는 None}. read_forces 와 같은 규약이다.

        --- 왜 수직력만으로는 부족한가 ---
        슬립은 접촉면에서 전단력이 마찰 한계를 넘는 현상이다. 손가락이
        같은 힘으로 누르는 채로 물체가 그 아래로 흘러내리면 nf 는 거의
        안 변한다. 2026-08-21 로그 감사가 그걸 확인했다 -- a 가 3초 이상
        고정된 HOLD 구간 68개에서 dF/dt 중앙값이 0 이고 감소:증가가
        46:54 로 대칭이었다. 슬립이라면 한쪽으로 쏠려야 한다.

        --- 지금은 로그용이다 ---
        제어에는 아직 안 쓴다. 임계값을 정하려면 "미끄러질 때 tf 가
        얼마인가"의 실측이 필요한데 그 데이터가 한 줄도 없다. 먼저
        쌓는다.

        tf 가 빈 배열이면 None 이다. 개체에 따라 전단력을 안 보내는데,
        0.0 으로 바꾸면 '전단력 없음'과 '측정 안 됨'이 같아 보여서
        임계값을 잘못 잡는다. baseline 은 빼지 않는다 -- nf 와 달리
        무부하 기준을 아직 안 재봤고, 원시값 그대로 쌓아야 나중에
        어떻게 다룰지 정할 수 있다.
        """
        with self._lock:
            raw = {i: dict(d) for i, d in self._raw.items()}
            frames = dict(self._frames)
        now = time.monotonic()
        shear = {}
        for index, name in hand_config.SENSOR_CHANNEL_MAP.items():
            entry = frames.get(index)
            snapshot = raw.get(index)
            if entry is None or snapshot is None:
                shear[name] = None
                continue
            if now - entry[1] > hand_config.STALE_TIMEOUT_S:
                shear[name] = None
                continue
            tf = snapshot.get("tf")
            shear[name] = float(tf[0]) if tf else None
        return shear

    def read_raw(self):
        """-> {손가락이름: 진단 필드 dict}. 프레임이 없는 손가락은 키가 없다.

        제어 루프는 이걸 안 쓴다. --sensor-diag 로 tf(전단력)와 taxel
        분포가 실물에서 살아 있는지 확인하기 위한 것이다.

        read_forces 와 달리 STALE 을 걸러내지 않는다. 대신 age_s 를 같이
        주므로 보는 쪽이 판단한다 -- 진단에서는 "값이 안 바뀐다"는 사실
        자체가 봐야 할 정보다.
        """
        with self._lock:
            raw = {i: dict(d) for i, d in self._raw.items()}
            frames = dict(self._frames)
        now = time.monotonic()
        out = {}
        for index, name in hand_config.SENSOR_CHANNEL_MAP.items():
            snapshot = raw.get(index)
            if snapshot is None:
                continue
            entry = frames.get(index)
            snapshot["nf"] = list(entry[0]) if entry else []
            snapshot["age_s"] = None if entry is None else now - entry[1]
            snapshot["baseline"] = self._baseline.get(index, 0.0)
            out[name] = snapshot
        return out
