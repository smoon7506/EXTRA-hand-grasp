# -*- coding: utf-8 -*-
"""다중 촉각 센서 래퍼. 가짜 드라이버라 하드웨어가 필요 없다."""

import time

import pytest

import hand_config
import tactile
from tactile import TactileHand


class FakeFrame:
    """cap_read.fingerDataPack 대역. 우리가 쓰는 두 필드만 있으면 된다.

    진단 필드(tf 등)는 일부러 안 넣는다 -- 없어도 통과해야 한다는 것이
    _snapshot_frame 의 계약이다. 필요한 테스트는 FakeDiagFrame 을 쓴다.
    """

    def __init__(self, sensor_index, nf):
        self.sensorIndex = sensor_index
        self.nf = list(nf)


class FakeDiagFrame(FakeFrame):
    """진단 필드까지 채운 프레임."""

    def __init__(self, sensor_index, nf, tf=None, tf_dir=None,
                 channels=None, touch_num=None, prj="가짜"):
        super().__init__(sensor_index, nf)
        self.tf = list(tf or [])
        self.tfDir = list(tf_dir or [])
        self.channelCapData = list(channels or [])
        self.touchNum = touch_num
        self.sensorNum = len(self.channelCapData)
        self.prjName = prj


class FakeDriver:
    """cap_read 모듈 대역. 프레임 큐를 사람이 직접 채운다."""

    def __init__(self):
        self.queue = []
        self.exited = False

        class _Thread:
            def start(_self):
                pass

        self.threadFingerRead = _Thread()

    def push(self, sensor_index, nf):
        self.queue.append(FakeFrame(sensor_index, nf))

    def push_frame(self, frame):
        self.queue.append(frame)

    def getFingerData(self, no_wait):
        if self.queue:
            return self.queue.pop(0)
        return None

    def fingerExit(self):
        self.exited = True


@pytest.fixture
def driver():
    return FakeDriver()


@pytest.fixture
def sensors(driver):
    """_poll 스레드를 띄우지 않고 수동으로 큐를 비운다.

    스레드를 쓰면 테스트가 타이밍에 의존해서 간헐적으로 실패한다.
    _drain_once() 를 직접 부르는 쪽이 결정적이다.
    """
    t = TactileHand(driver=driver)
    return t


class Test진단_필드:
    """드라이버는 매 프레임 tf(전단력)와 taxel 별 정전용량을 같이 보낸다.
    제어 루프는 안 쓰지만, 슬립 신호로 쓸 수 있는지 보려면 보존해야 한다.
    """

    def _name(self):
        return hand_config.SENSOR_CHANNEL_MAP[
            sorted(hand_config.SENSOR_CHANNEL_MAP)[0]]

    def _index(self):
        return sorted(hand_config.SENSOR_CHANNEL_MAP)[0]

    def test_tf와_taxel_채널이_보존된다(self, sensors, driver):
        driver.push_frame(FakeDiagFrame(
            self._index(), nf=[1.5], tf=[0.4], tf_dir=[2048],
            channels=[10, 20, 30], touch_num=3))
        sensors._drain_once()
        raw = sensors.read_raw()[self._name()]
        assert raw["tf"] == [0.4]
        assert raw["tfDir"] == [2048]
        assert raw["channelCapData"] == [10, 20, 30]
        assert raw["touchNum"] == 3
        assert raw["nf"] == [1.5]

    def test_없는_필드는_None이다(self, sensors, driver):
        # 가짜 프레임처럼 nf 만 있는 객체도 그대로 통과해야 한다.
        driver.push(self._index(), [1.0])
        sensors._drain_once()
        raw = sensors.read_raw()[self._name()]
        assert raw["tf"] is None
        assert raw["prjName"] is None

    def test_프레임이_없는_손가락은_키가_없다(self, sensors, driver):
        driver.push(self._index(), [1.0])
        sensors._drain_once()
        raw = sensors.read_raw()
        assert self._name() in raw
        assert len(raw) == 1

    def test_사본이라_드라이버가_덮어써도_안_섞인다(self, sensors, driver):
        frame = FakeDiagFrame(self._index(), nf=[1.0], tf=[0.4],
                              channels=[10, 20, 30])
        driver.push_frame(frame)
        sensors._drain_once()
        frame.tf[0] = 99.0          # 드라이버 스레드가 제자리에서 덮어씀
        frame.channelCapData[0] = 99
        raw = sensors.read_raw()[self._name()]
        assert raw["tf"] == [0.4]
        assert raw["channelCapData"] == [10, 20, 30]

    def test_read_forces는_영향을_안_받는다(self, sensors, driver):
        # 진단 경로를 추가해도 제어 경로의 값이 바뀌면 안 된다.
        driver.push_frame(FakeDiagFrame(self._index(), nf=[2.0], tf=[9.9]))
        sensors._drain_once()
        assert sensors.read_forces()[self._name()] == pytest.approx(2.0)


class TestCompute힘:
    def test_taxel_절대값_합에서_baseline을_뺀다(self):
        assert tactile.compute_force([1.0, -2.0, 0.5], 0.5) == pytest.approx(3.0)

    def test_음수로는_안_내려간다(self):
        assert tactile.compute_force([0.1], 5.0) == 0.0

    def test_데이터가_없으면_None(self):
        assert tactile.compute_force(None) is None
        assert tactile.compute_force([]) is None


class Test채널을_안_섞는다:
    def test_다섯_채널이_각자_값을_유지한다(self, sensors, driver):
        # sensor.py 의 단일 _latest 가 못 하던 것. 5개 중 4개를 버렸다.
        for ch in (2, 3, 4, 5, 6):
            driver.push(ch, [float(ch)])
        sensors._drain_once()

        forces = sensors.read_forces()
        assert forces["r_finger1"] == pytest.approx(2.0)
        assert forces["r_finger2"] == pytest.approx(3.0)
        assert forces["r_finger3"] == pytest.approx(4.0)
        assert forces["r_finger4"] == pytest.approx(5.0)
        assert forces["r_finger5"] == pytest.approx(6.0)

    def test_한_채널이_여러_번_오면_최신값만_남는다(self, sensors, driver):
        # 센서 66Hz, 루프 20Hz. 안 비우면 지연이 누적된다.
        driver.push(2, [1.0])
        driver.push(2, [9.0])
        sensors._drain_once()
        assert sensors.read_forces()["r_finger1"] == pytest.approx(9.0)

    def test_안_온_채널은_None(self, sensors, driver):
        driver.push(2, [1.0])
        sensors._drain_once()
        forces = sensors.read_forces()
        assert forces["r_finger1"] == pytest.approx(1.0)
        assert forces["r_finger2"] is None

    def test_매핑에_없는_채널은_무시한다(self, sensors, driver):
        # 조용히 자동 배정하지 않는다. 배선 실수가 숨어버린다.
        driver.push(7, [1.0])
        sensors._drain_once()
        forces = sensors.read_forces()
        assert "r_finger1" in forces
        assert all(v is None for v in forces.values())


class TestSTALE:
    def test_오래된_채널만_None이_된다(self, sensors, driver, monkeypatch):
        # CH341 이 뽑히면 드라이버가 마지막 값을 계속 물고 있다.
        # 이게 없으면 손가락이 조인 채로 영원히 방치된다.
        fake_now = [1000.0]
        monkeypatch.setattr(tactile.time, "monotonic", lambda: fake_now[0])

        driver.push(2, [1.0])
        driver.push(3, [2.0])
        sensors._drain_once()

        fake_now[0] += hand_config.STALE_TIMEOUT_S + 0.1
        driver.push(3, [2.0])            # 3번만 새 프레임
        sensors._drain_once()

        forces = sensors.read_forces()
        assert forces["r_finger1"] is None          # 끊김
        assert forces["r_finger2"] == pytest.approx(2.0)


class Testbaseline:
    """calibrate 는 '새 프레임이 올 때까지' 기다리는 함수다.

    실제로는 센서가 66Hz 로 프레임을 밀어넣지만 테스트에는 폴링
    스레드가 없다. 그래서 tactile.time.sleep 을 가로채서, calibrate 가
    잘 때마다 프레임을 한 장씩 밀어 넣어 센서를 흉내낸다. 스레드를
    띄우는 것보다 결정적이다.
    """

    def test_채널별로_따로_잡힌다(self, sensors, driver, monkeypatch):
        # 센서마다 무부하 값이 다르다. 하나로 뭉치면 어떤 손가락은
        # 항상 눌린 것으로, 어떤 손가락은 항상 0으로 보인다.
        def pump(_seconds):
            driver.push(2, [1.0])
            driver.push(3, [5.0])
            sensors._drain_once()

        monkeypatch.setattr(tactile.time, "sleep", pump)
        assert sensors.calibrate() is True

        # 중앙값 창(MEDIAN_WINDOW=3)이라 새 값이 과반이 돼야 반영된다.
        # 한 장만 밀어 넣으면 옛 값이 여전히 중앙값이다 -- 그게 한 장짜리
        # 글리치를 지우는 원리이고, 대가는 프레임 2장만큼의 지연이다.
        for _ in range(2):
            driver.push(2, [1.5])
            driver.push(3, [5.5])
            sensors._drain_once()
        forces = sensors.read_forces()
        assert forces["r_finger1"] == pytest.approx(0.5)
        assert forces["r_finger2"] == pytest.approx(0.5)

    def test_새_프레임이_없으면_같은_값을_두_번_세지_않는다(
            self, sensors, driver, monkeypatch):
        """_frames 는 채널별 '최신 한 장'만 들고 있다.

        ts 를 안 보면 새 프레임이 안 온 사이클에서도 캐시에 남은 값을
        다시 세고, BASELINE_FRAMES 가 '여러 번 측정의 평균'이 아니라
        '한 측정값'이 된다.

        아래는 프레임을 1.0 -> (없음) -> 3.0 순서로 흘린다.
            제대로 세면 : (1.0 + 3.0) / 2 = 2.0
            두 번 세면  : 1.0 을 두 번 세고 먼저 끝나 1.0
        """
        monkeypatch.setattr(hand_config, "BASELINE_FRAMES", 2)
        monkeypatch.setattr(hand_config, "BASELINE_TIMEOUT_S", 1.0)
        schedule = iter([1.0, None, 3.0])

        def pump(_seconds):
            try:
                value = next(schedule)
            except StopIteration:
                return
            if value is None:
                return          # 이번 사이클에는 새 프레임이 없다
            driver.push(2, [value])
            sensors._drain_once()

        monkeypatch.setattr(tactile.time, "sleep", pump)
        assert sensors.calibrate() is True
        assert sensors._baseline[2] == pytest.approx(2.0)

    def test_타임스탬프가_같아도_서로_다른_프레임으로_센다(
            self, sensors, driver, monkeypatch):
        """Windows 의 time.monotonic() 해상도는 약 15.6ms 인데 센서는
        66Hz(약 15ms) 다. 연속한 두 프레임이 같은 ts 를 받는 일이 흔하다.

        그래서 시계를 얼려 두고, 그래도 두 프레임이 각각 세어지는지
        본다. 판별을 ts 로 하면 두 번째 프레임을 버려서 counts 가 2 에
        영영 못 닿는다.

        시계가 영원히 멈춰 있으면 그 실패가 무한 루프가 되어 테스트가
        끝나지 않는다. 그래서 50번 읽은 뒤에는 deadline 을 훌쩍 넘겨
        루프가 반드시 끝나게 한다 -- 실패는 hang 이 아니라 assert 로
        드러나야 한다.
        """
        monkeypatch.setattr(hand_config, "BASELINE_FRAMES", 2)
        monkeypatch.setattr(hand_config, "BASELINE_TIMEOUT_S", 1.0)
        frozen = iter([1000.0] * 50)
        monkeypatch.setattr(tactile.time, "monotonic",
                            lambda: next(frozen, 2000.0))
        schedule = iter([1.0, 3.0])

        def pump(_seconds):
            try:
                value = next(schedule)
            except StopIteration:
                return
            driver.push(2, [value])
            sensors._drain_once()

        monkeypatch.setattr(tactile.time, "sleep", pump)
        assert sensors.calibrate() is True
        assert sensors._baseline[2] == pytest.approx(2.0)


class TestDiscover:
    def test_들어온_채널의_손가락_이름을_돌려준다(self, sensors, driver):
        driver.push(2, [1.0])
        driver.push(5, [1.0])
        sensors._drain_once()
        assert sorted(sensors.discovered()) == ["r_finger1", "r_finger4"]


class TestStop:
    def test_드라이버를_종료한다(self, sensors, driver):
        sensors.stop()
        assert driver.exited is True

    def test_종료_중_예외가_나도_안_죽는다(self, sensors, driver):
        def boom():
            raise RuntimeError("USB 뽑힘")

        driver.fingerExit = boom
        sensors.stop()   # 예외가 밖으로 나오면 안 된다


class Test스파이크_제거:
    """센서가 한 프레임짜리 이상값을 뱉는다 (2026-08-13 실물 확인).

    1.21 -> 38.63 -> 1.21 로 튀었고, 그 38.63 이 F_ABORT(8N)를 넘겨
    손가락을 헛되이 후퇴시켰다. 안전장치는 제대로 돌았지만 입력이
    가짜였다. 중앙값 창이 이걸 지운다.
    """

    def _fill(self, sensors, driver, value, times=3):
        for _ in range(times):
            driver.push(2, [value])
            sensors._drain_once()

    def test_한_프레임짜리_스파이크는_무시된다(self, sensors, driver):
        self._fill(sensors, driver, 1.21)
        driver.push(2, [38.63])
        sensors._drain_once()
        # 창 = [1.21, 1.21, 38.63] -> 중앙값 1.21
        assert sensors.read_forces()["r_finger1"] == pytest.approx(1.21)

    def test_진짜로_힘이_오르면_따라간다(self, sensors, driver):
        # 스파이크를 지우겠다고 실제 접촉까지 못 보면 쓸모가 없다.
        self._fill(sensors, driver, 0.1)
        self._fill(sensors, driver, 5.0, times=2)
        # 창 = [0.1, 5.0, 5.0] -> 중앙값 5.0
        assert sensors.read_forces()["r_finger1"] == pytest.approx(5.0)

    def test_두_프레임_연속_스파이크는_통과한다(self, sensors, driver):
        # 창 길이 3 의 한계를 명시해 둔다. 실물에서 두 장 연속으로
        # 튀는 게 관찰되면 MEDIAN_WINDOW 를 5 로 올려야 한다.
        self._fill(sensors, driver, 1.0)
        self._fill(sensors, driver, 38.0, times=2)
        assert sensors.read_forces()["r_finger1"] == pytest.approx(38.0)


class Test전단력_읽기:
    """슬립은 접촉면에서 전단력이 마찰 한계를 넘는 현상이다.

    2026-08-21 로그 감사: a 가 3초 이상 고정된 HOLD 구간 68개에서
    dF/dt 중앙값이 0 이고 감소:증가가 46:54 로 대칭이었다. 슬립이라면
    한쪽으로 쏠려야 하는데 안 그렇다 -- 수직력(nf)에는 안 보인다는 뜻이다.
    드라이버는 tf 를 매 프레임 보내주는데 지금까지 로그에 한 줄도 안
    남겼다. 임계값을 정할 데이터가 없어서 슬립 대응을 못 만든다.

    read_forces 와 같은 규약을 지킨다: SENSOR_CHANNEL_MAP 의 모든
    손가락이 키로 나오고, 프레임이 없거나 STALE 이면 None.
    """

    def test_tf를_손가락_이름으로_돌려준다(self):
        driver = FakeDriver()
        t = TactileHand(driver=driver)
        driver.push_frame(FakeDiagFrame(2, [0.0], tf=[1.25]))
        t._drain_once()
        assert t.read_shear()["r_finger1"] == pytest.approx(1.25)

    def test_tf가_없는_개체는_None(self):
        # 개체에 따라 tf 를 안 보낸다. 0.0 으로 바꾸면 '전단력 없음'과
        # '측정 안 됨'이 같아 보여서 임계값을 잘못 잡는다.
        driver = FakeDriver()
        t = TactileHand(driver=driver)
        driver.push_frame(FakeDiagFrame(2, [0.0], tf=[]))
        t._drain_once()
        assert t.read_shear()["r_finger1"] is None

    def test_프레임이_없으면_None(self):
        t = TactileHand(driver=FakeDriver())
        assert t.read_shear()["r_finger1"] is None

    def test_모든_손가락이_키로_나온다(self):
        t = TactileHand(driver=FakeDriver())
        assert (set(t.read_shear())
                == set(hand_config.SENSOR_CHANNEL_MAP.values()))
