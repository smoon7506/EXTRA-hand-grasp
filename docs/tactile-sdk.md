# 촉각 센서 SDK 붙이기

이 저장소에는 촉각 센서 드라이버가 **없다.** Tashan(센서)과 WCH(CH341 브리지 칩)
소프트웨어라 공개 재배포하지 않는다. 별도로 받아서 경로만 알려주면 된다.

**없어도 나머지는 다 돈다.** `--simple-grasp` 로 촉각 없이 고정 자세 파지를 쓸 수
있고, 카메라·정렬·손목·상태기계는 전부 정상 동작한다. 촉각 힘 제어와 강성 분류,
벌림 탐색만 못 쓴다.

---

## 무엇을 받나

`tactile-sdk-backup.zip` — 이 프로젝트를 넘길 때 같이 주는 묶음이다. 안에 이게 들어 있다:

```
tactile-sdk-backup/
  capRead_Python-win&Linux-64bit/     ← 이게 CAPREAD_DIR 이 가리킬 폴더
    cap_read.py                       tactile.py 가 import 하는 것
    class_ch341.py                    *** 제조사 원본이 아니다. 아래 참고 ***
    class_finger.py  class_sensorcmd.py  sensorPara.py
    tactile_geometry.py               taxel 물리 배치 (--sensor-diag 가 쓴다)
    lib/ch341/                        .so / .DLL / 커널 모듈 소스
  README.txt                          백업 자체 설명
  Tashan ... Product Manual (1).pdf   센서 매뉴얼
  리파인 택타일 센서 사용법 ....txt      윈도우 GUI 로 먼저 확인하는 법
```

센서를 새로 사면 제조사가 같은 것을 준다. 다만 **제조사 원본에는 아래 aarch64
수정이 없다.**

---

## 어디에 두나

기본값은 저장소 옆의 `vendor/capRead` 다. 여기에 두면 설정이 필요 없다.

```bash
# 저장소 루트에서
mkdir -p vendor
unzip ~/tactile-sdk-backup.zip -d /tmp/
cp -r "/tmp/tactile-sdk-backup/capRead_Python-win&Linux-64bit" vendor/capRead
```

**폴더 이름의 `&` 때문에 반드시 따옴표로 감싼다.** 안 그러면 셸이 백그라운드
실행으로 해석해서 엉뚱한 데를 복사한다.

확인:

```bash
ls vendor/capRead/cap_read.py
```

`vendor/` 는 `.gitignore` 에 있다. 저장소에 안 올라간다.

### 다른 데 두고 싶으면

```bash
export CAPREAD_DIR=~/어디든/capRead_Python-win\&Linux-64bit
```

`hand_config.py` 가 이 환경변수를 먼저 본다. 셸을 새로 열 때마다 풀리므로,
계속 쓸 거면 `~/.bashrc` 에 넣는다.

---

## 라즈베리파이에서 추가로 필요한 것

리눅스는 파이썬 파일만으로 안 된다. **두 가지가 더 있어야 한다.**
자세한 절차와 함정은 `pi-setup.md` 3장에 있고, 여기서는 무엇이 왜 필요한지만 짚는다.

### 1. `libch347.so` (aarch64) 를 `/usr/lib` 에

```bash
sudo cp vendor/capRead/lib/ch341/CH341PAR_LINUX/lib/aarch64/dynamic/libch347.so /usr/lib/
```

안 하면 `未找到库文件`(라이브러리를 못 찾음)로 실패한다.

### 2. CH341 커널 모듈 빌드

```bash
# & 없는 곳으로 복사해서 빌드한다 -- 제자리에서 make 하면 깨진다
cp -r vendor/capRead/lib/ch341/CH341PAR_LINUX/driver ~/ch341drv
cd ~/ch341drv && make && sudo make install
sudo modprobe ch34x_pis
```

`&` 가 든 경로에서 `make` 하면 이렇게 깨진다:

```
/bin/sh: 1: Linux-64bit/lib/ch341/CH341PAR_LINUX/driver: not found
```

모듈이 올라오면 `/dev/ch34x_pis0` 가 생긴다. **`lsusb` 에 장치가 보이는 것과는
별개다** — 보여도 모듈이 없으면 못 연다.

---

## ⚠ `class_ch341.py` 는 수정본이다

제조사 원본이 x86_64 경로를 하드코딩해서 라즈베리파이(aarch64)에서 실패한다.
백업본에는 `init()` 안에 분기가 들어가 있다:

```python
if platform.machine() in ['aarch64', 'arm64']:
    libPath = '/usr/lib/libch347.so'
else:
    libPath = os.path.join(script_dir, 'lib', 'ch341', 'CH341PAR_LINUX',
                           'lib', 'x64', 'dynamic', 'libch347.so')
```

파일 맨 위에 `import platform` 도 추가돼 있다.

**제조사에서 SDK 를 새로 받아 덮어쓰면 이 수정이 날아간다.** 갑자기
`未找到库文件` 가 뜨면 이걸 먼저 의심한다.

같은 이유로 **`vendor/` 는 파이 배포에서 제외한다**(`pi_manifest.txt` 참고).
PC 판으로 덮으면 파이에서 고친 것이 사라진다.

---

## 잘 붙었는지 확인

### 1) 채널 매핑 — 이게 먼저다

```bash
python hand_control/grasp_main.py --sensor-only
```

손가락 끝을 **하나씩** 눌러 화면의 이름과 실제 손가락이 맞는지 본다.

**틀리면 검지 힘을 보고 새끼를 조인다.** 증상만 보고는 절대 못 찾는 종류의
버그라 반드시 여기서 확인한다. 어긋나면 `hand_config.SENSOR_CHANNEL_MAP` 을
고친다(PCA9548 채널 → 손가락 이름).

### 2) 전단력이 살아 있나

```bash
python hand_control/grasp_main.py --sensor-diag r_finger4
```

손가락 위에 물체를 올리고 **옆으로 밀어** 본다.

| 관찰 | 뜻 |
|---|---|
| `nf` 는 그대로인데 `tf` 가 오른다 | 전단력이 살아 있다. 슬립 감지에 쓸 수 있다 |
| `tf` 가 0 만 나온다 | 개체가 전단력을 안 보낸다. taxel 중심 이동을 대신 본다 |
| 시작하자마자 `tf 배열이 비어 있습니다` | 이 개체는 전단력을 아예 안 준다 |

### 3) 실제 파지에서

데몬을 띄우고 콘솔을 붙이면 오른쪽에 촉각 패널이 뜬다.

| 표시 | 뜻 |
|---|---|
| `f1~f5` 막대와 숫자 | 정상 |
| `sensor off` | 데몬은 새것인데 센서가 없다 — `CAPREAD_DIR` 또는 커널 모듈 |
| `no force in telemetry` | 데몬이 옛 버전. 배포하고 **재시작**했는지 |

`n/a` 는 그 채널만 끊긴 것이다. `0.00`(안 눌림)과 다르다.

---

## 안 될 때

| 증상 | 원인 |
|---|---|
| `未找到库文件` | `libch347.so` 를 `/usr/lib` 에 안 넣었거나, `class_ch341.py` 수정이 날아갔다 |
| `ch341加载成功` 다음 `No CH341 device found on Linux` | 커널 모듈 미로드. `sudo modprobe ch34x_pis` |
| `CH341 device open failed on Linux` | 권한. `ls -l /dev/ch34x_pis*` 로 확인 |
| `make` 가 경로를 못 찾음 | 폴더 이름의 `&`. `driver/` 를 `&` 없는 곳으로 복사 |
| 데이터가 영영 안 옴 (에러도 없음) | 윈도우에서 DLL 경로를 못 찾는 경우다. `tactile.py` 의 `_import_driver` 가 `sys.argv[0]` 를 패치해 해결한다 |
| 손가락 이름이 뒤바뀜 | `SENSOR_CHANNEL_MAP`. 위 1)번 |

**센서가 안 되어도 전체가 멈추지는 않는다.** 카메라·각도·손목을 먼저 확인하고
나중에 돌아오는 편이 문제를 가리기 쉽다.

---

## 윈도우 GUI 로 먼저 확인하기

센서 자체가 살아 있는지는 제조사 GUI 로 보는 게 제일 빠르다.
(백업 안 `리파인 택타일 센서 사용법` 메모 요약)

1. `CH341PAR.EXE` 설치
2. `Tashan_Tactile_Sensor_Software_260312` 폴더의 `SensorShow.exe` 실행
3. 처음엔 중국어다. 전체화면 후 **우측 상단에서 English** 로 바꾼다
4. `connect` 를 누르면 알아서 핀을 잡는다

여기서 값이 보이면 하드웨어는 정상이고, 문제는 파이썬 쪽(경로·커널 모듈)이다.
이 GUI 와 `CH341PAR.EXE` 는 윈도우 전용이라 백업 zip 에는 안 들어 있다.

---

## 라이선스

벤더 SDK 에는 이 저장소의 Apache 2.0 이 **적용되지 않는다.** Tashan 과 WCH 각
제조사의 조건을 따른다. 그래서 저장소에 포함하지 않고 경로로만 참조한다.
