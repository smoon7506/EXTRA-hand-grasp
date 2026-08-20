# 라즈베리파이 사전 준비

파이가 맡을 일: **D405 카메라 + 손(SCS0009 ×10) + 손목(STS3215) + 촉각 센서**를 물고 판정·구동 루프를 돈다. PC는 화면과 키 입력만 담당한다.

설계: `docs/superpowers/specs/2026-08-18-pi-pc-split-design.md`

측정 환경: Pi 5 Model B Rev 1.1 / aarch64 / RAM 8GB / Python 3.13.5 (Raspberry Pi OS Trixie)

---

## 한눈에 보기

| # | 항목 | 상태 | 막히면 |
|---|---|---|---|
| 0 | apt 패키지 + venv | ✅ 완료 | — |
| 1 | `rustypot` | ✅ **된다** | 서보를 못 돌림 (해소됨) |
| 2 | `pyrealsense2` | 🔄 소스 빌드 | D405를 못 염 |
| 3 | CH341 촉각 센서 커널 모듈 | ⬜ 미착수 | 촉각 힘 제어 불가 (`--simple-grasp` 로 우회 가능) |
| 4 | 코드 옮기기 | ⬜ 미착수 | 아무것도 못 함 |
| 5 | USB 권한 / udev | ⬜ 미착수 | permission denied 로 헤맴 |

디스크는 librealsense 빌드에 **3GB 정도** 필요하다. 시간은 2번이 30~60분, 3번이 5분, 나머지는 금방이다.

---

## 0. 이미 하신 것

```bash
sudo apt update
sudo apt install python3-opencv python3-venv python3-dev cmake \
                 libssl-dev libusb-1.0-0-dev pkg-config pybind11-dev
python3 -m venv venv --system-site-packages
source venv/bin/activate
```

`--system-site-packages` 가 중요하다. apt로 깐 `python3-opencv` 와 `python3-numpy` 를 venv 안에서 그대로 쓴다 — 파이에서 opencv를 소스 빌드하면 한 시간씩 걸린다.

Trixie는 PEP 668을 강제해서 시스템 Python에 직접 `pip install` 이 거부된다. **앞으로 모든 pip 명령은 venv를 켠 상태에서** 한다:

```bash
source ~/venv/bin/activate
```

---

## 1. rustypot ✅ 해결됨

```bash
pip install rustypot
```

`~/venv/lib/python3.13/site-packages/rustypot/` 에 설치된 것을 확인했다. PyO3 abi3 휠이라 Python 3.13 aarch64에서 그대로 붙는다 — Rust 툴체인이 필요 없다.

**이게 이 프로젝트에서 제일 큰 고비였다.** 손과 손목을 돌리는 유일한 경로라서, 이게 안 되면 파이로 옮기는 것 자체가 무의미했다.

---

## 2. pyrealsense2 — 소스 빌드

Python 3.13 aarch64용 휠은 사실상 없다. librealsense를 직접 빌드한다.

```bash
source ~/venv/bin/activate
cd ~ && git clone --depth 1 https://github.com/IntelRealSense/librealsense.git
cd librealsense && mkdir -p build && cd build

cmake .. -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_EXAMPLES=false -DBUILD_GRAPHICAL_EXAMPLES=false \
  -DBUILD_PYTHON_BINDINGS=true \
  -DPYTHON_EXECUTABLE=$(which python) \
  -DFORCE_RSUSB_BACKEND=true

make -j4 && sudo make install
```

**`-DFORCE_RSUSB_BACKEND=true` 가 핵심이다.** 이게 없으면 librealsense가 커널 패치를 요구하는데, 라즈베리파이 커널에 그 패치를 넣으면 **커널 업데이트마다 깨진다.** USB 백엔드로 가면 커널을 안 건드려도 되고 성능 차이도 이 용도에서는 없다.

확인:

```bash
python -c "import pyrealsense2 as rs; print('rs OK', rs.__version__)"
```

`ModuleNotFoundError` 가 나면 빌드는 됐는데 경로를 못 찾는 것이다. 위치를 찾아서 `PYTHONPATH` 에 넣는다:

```bash
sudo find /usr/local/lib -name "pyrealsense2*"
# 예: /usr/local/lib/python3.13/site-packages/pyrealsense2/
echo 'export PYTHONPATH=$PYTHONPATH:/usr/local/lib/python3.13/site-packages' >> ~/.bashrc
```

빌드가 메모리 부족으로 죽으면 `make -j2` 로 낮춘다.

---

## 3. CH341 촉각 센서 — 커널 모듈이 필요하다

Tashan 촉각 센서는 CH341 USB-I²C 브리지를 통해 붙는다. **윈도우와 달리 리눅스에서는 커널 모듈을 직접 빌드해서 올려야 한다.** 이건 미리 알아두지 않으면 실물 브링업 때 막힌다.

### 3-1. 커널 헤더

```bash
sudo apt install raspberrypi-kernel-headers
# 위가 없다고 하면:
sudo apt install linux-headers-rpi-v8
```

### 3-2. 드라이버 빌드 — 경로의 `&` 때문에 그 자리에서는 안 된다

**드라이버 폴더를 `&` 없는 곳으로 복사해서 빌드해야 한다.** 원래 자리에서 `make` 하면 이렇게 깨진다:

```
/bin/sh: 1: Linux-64bit/lib/ch341/CH341PAR_LINUX/driver: not found
*** specified external module directory ".../capRead_Python-win" does not exist.
```

Makefile 이 `M=$(PWD)` 를 따옴표 없이 넘기는데, 셸이 경로 안의 `&` 를 백그라운드 연산자로 읽어 경로를 두 동강 낸다.

```bash
cp -r ~/haram_code/tactile_sensor/"capRead_Python-win&Linux-64bit-20260727T045120Z-1-001"/"capRead_Python-win&Linux-64bit"/lib/ch341/CH341PAR_LINUX/driver ~/ch341-driver

cd ~/ch341-driver
make                       # ch34x_pis.ko 가 생기면 성공
sudo insmod ch34x_pis.ko   # 먼저 올려서 확인
sudo make install          # 되면 영구 등록
```

드라이버는 어디서 빌드하든 상관없다 — 커널에 올라가는 것이라 파이썬 코드 위치와 무관하다. 파이썬 쪽은 `/dev/ch34x_pis*` 만 찾는다.

커널 6.18.39 (Trixie / Pi 5) 에서 경고만 나고 빌드된다.

### 3-3. aarch64 라이브러리 등록

레포에 아키텍처별 `.so` 가 이미 들어 있다. **aarch64 것을** 시스템 경로에 넣는다:

```bash
cd ~/haram_code/tactile_sensor/"capRead_Python-win&Linux-64bit-20260727T045120Z-1-001"/"capRead_Python-win&Linux-64bit"/lib/ch341/CH341PAR_LINUX
sudo cp lib/aarch64/dynamic/libch347.so /usr/lib/
sudo ldconfig
```

`class_ch341.py` 는 aarch64 에서 `/usr/lib/libch347.so` 를 읽도록 고쳐 두었다. 이 복사를 안 하면 `未找到库文件` 로 실패한다.

### 3-4. 확인

센서를 꽂고:

```bash
lsusb | grep 1a86          # CH341 의 VID 는 0x1A86
ls /dev/ch34x_pis*         # 드라이버가 올라왔으면 여기 보인다
```

**`lsusb` 에 보이는 것과 열리는 것은 별개다.** 장치가 목록에 있어도 커널 모듈이 없으면 `/dev/ch34x_pis*` 가 안 생기고, `class_ch341.open()` 이 `glob('/dev/ch34x_pis*')` 로 찾으므로 `No CH341 device found on Linux` 로 실패한다.

증상별로 어느 단계가 빠졌는지:

| 로그 | 빠진 것 |
|---|---|
| `未找到库文件` (라이브러리 없음) | 3-3 복사 |
| `ch341加载成功` 다음 `No CH341 device found on Linux` | 3-2 커널 모듈 |
| `CH341 device open failed on Linux` | 권한 (`ls -l /dev/ch34x_pis*`) |

촉각 센서가 안 되어도 `--simple-grasp` 옵션으로 고정 자세 파지는 된다. 즉 이 항목이 막혀도 전체가 멈추지는 않는다 — 먼저 카메라·각도·손목을 확인하고 나중에 돌아오는 편이 문제를 가리기 쉽다.

---

## 4. 코드 옮기기 — 주의할 점이 있다

**이 저장소에는 git 원격이 없다.** GitHub에서 clone 할 수 없으므로 PC에서 직접 밀어 넣어야 한다. 게다가 **`tactile_sensor/` 와 `tactile_motor_test/` 는 git에 추적조차 안 되고 있어서**, 설령 원격이 있어도 clone 으로는 안 따라온다.

### 방법 A — PC에서 scp 로 직접 (가장 단순)

PC(Windows PowerShell)에서:

```powershell
# 필요한 것만 골라서 보낸다. AmazingHand-main2 같은 큰 폴더는 파이에 필요 없다.
scp -r C:\Users\desktop\Desktop\haram_code\detection      wearlab@192.168.137.236:~/haram_code/
scp -r C:\Users\desktop\Desktop\haram_code\hand_control   wearlab@192.168.137.236:~/haram_code/
scp -r C:\Users\desktop\Desktop\haram_code\tactile_sensor wearlab@192.168.137.236:~/haram_code/
```

### 방법 B — 파이를 git 원격으로 (앞으로 계속 밀어 넣을 거라면 이쪽)

파이에서:

```bash
mkdir -p ~/haram_code.git && cd ~/haram_code.git && git init --bare
```

PC에서:

```powershell
git remote add pi wearlab@192.168.137.236:~/haram_code.git
git push pi roi-align-grasp
```

파이에서 작업 트리로 꺼내기:

```bash
git clone ~/haram_code.git ~/haram_code && cd ~/haram_code && git checkout roi-align-grasp
```

방법 B를 쓰더라도 `tactile_sensor/` 는 추적되지 않으므로 **scp 로 따로** 보내야 한다.

### SSH 키를 넣어두면 편하다

지금은 접속할 때마다 비밀번호를 묻는다. PC에서 한 번만:

```powershell
ssh-keygen -t ed25519 -f $HOME\.ssh\id_ed25519
type $env:USERPROFILE\.ssh\id_ed25519.pub | ssh wearlab@192.168.137.236 "mkdir -p ~/.ssh && cat >> ~/.ssh/authorized_keys && chmod 700 ~/.ssh && chmod 600 ~/.ssh/authorized_keys"
```

---

## 5. USB 권한과 udev

안 해두면 나중에 `permission denied` 로 한참 헤맨다.

```bash
# 서보 버스(URT-1) 와 CH341 접근용
sudo usermod -aG dialout,plugdev $USER

# D405 udev 규칙
sudo curl -o /etc/udev/rules.d/99-realsense-libusb.rules \
  https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

**그룹 변경은 로그아웃 후 다시 접속해야 적용된다.**

### 서보 포트 이름 확인

윈도우의 `COM11` 이 파이에서는 `/dev/ttyUSB0` 이 된다. URT-1을 꽂고:

```bash
ls -l /dev/ttyUSB*
dmesg | tail -20          # 어느 장치로 잡혔는지
```

포트가 여러 개면 `/dev/serial/by-id/` 로 고정 이름을 쓰는 게 안전하다.

---

## 6. 전원 — 이것 때문에 프레임이 튈 수 있다

D405는 USB3에서 700mA 가까이 쓴다. 파이 5 포트에 D405 + URT-1 + CH341을 다 직결하면 프레임 드롭이나 USB 재열거(re-enumeration)로 나타날 수 있다.

증상이 보이면 **셀프파워 USB3 허브**를 쓴다. 미리 하나 준비해두면 원인 추적에 쓸 시간을 아낀다.

서보 전원은 USB와 별개다. 서보 전원이 안 들어오면 포트가 정상으로 보여도 모든 모터가 `Parsing error` 로 나온다 — 그때는 코드가 아니라 전원부터 본다.

---

## 7. 최종 확인 — 한 번에 복사해서 붙여넣기

```bash
source ~/venv/bin/activate
echo "== Python =="; python --version
echo "== rustypot =="; python -c "import rustypot; print('OK')" 2>&1 | tail -1
echo "== pyrealsense2 =="; python -c "import pyrealsense2 as rs; print('OK', rs.__version__)" 2>&1 | tail -1
echo "== cv2 / numpy =="; python -c "import cv2, numpy; print('OK', cv2.__version__, numpy.__version__)" 2>&1 | tail -1
echo "== 코드 =="; ls ~/haram_code/detection/roi_grasp.py ~/haram_code/hand_control/hand.py 2>&1
echo "== CH341 드라이버 =="; ls /dev/ch34x_pis* 2>&1 | head -1
echo "== 서보 포트 =="; ls /dev/ttyUSB* 2>&1
echo "== 그룹 =="; groups | tr ' ' '\n' | grep -E 'dialout|plugdev' || echo "  dialout/plugdev 없음 - 재로그인 필요"
echo "== USB 장치 =="; lsusb
```

---

## 깔지 않아도 되는 것

파이에 필요 없다. PC가 담당한다.

- **PyTorch / ultralytics (YOLO)** — 무거운 비전은 PC의 GPU가 한다. 파이의 판정은 깊이 임계 + PCA 뿐이라 2ms도 안 걸린다
- **MuJoCo** — 시뮬레이션은 PC 전용
- **ROS2 / dora** — 이 프로젝트는 미들웨어를 안 쓴다. PC↔파이는 평문 TCP + 줄단위 JSON 이다
- **Rust 툴체인** — `rustypot` 이 휠로 붙어서 필요 없어졌다
- **ESP32 관련 도구** — 설계에서 뺐다. 모터 구동은 1Mbaud 버스에 sync_write 0.4ms 라 이미 공짜고, ESP32를 넣으면 얻는 것 없이 경계만 하나 더 생긴다

---

## 순서 요약

1. ✅ apt + venv
2. ✅ `pip install rustypot`
3. 🔄 librealsense 빌드 (`FORCE_RSUSB_BACKEND` 잊지 말 것)
4. 코드 옮기기 (scp 또는 파이를 bare 원격으로) + SSH 키
5. `usermod -aG dialout,plugdev` + realsense udev → **재로그인**
6. 커널 헤더 → CH341 드라이버 `make && sudo make install` → `libch347.so` (aarch64) 복사
7. 7번 확인 스크립트 돌려서 결과 공유

3번과 6번이 실패할 수 있는 지점이다. 막히면 에러 메시지를 그대로 보내주면 된다.
