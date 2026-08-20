# EXTRA-hand-grasp — D405 + EXTRA Hand 자율 파지

카메라로 물체를 보고, 손목을 돌려 각도를 맞춘 뒤, 잡는다.
**라즈베리파이(판정·구동)와 PC(화면·입력) 두 프로세스로 나뉘어 있다.**

```
파이 (grasp_daemon.py)                     PC (grasp_console.py)
  D405 → 깊이 판정 → 상태기계 → 모터
         └─ 33ms 안에 전부 파이 안에서 끝난다
              │
              │  텔레메트리 465B + JPEG + ROI마스크 345B   (5001/5002)
              ├─────────────────────────────────────────→  화면에 그림
              │                                               사람이 키를 누름
              ←─────────────────────────────────────────┤
                 명령 {"cmd":"capture_target"}
```

**깊이 배열은 네트워크로 안 나간다.** 640×480 float32 는 1.2MB/프레임이다.
화면이 필요로 하는 건 ROI 크기 이진 마스크(약 345B)뿐이라 그것만 보낸다.

**네트워크가 판정 루프 안에 없다.** WiFi 가 끊겨도 손목이 정렬 도중에 굳지 않는다.

---

## 하드웨어

<img width="400" height="400" alt="image" src="https://github.com/user-attachments/assets/0d8d9a55-846f-4456-98fa-8dae1afb0b2e" />

| 장치 | 연결 | VID:PID |
|---|---|---|
| EXTRA Hand — SCS0009 ×10 | URT-1 → `/dev/ttyUSB0` | `1a86:7523` |
| 손목 — STS3215 | **같은 버스** (URT-1 커넥터는 병렬) | — |
| Intel RealSense D405 | USB3 | `8086:0b5b` |
| Tashan 촉각 센서 ×5 | CH341 I2C → `/dev/ch34x_pis0` | `1a86:5512` |

손과 손목이 같은 시리얼 버스를 쓴다. 동시에 못 열어서 `servo_bus.ServoLease`
로 시분할하고, 상태에 따라 데몬이 자동으로 넘긴다.

---

## 설치

**저장소 하나를 양쪽에 그대로 clone 한다.** 나누지 않는다 — `link.py` 는 양쪽이
같은 파일이어야 하고, 한쪽만 고치면 통신이 깨진다. 전부 합쳐 187KB 라 쓰지 않는
파일이 몇 개 같이 있어도 손해가 아니다.

```bash
git clone https://github.com/smoon7506/EXTRA-hand-grasp.git
cd EXTRA-hand-grasp
```

### 파이 (라즈베리파이 5)

```bash
python3 -m venv venv --system-site-packages && source venv/bin/activate
pip install -r requirements.txt
```

`pyrealsense2` 소스 빌드와 CH341 커널 모듈이 더 필요하다 —
**`docs/pi-setup.md` 를 따라간다.** 거기 함정이 몇 개 있다.

촉각 센서 SDK(Tashan capRead)는 이 저장소에 없다. 센서와 같이 받은 것을 쓰고
경로를 알려준다(`export CAPREAD_DIR=...`). 없으면 `--simple-grasp` 로 촉각 없이
돌릴 수 있다.

### PC (윈도우)

```powershell
python -m venv venv; .\venv\Scripts\activate
pip install numpy opencv-python
```

**PC 에는 `pyrealsense2` 도 `rustypot` 도 필요 없다.** 카메라와 모터를 안 만진다.

---

## 어느 파일이 어디서 도나

| 어디서 | 파일 |
|---|---|
| **파이에서만** | `detection/` — `grasp_daemon` `roi_judge` `orientation` `wrist_align` `grasp_state` `grasp_commands` `roi_grasp`<br>`hand_control/` — **전부**<br>`config/r_hand.toml` |
| **PC 에서만** | `detection/` — `grasp_console` `console_input` |
| **양쪽 공용** | `detection/` — `link` `link_sender` `roi_config` |

`hand_control/` 은 통째로 파이 쪽이다. 모터와 촉각 센서를 만지는 코드라 PC 에는
필요 없다.

`link.py` 는 프로토콜이라 **양쪽이 같은 파일을 써야 한다.** 이것 하나 때문에
저장소를 나누지 않는다.

### 하드웨어는 전부 파이에 붙는다

D405, URT-1(손+손목), 촉각 센서 모두 파이 USB 에 꽂는다. PC 에는 아무것도 안
꽂는다 — 화면과 키보드만 쓴다.

---

## 실행

터미널 두 개가 필요하다. **파이를 먼저 띄운다** — 콘솔이 클라이언트라 데몬이
없으면 `ConnectionRefused` 로 바로 죽는다.

**터미널 1 — 파이 (SSH)**

```bash
source ~/venv/bin/activate
cd ~/EXTRA-hand-grasp/detection
python grasp_daemon.py
```

창이 안 뜨는 게 정상이다. 화면은 PC 가 그린다.
SSH 가 끊기면 데몬도 죽으니 오래 띄울 거면 `tmux` 를 쓴다.

**터미널 2 — PC**

```powershell
cd EXTRA-hand-grasp\detection
python grasp_console.py --host <파이IP>
```

여기서 영상 창이 뜬다. `--host` 는 파이 IP 다(기본값 `127.0.0.1` 은 같은 기계에서
쓸 때만 맞다).

붙으면 화면에 `DISARMED` 가 보인다. **`m` 을 눌러야 판정이 시작된다** —
부팅하자마자 손이 알아서 잡으면 안 되기 때문이다.

데몬 옵션: `--no-hand`(카메라만) `--no-wrist` `--simple-grasp`(촉각 대신 고정 자세)
`--preview-fps` `--preview-scale`

처음이면 **`docs/pi-run.md` 의 4단계 브링업**을 따라간다. 한 번에 다 붙이면
안 될 때 원인을 못 가린다.

### 키

| 키 | | 키 | |
|---|---|---|---|
| 드래그 | ROI 지정 | `t` | 지금 각도를 목표로 |
| `m` | **무장 / 해제** | `,` `.` | 목표 각도 ±1도 |
| `a` | 자동 정렬 on/off | `h` | 손 마스크 저장 (물체 치우고) |
| `n` `f` | 밴드 near/far 캘리브레이션 | `w` `W` | 손목 수동 조그 |
| `[` `]` | near ±5mm | `r` | 놓기 |
| `-` `=` | far ±5mm | `space` | **비상 폄** |
| | | `q` | 콘솔 종료 (데몬은 계속 산다) |

---

## 파일

### 진입점 3개

| | 무엇 |
|---|---|
| `detection/grasp_daemon.py` | **파이.** 카메라·손·손목·촉각을 물고 판정 루프. 화면 없음 |
| `detection/grasp_console.py` | **PC.** 화면을 그리고 키를 명령으로 보낸다 |
| `hand_control/main.py` | 손 수동 조작 (`a`=굽힘 `s`=벌림). 브링업·점검용 |

### `detection/` — 비전과 판정

| 묶음 | 파일 |
|---|---|
| 통신 | `link` (프로토콜) `link_sender` (막히면 버림) `link_watchdog` (끊김 대응) `grasp_commands` (명령→동작) |
| 판정 | `roi_judge` (깊이→물체 유무) `orientation` (장축 각도) `wrist_align` (각도→손목 goal) `grasp_state` (상태기계) |
| 설정·입력 | `roi_config` (ROI·밴드·목표각) `console_input` (키·좌표 환산) |
| 기준선 | `roi_grasp.py` — 아래 참고 |

판정 4개는 **하드웨어도 화면도 네트워크도 모른다.** 그래서 카메라 없이 전부 테스트된다.

### `hand_control/` — 모터와 촉각

| 묶음 | 파일 |
|---|---|
| 하드웨어 | `hand` (손 10모터) `wrist` (STS3215) `servo_bus` (포트 시분할) `tactile` (촉각 5개) |
| 파지 | `grasp_runner` `grasp` `force_control` `stiffness` `sequence` `grasp_log` |
| 계산·설정 | `kinematics` `hand_config` |
| 도구 | `main.py` (수동 조작) `servo_id_tool.py` (서보 ID 변경) |

### `config/r_hand.toml`

모터 ID 1~10 과 오프셋. **이 저장소가 이 파일의 주인이다.**
손을 새로 조립했거나 서보를 바꿨으면 여기만 고친다. 사본을 두 곳에 두지 말 것 —
예전에 오프셋 사본이 원본과 어긋나 손가락이 엉뚱한 각도로 간 적이 있다.

---

## 반드시 알아야 할 규약 5가지

이 다섯 개만 지키면 큰 사고는 안 난다.

### 1. 콘솔은 "값"이 아니라 "동작"을 보낸다

`n` 키(밴드 캘리브레이션)는 *그 프레임의* median 으로 값을 정한다. 콘솔이 받은
median 은 이미 100ms 낡았으므로, 콘솔이 계산해서 보내면 **캘리브레이션이 조용히
틀린다.**

| 키 | 틀린 방식 | 올바른 방식 |
|---|---|---|
| `n`/`f` | `{"set_band":{"near_m":0.183}}` | `{"cmd":"calib_band","edge":"near"}` |
| `t` | `{"set_target_angle":8.2}` | `{"cmd":"capture_target"}` |
| `h` | 마스크 배열을 올려보냄 | `{"cmd":"save_hand_mask"}` |

`[ ] - =` 와 `,` `.` 는 원래 "현재 값에서 얼마만큼"이라 상대량을 보내면 맞다.

### 2. 무장은 항상 사람이 건다

데몬은 `armed=False` 로 기동한다. 링크가 끊겼다 돌아와도 **자동 재무장하지
않는다.** 사람이 `m` 을 눌러야 한다.

### 3. 부분 송신된 프레임은 반드시 마저 보낸다

논블로킹 `send()` 는 커널 버퍼에 들어가는 만큼만 받는다. 이미 나간 바이트는
되돌릴 수 없으므로 나머지를 버리면 수신 측 스트림이 **영구히** 어긋난다
(`미리보기 프레임이 3537097941 바이트다` 로 터진 적이 있다).

`DropSender` 가 꼬리를 `_pending` 에 들고 있다가 `flush()` 로 마저 보낸다.
그동안 들어온 새 프레임은 통째로 버린다 — **버리는 자리는 항상 프레임 경계다.**

### 4. `roi.json` / `hand_mask.npy` 는 파이가 소유한다

판정하는 쪽이 가져야 PC 와 어긋나지 않는다. PC 는 `detection/backup/` 에 사본만
떨군다(git 에 안 들어간다). **이 백업을 읽어서 쓰지 않는다.**

### 5. `roi_grasp.py` 는 기준선이다

분할 전과 같이 동작하는 단일 프로세스 버전이다. 모니터를 붙이면 이것만으로 돈다.

```bash
python roi_grasp.py
```

데몬+콘솔이 이상할 때 이걸 돌려 원인을 가른다 — 여기서도 이상하면 분리 탓이
아니고(하드웨어·캘리브레이션), 여기서 멀쩡하면 분리하면서 생긴 문제다.

이 파일은 `roi_config`/`roi_judge`/`grasp_state` 를 import 해서 쓴다.
자체 정의를 갖고 있으면 옛 통짜본이 덮인 것이다:

```bash
grep -c "^def band_ratio" detection/roi_grasp.py   # 0 이어야 정상
```

---

## 링크가 끊기면

PC 가 2초간 아무것도 안 보내면 끊긴 것으로 본다(30Hz 기준 60프레임).

| 그때 상태 | 조치 |
|---|---|
| ARMED | 무장 해제 |
| ALIGNING / CONFIRMING | 즉시 물러나 무장 해제 |
| GRASPING | **끝까지 진행** → HOLDING (중간에 멈추면 손가락이 반쯤 닫힌 채 끼인다) |
| HOLDING | 30초 후 자동 폄. 링크가 돌아오면 타이머 취소 |
| RELEASING | 끝까지 |

과열 보호는 별개로 항상 돈다 — 1초마다 온도를 읽고, 한계를 넘거나 못 읽으면 편다.

---

## 안 될 때

| 증상 | 원인 |
|---|---|
| 모든 모터가 `Parsing error` | **서보 전원.** 포트가 정상으로 보여도 그렇다 |
| `No CH341 device found on Linux` | 커널 모듈 미로드. `lsusb` 에 보이는 것과 별개다 |
| CH341 `make` 가 경로를 못 찾음 | 폴더 이름의 `&` 때문. `driver/` 를 `&` 없는 곳으로 복사해서 빌드 |
| 물체가 손에 있는데 안 잡힘 | `near_m` 이 D405 Min-Z(약 7cm)보다 낮으면 그 구간은 센서가 원래 못 본다 |
| 영상이 뚝뚝 끊김 | `--preview-fps 8 --preview-scale 0.5`. 판정은 30Hz 그대로. D405 가 USB 2.0 포트인지도 확인 |
| ROI 박스가 드래그한 자리와 다름 | 좌표 환산. `--preview-scale 1.0` 으로 돌려 확인 |
| 키를 눌렀는데 반응 없음 | PC 터미널에 `[NAK]` 로 이유가 뜬다 |
| 손이 접힌 채로 있음 | `hand.connect()` 후 `hand.release()`. 현재 위치를 먼저 읽고 토크를 켜므로 안 튄다 |

시리얼 포트가 다르면 환경변수로 덮는다:

```bash
export HAND_SERIAL_PORT=/dev/ttyUSB1
```

---

## 테스트 — 고치기 전에 이것부터

```bash
cd detection     && python -m pytest tests/ -q    # 184개
cd hand_control  && python -m pytest tests/ -q    # 258개
```

전부 **하드웨어 없이** 돈다. 카메라도 모터도 없는 노트북에서 몇 초면 끝난다.

### 왜 남겨 뒀나

그대로 쓰기만 할 거면 필요 없다. 하지만 손이 다르거나 서보가 다르거나 카메라가
바뀌면 값을 고쳐야 하고, **그때 안 깨졌는지 확인할 유일한 수단이 이것이다.**

일부는 물리적 사고를 막는다. 예를 들어 `hand_config.py` 에 이런 표가 있다:

```python
CLOSE_DELAY_S = {..., "r_finger5": 0.6}   # 엄지가 마지막에 덮는다
OPEN_DELAY_S  = {"r_finger5": 0.0, ...}   # 펼 때는 엄지가 먼저
```

`test_sequence.py` 의 `test_두_방향이_서로_역순이다` 가 이 둘이 역순인지 본다.
한쪽만 고치면 **펼 때 엄지가 검지 경로 위에 남아 링키지가 부딪힌다.** 테스트가
없으면 손을 부순 뒤에 안다.

`test_grasp_states.py` 도 마찬가지다 — 상태 전이를 잘못 건드리면 손이 물체를
쥔 채 안 놓거나 반쯤 닫힌 채 멈춘다.

### 문서로도 읽힌다

테스트 이름이 한글이라 "이 코드가 무엇을 보장하는가"를 그대로 말해 준다.
README 보다 정확하다 — 코드가 바뀌면 테스트가 깨지기 때문이다.

```bash
python -m pytest tests/ -q --collect-only    # 무엇을 보장하는지 목록으로
```

**코드를 고쳤으면 파이에 올리기 전에 여기서 먼저 돌린다.**

---

## 더 읽을 것

| 문서 | 내용 |
|---|---|
| `docs/pi-setup.md` | 파이 사전 준비. rustypot / librealsense / CH341 커널 모듈 / USB 권한 |
| `docs/pi-run.md` | 4단계 브링업과 문제 해결 |
| `docs/design.md` | **왜 이렇게 나눴는지.** 연산 측정치, ESP32 를 뺀 이유, 계약 설계 |

---

## 이 저장소에 없는 것

**촉각 센서 SDK.** Tashan 센서와 WCH CH341 칩의 드라이버는 각 제조사 것이라
재배포하지 않는다. 센서를 살 때 같이 받은 것을 쓰고 경로만 알려준다:

```bash
export CAPREAD_DIR=~/capRead_Python-win\&Linux-64bit
```

리눅스에서는 라이브러리만으로 부족하고 CH341 커널 모듈을 직접 빌드해야 한다
(`docs/pi-setup.md` 3장). 센서가 없거나 SDK 가 없으면 `--simple-grasp` 로
촉각 없이 고정 자세 파지를 쓸 수 있다 — 나머지는 전부 정상 동작한다.

**MuJoCo 시뮬레이션 씬.** 파지에 필요 없고 파일이 커서 뺐다.

---

## 라이선스

**Apache License 2.0** — `LICENSE` 참고. 쓰고 고치고 배포해도 된다.
조건은 셋이다: 라이선스 사본을 같이 주고, 바꾼 파일에 바꿨다고 표시하고,
저작권 표시를 지우지 않는다.

upstream 인 [AmazingHand](https://github.com/pollen-robotics/AmazingHand) 가
Apache 2.0 이라 맞췄다. `config/r_hand.toml` 이 거기서 온 파일이고, 출처와
변경 사항은 `NOTICE` 에 적혀 있다.

**벤더 SDK(Tashan capRead, WCH CH341)에는 이 라이선스가 적용되지 않는다.**
저장소에 포함하지 않았고, 각 제조사의 조건을 따른다.
