# 파이 배포와 제어 가이드

PC 와 라즈베리파이 두 대에 나뉘어 돈다. 이 문서는 **무엇을 어디에 두고, 어떻게 보내고, 어떻게 굴리는지**를 다룬다.

- 파이 준비(OS·드라이버·커널 모듈): `pi-setup.md`
- 브링업 순서와 화면 읽는 법: `pi-run.md`

---

## 1. 누가 무엇을 쓰나

원칙 하나로 갈린다. **하드웨어를 만지면 파이, 화면을 그리면 PC.**

```
파이 (grasp_daemon.py)                     PC (grasp_console.py)
  D405 → 깊이 판정 → 상태기계 → 모터·촉각
         └ 33ms 안에 전부 파이 안에서 끝난다
              │
              │  텔레메트리 + JPEG + ROI마스크   (5001 / 5002)
              ├───────────────────────────────→  화면·촉각 패널
              │                                   버튼/키 입력
              ←───────────────────────────────┤
                 명령 {"cmd": "grasp"}
```

`grasp_daemon.py` 가 실제로 import 하는 것을 따라가면 **23개**다:

| | 파일 |
|---|---|
| `detection/` | `grasp_daemon` `grasp_commands` `grasp_state` `link` `link_sender` `link_watchdog` `orientation` `roi_config` `roi_judge` `wrist_align` |
| `hand_control/` | `hand` `hand_config` `kinematics` `sequence` `servo_bus` `wrist` `tactile` `grasp` `grasp_log` `grasp_runner` `force_control` `stiffness` `spread_seek` |

여기에 **데몬은 안 쓰지만 파이에 있어야 하는 도구**가 붙는다. 하드웨어가 파이에 붙어 있으니 진단도 거기서 해야 한다:

| 파일 | 언제 쓰나 |
|---|---|
| `hand_control/grasp_main.py` | `--sensor-only` 채널 매핑 확인, `--sensor-diag` 전단력(`tf`) 확인, `--dry-run` 상태 전이 |
| `hand_control/main.py` | `a`/`s` 키로 손 직접 제어 |
| `hand_control/calibrate_open.py` | 서보 영점 캘리브레이션 |
| `hand_control/servo_id_tool.py` | 서보 ID 설정 |
| `detection/roi_grasp.py` | 단일 프로세스 기준선(카메라+모터). 데몬/콘솔 쌍과 비교할 잣대 |

**PC 전용**은 `detection/grasp_console.py`, `console_input.py`, `dashboard/` 뿐이다.

---

## 2. 보내는 방법

```bash
./deploy_pi.sh --dry-run            # 무엇이 갈지 먼저 본다
./deploy_pi.sh                      # 보낸다
./deploy_pi.sh wearlab@192.168.0.5  # 다른 주소로
```

**Git Bash 에서 돌린다.** PowerShell 은 파이프를 텍스트로 다뤄 tar 스트림을 망가뜨린다. 그래서 스크립트는 파이프 대신 파일로 만들어 `scp` 한다 — 어느 셸에서든 같게 동작한다.

보낼 목록은 `pi_manifest.txt` 에 있다. 스크립트 안의 플래그로 숨기지 않은 이유는, **git diff 에 "무엇을 파이로 보내는가"의 변화가 보이게** 하려고다.

### `scp -r` 을 직접 쓰지 말 것

`scp -r detection hand_control pi:~/repo/` 는 `logs/`(수 MB, 수백 파일)와 `__pycache__` 까지 보낸다. 2026-08-21 에 실제로 그러다 전송이 실패했다. 게다가 `.pyc` 는 PC 가 3.12, 파이가 3.13 이라 쓰지도 못하면서 "어느 소스가 도는지" 만 헷갈리게 한다.

### 파이에만 있는 것을 덮지 않는다

`tar` 는 덮어쓰기만 하고 삭제하지 않는다. 매니페스트에 없는 것은 손대지 않는다:

| 파이에만 | 왜 |
|---|---|
| `vendor/` (Tashan capRead) | CH341 커널 모듈을 파이에서 빌드했고 `class_ch341.py` 를 aarch64 용으로 고쳤다. 덮으면 날아간다 |
| `venv/` | aarch64 휠 |
| `detection/roi.json`, `hand_mask.npy` | 파이가 자기 카메라로 잡은 캘리브레이션 |

**한 번 겪은 사고:** 작업 트리(`haram_code`)의 `hand_control/` 을 그대로 `scp` 했더니, 이 저장소 판의 멀쩡한 경로 설정이 윈도우 절대경로로 덮여 데몬이 `FileNotFoundError` 로 죽었다. 그래서 **경로는 코드에 박지 않는다** — `hand_config.py` 가 저장소 위치에서 계산하고, 다르면 환경변수로 덮는다:

```bash
export HAND_TOML=/path/to/l_hand.toml         # 다른 손을 쓸 때
export CAPREAD_DIR=~/capRead_Python-...       # 드라이버가 다른 데 있을 때
export HAND_SERIAL_PORT=/dev/ttyUSB1          # 포트가 다를 때
```

---

## 3. 굴리는 순서

### 3-1. 파이: 데몬

```bash
ssh wearlab@<파이주소>
source ~/venv/bin/activate
cd ~/EXTRA-hand-grasp/detection
python grasp_daemon.py
```

옵션:

| 옵션 | 뜻 |
|---|---|
| `--no-hand` | 모터 없이 카메라·판정만 |
| `--no-wrist` | 손목 정렬 없이 |
| `--simple-grasp` | 촉각 없이 고정 자세 파지 |
| `--preview-scale 0.5` | 미리보기를 줄여 보내 대역폭 절약 |

### 3-2. PC: 콘솔

```powershell
cd EXTRA-hand-grasp\detection
python grasp_console.py --host <파이주소>
```

### 3-3. 화면 읽기

```
┌─────────────────────────┬──────────────────┐
│                         │ HAND FORCE       │
│      CAMERA + ROI       │  total  2.41 N v │
│                         │  contact 3/5     │
│   state: HOLDING        │ f1 ███████  1.02 │
│   angle 3°  wrist 12°   │ f4 ██████████1.52│
│                         │ f5 n/a           │
│                         │  (최근 8초 그래프) │
├─────────────────────────┴──────────────────┤
│ [ARM][ALIGN][GRASP][RELEASE][OPEN!] …      │
└────────────────────────────────────────────┘
```

오른쪽 촉각 패널이 갈리는 세 가지:

| 표시 | 뜻 |
|---|---|
| 손가락 막대와 숫자 | 정상 |
| `no force in telemetry` | 파이 데몬이 옛 버전. 배포/재시작이 안 먹었다 |
| `sensor off` | 데몬은 새것인데 센서가 없다(`--simple-grasp`/`--no-hand`) |

**`n/a`** 는 그 손가락 채널이 끊긴 것이다. `0.00` 과 다르다 — 0 은 "안 눌림", `n/a` 는 "못 읽음".

### 3-4. 조작

버튼과 키가 **같은 명령**을 보낸다. 둘 다 살아 있다.

| 키 | 버튼 | 하는 일 |
|---|---|---|
| 드래그 | — | ROI 지정 |
| `n` / `f` | near / far | 지금 깊이로 밴드 끝을 잡는다 |
| `[` `]` `-` `=` | n− n+ f− f+ | 밴드를 조금씩 민다 |
| `t` | target | 지금 각도를 목표로 잡는다 |
| `,` `.` | t< t> | 목표 각도 미세조정 |
| `w` `W` | <w w> | 손목 조그 |
| `h` | mask | 지금 밴드를 '손'으로 저장 (물체를 치우고) |
| `a` | ALIGN | 손목 정렬 토글 |
| `m` | ARM | 무장 토글 |
| **`g`** | **GRASP** | **ROI 판정을 안 기다리고 지금 잡는다** |
| `r` | RELEASE | 놓는다 |
| `space` | OPEN! | 비상 폄 |
| `q` | — | 종료 |

`g`(수동 파지)는 브링업에서 가장 많이 쓴다. 카메라 조건을 매번 맞추지 않고 파지 로직만 반복 시험할 수 있다.

---

## 4. 파지가 언제 끝나나

**스스로는 안 끝난다.** `HOLDING` 에 종료 조건이 없다. 끝나는 길은 다섯이고 전부 외부 요인이다:

1. `r` / RELEASE
2. `space` / OPEN!
3. 모터 온도 `TEMP_LIMIT_C` 초과 (1초마다 확인)
4. 온도 읽기 연속 실패 — 과열을 감시할 수 없으므로 중단
5. 링크 끊김 — ping 이 2초 없으면 무장 해제, `HOLDING` 이면 30초 뒤 자동 폄

3·4 는 물체가 아니라 **모터를 보호**하는 것이다. 파지의 성공/실패는 아직 아무도 판정하지 않는다 — 물체를 놓쳐도 상태기계는 `HOLDING` 에 남는다. 그래서 화면의 `contact n/5` 를 사람이 봐야 한다.

---

## 5. 처음 돌릴 때 순서

한 번에 다 켜지 말고 아래 순서로 하나씩 늘린다. 무엇이 틀렸는지 가려내려면 이 순서여야 한다.

1. **센서 매핑** — 파이에서 `python hand_control/grasp_main.py --sensor-only`, 손가락을 하나씩 눌러 이름이 맞는지 본다. 틀리면 검지 힘을 보고 새끼를 조인다
2. **전단력** — `--sensor-diag r_finger4`, 물체를 옆으로 밀며 `tf` 가 오르는지 본다
3. **상태 전이** — `--dry-run` (하드웨어 없이 전체 사이클)
4. **손가락 하나** — `hand_config.ACTIVE_FINGERS` 를 하나로 줄이고 실물
5. **데몬 + 콘솔** — `--no-hand` 로 카메라·판정만 먼저, 그다음 모터

---

## 6. 안 될 때

| 증상 | 원인 |
|---|---|
| `FileNotFoundError: ...r_hand.toml` | 경로가 박힌 옛 `hand_config.py`. 다시 배포한다 |
| 모든 모터가 `Parsing error` | COM 포트가 아니라 **서보 전원**부터 확인 |
| 패널이 `no force in telemetry` | 데몬이 옛 버전. 배포 후 데몬을 **재시작**했는지 |
| 촉각이 `未找到库文件` | CH341 라이브러리. `pi-setup.md` 3-3 |
| `ImportError` (파이) | 새 모듈이 매니페스트에 안 걸렸다. `--dry-run` 으로 확인 |
| 버튼이 안 먹음 | 콘솔이 옛 버전. PC 쪽 재시작 |
