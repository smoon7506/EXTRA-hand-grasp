# PC / 라즈베리파이 2노드 분리 설계 (ROI 파지 시스템)

날짜: 2026-08-18
대상: `detection/roi_grasp.py` (분할), `detection/roi_config.py` ·
      `detection/roi_judge.py` · `detection/grasp_state.py` ·
      `detection/grasp_daemon.py` · `detection/grasp_console.py` ·
      `detection/link.py` (신규), `hand_control/hand_config.py` (수정)

선행 설계:
- `2026-08-04-roi-depth-grasp-trigger-design.md` — ROI 깊이 밴드 판정
- `2026-08-14-roi-align-grasp-design.md` — 손목 정렬 + 촉각 힘 제어 통합

## 목표

지금 `detection/roi_grasp.py` 는 1054줄 한 파일에서 전부를 한다 — D405
파이프라인, 깊이 판정, 상태기계, 서보 버스 시분할, 촉각 센서, cv2 창과
키 입력. 한 대의 PC 위에서 한 프로세스로 돈다.

이것을 두 대로 나눈다.

- **라즈베리파이 5** — D405, 손(SCS0009 ×10), 손목(STS3215), 촉각 센서를
  물고 **반사 루프**를 돈다. 판정과 구동이 전부 여기 있다.
- **메인 PC** — 화면, ROI 드래그, 키 입력. **판단 루프**를 담당한다.
  나중에 YOLO 를 붙일 자리도 여기다.

## 왜 이렇게 나누는가

### 나누는 축은 "모터 vs 연산"이 아니라 "빠른 루프 vs 느린 루프"다

처음 구상은 ESP32 가 모터를 구동하고 PC 가 비전을 처리하는 3노드였다.
숫자를 재 보면 그 축이 아니다.

| 작업 | 프레임당 비용 |
|---|---|
| D405 캡처 + `rs.align(color)` (픽셀별 재투영) | 지배적 |
| `band_ratio` — ROI 서브배열 numpy 임계 | < 0.5 ms |
| `principal_axis` — connectedComponents + 2×2 eigh | ~ 1 ms |
| 서보 sync_write — 10 모터 40 바이트 @ 1 Mbaud | ~ 0.4 ms |

모터 구동은 이미 공짜다. ESP32 로 넘겨도 파이가 아끼는 연산이 없고,
대신 시리얼 홉 하나와 유지할 프로토콜 하나가 는다. 그리고 `hand.py` 의
안전 순서(위치읽기 → goal → speed → torque), `kinematics.py`,
`servo_bus.ServoLease`, `wrist.py` 를 C++ 로 재구현해야 하며,
`test_grasp_states.py` · `test_hand_sync.py` · `test_grasp_config.py` 가
검증하던 로직이 펌웨어로 넘어가 테스트 밖으로 나간다.

**그래서 ESP32 는 이번 구성에서 뺀다.** 파이가 URT-1(USB-TTL)로 서보
버스를, CH341(USB)로 촉각 센서를 직접 잡는다. 하드 리얼타임이 필요해지는
날이 오면 그때 파이 아래에 넣는다.

### 판정 루프가 네트워크를 건너면 안 된다

ALIGNING 은 30 Hz 시각 서보 루프다(`wrist_align.WristAligner`, 오차 1도당
`ALIGN_GAIN=0.3` 만큼 손목을 돌린다). 카메라와 손목 사이에 WiFi 를 넣으면
지터 20~100 ms 가 루프 안으로 들어와 이 게인 튜닝이 무의미해진다.

그리고 현재 판정은 YOLO 를 전혀 쓰지 않는다 — 깊이 임계 + PCA 뿐이라
2 ms 도 안 걸린다. 이 2 ms 를 PC 에서 하려고 프레임을 보내면 인코딩 +
네트워크 + 디코딩 10~30 ms 를 대신 낸다. 깊이는 손실 압축을 못 써서
640×480×16bit×30 = 18 MB/s 다.

**따라서 판정과 구동은 전부 파이에 둔다.** PC 의 GPU 는 나중에 YOLO 처럼
진짜 무거운 것에 쓴다.

### 깊이 배열은 네트워크로 보내지 않는다

`draw_overlay` 가 `depth_m` 을 쓰는 유일한 곳은 ROI 안 초록 틴트 마스크를
다시 만드는 부분이다(`roi_grasp.py:648-651`). 나머지는 전부 텍스트다.

그 마스크는 파이가 이미 루프 안에서 만든다. ROI 크기 이진 PNG 로 보내면
150×150 기준 약 500 바이트다. 640×480 float32 깊이 배열 1.2 MB 를 보낼
이유가 없다. **이 사실이 이 설계가 성립하는 근거다.**

## 파일 경계

`roi_grasp.py` 1054줄을 여섯 조각으로 나눈다. 1054줄은 이미 혼자 너무 많은
일을 하고 있어서, 쪼개는 김에 파일 경계도 정리한다.

### 파이에서 도는 것

| 파일 | 옮겨오는 것 | 근거 |
|---|---|---|
| `detection/roi_config.py` | `RoiConfig`, `nudge_band`, `nudge_target`, `_save_band` | ROI·밴드·목표각의 진실의 출처. 판정하는 쪽이 소유해야 PC 와 어긋나지 않는다 |
| `detection/roi_judge.py` | `band_ratio`, `roi_median`, `RatioTrigger`, `_axis_segment` | 깊이 배열이 있어야 도는 것들 |
| `detection/grasp_state.py` | `GraspStateMachine`, `SequenceExecutor`, 상태 상수 | 하드웨어를 모르는 순수 로직. 지금도 그렇게 짜여 있다 |
| `detection/grasp_daemon.py` | `main()` 의 카메라·리스·하드웨어 부분, `_wrist_lease_fns`, `_hand_lease_fns` | cv2 창 없이 도는 데몬. 여기만 하드웨어를 안다 |

### PC 에서 도는 것

| 파일 | 옮겨오는 것 |
|---|---|
| `detection/grasp_console.py` | `draw_overlay`, `RoiDragger`, `_HINT`, 키 → 명령 매핑, TCP 클라이언트 |

### 양쪽이 공유

| 파일 | 내용 |
|---|---|
| `detection/link.py` | 메시지 스키마 + 인코딩/디코딩. 파이에도 레포 전체를 두므로 한 파일을 양쪽이 그대로 import |

### 건드리지 않는 것

`detection/orientation.py`, `detection/wrist_align.py`, 그리고 `hand_control/`
전부 — `hand.py` · `wrist.py` · `servo_bus.py` · `tactile.py` ·
`grasp_runner.py` · `kinematics.py` · `sequence.py`.

`hand_config.py` 만 한 곳 바뀐다: `SERIAL_PORT = "COM11"` 을 환경변수로
갈라 파이에서 `/dev/ttyUSB0` 을 쓰게 한다.

### 설정 파일의 소유권

`roi.json` 과 `hand_mask.npy` 는 **파이가 소유한다.** 판정하는 쪽이 가져야
PC 와 의견이 갈릴 여지가 없다.

다만 값이 바뀔 때마다 PC 가 텔레메트리로 받아 `detection/backup/roi.json`
으로 자동 백업한다. 파이 SD 카드가 날아가도 캘리브레이션을 잃지 않는다.

## 와이어 계약

### 연결 모델

파이가 서버, PC 가 클라이언트다. 파이는 헤드리스로 늘 떠 있고 콘솔은
붙었다 떨어졌다 한다.

제어 소켓은 **한 번에 하나만** 받되, 새 연결이 오면 기존 것을 끊고 새 것을
받는다. 거절하면 콘솔이 비정상 종료했을 때 좀비 연결 때문에 다시 못 붙는다.

포트는 두 개다. 나누는 이유는 드롭 정책이 다르기 때문이다 — 명령은 하나도
잃으면 안 되고, 미리보기 프레임은 얼마든지 버려도 된다.

- **5001 제어** — 양방향, 줄단위 JSON (UTF-8, `\n` 구분)
- **5002 미리보기** — 파이 → PC 단방향, 바이너리

### 명령 (PC → 파이)

| 명령 | 필드 | 키 | 비고 |
|---|---|---|---|
| `hello` | `proto` | 연결 시 | 버전 불일치면 파이가 거절 |
| `ping` | `seq` | 30 Hz | 워치독 먹이 |
| `arm` / `disarm` | — | — | **신규 개념**. 아래 참고 |
| `set_roi` | `x,y,w,h` | 드래그 | **전체 해상도** 픽셀 |
| `calib_band` | `edge:"near"\|"far"` | `n`/`f` | 파이가 자기 median 으로 |
| `nudge_band` | `edge`, `delta_m` | `[ ] - =` | 상대량 |
| `capture_target` | — | `t` | 파이가 자기 angle 로 |
| `nudge_target` | `delta_deg` | `,` `.` | 상대량 |
| `save_hand_mask` | — | `h` | 파이가 자기 band 로 |
| `set_align` | `on` | `a` | |
| `jog_wrist` | `dir:±1` | `w`/`W` | |
| `release` | — | `r` | HOLDING 일 때만 |
| `emergency_open` | — | `space` | 어느 상태에서든 |
| `bye` | — | `q` | 의도적 종료. 링크 끊김과 구분된다 |

파이는 각 명령에 응답한다.

```json
{"t":"ack","cmd":"capture_target","ok":false,
 "msg":"각도를 못 재고 있습니다 (too few pixels)"}
```

지금 `print()` 로 나가던 거절 메시지들(`roi_grasp.py:990`, `1013`, `1026`)이
전부 여기로 와야 한다. 안 그러면 사용자가 키를 눌렀는데 아무 일도 안
일어나고 이유도 모른다.

### PC 는 "값"이 아니라 "동작"을 보낸다

이것이 계약의 핵심 원칙이고, 지키지 않으면 조용히 틀린다.

`n` 키는 지금 **그 프레임의** median 으로 near 를 정한다(`roi_grasp.py:1010`).
나이브하게 쪼개면 PC 가 텔레메트리로 받은 median 을 써서 값을 계산해
보내는데, 그 median 은 이미 100 ms 낡았다. 캘리브레이션이 낡은 값으로
되면서 조용히 틀린다.

| 키 | 틀린 방식 | 올바른 방식 |
|---|---|---|
| `n`/`f` | `{"set_band":{"near_m":0.183}}` | `{"cmd":"calib_band","edge":"near"}` |
| `t` | `{"set_target_angle":8.2}` | `{"cmd":"capture_target"}` |
| `h` | 마스크 배열을 올려보냄 | `{"cmd":"save_hand_mask"}` |

이렇게 하면 이 세 키는 분리 전과 **완전히 동일하게** 동작한다. 반면
`[ ] - =` 와 `,` `.` 는 원래 "현재 값에서 얼마만큼"이라 상대량을 보내면
그대로 맞다.

### 텔레메트리 (파이 → PC, 매 프레임)

```json
{"t":"tel","seq":1234,
 "state":"ALIGNING","ratio":0.41,"valid":0.92,"median":0.183,
 "angle":8.2,"angle_reason":null,"axis":[[12.0,80.0],[138.0,64.0]],
 "wrist_deg":-3.1,"align_status":"aligning","align_on":true,
 "armed":true,"confirm_left":0.0,"rearm_left":0.0,"release_in":null,
 "has_hand_mask":true,"bus_owner":"wrist",
 "roi":{"x":220,"y":150,"w":150,"h":150,
        "near_m":0.19,"far_m":0.14,"target_angle_deg":8.0},
 "cfg":{"enter_ratio":0.30,"exit_ratio":0.15,"min_valid_ratio":0.30}}
```

`roi` 와 `cfg` 를 **매 프레임 통째로** 싣는다. 그러면 콘솔이 상태를 하나도
안 들고 있어도 되고(스테이트리스), PC 와 파이가 ROI 에 대해 의견이 갈릴
여지가 사라진다. 400 바이트 × 30 Hz = 96 kbps 로 무시할 수준이다.

`cfg` 를 싣는 것은 덤으로 하나를 고친다. `draw_overlay` 가 지금
`ENTER_RATIO`/`EXIT_RATIO` 상수를 직접 읽는데(`roi_grasp.py:673`), 이제
화면이 **파이가 실제로 쓰는 값**을 보여준다. 두 곳의 상수가 어긋날 여지가
없어진다.

`axis` 는 ROI 안 좌표계 그대로다(지금 `draw_overlay` 가 그렇게 받는다).

### 미리보기 프레임 (파이 → PC)

```
[4B 전체길이 BE][JSON 헤더 + \n][JPEG 바이트][마스크 PNG 바이트]

헤더: {"seq":1234,"src_w":640,"src_h":480,
       "jpeg_len":31200,"mask_len":480,"roi":[220,150,150,150]}
```

- `seq` 가 텔레메트리의 `seq` 와 같다. PC 가 그림과 숫자의 짝을 맞춘다.
  짝을 안 맞추면 화면의 각도 숫자와 그림이 서로 다른 프레임이 된다
- `src_w`/`src_h` 는 **원본** 해상도다. 미리보기를 축소해 보낼 때 PC 가
  드래그 좌표를 원본으로 환산해야 하는데, 이게 빠지면 ROI 가 조용히
  엉뚱한 곳에 잡힌다
- 마스크는 ROI 크기 이진 PNG. `draw_overlay` 의 초록 틴트를 되살린다
- 기본 640×480 / 15 fps / JPEG q70 → 약 3.6 Mbps.
  `--preview-fps` 와 `--preview-scale` 로 낮출 수 있다

### 드롭 정책

**제어 루프는 어떤 경우에도 소켓 때문에 블록되지 않는다.**

미리보기는 전용 스레드 + 1칸 큐(최신 프레임이 이김)로 보내고, 텔레메트리도
논블로킹으로 보내다 버퍼가 차면 그 프레임은 버린다.

WiFi 가 한 번 딸꾹질할 때 30 Hz 판정 루프가 같이 멈추면 손목이 정렬 도중에
굳거나 파지가 늦는다. 이것이 WiFi 환경에서 가장 중요한 규칙이다.

## 링크가 끊겼을 때

### 끊김을 어떻게 아는가

TCP 는 WiFi 가 조용히 죽으면 몇 분간 눈치를 못 챈다. 그래서 애플리케이션
레벨 하트비트를 쓴다. PC 가 매 프레임 `ping` 을 보내고, 파이는 마지막 수신
시각을 기록한다. **`LINK_TIMEOUT_S = 2.0`** 을 넘으면 링크 끊김으로 본다
(30 Hz 기준 60 번 연속 유실이라 오탐 여지가 거의 없다).

### 상태별 조치

원칙은 하나다 — *사람이 안 보고 있으면 새로 시작하지 않는다. 하지만 진행
중인 동작을 중간에 얼리지도 않는다.*

| 상태 | 조치 | 이유 |
|---|---|---|
| ARMED | 무장 해제 — 판정 정지 | 화면을 아무도 안 보는데 손이 알아서 잡으면 안 된다 |
| ALIGNING | 즉시 물러나 무장 해제 | 손목이 도는 중이다. 감시자가 없으면 멈춘다 |
| CONFIRMING | 같음 | |
| GRASPING | 끝까지 진행 → HOLDING | 중간에 멈추면 손가락이 반쯤 닫힌 채 물체에 끼인다 |
| HOLDING | **`HOLD_RELEASE_S = 30.0` 후 자동 폄** | 아래 참고 |
| RELEASING | 끝까지 진행 | 펴는 것은 항상 끝까지 |

### HOLDING 자동 폄

30 초를 고른 근거: WiFi 재연결은 DHCP 까지 포함해 보통 5~15 초에 끝난다.
30 초면 일시적 딸꾹질은 전부 살아남고 진짜 끊긴 경우만 놓는다.

- **링크가 돌아오면 타이머를 취소한다.** 링크 복구 = 사람이 돌아옴이다
- 남은 시간을 텔레메트리 `release_in` 에 실어 PC 가 "23초 뒤 자동 폄"을
  화면에 띄운다
- 자동 폄 후에는 **무장 해제 상태로 남는다**

과열 보호는 이 정책과 별개로 이미 살아 있다. `grasp_runner.py:152-172` 의
`ForceGraspRunner` 가 1초마다 모터 온도를 읽고, 한계를 넘거나 읽기가 연속
실패하면 `"abort"` 를 내며, `GraspStateMachine` 이 자동으로 RELEASING 으로
간다. 전부 파이 안에 있어 링크가 죽어도 계속 돈다.

### 무장은 언제나 사람이 건다

**데몬은 무장 해제 상태로 기동한다.** 부팅하자마자, 혹은 systemd 가 재시작
하자마자 손이 알아서 잡기 시작하면 안 된다. PC 가 붙어 `arm` 을 보내야
판정이 시작된다.

링크가 돌아왔을 때도 마찬가지다. 파이는 무장 해제 상태로 텔레메트리만
올리고, PC 가 명시적으로 `arm` 을 보내야 다시 판정한다 — 사람이 화면을 다시
보고 있다는 확인이다.

`bye`(콘솔에서 `q`)는 **링크 끊김과 같은 조치를 2초 대기 없이 즉시** 적용
한다. 사람이 의도적으로 나간 것이므로 워치독이 눈치채기를 기다릴 이유가
없다. HOLDING 이었다면 자동 폄 타이머도 그 자리에서 시작한다.

이 때문에 `GraspStateMachine` 에 `armed` 플래그가 하나 붙는다. ARMED 상태
에서 그 플래그가 꺼져 있으면 `trigger.update()` 를 건너뛴다. 이미 쿨다운
(`_rearm_at`)이 똑같은 자리에서 똑같은 일을 하고 있어서 자연스럽게 들어간다.

### 비상 폄의 지연

WiFi 왕복 5~30 ms 는 사람 반응시간(200~300 ms)에 비하면 작다. 진짜 위험은
지연이 아니라 링크가 죽어 `space` 가 아예 안 가는 경우이고, 그건 위 워치독이
처리한다.

**향후 과제(이번 범위 밖):** 파이 GPIO 에 물리 비상정지 버튼을 달아
네트워크와 무관하게 `emergency_open` 을 부른다. 자리만 비워 둔다.

### 데몬 종료

지금 `finally` 블록(`roi_grasp.py:1041-1053`)이 하는 순서 — 손목 토크 해제 →
리스 반납 → 손 편 자세 + 토크 오프 → 센서 정지 — 를 `SIGTERM` 과 `SIGINT`
양쪽에서 그대로 실행한다.

헤드리스라 Ctrl+C 대신 systemd 가 SIGTERM 을 보낼 수 있다. 이게 없으면
모터가 토크 걸린 채 방치된다.

## 무엇이 동일하고 무엇이 달라지는가

이 설계가 답해야 할 질문은 "쪼개면 동일하게 동작하는가"다.

**파일 6개로 쪼개기 — 완전히 동일하다.** import 위치만 옮기는 기계적
작업이고 로직은 한 줄도 안 바뀐다.

**두 머신으로 나누기 — 네 가지가 달라진다.**

1. **판정과 구동은 그대로다.** ratio 계산 → 상태 전이 → 손목/손 명령이 전부
   파이 안에서 지금과 똑같은 순서로 돈다. 네트워크가 이 루프 안에 안 들어간다
2. **사람 입력에 왕복 지연 5~30 ms 가 붙는다.** 튜닝 키는 문제없고, 비상 폄은
   위에서 다뤘다
3. **`n`/`f`/`t`/`h` 는 "동작을 보낸다" 원칙으로 동일성이 유지된다.** 이 원칙을
   어기면 조용히 틀린다
4. **화면이 100 ms 늦다.** 판정은 안 바뀌지만 사람이 보는 그림은 과거다.
   `seq` 짝맞춤으로 최소한 그림과 숫자는 일치시킨다

## 테스트

### 기존 테스트가 그물이다

`tests/test_roi_grasp.py` 793줄 + `tests/test_wrist_align.py` 261줄 +
`tests/test_orientation.py` 176줄이 **모든 단계에서 계속 통과해야 한다.**

`test_roi_grasp.py` 가 검증하는 것이 정확히 파이로 옮겨갈 순수 로직
(`band_ratio`, `RatioTrigger`, `RoiConfig`, `GraspStateMachine`,
`SequenceExecutor`)이다. import 경로만 바꿔서 통과하면 분할의 동일성이
증명된 것이다.

### 새로 필요한 테스트

전부 하드웨어 없이 돈다.

1. **`link.py` 왕복** — 모든 메시지가 encode → decode 후 동일한가.
   그리고 **한 바이트씩 먹여도** 프레임이 제대로 나오는가. TCP 는 메시지
   경계를 안 지켜서 실제 버그가 여기서 난다
2. **명령 디스패치** — 각 명령이 올바른 메서드를 부르는가, 거절 시
   `ack.ok=false` 와 이유가 오는가
3. **`calib_band` 가 파이의 최신 median 을 쓰는가** — 낡은 값을 안 쓴다는
   것을 못 박는 테스트
4. **워치독** — 가짜 시계로 2초 경과 시 상태별 조치가 표대로인가,
   HOLDING 30초 자동 폄, 링크 복구 시 타이머 취소
5. **`armed` 플래그** — 무장 해제 상태에서 ratio 가 아무리 높아도
   ALIGNING 으로 안 가는가
6. **미리보기 드롭** — 가짜 소켓이 `BlockingIOError` 를 던져도 제어 루프가
   안 멈추는가
7. **좌표 환산** — `preview_scale=0.5` 일 때 드래그 좌표가 원본 해상도로
   올바로 환산되는가

구현은 TDD 로 간다. 특히 `link.py` 와 워치독은 테스트를 먼저 쓰기 좋은
모양이다.

## 구현 순서

각 단계가 끝날 때마다 되돌아갈 수 있는 지점이다.

| 단계 | 내용 | 검증 |
|---|---|---|
| **0. 순수 분할** | `roi_grasp.py` → `roi_config`/`roi_judge`/`grasp_state`. `roi_grasp.py` 는 이 셋을 import 해 그대로 돈다 | 793줄 통과 + 실물 동작이 지금과 같음 |
| **1. 프로토콜** | `link.py`, `arm`/`disarm`, 워치독. 아직 프로세스 하나 | 새 테스트 1~5 |
| **2. 두 프로세스** | 데몬/콘솔 분리, 같은 PC 의 `127.0.0.1`. 하드웨어는 전부 제자리 | 동작 동일성 + 테스트 6~7 |
| **3. 파이 이사** | rustypot aarch64, CH341 경로, 포트 환경변수 | 파이에서 실물 |
| **4. WiFi 튜닝** | 미리보기 프레임률, 워치독 값 실측 | — |

### 왜 2단계가 같은 PC 인가

D405 도 손도 지금 PC 에 붙어 있다. 먼저 같은 PC 안에서 `127.0.0.1` 로
데몬과 콘솔을 띄워 동작이 지금과 같은지 확인하고, **그 다음에** 데몬만
파이로 옮긴다.

그러면 실물에서 문제가 생겼을 때 원인이 갈린다. 2단계에서 깨지면 100 %
쪼갠 탓이고, 3단계에서 깨지면 100 % 파이 환경 탓이다.

0~2단계가 코드 작업이고, 3~4단계는 파이 환경 구축과 실물 브링업이다. 구현
계획은 0~2단계를 먼저 다루고, 3단계는 위 "파이 환경" 절의 위험이 해소된
뒤에 별도로 계획한다.

### 왜 `roi_grasp.py` 를 안 지우는가

0단계에서 `roi_grasp.py` 를 남겨 두면 언제든 "쪼개기 전"과 비교할 수 있는
기준선이 된다. 2단계에서 데몬이 이상하면 바로 이걸 돌려 파이 탓인지 분할
탓인지 가른다. 4단계가 끝나면 그때 지운다.

## 파이 환경 (2026-08-18 실측)

```
Raspberry Pi 5 Model B Rev 1.1 / aarch64 / RAM 7.9 GiB
Python 3.13.5          (Raspberry Pi OS Trixie = Debian 13 계열)
cargo                  없음
pyrealsense2 / rustypot / cv2   전부 없음
USB3 root hub 2개      (D405 는 아직 PC 에 있다)
```

### 의존성 (2026-08-18 실측 갱신)

Python 3.13 은 아직 새 버전이라 휠이 없는 패키지가 많아 이식의 최대 위험으로
잡아 두었다. 실측 결과 절반이 해소됐다.

- **`rustypot` — 해결됐다.** `pip install rustypot` 이 그대로 붙는다
  (`~/venv/lib/python3.13/site-packages/rustypot/`). PyO3 abi3 휠이라 3.13
  aarch64 에서 문제없다. Rust 툴체인도 필요 없다. **이 프로젝트에서 가장 컸던
  미해결 위험이 사라졌다 — 손과 손목을 돌릴 길이 확보됐다.**
- **`pyrealsense2` — 소스 빌드가 필요할 가능성이 높다.** 3.13 aarch64 휠은
  거의 없다. librealsense 를 아래 옵션으로 빌드한다:

  ```
  cmake .. -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_EXAMPLES=false -DBUILD_GRAPHICAL_EXAMPLES=false \
    -DBUILD_PYTHON_BINDINGS=true \
    -DPYTHON_EXECUTABLE=$(which python) \
    -DFORCE_RSUSB_BACKEND=true
  ```

  `FORCE_RSUSB_BACKEND` 가 핵심이다. 없으면 커널 패치를 요구하는데, 라즈베리파이
  커널에 그 패치를 넣으면 커널 업데이트마다 깨진다. USB 백엔드면 커널을 안
  건드린다.
- **`cv2` / `numpy` — apt 로 해결한다.** `python3-opencv` / `python3-numpy` 를
  깔고 venv 를 `--system-site-packages` 로 만든다. 파이에서 opencv 를 소스
  빌드하면 한 시간씩 걸린다.

Trixie 는 PEP 668 을 강제해서 시스템 Python 에 `pip install` 이 거부된다.
파이에서는 venv 를 쓴다.

### 그 밖에 3단계에서 확인할 것

- **CH341 촉각 드라이버** — `lib/ch341/CH341PAR_LINUX/lib/aarch64/dynamic/libch347.so`
  가 이미 레포에 들어 있다. `class_ch341.py:69` 가 `x64` 경로를 하드코딩
  하고 있으므로 아키텍처로 갈라야 한다. 이 항목은 위험이 낮다
- **`cv2`** — Trixie 의 `python3-opencv` 로 해결 가능. 위험이 낮다
- **`rs.align` 비용** — D405 는 컬러가 좌측 depth 이미저에서 나오는 구조라
  depth↔color 외부 파라미터가 거의 항등에 가깝다. 그래서 D435i 만큼 비싸지
  않을 가능성이 높지만 **추정이다.** 비싸면 레버가 있다 — 판정은 어차피 ROI
  안에서만 하므로 전체 프레임을 align 할 필요가 없다
- **D405 USB3 전원** — 700 mA 가까이 쓴다. 파이 5 포트 직결 시 프레임 드롭
  이나 재열거로 나타날 수 있다. 셀프파워 허브가 필요한지

## 범위 밖

- **YOLO / GGCNN** — 이번 라운드에서 PC 의 판단 근거는 사람의 키보드다.
  나중에 붙일 때는 같은 명령 통로(`set_roi`, `capture_target` 대신
  `set_target_angle`)에 꽂으면 된다. 계약이 이미 그 모양이다
- **ESP32** — 위에서 논한 이유로 뺀다
- **물리 비상정지 버튼** — 설계에만 자리를 비워 둔다
- **파이 단독 자율 운전** — PC 없이 파이가 알아서 잡는 모드는 만들지 않는다.
  무장은 항상 사람이 건다
