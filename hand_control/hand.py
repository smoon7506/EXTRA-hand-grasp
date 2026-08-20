# -*- coding: utf-8 -*-
"""여러 손가락(모터 2N개)을 한꺼번에 다루는 래퍼.

tactile_motor_test/motor.py 의 ThumbMotor 를 손가락 N개로 늘린 것이다.
안전 규약은 그대로 가져온다.

--- 절대 바꾸면 안 되는 순서 ---
연결할 때:  현재 위치 읽기 -> 그 값을 goal 로 쓰기 -> 속도 설정 -> 토크 켜기

토크를 먼저 켜면 서보가 goal-position 레지스터에 남아 있던 '옛날 값'으로
그 순간 튄다. 손가락 1개면 놀라고 끝나지만, 10개가 동시에 튀면 링키지끼리
부딪힌다. tactile_motor_test/jog_motor.py:32-39 는 토크를 먼저 켜므로
그 파일을 복사 베이스로 삼지 말 것.
"""

import gc
import math
import time

import hand_config
import kinematics


def _pose(a, s, fingers):
    """hand_config 의 한계값을 채워 kinematics.hand_pose 를 부른다.

    한계값을 매번 7개씩 넘기면 호출부가 지저분해지고, 한 군데서 다른
    값을 넘기는 실수가 생긴다. 설정을 읽는 지점을 여기 하나로 모은다.
    """
    return kinematics.hand_pose(
        a, s, fingers,
        hand_config.FLEX_LIMIT_RAD,
        hand_config.SPREAD_LIMIT_RAD,
        hand_config.MOTOR_MIN_RAD,
        hand_config.MOTOR_MAX_RAD,
    )


class Hand:
    def __init__(self, fingers):
        """fingers: hand_config.load_fingers() 가 돌려준 Finger 목록."""
        self.fingers = fingers
        self._c = None

    def motor_ids(self):
        """이 손이 다루는 모터 ID 를 손가락 순서대로."""
        ids = []
        for finger in self.fingers:
            ids.append(finger.id1)
            ids.append(finger.id2)
        return ids

    def connect(self):
        """모터에 연결하고 '제자리 유지' 상태로 토크를 켠다."""
        # 함수 안에서 import 한다. 하드웨어가 없는 환경에서도 이 모듈을
        # import 만 하는 테스트가 돌 수 있어야 하기 때문이다.
        from rustypot import Scs0009PyController

        self._c = Scs0009PyController(
            serial_port=hand_config.SERIAL_PORT,
            baudrate=hand_config.BAUDRATE,
            timeout=hand_config.SERIAL_TIMEOUT_S,
        )

        # 현재 위치를 못 읽었을 때만 쓸 대체값(편 자세). 미리 계산해 둔다.
        straight = _pose(0.0, 0.0, self.fingers)

        for mid in self.motor_ids():
            # 토크를 켜는 순간 서보는 이미 레지스터에 있는 목표위치로 즉시
            # 움직인다. 목표위치를 모른 채 토크부터 켜면 손으로 잡아 둔
            # 자세에서 낡은 값으로 갑자기 튄다. 그래서 먼저 현재 위치를
            # 읽어 그대로 목표위치로 써서 '제자리 유지'를 만든 뒤에 켠다.
            # 읽기가 실패하면 편 자세로 대체한다 -- 목표를 모르는 채로는
            # 절대 토크를 켜지 않는다.
            try:
                pos = self._c.read_present_position(mid)
                # read_present_position 은 단일 ID 를 넣어도 리스트(Rust
                # Vec 이 파이썬으로 넘어온 것)를 돌려준다.
                if isinstance(pos, (list, tuple)):
                    pos = pos[0]
                goal = float(pos)
            except Exception as e:
                print(f"[WARN] 모터 {mid} 현재 위치 읽기 실패, "
                      f"편 자세로 대체: {e}")
                goal = straight[mid]
            self._c.write_goal_position(mid, goal)
            self._c.write_goal_speed(mid, hand_config.GOAL_SPEED)
            self._c.write_torque_enable(mid, 1)

    def set_pose(self, a, s=0.0, fingers=None):
        """손 자세를 (a, s) 로 바꾸고, 보낸 {ID: 각도} 를 돌려준다.

        fingers 에 목록을 주면 그 손가락 모터에만 쓴다. 나머지 모터에는
        아무것도 보내지 않는다 -- 서보는 마지막 goal position 을 스스로
        유지하므로, '붙잡아두기' 위해 다시 쓸 필요가 없다.
        sequence.PoseSequencer 가 이 성질에 기대고 있다.
        """
        targets = self.fingers if fingers is None else fingers
        pose = _pose(a, s, targets)
        for mid, angle in pose.items():
            # write_goal_position 은 스칼라를 받고 단위는 라디안이다.
            self._c.write_goal_position(mid, angle)
        return pose

    def set_pose_map(self, a_by_finger, s=0.0):
        """손가락별로 다른 a 를 보낸다. -> 보낸 {모터ID: 각도}.

        set_pose 는 손 전체에 a 하나를 쓴다. 파지 제어는 손가락마다
        독립된 힘 루프를 돌리므로 손가락별 a 가 필요하다.

        맵에 없는 손가락에는 아무것도 보내지 않는다 -- 서보는 마지막
        goal position 을 스스로 유지하므로 붙잡아두려고 다시 쓸 필요가
        없다. set_pose 의 fingers 인자와 같은 성질이다.

        sync_write 로 한 번에 보내는 이유: 20Hz 루프에서 손가락마다
        따로 쓰면 사이클당 10번의 버스 왕복이 되어 시리얼이 막힌다.
        """
        known = {f.name: f for f in self.fingers}
        unknown = sorted(set(a_by_finger) - set(known))
        if unknown:
            # 오타 하나로 "손가락이 안 움직이는데 에러도 없는" 상태가
            # 되면 제일 찾기 어렵다. load_fingers 와 같은 방침이다.
            raise ValueError(
                f"이 손에 없는 손가락 이름: {unknown}. "
                f"사용 가능한 이름: {sorted(known)}"
            )

        ids, values = [], []
        seen = {}
        for name, a in a_by_finger.items():
            finger = known[name]
            m1, m2 = kinematics.finger_command(
                a, s, finger,
                hand_config.FLEX_LIMIT_RAD, hand_config.SPREAD_LIMIT_RAD,
                hand_config.MOTOR_MIN_RAD, hand_config.MOTOR_MAX_RAD,
            )
            # kinematics.hand_pose 가 하는 것과 같은 검사다. 여기서는
            # hand_pose 를 거치지 않고 finger_command 를 직접 부르므로
            # 그 검사를 이어받지 못한다 -- 따로 해야 한다. r_hand.toml
            # 오타로 두 손가락이 같은 모터 ID 를 쓰면, 검사 없이는
            # dict(zip(...)) 에서 한쪽 각도가 조용히 덮어써져 "손가락
            # 하나가 안 움직이는데 에러도 없는" 상태가 된다.
            for motor_id in (finger.id1, finger.id2):
                if motor_id in seen:
                    raise ValueError(
                        f"모터 ID {motor_id} 가 중복된다 (손가락 "
                        f"{seen[motor_id]!r}, {finger.name!r}). "
                        f"r_hand.toml 의 id 를 확인할 것."
                    )
                seen[motor_id] = finger.name
            ids.extend((finger.id1, finger.id2))
            values.extend((m1, m2))

        if ids:
            self._c.sync_write_goal_position(ids, values)
        return dict(zip(ids, values))

    def _read_positions(self, ids):
        """{모터ID: 현재 위치(rad)}. 못 읽은 모터는 키가 없다.

        --- 왜 sync_read 를 안 쓰는가 ---
        SCS0009 는 SYNC READ 에 응답하지 않는다. 근거:
          - AmazingHand 원본(get_zeros.rs:106, goto.rs:43)은 위치를
            모터 하나씩 read_present_position 으로 읽는다. sync_read 를
            쓰는 곳이 한 군데도 없다.
          - amazing_hand_gui.py:3377 에 "sync_read_moving timed out;
            falling back to per-servo reads" 가 이미 있다.
          - 실물에서도 매번 "Operation timed out" 이 났다.
        시도했다가 폴백하는 코드가 있었지만, 성공한 적이 없으면서
        SERIAL_TIMEOUT_S(0.5초)만 버리고 경고를 찍어서 지웠다.

        쓰기는 다르다. sync_write_goal_position 은 정상 동작하므로
        set_pose_map 은 여전히 버스 왕복 한 번이다 -- 읽기만 개별이다.
        """
        positions = {}
        for motor_id in ids:
            try:
                value = self._c.read_present_position(motor_id)
                # read_present_position 은 단일 ID 를 넣어도 리스트를
                # 돌려준다 (connect() 가 같은 처리를 한다).
                if isinstance(value, (list, tuple)):
                    value = value[0]
                positions[motor_id] = float(value)
            except Exception:
                # 이 모터만 빠진다. 호출부가 그 손가락을 None 으로 만든다.
                pass
        return positions

    def read_temperatures(self):
        """모터 온도 목록(섭씨). 하나도 못 읽으면 빈 리스트.

        위치 읽기와 같은 이유로 sync_read 를 안 쓴다(_read_positions
        주석 참고). 온도는 초당 한 번만 읽으므로 개별 읽기 10회여도
        루프에 부담이 안 된다.

        예외를 밖으로 던지지 않는다. 온도는 감시용이고, 감시가 실패했다고
        제어 루프가 죽으면 정작 손이 물체를 문 채 방치된다. 대신 빈
        리스트를 돌려줘서 "못 읽었다"를 호출부가 셀 수 있게 한다.
        """
        ids = self.motor_ids()
        temperatures = []
        for motor_id in ids:
            try:
                value = self._c.read_present_temperature(motor_id)
                if isinstance(value, (list, tuple)):
                    value = value[0]
                temperatures.append(float(value))
            except Exception:
                # 이 모터만 건너뛴다. 나머지 온도는 여전히 감시된다.
                pass
        return temperatures

    def read_flex(self):
        """실제로 굽어 있는 각도. -> {손가락이름: flex_rad 또는 None}.

        --- 왜 명령값이 아니라 실측값인가 ---
        강체를 잡으면 명령은 계속 나가는데 실제 관절은 막혀서 멈춘다.
        강성 k = ΔF/Δx 의 x 에 명령값을 쓰면 "많이 밀었는데 힘이 조금
        올랐다"가 되어 강체가 연체로 분류된다. 판별이 거꾸로 된다.

        역변환은 kinematics.py:15 의 규약이다:
            flex = ((m1 - offset1) - (m2 - offset2)) / 2
        차분이라 벌림(spread) 성분은 저절로 상쇄된다.

        예외를 밖으로 던지지 않는 이유: 20Hz 제어 루프가 통째로 죽으면
        손가락이 조인 채로 방치된다. None 을 돌려주면 호출부(grasp)가
        그 손가락만 동결하고 나머지는 계속 간다.
        """
        ids = self.motor_ids()
        by_id = self._read_positions(ids)
        flex = {}
        for finger in self.fingers:
            p1 = by_id.get(finger.id1)
            p2 = by_id.get(finger.id2)
            if p1 is None or p2 is None:
                flex[finger.name] = None
                continue
            # 읽기 호출 자체는 성공해도, 흔들리는 시리얼 버스에서는
            # 값 하나가 깨져서 돌아올 수 있다(호출 실패와는 다른 고장
            # 모드). float() 변환을 개별 손가락 단위로 감싸서, 값 하나가
            # 깨져도 그 손가락만 None 이 되고 나머지는 영향을 안 받는다.
            try:
                flex[finger.name] = (
                    (float(p1) - finger.offset1)
                    - (float(p2) - finger.offset2)
                ) / 2.0
            except (TypeError, ValueError) as e:
                print(f"[WARN] 손가락 {finger.name} 위치값이 깨졌다, "
                      f"이번 사이클은 건너뛴다: {e}")
                flex[finger.name] = None
        return flex

    def disconnect(self):
        """포트만 놓는다. **토크는 끄지 않는다.**

        토크는 서보 안의 레지스터라 포트를 닫아도 안 꺼진다. 그래서 손은
        마지막 goal position(보통 편 자세)을 그대로 유지한다. 손목을 쓰는
        동안 포트를 비켜 주려고 쓰는 것이지, 손을 놓으려는 게 아니다.

        토크까지 끄려면 release() 를 쓴다 -- 그건 종료 경로다.

        gc.collect() 를 부르는 이유: 파이썬 객체를 놓기만 해서는 Rust 쪽
        드롭이 안 돌아 포트가 안 풀린다.
        """
        self._c = None
        gc.collect()

    def release(self):
        """편 자세로 되돌린 뒤 토크를 끈다. 종료 경로 전용.

        주먹 쥔 상태에서 전 손가락을 한 번에 펴면 엄지와 검지가 가는
        도중 부딪힌다 -- 굽힐 때와 같은 경로 문제다. 그래서 hand_config
        의 폄 지연 표대로 묶어서 순서대로 편다.

        여기서는 sequence.PoseSequencer 를 쓰지 않는다. 이 함수는 예외
        안전성이 최우선이고(무슨 일이 있어도 토크는 꺼야 한다) 막혀도
        되는 경로다. 의존성을 늘리는 것보다 표만 빌려 쓰는 쪽이 안전하다.

        펴기와 토크 끄기를 분리된 try 로 감싼다. 펴기가 실패하거나
        sleep 중에 Ctrl+C 가 한 번 더 들어와도 토크 끄기는 반드시
        실행돼야 한다 -- 그러지 않으면 서보가 가열된 채 방치된다.
        KeyboardInterrupt 는 Exception 의 하위가 아니므로 BaseException 을 잡는다.
        """
        if self._c is None:
            return
        try:
            groups = {}
            for finger in self.fingers:
                delay = hand_config.start_delay(finger.name, closing=False)
                groups.setdefault(delay, []).append(finger)
            # 지연은 '시작 시점부터의 절대 시각'이므로 앞 그룹과의 차이만 잔다.
            previous = 0.0
            for delay in sorted(groups):
                if delay > previous:
                    time.sleep(delay - previous)
                    previous = delay
                self.set_pose(0.0, 0.0, groups[delay])
            # 마지막 그룹이 아직 이동 중이다. 다 펴기 전에 토크를 끊으면
            # 손이 중간 자세에서 툭 떨어진다.
            time.sleep(hand_config.RELEASE_SETTLE_S)
        except BaseException as e:
            print(f"[WARN] 손 펴기 실패, 토크 해제는 계속한다: {e}")
        for mid in self.motor_ids():
            try:
                # SCS0009: torque_enable 0 = off, 1 = on.
                self._c.write_torque_enable(mid, 0)
            except BaseException as e:
                # 한 모터가 실패해도 나머지 모터의 토크는 반드시 끈다.
                print(f"[WARN] 모터 {mid} 토크 해제 실패: {e}")


def preview_pose(a, s, fingers):
    """하드웨어 없이 목표 각도만 계산해 사람이 읽을 문자열로 만든다.

    첫 실행 전에 --dry-run 으로 숫자를 눈으로 확인하기 위한 것이다.
    모터를 돌리기 전에 "a=0.2 를 주면 몇 도로 가는가"를 알 수 있으면
    한계 각도를 훨씬 안전하게 찾을 수 있다.

    라디안이 아니라 도(°)로 출력한다. 0.3839 보다 22.0 이 눈에 들어온다.
    """
    pose = _pose(a, s, fingers)
    lines = []
    for finger in fingers:
        d1 = math.degrees(pose[finger.id1])
        d2 = math.degrees(pose[finger.id2])
        lines.append(
            f"  {finger.name:<10} "
            f"m1(id {finger.id1:>2}) = {d1:+7.2f}도   "
            f"m2(id {finger.id2:>2}) = {d2:+7.2f}도"
        )
    return "\n".join(lines)
