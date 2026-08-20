# -*- coding: utf-8 -*-
"""STS3215 손목. 하드웨어를 아는 유일한 손목 코드.

손과 같은 COM11 을 쓴다. 동시에 열 수 없으므로 servo_bus.ServoLease 가
번갈아 잡아 준다.

--- rustypot 의 함정 두 가지 ---
  read_* 는 단일 ID 를 넣어도 리스트를 돌려준다
  ping 은 False 를 주기도 하고 RuntimeError 를 던지기도 한다

--- STS3215 는 SCS0009 와 다르다 ---
write_torque_enable 이 bool 만 받는다. SCS0009 는 0/1 을 받는다.
레지스터 주소와 바이트 순서도 달라서 컨트롤러를 섞어 쓰면 안 된다.
"""

import gc


def _one(value):
    """rustypot 의 리스트 반환을 벗긴다."""
    if isinstance(value, (list, tuple)):
        return value[0]
    return value


class Sts3215Wrist:
    def __init__(self, servo_id, port, baudrate, timeout_s, *,
                 speed, min_rad, max_rad, factory=None):
        """factory: 컨트롤러를 만드는 콜러블. 테스트가 가짜를 넣는다.

        기본값이 None 인 이유: rustypot 을 모듈 최상단에서 import 하면
        하드웨어가 없는 환경에서 이 파일을 import 만 하는 테스트가 죽는다
        (hand.py 와 같은 이유). connect() 안에서 늦게 import 한다.
        """
        self.id = servo_id
        self.port = port
        self.baudrate = baudrate
        self.timeout_s = timeout_s
        self.speed = speed
        self.min_rad = min_rad
        self.max_rad = max_rad
        self._factory = factory
        self._c = None

    def _make_controller(self):
        if self._factory is not None:
            return self._factory(serial_port=self.port,
                                 baudrate=self.baudrate,
                                 timeout=self.timeout_s)
        from rustypot import Sts3215PyController
        return Sts3215PyController(serial_port=self.port,
                                   baudrate=self.baudrate,
                                   timeout=self.timeout_s)

    def _alive(self):
        """이 ID 가 응답하나. 어떤 식으로 실패하든 False 로 뭉갠다."""
        try:
            return bool(self._c.ping(self.id))
        except Exception:
            return False

    def connect(self):
        """연결하고 '제자리 유지' 상태로 토크를 켠다.

        순서는 손과 동일하다: 현재 위치 읽기 -> 그 값을 goal 로 쓰기 ->
        속도 -> 토크. 토크를 먼저 켜면 레지스터에 남은 옛 goal 로 손목이
        튀는데, 손목 위에는 카메라가 실려 있다.
        """
        self._c = self._make_controller()

        if not self._alive():
            self._c = None
            raise RuntimeError(
                f"손목 서보 ID {self.id} 가 응답하지 않습니다. "
                f"서보 전원부터 확인하세요 -- 전원이 없으면 COM 포트는 "
                f"정상으로 보이면서 통신만 실패합니다. ID 가 손(1~10)과 "
                f"겹쳐도 둘 다 응답하지 않습니다."
            )

        # !!! 위치 모드인지 반드시 확인한다 !!!
        # mode=1 은 연속 회전(wheel) 모드다. 그 상태에서 goal 을 쓰면
        # **속도 명령**으로 해석돼 손목이 멈추지 않고 돌아간다. EEPROM 이라
        # 전원을 꺼도 남으므로, 예전에 wheel 모드로 쓴 개체면 조용히 이
        # 사고가 난다.
        mode = int(_one(self._c.read_mode(self.id)))
        if mode != 0:
            self._c = None
            raise RuntimeError(
                f"손목 서보 ID {self.id} 가 mode={mode} 입니다. 위치 모드"
                f"(mode=0)가 아니면 goal 이 속도 명령으로 해석돼 손목이 "
                f"멈추지 않고 돕니다. EEPROM 이라 전원을 꺼도 유지되므로 "
                f"lock 을 풀고 mode=0 으로 되돌린 뒤 다시 실행하세요."
            )

        present = float(_one(self._c.read_present_position(self.id)))
        self._c.write_goal_position(self.id, present)
        self._c.write_goal_speed(self.id, self.speed)
        # SCS0009 는 0/1, STS3215 는 bool 이다.
        self._c.write_torque_enable(self.id, True)

    def write_goal(self, rad):
        """목표 각도를 보낸다. -> 실제로 보낸(클램프된) 값."""
        if self._c is None:
            raise RuntimeError(
                "손목이 연결돼 있지 않습니다. ServoLease 로 포트를 먼저 "
                "잡아야 합니다 -- 조용히 성공하면 '왜 안 도는지'를 못 찾습니다."
            )
        clamped = max(self.min_rad, min(self.max_rad, float(rad)))
        self._c.write_goal_position(self.id, clamped)
        return clamped

    def read_position(self):
        """현재 각도(rad). 못 읽으면 None.

        예외를 밖으로 던지지 않는다. 읽기 하나가 실패해서 루프가 죽으면
        손목이 중간 자세에 방치된다 (hand.read_flex 와 같은 방침).
        """
        if self._c is None:
            return None
        try:
            return float(_one(self._c.read_present_position(self.id)))
        except Exception:
            return None

    def disconnect(self, torque_off=False):
        """포트를 놓는다. 기본은 **토크를 켜 둔 채**.

        손목 리스를 손에게 비켜 주는 게 보통이고, 그때 토크를 끄면 손목이
        카메라를 든 채 늘어진다. torque_off=True 는 종료 경로 전용이다.
        """
        if self._c is None:
            return
        if torque_off:
            try:
                self._c.write_torque_enable(self.id, False)
            except Exception as e:
                print(f"[WARN] 손목 토크 해제 실패: {e}")
        self._c = None
        gc.collect()
