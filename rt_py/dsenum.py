from enum import IntEnum

__all__ = [
    "TrajState",
    "RobotState",
    "OpenConnError",
    "CloseConnError",
    "ServoOnError",
    "ServoOffError",
    "SafetyMode",
    "SafetyModeEvent",
]

# ==============================================================
# 궤적 상태 (C++ TrajState와 1:1 매칭)
# ==============================================================
class TrajState(IntEnum):
    STOP = 0
    STOPPING = 1
    TRAPJ = 2
    ATTRJ = 3
    ATTRL = 4
    PLAYJ = 5

# ==============================================================
# 로봇 일반 상태 (두산 로보틱스 기본 상태 코드 예시)
# ==============================================================
class RobotState(IntEnum):
    STATE_INITIALIZING = 0
    STATE_STANDBY = 1
    STATE_MOVING = 2
    STATE_SAFE_OFF = 3
    STATE_TEACHING = 4
    STATE_SAFE_STOP = 5
    STATE_EMERGENCY_STOP = 6
    STATE_HOMING = 7
    STATE_RECOVERY = 8
    STATE_SAFE_STOP2 = 9
    STATE_SAFE_CLEAR = 10

# ==============================================================
# 연결 및 제어 관련 에러 코드
# (보통 0이 SUCCESS, 음수가 에러/타임아웃을 의미합니다)
# ==============================================================
class OpenConnError(IntEnum):
    SUCCESS = 0
    FAILED = -1
    TIMEOUT = -2

class CloseConnError(IntEnum):
    SUCCESS = 0
    FAILED = -1
    TIMEOUT = -2

class ServoOnError(IntEnum):
    SUCCESS = 0
    FAILED = -1
    TIMEOUT = -2

class ServoOffError(IntEnum):
    SUCCESS = 0
    FAILED = -1
    TIMEOUT = -2

# ==============================================================
# 안전 모드 관련
# ==============================================================
class SafetyMode(IntEnum):
    SAFETY_MODE_NONE = 0
    SAFETY_MODE_AUTONOMOUS = 1
    SAFETY_MODE_MANUAL = 2

class SafetyModeEvent(IntEnum):
    SAFETY_EVENT_NONE = 0
    SAFETY_EVENT_STOP = 1
    SAFETY_EVENT_REDUCE_SPEED = 2