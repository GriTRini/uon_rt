from enum import Enum
try:
    from . import rt_bind as _dsrb
except ImportError:
    import rt_bind as _dsrb

class TrajState(Enum):
    STOP = _dsrb.TrajState.STOP
    STOPPING = _dsrb.TrajState.STOPPING
    TRAPJ = _dsrb.TrajState.TRAPJ
    ATTRJ = _dsrb.TrajState.ATTRJ
    ATTRL = _dsrb.TrajState.ATTRL
    PLAYJ = _dsrb.TrajState.PLAYJ

class OpenConnError(Enum):
    NO_ERROR = 0
    # ... 이전 제공해주신 에러 코드들 나열