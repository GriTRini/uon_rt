"""dsdata.py"""

import datetime
from typing import Union

# 🌟 중요: dsrbind 대신 rt_bind를 사용 (상대 임포트 권장)
try:
    from . import rt_bind as _dsrb
except ImportError:
    import rt_bind as _dsrb

from . import rtenum as dn

__all__ = [
    "LogAlarm",
]


class LogAlarm:
    """
    C++의 _dsrb.LogAlarm 데이터를 파이썬에서 다루기 쉬운 형태로 변환하는 래퍼 클래스입니다.
    """

    time: datetime.datetime
    level: dn.LogLevel
    group: dn.LogGroup
    index: Union[
        int,
        dn.LogGroupSystemFMK,
        dn.LogGroupMotionLib,
        dn.LogGroupSafetyController,
    ]
    param1: str
    param2: str
    param3: str

    def __init__(self, alarm: _dsrb.LogAlarm) -> None:
        # C++에서 넘어온 time (보통 datetime 객체로 바인딩됨)
        self.time = alarm.time

        # 정수형 데이터를 dsenum.py에 정의된 Enum으로 변환
        self.level = dn.LogLevel(alarm.iLevel)
        self.group = dn.LogGroup(alarm.iGroup)

        try:
            if self.group == dn.LogGroup.LOG_GROUP_SYSTEMFMK:
                self.index = dn.LogGroupSystemFMK(alarm.iIndex)
            elif self.group == dn.LogGroup.LOG_GROUP_MOTIONLIB:
                self.index = dn.LogGroupMotionLib(alarm.iIndex)
            elif self.group == dn.LogGroup.LOG_GROUP_SAFETYCONTROLLER:
                self.index = dn.LogGroupSafetyController(alarm.iIndex)
            else:
                self.index = alarm.iIndex

        except ValueError:
            # 매핑되는 Enum이 없을 경우 raw integer 값 유지
            self.index = alarm.iIndex

        self.param1 = alarm.szParam1
        self.param2 = alarm.szParam2
        self.param3 = alarm.szParam3

    def __str__(self) -> str:
        return (f"LogAlarm(Time: {self.time}, Level: {self.level.name}, "
                f"Group: {self.group.name}, Index: {self.index}, "
                f"Params: [{self.param1}, {self.param2}, {self.param3}])")