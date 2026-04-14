import numpy as np
import numpy.typing as npt
from typing import Optional
import datetime
from . import rt_bind as _rtb

def create_robot(model_name: str = "m1013"):
    """
    C++ RobotFactory를 호출하여 모델에 맞는 로봇 객체를 생성합니다.
    """
    return _rtb.create_robot(model_name)

class Robot(_rtb.RobotBase):
    """
    Doosan Robotics 실로봇 제어를 위한 파이썬 인터페이스 (Header 역할).
    C++ DsrRobot 클래스의 기능을 상속받아 파이썬 친화적인 API를 제공합니다.
    """

    def __init__(self, model_name: str = "m1013", update_dt_ms: int = 1):
        """
        로봇 객체를 초기화합니다.
        
        Args:
            model_name: 로봇 모델 이름 (기본값: "m1013")
            update_dt_ms: 제어 루프 주기 [ms] (기본값: 1ms)
        """
        # C++ RobotBase(model_name, update_dt) 생성자 호출
        super().__init__(model_name, update_dt_ms)

    # ==============================================================
    # 🔌 연결 및 하드웨어 제어 API
    # ==============================================================

    def open_connection(self, ip: str = "192.168.1.30", port: int = 12345) -> bool:
        """
        로봇 컨트롤러와 TCP/IP 연결을 시도합니다. (내부적으로 권한 요청 포함)
        
        Args:
            ip: 컨트롤러 IP 주소
            port: 연결 포트 (기본: 12345)
        """
        return super().open_connection(ip, port)

    def connect_rt(self, ip: str = "192.168.1.30", port: int = 12347) -> bool:
        """
        실시간 제어를 위한 UDP RT 연결을 수행합니다.
        
        Args:
            ip: 컨트롤러 IP 주소
            port: RT 포트 (기본: 12347)
        """
        return super().connect_rt(ip, port)

    def servo_on(self) -> bool:
        """
        로봇의 서보를 켭니다. 
        성공 시 내부적으로 현재 각도를 기반으로 궤적 엔진이 자동 초기화됩니다.
        """
        return super().servo_on()

    def servo_off(self) -> bool:
        """실시간 제어 루프를 중단하고 서보를 끕니다."""
        return super().servo_off()

    def set_digital_output(self, index: int, value: bool):
        """
        컨트롤 박스의 디지털 출력을 제어합니다.
        
        Args:
            index: 디지털 출력 핀 번호 (1~16)
            value: True(ON) / False(OFF)
        """
        # C++ 내부에서 index-1 처리를 수행함
        super().set_digital_output(index, value)

    # ==============================================================
    # 🚀 궤적 제어 API (Movement Commands)
    # ==============================================================

    def set_tcp(self, x: float, y: float, z: float, r: float, p: float, yaw: float):
        """툴 오프셋(TCP)을 설정합니다. (단위: m, deg)"""
        super().set_tcp(x, y, z, r, p, yaw)

    def trapj(self, goal_q: npt.NDArray) -> bool:
        """관절 공간 사다리꼴 궤적 이동 명령 (Non-blocking)"""
        return super().trapj(goal_q)

    def attrl(self, goal_tmat: npt.NDArray, kp: float = 50.0) -> bool:
        """작업 공간 어트랙터 기반 직선 이동 명령 (4x4 Matrix 입력)"""
        return super().attrl(goal_tmat, kp)

    def align_to_floor(self, yaw_deg: float = 0.0, kp: float = 100.0) -> bool:
        """현재 위치를 유지하며 TCP를 바닥 방향(-Z)으로 정렬합니다."""
        return super().align_to_floor(yaw_deg, kp)

    def stop(self):
        """로봇을 즉시 부드럽게 정지시킵니다."""
        super().stop()

    # ==============================================================
    # 📊 상태 모니터링 API (Properties)
    # ==============================================================

    @property
    def angles(self) -> npt.NDArray[np.float64]:
        """실시간 로봇 관절 각도 [deg]"""
        return super().angles

    @property
    def tmat(self) -> npt.NDArray[np.float64]:
        """실시간 TCP 포즈 행렬 (4x4)"""
        return super().tmat

    @property
    def pos_enorm(self) -> float:
        """현재 목표 위치와 실제 위치 사이의 거리 오차 [m]"""
        return super().pos_enorm

    def goal_reached(self, q_th: float = 2.0, p_th: float = 0.002, r_th: float = 3.0) -> bool:
        """
        로봇이 목표 지점에 도달했는지 확인합니다.
        
        Args:
            q_th: 관절 오차 임계값 [deg]
            p_th: 위치 오차 임계값 [m]
            r_th: 회전 오차 임계값 [deg]
        """
        return super().goal_reached(q_th, p_th, r_th)