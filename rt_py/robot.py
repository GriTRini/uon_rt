import datetime
from typing import Optional, overload

import numpy as np
import numpy.typing as npt

# 빌드된 pybind11 모듈 (이름은 실제 환경에 맞게 수정하세요)
import uon_rt as _uon 

__all__ = [
    "Robot",
]

class Robot:
    def __init__(self, model_name: str) -> None:
        # C++ RobotFactory를 통해 실제 로봇 객체 생성 및 바인딩
        self._arm = _uon.RobotFactory.create(model_name)
        if self._arm is None:
            raise RuntimeError(f"[Error] 지원하지 않거나 인식할 수 없는 로봇 모델입니다: {model_name}")

    # ==============================================================
    # 🌟 공통 수학 및 기구학 API
    # ==============================================================

    def set_tcp(
        self, 
        x: float, y: float, z: float, 
        r_deg: float, p_deg: float, yaw_deg: float
    ) -> None:
        """TCP(Tool Center Point) 오프셋을 설정합니다."""
        self._arm.set_tcp(x, y, z, r_deg, p_deg, yaw_deg)

    def get_current_pos(self) -> npt.NDArray[np.float64]:
        """현재 TCP 포즈(4x4 동차 변환 행렬)를 반환합니다."""
        return self._arm.get_current_pos()

    def get_current_flange_pos(self) -> npt.NDArray[np.float64]:
        """순수 로봇 손목(Flange) 포즈(4x4 동차 변환 행렬)를 반환합니다."""
        return self._arm.get_current_flange_pos()

    def get_jacobian(self) -> npt.NDArray[np.float64]:
        """현재 자코비안 행렬(6x6)을 반환합니다."""
        return self._arm.get_jacobian()

    def get_task_vel(self) -> npt.NDArray[np.float64]:
        """현재 작업 공간의 선/각속도 (Task Velocity, 6x1)를 반환합니다."""
        return self._arm.get_task_vel()

    def solve_forward(self, q: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
        """주어진 관절 각도(q)에 대한 Forward Kinematics (4x4 행렬)를 계산하여 반환합니다."""
        return self._arm.solve_forward(q)

    # ==============================================================
    # 🌟 공통 궤적 제어 API
    # ==============================================================

    def stop(self) -> None:
        """현재 궤적을 부드럽게 정지(STOPPING 상태로 전환)합니다."""
        self._arm.stop()

    def trapj(
        self, 
        goal_angles: npt.NDArray[np.float64], 
        goal_angvels: Optional[npt.NDArray[np.float64]] = None
    ) -> bool:
        """Joint 공간 사다리꼴 속도(Trapezoidal) 제어를 시작합니다."""
        return self._arm.trapj(goal_angles, goal_angvels)

    @overload
    def attrl(self, goal_tmat: npt.NDArray[np.float64], kp: float = 50.0) -> bool: ...

    @overload
    def attrl(self, x: float, y: float, z: float, r_deg: float, p_deg: float, yaw_deg: float, kp: float = 50.0) -> bool: ...

    def attrl(self, *args, **kwargs) -> bool:
        """Cartesian 공간 Attractor 제어를 시작합니다. (행렬 또는 6D 좌표 사용 가능)"""
        return self._arm.attrl(*args, **kwargs)

    def align_tcp_to_floor(self, yaw_deg: float = 0.0, kp: float = 100.0) -> bool:
        """바닥(-Z) 방향으로 TCP를 정렬합니다."""
        return self._arm.align_tcp_to_floor(yaw_deg, kp)

    def align_tcp_to_front(self, kp: float = 100.0) -> bool:
        """정면(+X) 방향으로 TCP를 정렬합니다."""
        return self._arm.align_tcp_to_front(kp)

    def get_goal_reached(
        self, 
        q_th: Optional[float] = None, 
        p_th: Optional[float] = None, 
        r_th: Optional[float] = None
    ) -> bool:
        """
        현재 궤적이 목표에 도달했는지 확인합니다.
        
        Args:
            q_th: 관절 각도 오차 허용치 [deg]
            p_th: 위치 오차 허용치 [m]
            r_th: 회전 오차 허용치 [deg]
            
        Returns:
            목표에 도달했으면 True, 이동 중이면 False
        """
        return self._arm.get_goal_reached(q_th, p_th, r_th)

    # ==============================================================
    # 🔌 하드웨어 종속 API 
    # ==============================================================

    def open_connection(self, ip: str = "192.168.1.30", port: int = 12345) -> bool:
        """로봇 컨트롤러와 일반 연결을 시작합니다."""
        return self._arm.open_connection(ip, port)

    def close_connection(self) -> bool:
        """로봇 컨트롤러와의 일반 연결을 해제합니다."""
        return self._arm.close_connection()

    def connect_rt(self, ip: str = "192.168.1.30", port: int = 12347) -> bool:
        """로봇 컨트롤러와 실시간(RT) 통신을 시작합니다."""
        return self._arm.connect_rt(ip, port)

    def disconnect_rt(self) -> None:
        """실시간(RT) 통신을 해제합니다."""
        self._arm.disconnect_rt()

    def servo_on(self) -> bool:
        """서보 모터를 켭니다."""
        return self._arm.servo_on()

    def servo_off(self) -> bool:
        """서보 모터를 끕니다."""
        return self._arm.servo_off()

    def get_current_angles(self) -> Optional[npt.NDArray[np.float64]]:
        """하드웨어에서 읽어온 현재 관절 각도(6x1)를 반환합니다."""
        return self._arm.get_current_angles()

    def get_current_angvels(self) -> Optional[npt.NDArray[np.float64]]:
        """하드웨어에서 읽어온 현재 관절 각속도(6x1)를 반환합니다."""
        return self._arm.get_current_angvels()

    def set_digital_output(self, index: int, value: bool) -> None:
        """디지털 출력 포트의 값을 설정합니다."""
        self._arm.set_digital_output(index, value)