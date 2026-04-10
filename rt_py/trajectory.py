import sys
from typing import Optional, overload

import numpy as np
import numpy.typing as npt

import uon_rt as _uon  # C++ 바인딩 모듈
from dsenum import TrajState

__all__ = [
    "TrajGenerator",
]


class TrajGenerator(_uon.TrajGenerator):
    """C++ TrajGenerator의 1:1 Python 래퍼 클래스"""

    def __init__(self) -> None:
        super().__init__()

    # 🌟 수정된 부분: 첫 번째 인자를 robot_model 객체 대신 model_name(문자열)으로 받습니다.
    def initialize(
        self,
        model_name: str,
        start_angles: npt.NDArray[np.float64],
        start_angvels: npt.NDArray[np.float64],
        start_angaccs: npt.NDArray[np.float64],
    ) -> None:
        # 1. 넘겨받은 문자열("m1013" 등)을 이용해 내부적으로 C++ RobotModel 객체를 생성합니다.
        c_model = _uon.RobotModel(model_name)
        
        # 2. 생성된 객체를 C++ 부모 클래스(super)의 initialize로 넘겨줍니다.
        super().initialize(c_model, start_angles, start_angvels, start_angaccs)

    def update(self, dt: float) -> None:
        super().update(dt)

    # --- TCP Management ---
    def set_tcp_tmat(self, new_shift_tmat: npt.NDArray[np.float64]) -> None:
        super().set_tcp_tmat(new_shift_tmat)

    def set_tcp(
        self,
        x: float,
        y: float,
        z: float,
        r_deg: float,
        p_deg: float,
        yaw_deg: float,
    ) -> None:
        super().set_tcp(x, y, z, r_deg, p_deg, yaw_deg)

    # --- Trajectory Commands ---
    def stop(self) -> None:
        super().stop()

    # 1~6. 에러(enorm) 측정 함수들
    def angles_enorm(self) -> Optional[float]:
        return super().angles_enorm()

    def pos_enorm(self) -> Optional[float]:
        return super().pos_enorm()

    def rot_enorm(self) -> Optional[float]:
        return super().rot_enorm()

    def angvels_enorm(self) -> Optional[float]:
        return super().angvels_enorm()

    def vel_enorm(self) -> Optional[float]:
        return super().vel_enorm()

    def w_enorm(self) -> Optional[float]:
        return super().w_enorm()

    # 목표 조회
    def goal_angles(self) -> Optional[npt.NDArray[np.float64]]:
        return super().goal_angles()

    def goal_tmat(self) -> Optional[npt.NDArray[np.float64]]:
        return super().goal_tmat()

    # 제어 명령
    def trapj(
        self,
        goal_angles: npt.NDArray[np.float64],
        goal_angvels: npt.NDArray[np.float64] = np.zeros(6),
    ) -> bool:
        return super().trapj(goal_angles, goal_angvels)

    # attrl 오버로딩
    @overload
    def attrl(self, goal_tmat: npt.NDArray[np.float64], kp: float = 50.0) -> bool: ...

    @overload
    def attrl(self, x: float, y: float, z: float, r_deg: float, p_deg: float, yaw_deg: float, kp: float = 50.0) -> bool: ...

    def attrl(self, *args, **kwargs) -> bool:
        return super().attrl(*args, **kwargs)

    def align_tcp_to_floor(self, yaw_deg: float = 0.0, kp: float = 100.0) -> bool:
        return super().align_tcp_to_floor(yaw_deg, kp)

    def align_tcp_to_front(self, kp: float = 100.0) -> bool:
        return super().align_tcp_to_front(kp)

    # --- Kinematics Solvers ---
    def solve_forward(self, q: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
        return super().solve_forward(q)

    # --- Getters ---
    def angvels(self) -> npt.NDArray[np.float64]:
        return super().angvels()

    def angaccs(self) -> npt.NDArray[np.float64]:
        return super().angaccs()

    def jmat(self) -> npt.NDArray[np.float64]:
        return super().jmat()

    def a(self) -> npt.NDArray[np.float64]:
        return super().a()

    def angles(self) -> npt.NDArray[np.float64]:
        return super().angles()

    def tmat(self) -> npt.NDArray[np.float64]:
        return super().tmat()

    def flange_tmat(self) -> npt.NDArray[np.float64]:
        return super().flange_tmat()

    def goal_reached(
        self,
        angles_enorm_thold: Optional[float] = 2.0,
        pos_enorm_thold: Optional[float] = 0.002,
        rot_enorm_thold: Optional[float] = 3.0,
        angvels_enorm_thold: Optional[float] = 4.0,
        vel_enorm_thold: Optional[float] = 0.004,
        w_enorm_thold: Optional[float] = 6.0,
    ) -> bool:
        return super().goal_reached(
            angles_enorm_thold,
            pos_enorm_thold,
            rot_enorm_thold,
            angvels_enorm_thold,
            vel_enorm_thold,
            w_enorm_thold,
        )