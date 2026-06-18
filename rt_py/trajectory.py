import numpy as np
import numpy.typing as npt
from typing import Optional, List, Dict, Any  # 🌟 playj를 위해 Dict, Any 추가
import rt_bind as _rtb

class RobotModel(_rtb.RobotModel):
    """
    로봇 기구학 모델 정보를 관리하는 클래스 (물리적 연결 없음)
    """
    def __init__(self, model_name: str):
        super().__init__(model_name)

class TrajGenerator(_rtb.TrajGenerator):
    """
    순수 시뮬레이션용 궤적 생성기 인터페이스.
    하드웨어 연결 없이 파이썬에서 update(dt)를 직접 호출하여 궤적을 연산합니다.
    """
    
    def __init__(self):
        super().__init__()

    def initialize(self, model: RobotModel, q: npt.NDArray, dq: npt.NDArray, ddq: npt.NDArray):
        """궤적 엔진을 특정 상태로 초기화합니다."""
        super().initialize(model, q, dq, ddq)

    def update(self, dt: float):
        """시간 간격(dt)만큼 궤적 상태를 갱신합니다. (보통 0.001초 간격으로 호출)"""
        super().update(dt)

    def set_tcp(self, x: float, y: float, z: float, r: float, p: float, yaw: float):
        super().set_tcp(x, y, z, r, p, yaw)

    def trapj(self, goal_q: npt.NDArray, goal_dq: Optional[npt.NDArray] = None) -> bool:
        """관절 공간 사다리꼴 궤적 생성 (시뮬레이션)"""
        return super().trapj(goal_q, goal_dq)

    # 🌟 신규 추가: playj 시뮬레이터 바인딩 함수
    def playj(self, waypoints: List[Dict[str, Any]], peak_vels: Optional[npt.NDArray] = None, peak_accs: Optional[npt.NDArray] = None, p_gain: float = 5.0) -> bool:
        """
        다중 웨이포인트 스무딩 연속 이동 (시뮬레이션)
        
        Args:
            waypoints: 각도(q)와 블렌딩 반경(attrl)을 담은 딕셔너리 리스트. 
                       예: [{"q": np.array([...]), "attrl": 0.15}, ...]
            peak_vels: 조인트 최대 속도 한계 배열. None이면 로봇의 물리적 한계 자동 사용.
            peak_accs: 조인트 최대 가속도 한계 배열. None이면 로봇의 물리적 한계 자동 사용.
            p_gain: 궤적 추종 위치 게인. 값이 클수록 타겟에 더 빨리 붙음. (기본값: 5.0)
        """
        return super().playj(waypoints, peak_vels, peak_accs, p_gain)

    def attrl(self, goal_tmat: npt.NDArray, attrl_kp: float = 50.0, attrj_kp: float = 150.0, target_speed: float = 0.20) -> bool:
        """작업 공간 어트랙터 기반 궤적 생성 (시뮬레이션)"""
        return super().attrl(goal_tmat, attrl_kp, attrj_kp, target_speed)

    def align_to_floor(self, yaw_deg: float = 0.0, kp: float = 100.0) -> bool:
        return super().align_to_floor(yaw_deg, kp)
    
    def align_to_front(self, kp: float = 100.0) -> bool:
        return super().align_to_front(kp)

    def stop(self):
        super().stop()

    def goal_reached(self, q_th: float = 2.0, p_th: float = 0.002, r_th: float = 3.0) -> bool:
        """현재 가상 궤적이 목표지점에 도달했는지 확인"""
        return super().goal_reached(q_th, p_th, r_th)

    # ==============================================================
    # 📊 가상 상태 모니터링 Properties
    # ==============================================================

    @property
    def angles(self) -> npt.NDArray: 
        return super().angles

    @property
    def tmat(self) -> npt.NDArray: 
        return super().tmat

    @property
    def flange_tmat(self) -> npt.NDArray:
        """TCP 오프셋이 적용되지 않은 순수 손목 포즈"""
        return super().flange_tmat
        
    @property
    def jmat(self) -> npt.NDArray:
        """현재 가상 자세의 자코비안 행렬"""
        return super().jmat