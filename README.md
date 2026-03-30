## 🚀 Motion Control & Kinematics Engine

본 프로젝트는 6자유도(6-DoF) 다관절 로봇 매니퓰레이터의 안정적이고 정밀한 모션 제어를 위해, 자체적으로 구현한 **관절/데카르트 공간 제어기(Motion Generators)**와 **최적화 기반의 역기구학(QP-based IK) 솔버**를 포함하고 있습니다.

### 1. Joint-Space Motion Planning (관절 공간 제어기)

#### `TrajTrapJ` (Synchronized Trapezoidal Profile Generator)
다관절 로봇의 부드러운 이동을 위해 모든 조인트가 동시에 출발하고 동시에 도착하도록 동기화(Synchronization)하는 사다리꼴 속도 프로파일 생성기입니다.
* **Analytical Time-Optimal Solution**: 임의의 초기 속도(v0)와 최종 속도(vf) 상태에서도 가속도 한계를 넘지 않는 최적의 해석적 해를 도출합니다.
* **Dynamic Replanning**: 각 조인트별로 계산된 최소 소요 시간 중 최대값(m_max_duration)을 기준으로, 나머지 조인트들의 가감속 구간 및 등속 구간을 재계산하여 완벽한 동기화를 이룹니다.
* **Edge-case Handling**: 주어진 가속도로 목표 속도 도달이 불가능할 경우, 사다리꼴 형태를 삼각형(Triangular Profile) 프로파일로 자동 강제 전환하여 오버슈트를 방지합니다.

#### `TrajAttrJ` (Joint-Space PD Attractor)
로봇의 현재 관절 위치를 목표 위치로 부드럽게 끌어당기는 Proportional-Derivative(PD) 기반의 관절 공간 어트랙터입니다.
* **Critical Damping Auto-Tuning**: 사용자가 위치 제어 게인(Kp)만 설정하면, 오버슈트가 발생하지 않는 임계 감쇠(Critical Damping) 조건인 `Kd = 2 * sqrt(Kp)`를 알고리즘이 자동으로 계산하여 적용합니다.
* **Hardware Limit Protection**: 매 제어 주기(Update Loop)마다 각 조인트의 하드웨어 스펙(최대 위치, 속도, 가속도)을 초과하지 않도록 안전하게 클램핑(Clamping)합니다.

---

### 2. Cartesian-Space Motion Planning (데카르트 공간 제어기)

#### `CartesianAttractor` (Task-Space Attractor)
툴 끝단(TCP)이 작업 공간(Task Space)상에서 최단 직선 궤적과 부드러운 회전 궤적을 그리며 목표 자세(Pose)를 추종하는 제어기입니다.
* **SO(3) Exponential Map Control**: 회전(Orientation) 오차를 오일러 각(Euler Angles)이 아닌, SO(3) 공간의 지수 사상(Exponential Map)과 Angle-Axis 변환을 통해 계산하여 짐벌락(Gimbal Lock) 현상 없이 정확한 회전 모션을 생성합니다.
* **Independent Axis Clipping**: 선가속도, 선속도, 각가속도, 각속도를 각각 독립적으로 추적하고 제한(clip_norm)하여, 고속 이동 시에도 로봇 툴이 경로를 이탈하지 않도록 방어합니다.

---

### 3. Advanced Kinematics & IK Solver (고도화된 기구학 엔진)

일반적인 DLS(Damped Least Squares) 기반 역기구학이 가지는 치명적인 한계(특이점 부근에서의 속도 발산, 하드웨어 한계 무시, 장거리 이동 시의 궤적 휘청거림)를 극복하기 위해, **Projected Gauss-Seidel QP (Quadratic Programming)** 알고리즘을 도입한 고급 IK 솔버를 구축했습니다.

#### TCP Offset Integration (`compute_forward_and_jacobian`)
플랜지(Flange) 끝단이 아닌, 크기와 비틀림(RPY)이 적용된 복잡한 형상의 툴(TCP)을 기준으로 기하학적 자코비안(Geometric Jacobian)을 실시간으로 도출하여, 실제 작업점 중심의 제어가 가능하도록 설계했습니다.

#### Optimization-based Solver (`solve_step_qp`)
IK 문제를 최적화 문제로 재정의하여 조인트 한계(Joint Limits)와 속도 제한을 하드 제약 조건(Hard Constraints)으로 두었습니다.
* **Objective Function**: `min(dq) 0.5 * dq^T * H * dq + g^T * dq` 
  (단, `H = J^T * J + lambda^2 * I`, `g = -J^T * error`)
* **Box Constraints**: `lb <= dq <= ub` 

#### 🌟 3대 핵심 방어 로직 (Robustness Features)
로봇을 극한의 궤적(예: 180도 반대편으로 순간이동)으로 구동할 때 발생하는 역행렬 폭주와 궤적 찌그러짐을 해결하기 위해 다음의 3가지 알고리즘을 결합했습니다.

1. **Cartesian Error Clamping (공간 에러 쪼개기)**
   * 타겟이 수 미터 떨어져 있을 때 발생하는 자코비안 역행렬의 속도 폭주를 막기 위해, 한 번의 IK 스텝당 처리할 수 있는 최대 공간 에러를 위치 5cm, 회전 5.7도 수준으로 제한합니다. 이를 통해 먼 거리 이동 시에도 각 조인트가 일정한 비율을 유지하여 곧은 직선 궤적을 만들어 냅니다.
2. **Adaptive Continuous Damping (적응형 연속 댐핑)**
   * 특이점(Singularity) 부근이나 목표 도달 직전에 발생하는 부동 소수점 단위의 잔진동(Jittering)을 제거하기 위해, 목표물에 다가갈수록 댐핑 계수(lambda)가 0에 가깝게 연속적이고 부드럽게 감소하도록 비선형 함수를 적용했습니다.
3. **Null-Space Joint Centering (영공간 조인트 중앙화)**
   * 최적화 목적 함수(g)에 Primary Task(TCP 이동)를 방해하지 않는 아주 미세한 보조 인력(Secondary Task)을 추가했습니다. 로봇 팔이 특이점이나 관절 한계에 가까워지려 할 때, 각 조인트가 가급적 기구학적 중앙(Center)을 유지하려는 성질을 부여하여 사람의 팔처럼 자연스럽고 우아하게 장애 자세를 우회합니다.