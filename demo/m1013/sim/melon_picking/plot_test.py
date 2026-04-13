import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ==============================================================================
# 🌟 M1013 Forward Kinematics (C++ 파라미터 완벽 이식)
# ==============================================================================

def xyzrpy_to_matrix(x, y, z, roll, pitch, yaw):
    """XYZ 이동 후 ZYX(Roll-Pitch-Yaw) 순서로 회전하는 4x4 변환 행렬 생성"""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    
    T = np.eye(4)
    T[:3, :3] = Rz @ Ry @ Rx
    T[:3, 3] = [x, y, z]
    return T

# C++ 코드에 정의된 관절별 Origin 파라미터 (x, y, z, r, p, yaw)
M1013_PARAMS = [
    (0, 0, 0.1525, 0, 0, 0),                     # Joint 1
    (0, 0.0345, 0, 0, -np.pi/2, -np.pi/2),       # Joint 2
    (0.62, 0, 0, 0, 0, np.pi/2),                 # Joint 3
    (0, -0.559, 0, np.pi/2, 0, 0),               # Joint 4
    (0, 0, 0, -np.pi/2, 0, 0),                   # Joint 5
    (0, -0.121, 0, np.pi/2, 0, 0)                # Joint 6
]

# 연산 속도 최적화를 위해 Origin 변환 행렬을 미리 계산해 둡니다.
T_ORIGINS = [xyzrpy_to_matrix(*p) for p in M1013_PARAMS]

def get_joint_transform(idx, q_rad):
    """특정 조인트의 현재 각도를 반영한 지역 변환 행렬 계산"""
    T_rot = np.eye(4)
    c, s = np.cos(q_rad), np.sin(q_rad)
    # 모든 관절이 UnitZ축 기준으로 회전함
    T_rot[:3, :3] = np.array([
        [c, -s, 0],
        [s,  c, 0],
        [0,  0, 1]
    ])
    return T_ORIGINS[idx] @ T_rot

def calculate_m1013_fk(q_deg):
    """6개의 관절 각도(Degree)를 받아 Base부터 Flange까지 7개의 3D 좌표 반환"""
    q_rad = np.radians(q_deg)
    points = [np.array([0.0, 0.0, 0.0])] # Base 좌표 (0,0,0)
    T_accum = np.eye(4)
    
    for i in range(6):
        T_accum = T_accum @ get_joint_transform(i, q_rad[i])
        points.append(T_accum[:3, 3])
        
    return np.array(points)

# ==============================================================================
# 🌟 3D 애니메이션 시각화
# ==============================================================================

def animate_m1013(file_path):
    # CSV 데이터 로드
    try:
        df = pd.read_csv(file_path)
    except FileNotFoundError:
        print(f"❌ 파일을 찾을 수 없습니다: {file_path}")
        return

    # 1ms(1000Hz) 데이터는 너무 무거우므로 50개씩 건너뛰어 20Hz(50ms)로 샘플링
    df_sampled = df.iloc[::5, :].reset_index(drop=True)

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # 작업 반경(Reach)에 맞춘 3D 공간 비율 고정
    limit = 1.3
    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    ax.set_zlim(0, limit)
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')

    # 로봇 팔(Link) 라인과 TCP 궤적(Trace) 라인 초기화
    line, = ax.plot([], [], [], 'o-', lw=6, markersize=8, color='#2c3e50', label='M1013 Links')
    trace, = ax.plot([], [], [], 'r-', lw=2, alpha=0.7, label='TCP Trace')
    
    title = ax.set_title('Doosan M1013 Simulation')
    trace_x, trace_y, trace_z = [], [], []

    def init():
        line.set_data([], [])
        line.set_3d_properties([])
        trace.set_data([], [])
        trace.set_3d_properties([])
        return line, trace

    def update(frame):
        # 🌟 수정됨: .astype(float)를 붙여서 완벽한 실수형 Numpy 배열로 강제 변환!
        angles = df_sampled.iloc[frame][['J1', 'J2', 'J3', 'J4', 'J5', 'J6']].values.astype(float)
        time_sec = float(df_sampled.iloc[frame]['Time']) # 혹시 몰라 시간도 강제 변환
        
        # StepInfo가 있으면 가져오고, 없으면 Step(숫자) 가져오기
        step_info = df_sampled.iloc[frame].get('StepInfo', df_sampled.iloc[frame].get('Step', ''))
        
        # 순기구학 계산으로 3D 좌표 획득
        points = calculate_m1013_fk(angles)
        
        # 관절 라인 그리기 (X, Y 배열 세팅 후 Z 배열 세팅)
        line.set_data(points[:, 0], points[:, 1])
        line.set_3d_properties(points[:, 2])
        
        # 끝단(TCP/Flange) 좌표 누적하여 궤적 그리기
        trace_x.append(points[-1, 0])
        trace_y.append(points[-1, 1])
        trace_z.append(points[-1, 2])
        trace.set_data(trace_x, trace_y)
        trace.set_3d_properties(trace_z)
        
        title.set_text(f'Time: {time_sec:.2f}s | Step: {step_info}\nTCP Z: {points[-1, 2]:.3f} m')
        return line, trace, title

    # 애니메이션 실행 (interval=50 -> 20FPS)
    ani = FuncAnimation(fig, update, frames=len(df_sampled), 
                        init_func=init, blit=False, interval=50, repeat=False)
    
    plt.legend()
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # C++ 시뮬레이션 코드에서 만든 파일 이름을 넣어주세요
    animate_m1013('generator_diagnosis_log.csv')