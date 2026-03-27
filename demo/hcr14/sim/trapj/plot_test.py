import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ==========================================
# 1. HCR-14 기구학 파라미터 (C++ 코드 기반)
# ==========================================
# (x, y, z, r, p, yaw) 순서 
params = [
    [0.0,     0.0,  0.0,      0.0,    0.0, 0.0],       # Joint 1
    [0.0,     0.0,  0.207,    1.5708, 0.0, 0.0],       # Joint 2
    [-0.73,   0.0,  0.0,      0.0,    0.0, 0.0],       # Joint 3
    [-0.5388, 0.0,  0.0,      0.0,    0.0, 0.0],       # Joint 4
    [0.0,     0.0,  0.1847,   1.5708, 0.0, 0.0],       # Joint 5
    [0.0,     0.0,  0.1512,  -1.5708, 0.0, 0.0]        # Joint 6
]

def get_transform_matrix(xyzrpy, q_rad):
    """주어진 xyzrpy 오프셋과 현재 조인트 각도(q_rad)를 이용해 4x4 변환 행렬 생성"""
    x, y, z, r, p, yaw = xyzrpy
    
    # 1. Translation Matrix
    T = np.eye(4)
    T[0, 3], T[1, 3], T[2, 3] = x, y, z
    
    # 2. Rotation Matrix from RPY (Roll(X) -> Pitch(Y) -> Yaw(Z))
    Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
    R_offset = Rz @ Ry @ Rx
    
    T_rot = np.eye(4)
    T_rot[0:3, 0:3] = R_offset
    
    # 3. Joint Rotation (Z축 기준 회전)
    T_q = np.eye(4)
    T_q[0:3, 0:3] = np.array([
        [np.cos(q_rad), -np.sin(q_rad), 0],
        [np.sin(q_rad),  np.cos(q_rad), 0],
        [0,              0,             1]
    ])
    
    # 최종 변환 행렬 = T(offset) * R(offset) * R(joint)
    return T @ T_rot @ T_q

def forward_kinematics(q_degrees):
    """6개 조인트 각도(도)를 받아 각 링크의 3D 좌표를 반환"""
    q_rad = np.radians(q_degrees)
    
    # Base 좌표
    positions = [np.array([0.0, 0.0, 0.0])]
    T_current = np.eye(4)
    
    # 각 조인트마다 변환 행렬 누적 곱
    for i in range(6):
        T_link = get_transform_matrix(params[i], q_rad[i])
        T_current = T_current @ T_link
        positions.append(T_current[0:3, 3])
        
    return np.array(positions)

# ==========================================
# 2. 데이터 로드 및 시각화 준비
# ==========================================
df = pd.read_csv('hcr14_kinematics_data.csv')

# 1000Hz 데이터는 너무 많으므로 애니메이션을 위해 20스텝(50Hz)마다 샘플링
df_sampled = df.iloc[::20, :].reset_index(drop=True)

fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# 로봇 팔(Line)과 조인트(Scatter) 초기화
line, = ax.plot([], [], [], 'o-', lw=4, markersize=8, color='b', label='HCR-14')
tcp_path, = ax.plot([], [], [], '-', lw=2, color='r', alpha=0.5, label='TCP Path')

# 축 범위 설정 (HCR-14의 팔 길이가 약 1.5m 정도 됨을 감안)
limit = 1.5
ax.set_xlim([-limit, limit])
ax.set_ylim([-limit, limit])
ax.set_zlim([-0.2, limit])
ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.set_zlabel('Z [m]')
ax.set_title('HCR-14 Robot Arm 3D Kinematics')
ax.legend()

tcp_history_x, tcp_history_y, tcp_history_z = [], [], []

def update(frame):
    # 현재 프레임의 조인트 각도 가져오기
    q_current = df_sampled.iloc[frame][['J1', 'J2', 'J3', 'J4', 'J5', 'J6']].values
    
    # 순기구학으로 3D 좌표 계산 (shape: 7 x 3)
    pos = forward_kinematics(q_current)
    
    # 로봇 팔 업데이트
    line.set_data(pos[:, 0], pos[:, 1])
    line.set_3d_properties(pos[:, 2])
    
    # TCP 궤적 (마지막 링크 끝단) 누적
    tcp_history_x.append(pos[-1, 0])
    tcp_history_y.append(pos[-1, 1])
    tcp_history_z.append(pos[-1, 2])
    
    tcp_path.set_data(tcp_history_x, tcp_history_y)
    tcp_path.set_3d_properties(tcp_history_z)
    
    return line, tcp_path

# 애니메이션 실행
ani = animation.FuncAnimation(fig, update, frames=len(df_sampled), interval=20, blit=False)

plt.show()