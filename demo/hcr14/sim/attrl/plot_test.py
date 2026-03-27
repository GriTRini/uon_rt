import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ==========================================
# 1. HCR-14 기구학 파라미터 
# ==========================================
# (x, y, z, r, p, yaw) 순서 (각도는 라디안)
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
    
    T = np.eye(4)
    T[0, 3], T[1, 3], T[2, 3] = x, y, z
    
    Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
    R_offset = Rz @ Ry @ Rx
    
    T_rot = np.eye(4)
    T_rot[0:3, 0:3] = R_offset
    
    T_q = np.eye(4)
    T_q[0:3, 0:3] = np.array([
        [np.cos(q_rad), -np.sin(q_rad), 0],
        [np.sin(q_rad),  np.cos(q_rad), 0],
        [0,              0,             1]
    ])
    
    return T @ T_rot @ T_q

def forward_kinematics(q_degrees, tcp_offset):
    """6개 조인트 각도와 TCP 오프셋을 받아 각 링크 및 TCP의 3D 좌표를 반환"""
    q_rad = np.radians(q_degrees)
    
    positions = [np.array([0.0, 0.0, 0.0])]
    T_current = np.eye(4)
    
    # 1. 로봇 플랜지(손목 끝단)까지의 FK 계산
    for i in range(6):
        T_link = get_transform_matrix(params[i], q_rad[i])
        T_current = T_current @ T_link
        positions.append(T_current[0:3, 3])
        
    # 2. TCP 오프셋 추가 계산 (툴 끝단)
    # TCP는 추가적인 관절 회전(q_rad)이 없으므로 0.0 대입
    T_tcp = get_transform_matrix(tcp_offset, 0.0)
    T_final = T_current @ T_tcp
    positions.append(T_final[0:3, 3])
        
    return np.array(positions)

# ==========================================
# 2. 데이터 로드 및 환경 설정
# ==========================================
# C++에서 뽑아낸 HCR-14 사각형 궤적 데이터 로드
df = pd.read_csv('hcr14_tcp_square_sim_data.csv')

# 애니메이션 렌더링을 위해 샘플링 (1000Hz -> 50Hz)
step_size = 20
df_sampled = df.iloc[::step_size, :].reset_index(drop=True)

# CSV의 첫 줄에서 TCP 설정값 읽어오기 (RPY는 라디안으로 변환)
tcp_raw = df.iloc[0][['TCP_X', 'TCP_Y', 'TCP_Z', 'TCP_R', 'TCP_P', 'TCP_Yw']].values
tcp_offset = [
    tcp_raw[0], tcp_raw[1], tcp_raw[2], 
    np.radians(tcp_raw[3]), np.radians(tcp_raw[4]), np.radians(tcp_raw[5])
]

# ==========================================
# 3. 3D 플롯 초기화
# ==========================================
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# 시각화 요소들
arm_line, = ax.plot([], [], [], 'o-', lw=5, markersize=8, color='#1f77b4', label='HCR-14 Arm')
tcp_path, = ax.plot([], [], [], '-', lw=3, color='#2ca02c', label='Actual TCP Path')
target_point, = ax.plot([], [], [], '*', markersize=15, color='#d62728', label='Target')

# 축 범위 설정 (HCR-14 크기 감안)
limit = 1.2
ax.set_xlim([-limit, limit])
ax.set_ylim([-limit, limit])
ax.set_zlim([-0.2, limit])
ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.set_zlabel('Z [m]')
ax.set_title('HCR-14 Cartesian Space Control (attrl)')
ax.legend()

history_x, history_y, history_z = [], [], []

# ==========================================
# 4. 애니메이션 업데이트 함수
# ==========================================
def update(frame):
    # 1. 데이터 파싱
    row = df_sampled.iloc[frame]
    q_current = row[['J1', 'J2', 'J3', 'J4', 'J5', 'J6']].values
    t_x, t_y, t_z = row['Target_X'], row['Target_Y'], row['Target_Z']
    
    # 2. 기구학 계산 (플랜지 좌표 + TCP 좌표 반환, 총 8개 포인트)
    pos = forward_kinematics(q_current, tcp_offset)
    
    # 3. 로봇 팔 업데이트
    arm_line.set_data(pos[:, 0], pos[:, 1])
    arm_line.set_3d_properties(pos[:, 2])
    
    # 4. 궤적 누적 업데이트 (배열의 가장 마지막인 [-1]이 TCP의 위치)
    history_x.append(pos[-1, 0])
    history_y.append(pos[-1, 1])
    history_z.append(pos[-1, 2])
    tcp_path.set_data(history_x, history_y)
    tcp_path.set_3d_properties(history_z)
    
    # 5. 목표 지점(Target) 업데이트
    target_point.set_data([t_x], [t_y])
    target_point.set_3d_properties([t_z])
    
    return arm_line, tcp_path, target_point

# 애니메이션 실행
ani = animation.FuncAnimation(fig, update, frames=len(df_sampled), interval=20, blit=False)

plt.show()