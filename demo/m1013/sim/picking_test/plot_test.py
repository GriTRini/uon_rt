import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

# --- 전역 변수: m1013 기구학 파라미터 (제공된 Joint origin 기반) ---
def get_joint_transform(idx, q_deg):
    """
    제공된 C++ joints 구조체의 origin과 axis를 기반으로 4x4 변환 행렬을 생성합니다.
    """
    q = np.radians(q_deg)
    
    # 헬퍼 함수: xyzrpy (XYZ 이동 후 ZYX 회전)
    def xyzrpy_matrix(x, y, z, r, p, yaw):
        # Rotation matrices
        Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
        Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
        R = Rz @ Ry @ Rx
        
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T

    # Joint 정의 (ID, x, y, z, r, p, yaw)
    params = [
        (0, 0, 0.1525, 0, 0, 0),         # joint1
        (0, 0.0345, 0, 0, -np.pi/2, -np.pi/2), # joint2
        (0.62, 0, 0, 0, 0, np.pi/2),      # joint3
        (0, -0.559, 0, np.pi/2, 0, 0),    # joint4
        (0, 0, 0, -np.pi/2, 0, 0),        # joint5
        (0, -0.121, 0, np.pi/2, 0, 0)     # joint6
    ]
    
    p = params[idx]
    T_origin = xyzrpy_matrix(p[0], p[1], p[2], p[3], p[4], p[5])
    
    # 각 조인트는 모두 UnitZ 축(Axis Z)으로 회전함
    T_rot = np.eye(4)
    T_rot[:3, :3] = [[np.cos(q), -np.sin(q), 0],
                     [np.sin(q), np.cos(q), 0],
                     [0, 0, 1]]
    
    return T_origin @ T_rot

def calculate_m1013_fk(joint_angles):
    """
    6개의 관절 각도를 받아 각 조인트의 전역 위치를 계산합니다.
    """
    points = [np.zeros(3)] # Base (0,0,0)
    T_accum = np.eye(4)
    
    for i in range(6):
        T_joint = get_joint_transform(i, joint_angles[i])
        T_accum = T_accum @ T_joint
        points.append(T_accum[:3, 3])
        
    return np.array(points)

def animate_m1013(file_path):
    df = pd.read_csv(file_path)
    # 데이터 샘플링 (1000Hz -> 20Hz 출력 위해 50개씩 건너뜀)
    df_sampled = df.iloc[::50, :].reset_index(drop=True)

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # m1013 작업 반경에 맞춰 범위 설정
    limit = 1.3
    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    ax.set_zlim(0, limit)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')

    line, = ax.plot([], [], [], 'o-', lw=5, markersize=8, color='#34495e', label='m1013 Link')
    trace, = ax.plot([], [], [], 'r-', lw=1.5, alpha=0.6, label='TCP Trace')
    
    title = ax.set_title('m1013 Simulation')
    trace_x, trace_y, trace_z = [], [], []

    def init():
        line.set_data([], [])
        line.set_3d_properties([])
        trace.set_data([], [])
        trace.set_3d_properties([])
        return line, trace

    def update(frame):
        angles = df_sampled.iloc[frame][['J1', 'J2', 'J3', 'J4', 'J5', 'J6']].values
        time = df_sampled.iloc[frame]['Time']
        step = df_sampled.iloc[frame]['Step']
        
        # 실제 m1013 기구학 적용
        points = calculate_m1013_fk(angles)
        
        # 로봇 링크 업데이트
        line.set_data(points[:, 0], points[:, 1])
        line.set_3d_properties(points[:, 2])
        
        # TCP 궤적 업데이트
        trace_x.append(points[-1, 0])
        trace_y.append(points[-1, 1])
        trace_z.append(points[-1, 2])
        trace.set_data(trace_x, trace_y)
        trace.set_3d_properties(trace_z)
        
        title.set_text(f'Time: {time:.2f}s | Step: {int(step)} | Pos Z: {points[-1, 2]:.3f}m')
        return line, trace, title

    # interval=50 (20fps)
    ani = FuncAnimation(fig, update, frames=len(df_sampled), 
                        init_func=init, blit=True, interval=50, repeat=False)
    
    plt.legend()
    plt.show()

if __name__ == "__main__":
    animate_m1013('complex_sim_data.csv')