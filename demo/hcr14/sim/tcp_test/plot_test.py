import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# --- HCR-14 기구학 및 변환 함수 (동일) ---
def get_transform(idx, q_deg):
    q = np.radians(q_deg)
    def m(x, y, z, r, p, yaw):
        Rx = np.array([[1,0,0],[0,np.cos(r),-np.sin(r)],[0,np.sin(r),np.cos(r)]])
        Ry = np.array([[np.cos(p),0,np.sin(p)],[0,1,0],[-np.sin(p),0,np.cos(p)]])
        Rz = np.array([[np.cos(yaw),-np.sin(yaw),0],[np.sin(yaw),np.cos(yaw),0],[0,0,1]])
        T = np.eye(4); T[:3,:3] = Rz @ Ry @ Rx; T[:3,3] = [x,y,z]
        return T
    
    # HCR-14 파라미터
    params = [
        (0.0,     0.0,  0.0,      0.0,    0.0, 0.0),       
        (0.0,     0.0,  0.207,    1.5708, 0.0, 0.0),       
        (-0.73,   0.0,  0.0,      0.0,    0.0, 0.0),       
        (-0.5388, 0.0,  0.0,      0.0,    0.0, 0.0),       
        (0.0,     0.0,  0.1847,   1.5708, 0.0, 0.0),       
        (0.0,     0.0,  0.1512,  -1.5708, 0.0, 0.0)        
    ]
    p = params[idx]
    T_origin = m(p[0], p[1], p[2], p[3], p[4], p[5])
    T_rot = np.eye(4); T_rot[:3,:3] = [[np.cos(q),-np.sin(q),0], [np.sin(q),np.cos(q),0], [0,0,1]]
    return T_origin @ T_rot

def get_tcp_matrix(x, y, z, r_deg, p_deg, y_deg):
    r, p, yaw = np.radians([r_deg, p_deg, y_deg])
    Rx = np.array([[1,0,0],[0,np.cos(r),-np.sin(r)],[0,np.sin(r),np.cos(r)]])
    Ry = np.array([[np.cos(p),0,np.sin(p)],[0,1,0],[-np.sin(p),0,np.cos(p)]])
    Rz = np.array([[np.cos(yaw),-np.sin(yaw),0],[np.sin(yaw),np.cos(yaw),0],[0,0,1]])
    T = np.eye(4); T[:3,:3] = Rz @ Ry @ Rx; T[:3,3] = [x, y, z]
    return T

# --- 메인 시각화 함수 ---
def animate_multi_robust(file_path):
    print("🚀 파이썬 시각화 스크립트가 정상적으로 실행되었습니다!")
    df = pd.read_csv(file_path)
    df_sampled = df.iloc[::40, :].reset_index(drop=True)

    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')
    
    # 축 영역 설정
    limit = 1.5
    ax.set_xlim(-limit, limit); ax.set_ylim(-limit, limit); ax.set_zlim(-0.8, limit)
    ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
    
    # 그릴 요소들 정의
    line, = ax.plot([], [], [], 'o-', lw=6, color='#34495e', label='Robot Arm')
    tool_line, = ax.plot([], [], [], '-', lw=6, color='#A9A9A9', label='Tool (TCP Offset)')
    z_axis, = ax.plot([], [], [], '-', lw=3, color='#ff7f0e', label='TCP Z-Axis')
    target_dot, = ax.plot([], [], [], 'r*', markersize=15, label='Target')

    # 🌟 지나간 경로를 표시할 초록색 줄 추가 
    tcp_path, = ax.plot([], [], [], '-', lw=2, color='#2ca02c', label='Actual TCP Path')
    
    # 🌟 이전 프레임의 데이터를 저장할 변수
    last_valid_pts = np.zeros((7, 3))
    last_valid_tcp = np.zeros(3)
    last_valid_z_end = np.zeros(3)
    
    # 🌟 지나간 경로를 저장할 리스트
    history_x, history_y, history_z = [], [], []

    def update(frame):
        nonlocal last_valid_pts, last_valid_tcp, last_valid_z_end
        row = df_sampled.iloc[frame]
        
        angles = row[['J1', 'J2', 'J3', 'J4', 'J5', 'J6']].values.astype(float)
        scenario = row['Scenario']
        
        if frame % 20 == 0:
            print(f"▶ 진행 상황: 프레임 {frame}/{len(df_sampled)} | 현재 동작: {scenario}")

        try:
            # 1. 로봇 본체 계산 (FK)
            T_accum = np.eye(4); pts = [np.zeros(3)]
            for i in range(6):
                T_joint = get_transform(i, angles[i])
                if np.isnan(T_joint).any() or np.abs(T_joint).max() > 1e5:
                    raise ValueError(f"Invalid joint transform at J{i+1}")
                T_accum = T_accum @ T_joint
                pts.append(T_accum[:3, 3])
            
            pts = np.array(pts)

            # 2. TCP 및 Z축 계산
            t_x, t_y, t_z = float(row['TCP_X']), float(row['TCP_Y']), float(row['TCP_Z'])
            t_r, t_p, t_yw = float(row['TCP_R']), float(row['TCP_P']), float(row['TCP_Yw'])
            
            T_tcp_off = get_tcp_matrix(t_x, t_y, t_z, t_r, t_p, t_yw)
            T_tcp_final = T_accum @ T_tcp_off
            
            p_tcp = T_tcp_final[:3, 3]
            z_end = p_tcp + T_tcp_final[:3, 2] * 0.3 

            last_valid_pts = pts
            last_valid_tcp = p_tcp
            last_valid_z_end = z_end

        except Exception as e:
            print(f"⚠️ Error at frame {frame} ({scenario}): {e}")
            pts = last_valid_pts
            p_tcp = last_valid_tcp
            z_end = last_valid_z_end

        # 🌟 지나간 경로 데이터 업데이트
        history_x.append(p_tcp[0])
        history_y.append(p_tcp[1])
        history_z.append(p_tcp[2])

        # 그래프 업데이트
        line.set_data(pts[:, 0], pts[:, 1]); line.set_3d_properties(pts[:, 2])
        
        # 툴(회색 선)과 Z축(주황색 선) 연결
        tool_line.set_data([pts[6,0], p_tcp[0]], [pts[6,1], p_tcp[1]])
        tool_line.set_3d_properties([pts[6,2], p_tcp[2]])
        
        z_axis.set_data([p_tcp[0], z_end[0]], [p_tcp[1], z_end[1]])
        z_axis.set_3d_properties([p_tcp[2], z_end[2]])

        target_dot.set_data([float(row['Target_X'])], [float(row['Target_Y'])])
        target_dot.set_3d_properties([float(row['Target_Z'])])
        
        # 🌟 지나간 경로 그래프 업데이트
        tcp_path.set_data(history_x, history_y)
        tcp_path.set_3d_properties(history_z)
        
        ax.set_title(f"Scenario: {scenario} | Frame: {frame}")
        return line, tool_line, z_axis, target_dot, tcp_path

    ani = FuncAnimation(fig, update, frames=len(df_sampled), interval=50, blit=False, repeat=False)
    plt.legend(loc='upper right'); plt.show()

if __name__ == "__main__":
    animate_multi_robust('hcr14_tcp_square_sim_data.csv')