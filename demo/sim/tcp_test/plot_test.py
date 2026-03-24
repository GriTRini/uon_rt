import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# --- 기구학 및 변환 함수 (동일) ---
def get_transform(idx, q_deg):
    q = np.radians(q_deg)
    def m(x, y, z, r, p, yaw):
        Rx = np.array([[1,0,0],[0,np.cos(r),-np.sin(r)],[0,np.sin(r),np.cos(r)]])
        Ry = np.array([[np.cos(p),0,np.sin(p)],[0,1,0],[-np.sin(p),0,np.cos(p)]])
        Rz = np.array([[np.cos(yaw),-np.sin(yaw),0],[np.sin(yaw),np.cos(yaw),0],[0,0,1]])
        T = np.eye(4); T[:3,:3] = Rz @ Ry @ Rx; T[:3,3] = [x,y,z]
        return T
    params = [(0,0,0.1525,0,0,0), (0,0.0345,0,0,-np.pi/2,-np.pi/2), (0.62,0,0,0,0,np.pi/2),
              (0,-0.559,0,np.pi/2,0,0), (0,0,0,-np.pi/2,0,0), (0,-0.121,0,np.pi/2,0,0)]
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

def animate_multi_robust(file_path):
    df = pd.read_csv(file_path)
    # 🌟 샘플링 간격을 넓혀서 진행 속도를 높입니다. (디버깅용)
    df_sampled = df.iloc[::80, :].reset_index(drop=True)

    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')
    # 🌟 작업 영역을 조금 더 넓게 잡습니다.
    ax.set_xlim(-0.4, 1.0); ax.set_ylim(-0.6, 0.6); ax.set_zlim(0, 1.2)
    
    line, = ax.plot([], [], [], 'o-', lw=6, color='#34495e', label='Robot Arm')
    tool_line, = ax.plot([], [], [], 'o-', lw=4, color='#27ae60', label='Tool (TCP)')
    z_axis, = ax.plot([], [], [], 'b-', lw=2, label='TCP Z-Axis')
    target_dot, = ax.plot([], [], [], 'r*', markersize=15, label='Target')

    # 🌟 이전 프레임의 데이터를 저장할 변수 (에러 발생 시 사용)
    last_valid_pts = np.zeros((7, 3))
    last_valid_tcp = np.zeros(3)
    last_valid_z_end = np.zeros(3)

    def update(frame):
        nonlocal last_valid_pts, last_valid_tcp, last_valid_z_end
        row = df_sampled.iloc[frame]
        angles = row[['J1', 'J2', 'J3', 'J4', 'J5', 'J6']].values
        scenario = row['Scenario']
        
        # 🌟 디버깅 정보 출력
        if frame % 20 == 0:
            print(f"Processing frame {frame}/{len(df_sampled)} | Scenario: {scenario}")

        try:
            # 1. 로봇 본체 계산 (FK)
            T_accum = np.eye(4); pts = [np.zeros(3)]
            for i in range(6):
                T_joint = get_transform(i, angles[i])
                
                # 🌟 데이터 이상 증상 체크 (NaN 또는 너무 큰 값)
                if np.isnan(T_joint).any() or np.abs(T_joint).max() > 1e5:
                    raise ValueError(f"Invalid joint transform at J{i+1}")
                    
                T_accum = T_accum @ T_joint
                pts.append(T_accum[:3, 3])
            
            pts = np.array(pts)

            # 2. TCP 및 Z축 계산
            T_tcp_off = get_tcp_matrix(row['TCP_X'], row['TCP_Y'], row['TCP_Z'], 
                                       row['TCP_R'], row['TCP_P'], row['TCP_Yw'])
            T_tcp_final = T_accum @ T_tcp_off
            p_tcp = T_tcp_final[:3, 3]
            z_end = p_tcp + T_tcp_final[:3, 2] * 0.15 # Z축 방향 15cm 시각화

            # 🌟 데이터 정상 판명 시 저장
            last_valid_pts = pts
            last_valid_tcp = p_tcp
            last_valid_z_end = z_end

        except Exception as e:
            # 🌟 에러 발생 시 터미널에 출력하고 이전 데이터를 사용
            print(f"⚠️ Error at frame {frame} ({scenario}): {e}")
            pts = last_valid_pts
            p_tcp = last_valid_tcp
            z_end = last_valid_z_end

        # 그래프 업데이트
        line.set_data(pts[:, 0], pts[:, 1]); line.set_3d_properties(pts[:, 2])
        tool_line.set_data([pts[6,0], p_tcp[0]], [pts[6,1], p_tcp[1]])
        tool_line.set_3d_properties([pts[6,2], p_tcp[2]])
        z_axis.set_data([p_tcp[0], z_end[0]], [p_tcp[1], z_end[1]])
        z_axis.set_3d_properties([p_tcp[2], z_end[2]])

        target_dot.set_data([row['Target_X']], [row['Target_Y']])
        target_dot.set_3d_properties([row['Target_Z']])
        
        ax.set_title(f"Scenario: {scenario} | Frame: {frame}")
        return line, tool_line, z_axis, target_dot

    # blit=False로 설정하여 안정성을 높입니다.
    ani = FuncAnimation(fig, update, frames=len(df_sampled), interval=50, blit=False, repeat=False)
    plt.legend(loc='upper left'); plt.show()

if __name__ == "__main__":
    # 파일명이 맞는지 확인하세요
    animate_multi_robust('tcp_square_sim_data.csv')