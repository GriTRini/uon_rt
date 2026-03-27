import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# --- 기구학 및 변환 함수 (기존과 동일) ---
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

def animate_flange_tracking(file_path):
    df = pd.read_csv(file_path)
    # 디버깅용: 샘플링 간격을 80정도로 넓혀서 렌더링 속도 향상
    df_sampled = df.iloc[::80, :].reset_index(drop=True)

    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')
    
    # 작업 영역 설정
    ax.set_xlim(-0.4, 1.0)
    ax.set_ylim(-0.6, 0.6)
    ax.set_zlim(0, 1.2)
    ax.set_xlabel('X [m]'); ax.set_ylabel('Y [m]'); ax.set_zlabel('Z [m]')
    
    # 시각화 객체 초기화 (TCP 제거, 로봇 Arm과 타겟만 남김)
    line, = ax.plot([], [], [], 'o-', lw=6, color='#34495e', label='Robot Arm (Flange)')
    flange_z_axis, = ax.plot([], [], [], 'b-', lw=2, label='Flange Z-Axis') # 자세 변화를 보기 위한 축
    target_dot, = ax.plot([], [], [], 'r*', markersize=15, label='Moving Target')
    
    # 과거 궤적을 그리기 위한 리스트
    flange_path_x, flange_path_y, flange_path_z = [], [], []
    flange_trace, = ax.plot([], [], [], 'k--', lw=1, alpha=0.5, label='Flange Path')

    def update(frame):
        row = df_sampled.iloc[frame]
        angles = row[['J1', 'J2', 'J3', 'J4', 'J5', 'J6']].values
        scenario = row['Scenario']
        
        # 1. 로봇 본체 계산 (FK)
        T_accum = np.eye(4)
        pts = [np.zeros(3)] # Base 좌표
        
        for i in range(6):
            T_joint = get_transform(i, angles[i])
            T_accum = T_accum @ T_joint
            pts.append(T_accum[:3, 3])
            
        pts = np.array(pts)
        
        # 2. Flange(J6 끝단)의 Z축 방향 시각화 (자세 틸팅 확인용)
        # T_accum은 J6까지의 변환 행렬. Z축(3번째 열) 벡터를 추출
        flange_pos = pts[-1]
        flange_z_dir = T_accum[:3, 2]
        z_end = flange_pos + flange_z_dir * 0.15 # 15cm 길이로 그림

        # 3. 궤적(Trace) 업데이트
        flange_path_x.append(flange_pos[0])
        flange_path_y.append(flange_pos[1])
        flange_path_z.append(flange_pos[2])
        flange_trace.set_data(flange_path_x, flange_path_y)
        flange_trace.set_3d_properties(flange_path_z)

        # 4. 그래프 데이터 갱신
        line.set_data(pts[:, 0], pts[:, 1])
        line.set_3d_properties(pts[:, 2])
        
        flange_z_axis.set_data([flange_pos[0], z_end[0]], [flange_pos[1], z_end[1]])
        flange_z_axis.set_3d_properties([flange_pos[2], z_end[2]])

        target_dot.set_data([row['Target_X']], [row['Target_Y']])
        target_dot.set_3d_properties([row['Target_Z']])
        
        ax.set_title(f"Scenario: {scenario} | Frame: {frame}/{len(df_sampled)-1}\nTarget: ({row['Target_X']:.2f}, {row['Target_Y']:.2f}, {row['Target_Z']:.2f})")
        
        return line, flange_z_axis, target_dot, flange_trace

    ani = FuncAnimation(fig, update, frames=len(df_sampled), interval=50, blit=False, repeat=False)
    plt.legend(loc='upper left')
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # 방금 생성한 CSV 파일 이름과 정확히 일치해야 합니다.
    animate_flange_tracking('flange_fig8_sim_data_1.csv')