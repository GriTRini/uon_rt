import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import sys

# --- M1013 기구학 파라미터 ---
def get_transform(idx, q_deg):
    q = np.radians(q_deg)
    def m(x, y, z, r, p, yaw):
        Rx = np.array([[1,0,0],[0,np.cos(r),-np.sin(r)],[0,np.sin(r),np.cos(r)]])
        Ry = np.array([[np.cos(p),0,np.sin(p)],[0,1,0],[-np.sin(p),0,np.cos(p)]])
        Rz = np.array([[np.cos(yaw),-np.sin(yaw),0],[np.sin(yaw),np.cos(yaw),0],[0,0,1]])
        T = np.eye(4); T[:3,:3] = Rz @ Ry @ Rx; T[:3,3] = [x,y,z]
        return T
    
    # M1013 전용 파라미터 적용
    params = [
        (0.0,  0.0,    0.1525, 0.0,      0.0,       0.0), 
        (0.0,  0.0345, 0.0,    0.0,      -np.pi/2, -np.pi/2), 
        (0.62, 0.0,    0.0,    0.0,      0.0,       np.pi/2),
        (0.0, -0.559,  0.0,    np.pi/2,  0.0,       0.0), 
        (0.0,  0.0,    0.0,   -np.pi/2,  0.0,       0.0), 
        (0.0, -0.121,  0.0,    np.pi/2,  0.0,       0.0)
    ]
    p = params[idx]
    T_origin = m(p[0], p[1], p[2], p[3], p[4], p[5])
    T_rot = np.eye(4); T_rot[:3,:3] = [[np.cos(q),-np.sin(q),0], [np.sin(q),np.cos(q),0], [0,0,1]]
    return T_origin @ T_rot

def animate_m1013_extreme(file_path):
    try:
        df = pd.read_csv(file_path)
    except FileNotFoundError:
        print(f"❌ 에러: '{file_path}' 파일을 찾을 수 없습니다.")
        sys.exit()

    df_sampled = df.iloc[::40, :].reset_index(drop=True)

    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')
    
    # 로봇이 미친듯이 뻗어나가므로 축을 넉넉하게 잡습니다.
    limit = 1.2
    ax.set_xlim(-limit, limit); ax.set_ylim(-limit, limit); ax.set_zlim(0, limit)
    ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
    ax.set_title('M1013 attrl Extreme Test (Singularity & Wobble Check)')
    
    line, = ax.plot([], [], [], 'o-', lw=6, color='#2980b9', label='M1013 Arm')
    target_dot, = ax.plot([], [], [], 'r*', markersize=15, label='Target Goal')
    path_line, = ax.plot([], [], [], '-', lw=2, color='#2ca02c', label='Flange Path')

    history_x, history_y, history_z = [], [], []

    def update(frame):
        row = df_sampled.iloc[frame]
        angles = row[['J1', 'J2', 'J3', 'J4', 'J5', 'J6']].values.astype(float)
        scenario = row['Scenario']
        
        # 기구학 계산 (TCP 없음, Flange 끝단까지만)
        T_accum = np.eye(4); pts = [np.zeros(3)]
        for i in range(6):
            T_accum = T_accum @ get_transform(i, angles[i])
            pts.append(T_accum[:3, 3])
        pts = np.array(pts)
        
        p_flange = pts[6] # 손목 끝단
        
        # 궤적 누적
        history_x.append(p_flange[0])
        history_y.append(p_flange[1])
        history_z.append(p_flange[2])

        # 그래프 업데이트
        line.set_data(pts[:, 0], pts[:, 1]); line.set_3d_properties(pts[:, 2])
        path_line.set_data(history_x, history_y); path_line.set_3d_properties(history_z)
        target_dot.set_data([float(row['Target_X'])], [float(row['Target_Y'])])
        target_dot.set_3d_properties([float(row['Target_Z'])])
        
        ax.set_title(f"Scenario: {scenario} | Frame: {frame}")
        return line, target_dot, path_line

    ani = FuncAnimation(fig, update, frames=len(df_sampled), interval=30, blit=False, repeat=False)
    plt.legend(loc='upper right')
    plt.show()

if __name__ == "__main__":
    animate_m1013_extreme('flange_fig8_sim_data.csv')