import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# --- M1013 로봇 모델 기구학 (제공된 치수 적용) ---
def get_link_transform(idx, q_deg):
    q = np.radians(q_deg)
    def m(x, y, z, r, p, yaw):
        Rx = np.array([[1,0,0],[0,np.cos(r),-np.sin(r)],[0,np.sin(r),np.cos(r)]])
        Ry = np.array([[np.cos(p),0,np.sin(p)],[0,1,0],[-np.sin(p),0,np.cos(p)]])
        Rz = np.array([[np.cos(yaw),-np.sin(yaw),0],[np.sin(yaw),np.cos(yaw),0],[0,0,1]])
        T = np.eye(4); T[:3,:3] = Rz @ Ry @ Rx; T[:3,3] = [x,y,z]
        return T

    # 사용자 제공 m1013::joints 파라미터
    params = [
        (0, 0, 0.1525, 0, 0, 0),               # J1 Offset
        (0, 0.0345, 0, 0, -np.pi/2, -np.pi/2),  # J2 Offset
        (0.62, 0, 0, 0, 0, np.pi/2),           # J3 Offset
        (0, -0.559, 0, np.pi/2, 0, 0),         # J4 Offset
        (0, 0, 0, -np.pi/2, 0, 0),              # J5 Offset
        (0, -0.121, 0, np.pi/2, 0, 0)          # J6 Offset
    ]
    
    p = params[idx]
    T_link = m(p[0], p[1], p[2], p[3], p[4], p[5])
    T_rot = np.eye(4)
    T_rot[:3,:3] = [[np.cos(q), -np.sin(q), 0], [np.sin(q), np.cos(q), 0], [0, 0, 1]]
    return T_link @ T_rot

def run_visualizer(csv_file):
    df = pd.read_csv(csv_file)
    df_sampled = df.iloc[::40, :].reset_index(drop=True) # 40ms 간격 샘플링

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim(-0.6, 1.0); ax.set_ylim(-0.8, 0.8); ax.set_zlim(0, 1.2)
    
    line, = ax.plot([], [], [], 'o-', lw=5, color='#2c3e50', label='M1013 Skeleton')
    target_path, = ax.plot(df['X'], df['Y'], df['Z'], 'r--', lw=1, alpha=0.5, label='Target Path')
    
    def update(frame):
        row = df_sampled.iloc[frame]
        angles = [row['J1'], row['J2'], row['J3'], row['J4'], row['J5'], row['J6']]
        
        T_accum = np.eye(4)
        points = [np.zeros(3)] # Base
        for i in range(6):
            T_accum = T_accum @ get_link_transform(i, angles[i])
            points.append(T_accum[:3, 3])
        
        pts = np.array(points)
        line.set_data(pts[:, 0], pts[:, 1])
        line.set_3d_properties(pts[:, 2])
        ax.set_title(f"M1013 3D Tracking | Time: {row['Time']:.2f}s")
        return line,

    ani = FuncAnimation(fig, update, frames=len(df_sampled), interval=40, blit=True)
    plt.legend(); plt.show()

if __name__ == "__main__":
    run_visualizer('robot_joint_data.csv')