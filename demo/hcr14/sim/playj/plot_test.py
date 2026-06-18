import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider

# --- 1. RPY 변환 행렬 생성 함수 ---
def get_transform_matrix(x, y, z, roll, pitch, yaw):
    rx = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
    ry = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
    rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
    
    R = rz @ ry @ rx
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T

# ==============================================================================
# [로봇 구조 및 오프셋 설정]
# ==============================================================================
LINK_OFFSETS = [
    get_transform_matrix(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),                     # Joint 1
    get_transform_matrix(0.0, 0.0, 0.207, 1.5708, 0.0, 0.0),                # Joint 2
    get_transform_matrix(-0.73, 0.0, 0.0, 0.0, 0.0, 0.0),                   # Joint 3
    get_transform_matrix(-0.5388, 0.0, 0.0, 0.0, 0.0, 0.0),                 # Joint 4
    get_transform_matrix(0.0, 0.0, 0.1847, 1.5708, 0.0, 0.0),               # Joint 5
    get_transform_matrix(0.0, 0.0, 0.1512, -1.5708, 0.0, 0.0)               # Joint 6
]

FLANGE_OFFSET = get_transform_matrix(0.0, 0.0, 0.1325, 0.0, 0.0, 0.0)
TCP_OFFSET = get_transform_matrix(0.0, 0.0, 0.25, 0.0, 0.0, 0.0) # 🌟 C++의 TCP 세팅과 동일하게 추가

# --- 2. 정기구학(FK) 계산 함수 (TCP 포함) ---
def get_joint_positions(angles_deg):
    angles_rad = np.radians(angles_deg)
    positions = [[0, 0, 0]]
    T_current = np.eye(4)
    
    for i in range(6):
        T_current = T_current @ LINK_OFFSETS[i]
        theta = angles_rad[i]
        Rz = np.eye(4)
        Rz[0, 0], Rz[0, 1] = np.cos(theta), -np.sin(theta)
        Rz[1, 0], Rz[1, 1] = np.sin(theta), np.cos(theta)
        
        T_current = T_current @ Rz
        positions.append(T_current[:3, 3].tolist())
        
    T_flange = T_current @ FLANGE_OFFSET
    positions.append(T_flange[:3, 3].tolist())
    
    # 🌟 TCP 노드 추가 (Flange에서 한 번 더 이동)
    T_tcp = T_flange @ TCP_OFFSET
    positions.append(T_tcp[:3, 3].tolist())
        
    return np.array(positions)

# --- 3. CSV 파싱 및 웨이포인트 추출 함수 ---
def parse_log_file(file_path):
    waypoints = []
    data_start_line = 0
    try:
        with open(file_path, 'r') as f:
            lines = f.readlines()
            for i, line in enumerate(lines):
                if line.startswith("# [LOG_DATA]"):
                    data_start_line = i + 1
                    break
                # 웨이포인트 데이터 파싱 (예: # 1,97.64,-90.0...)
                if line.startswith("# ") and not line.startswith("# [WAYPOINTS]") and not line.startswith("# WP"):
                    parts = line.strip().replace("# ", "").split(',')
                    if len(parts) == 8:
                        waypoints.append({
                            'idx': parts[0],
                            'angles': np.array([float(x) for x in parts[1:7]]),
                            'attrl': float(parts[7])
                        })
        
        # 실제 데이터프레임 로드
        df = pd.read_csv(file_path, skiprows=data_start_line) if data_start_line > 0 else pd.read_csv(file_path)
        return df, waypoints
    except Exception as e:
        print(f"❌ 데이터 로드 실패: {e}")
        return None, []

# --- 4. 통합 시각화 플로팅 함수 ---
def plot_robot_trajectory_3d(file_path="playj_test_log.csv"):
    df, waypoints = parse_log_file(file_path)
    if df is None: return
    
    print(f"✅ 데이터 로드 완료: {len(df)} 라인, 웨이포인트 {len(waypoints)}개 확인됨.")

    fig = plt.figure(figsize=(18, 9))
    gs = gridspec.GridSpec(2, 2, width_ratios=[1.2, 1], height_ratios=[1, 1])
    fig.subplots_adjust(bottom=0.15, wspace=0.15, hspace=0.3)

    # ------------------ [좌측 상단] X, Y, Z 좌표 변화 ------------------
    ax_xyz = fig.add_subplot(gs[0, 0])
    ax_xyz.plot(df['Time'], df['TCP_X'], label='TCP X', color='red', linewidth=2)
    ax_xyz.plot(df['Time'], df['TCP_Y'], label='TCP Y', color='green', linewidth=2)
    ax_xyz.plot(df['Time'], df['TCP_Z'], label='TCP Z', color='blue', linewidth=2)
    vline_xyz = ax_xyz.axvline(x=df['Time'].iloc[0], color='black', linestyle=':', linewidth=1.5)

    ax_xyz.set_title('Task Space: TCP Position (X, Y, Z)', fontsize=11, fontweight='bold')
    ax_xyz.set_ylabel('Position (m)')
    ax_xyz.grid(True, linestyle=':', alpha=0.6)
    ax_xyz.legend(loc='upper right', ncol=3, fontsize=8)

    # ------------------ [좌측 하단] 조인트 각도 변화 ------------------
    ax_joints = fig.add_subplot(gs[1, 0], sharex=ax_xyz)
    colors = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red', 'tab:purple', 'tab:brown']
    
    for i in range(1, 7):
        ax_joints.plot(df['Time'], df[f'J{i}'], label=f'J{i}', color=colors[i-1], linewidth=1.5)

    vline_joints = ax_joints.axvline(x=df['Time'].iloc[0], color='black', linestyle=':', linewidth=1.5)

    ax_joints.set_title('Joint Space: Angle Trajectories (Look for Smooth Blending)', fontsize=11, fontweight='bold')
    ax_joints.set_xlabel('Time (seconds)')
    ax_joints.set_ylabel('Angle (deg)')
    ax_joints.grid(True, linestyle=':', alpha=0.6)
    ax_joints.legend(loc='upper right', ncol=6, fontsize=8)

    # ------------------ [우측] 3D 로봇 모델 시각화 ------------------
    ax_3d = fig.add_subplot(gs[:, 1], projection='3d')

    # 전체 TCP 궤적 가이드라인 표시
    ax_3d.plot(df['TCP_X'], df['TCP_Y'], df['TCP_Z'], color='blue', linestyle='--', linewidth=1.5, alpha=0.4, label='Actual TCP Path')

    # 🌟 CSV에서 읽어온 웨이포인트(Target)를 3D 공간에 시각화
    for wp in waypoints:
        wp_positions = get_joint_positions(wp['angles'])
        target_tcp = wp_positions[-1] # TCP 위치 추출
        
        # 자홍색(Magenta) 별 모양으로 마킹
        ax_3d.scatter(target_tcp[0], target_tcp[1], target_tcp[2], color='magenta', s=200, marker='*', zorder=10)
        # 반경 정보 텍스트 표기
        ax_3d.text(target_tcp[0], target_tcp[1], target_tcp[2] + 0.05, f"WP {wp['idx']}\n(attrl: {wp['attrl']})", 
                   color='purple', fontsize=9, fontweight='bold', ha='center', va='bottom')

    # 최초 프레임 로봇 팔 시각화 (Base 부터 TCP까지 렌더링)
    initial_angles = df.iloc[0][['J1', 'J2', 'J3', 'J4', 'J5', 'J6']].values
    joints = get_joint_positions(initial_angles)
    
    line_arm, = ax_3d.plot(joints[:, 0], joints[:, 1], joints[:, 2], 
                           color='dodgerblue', linewidth=5, marker='o', markersize=6, markerfacecolor='orange', label='HCR-14 Arm')

    ax_3d.legend(fontsize=9, loc='upper left')
    ax_3d.set_title('PLAYJ Trajectory vs Target Waypoints', fontsize=13, fontweight='bold')
    ax_3d.set_xlabel('X (m)')
    ax_3d.set_ylabel('Y (m)')
    ax_3d.set_zlabel('Z (m)')

    ax_3d.set_xlim([-1.2, 1.2])
    ax_3d.set_ylim([-1.2, 1.2])
    ax_3d.set_zlim([-0.2, 1.5])
    ax_3d.view_init(elev=25, azim=135)

    # ------------------ [하단 UI] 슬라이더 ------------------
    ax_slider = plt.axes([0.15, 0.04, 0.7, 0.03])
    slider = Slider(ax_slider, 'Time Step', 0, len(df) - 1, valinit=0, valfmt='%0.0f')

    def update(val):
        idx = int(slider.val)
        
        angles = df.iloc[idx][['J1', 'J2', 'J3', 'J4', 'J5', 'J6']].values
        new_joints = get_joint_positions(angles)
        
        line_arm.set_data(new_joints[:, 0], new_joints[:, 1])
        line_arm.set_3d_properties(new_joints[:, 2])
        
        current_time = df['Time'].iloc[idx]
        vline_xyz.set_xdata([current_time, current_time])
        vline_joints.set_xdata([current_time, current_time])
        
        fig.canvas.draw_idle()

    slider.on_changed(update)
    plt.show()

if __name__ == "__main__":
    plot_robot_trajectory_3d("playj_test_log.csv")