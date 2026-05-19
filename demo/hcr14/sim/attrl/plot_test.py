import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider

# --- 1. RPY 변환 행렬 생성 함수 ---
def get_transform_matrix(x, y, z, roll, pitch, yaw):
    """xyzrpy를 4x4 동차 변환 행렬(Homogeneous Transformation Matrix)로 변환"""
    # Roll (X축 회전)
    rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    # Pitch (Y축 회전)
    ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    # Yaw (Z축 회전)
    rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    
    # RPY 회전 (Z * Y * X 순서)
    R = rz @ ry @ rx
    
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T

# ==============================================================================
# 🌟 [한화 HCR-14 로봇 전용 기하학 구조 설정]
# C++ 코드의 xyzrpy(x, y, z, roll, pitch, yaw) 오프셋을 변환 행렬로 매핑 완료
# ==============================================================================
LINK_OFFSETS = [
    # Joint 1: Base -> Link1
    get_transform_matrix(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    
    # Joint 2: Link1 -> Link2
    get_transform_matrix(0.0, 0.0, 0.207, 1.5708, 0.0, 0.0),
    
    # Joint 3: Link2 -> Link3
    get_transform_matrix(-0.73, 0.0, 0.0, 0.0, 0.0, 0.0),
    
    # Joint 4: Link3 -> Link4
    get_transform_matrix(-0.5388, 0.0, 0.0, 0.0, 0.0, 0.0),
    
    # Joint 5: Link4 -> Link5
    get_transform_matrix(0.0, 0.0, 0.1847, 1.5708, 0.0, 0.0),
    
    # Joint 6: Link5 -> Link6
    get_transform_matrix(0.0, 0.0, 0.1512, -1.5708, 0.0, 0.0)
]

# --- 2. 정기구학(Forward Kinematics) 계산 함수 ---
def get_joint_positions(angles_deg):
    """
    관절 각도(degree)를 입력받아 각 링크의 절대 좌표 반환
    """
    angles_rad = np.radians(angles_deg)
    
    positions = [[0, 0, 0]] # Base 좌표
    T_current = np.eye(4)
    
    for i in range(6):
        # 1. 이전 조인트에서 현재 조인트까지의 기본 오프셋 적용
        T_current = T_current @ LINK_OFFSETS[i]
        
        # 2. 현재 조인트의 회전 각도(Z축 기준, Eigen::Vector3d::UnitZ()) 적용
        theta = angles_rad[i]
        Rz = np.eye(4)
        Rz[0, 0] = np.cos(theta)
        Rz[0, 1] = -np.sin(theta)
        Rz[1, 0] = np.sin(theta)
        Rz[1, 1] = np.cos(theta)
        
        T_current = T_current @ Rz
        
        # 변환된 위치(X, Y, Z) 저장
        positions.append(T_current[:3, 3].tolist())
        
    return np.array(positions)

# --- 3. 통합 시각화 플로팅 함수 ---
def plot_robot_trajectory_3d(file_path="reach_and_lift_test.csv"):
    try:
        df = pd.read_csv(file_path)
        df = df.sort_values(by='Time').reset_index(drop=True)
        print(f"✅ 데이터를 불러왔습니다. 총 {len(df)} 라인")
    except Exception as e:
        print(f"❌ 데이터 로드 실패: {e}\n경로를 확인해주세요.")
        return

    # 그래프 창 전체 크기 및 여백 설정 (1행 2열 배치)
    fig = plt.figure(figsize=(16, 8))
    fig.subplots_adjust(bottom=0.22, wspace=0.25)

    # ------------------ [좌측 차트] 시간에 따른 X, Y, Z 좌표 변화 그래프 ------------------
    ax_xyz = fig.add_subplot(121)
    
    # 시간(Time) 데이터에 따른 각각의 직교 좌표 플로팅
    ax_xyz.plot(df['Time'], df['X'], label='Actual X (m)', color='red', linewidth=2)
    ax_xyz.plot(df['Time'], df['Y'], label='Actual Y (m)', color='green', linewidth=2)
    ax_xyz.plot(df['Time'], df['Z'], label='Actual Z (m)', color='blue', linewidth=2)
    
    # 슬라이더 상의 현재 재생 시점을 동기화하여 보여줄 세로 점선 커서(Vline) 정의
    vline = ax_xyz.axvline(x=df['Time'].iloc[0], color='black', linestyle='--', linewidth=1.5, label='Current Time')

    ax_xyz.set_title('TCP Position Trajectory (X, Y, Z)', fontsize=13, fontweight='bold')
    ax_xyz.set_xlabel('Time (seconds)', fontsize=11)
    ax_xyz.set_ylabel('Position (meters)', fontsize=11)
    ax_xyz.grid(True, linestyle=':', alpha=0.6)
    ax_xyz.legend(loc='upper right')

    # ------------------ [우측 차트] 한화 HCR-14 3D 로봇 모델 시각화 ------------------
    ax_3d = fig.add_subplot(122, projection='3d')

    # 전체 TCP 궤적 가이드라인 표시
    ax_3d.plot(df['X'], df['Y'], df['Z'], color='gray', linestyle='--', linewidth=1, alpha=0.5, label='TCP Trajectory')

    # 최초 프레임의 데이터로 초기 로봇 팔 뼈대라인 생성
    initial_angles = df.iloc[0][['J1', 'J2', 'J3', 'J4', 'J5', 'J6']].values
    joints = get_joint_positions(initial_angles)
    
    # 뼈대 선(Line)과 관절 노드(Marker)
    line_arm, = ax_3d.plot(joints[:, 0], joints[:, 1], joints[:, 2], 
                           color='dodgerblue', linewidth=5, marker='o', markersize=8, markerfacecolor='orange', label='HCR-14 Arm')

    ax_3d.legend()
    ax_3d.set_title('Hanwha HCR-14 Kinematics Simulation', fontsize=13, fontweight='bold')
    ax_3d.set_xlabel('X (m)')
    ax_3d.set_ylabel('Y (m)')
    ax_3d.set_zlabel('Z (m)')

    # 🌟 [스케일 수정] HCR-14 로봇의 대형 가동 범위를 고려한 가상 공간 축 고정
    ax_3d.set_xlim([-1.5, 1.5])
    ax_3d.set_ylim([-1.5, 1.5])
    ax_3d.set_zlim([-0.2, 1.6])
    ax_3d.view_init(elev=25, azim=135) # 한화 로봇 링크 방향에 맞춘 최적 뷰 카메라 앵글

    # ------------------ [하단 UI] 인터랙티브 타임라인 슬라이더 ------------------
    ax_slider = plt.axes([0.2, 0.08, 0.6, 0.03])
    slider = Slider(ax_slider, 'Time Step', 0, len(df) - 1, valinit=0, valfmt='%0.0f')

    # 슬라이더 바가 드래그되거나 클릭되어 변경될 때 작동하는 실시간 동기화 함수
    def update(val):
        idx = int(slider.val)
        
        # 1. 우측 3D 차트: 슬라이더 인덱스 프레임의 관절 각도를 추출해 정기구학 재연산 후 갱신
        angles = df.iloc[idx][['J1', 'J2', 'J3', 'J4', 'J5', 'J6']].values
        new_joints = get_joint_positions(angles)
        line_arm.set_data(new_joints[:, 0], new_joints[:, 1])
        line_arm.set_3d_properties(new_joints[:, 2])
        
        # 2. 좌측 2D 차트: 현재 프레임의 시간(Time) 값을 읽어 세로선 커서 위치 변경
        current_time = df['Time'].iloc[idx]
        vline.set_xdata([current_time, current_time])
        
        # UI 레이아웃 리프레시 명령
        fig.canvas.draw_idle()

    # 이벤트 바인딩 후 루프 활성화
    slider.on_changed(update)
    plt.show()

if __name__ == "__main__":
    # 제어 테스트 결과가 들어있는 CSV 파일 경로를 지정하세요.
    plot_robot_trajectory_3d("reach_and_lift_test.csv")