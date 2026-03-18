import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

def plot_attrl_comparison(file_path='attrl_debug_report.csv'):
    # 1. 데이터 로드
    try:
        df = pd.read_csv(file_path)
    except FileNotFoundError:
        print(f"Error: {file_path} 파일을 찾을 수 없습니다. C++ 디버깅 프로그램을 먼저 실행하세요.")
        return

    # 스타일 설정
    plt.style.use('seaborn-v0_8-muted')
    fig = plt.figure(figsize=(18, 11))

    # --- (1) 3D 공간 궤적 비교 (직선성 및 추종 오차 확인) ---
    ax1 = fig.add_subplot(2, 2, 1, projection='3d')
    
    # 유령의 궤적 (점선)
    ax1.plot(df['Att_X'], df['Att_Y'], df['Att_Z'], 'k--', label='Ghost Trajectory (Attractor)', alpha=0.6, lw=1.5)
    # 로봇의 실제 궤적 (실선)
    ax1.plot(df['X'], df['Y'], df['Z'], 'b-', label='Actual Robot Path', lw=2.5)
    
    # 시작점과 목표점 표시
    ax1.scatter(df['Att_X'].iloc[0], df['Att_Y'].iloc[0], df['Att_Z'].iloc[0], color='green', s=100, label='Start')
    ax1.scatter(df['Goal_X'].iloc[-1], df['Goal_Y'].iloc[-1], df['Goal_Z'].iloc[-1], color='red', marker='*', s=200, label='Goal')
    
    ax1.set_title("3D Trajectory Comparison\n(Ghost vs Actual)", fontsize=14, fontweight='bold')
    ax1.set_xlabel("X [m]")
    ax1.set_ylabel("Y [m]")
    ax1.set_zlabel("Z [m]")
    ax1.legend()

    # --- (2) 시간별 추종 지연 확인 (X축 기준 예시) ---
    ax2 = fig.add_subplot(2, 2, 2)
    ax2.plot(df['Time'], df['Att_X'], 'k--', label='Ghost X', alpha=0.7)
    ax2.plot(df['Time'], df['X'], 'b-', label='Actual X', lw=2)
    ax2.set_title("Time Sync Check (X-axis Profile)", fontsize=13)
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Position [m]")
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    # --- (3) 속도 프로파일 (Velocity Norm) ---
    ax3 = fig.add_subplot(2, 2, 3)
    ax3.plot(df['Time'], df['Vel_Norm'], color='teal', label='Robot Velocity', lw=2)
    ax3.set_title("Velocity Magnitude (Linear Speed)", fontsize=13)
    ax3.set_xlabel("Time [s]")
    ax2.set_ylabel("Speed [m/s]")
    ax3.fill_between(df['Time'], df['Vel_Norm'], color='teal', alpha=0.1)
    ax3.grid(True, alpha=0.3)

    # --- (4) 추종 오차 (Follow Error: 유령과 로봇 사이의 거리) ---
    # CSV에 P_Err가 이미 있을 수 있지만, 여기서는 실시간 '추종' 오차를 직접 계산합니다.
    follow_err = np.sqrt((df['Att_X'] - df['X'])**2 + (df['Att_Y'] - df['Y'])**2 + (df['Att_Z'] - df['Z'])**2)
    
    ax4 = fig.add_subplot(2, 2, 4)
    ax4.plot(df['Time'], follow_err * 1000, color='red', label='Follow Error (mm)')
    ax4.set_title("Tracking Error (Ghost to Actual Distance)", fontsize=13)
    ax4.set_xlabel("Time [s]")
    ax4.set_ylabel("Error [mm]")
    ax4.axhline(y=1.0, color='black', linestyle=':', label='1mm Tolerance')
    ax4.legend()
    ax4.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('attrl_performance_report.png', dpi=300)
    plt.show()

if __name__ == "__main__":
    # 파일 이름이 C++에서 저장한 파일명과 일치하는지 확인하세요.
    plot_attrl_comparison('attrl_debug_report.csv')