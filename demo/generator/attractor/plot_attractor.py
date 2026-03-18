import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

def plot_ghost_debug(file_path='ghost_debug.csv'):
    # 1. 데이터 로드
    try:
        df = pd.read_csv(file_path)
    except FileNotFoundError:
        print(f"Error: {file_path} 파일을 찾을 수 없습니다.")
        return

    # 스타일 설정
    plt.style.use('seaborn-v0_8-muted')
    fig = plt.figure(figsize=(16, 10))

    # --- (1) 3D 공간 궤적 (직선성 확인) ---
    ax1 = fig.add_subplot(2, 2, 1, projection='3d')
    ax1.plot(df['X'], df['Y'], df['Z'], label='Ghost Path', color='blue', lw=2)
    ax1.scatter(df['X'].iloc[0], df['Y'].iloc[0], df['Z'].iloc[0], color='green', s=100, label='Start')
    ax1.scatter(df['X'].iloc[-1], df['Y'].iloc[-1], df['Z'].iloc[-1], color='red', marker='*', s=200, label='Goal')
    
    ax1.set_title("3D Cartesian Trajectory (Ghost Only)", fontsize=13, fontweight='bold')
    ax1.set_xlabel("X [m]")
    ax1.set_ylabel("Y [m]")
    ax1.set_zlabel("Z [m]")
    ax1.legend()

    # --- (2) 시간별 좌표 변화 (수렴성 확인) ---
    ax2 = fig.add_subplot(2, 2, 2)
    ax2.plot(df['Time'], df['X'], label='X', alpha=0.8)
    ax2.plot(df['Time'], df['Y'], label='Y', alpha=0.8)
    ax2.plot(df['Time'], df['Z'], label='Z', alpha=0.8)
    ax2.set_title("Position Components over Time", fontsize=13)
    ax2.set_ylabel("Position [m]")
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    # --- (3) 속도 벡터 크기 (S-Curve 확인) ---
    # 속도 성분으로부터 전체 속도(Norm) 계산
    speed = np.sqrt(df['Vel_X']**2 + df['Vel_Y']**2 + df['Vel_Z']**2)
    ax3 = fig.add_subplot(2, 2, 3)
    ax3.plot(df['Time'], speed, color='teal', label='Total Speed')
    ax3.set_title("Velocity Profile (Magnitude)", fontsize=13)
    ax3.set_xlabel("Time [s]")
    ax3.set_ylabel("Speed [m/s]")
    ax3.fill_between(df['Time'], speed, color='teal', alpha=0.1)
    ax3.grid(True, alpha=0.3)

    # --- (4) 오차 감소 (Log Scale - 정밀도 확인) ---
    ax4 = fig.add_subplot(2, 2, 4)
    ax4.plot(df['Time'], df['Err_Norm'], color='red', label='Distance to Goal')
    ax4.set_yscale('log') # 오차 수렴은 로그 스케일이 가장 정확함
    ax4.set_title("Distance Error (Log Scale)", fontsize=13)
    ax4.set_xlabel("Time [s]")
    ax4.set_ylabel("Error [m]")
    ax4.axhline(y=0.001, color='black', linestyle='--', alpha=0.5, label='1mm Limit')
    ax4.legend()
    ax4.grid(True, which='both', alpha=0.3)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    plot_ghost_debug()