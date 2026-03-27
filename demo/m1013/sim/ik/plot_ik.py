import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

def plot_ik_3d_convergence(file_path='ik_convergence_test.csv'):
    try:
        df = pd.read_csv(file_path)
    except FileNotFoundError:
        print(f"Error: {file_path}를 찾을 수 없습니다.")
        return

    plt.style.use('seaborn-v0_8-muted')
    fig = plt.figure(figsize=(18, 10))

    # --- (1) 3D Cartesian Path (가장 중요한 궤적 확인) ---
    ax1 = fig.add_subplot(1, 2, 1, projection='3d')
    
    # IK가 찾아가는 경로 (파란 실선 + 점)
    ax1.plot(df['X'], df['Y'], df['Z'], 'b-o', markersize=3, label='IK Solver Path', alpha=0.6)
    
    # 시작점 (녹색)
    ax1.scatter(df['X'].iloc[0], df['Y'].iloc[0], df['Z'].iloc[0], color='green', s=100, label='Start')
    
    # 목표점 (빨간 별)
    ax1.scatter(df['Goal_X'].iloc[0], df['Goal_Y'].iloc[0], df['Goal_Z'].iloc[0], 
                color='red', marker='*', s=200, label='Target Goal', zorder=10)

    ax1.set_title("3D IK Convergence Path", fontsize=14, fontweight='bold')
    ax1.set_xlabel("X [m]")
    ax1.set_ylabel("Y [m]")
    ax1.set_zlabel("Z [m]")
    ax1.legend()

    # --- (2) 오차 감소 그래프 (수렴 성능 확인) ---
    ax2 = fig.add_subplot(2, 2, 2)
    ax2.plot(df['Iteration'], df['P_Err'] * 1000, 'r-x', label='Position Error (mm)')
    ax2.set_yscale('log')
    ax2.set_title("Position Error Convergence (Log Scale)", fontsize=12)
    ax2.set_ylabel("Error [mm]")
    ax2.grid(True, which='both', alpha=0.3)
    ax2.legend()

    # --- (3) 관절 각도 변화 ---
    ax3 = fig.add_subplot(2, 2, 4)
    for i in range(1, 7):
        ax3.plot(df['Iteration'], df[f'Q{i}'], label=f'Joint {i}')
    ax3.set_title("Joint Angles Change", fontsize=12)
    ax3.set_xlabel("Iteration")
    ax3.set_ylabel("Degrees")
    ax3.legend(ncol=3, fontsize='small')
    ax3.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    plot_ik_3d_convergence()