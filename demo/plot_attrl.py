import pandas as pd
import matplotlib.pyplot as plt
import os

file_path = "/home/uon/uon_rt/demo/attrl_debug.csv"

if not os.path.exists(file_path):
    print(f"Error: {file_path} 파일이 없습니다.")
else:
    df = pd.read_csv(file_path)

    # 3행 1열 그래프 (X, Y, Z 위치 변화)
    fig, axs = plt.subplots(3, 1, figsize=(10, 12), sharex=True)
    fig.suptitle('Trajectory Generator Debug (Task Space - Attrl)', fontsize=16)

    # (1) X Position
    axs[0].plot(df['Time'], df['Curr_X'], color='blue', label='Current X')
    axs[0].set_ylabel('X [m]')
    axs[0].grid(True)
    axs[0].legend()

    # (2) Y Position
    axs[1].plot(df['Time'], df['Curr_Y'], color='green', label='Current Y')
    axs[1].set_ylabel('Y [m]')
    axs[1].grid(True)
    axs[1].legend()

    # (3) Z Position (가장 큰 변화가 있어야 함)
    axs[2].plot(df['Time'], df['Curr_Z'], color='red', label='Current Z')
    axs[2].plot(df['Time'], df['Goal_Z'], 'r--', alpha=0.5, label='Goal Z') # 목표선
    axs[2].set_ylabel('Z [m]')
    axs[2].set_xlabel('Time [s]')
    axs[2].grid(True)
    axs[2].legend()

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()