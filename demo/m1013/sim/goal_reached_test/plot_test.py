import pandas as pd
import matplotlib.pyplot as plt

def analyze_thresholds(file_path):
    df = pd.read_csv(file_path)
    
    fig, axes = plt.subplots(3, 1, figsize=(12, 12), sharex=True)
    
    # 1. 관절 오차와 임계값 (0.1 deg)
    axes[0].plot(df['Time'], df['Q_Err'], label='Joint Error (Enorm)', color='blue')
    axes[0].axhline(y=0.1, color='red', linestyle='--', label='Threshold (0.1)')
    axes[0].set_ylabel('Error (deg)')
    axes[0].set_title('Joint Angle Convergence')
    axes[0].set_yscale('log')
    axes[0].legend()

    # 2. 위치 오차와 임계값 (0.002 m = 2mm)
    axes[1].plot(df['Time'], df['Pos_Err'], label='Position Error', color='green')
    axes[1].axhline(y=0.002, color='red', linestyle='--', label='Threshold (2mm)')
    axes[1].set_ylabel('Error (m)')
    axes[1].set_title('Cartesian Position Convergence')
    axes[1].set_yscale('log')
    axes[1].legend()

    # 3. 속도 오차와 임계값 (0.5 deg/s)
    axes[2].plot(df['Time'], df['dQ_Err'], label='Velocity Error', color='orange')
    axes[2].axhline(y=0.5, color='red', linestyle='--', label='Threshold (0.5)')
    axes[2].set_ylabel('Vel Error (deg/s)')
    axes[2].set_title('Velocity Convergence (Stability)')
    axes[2].legend()

    # Goal Reached 시점 표시
    reached_times = df[df['GoalReached'] == 1]['Time']
    for ax in axes:
        for t in reached_times:
            ax.axvline(x=t, color='black', alpha=0.3)

    plt.tight_layout()
    plt.show()

analyze_thresholds('goal_reached_fail_test.csv')