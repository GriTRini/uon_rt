import pandas as pd
import matplotlib.pyplot as plt

# 데이터 로드
df = pd.read_csv('robot_dynamics_data.csv')

# 그래프 설정 (3행 1열: Position, Velocity, Acceleration)
fig, axes = plt.subplots(3, 1, figsize=(12, 15), sharex=True)
plt.subplots_adjust(hspace=0.3)

colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b']
labels = [f'J{i}' for i in range(1, 7)]

# 1. Position Plot
for i in range(6):
    axes[0].plot(df['Time'], df[f'J{i+1}_p'], color=colors[i], label=labels[i])
axes[0].set_ylabel('Position (deg)')
axes[0].set_title('Joint Positions')
axes[0].grid(True, alpha=0.3)
axes[0].legend(loc='upper right', ncol=3)

# 2. Velocity Plot
for i in range(6):
    axes[1].plot(df['Time'], df[f'J{i+1}_v'], color=colors[i], label=labels[i])
axes[1].set_ylabel('Velocity (deg/s)')
axes[1].set_title('Joint Velocities')
axes[1].grid(True, alpha=0.3)

# 3. Acceleration Plot (Spike 확인용)
for i in range(6):
    axes[2].plot(df['Time'], df[f'J{i+1}_a'], color=colors[i], label=labels[i], alpha=0.7)
axes[2].set_ylabel('Acceleration (deg/s²)')
axes[2].set_xlabel('Time (sec)')
axes[2].set_title('Joint Accelerations (Check for Discontinuity)')
axes[2].grid(True, alpha=0.3)

# Mode 영역 표시 (AttrL, Stop, TrapJ 구간 구분)
for ax in axes:
    # Mode 2 (Stabilization) 구간을 회색 배경으로 표시
    stop_start = df[df['Mode'] == 2]['Time'].min()
    stop_end = df[df['Mode'] == 2]['Time'].max()
    ax.axvspan(stop_start, stop_end, color='gray', alpha=0.2, label='Stabilization')

plt.suptitle('Robot Joint Dynamics & Transition Analysis', fontsize=16)
plt.show()