import pandas as pd
import matplotlib.pyplot as plt

# 1. CSV 데이터 로드
csv_path = "/home/uon/uon_rt/demo/trajectory_data.csv"
try:
    df = pd.read_csv(csv_path)
except FileNotFoundError:
    print(f"Error: {csv_path} 파일을 찾을 수 없습니다.")
    exit()

time = df['Time']

# 2. 그래프 그리기 (2개의 서브플롯: Joint / XYZ)
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10), sharex=True)

# --- Subplot 1: Joint Angles ---
colors = ['r', 'g', 'b', 'c', 'm', 'y']
for i in range(1, 7):
    ax1.plot(time, df[f'Q{i}'], label=f'Joint {i}', color=colors[i-1])

ax1.set_ylabel('Joint Angle (Degree)')
ax1.set_title('Joint Angles over Time')
ax1.legend(loc='upper right')
ax1.grid(True)

# 구간별 모드(Mode) 표시 (배경색 변경)
modes = df['Mode'].unique()
for mode in modes:
    mode_data = df[df['Mode'] == mode]
    if not mode_data.empty:
        start_t = mode_data['Time'].iloc[0]
        end_t = mode_data['Time'].iloc[-1]
        ax1.axvspan(start_t, end_t, alpha=0.1, label=f'Mode: {mode}')
        ax1.text(start_t + (end_t - start_t)/2, ax1.get_ylim()[1]*0.9, mode, 
                 horizontalalignment='center', fontsize=12, fontweight='bold', alpha=0.5)

# --- Subplot 2: End-effector Position (X, Y, Z) ---
ax2.plot(time, df['X'], label='X Position', color='r')
ax2.plot(time, df['Y'], label='Y Position', color='g')
ax2.plot(time, df['Z'], label='Z Position', color='b')

ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Position (m)')
ax2.set_title('End-effector Position (Task Space)')
ax2.legend(loc='upper right')
ax2.grid(True)

# 구간별 모드(Mode) 표시 (배경색 변경)
for mode in modes:
    mode_data = df[df['Mode'] == mode]
    if not mode_data.empty:
        start_t = mode_data['Time'].iloc[0]
        end_t = mode_data['Time'].iloc[-1]
        ax2.axvspan(start_t, end_t, alpha=0.1)

plt.tight_layout()
plt.show()