import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# 1. 데이터 로드
try:
    df = pd.read_csv('full_picking_log.csv')
except FileNotFoundError:
    print("Error: 'full_picking_log.csv' 파일을 찾을 수 없습니다.")
    exit()

# 2. 그래프 설정 (3행 1열 구조)
fig, axes = plt.subplots(3, 1, figsize=(14, 18), sharex=True)
plt.subplots_adjust(hspace=0.3)

joints = [f'J{i}' for i in range(1, 7)]
colors = plt.cm.tab10(np.linspace(0, 1, 6)) # 조인트별 고유 색상

# --- (1) Position Plot ---
for i, j in enumerate(joints):
    axes[0].plot(df['Time'], df[f'{j}_p'], label=j, color=colors[i], linewidth=1.5)
axes[0].set_ylabel('Position (deg)', fontsize=12)
axes[0].set_title('Joint Positions', fontsize=14, fontweight='bold')
axes[0].legend(loc='upper right', ncol=6)
axes[0].grid(True, linestyle='--', alpha=0.6)

# --- (2) Velocity Plot ---
for i, j in enumerate(joints):
    axes[1].plot(df['Time'], df[f'{j}_v'], label=j, color=colors[i], linewidth=1.5)
axes[1].set_ylabel('Velocity (deg/s)', fontsize=12)
axes[1].set_title('Joint Velocities', fontsize=14, fontweight='bold')
axes[1].grid(True, linestyle='--', alpha=0.6)

# --- (3) Acceleration Plot (충격 및 불연속성 확인용) ---
for i, j in enumerate(joints):
    axes[2].plot(df['Time'], df[f'{j}_a'], label=j, color=colors[i], linewidth=1.2, alpha=0.8)
axes[2].set_ylabel('Acceleration (deg/s²)', fontsize=12)
axes[2].set_xlabel('Time (sec)', fontsize=12)
axes[2].set_title('Joint Accelerations (Check for E-Stop Spikes)', fontsize=14, fontweight='bold')
axes[2].grid(True, linestyle='--', alpha=0.6)

# --- (4) Step 구분선 및 텍스트 추가 ---
# Step이 바뀌는 인덱스를 찾아 수직선 표시
step_changes = df[df['Step'] != df['Step'].shift()].index.tolist()
for idx in step_changes:
    t = df.iloc[idx]['Time']
    step_name = df.iloc[idx]['Step']
    for ax in axes:
        ax.axvline(x=t, color='red', linestyle=':', linewidth=1.5, alpha=0.7)
        # 첫 번째 plot에만 Step 이름 표시
        if ax == axes[0]:
            ax.text(t, ax.get_ylim()[1], f' {step_name}', color='red', verticalalignment='bottom', fontweight='bold')

plt.suptitle('Robot Picking Cycle Analysis: Full Joint Dynamics', fontsize=18, y=0.98)
plt.tight_layout(rect=[0, 0.03, 1, 0.95])
plt.show()

# (선택) TCP 3D 경로 확인이 필요한 경우 별도 실행 가능
fig_3d = plt.figure(figsize=(10, 8))
ax_3d = fig_3d.add_subplot(111, projection='3d')
ax_3d.plot(df['TCP_X'], df['TCP_Y'], df['TCP_Z'], label='TCP Path')
ax_3d.set_xlabel('X (m)'); ax_3d.set_ylabel('Y (m)'); ax_3d.set_zlabel('Z (m)')
plt.show()