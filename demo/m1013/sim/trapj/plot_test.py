import pandas as pd
import matplotlib.pyplot as plt

# 1. 데이터 불러오기
file_path = 'joint_dynamics_test.csv'  # 생성된 CSV 파일명
try:
    df = pd.read_csv(file_path)
except FileNotFoundError:
    print(f"Error: {file_path} 파일을 찾을 수 없습니다. C++ 프로그램을 먼저 실행하세요.")
    exit()

# 2. 그래프 설정 (3행 1열 구조: 위치, 속도, 가속도)
fig, axes = plt.subplots(3, 1, figsize=(12, 15), sharex=True)
plt.subplots_adjust(hspace=0.3)

joints = [f'J{i}' for i in range(1, 7)]
colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b']

# --- (1) Position Plot ---
for i, joint in enumerate(joints):
    axes[0].plot(df['Time'], df[f'{joint}_Pos'], label=f'{joint}', color=colors[i], linewidth=1.5)
axes[0].set_ylabel('Position (deg)', fontsize=12)
axes[0].set_title('Joint Positions', fontsize=14, fontweight='bold')
axes[0].legend(loc='upper right', ncol=3)
axes[0].grid(True, linestyle='--', alpha=0.7)

# --- (2) Velocity Plot ---
for i, joint in enumerate(joints):
    axes[1].plot(df['Time'], df[f'{joint}_Vel'], label=f'{joint}', color=colors[i], linewidth=1.5)
axes[1].set_ylabel('Velocity (deg/s)', fontsize=12)
axes[1].set_title('Joint Velocities (Trapezoidal Profile)', fontsize=14, fontweight='bold')
axes[1].legend(loc='upper right', ncol=3)
axes[1].grid(True, linestyle='--', alpha=0.7)

# --- (3) Acceleration Plot ---
for i, joint in enumerate(joints):
    axes[2].plot(df['Time'], df[f'{joint}_Acc'], label=f'{joint}', color=colors[i], linewidth=1.2, alpha=0.8)
axes[2].set_ylabel('Acceleration (deg/s²)', fontsize=12)
axes[2].set_xlabel('Time (sec)', fontsize=12)
axes[2].set_title('Joint Accelerations', fontsize=14, fontweight='bold')
axes[2].legend(loc='upper right', ncol=3)
axes[2].grid(True, linestyle='--', alpha=0.7)

# 결과 출력
print(f"Plotting data from {len(df)} samples...")
plt.suptitle(f'Robot Joint Dynamics Analysis ({file_path})', fontsize=16, y=0.95)
plt.show()

# (선택 사항) 이미지로 저장하고 싶을 경우 아래 주석 해제
# plt.savefig('trajectory_plot.png', dpi=300)