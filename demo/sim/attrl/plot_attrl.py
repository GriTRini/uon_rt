import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# 1. 데이터 로드
try:
    df = pd.read_csv('robot_joint_data.csv')
except FileNotFoundError:
    print("CSV 파일을 찾을 수 없습니다. C++ 프로그램을 먼저 실행하세요.")
    exit()

# 2. 3D 말단(Flange) 궤적 플롯
fig = plt.figure(figsize=(12, 5))
ax1 = fig.add_subplot(121, projection='3d')
ax1.plot(df['X'], df['Y'], df['Z'], label='Flange Path', color='b', lw=2)
ax1.scatter(df['X'].iloc[0], df['Y'].iloc[0], df['Z'].iloc[0], color='g', s=50, label='Start')
ax1.set_title('3D Cartesian Path (m)')
ax1.set_xlabel('X')
ax1.set_ylabel('Y')
ax1.set_zlabel('Z')
ax1.legend()

# 3. 조인트 각도 변화 플롯
ax2 = fig.add_subplot(122)
for i in range(1, 7):
    ax2.plot(df['Time'], df[f'J{i}'], label=f'Joint {i}')
ax2.set_title('Joint Angles vs Time (deg)')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Angle (deg)')
ax2.grid(True)
ax2.legend(loc='upper right', fontsize='small')

plt.tight_layout()
plt.show()