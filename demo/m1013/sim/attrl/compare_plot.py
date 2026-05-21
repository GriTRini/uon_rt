import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# ==========================================
# 1. 데이터 로드 및 기본 정보 출력
# ==========================================
file1_path = 'before.csv'
file2_path = 'after.csv'

df1 = pd.read_csv(file1_path)
df2 = pd.read_csv(file2_path)

print(f"[기본 정보]")
print(f"File 1 ({file1_path}) 총 데이터 수: {len(df1)}, 최대 시간: {df1['Time'].max():.3f}s")
print(f"File 2 ({file2_path}) 총 데이터 수: {len(df2)}, 최대 시간: {df2['Time'].max():.3f}s")
print("-" * 50)

# ==========================================
# 2. Settling Time 계산 함수
# ==========================================
def get_settle_time(df, tol=1e-5):
    pos_cols = ['X', 'Y', 'Z']
    diff = df[pos_cols].diff().abs().sum(axis=1)
    moving_idx = diff > tol
    if moving_idx.any():
        last_move_idx = moving_idx[::-1].idxmax()
        return df['Time'].iloc[last_move_idx]
    return 0.0

print(f"[안착 시간 (Movement Stopped)]")
print(f"File 1 안착 시간: {get_settle_time(df1):.3f} s")
print(f"File 2 안착 시간: {get_settle_time(df2):.3f} s")
print("-" * 50)

# ==========================================
# 3. 99% 도달 시간 및 총 이동 거리 계산
# ==========================================
start_pos1 = df1[['X', 'Y', 'Z']].iloc[0].values
end_pos1 = df1[['X', 'Y', 'Z']].iloc[-1].values
dist1 = np.linalg.norm(end_pos1 - start_pos1)

start_pos2 = df2[['X', 'Y', 'Z']].iloc[0].values
end_pos2 = df2[['X', 'Y', 'Z']].iloc[-1].values
dist2 = np.linalg.norm(end_pos2 - start_pos2)

df1['dist_from_start'] = np.linalg.norm(df1[['X', 'Y', 'Z']].values - start_pos1, axis=1)
df2['dist_from_start'] = np.linalg.norm(df2[['X', 'Y', 'Z']].values - start_pos2, axis=1)

t_99_1 = df1.loc[df1['dist_from_start'] >= 0.99 * dist1, 'Time'].iloc[0] if (df1['dist_from_start'] >= 0.99 * dist1).any() else df1['Time'].max()
t_99_2 = df2.loc[df2['dist_from_start'] >= 0.99 * dist2, 'Time'].iloc[0] if (df2['dist_from_start'] >= 0.99 * dist2).any() else df2['Time'].max()

print(f"[목표 도달 분석 (99% 기준)]")
print(f"File 1 총 이동 거리: {dist1:.4f} m | 99% 도달 시간: {t_99_1:.3f} s")
print(f"File 2 총 이동 거리: {dist2:.4f} m | 99% 도달 시간: {t_99_2:.3f} s")
print("-" * 50)

# ==========================================
# 4. TCP 이동 속도(Velocity) 계산
# ==========================================
df1['dt'] = df1['Time'].diff()
df2['dt'] = df2['Time'].diff()

df1['V_norm'] = np.sqrt(df1['X'].diff()**2 + df1['Y'].diff()**2 + df1['Z'].diff()**2) / df1['dt']
df2['V_norm'] = np.sqrt(df2['X'].diff()**2 + df2['Y'].diff()**2 + df2['Z'].diff()**2) / df2['dt']

print(f"[최고 이동 속도 (TCP Max Velocity)]")
print(f"File 1 최고 속도: {df1['V_norm'].max():.3f} m/s")
print(f"File 2 최고 속도: {df2['V_norm'].max():.3f} m/s")
print("-" * 50)

# ==========================================
# 5. 그래프 시각화 (Plots)
# ==========================================

# (1) XYZ 위치 궤적 비교
plt.figure(figsize=(15, 5))
for i, col in enumerate(['X', 'Y', 'Z']):
    plt.subplot(1, 3, i+1)
    # 🌟 수정: label을 파일명 변수로 동적 할당
    plt.plot(df1['Time'], df1[col], label=f'{file1_path}', alpha=0.8)
    plt.plot(df2['Time'], df2[col], label=f'{file2_path}', alpha=0.8, linestyle='--')
    plt.xlabel('Time (s)')
    plt.ylabel(f'{col} Position (m)')
    plt.title(f'TCP {col} Position')
    plt.legend()
    plt.grid(True)
plt.tight_layout()
plt.savefig('xyz_comparison.png')
plt.show()

# (2) 주요 관절(J1, J2, J3) 궤적 비교
plt.figure(figsize=(15, 5))
for i, col in enumerate(['J1', 'J2', 'J3']):
    plt.subplot(1, 3, i+1)
    # 🌟 수정: label을 파일명 변수로 동적 할당
    plt.plot(df1['Time'], df1[col], label=f'{file1_path}', alpha=0.8)
    plt.plot(df2['Time'], df2[col], label=f'{file2_path}', alpha=0.8, linestyle='--')
    plt.xlabel('Time (s)')
    plt.ylabel(f'{col} Angle (deg)')
    plt.title(f'{col} Joint Angle')
    plt.legend()
    plt.grid(True)
plt.tight_layout()
plt.savefig('joints_comparison.png')
plt.show()

# (3) TCP 속도(Velocity) 비교
plt.figure(figsize=(8, 5))
# 🌟 수정: label을 파일명 변수로 동적 할당
plt.plot(df1['Time'], df1['V_norm'], label=f'{file1_path}')
plt.plot(df2['Time'], df2['V_norm'], label=f'{file2_path}', linestyle='--')
plt.xlabel('Time (s)')
plt.ylabel('TCP Velocity (m/s)')
plt.title('TCP Velocity Comparison')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig('velocity_comparison.png')
plt.show()

print("분석 완료 및 그래프 저장 성공!")