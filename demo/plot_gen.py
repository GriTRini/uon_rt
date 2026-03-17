import pandas as pd
import matplotlib.pyplot as plt
import os

# 1. CSV 파일 불러오기
file_path = "/home/uon/uon_rt/demo/gen_only_debug.csv"

if not os.path.exists(file_path):
    print(f"Error: {file_path} 파일이 존재하지 않습니다. 먼저 C++ 코드를 실행해 주세요.")
else:
    df = pd.read_csv(file_path)

    # 2. 그래프 그리기 (3행 1열 구조)
    fig, axs = plt.subplots(3, 1, figsize=(10, 12), sharex=True)
    fig.suptitle('Trajectory Generator Debug (Joint 0)', fontsize=16)

    # (1) Position - 목표 각도 (deg)
    axs[0].plot(df['Time'], df['Desired_Q'], color='blue', linewidth=2, label='Desired Q')
    axs[0].set_ylabel('Position [deg]')
    axs[0].grid(True, linestyle='--', alpha=0.7)
    axs[0].legend()

    # (2) Velocity - 목표 속도 (deg/s)
    axs[1].plot(df['Time'], df['Desired_DQ'], color='green', linewidth=2, label='Desired DQ')
    axs[1].set_ylabel('Velocity [deg/s]')
    axs[1].grid(True, linestyle='--', alpha=0.7)
    axs[1].legend()

    # (3) Acceleration - 목표 가속도 (deg/s^2)
    # 🌟 이 그래프가 수직으로 튀는 지점이 Jerk 발생 지점입니다.
    axs[2].plot(df['Time'], df['Desired_DDQ'], color='red', linewidth=2, label='Desired DDQ')
    axs[2].set_ylabel('Acceleration [deg/s^2]')
    axs[2].set_xlabel('Time [s]')
    axs[2].grid(True, linestyle='--', alpha=0.7)
    axs[2].legend()

    # 그래프 간격 조절 및 출력
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()
    
    # 🌟 Jerk(가가속도) 계산 및 분석
    dt = df['Time'].diff().mean()
    jerk = df['Desired_DDQ'].diff() / dt
    print(f"--- 분석 결과 ---")
    print(f"최대 가속도: {df['Desired_DDQ'].max():.2f} deg/s^2")
    print(f"최대 Jerk(계산치): {jerk.abs().max():.2f} deg/s^3")
    if jerk.abs().max() > 10000:
        print("경고: 가속도가 불연속적으로 변하고 있습니다(TrapJ 특성). 실제 로봇에서 충격이 발생할 수 있습니다.")