import pandas as pd
import matplotlib.pyplot as plt
import os

def plot_attrl_results(file_path):
    if not os.path.exists(file_path):
        print(f"에러: {file_path} 파일이 없습니다. C++ 코드를 먼저 실행하세요.")
        return

    # 1. 데이터 로드
    df = pd.read_csv(file_path)

    # 2. 그래프 설정 (3개 서브플롯)
    fig, axes = plt.subplots(3, 1, figsize=(12, 15), sharex=True)
    plt.subplots_adjust(hspace=0.3)

    time = df['Time']

    # --- (1) TCP Z-axis Position ---
    axes[0].plot(time, df['TCP_Z'], color='red', linewidth=2.5, label='TCP Z [m]')
    axes[0].set_ylabel('Position [m]', fontsize=12)
    axes[0].set_title('TCP Cartesian Path (Z-axis Lift)', fontsize=14)
    axes[0].grid(True, linestyle='--', alpha=0.7)
    axes[0].legend(loc='upper left')

    # --- (2) Joint Positions (J1 ~ J6) ---
    joints = ['J1_P', 'J2_P', 'J3_P', 'J4_P', 'J5_P', 'J6_P']
    colors = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red', 'tab:purple', 'tab:brown']
    
    for joint, color in zip(joints, colors):
        axes[1].plot(time, df[joint], label=joint, color=color, alpha=0.8)
    
    axes[1].set_ylabel('Angle [deg]', fontsize=12)
    axes[1].set_title('Joint Space Positions', fontsize=14)
    axes[1].grid(True, linestyle='--', alpha=0.7)
    axes[1].legend(loc='center left', bbox_to_anchor=(1, 0.5))

    # --- (3) Velocity & Acceleration Norm ---
    # 시스템의 전체적인 역동성을 확인하기 위한 Norm 값
    axes[2].plot(time, df['V_Norm'], color='darkorange', linewidth=2, label='Velocity Norm')
    axes[2].plot(time, df['A_Norm'], color='forestgreen', linewidth=1.5, label='Acceleration Norm', linestyle=':')
    
    axes[2].set_xlabel('Time [sec]', fontsize=12)
    axes[2].set_ylabel('Norm Value', fontsize=12)
    axes[2].set_title('Total Joint Dynamics (Norm)', fontsize=14)
    axes[2].grid(True, linestyle='--', alpha=0.7)
    axes[2].legend(loc='upper right')

    # 결과 출력
    plt.tight_layout()
    plt.savefig("attrl_analysis.png")
    print("📈 그래프가 'attrl_analysis.png'로 저장되었습니다.")
    plt.show()

if __name__ == "__main__":
    CSV_FILE = "attrl_debug_data.csv"
    plot_attrl_results(CSV_FILE)