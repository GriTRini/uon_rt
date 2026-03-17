import pandas as pd
import matplotlib.pyplot as plt
import os

def plot_robot_trajectory(file_path):
    # 1. CSV 파일 로드
    if not os.path.exists(file_path):
        print(f"에러: {file_path} 파일을 찾을 수 없습니다.")
        return

    df = pd.read_csv(file_path)

    # 2. 그래프 설정 (3개의 행으로 구성된 서브플롯)
    fig, axes = plt.subplots(3, 1, figsize=(10, 12), sharex=True)
    plt.subplots_adjust(hspace=0.3)

    # 타임스탬프 (X축)
    time = df['Time']

    # --- (1) Position Plot (Desired_Q) ---
    axes[0].plot(time, df['Desired_Q'], color='royalblue', linewidth=2, label='Position (deg)')
    axes[0].set_ylabel('Position [deg]', fontsize=12)
    axes[0].set_title('Joint Trajectory: Position', fontsize=14)
    axes[0].grid(True, linestyle='--', alpha=0.7)
    axes[0].legend(loc='upper right')

    # --- (2) Velocity Plot (Desired_DQ) ---
    axes[1].plot(time, df['Desired_DQ'], color='darkorange', linewidth=2, label='Velocity (deg/s)')
    axes[1].set_ylabel('Velocity [deg/s]', fontsize=12)
    axes[1].set_title('Joint Trajectory: Velocity', fontsize=14)
    axes[1].grid(True, linestyle='--', alpha=0.7)
    axes[1].legend(loc='upper right')

    # --- (3) Acceleration Plot (Desired_DDQ) ---
    axes[2].plot(time, df['Desired_DDQ'], color='crimson', linewidth=2, label='Acceleration (deg/s²)')
    axes[2].set_xlabel('Time [sec]', fontsize=12)
    axes[2].set_ylabel('Acceleration [deg/s²]', fontsize=12)
    axes[2].set_title('Joint Trajectory: Acceleration', fontsize=14)
    axes[2].grid(True, linestyle='--', alpha=0.7)
    axes[2].legend(loc='upper right')

    # 전체 레이아웃 조정 및 저장/출력
    plt.tight_layout()
    output_image = "trajectory_plot.png"
    plt.savefig(output_image)
    print(f"그래프가 {output_image}로 저장되었습니다.")
    plt.show()

if __name__ == "__main__":
    # 사용자의 경로에 맞춰 파일 지정
    CSV_PATH = "/home/uon/uon_rt/demo/gen_only_trapj.csv"
    plot_robot_trajectory(CSV_PATH)