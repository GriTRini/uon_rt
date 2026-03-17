import pandas as pd
import matplotlib.pyplot as plt
import os

def plot_trapj():
    file_path = "trapj_reached_test.csv"
    
    if not os.path.exists(file_path):
        print(f"❌ 에러: {file_path} 파일이 없습니다.")
        return

    # CSV 로드
    df = pd.read_csv(file_path)

    # 그래프 그리기 (3행 1열)
    fig, axes = plt.subplots(3, 1, figsize=(10, 12), sharex=True)

    # 1. Position (J3_Pos)
    axes[0].plot(df['Time'], df['J3_Pos'], label='J3 Position [deg]', color='b', linewidth=2)
    axes[0].set_ylabel('Position [deg]')
    axes[0].set_title('TrapJ Joint Trajectory (J3)')
    axes[0].grid(True)
    axes[0].legend()

    # 2. Velocity (J3_Vel)
    axes[1].plot(df['Time'], df['J3_Vel'], label='J3 Velocity [deg/s]', color='g', linewidth=2)
    axes[1].set_ylabel('Velocity [deg/s]')
    axes[1].grid(True)
    axes[1].legend()

    # 3. Acceleration (J3_Acc)
    axes[2].plot(df['Time'], df['J3_Acc'], label='J3 Acceleration [deg/s²]', color='r', linewidth=2)
    axes[2].set_ylabel('Acceleration [deg/s²]')
    axes[2].set_xlabel('Time [sec]')
    axes[2].grid(True)
    axes[2].legend()

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    plot_trapj()