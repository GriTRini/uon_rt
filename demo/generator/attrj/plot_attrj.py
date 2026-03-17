import pandas as pd
import matplotlib.pyplot as plt

def plot_attrj():
    try:
        df = pd.read_csv("attrj_debug_data.csv")
    except FileNotFoundError:
        print("CSV 파일을 찾을 수 없습니다.")
        return

    fig, axes = plt.subplots(3, 1, figsize=(10, 12), sharex=True)

    # Position - 1차 지연 시스템처럼 곡선으로 수렴하는지 확인
    axes[0].plot(df['Time'], df['Pos'], label='Position [deg]', color='royalblue', linewidth=2)
    axes[0].set_ylabel('Angle [deg]')
    axes[0].set_title('Joint 2 Attractor(AttrJ) Motion Profile')
    axes[0].grid(True, linestyle='--')
    axes[0].legend()

    # Velocity - 초기에 가장 빠르고 점차 줄어드는지 확인
    axes[1].plot(df['Time'], df['Vel'], label='Velocity [deg/s]', color='darkorange', linewidth=2)
    axes[1].set_ylabel('Vel [deg/s]')
    axes[1].grid(True, linestyle='--')
    axes[1].legend()

    # Acceleration - 초기 충격(Jerk)이 제한치 내에 있는지 확인
    axes[2].plot(df['Time'], df['Acc'], label='Acceleration [deg/s²]', color='crimson', linewidth=2)
    axes[2].set_ylabel('Acc [deg/s²]')
    axes[2].set_xlabel('Time [sec]')
    axes[2].grid(True, linestyle='--')
    axes[2].legend()

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    plot_attrj()