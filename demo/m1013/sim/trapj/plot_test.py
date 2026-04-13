import pandas as pd
import matplotlib.pyplot as plt
import os

def plot_trapj_results(file_path):
    # 데이터 로드
    if not os.path.exists(file_path):
        print(f"Error: {file_path} 파일을 찾을 수 없습니다.")
        return

    print(f"Plotting results from {file_path}...")
    data = pd.read_csv(file_path)

    # 6개 관절 그래프 생성
    fig, axes = plt.subplots(3, 2, figsize=(12, 10), sharex=True)
    fig.suptitle('Multi-Joint Trapezoidal Trajectory (TrapJ)', fontsize=16)

    joint_names = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']
    colors = ['blue', 'green', 'red', 'cyan', 'magenta', 'orange']

    for i, (ax, name, color) in enumerate(zip(axes.flatten(), joint_names, colors)):
        ax.plot(data['Time'], data[f'{name}_Pos'], label=f'{name} Position', color=color, linewidth=2)
        ax.set_ylabel('Angle [deg]')
        ax.set_title(f'Joint {i+1} Movement')
        ax.grid(True, linestyle='--', alpha=0.7)
        ax.legend(loc='upper right')

    # X축 라벨 설정 (맨 아래 그래프)
    axes[2, 0].set_xlabel('Time [s]')
    axes[2, 1].set_xlabel('Time [s]')

    # 도달 시점 표시 (Reached == 1)
    reached_points = data[data['Reached'] == 1]['Time']
    if not reached_points.empty:
        # 연속된 Reached 구간 중 첫 번째 시점들만 표시
        diff = reached_points.diff()
        start_reached = reached_points[diff > 0.1] # 100ms 이상 간격 차이 날 때
        if len(start_reached) == 0 and len(reached_points) > 0:
            start_reached = [reached_points.iloc[0]]
            
        for t in start_reached:
            for ax in axes.flatten():
                ax.axvline(x=t, color='black', linestyle=':', alpha=0.5)

    plt.tight_layout()
    plt.subplots_adjust(top=0.9) # 제목 공간 확보
    plt.show()

if __name__ == "__main__":
    plot_trapj_results('trapj_debug_data.csv')