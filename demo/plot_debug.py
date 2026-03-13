import pandas as pd
import matplotlib.pyplot as plt

def plot_engineering_debug():
    try:
        df = pd.read_csv('fk_output.csv')
    except FileNotFoundError:
        print("fk_output.csv 파일이 없습니다. C++ 디버그 코드를 먼저 실행하세요.")
        return

    # 그래프 2개를 위아래로 배치
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10), sharex=True)

    # ==========================================
    # 1. 상단 그래프: 6개 조인트 각도 (Joint Angles)
    # ==========================================
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b']
    for i in range(1, 7):
        ax1.plot(df['time'], df[f'q{i}'], label=f'Joint {i} (q{i})', color=colors[i-1], linewidth=2)
    
    ax1.set_title("Joint Angles Trajectory (q1 ~ q6)", fontsize=14, fontweight='bold')
    ax1.set_ylabel("Angle [rad]", fontsize=12)
    ax1.grid(True, linestyle='--', alpha=0.7)
    ax1.legend(loc='center left', bbox_to_anchor=(1, 0.5))

    # Stage 구분선 (TrapJ 완료 지점 대략 1.5초 부근)
    ax1.axvline(x=1.5, color='gray', linestyle='-.', alpha=0.8)
    ax1.text(0.5, ax1.get_ylim()[1]*0.9, 'Stage 1: TrapJ (Ready Pose)', fontsize=10, color='gray')
    ax1.text(2.0, ax1.get_ylim()[1]*0.9, 'Stage 2: AttrL (Cartesian Waypoints)', fontsize=10, color='gray')


    # ==========================================
    # 2. 하단 그래프: 로봇 끝단 XYZ 위치 (End-Effector Position)
    # ==========================================
    ax2.plot(df['time'], df['x6'], label='X Position', color='red', linewidth=2.5)
    ax2.plot(df['time'], df['y6'], label='Y Position', color='green', linewidth=2.5)
    ax2.plot(df['time'], df['z6'], label='Z Position', color='blue', linewidth=2.5)

    ax2.set_title("End-Effector XYZ Position Tracking", fontsize=14, fontweight='bold')
    ax2.set_xlabel("Time [sec]", fontsize=12)
    ax2.set_ylabel("Position [m]", fontsize=12)
    ax2.grid(True, linestyle='--', alpha=0.7)
    ax2.legend(loc='center left', bbox_to_anchor=(1, 0.5))

    ax2.axvline(x=1.5, color='gray', linestyle='-.', alpha=0.8)

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    plot_engineering_debug()