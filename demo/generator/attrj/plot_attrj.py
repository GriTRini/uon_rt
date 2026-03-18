import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def plot_attrj_analysis(file_path):
    # 1. 데이터 로드
    try:
        data = pd.read_csv(file_path)
    except FileNotFoundError:
        print(f"Error: {file_path} 파일을 찾을 수 없습니다.")
        return

    # 스타일 설정
    plt.style.use('seaborn-v0_8-whitegrid')
    
    # 그래프 영역 생성 (2 x 1 레이아웃)
    fig = plt.figure(figsize=(14, 12))
    gs = fig.add_gridspec(2, 2)

    # --- 영역 1: 6개 관절의 위치 (Joint Positions) ---
    ax_pos = fig.add_subplot(gs[0, :])
    joint_cols = [f'J{i}_Pos' for i in range(1, 7)]
    colors = plt.cm.tab10(np.linspace(0, 1, 6))

    for col, color in zip(joint_cols, colors):
        ax_pos.plot(data['Time'], data[col], label=col.replace('_Pos', ''), color=color, linewidth=2)
    
    ax_pos.set_title('Joint Space Positions (AttrJ)', fontsize=15, fontweight='bold')
    ax_pos.set_ylabel('Angle [deg]', fontsize=12)
    ax_pos.legend(loc='upper right', ncol=3)
    ax_pos.grid(True, linestyle='--', alpha=0.7)

    # --- 영역 2: 위치 오차 수렴 (Q_Err - Log Scale) ---
    ax_err = fig.add_subplot(gs[1, 0])
    ax_err.plot(data['Time'], data['Q_Err'], color='red', label='Joint Error Norm', linewidth=1.5)
    ax_err.axhline(y=0.1, color='black', linestyle='--', alpha=0.6, label='Threshold (0.1 deg)')
    
    ax_err.set_yscale('log') # 오차는 로그 스케일로 보는 것이 수렴 확인에 유리함
    ax_err.set_title('Joint Position Error Norm (Log Scale)', fontsize=13)
    ax_err.set_xlabel('Time [s]', fontsize=12)
    ax_err.set_ylabel('Error [deg]', fontsize=12)
    ax_err.legend()

    # --- 영역 3: 속도 변화 (DQ_Norm) ---
    ax_vel = fig.add_subplot(gs[1, 1])
    ax_vel.plot(data['Time'], data['DQ_Norm'], color='blue', label='Velocity Norm', linewidth=1.5)
    ax_vel.axhline(y=0.5, color='black', linestyle='--', alpha=0.6, label='Threshold (0.5 deg/s)')
    
    ax_vel.set_title('Joint Velocity Norm', fontsize=13)
    ax_vel.set_xlabel('Time [s]', fontsize=12)
    ax_vel.set_ylabel('Velocity [deg/s]', fontsize=12)
    ax_vel.legend()

    # 스텝 전환(Step) 및 도달(Reached) 시점 표시
    step_changes = data[data['Step'].diff() != 0]['Time']
    reached_points = data[data['Reached'] == 1]['Time']

    for t in step_changes:
        for ax in [ax_pos, ax_err, ax_vel]:
            ax.axvline(x=t, color='gray', linestyle='-', alpha=0.3)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    plot_attrj_analysis('attrj_debug_report.csv')