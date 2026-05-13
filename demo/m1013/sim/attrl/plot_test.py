import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # 3D 프로팅을 위한 모듈
import numpy as np

def plot_robot_trajectory_3d(file_path="apple_pick_place_test.csv"):
    try:
        # 1. 데이터 읽기 및 정렬
        df = pd.read_csv(file_path)
        df = df.sort_values(by='Time').reset_index(drop=True)
        print(f"✅ 데이터를 불러왔습니다. 총 {len(df)} 라인")
    except Exception as e:
        print(f"❌ 에러 발생: {e}")
        return

    # 2. 그래프 설정
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')

    # 3. Mode별 색상 지정 (0: TrapJ, 1: Linear, 2: Wait)
    # Mode에 따라 선의 색상을 다르게 그리면 궤적 분석이 더 쉽습니다.
    modes = df['Mode'].unique()
    # 0: 빨강(TrapJ), 1: 녹색(Linear), 2: 파랑(Wait) 등으로 매핑 가능
    colors_map = {0: 'red', 1: 'green', 2: 'blue'}
    labels_map = {0: 'TrapJ (Joint)', 1: 'Linear (Cartesian)', 2: 'Wait (Gripper)'}

    # 연속된 데이터를 Mode가 바뀔 때마다 나누어서 plot (색상 구분을 위해)
    start_idx = 0
    for i in range(1, len(df)):
        if df['Mode'].iloc[i] != df['Mode'].iloc[start_idx] or i == len(df) - 1:
            segment = df.iloc[start_idx:i+1]
            current_mode = df['Mode'].iloc[start_idx]
            
            ax.plot(segment['X'], segment['Y'], segment['Z'], 
                    color=colors_map.get(current_mode, 'black'), 
                    linewidth=2, 
                    alpha=0.8)
            start_idx = i

    # 4. 시작점과 끝점 표시
    ax.scatter(df['X'].iloc[0], df['Y'].iloc[0], df['Z'].iloc[0], color='black', s=100, label='START')
    ax.scatter(df['X'].iloc[-1], df['Y'].iloc[-1], df['Z'].iloc[-1], color='magenta', s=100, marker='*', label='END')

    # 5. 범례 추가 (중복 방지)
    from matplotlib.lines import Line2D
    legend_elements = [Line2D([0], [0], color='red', lw=2, label='TrapJ'),
                       Line2D([0], [0], color='green', lw=2, label='Linear'),
                       Line2D([0], [0], color='blue', lw=2, label='Wait'),
                       Line2D([0], [0], color='black', marker='o', ls='', label='Start'),
                       Line2D([0], [0], color='magenta', marker='*', ls='', label='End')]
    ax.legend(handles=legend_elements)

    # 6. 축 레이블 및 타이틀
    ax.set_title('Robot TCP 3D Trajectory', fontsize=15)
    ax.set_xlabel('X Axis (m)')
    ax.set_ylabel('Y Axis (m)')
    ax.set_zlabel('Z Axis (m)')

    # 보기 좋은 각도로 설정
    ax.view_init(elev=30, azim=45)
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    plot_robot_trajectory_3d()