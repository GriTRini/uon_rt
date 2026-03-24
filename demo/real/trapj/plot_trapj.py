import pandas as pd
import matplotlib.pyplot as plt

def analyze_robot_data(file_path):
    # 1. 데이터 불러오기
    try:
        df = pd.read_csv(file_path)
    except FileNotFoundError:
        print(f"파일을 찾을 수 없습니다: {file_path}")
        return

    # 2. 그래프 설정
    fig, axes = plt.subplots(3, 1, figsize=(12, 15), sharex=True)
    plt.subplots_adjust(hspace=0.3)

    # --- 그래프 1: J3 관절 추종 성능 (목표 vs 실제) ---
    # 사용자가 J3(index 2) 위주로 움직였으므로 J3를 중점적으로 봅니다.
    axes[0].plot(df['Time'], df['Target_J3'], 'r--', label='Target J3', alpha=0.8)
    axes[0].plot(df['Time'], df['Actual_J3'], 'b-', label='Actual J3', alpha=0.6)
    axes[0].set_ylabel('Joint Angle (deg)')
    axes[0].set_title('Joint 3 Tracking Performance')
    axes[0].legend()
    axes[0].grid(True)

    # --- 그래프 2: 위치 오차 (Q_Error) ---
    # 모든 관절의 총합 오차(L2 Norm)를 시각화합니다.
    axes[1].plot(df['Time'], df['Q_Error'], 'g-', label='L2 Norm Error')
    axes[1].set_ylabel('Error Magnitude')
    axes[1].set_title('Total Joint Position Error (Q_Error)')
    axes[1].legend()
    axes[1].grid(True)

    # --- 그래프 3: 루프 실행 시간 (Exec_us) ---
    # 실시간성이 잘 유지되었는지(1000us 미만인지) 확인합니다.
    axes[2].plot(df['Time'], df['Exec_us'], 'k.', markersize=1, label='Loop Exec Time')
    axes[2].axhline(y=1000, color='r', linestyle='--', label='1ms Limit')
    axes[2].set_xlabel('Time (s)')
    axes[2].set_ylabel('Execution Time (us)')
    axes[2].set_title('Real-time Loop Execution Time')
    axes[2].set_ylim(0, max(df['Exec_us'].max() * 1.1, 1200)) # 1ms 근처로 보기 좋게 설정
    axes[2].legend()
    axes[2].grid(True)

    plt.suptitle(f"Robot Trajectory Analysis: {file_path}", fontsize=16)
    
    # 3. 통계 정보 출력
    print("===== 📊 로봇 제어 벤치마킹 결과 요약 =====")
    print(f"총 주행 시간: {df['Time'].iloc[-1]:.2f} s")
    print(f"최대 위치 오차: {df['Q_Error'].max():.4f}")
    print(f"평균 루프 실행 시간: {df['Exec_us'].mean():.2f} us")
    print(f"최대 루프 실행 시간: {df['Exec_us'].max():.2f} us")
    
    # 지연(Jitter) 발생 횟수 확인 (1ms 초과)
    jitter_count = len(df[df['Exec_us'] > 1000])
    print(f"실시간성 위반 (1ms 초과): {jitter_count} 회")
    print("========================================")

    plt.show()

if __name__ == "__main__":
    analyze_robot_data('real_robot_bench_data.csv')