import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def analyze_attrj_data(file_path):
    # 1. 데이터 불러오기
    try:
        df = pd.read_csv(file_path)
    except FileNotFoundError:
        print(f"파일을 찾을 수 없습니다: {file_path}")
        return

    # 2. 그래프 설정 (2행 2열 구조)
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    plt.subplots_adjust(hspace=0.3, wspace=0.2)

    # --- (1,1) J3 & J5 추종 성능 (주요 움직임 관절) ---
    axes[0, 0].plot(df['Time'], df['Target_J3'], 'r--', label='Target J3')
    axes[0, 0].plot(df['Time'], df['Actual_J3'], 'r-', label='Actual J3', alpha=0.6)
    axes[0, 0].plot(df['Time'], df['Target_J5'], 'b--', label='Target J5')
    axes[0, 0].plot(df['Time'], df['Actual_J5'], 'b-', label='Actual J5', alpha=0.6)
    axes[0, 0].set_title('Joint 3 & 5 Tracking (attrj)')
    axes[0, 0].set_ylabel('Angle (deg)')
    axes[0, 0].legend()
    axes[0, 0].grid(True)

    # --- (1,2) 위치 오차 (Q_Error) - 지수 감쇠 확인 ---
    axes[0, 1].plot(df['Time'], df['Q_Error'], 'g-', label='L2 Norm Error')
    axes[0, 1].set_title('Position Error Magnitude')
    axes[0, 1].set_ylabel('Error')
    axes[0, 1].set_yscale('log')  # attrj 특성을 보기 위해 로그 스케일 권장
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].legend()
    axes[0, 1].grid(True, which="both", ls="-", alpha=0.5)

    # --- (2,1) 제어 주기 실시간성 (Exec_us) ---
    axes[1, 0].scatter(df['Time'], df['Exec_us'], s=1, c='black', label='Exec Time')
    axes[1, 0].axhline(y=1000, color='r', linestyle='--', label='1ms Boundary')
    axes[1, 0].set_title('Loop Execution Time')
    axes[1, 0].set_ylabel('Time (us)')
    axes[1, 0].set_ylim(0, 1200)
    axes[1, 0].legend()
    axes[1, 0].grid(True)

    # --- (2,2) 위상 평면 (Phase Portrait) - 제어 안정성 확인 ---
    # 오차의 변화를 통해 제어기가 목표점에 안정적으로 수렴하는지 봅니다.
    axes[1, 1].plot(df['Actual_J3'], df['Q_Error'], 'm-', alpha=0.7)
    axes[1, 1].set_title('Phase Portrait (J3 vs Total Error)')
    axes[1, 1].set_xlabel('Actual J3 Angle')
    axes[1, 1].set_ylabel('Total Error')
    axes[1, 1].grid(True)

    plt.suptitle(f"ATTRJ Control Analysis: {file_path}", fontsize=16)

    # 3. 데이터 통계 요약
    print("===== 🧲 ATTRJ 제어 분석 보고서 =====")
    print(f"총 데이터 포인트: {len(df)} 개")
    print(f"최대 오차(Peak Error): {df['Q_Error'].max():.4f}")
    
    # 정착 시간(Settling Time) 대략적 계산 (오차가 0.1 이하로 유지되는 시점)
    threshold = 0.1
    under_threshold = df[df['Q_Error'] < threshold]
    if not under_threshold.empty:
        print(f"오차 {threshold} 이내 진입 성공 여부: YES")
    
    print(f"평균 루프 연산 속도: {df['Exec_us'].mean():.2f} us")
    print(f"지터(Jitter) 발생 건수(>1ms): {len(df[df['Exec_us'] > 1000])} 건")
    print("======================================")

    plt.show()

if __name__ == "__main__":
    # 파일 이름을 실제 생성된 이름과 맞춰주세요
    analyze_attrj_data('real_robot_attrj_data.csv')