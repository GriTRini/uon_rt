import trajectory as rc
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import time

# --- 분석 및 저장 함수 ---
def analyze_and_store_all_joints(history, dt):
    time_arr = np.array(history["time"])
    q_arr = np.array(history["q"])       # Shape: (N, 6)
    mode_arr = np.array(history["mode"]) # Shape: (N,)

    # 1. 수치 미분을 통한 속도 및 가속도 계산 (6개 조인트 일괄)
    # 각도 -> 각속도 (deg/s)
    dq_arr = np.gradient(q_arr, axis=0) / dt
    # 각속도 -> 각가속도 (deg/s^2)
    ddq_arr = np.gradient(dq_arr, axis=0) / dt

    # 2. 통합 데이터프레임 구축 (CSV 저장용)
    data_dict = {"time": time_arr, "mode": mode_arr}
    for i in range(6):
        data_dict[f"j{i+1}_pos"] = q_arr[:, i]
        data_dict[f"j{i+1}_vel"] = dq_arr[:, i]
        data_dict[f"j{i+1}_acc"] = ddq_arr[:, i]

    df_full = pd.DataFrame(data_dict)
    csv_filename = "all_joints_kinematics.csv"
    df_full.to_csv(csv_filename, index=False)
    print(f"\n💾 [저장] 6개 조인트 전체 데이터: {csv_filename}")

    # 3. 터미널 통계 출력 (Max/Min/Mean)
    print("\n" + "="*110)
    print(f"{'Joint':^8} | {'Type':^12} | {'Mean':^15} | {'Max':^15} | {'Min':^15} | {'Unit'}")
    print("-" * 110)

    for i in range(6):
        metrics = [
            ("Position", q_arr[:, i], "deg"),
            ("Velocity", dq_arr[:, i], "deg/s"),
            ("Accel", ddq_arr[:, i], "deg/s^2")
        ]
        for label, data, unit in metrics:
            print(f"Joint {i+1} | {label:12s} | {np.mean(data):15.4f} | {np.max(data):15.4f} | {np.min(data):15.4f} | {unit}")
        print("-" * 110)

    # 4. 시각화 및 PNG 저장 (GUI 미지원 환경 대비)
    fig, axs = plt.subplots(3, 1, figsize=(12, 12), sharex=True)
    colors = ['r', 'g', 'b', 'c', 'm', 'y'] # 조인트별 색상
    
    for i in range(6):
        axs[0].plot(time_arr, q_arr[:, i], color=colors[i], alpha=0.7, label=f'J{i+1}')
        axs[1].plot(time_arr, dq_arr[:, i], color=colors[i], alpha=0.7, label=f'J{i+1}')
        axs[2].plot(time_arr, ddq_arr[:, i], color=colors[i], alpha=0.7, label=f'J{i+1}')

    axs[0].set_ylabel("Angle (deg)")
    axs[0].set_title("6-Joint Kinematics Analysis")
    axs[1].set_ylabel("Velocity (deg/s)")
    axs[2].set_ylabel("Acceleration (deg/s^2)")
    axs[2].set_xlabel("Time (s)")
    
    for ax in axs:
        ax.grid(True, linestyle='--', alpha=0.6)
        ax.legend(loc='upper right', ncol=3, fontsize='small')

    plt.tight_layout()
    plot_filename = "all_joints_analysis.png"
    plt.savefig(plot_filename)
    print(f"📊 [저장] 분석 그래프: {plot_filename}")

# --- 메인 시뮬레이션 루프 ---
def run_simulation():
    print("🚀 [Simulation] 6-Joint Kinematics 통합 분석 시작")
    
    # 모델 및 제네레이터 초기화
    model = rc.RobotModel("m1013")
    gen = rc.TrajGenerator()
    
    home_q = np.array([-90.0, 0.0, -90.0, 0.0, -90.0, 0.0])
    gen.initialize(model, home_q, np.zeros(6), np.zeros(6))
    gen.set_tcp(-0.029, 0.0, 0.3819, 0.0, 0.0, 0.0)

    dt = 0.001 # 1ms cycle
    t = 0.0
    history = {"time": [], "q": [], "mode": []}

    def update_sim(mode_id, timeout=5.0):
        nonlocal t
        start_t = t
        while not gen.goal_reached(q_th=0.1, p_th=0.001, r_th=0.1):
            gen.update(dt)
            history["time"].append(t)
            history["q"].append(gen.angles.copy())
            history["mode"].append(mode_id)
            t += dt
            if (t - start_t) > timeout: 
                print(f"  ⚠️ Mode {mode_id} Timeout!")
                break

    # [시나리오]
    # 1. 초기 위치 (Trapj)
    print("[Step 1] Home 이동 (Trapj)")
    gen.trapj(home_q)
    update_sim(1)

    # 2. 목표 지점 이동 (Attrl)
    print("[Step 2] 목표 지점 이동 (Attrl)")
    target_mat = gen.tmat.copy()
    target_mat[:3, 3] = [0.4, 0.1, 0.045] # X Y Z
    gen.attrl(target_mat, kp=150.0)
    update_sim(2)

    # 3. 다시 Home 복귀 (Trapj - 문제 발생 지점)
    print("[Step 3] Home 복귀 (Trapj)")
    gen.trapj(home_q)
    update_sim(3)

    # 분석 및 저장 실행
    if history["time"]:
        analyze_and_store_all_joints(history, dt)

if __name__ == "__main__":
    run_simulation()