import rt_bind as rc
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import time

def run_stress_test():
    print("🔥 [Stress Test] 로봇 각속도/각가속도 한계 테스트 시작")
    
    # 1. 초기화
    model = rc.RobotModel("m1013")
    gen = rc.TrajGenerator()
    
    # 시작 자세 (중립적인 위치)
    q_init = np.array([0.0, 0.0, -90.0, 0.0, -90.0, 0.0])
    gen.initialize(model, q_init, np.zeros(6), np.zeros(6))
    gen.set_tcp(0.0, 0.0, 0.0, 0.0, 0.0, 0.0) # 계산 단순화를 위해 툴 오프셋 0

    dt = 0.001 # 1ms 정밀 시뮬레이션
    t = 0.0
    
    # 데이터 기록용
    history = {
        "time": [], "q": [], "dq": [], "ddq": [], "pos": []
    }

    # ---------------------------------------------------------
    # [Stress Test 설정]
    # ---------------------------------------------------------
    # 매우 먼 두 지점을 설정하여 순간 속도를 극대화
    targets = [
        np.array([[1, 0, 0, 0.5], [0, 1, 0, -0.3], [0, 0, 1, 0.6], [0, 0, 0, 1]]), # Point A
        np.array([[1, 0, 0, -0.4], [0, 1, 0, 0.4], [0, 0, 1, 0.1], [0, 0, 0, 1]]) # Point B
    ]
    
    aggressive_kp = 500.0  # 과격한 게인 설정 (보통 50~100 사용)
    num_cycles = 2
    
    prev_q = q_init.copy()
    prev_dq = np.zeros(6)

    print(f"🚀 테스트 설정: KP = {aggressive_kp}, 목표 지점 간 이동")

    for i in range(num_cycles):
        for idx, target_mat in enumerate(targets):
            print(f"  ▶ Cycle {i+1} - Target {idx+1} 이동 중...")
            gen.attrl(target_mat, kp=aggressive_kp)
            
            # 이동 완료 시까지 루프
            timeout = 0
            while not gen.goal_reached(p_th=0.001, r_th=0.1):
                gen.update(dt)
                
                curr_q = gen.angles.copy()
                
                # 수치 미분: 각속도 (dq = delta_q / dt)
                curr_dq = (curr_q - prev_q) / dt
                # 수치 미분: 각가속도 (ddq = delta_dq / dt)
                curr_ddq = (curr_dq - prev_dq) / dt
                
                # 기록
                history["time"].append(t)
                history["q"].append(curr_q)
                history["dq"].append(curr_dq)
                history["ddq"].append(curr_ddq)
                history["pos"].append(gen.tmat[:3, 3].copy())
                
                # 변수 업데이트
                prev_q = curr_q
                prev_dq = curr_dq
                t += dt
                timeout += 1
                if timeout > 5000: break # 5초 이상 걸리면 강제 종료

    # ---------------------------------------------------------
    # [데이터 분석 및 시각화]
    # ---------------------------------------------------------
    dq_all = np.array(history["dq"])
    ddq_all = np.array(history["ddq"])
    
    # 최대치 계산 (각 관절별)
    max_dq = np.max(np.abs(dq_all), axis=0)
    max_ddq = np.max(np.abs(ddq_all), axis=0)

    print("\n" + "="*50)
    print("📊 [Stress Test 결과 요약]")
    for j in range(6):
        print(f"Joint {j+1}: Max Vel = {max_dq[j]:8.2f} deg/s | Max Accel = {max_ddq[j]:10.2f} deg/s²")
    print("="*50)

    # 그래프 그리기
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10), sharex=True)

    # 1. 각속도 그래프
    for j in range(6):
        ax1.plot(history["time"], dq_all[:, j], label=f'J{j+1}')
    ax1.set_title(f"Joint Velocities (KP={aggressive_kp})")
    ax1.set_ylabel("Velocity (deg/s)")
    ax1.grid(True); ax1.legend(loc='right')

    # 2. 각가속도 그래프
    for j in range(6):
        ax2.plot(history["time"], ddq_all[:, j], label=f'J{j+1}')
    ax2.set_title("Joint Accelerations")
    ax2.set_xlabel("Time (s)"); ax2.set_ylabel("Acceleration (deg/s²)")
    ax2.grid(True); ax2.legend(loc='right')

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    run_stress_test()