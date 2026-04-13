import rt_control_cpp_impl as rc
import numpy as np
import matplotlib.pyplot as plt
import time

def run_hybrid_test():
    # 1. 초기화
    print("🤖 하이브리드 제어 테스트 시작 (m1013)")
    model = rc.RobotModel("m1013")
    gen = rc.TrajGenerator()
    
    q_start = np.zeros(6)
    gen.initialize(model, q_start, q_start, q_start)
    
    dt = 0.001
    history = {"time": [], "q": [], "pos": [], "state": []}

    # ---------------------------------------------------------
    # 단계 1: Joint Space TrapJ (관절 1, 2, 3번 이동)
    # ---------------------------------------------------------
    target_q = np.array([30.0, -30.0, 60.0, 0.0, 0.0, 0.0])
    print(f"\n[Step 1] TrapJ 시작 -> {target_q}")
    gen.trapj(target_q)
    
    t = 0.0
    while not gen.goal_reached(q_th=0.1):
        gen.update(dt)
        
        history["time"].append(t)
        history["q"].append(gen.angles.copy())
        history["pos"].append(gen.tmat[:3, 3].copy())
        history["state"].append(1) # TrapJ 상태
        
        t += dt
        if t > 5.0: break # Safety Timeout

    print(f"✅ TrapJ 도달 (시간: {t:.2f}s)")

    # ---------------------------------------------------------
    # 단계 2: Task Space AttrL (현재 위치에서 Z축 방향 이동)
    # ---------------------------------------------------------
    current_tmat = gen.tmat
    target_tmat = current_tmat.copy()
    target_tmat[2, 3] += 0.1  # 현재 위치에서 위로 10cm 이동
    
    print(f"\n[Step 2] AttrL 시작 (Z+10cm) -> 목표 Z: {target_tmat[2, 3]:.3f}")
    gen.attrl(target_tmat, kp=40.0)
    
    attrl_start_t = t
    while not gen.goal_reached(p_th=0.001):
        gen.update(dt)
        
        history["time"].append(t)
        history["q"].append(gen.angles.copy())
        history["pos"].append(gen.tmat[:3, 3].copy())
        history["state"].append(2) # AttrL 상태
        
        t += dt
        if (t - attrl_start_t) > 3.0: break # Safety Timeout

    print(f"✅ AttrL 도달 (총 소요 시간: {t:.2f}s)")

    # ---------------------------------------------------------
    # 3. 결과 시각화
    # ---------------------------------------------------------
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    # 관절 각도 그래프
    history_q = np.array(history["q"])
    for i in range(6):
        ax1.plot(history["time"], history_q[:, i], label=f'Joint {i+1}')
    ax1.axvline(x=attrl_start_t, color='r', linestyle='--', label='Mode Switch')
    ax1.set_ylabel("Joint Angles (deg)")
    ax1.legend()
    ax1.grid(True)

    # 끝단 위치(Z) 그래프
    history_pos = np.array(history["pos"])
    ax2.plot(history["time"], history_pos[:, 2], color='blue', linewidth=2, label='TCP Z-Pos')
    ax2.axvline(x=attrl_start_t, color='r', linestyle='--')
    ax2.set_ylabel("TCP Z Position (m)")
    ax2.set_xlabel("Time (s)")
    ax2.legend()
    ax2.grid(True)

    plt.suptitle("Hybrid Motion: TrapJ (Joint) -> AttrL (Task)")
    plt.tight_layout()
    plt.savefig("motion_result.png")
    print("📊 그래프가 'motion_result.png'로 저장되었습니다. 확인해 보세요!")

if __name__ == "__main__":
    run_hybrid_test()