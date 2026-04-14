import rt_py as rp  # rt_py/__init__.py를 통해 trajectory, robot 등을 가져옴
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import time

def run_simulation():
    print("🚀 [Simulation] 인터랙티브 Pick & Place 시뮬레이션 시작 (Package Interface)")
    
    # 1. 초기화 (rt_py 패키지의 클래스 사용)
    # trajectory.py에 정의된 RobotModel과 TrajGenerator를 사용합니다.
    model = rp.RobotModel("m1013")
    gen = rp.TrajGenerator()
    
    q_init = np.array([-90.0, 0.0, -90.0, 0.0, -90.0, 0.0]) # 초기 자세
    gen.initialize(model, q_init, np.zeros(6), np.zeros(6))
    
    # 2. TCP 설정 (m 단위)
    tcp_x, tcp_y, tcp_z = -0.029, 0.0, 0.3819
    gen.set_tcp(tcp_x, tcp_y, tcp_z, 0.0, 0.0, 0.0)
    print(f"📍 TCP Offset 설정 완료: Z {tcp_z*1000:.1f}mm")

    dt = 0.001
    t = 0.0
    history = {"time": [], "q": [], "pos": [], "mode": []}

    def update_sim(mode_id, duration=10.0):
        nonlocal t
        start_t = t
        # trajectory.py의 goal_reached(kwargs) 사용
        while not gen.goal_reached(q_th=0.1, p_th=0.001, r_th=0.1):
            gen.update(dt)
            
            # trajectory.py의 @property 속성 사용
            history["time"].append(t)
            history["q"].append(gen.angles.copy())
            history["pos"].append(gen.tmat[:3, 3].copy())
            history["mode"].append(mode_id)
            
            t += dt
            if (t - start_t) > duration: 
                break

    # ---------------------------------------------------------
    # [Step 1] Home 자세 이동 및 바닥 정렬
    # ---------------------------------------------------------
    print("\n[Step 1] 초기 자세 및 바닥 정렬 중...")
    gen.trapj(q_init)
    update_sim(1)
    
    gen.align_to_floor(yaw_deg=90.0, kp=60.0)
    update_sim(2)

    # ---------------------------------------------------------
    # [Step 2] 인터랙티브 루프
    # ---------------------------------------------------------
    print("\n" + "="*50)
    print("시뮬레이션 좌표 입력 (m 단위, 예: 0.5 0.1 0.2)")
    print("'q' 입력 시 종료 및 데이터 저장")
    print("="*50)

    try:
        while True:
            val = input("\n🟩 [PICK] 목표 X Y Z: ")
            if val.lower() == 'q': break
            try:
                px, py, pz = map(float, val.split())
            except: continue

            # Pick 시퀀스 (Approach -> Reach -> Retract)
            for sub_name, z_off in [("Approach", 0.05), ("Reach", 0.0), ("Retract", 0.1)]:
                print(f"  ▶ Pick {sub_name} 중...")
                target_mat = gen.tmat.copy()
                target_mat[:3, 3] = [px, py, pz + z_off]
                gen.attrl(target_mat, kp=40.0)
                update_sim(3)

            val = input("🟦 [PLACE] 목표 X Y Z: ")
            if val.lower() == 'q': break
            try:
                lx, ly, lz = map(float, val.split())
            except: continue

            # Place 시퀀스 (Approach -> Reach -> Retract)
            for sub_name, z_off in [("Approach", 0.05), ("Reach", 0.0), ("Retract", 0.1)]:
                print(f"  ▶ Place {sub_name} 중...")
                target_mat = gen.tmat.copy()
                target_mat[:3, 3] = [lx, ly, lz + z_off]
                gen.attrl(target_mat, kp=40.0)
                update_sim(4)
            
            print("✅ 시뮬레이션 사이클 완료!")

    except KeyboardInterrupt:
        pass

    # ---------------------------------------------------------
    # [Step 3] 데이터 저장 및 시각화
    # ---------------------------------------------------------
    if history["time"]:
        print(f"\n✅ 시뮬레이션 종료 (총 시간: {t:.2f}s)")
        pos_all = np.array(history["pos"])
        
        df = pd.DataFrame({
            "time": history["time"],
            "x": pos_all[:, 0], "y": pos_all[:, 1], "z": pos_all[:, 2],
            "mode": history["mode"]
        })
        df.to_csv("simulation_interactive_data.csv", index=False)
        print("💾 저장 완료: simulation_interactive_data.csv")

        # 결과 그래프 (TCP Z축 궤적)
        plt.figure(figsize=(10, 6))
        plt.plot(history["time"], pos_all[:, 2], color='blue', label='TCP Z-height')
        plt.fill_between(history["time"], 0, pos_all[:, 2], alpha=0.1)
        plt.title("Simulation: TCP Z-axis Motion Profile (via rt_py)")
        plt.xlabel("Time (s)"); plt.ylabel("Position (m)")
        plt.grid(True); plt.legend()
        plt.show()

if __name__ == "__main__":
    run_simulation()