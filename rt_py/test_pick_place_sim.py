from trajectory import TrajGenerator
import numpy as np
import csv

def run_pick_and_place_sim():
    print("--- 🤖 TrajGenerator Pick & Place 대화형 시뮬레이션 ---")
    
    # 1. 초기화 (🌟 uon_rt 직접 호출 제거, 문자열로 모델명만 전달)
    gen = TrajGenerator()
    q0 = np.zeros(6, dtype=np.float64)
    gen.initialize("m1013", q0, q0, q0)
    
    # [설정] TCP 오프셋
    tx, ty, tz = -0.029, 0.0, 0.382
    tr, tp, tyw = 0.0, 0.0, 0.0
    gen.set_tcp(tx, ty, tz, tr, tp, tyw)

    # 데이터 로깅 준비
    csv_data = [["Time", "Step", "J1", "J2", "J3", "J4", "J5", "J6", "TCP_X", "TCP_Y", "TCP_Z"]]
    
    total_time = 0.0
    dt = 0.001

    # =========================================================================
    # 🌟 모듈 1: 목표 도달 대기 (시뮬레이션 루프)
    # =========================================================================
    def wait_goal(info: str, q_th=None, p_th=0.02, r_th=1.0, timeout_sec=100.0) -> bool:
        nonlocal total_time
        loop_count = 0
        start_time = total_time
        
        while True:
            gen.update(dt)
            total_time += dt
            loop_count += 1

            reached = gen.goal_reached(
                angles_enorm_thold=q_th, 
                pos_enorm_thold=p_th, 
                rot_enorm_thold=r_th
            )
            
            cur_q = gen.angles()
            cur_tcp = gen.tmat() # 4x4 matrix
            x, y, z = cur_tcp[0,3], cur_tcp[1,3], cur_tcp[2,3]

            csv_data.append([
                round(total_time, 4), info,
                round(cur_q[0], 4), round(cur_q[1], 4), round(cur_q[2], 4),
                round(cur_q[3], 4), round(cur_q[4], 4), round(cur_q[5], 4),
                round(x, 4), round(y, 4), round(z, 4)
            ])

            if loop_count % 500 == 0:
                print(f"[T: {total_time:6.3f}s] {info:18s} | J: {cur_q.round(2)}")

            if reached:
                print(f"      ✅ {info} 완료\n")
                return True
            
            if (total_time - start_time) >= timeout_sec:
                print(f"      ⚠️ {info} 타임아웃 발생\n")
                return False

    # =========================================================================
    # 🌟 모듈 2: 동작 실행 래퍼 (현재 TMAT 기반 Offset 적용)
    # =========================================================================
    def execute_move(x, y, z, step_info) -> bool:
        # 1. 시뮬레이터에서 현재 포즈 행렬 가져오기
        current_tmat = gen.tmat()
        
        # 2. 복사본 생성 후 위치(X,Y,Z)만 덮어쓰기 (자세는 현재 상태 유지)
        goal_tmat = current_tmat.copy()
        goal_tmat[0, 3] = x
        goal_tmat[1, 3] = y
        goal_tmat[2, 3] = z
        
        # 3. attrl(goal_tmat, kp) 버전 호출
        if not gen.attrl(goal_tmat, 150.0):
            return False
        return wait_goal(step_info, q_th=None, p_th=0.01, r_th=1.0, timeout_sec=100.0)

    # =========================================================================
    # 🚀 메인 실행부
    # =========================================================================
    try:
        print("\n1️⃣ [Set TCP] 툴 정보를 로봇에 입력합니다.")
        
        print("\n2️⃣ [TrapJ] 초기 자세(J1, J3, J5 -90도)로 이동합니다...")
        q_pose = np.zeros(6, dtype=np.float64)
        q_pose[0] = -90.0
        q_pose[2] = -90.0
        q_pose[4] = -90.0
        if gen.trapj(q_pose):
            wait_goal("2_Setup_TrapJ", q_th=0.5, p_th=None, r_th=None)

        print("\n3️⃣ [Align] 툴 팁을 바닥(-Z) 방향으로 정렬합니다...")
        if gen.align_tcp_to_floor(yaw_deg=90.0, kp=100.0):
            wait_goal("3_Align_Vertical", q_th=None, p_th=0.01, r_th=0.5)

        print("\n==========================================================")
        print("4️⃣ [Pick & Place Sequence] 터미널에서 좌표를 입력하세요.")
        print("▶ 단위: 미터(m) / 입력 예시: -0.5 0.1 0.2")
        print("▶ 종료하려면 'q'를 입력하세요.")
        print("==========================================================\n")
        
        while True:
            # ---------------------------------------------------------
            # [Phase 1] PICK 파트
            # ---------------------------------------------------------
            inp = input("\n>> 🟩 [PICK] 집으러 갈 좌표 X Y Z 입력 (또는 q): ")
            if inp.strip().lower() == 'q':
                break
            
            try:
                pick_x, pick_y, pick_z = map(float, inp.split())
            except ValueError:
                print("⚠️ 잘못된 입력입니다. 숫자 3개를 띄어쓰기로 구분해 입력하세요.")
                continue

            print(f"\n   🚀 [Pick 시퀀스 시작] 목표: [{pick_x}, {pick_y}, {pick_z}]")

            # 🌟 회전 인자(r, p, yaw) 제거
            print("\n   ▶ [Step 1] Pick 상단 5cm 위치로 접근 중...")
            if not execute_move(pick_x, pick_y, pick_z + 0.05, "Pick_Approach"): continue

            print("\n   ▶ [Step 2] Pick 지점으로 수직 하강 중...")
            if not execute_move(pick_x, pick_y, pick_z, "Pick_Reach"): continue

            print("\n   🧲 [I/O 제어] 그리퍼 작동 (ON) -> 시뮬레이션 대기 0.5초")
            for _ in range(500):
                gen.update(dt); total_time += dt

            print("\n   ▶ [Step 3] 10cm 위로 수직 상승 중...")
            if not execute_move(pick_x, pick_y, pick_z + 0.10, "Pick_Retract"): continue


            # ---------------------------------------------------------
            # [Phase 2] PLACE 파트
            # ---------------------------------------------------------
            inp = input("\n>> 🟦 [PLACE] 놓으러 갈 좌표 X Y Z 지점 입력 (또는 q): ")
            if inp.strip().lower() == 'q':
                break
            
            try:
                place_x, place_y, place_z = map(float, inp.split())
            except ValueError:
                print("⚠️ 잘못된 입력입니다. 루프를 처음(Pick)부터 다시 시작합니다.")
                continue

            print(f"\n   🚀 [Place 시퀀스 시작] 목표: [{place_x}, {place_y}, {place_z}]")

            print("\n   ▶ [Step 4] Place 상단 5cm 위치로 접근 중...")
            if not execute_move(place_x, place_y, place_z + 0.05, "Place_Approach"): continue

            print("\n   ▶ [Step 5] Place 지점으로 수직 하강 중...")
            if not execute_move(place_x, place_y, place_z, "Place_Reach"): continue

            print("\n   👐 [I/O 제어] 그리퍼 해제 (OFF) -> 시뮬레이션 대기 0.5초")
            for _ in range(500):
                gen.update(dt); total_time += dt

            print("\n   ▶ [Step 6] 10cm 위로 수직 상승 중...")
            if not execute_move(place_x, place_y, place_z + 0.10, "Place_Retract"): continue

            print("\n   🎉 1회 Pick & Place 완료!")

    except Exception as e:
        print(f"❌ Error: {e}")

    finally:
        filename = "sim_pick_place_log.csv"
        with open(filename, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerows(csv_data)
        
        print(f"\n🏁 [시뮬레이션 종료] 궤적 데이터가 '{filename}'에 저장되었습니다.")

if __name__ == "__main__":
    run_pick_and_place_sim()