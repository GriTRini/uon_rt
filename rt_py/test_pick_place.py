from robot import Robot
import numpy as np
import time
import csv

def run_real_pick_and_place():
    print("--- 🤖 실제 로봇 Pick & Place 대화형 제어 ---")
    
    # 1. 로봇 객체 초기화 및 연결
    ROBOT_IP = "192.168.1.30"
    model_name = "m1013"
    try:
        robot = Robot(model_name)
    except RuntimeError as e:
        print(e)
        return

    print(f"\n🔌 [{ROBOT_IP}] 로봇에 연결을 시도합니다...")
    if not robot.open_connection(ROBOT_IP, 12345):
        print("❌ 일반 통신 연결 실패!")
        return
    if not robot.connect_rt(ROBOT_IP, 12347):
        print("❌ 실시간(RT) 통신 연결 실패!")
        robot.close_connection()
        return
        
    dt = 0.001 # 1ms 제어/로깅 주기

    # 🌟 긴 sleep 대신 사용할 카운터 기반 대기 함수 (코어 덤프 방지)
    def wait_counts(target_counts: int):
        for _ in range(target_counts):
            time.sleep(dt)

    print("✅ 연결 성공! 서보 모터를 켭니다.")
    robot.servo_on()
    wait_counts(3000) # 서보 켜질 때까지 안정화 대기 (3초 = 3000카운트)

    # [설정] TCP 오프셋 설정 (시뮬레이션에서 쓰신 값 적용)
    print("\n1️⃣ [Set TCP] 툴 정보를 로봇에 입력합니다.")
    tx, ty, tz = -0.029, 0.0, 0.382
    tr, tp, tyw = 0.0, 0.0, 0.0
    robot.set_tcp(tx, ty, tz, tr, tp, tyw)
    wait_counts(500) # TCP 적용 대기 (0.5초)

    # 데이터 로깅 준비
    csv_data = [["Time", "Step", "J1", "J2", "J3", "J4", "J5", "J6", "TCP_X", "TCP_Y", "TCP_Z"]]
    global_start_time = time.time()

    # =========================================================================
    # 🌟 모듈 1: 목표 도달 대기 (실시간 루프)
    # =========================================================================
    def wait_goal(info: str, q_th=None, p_th=0.02, r_th=1.0, timeout_sec=100.0) -> bool:
        start_wait = time.time()
        loop_count = 0
        
        while True:
            # 1. 목표 도달 확인
            reached = robot.get_goal_reached(q_th=q_th, p_th=p_th, r_th=r_th)
            
            # 2. 하드웨어에서 현재 상태 읽기
            cur_q = robot.get_current_angles()
            cur_tcp = robot.get_current_pos() # 4x4 matrix
            
            cur_time = time.time() - global_start_time

            # 3. 데이터가 정상적으로 들어왔을 때만 로깅
            if cur_q is not None and cur_tcp is not None:
                x, y, z = cur_tcp[0,3], cur_tcp[1,3], cur_tcp[2,3]
                csv_data.append([
                    round(cur_time, 4), info,
                    round(cur_q[0], 4), round(cur_q[1], 4), round(cur_q[2], 4),
                    round(cur_q[3], 4), round(cur_q[4], 4), round(cur_q[5], 4),
                    round(x, 4), round(y, 4), round(z, 4)
                ])

                # 4. 콘솔 출력 (약 0.5초마다: 500 * 1ms = 0.5s)
                if loop_count % 500 == 0:
                    print(f"[T: {cur_time:6.3f}s] {info:18s} | J: {cur_q.round(2)}")

            if reached:
                print(f"      ✅ {info} 완료\n")
                return True
            
            if (time.time() - start_wait) >= timeout_sec:
                print(f"      ⚠️ {info} 타임아웃 발생\n")
                return False
                
            time.sleep(dt)
            loop_count += 1

    # =========================================================================
    # 🌟 모듈 2: 동작 실행 래퍼 (현재 TMAT 기반 Offset 적용)
    # =========================================================================
    def execute_move(x, y, z, step_info) -> bool:
        # 1. 실제 하드웨어에서 현재 포즈 행렬 가져오기
        current_tmat = robot.get_current_pos()
        if current_tmat is None:
            print("❌ 현재 위치를 읽어올 수 없습니다.")
            return False
            
        # 2. 복사본 생성 후 위치(X,Y,Z)만 덮어쓰기 (자세는 현재 상태 완벽 유지)
        goal_tmat = current_tmat.copy()
        goal_tmat[0, 3] = x
        goal_tmat[1, 3] = y
        goal_tmat[2, 3] = z
        
        # 3. attrl(goal_tmat, kp) 버전 호출
        if not robot.attrl(goal_tmat, 150.0):
            return False
        return wait_goal(step_info, q_th=None, p_th=0.01, r_th=1.0, timeout_sec=100.0)

    # =========================================================================
    # 🚀 메인 실행부
    # =========================================================================
    try:
        print("\n2️⃣ [TrapJ] 초기 자세(J1, J3, J5 -90도)로 이동합니다...")
        # C++ 메모리 에러를 막기 위해 dtype=np.float64 필수 적용
        q_pose = np.zeros(6, dtype=np.float64)
        q_pose[0] = -90.0
        q_pose[2] = -90.0
        q_pose[4] = -90.0
        if robot.trapj(q_pose):
            wait_goal("2_Setup_TrapJ", q_th=0.5, p_th=None, r_th=None)

        print("\n3️⃣ [Align] 툴 팁을 바닥(-Z) 방향으로 정렬합니다...")
        if robot.align_tcp_to_floor(yaw_deg=90.0, kp=100.0):
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

            # 1. Pick 접근
            print("\n   ▶ [Step 1] Pick 상단 5cm 위치로 접근 중...")
            if not execute_move(pick_x, pick_y, pick_z + 0.05, "Pick_Approach"): continue

            # 2. Pick 하강
            print("\n   ▶ [Step 2] Pick 지점으로 수직 하강 중...")
            if not execute_move(pick_x, pick_y, pick_z, "Pick_Reach"): continue

            # 💡 [I/O ON] 실제 로봇 그리퍼 작동
            print("\n   🧲 [I/O 제어] 그리퍼 작동 (ON) -> 대기 0.5초")
            robot.set_digital_output(9, True)
            wait_counts(500)

            # 3. Pick 상승
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

            # 4. Place 접근
            print("\n   ▶ [Step 4] Place 상단 5cm 위치로 접근 중...")
            if not execute_move(place_x, place_y, place_z + 0.05, "Place_Approach"): continue

            # 5. Place 하강
            print("\n   ▶ [Step 5] Place 지점으로 수직 하강 중...")
            if not execute_move(place_x, place_y, place_z, "Place_Reach"): continue

            # 💡 [I/O OFF] 실제 로봇 그리퍼 해제
            print("\n   👐 [I/O 제어] 그리퍼 해제 (OFF) -> 대기 0.5초")
            robot.set_digital_output(9, False)
            wait_counts(500)

            # 6. Place 상승
            print("\n   ▶ [Step 6] 10cm 위로 수직 상승 중...")
            if not execute_move(place_x, place_y, place_z + 0.10, "Place_Retract"): continue

            print("\n   🎉 1회 Pick & Place 완료!")

    except Exception as e:
        print(f"❌ Error: {e}")

    finally:
        # ==========================================================
        # 🏁 [종료] 로봇 안전 종료 및 CSV 저장
        # ==========================================================
        print("\n통신을 안전하게 종료합니다...")
        robot.stop()
        robot.servo_off()
        robot.disconnect_rt()
        robot.close_connection()

        filename = "real_pick_place_log.csv"
        with open(filename, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerows(csv_data)
        
        print(f"\n🏁 [구동 종료] 실제 하드웨어 궤적 데이터가 '{filename}'에 저장되었습니다.")

if __name__ == "__main__":
    run_real_pick_and_place()