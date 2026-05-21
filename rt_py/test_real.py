# test_real.py
import numpy as np
import pandas as pd
import time
import matplotlib.pyplot as plt
from robot import Robot  # 🌟 클래스 구조로 명확히 임포트

def wait_until_reached(robot, state_id, history, start_time):
    """
    로봇이 목표에 도달할 때까지 대기하며 데이터를 로깅하는 헬퍼 함수
    """
    # 🌟 [핵심] 명령을 내린 직후 C++ 궤적 생성기가 반응하고 
    # 실제 로봇이 움직이기 시작할 때까지 0.1초의 유예 시간을 줍니다. (중복 도착 판정 방지)
    time.sleep(0.1)
    
    while True:
        # 데이터 로깅
        curr_t = time.time() - start_time[0]
        history["time"].append(curr_t)
        history["q"].append(robot.angles.copy())
        history["mode"].append(state_id)
        
        # 🌟 안착 여부 판단 (질문자님 기존 로직)
        if robot.goal_reached(q_th=0.3, v_th=0.5):
            break
            
        time.sleep(0.005) # 5ms 대기 (소켓/CPU 점유 방지)

def run_real_robot():
    print("🤖 [Hanwha HCR14] J1 상대각도(+10°) 반복 왕복 구동 테스트")
    
    robot = Robot("hcr14")
    robot_ip = "192.168.100.200" 

    history = {"time": [], "q": [], "mode": []}
    start_time = [0.0]

    try:
        if not robot.open_connection(robot_ip):
            print(f"❌ 로봇 연결 실패 (IP: {robot_ip})")
            return

        robot.connect_rt()
        if not robot.servo_on():
            print("❌ 서보 온 실패!")
            return

        start_time[0] = time.time()
        print("⚡ 한화 HCR14 로봇 실시간 제어 엔진 구동.")

        # 현재 위치 기반 홈 위치 설정 (초기 튐 방지)
        current_q = robot.angles.copy()
        home_q = np.array([current_q[0], -93.14, -88.68, -88.17, 90.0, -88.75])
        
        target_q = home_q.copy()
        target_q[0] += 10.0  # J1 +10도 타겟
        
        # ==========================================================
        # 🚀 순차적 이동 시퀀스 (while True 제거)
        # ==========================================================
        
        # 1. 초기 홈 위치 이동
        print(f"\n▶ [시퀀스 시작] 최초 지정 홈 포인트({home_q[0]:.2f}°)로 이동합니다.")
        robot.trapj(home_q)
        wait_until_reached(robot, 0, history, start_time)
        print(f"✅ [홈 포인트 안착] 현재 실측 J1: {robot.angles[0]:.2f}°")

        # 2. 왕복 구동 루프 (원하는 횟수만큼 반복, 예: 3회)
        for i in range(3):
            print(f"\n--- [왕복 {i+1}회차] ---")
            
            # 타겟 이동
            print(f"➡️ [명령 전달] J1 관절 {target_q[0]:.2f}° 타겟 포인트로 이동 개시.")
            robot.trapj(target_q)
            wait_until_reached(robot, 1, history, start_time)
            print(f"✅ [J1 +10° 지점 안착] 현재 실측 J1: {robot.angles[0]:.2f}°")

            # 홈 복귀
            print(f"➡️ [명령 전달] 다시 기준 홈 포인트({home_q[0]:.2f}°)로 복귀 개시.")
            robot.trapj(home_q)
            wait_until_reached(robot, 2, history, start_time)
            print(f"✅ [홈 포인트 안착] 현재 실측 J1: {robot.angles[0]:.2f}°")

    except KeyboardInterrupt:
        print("\n\n▲ 사용자에 의해 시퀀스가 안전하게 중단되었습니다 (Ctrl+C).")
    except Exception as e:
        print(f"\n\n🚨 런타임 예외 발생: {e}")

    finally:
        print("\n🛑 시스템 안전 종료 절차 개시")
        try:
            robot.stop()
            time.sleep(0.5) # 감속 대기
            robot.servo_off()
            robot.close_connection() 
        except Exception as e:
            print(f"종료 처리 중 오류: {e}")
        print("🏁 안전 종료가 완료되었습니다.")

    if history["time"]:
        save_and_plot(history)

def save_and_plot(history):
    df = pd.DataFrame({
        "time": history["time"],
        "j1": [q[0] for q in history["q"]],
        "mode": history["mode"]
    })
    df.to_csv("hcr14_relative_j1_log.csv", index=False)
    print("💾 로그 저장 완료: hcr14_relative_j1_log.csv")

    plt.figure(figsize=(10, 5))
    plt.plot(history["time"], [q[0] for q in history["q"]], label='J1 Angle', color='forestgreen', linewidth=2)
    plt.axhline(y=1.24, color='gray', linestyle='--')
    plt.title("HCR14 J1 Step Response Profile")
    plt.xlabel("Time (s)"); plt.ylabel("Joint Position (deg)"); plt.grid(True); plt.legend()
    plt.show()

if __name__ == "__main__":
    run_real_robot()