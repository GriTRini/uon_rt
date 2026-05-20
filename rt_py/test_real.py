import numpy as np
import pandas as pd
import time
import matplotlib.pyplot as plt
from robot import Robot 

def run_real_robot():
    print("🤖 [Hanwha HCR14] J1 상대각도(+10°) 반복 왕복 구동 테스트 (No Sleep)")
    
    # 1. 로봇 초기화 및 IP 세팅 (통합 인터페이스 레이어 사용)
    robot = Robot("hcr14")
    robot_ip = "192.168.100.200" 

    history = {"time": [], "q": [], "mode": []}
    start_time = [0.0]

    def log_data(mode_id):
        curr_t = time.time() - start_time[0]
        history["time"].append(curr_t)
        history["q"].append(robot.angles.copy())
        history["mode"].append(mode_id)

    try:
        # 2. 통신 연결 및 서보 온 (백엔드에서 .ini 파일 기구학 경로 자동 교정됨)
        if not robot.open_connection(robot_ip):
            print(f"❌ 로봇 연결 실패 (IP: {robot_ip})")
            return

        robot.connect_rt()
        if not robot.servo_on():
            print("❌ 서보 온 실패!")
            return

        start_time[0] = time.time()
        print("⚡ 한화 HCR14 로봇 실시간 제어 엔진 구동.")

        # 🌟 한화 로봇 기준 홈포인트 사양 설정
        home_q = np.array([1.24, -93.14, -88.68, -88.17, 90.0, -88.75])
        
        # 구동 상태 제어 변수
        # 0: 최초 홈 진입 중, 1: 홈에서 J1+10° 이동 중, 2: 다시 홈으로 복귀 중
        state = 0 
        
        print("\n▶ [시퀀스 시작] 최초 지정 홈 포인트로 이동합니다.")
        robot.trapj(home_q)

        loop_count = 0
        display_interval = 100 # I/O 병목 방지를 위한 100연산당 1출력 주사율 적용

        while True:
            loop_count += 1
            
            # 실시간 로깅 및 수렴 각도 획득
            log_data(state)
            current_j = robot.angles

            # 현재 구동 정보 터미널 스트리밍
            if loop_count % display_interval == 0:
                print(f" 🔄 [State {state}] 구동 중... 현재 J1: {current_j[0]:.2f}° | 목표 대비 오차: {abs(current_j[0] - (home_q[0] + 10.0 if state==1 else home_q[0])):.3f}°", end="\r")

            # 🌟 목표 지점 안착 상태 정밀 검사
            if robot.goal_reached(q_th=0.3, v_th=0.5):
                print("") # 터미널 줄바꿈 비우기
                
                if state == 0 or state == 2:
                    # 🏠 홈에 완벽히 도달한 경우 -> 현재 J1 각도를 실측하여 +10도 연산 후 타겟 변경
                    print(f"✅ [홈 포인트 안착] 현재 실측 J1: {current_j[0]:.2f}°")
                    
                    target_q = home_q.copy()
                    target_q[0] = current_j[0] + 10.0 # 정확히 현재 각도 기준 +10도 바인딩
                    
                    print(f"➡️ [명령 전달] J1 관절 {target_q[0]:.2f}° 타겟 포인트로 이동 개시.")
                    robot.trapj(target_q)
                    state = 1
                    
                elif state == 1:
                    # 🚀 +10도 지점에 도달한 경우 -> 다시 기준 홈 포인트로 회귀
                    print(f"✅ [J1 +10° 지점 안착] 현재 실측 J1: {current_j[0]:.2f}°")
                    print(f"➡️ [명령 전달] 다시 기준 홈 포인트({home_q[0]:.2f}°)로 복귀 개시.")
                    robot.trapj(home_q)
                    state = 2

    except KeyboardInterrupt:
        print("\n▲ 사용자에 의해 시퀀스가 안전하게 중단되었습니다 (Ctrl+C).")
    except Exception as e:
        print(f"\n🚨 런타임 예외 발생: {e}")

    finally:
        print("\n🛑 시스템 안전 종료 절차 개시")
        try:
            robot.stop()         # 1. 궤적 엔진 정지
            
            # 하드웨어가 댐핑되어 완전히 멈추도록 순수 레지스터 루프 틱 대기
            exit_wait = 0
            while exit_wait < 500000:
                exit_wait += 1
                
            robot.servo_off()    # 2. 제어 상태에서 서보 오프 안전 해제
            robot.close_connection() # 3. 소켓 세션 완전 반환
        except Exception as e:
            print(f"종료 처리 중 오류: {e}")
        print("🏁 안전 종료가 완료되었습니다.")

    # 📊 구동 데이터 그래프 시각화
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
    plt.plot(history["time"], [q[0] for q in history["q"]], label='J1 (Base Joint) Angle', color='forestgreen', linewidth=2)
    plt.axhline(y=1.24, color='gray', linestyle='--', label='Base Home Reference')
    plt.title("HCR14 J1 Relative Step Response Motion Profile (+10° Loop)")
    plt.xlabel("Time (s)")
    plt.ylabel("Joint Position (deg)")
    plt.grid(True)
    plt.legend()
    plt.show()

if __name__ == "__main__":
    run_real_robot()