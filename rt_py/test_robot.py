from robot import Robot
import numpy as np
import time
import csv

def test_real_robot():
    print("--- 실제 로봇 연결 및 CSV 저장 테스트 (카운트 기반) ---")
    
    # 1. 로봇 객체 초기화
    model_name = "m1013"
    try:
        robot = Robot(model_name)
    except RuntimeError as e:
        print(e)
        return

    # 🌟 로봇 IP 주소 설정 (실제 환경에 맞게 수정)
    ROBOT_IP = "192.168.1.30" 
    
    print(f"[{ROBOT_IP}] 로봇에 연결을 시도합니다...")
    
    # 2. 통신 연결
    if not robot.open_connection(ROBOT_IP, 12345):
        print("일반 통신 연결 실패! IP와 케이블을 확인하세요.")
        return
        
    if not robot.connect_rt(ROBOT_IP, 12347):
        print("실시간(RT) 통신 연결 실패!")
        robot.close_connection()
        return
        
    print("연결 성공! 서보 모터를 켭니다.")
    robot.servo_on()
    
    # [개선] 서보가 완전히 켜질 때까지 논블로킹으로 대기 (약 1초 = 1000ms)
    servo_wait_dt = 0.001
    servo_wait_count = 0
    while servo_wait_count < 1000:
        time.sleep(servo_wait_dt)
        servo_wait_count += 1
    
    # 3. 목표 설정 및 이동 명령 (TrapJ)
    target_q = np.array([-90.0, 0.0, -90.0, 0.0, -90.0, 0.0])
    print(f"목표 각도 설정: {target_q}")
    
    # 로봇에게 이동 명령 하달 (Non-blocking)
    if not robot.trapj(target_q):
        print("이동 명령 전달 실패!")
        robot.servo_off()
        robot.disconnect_rt()
        robot.close_connection()
        return
    
    # 4. 데이터 기록용 변수 준비
    dt = 0.005  # 통신 및 데이터 수집 주기 (예: 5ms)
    max_timeout_sec = 10.0
    max_steps = int(max_timeout_sec / dt)
    step_count = 0
    
    csv_data = [["Time(s)", "J1(deg)", "J2(deg)", "J3(deg)", "J4(deg)", "J5(deg)", "J6(deg)"]]
    
    print("이동을 시작합니다. 카운트 기반으로 데이터를 기록 중...")
    
    # 5. 상태 모니터링 및 로깅 루프 (카운트 기반)
    while not robot.get_goal_reached(q_th=0.5):
        # 🌟 하드웨어에서 실제 현재 각도 읽어오기
        current_q = robot.get_current_angles()
        
        current_time_sec = step_count * dt
        
        if current_q is not None:
            # CSV 기록 (소수점 이하 반올림)
            row = [round(current_time_sec, 3)] + [round(q, 4) for q in current_q]
            csv_data.append(row)
            
            # 대략 0.5초마다 화면에 출력 (0.5 / dt = 스텝 수)
            print_interval_steps = int(0.5 / dt)
            if step_count % print_interval_steps == 0:
                print(f"[Step: {step_count} | Time: {current_time_sec:.3f}s] 실제 각도: {current_q.round(2)}")
                
        # 타임아웃 처리 (스텝 수 초과)
        if step_count >= max_steps:
            print(f"타임아웃 발생! ({max_timeout_sec}초 초과). 로봇을 강제 정지합니다.")
            robot.stop()
            break
            
        # 다음 스텝 진행 (여기서만 짧게 sleep 하여 루프 주기를 맞춤)
        # ※ 실제 엄격한 RT 제어라면 time.sleep 대신 RTOS 타이머를 써야 하지만,
        # 파이썬 레벨에서는 이 정도의 짧은 대기가 최선입니다.
        time.sleep(dt)
        step_count += 1
        
    # 최종 도착 확인
    final_q = robot.get_current_angles()
    if final_q is not None:
        print(f"도착 완료! 총 스텝: {step_count}, 최종 실제 각도: {final_q.round(2)}")

    # 6. 통신 종료 및 안전 처리
    print("통신을 종료합니다.")
    robot.servo_off()
    robot.disconnect_rt()
    robot.close_connection()

    # 7. 수집된 데이터를 CSV 파일로 저장
    filename = "real_robot_trajectory_log.csv"
    with open(filename, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerows(csv_data)
        
    print(f"실제 로봇 궤적 데이터가 '{filename}'에 저장되었습니다. (총 {len(csv_data) - 1} 스텝 기록됨)")

if __name__ == "__main__":
    test_real_robot()