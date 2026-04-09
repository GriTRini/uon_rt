from trajectory import TrajGenerator
import uon_rt as _uon
import numpy as np
import time
import csv  # 🌟 CSV 저장을 위해 추가

def test_simulation():
    print("--- TrajGenerator 시뮬레이션 및 CSV 저장 테스트 ---")
    
    # 1. 초기화
    model = _uon.RobotModel("m1013")
    gen = TrajGenerator()
    q0 = np.zeros(6)
    gen.initialize(model, q0, q0, q0)
    
    # 2. 목표 설정 (TrapJ)
    target_q = np.array([10.0, 20.0, 30.0, 0.0, 45.0, 0.0])
    print(f"목표 각도 설정: {target_q}")
    gen.trapj(target_q)
    
    # 3. 데이터 기록용 변수 준비
    dt = 0.001
    start_t = time.time()
    current_sim_time = 0.0  # 시뮬레이션 내부 시간
    
    # CSV에 저장할 데이터를 담을 리스트 (첫 줄은 헤더)
    csv_data = [["Time(s)", "J1(deg)", "J2(deg)", "J3(deg)", "J4(deg)", "J5(deg)", "J6(deg)"]]
    
    print("이동을 시작합니다. 데이터를 기록 중...")
    
    # 4. 업데이트 루프
    while not gen.goal_reached(angles_enorm_thold=0.5):
        gen.update(dt)
        current_sim_time += dt
        
        # 현재 각도 가져오기
        current_q = gen.angles()
        
        # 현재 시간과 각도를 리스트에 추가 (소수점 4자리까지 기록)
        row = [round(current_sim_time, 3)] + [round(q, 4) for q in current_q]
        csv_data.append(row)
        
        # 0.5초(시뮬레이션 시간)마다 화면에 출력
        # 부동소수점 오차를 방지하기 위해 정수로 변환 후 계산
        if int(current_sim_time * 1000) % 500 == 0:
            print(f"[Time: {current_sim_time:.3f}s] 현재 각도: {current_q.round(2)}")
            
        # 안전장치 (실제 시간 5초 초과 시 강제 종료)
        if time.time() - start_t > 5.0:
            print("타임아웃!")
            break
            
    print(f"도착 완료! 최종 각도: {gen.angles().round(2)}")

    # 5. 수집된 데이터를 CSV 파일로 저장
    filename = "trajectory_log.csv"
    with open(filename, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerows(csv_data)
        
    # 헤더를 제외한 실제 데이터 개수(-1) 출력
    print(f"궤적 데이터가 '{filename}'에 저장되었습니다. (총 {len(csv_data) - 1} 스텝 기록됨)")

if __name__ == "__main__":
    test_simulation()