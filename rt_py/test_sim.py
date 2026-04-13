import sys
import os
import numpy as np
import matplotlib.pyplot as plt

# 1. 빌드된 .so 파일이 있는 경로를 시스템 경로에 추가
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

try:
    import rt_control_cpp_impl as rc
    print("✅ C++ 모듈 로드 성공!")
except ImportError as e:
    print(f"❌ 모듈 로드 실패: {e}")
    sys.exit(1)

def run_trajectory_test():
    print("--- 🤖 Trajectory Simulation Start ---")
    
    # 2. 모델 및 제너레이터 초기화
    # C++에서 바인딩한 RobotModel과 TrajGenerator를 사용합니다.
    model = rc.RobotModel("m1013")
    gen = rc.TrajGenerator()
    
    # 초기 상태 (모든 관절 0도)
    q_start = np.zeros(6)
    dq_start = np.zeros(6)
    ddq_start = np.zeros(6)
    
    gen.initialize(model, q_start, dq_start, ddq_start)
    
    # 3. 목표 각도 설정 (TrapJ)
    # 예: 1번 관절 45도, 3번 관절 60도, 5번 관절 90도로 이동
    target_q = np.array([45.0, 0.0, 60.0, 0.0, 90.0, 0.0])
    gen.trapj(target_q)
    
    # 4. 시뮬레이션 데이터 수집
    history_q = []
    history_t = []
    
    dt = 0.001  # 1ms 제어 주기
    total_steps = 2000 # 최대 2초 시뮬레이션
    
    print(f"목표 각도로 이동 중: {target_q}")
    
    for step in range(total_steps):
        t = step * dt
        gen.update(dt) # C++ 엔진의 수학 연산 호출
        
        # 데이터 저장 (gen.angles는 C++에서 NumPy 배열로 변환되어 넘어옴)
        history_q.append(gen.angles.copy())
        history_t.append(t)
        
        # 목표 도달 확인 (C++의 goal_reached 호출)
        if gen.goal_reached(q_th=0.01):
            print(f"🎯 목표 도달! 시간: {t:.3f}초")
            break
            
    # 5. 결과 시각화
    history_q = np.array(history_q)
    
    plt.figure(figsize=(10, 6))
    for i in range(6):
        plt.plot(history_t, history_q[:, i], label=f'Joint {i+1}')
    
    plt.title('Robot Trajectory Simulation (C++ Engine)')
    plt.xlabel('Time (s)')
    plt.ylabel('Joint Angle (deg)')
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    run_trajectory_test()