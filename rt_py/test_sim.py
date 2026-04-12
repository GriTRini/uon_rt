import sys
import os
import numpy as np

# 빌드된 .so 파일 경로 추가
sys.path.append(os.path.join(os.getcwd(), 'rt_py'))

try:
    import rt_control_cpp_impl as rc
    print("✅ 모듈 로드 성공!")
except ImportError as e:
    print(f"❌ 모듈 로드 실패: {e}")
    sys.exit(1)

def simulate():
    print("--- 🤖 Trajectory Simulator (No Hardware) ---")
    
    # 1. 모델 로드
    model = rc.RobotModel("m1013")
    print(f"Robot Loaded: {model.get_model_name()}")

    # 2. 제너레이터 초기화 (현재 각도 0도에서 시작)
    gen = rc.TrajGenerator()
    q_start = np.zeros(6)
    gen.initialize(model, q_start, q_start, q_start)
    
    print(f"Initial Pose Matrix:\n{gen.tmat}")

    # 3. 궤적 생성 (TrapJ: 1번 조인트 45도 이동)
    target_q = np.array([45.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    gen.trapj(target_q)
    
    # 4. 루프 시뮬레이션
    dt = 0.001 # 1ms
    for i in range(2000):
        gen.update(dt)
        
        if i % 200 == 0:
            print(f"Time {i}ms: Angles = {gen.angles}")
            
        if gen.goal_reached():
            print(f"🎯 Goal Reached at {i}ms!")
            break

    print(f"Final Pose Matrix:\n{gen.tmat}")

if __name__ == "__main__":
    simulate()