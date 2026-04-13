import sys
import os
import numpy as np
import time

# 1. 경로 설정 및 모듈 로드
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

print("[Step 1] 모듈 로드 시도...", flush=True)
try:
    import rt_control_cpp_impl as rc
    print("✅ 모듈 로드 성공", flush=True)
except Exception as e:
    print(f"❌ 모듈 로드 실패: {e}", flush=True)
    sys.exit(1)

def debug_trajectory():
    # 2. RobotModel 생성 단계
    print("\n[Step 2] RobotModel('m1013') 생성 시작...", flush=True)
    try:
        model = rc.RobotModel("m1013")
        print("✅ RobotModel 생성 성공", flush=True)
    except Exception as e:
        print(f"❌ RobotModel 생성 중 에러: {e}", flush=True)
        return

    # 3. TrajGenerator 생성 단계
    print("\n[Step 3] TrajGenerator() 생성 시작...", flush=True)
    try:
        gen = rc.TrajGenerator()
        print("✅ TrajGenerator 생성 성공", flush=True)
    except Exception as e:
        print(f"❌ TrajGenerator 생성 중 에러: {e}", flush=True)
        return

    # 4. 입력 데이터 준비 및 타입 확인
    print("\n[Step 4] 입력 데이터 준비 중...", flush=True)
    q = np.zeros(6, dtype=np.float64)
    print(f"데이터 타입: {q.dtype}, 형상: {q.shape}", flush=True)

    # 5. Initialize 호출 (가장 유력한 크래시 지점)
    print("\n[Step 5] gen.initialize() 호출 직전...", flush=True)
    # 여기서 죽는다면 C++ initialize() 함수 내부의 update_subordinates()가 원인입니다.
    gen.initialize(model, q, q, q)
    print("✅ gen.initialize() 호출 완료", flush=True)

    # 6. TrapJ 명령 호출
    print("\n[Step 6] gen.trapj() 호출 직전...", flush=True)
    target_q = np.array([10.0, 10.0, 10.0, 0, 0, 0], dtype=np.float64)
    gen.trapj(target_q)
    print("✅ gen.trapj() 호출 완료", flush=True)

    # 7. 단일 루프 업데이트 테스트
    print("\n[Step 7] gen.update(0.001) 1회 호출 시도...", flush=True)
    gen.update(0.001)
    print("✅ gen.update() 호출 완료", flush=True)

    # 8. 속성 접근 테스트
    print("\n[Step 8] gen.angles 속성 읽기 시도...", flush=True)
    curr_q = gen.angles
    print(f"✅ 읽기 성공: {curr_q}", flush=True)

    print("\n✨ 모든 기본 디버깅 단계 통과!", flush=True)

if __name__ == "__main__":
    debug_trajectory()