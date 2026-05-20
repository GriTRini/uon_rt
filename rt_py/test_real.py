import time
import numpy as np
from robot import create_robot

class AppleRobotController:
    def __init__(self):
        self.robot = create_robot("hcr14")
        self.robot_ip = "192.168.100.200"
        self.q_home = np.array([28.81, -93.14, -88.68, -88.17, 90.0, -88.75])

    def wait_until_reached(self):
        """while 문 대신 goal_reached를 체크하는 단순 실행 로직"""
        # 명령 전달 직후 로봇의 제어기가 반응할 시간을 줌
        time.sleep(0.5) 
        
        # 로봇이 목표에 도달했는지 확인
        if self.robot.goal_reached(q_th=0.1, p_th=0.002):
            return True
        else:
            # 도달하지 않았을 경우 재시도 (재귀 호출)
            time.sleep(0.1)
            return self.wait_until_reached()

    def run(self):
        if not self.robot.open_connection(self.robot_ip):
            return
        
        self.robot.connect_rt()
        self.robot.servo_on()
        
        # 1. Trapj 실행 및 대기
        print("Moving to Home (trapj)...")
        self.robot.trapj(self.q_home)
        self.wait_until_reached()
        print("Home reached.")

        # 2. Attrl 실행 및 대기
        print("Moving downward (attrl)...")
        target_tmat = self.robot.tmat.copy()
        target_tmat[2, 3] -= 0.1
        self.robot.attrl(target_tmat, kp=100.0)
        self.wait_until_reached()
        print("Downward move finished.")

        self.shutdown()

    def shutdown(self):
        self.robot.servo_off()
        self.robot.close_connection()
        print("Robot disconnected.")

if __name__ == "__main__":
    controller = AppleRobotController()
    controller.run()