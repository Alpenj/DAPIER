import time
import json
from pathlib import Path
import numpy as np
import mujoco as mj
from motor_control import MiniFeetechDriver

# 1. 로봇 관절 설정 (MuJoCo XML의 조인트 이름과 실제 모터 ID 매칭)
JOINT_CONFIGS = [
    {"name": "shoulder_pan",   "id": 1},
    {"name": "shoulder_lift",  "id": 2},
    {"name": "elbow_flex",     "id": 3},
    {"name": "wrist_flex",     "id": 4},
    {"name": "wrist_roll",     "id": 5},
    {"name": "gripper",        "id": 6},
]

class MujocoBridgeCalibrator:
    def __init__(self, xml_path="scene.xml", port="/dev/ttyUSB0", baudrate=1000000):
        # 실제 리더 드라이버
        self.driver = MiniFeetechDriver(port=port, baudrate=baudrate)
        self.HALF_TURN = 2047 
        
        # MuJoCo 모델 로드 (시뮬레이션 관절 범위를 자동으로 가져오기 위함)
        self.mj_model = mj.MjModel.from_xml_path(xml_path)
        print(f"MuJoCo 모델 '{xml_path}' 로드 완료.")

    def get_mj_joint_range(self, joint_name):
        """MuJoCo XML에 정의된 조인트의 라디안 범위를 가져옵니다."""
        try:
            joint_id = mj.mj_name2id(self.mj_model, mj.mjtObj.mjOBJ_JOINT, joint_name)
            return self.mj_model.jnt_range[joint_id] # [min_rad, max_rad]
        except:
            print(f" [주의] MuJoCo 모델에서 {joint_name}을 찾을 수 없습니다.")
            return [-1.57, 1.57] # 기본값

    def calibrate_sync(self, joint_name, motor_id):
        print(f"\n>>> [{joint_name}] 칼리브레이션 (Real Leader -> Sim Follower)")
        
        # 초기화
        self.driver.set_torque(motor_id, False)
        self.driver.set_homing_offset(motor_id, 0)
        self.driver.set_position_limits(motor_id, 0, 4095)
        time.sleep(0.1)

        # STEP 1: 중앙(0도) 설정
        input(f"  [STEP 1] 리더를 '중앙(0도/MuJoCo 0rad)' 자세로 정렬 후 [ENTER]")
        pos_center = self.driver.get_position(motor_id)
        print("position center: ", pos_center)
        homing_offset = int(pos_center) - self.HALF_TURN
        print("homing offset: ", homing_offset)

        # STEP 2: 리미트 설정 (실제 로봇의 가동 한계 측정)
        input(f"  [STEP 2] 리더를 '최소(Min)' 한계 위치로 옮긴 후 [ENTER]")
        pos_min = self.driver.get_position(motor_id)
        print("min pos: ", pos_min)

        input(f"  [STEP 3] 리더를 '최대(Max)' 한계 위치로 옮긴 후 [ENTER]")
        pos_max = self.driver.get_position(motor_id)
        print("max pos: ", pos_max)

        mj_range = self.get_mj_joint_range(joint_name)
        print("MUJOCO range: ", mj_range)

        return {
            "id": motor_id,
            "homing_offset": homing_offset,
            "hw_range_min": int(min(pos_min, pos_max)),
            "hw_range_max": int(max(pos_min, pos_max)),
            "sim_range_min": float(mj_range[0]), # MuJoCo 라디안 최소
            "sim_range_max": float(mj_range[1])  # MuJoCo 라디안 최대
        }   
    
    def run(self, out_path="./hybrid_calibration.json"):
        calib_results = {}
        for joint in JOINT_CONFIGS:
            res = self.calibrate_sync(joint["name"], joint["id"])
            if res:
                calib_results[joint["name"]] = res
            print(calib_results)
        with open(out_path, "w", encoding="utf-8") as f:
            json.dump(calib_results, f, indent=4, ensure_ascii=False)
        print(f"\n[완료] 하이브리드 칼리브레이션 파일 저장됨: {out_path}")

if __name__ == "__main__":
    # scene.xml 경로와 USB 포트를 확인하세요.
    calibrator = MujocoBridgeCalibrator(xml_path="scene.xml", port="/dev/ttyUSB0")
    calibrator.run()
    #print(calibrator.get_mj_joint_range("shoulder_pan"))