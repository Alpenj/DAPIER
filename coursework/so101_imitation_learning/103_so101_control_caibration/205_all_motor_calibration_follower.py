import time
import json
from pathlib import Path
from motor_control import MiniFeetechDriver

# 1. 로봇 관절 설정 (이름과 ID 매핑)
# 실제 로봇의 연결 상태에 따라 ID를 수정하세요.
JOINT_CONFIGS = [
    {"name": "shoulder_pan",   "id": 1},
    {"name": "shoulder_lift",  "id": 2},
    {"name": "elbow_flex",     "id": 3},
    {"name": "wrist_flex",     "id": 4},
    {"name": "wrist_roll",     "id": 5},
    {"name": "gripper",        "id": 6},
]

class FullArmCalibrator:
    def __init__(self, port="/dev/ttyUSB0", baudrate=1000000):
        self.driver = MiniFeetechDriver(port=port, baudrate=baudrate)
        self.HALF_TURN = 2047  # STS3215의 중앙값

    def calibrate_joint(self, joint_name, motor_id):
        """개별 관절에 대한 칼리브레이션 프로세스"""
        print(f"\n========================================")
        print(f" >>> [{joint_name} (ID: {motor_id})] 시작")
        print(f"========================================")

        # [중요] 이전 설정값이 영향을 주지 않도록 오프셋과 리미트를 초기화
        print("  - 이전 설정 초기화 중...")
        self.driver.set_homing_offset(motor_id, 0)
        self.driver.set_position_limits(motor_id, 0, 4095)
        time.sleep(0.2) # 모터 내부 반영 시간 대기

        # 0. 토크 해제
        self.driver.set_torque(motor_id, False)

        # 1. Homing Offset 설정
        input(f"  [STEP 1] {joint_name}을 '중앙(0도)' 자세로 정렬한 후 [ENTER]")
        pos_center = self.driver.get_position(motor_id)
        if pos_center is None:
            print(f"  [오류] ID {motor_id} 로부터 위치를 읽을 수 없습니다.")
            return None

        homing_offset = int(pos_center) - self.HALF_TURN
        print(f"  - 계산된 Homing Offset: {homing_offset}")
        self.driver.set_homing_offset(motor_id, homing_offset)
        time.sleep(0.1)

        # 2. Range 측정
        input(f"  [STEP 2] {joint_name}을 '최소(Min)' 위치로 끝까지 옮긴 후 [ENTER]")
        pos_min = self.driver.get_position(motor_id)

        input(f"  [STEP 3] {joint_name}을 '최대(Max)' 위치로 끝까지 옮긴 후 [ENTER]")
        pos_max = self.driver.get_position(motor_id)

        if pos_min is None or pos_max is None:
            print(f"  [오류] 범위를 읽는 데 실패했습니다.")
            return None

        range_min = int(min(pos_min, pos_max))
        range_max = int(max(pos_min, pos_max))

        # 3. 하드웨어 리미트 저장
        print(f"  - 하드웨어 리미트 기록: {range_min} ~ {range_max}")
        self.driver.set_position_limits(motor_id, range_min, range_max)
        
        return {
            "id": motor_id,
            "drive_mode": 0,
            "homing_offset": homing_offset,
            "range_min": range_min,
            "range_max": range_max
        }

    def run(self, out_json_path="./full_arm_calibration_follower.json"):
        all_calib_data = {}

        print("6축 로봇 칼리브레이션을 시작합니다. 모든 모터의 토크를 해제합니다.")
        for joint in JOINT_CONFIGS:
            self.driver.set_torque(joint["id"], False)

        # 각 관절별로 루프 실행
        for joint in JOINT_CONFIGS:
            result = self.calibrate_joint(joint["name"], joint["id"])
            if result:
                all_calib_data[joint["name"]] = result
            else:
                print(f"!! {joint['name']} 칼리브레이션 실패. 중단하거나 건너뜁니다.")
                if input("계속하시겠습니까? (y/n): ").lower() != 'y':
                    break

        # 결과 저장
        if all_calib_data:
            Path(out_json_path).parent.mkdir(parents=True, exist_ok=True)
            with open(out_json_path, "w", encoding="utf-8") as f:
                json.dump(all_calib_data, f, indent=4, ensure_ascii=False)
            print(f"\n[성공] 모든 데이터가 저장되었습니다: {out_json_path}")

if __name__ == "__main__":
    # 포트 주소 확인 필수! (leader/follower 여부에 따라 /dev/ttyUSB0 또는 1)
    calibrator = FullArmCalibrator(port="/dev/ttyUSB1")
    calibrator.run()