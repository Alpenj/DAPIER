import time
import json
from motor_control import MiniFeetechDriver

class Teleoperator:
    def __init__(self, leader_port="/dev/ttyUSB0", follower_port="/dev/ttyUSB1", config_dir="./"):
        self.leader   = MiniFeetechDriver(port=leader_port)
        self.follower = MiniFeetechDriver(port=follower_port)
        self.leader_cfg   = self._load_config(config_dir + "full_arm_calibration_leader.json")
        self.follower_cfg = self._load_config(config_dir + "full_arm_calibration_follower.json")
        self.joint_names  = ["shoulder_pan", "shoulder_lift", "elbow_flex",
                             "wrist_flex", "wrist_roll", "gripper"]

    def _load_config(self, path):
        with open(path) as f:
            return json.load(f)

    def normalize(self, value, min_val, max_val):
        if max_val == min_val: return 0.5
        value = max(min(value, max_val), min_val)
        return (value - min_val) / (max_val - min_val)

    def denormalize(self, ratio, min_val, max_val):
        return int(ratio * (max_val - min_val) + min_val)

    def run(self):
        print("텔레오퍼레이션 시작 (Ctrl+C로 종료)")

        for name in self.joint_names:
            self.leader.set_torque(self.leader_cfg[name]["id"], False)
            self.follower.set_torque(self.follower_cfg[name]["id"], True)

        # 루프 속도 측정용
        t_prev = time.time()
        loop_count = 0

        try:
            while True:
                # ---- 1) 리더 6관절 읽기 (read만 모아서) ----
                l_positions = {}
                for name in self.joint_names:
                    pos = self.leader.get_position(self.leader_cfg[name]["id"])
                    if pos is not None:
                        l_positions[name] = pos

                # ---- 2) 팔로워 6관절 쓰기 (write만 모아서) ----
                for name in self.joint_names:
                    if name not in l_positions:
                        continue
                    l_cfg = self.leader_cfg[name]
                    f_cfg = self.follower_cfg[name]

                    ratio  = self.normalize(l_positions[name], l_cfg["range_min"], l_cfg["range_max"])
                    f_goal = self.denormalize(ratio, f_cfg["range_min"], f_cfg["range_max"])
                    self.follower.set_position(f_cfg["id"], f_goal)

                # ---- 루프 속도 출력 (100루프마다) ----
                loop_count += 1
                if loop_count % 100 == 0:
                    now = time.time()
                    hz = 100 / (now - t_prev)
                    print(f"  loop: {hz:.1f} Hz")
                    t_prev = now

        except KeyboardInterrupt:
            print("\n종료 중...")
            for name in self.joint_names:
                self.follower.set_torque(self.follower_cfg[name]["id"], False)

if __name__ == "__main__":
    teleop = Teleoperator(
        leader_port="/dev/ttyACM0",
        follower_port="/dev/ttyACM1",
        config_dir="./"
    )
    teleop.run()
