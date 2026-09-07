import time
import json
import cv2
import torch
import shutil
from pathlib import Path
from motor_control import MiniFeetechDriver
from lerobot.datasets.lerobot_dataset import LeRobotDataset

class TeleopRecorder:
    def __init__(self, leader_port, follower_port, repo_id="my_robot_task"):
        # 1. 하드웨어 드라이버 설정
        self.leader = MiniFeetechDriver(port=leader_port)
        self.follower = MiniFeetechDriver(port=follower_port)
        
        # 2. 칼리브레이션 데이터 로드 (각 조인트의 범위를 알기 위함)
        with open("full_arm_calibration_leader.json", "r") as f:
            self.leader_cfg = json.load(f)
        with open("full_arm_calibration_follower.json", "r") as f:
            self.follower_cfg = json.load(f)
            
        self.joint_names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
        self.f_ids = [self.follower_cfg[name]["id"] for name in self.joint_names]

        # 3. LeRobot 데이터셋 설정
        self.repo_id = repo_id
        self.fps = 30
        self.dataset = self._setup_dataset()
        
        # 4. 제어 및 필터 변수
        self.alpha = 0.3 # EMA 필터 계수
        self.prev_goals = {name: None for name in self.joint_names}
        self.is_recording = False

    def _setup_dataset(self):
        # 기존 데이터셋 삭제 (새로 시작할 경우)
        dataset_path = Path(Path.home(), ".cache/huggingface/lerobot", self.repo_id)
        if dataset_path.exists():
            print(f"기존 데이터셋 삭제 중: {dataset_path}")
            shutil.rmtree(dataset_path)
            
        features = {
            "observation.image": {"dtype": "video", "shape": (3, 224, 224), "names": ["color"]},
            "observation.state": {"dtype": "float32", "shape": (6,)}, # 0.0 ~ 1.0 정규화된 값
            "action": {"dtype": "float32", "shape": (6,)},            # 0.0 ~ 1.0 정규화된 값
        }
        return LeRobotDataset.create(repo_id=self.repo_id, fps=self.fps, features=features)

    def run(self):
        cap = cv2.VideoCapture(0)
        # 카메라 해상도 설정 (필요시)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        print("\n[작동 가이드]")
        print("- 'r' 키: 녹화 시작 | 's' 키: 에피소드 저장 | 'q' 키: 종료")

        # 토크 ON
        for f_id in self.f_ids:
            self.follower.set_torque(f_id, True)

        try:
            while True:
                ret, frame = cap.read()
                if not ret: break

                goals_raw = []        # 모터에 보낼 실제 값 (예: 500~2500)
                norm_states = []      # 저장할 정규화된 상태 (0~1)
                norm_actions = []     # 저장할 정규화된 행동 (0~1)
                all_read_success = True

                self.leader.ser.reset_input_buffer()

                for name in self.joint_names:
                    l_cfg = self.leader_cfg[name]
                    f_cfg = self.follower_cfg[name]

                    # 1. 데이터 읽기
                    l_pos = self.leader.get_position(l_cfg["id"])
                    f_pos = self.follower.get_position(f_cfg["id"])
                    
                    if l_pos is None or f_pos is None:
                        all_read_success = False
                        break

                    # 2. 리더(입력) 정규화 및 매핑
                    # 리더의 현재 위치를 0~1 비율로 변환
                    l_ratio = (l_pos - l_cfg["range_min"]) / (l_cfg["range_max"] - l_cfg["range_min"])
                    l_ratio = max(0.0, min(1.0, l_ratio))

                    # 3. 팔로워(출력) 실제 값 계산 및 필터링
                    raw_goal = int(l_ratio * (f_cfg["range_max"] - f_cfg["range_min"]) + f_cfg["range_min"])
                    
                    if self.prev_goals[name] is None:
                        filtered_goal = raw_goal
                    else:
                        filtered_goal = int(self.alpha * raw_goal + (1 - self.alpha) * self.prev_goals[name])
                    
                    self.prev_goals[name] = filtered_goal
                    goals_raw.append(filtered_goal)

                    # 4. 저장용 정규화 데이터 생성 (0~1 범위)
                    # State: 팔로워의 현재 위치를 0~1로 변환
                    f_ratio = (f_pos - f_cfg["range_min"]) / (f_cfg["range_max"] - f_cfg["range_min"])
                    norm_states.append(max(0.0, min(1.0, f_ratio)))
                    
                    # Action: 필터링된 목표 위치를 0~1로 변환
                    target_ratio = (filtered_goal - f_cfg["range_min"]) / (f_cfg["range_max"] - f_cfg["range_min"])
                    norm_actions.append(max(0.0, min(1.0, target_ratio)))

                # 로봇 제어 및 기록
                if all_read_success and len(goals_raw) == 6:
                    self.follower.sync_write_position(self.f_ids, goals_raw)

                    if self.is_recording:
                        img_resized = cv2.resize(frame, (224, 224))
                        # OpenCV(HWC, BGR) -> PyTorch(CHW, RGB) 변환
                        img_rgb = cv2.cvtColor(img_resized, cv2.COLOR_BGR2RGB)
                        img_torch = torch.from_numpy(img_rgb).permute(2, 0, 1)
                        
                        self.dataset.add_frame({
                            "observation.image": img_torch,
                            "observation.state": torch.tensor(norm_states, dtype=torch.float32),
                            "action": torch.tensor(norm_actions, dtype=torch.float32),
                            "task": "pick up the object",
                        })

                # UI 렌더링
                display_frame = frame.copy()
                if self.is_recording:
                    cv2.putText(display_frame, "● RECORDING", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                
                cv2.imshow("Teleop & Record (Normalized)", display_frame)
                key = cv2.waitKey(1) & 0xFF

                if key == ord('r') and not self.is_recording:
                    self.is_recording = True
                    print(">> 녹화 시작")
                elif key == ord('s') and self.is_recording:
                    self.dataset.save_episode()
                    self.is_recording = False
                    print(">> 에피소드 저장 완료")
                elif key == ord('q'):
                    break

        finally:
            cap.release()
            cv2.destroyAllWindows()
            for f_id in self.f_ids:
                self.follower.set_torque(f_id, False)
            if hasattr(self, 'dataset'):
                print("데이터셋 정리 중...")
                del self.dataset

if __name__ == "__main__":
    # 포트 번호는 환경에 맞게 수정하세요.
    recorder = TeleopRecorder(leader_port="/dev/ttyUSB0", follower_port="/dev/ttyUSB1")
    recorder.run()