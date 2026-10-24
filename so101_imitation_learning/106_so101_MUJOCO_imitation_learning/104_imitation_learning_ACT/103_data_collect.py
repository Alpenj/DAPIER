import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
import json
import cv2
import torch
import shutil
from pathlib import Path
from motor_control import MiniFeetechDriver
from lerobot.datasets.lerobot_dataset import LeRobotDataset

class MujocoTeleopRecorder:
    def __init__(self, leader_port, repo_id="mujoco_robot_task"):
        # 1. 하드웨어 리더 설정
        self.leader = MiniFeetechDriver(port=leader_port)
        with open("full_arm_calibration_leader.json", "r") as f:
            self.leader_cfg = json.load(f)
        
        # 2. MuJoCo 시뮬레이션 설정
        xml_path = "lift_cube_calibration.xml"
        self.model = mj.MjModel.from_xml_path(xml_path)
        self.data = mj.MjData(self.model) 
        
        self.width = 1024
        self.height = 768
        
        # 관절 정보 매핑 (리더 ID -> MJ 조인트 이름)
        self.joint_mapping = [
            {"name": "shoulder_pan",  "id": 1},
            {"name": "shoulder_lift", "id": 2},
            {"name": "elbow_flex",    "id": 3},
            {"name": "wrist_flex",    "id": 4},
            {"name": "wrist_roll",    "id": 5},
            {"name": "gripper",       "id": 6}
        ]

        # 3. LeRobot 데이터셋 설정
        self.repo_id = repo_id
        self.fps = 30
        self.dataset = self._setup_dataset()
        
        # 4. 시각화 및 레코딩 변수
        self.is_recording = False
        self._init_viewer()

       

    def _setup_dataset(self):
        dataset_path = Path(Path.home(), ".cache/huggingface/lerobot", self.repo_id)
        if dataset_path.exists():
            shutil.rmtree(dataset_path)
            
        features = {
            "observation.image": {"dtype": "video", "shape": (3, 224, 224), "names": ["color"]},
            "observation.state": {"dtype": "float32", "shape": (6,)},
            "action": {"dtype": "float32", "shape": (6,)},
        }
        return LeRobotDataset.create(repo_id=self.repo_id, fps=self.fps, features=features)

    def _init_viewer(self):
        glfw.init()
        self.window = glfw.create_window(self.width, self.height, "MuJoCo Teleop Recorder", None, None)
        glfw.make_context_current(self.window)
        self.scene = mj.MjvScene(self.model, maxgeom=1000)
        self.cam = mj.MjvCamera()
        self.cam.lookat[:] = [0.35, 0.0, 0.15] 
        self.cam.distance = 0.6
        self.cam.azimuth = 180
        self.cam.elevation = -30
        self.ctx = mj.MjrContext(self.model, mj.mjtFontScale.mjFONTSCALE_150.value)


    def reset_block_position(self):
        """매 에피소드 시작 시 파란색 블록의 위치를 무작위로 변경"""
        print(">> 블록 위치 초기화 (Randomizing...)")
        # 'blue_block' 조인트 위치 찾기
        joint_id = self.model.joint('cube_joint').qposadr[0]
        
        # 랜덤 위치 설정 (x: 0.3~0.5, y: -0.15~0.15, z: 탁자 위)
        self.data.qpos[joint_id : joint_id+3] = [
            np.random.uniform(0.3, 0.5),
            np.random.uniform(-0.15, 0.15),
            0.23
        ]
        # 속도 초기화
        self.data.qvel[:] = 0
        mj.mj_forward(self.model, self.data)

    def get_image(self):
        """MuJoCo 렌더링 화면을 224x224 RGB 이미지로 캡처"""
        rect = mj.MjrRect(0, 0, self.width, self.height)
        mj.mjr_render(rect, self.scene, self.ctx)
        
        # 픽셀 읽기 (MuJoCo는 아래에서 위로 저장하므로 상하반전 필요)
        rgb = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        mj.mjr_readPixels(rgb, None, rect, self.ctx)
        rgb = np.flipud(rgb) # 상하 반전
        
        img_resized = cv2.resize(rgb, (224, 224))
        return img_resized

    def run(self):
        print("\n[작동 가이드] r: 녹화 시작 | s: 에피소드 저장 & 블록 리셋 | q: 종료")
        # 모든 리더 모터 토크 OFF (ID 1~6 반복문 권장)
        for j in self.joint_mapping:
            self.leader.set_torque(j["id"], False)

        try:
            while not glfw.window_should_close(self.window):
                time_prev = self.data.time
                
                norm_states = []
                norm_actions = []

                # 1. 하드웨어 리더 데이터 읽기 및 시뮬레이터 제어
                for i, j_info in enumerate(self.joint_mapping):
                    l_cfg = self.leader_cfg[j_info["name"]]
                    l_pos = self.leader.get_position(l_cfg["id"])
                    
                    if l_pos is not None:
                        l_ratio = (l_pos - l_cfg["range_min"]) / (l_cfg["range_max"] - l_cfg["range_min"])
                        l_ratio = max(0.0, min(1.0, l_ratio))
                        
                        mj_id = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_JOINT, j_info["name"])
                        mj_min, mj_max = self.model.jnt_range[mj_id]
                        target_rad = l_ratio * (mj_max - mj_min) + mj_min
                        
                        self.data.ctrl[i] = 100.0 * (target_rad - self.data.qpos[i]) - 2.0 * self.data.qvel[i]
                        
                        norm_states.append(l_ratio)
                        norm_actions.append(l_ratio)

                # 2. 물리 엔진 업데이트
                while (self.data.time - time_prev) < (1.0/60.0):
                    mj.mj_step(self.model, self.data)

                # 3. 이미지 수집 및 렌더링
                mj.mjv_updateScene(self.model, self.data, mj.MjvOption(), None, self.cam, mj.mjtCatBit.mjCAT_ALL.value, self.scene)
                img_rgb = self.get_image()
                
                # 4. 레코딩 로직
                if self.is_recording and len(norm_states) == 6:
                    img_torch = torch.from_numpy(img_rgb).permute(2, 0, 1)
                    self.dataset.add_frame({
                        "observation.image": img_torch,
                        "observation.state": torch.tensor(norm_states, dtype=torch.float32),
                        "action": torch.tensor(norm_actions, dtype=torch.float32),
                        "task": "touch the object",
                    })

                # 시각화 화면 표시
                display_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
                if self.is_recording:
                    cv2.putText(display_bgr, "REC", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
                cv2.imshow("MuJoCo ACT Collector", display_bgr)
                
                # 키 이벤트 처리
                key = cv2.waitKey(1) & 0xFF
                if key == ord('r'): 
                    self.is_recording = True
                    print(">> 녹화 시작")
                elif key == ord('s'):
                    self.dataset.save_episode()
                    self.is_recording = False
                    self.reset_block_position()
                    print(">> 에피소드 저장 완료")
                elif key == ord('q'): 
                    print(">> 종료 요청됨...")
                    break

                glfw.swap_buffers(self.window)
                glfw.poll_events()

        except Exception as e:
            print(f"!! 런타임 에러 발생: {e}")

        finally:
            # --- [핵심 수정 부분] ---
            if hasattr(self, 'dataset'):
                print(">> 데이터셋 메타데이터를 저장하고 닫는 중...")
                # 만약 녹화 도중 q를 눌러 종료했다면 마지막 에피소드 저장 시도
                if self.is_recording:
                    self.dataset.save_episode()
                
                # 명시적으로 객체 삭제 -> __del__ 호출을 유도하여 meta/episodes 생성
                del self.dataset 
                print(">> 데이터셋 안전하게 닫힘.")

            glfw.terminate()
            cv2.destroyAllWindows()
            print(">> 모든 리소스가 해제되었습니다.")

if __name__ == "__main__":
    recorder = MujocoTeleopRecorder(leader_port="/dev/ttyUSB0")
    recorder.run()