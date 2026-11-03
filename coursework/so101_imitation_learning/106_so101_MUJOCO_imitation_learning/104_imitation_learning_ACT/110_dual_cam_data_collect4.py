import json
import shutil
from pathlib import Path

import cv2
import mujoco as mj
import numpy as np
import torch
from mujoco.glfw import glfw

from motor_control import MiniFeetechDriver
from lerobot.datasets.lerobot_dataset import LeRobotDataset


def clip(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


def raw_to_norm(raw, cal):
    mn, mx = cal["range_min"], cal["range_max"]
    raw_b = clip(raw, mn, mx)
    norm = (((raw_b - mn) / (mx - mn)) * 200.0) - 100.0
    if cal.get("drive_mode", 0):
        norm = -norm
    return norm


class MujocoTeleopRecorder:
    def __init__(self, leader_port="/dev/ttyUSB0", repo_id="mujoco_pick_place_dualcam"):
        # 1. 하드웨어 설정
        self.leader = MiniFeetechDriver(port=leader_port)
        with open("full_arm_calibration_leader.json", "r", encoding="utf-8") as f:
            self.leader_cfg = json.load(f)

        # 2. MuJoCo 설정
        self.model = mj.MjModel.from_xml_path("lift_cube_calibration.xml")
        self.data = mj.MjData(self.model)

        self.joint_mapping = [
            {"name": "shoulder_pan",  "id": 1},
            {"name": "shoulder_lift", "id": 2},
            {"name": "elbow_flex",    "id": 3},
            {"name": "wrist_flex",    "id": 4},
            {"name": "wrist_roll",    "id": 5},
            {"name": "gripper",       "id": 6},
        ]

        # 3. 데이터셋 및 녹화 설정
        self.repo_id = repo_id
        self.fps = 30
        self.frame_dt = 1.0 / self.fps
        self.is_recording = False
        self.episode_buffer = []
        self.frame_count = 0
        self.next_record_time = 0.0
        self.task_text = "pick and place the blue block"

        self.kp, self.kd = 80.0, 5.0
        self.window_w, self.window_h = 1280, 720
        self.img_w, self.img_h = 224, 224
        self.pip_w, self.pip_h = 320, 240

        self.dataset = self._setup_dataset()
        self._init_viewer()

    def _setup_dataset(self):
        dataset_path = Path(Path.home(), ".cache/huggingface/lerobot", self.repo_id)
        if dataset_path.exists():
            shutil.rmtree(dataset_path)

        # 현재 로컬 LeRobot 버전은 task를 frame feature로 요구함
        features = {
            "observation.images.main": {
                "dtype": "video",
                "shape": (3, 224, 224),
                "names": ["channel", "height", "width"],
            },
            "observation.images.wrist": {
                "dtype": "video",
                "shape": (3, 224, 224),
                "names": ["channel", "height", "width"],
            },
            "observation.state": {
                "dtype": "float32",
                "shape": (6,),
                "names": ["joint"],
            },
            "action": {
                "dtype": "float32",
                "shape": (6,),
                "names": ["joint"],
            },
            "task": {
                "dtype": "string",
                "shape": (1,),
            },
        }

        return LeRobotDataset.create(
            repo_id=self.repo_id,
            fps=self.fps,
            features=features,
        )

    def _init_viewer(self):
        glfw.init()
        self.window = glfw.create_window(
            self.window_w,
            self.window_h,
            "MuJoCo ACT DualCam Recorder",
            None,
            None,
        )
        glfw.make_context_current(self.window)

        self.scene = mj.MjvScene(self.model, maxgeom=1000)
        self.off_scene = mj.MjvScene(self.model, maxgeom=1000)
        self.opt = mj.MjvOption()

        # 메인 카메라
        self.cam = mj.MjvCamera()
        self.cam.lookat[:] = [0.3, 0.0, 0.2]
        self.cam.distance = 1.0
        self.cam.azimuth = 135
        self.cam.elevation = -20

        # 손목 카메라 (고정)
        self.wrist_cam_id = mj.mj_name2id(
            self.model, mj.mjtObj.mjOBJ_CAMERA, "wrist_camera_sensor"
        )
        if self.wrist_cam_id == -1:
            raise ValueError("'wrist_camera_sensor'를 찾을 수 없습니다.")

        self.wrist_cam = mj.MjvCamera()
        self.wrist_cam.type = mj.mjtCamera.mjCAMERA_FIXED
        self.wrist_cam.fixedcamid = self.wrist_cam_id

        self.ctx = mj.MjrContext(
            self.model,
            mj.mjtFontScale.mjFONTSCALE_150.value,
        )

    def reset_block_position(self):
        joint_id = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_JOINT, "block_joint")
        if joint_id != -1:
            qpos_adr = self.model.jnt_qposadr[joint_id]

            # 작업 영역 랜덤 배치
            self.data.qpos[qpos_adr] = np.random.uniform(0.30, 0.42)
            self.data.qpos[qpos_adr + 1] = np.random.uniform(-0.12, 0.12)
            self.data.qpos[qpos_adr + 2] = 0.23

            self.data.qvel[:] = 0
            mj.mj_forward(self.model, self.data)
            print(">> [Reset] 블록 위치 랜덤 리셋 완료")

    def discard_episode(self):
        self.is_recording = False
        self.episode_buffer = []
        self.frame_count = 0
        print(">> [!] 에피소드 폐기 완료. 저장되지 않았습니다.")

    def get_state(self):
        state = []
        for j in self.joint_mapping:
            mj_id = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_JOINT, j["name"])
            qpos_idx = self.model.jnt_qposadr[mj_id]
            state.append(self.data.qpos[qpos_idx])
        return np.array(state, dtype=np.float32)

    def apply_teleop(self):
        action = []

        for i, j_info in enumerate(self.joint_mapping):
            raw = self.leader.get_position(j_info["id"])
            if raw is None:
                action.append(0.0)
                continue

            cal = self.leader_cfg[j_info["name"]]
            norm = raw_to_norm(raw, cal)

            mj_id = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_JOINT, j_info["name"])
            mj_min, mj_max = self.model.jnt_range[mj_id]

            target_rad = ((norm + 100.0) / 200.0) * (mj_max - mj_min) + mj_min
            qpos_curr = self.data.qpos[self.model.jnt_qposadr[mj_id]]
            qvel_curr = self.data.qvel[self.model.jnt_dofadr[mj_id]]

            if j_info["name"] == "gripper":
                torque = 200.0 * (target_rad - qpos_curr) - 3.0 * qvel_curr
            else:
                torque = self.kp * (target_rad - qpos_curr) - self.kd * qvel_curr

            self.data.ctrl[i] = torque
            action.append(target_rad)

        return np.array(action, dtype=np.float32)

    def render_and_capture(self, camera_obj, scene_obj, rect):
        mj.mjv_updateScene(
            self.model,
            self.data,
            self.opt,
            None,
            camera_obj,
            mj.mjtCatBit.mjCAT_ALL.value,
            scene_obj,
        )
        mj.mjr_render(rect, scene_obj, self.ctx)

        rgb = np.zeros((rect.height, rect.width, 3), dtype=np.uint8)
        mj.mjr_readPixels(rgb, None, rect, self.ctx)
        rgb = np.flipud(rgb)
        return cv2.resize(rgb, (self.img_w, self.img_h))

    def build_frame(self, main_rgb, wrist_rgb, state, action):
        # 중요: task는 문자열로 넣어야 함. 리스트로 넣으면 안 됨.
        frame = {
            "observation.images.main": torch.from_numpy(main_rgb.copy()).permute(2, 0, 1),
            "observation.images.wrist": torch.from_numpy(wrist_rgb.copy()).permute(2, 0, 1),
            "observation.state": torch.tensor(state, dtype=torch.float32),
            "action": torch.tensor(action, dtype=torch.float32),
            "task": self.task_text,
        }
        return frame

    def save_frame(self, main_rgb, wrist_rgb, state, action):
        if not self.is_recording:
            return
        if self.data.time < self.next_record_time:
            return

        frame = self.build_frame(main_rgb, wrist_rgb, state, action)
        self.episode_buffer.append(frame)

        self.frame_count += 1
        self.next_record_time += self.frame_dt

    def save_episode(self):
        if len(self.episode_buffer) == 0:
            print(">> 저장할 데이터가 없습니다.")
            return

        for frame in self.episode_buffer:
            self.dataset.add_frame(frame)

        self.dataset.save_episode()
        print(f">> 에피소드 저장 완료 ({len(self.episode_buffer)} frames)")

        self.episode_buffer = []
        self.frame_count = 0

    def run(self):
        print("\n[가이드] r:녹화시작 | s:저장&리셋 | d:폐기&리셋 | q:종료")

        for j in self.joint_mapping:
            self.leader.set_torque(j["id"], False)

        self.reset_block_position()

        try:
            while not glfw.window_should_close(self.window):
                time_prev = self.data.time

                # 1. 텔레오프 적용
                action = self.apply_teleop()

                # 2. 물리 시뮬레이션
                while (self.data.time - time_prev) < (1.0 / 60.0):
                    mj.mj_step(self.model, self.data)

                # 3. 메인 렌더링
                fb_w, fb_h = glfw.get_framebuffer_size(self.window)
                main_rect = mj.MjrRect(0, 0, fb_w, fb_h)
                main_rgb = self.render_and_capture(self.cam, self.scene, main_rect)

                # 4. wrist PiP 렌더링
                pip_rect = mj.MjrRect(0, 0, self.pip_w, self.pip_h)
                wrist_rgb = self.render_and_capture(self.wrist_cam, self.off_scene, pip_rect)

                glfw.swap_buffers(self.window)
                glfw.poll_events()

                # 5. 데이터 저장
                state = self.get_state()
                self.save_frame(main_rgb, wrist_rgb, state, action)

                # 6. OpenCV preview
                preview = cv2.hconcat([
                    cv2.cvtColor(main_rgb, cv2.COLOR_RGB2BGR),
                    cv2.cvtColor(wrist_rgb, cv2.COLOR_RGB2BGR),
                ])

                if self.is_recording:
                    cv2.putText(
                        preview,
                        "REC",
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (0, 0, 255),
                        2,
                    )

                cv2.putText(
                    preview,
                    f"Frames: {self.frame_count}",
                    (10, 65),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                )

                cv2.imshow("Main | Wrist", preview)

                # 7. 키 입력
                key = cv2.waitKey(1) & 0xFF

                if key == ord("r"):
                    self.is_recording = True
                    self.episode_buffer = []
                    self.frame_count = 0
                    self.next_record_time = self.data.time
                    print(">> 녹화 시작")

                elif key == ord("s"):
                    self.save_episode()
                    self.is_recording = False
                    self.reset_block_position()

                elif key == ord("d"):
                    self.discard_episode()
                    self.reset_block_position()

                elif key == ord("q"):
                    break

        finally:
            try:
                if hasattr(self, "dataset") and hasattr(self.dataset, "finalize"):
                    self.dataset.finalize()
            except Exception as e:
                print(f">> dataset.finalize() 중 예외: {e}")

            glfw.terminate()
            cv2.destroyAllWindows()
            print(">> 종료 완료")


if __name__ == "__main__":
    MujocoTeleopRecorder().run()
