import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import json
import cv2
import torch
import shutil
from pathlib import Path

from motor_control import MiniFeetechDriver
from lerobot.datasets.lerobot_dataset import LeRobotDataset
# import가 안 되면:
# from lerobot.common.datasets.lerobot_dataset import LeRobotDataset


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
        # 1. 하드웨어 리더 설정
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

        self.wrist_cam_id = mj.mj_name2id(
            self.model, mj.mjtObj.mjOBJ_CAMERA, "wrist_camera_sensor"
        )
        if self.wrist_cam_id == -1:
            raise ValueError("'wrist_camera_sensor' 카메라를 찾을 수 없습니다.")

        # 3. 레코딩 설정
        self.repo_id = repo_id
        self.fps = 30
        self.frame_dt = 1.0 / self.fps
        self.is_recording = False
        self.frame_count = 0
        self.next_record_time = 0.0
        self.task_text = "pick and place the blue block"

        self.kp = 80.0
        self.kd = 5.0

        # 화면 / 이미지 크기
        self.window_w = 1280
        self.window_h = 720
        self.img_w = 224
        self.img_h = 224
        self.pip_w = 320
        self.pip_h = 240

        self.dataset = self._setup_dataset()
        self._init_viewer()

    def _setup_dataset(self):
        dataset_path = Path(Path.home(), ".cache/huggingface/lerobot", self.repo_id)
        if dataset_path.exists():
            shutil.rmtree(dataset_path)

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
        }

        return LeRobotDataset.create(
            repo_id=self.repo_id,
            fps=self.fps,
            features=features,
        )

    def _init_viewer(self):
        glfw.init()
        self.window = glfw.create_window(
            self.window_w, self.window_h, "MuJoCo DualCam Recorder (PiP)", None, None
        )
        glfw.make_context_current(self.window)

        self.scene = mj.MjvScene(self.model, maxgeom=1000)
        self.off_scene = mj.MjvScene(self.model, maxgeom=1000)
        self.opt = mj.MjvOption()

        self.cam = mj.MjvCamera()
        self.cam.lookat[:] = [0.3, 0.0, 0.2]
        self.cam.distance = 1.0
        self.cam.azimuth = 135
        self.cam.elevation = -20

        self.ctx = mj.MjrContext(self.model, mj.mjtFontScale.mjFONTSCALE_150.value)

    def reset_block_position(self):
        joint_id = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_JOINT, "cube_joint")
        if joint_id != -1:
            qpos_adr = self.model.jnt_qposadr[joint_id]
            # 안전한 작업 영역으로 랜덤 배치
            self.data.qpos[qpos_adr] = np.random.uniform(0.30, 0.42)
            self.data.qpos[qpos_adr + 1] = np.random.uniform(0.12, 0.24)
            self.data.qpos[qpos_adr + 2] = 0
            self.data.qvel[:] = 0
            mj.mj_forward(self.model, self.data)
            print(">> 블록 위치 랜덤 리셋 완료")
        else:
            print(">> 블럭 배치 실패 ")


    def get_state(self):
        state = []
        for j in self.joint_mapping:
            mj_id = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_JOINT, j["name"])
            qpos_idx = self.model.jnt_qposadr[mj_id]
            state.append(self.data.qpos[qpos_idx])
        return np.array(state, dtype=np.float32)

    def apply_teleop(self):
        """
        리더 값을 읽어 follower를 제어하고,
        dataset에 저장할 target action(rad)을 반환
        """
        action = []
        valid_count = 0

        for i, j_info in enumerate(self.joint_mapping):
            j_name = j_info["name"]
            raw = self.leader.get_position(j_info["id"])

            if raw is None:
                action.append(0.0)
                continue

            cal = self.leader_cfg[j_name]
            norm = raw_to_norm(raw, cal)

            mj_id = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_JOINT, j_name)
            mj_min, mj_max = self.model.jnt_range[mj_id]
            qpos_idx = self.model.jnt_qposadr[mj_id]
            qvel_idx = self.model.jnt_dofadr[mj_id]

            target_rad = ((norm + 100.0) / 200.0) * (mj_max - mj_min) + mj_min

            qpos_curr = self.data.qpos[qpos_idx]
            qvel_curr = self.data.qvel[qvel_idx]

            if j_name == "gripper":
                torque = 200.0 * (target_rad - qpos_curr) - 3.0 * qvel_curr
            else:
                torque = self.kp * (target_rad - qpos_curr) - self.kd * qvel_curr

            self.data.ctrl[i] = torque
            action.append(target_rad)
            valid_count += 1

        return np.array(action, dtype=np.float32), valid_count

    def render_main_and_get_image(self):
        """
        메인 카메라 전체 화면 렌더 + dataset용 main 이미지 캡처
        """
        fb_w, fb_h = glfw.get_framebuffer_size(self.window)
        viewport = mj.MjrRect(0, 0, fb_w, fb_h)

        mj.mjv_updateScene(
            self.model,
            self.data,
            self.opt,
            None,
            self.cam,
            mj.mjtCatBit.mjCAT_ALL.value,
            self.scene,
        )
        mj.mjr_render(viewport, self.scene, self.ctx)

        rgb = np.zeros((fb_h, fb_w, 3), dtype=np.uint8)
        mj.mjr_readPixels(rgb, None, viewport, self.ctx)
        rgb = np.flipud(rgb)

        return cv2.resize(rgb, (self.img_w, self.img_h))

    def render_wrist_pip_and_get_image(self):
        """
        wrist 카메라를 PiP로 메인 화면 위에 덧그리고,
        동시에 dataset용 wrist 이미지 캡처
        """
        wrist_cam = mj.MjvCamera()
        wrist_cam.type = mj.mjtCamera.mjCAMERA_FIXED
        wrist_cam.fixedcamid = self.wrist_cam_id

        pip_rect = mj.MjrRect(0, 0, self.pip_w, self.pip_h)

        mj.mjv_updateScene(
            self.model,
            self.data,
            self.opt,
            None,
            wrist_cam,
            mj.mjtCatBit.mjCAT_ALL.value,
            self.off_scene,
        )
        mj.mjr_render(pip_rect, self.off_scene, self.ctx)

        rgb = np.zeros((self.pip_h, self.pip_w, 3), dtype=np.uint8)
        mj.mjr_readPixels(rgb, None, pip_rect, self.ctx)
        rgb = np.flipud(rgb)

        return cv2.resize(rgb, (self.img_w, self.img_h))

    def save_frame(self, main_rgb, wrist_rgb, state, action, valid_count):
        if not self.is_recording:
            return
        if valid_count != 6:
            return
        if self.data.time < self.next_record_time:
            return

        self.dataset.add_frame({
            "observation.images.main": torch.from_numpy(main_rgb.copy()).permute(2, 0, 1),
            "observation.images.wrist": torch.from_numpy(wrist_rgb.copy()).permute(2, 0, 1),
            "observation.state": torch.tensor(state, dtype=torch.float32),
            "action": torch.tensor(action, dtype=torch.float32),
            "task": self.task_text,
        })

        self.frame_count += 1
        self.next_record_time += self.frame_dt

    def save_episode(self):
        if self.frame_count > 0:
            self.dataset.save_episode()
            print(f">> 에피소드 저장 완료 ({self.frame_count} frames)")
            self.frame_count = 0
        else:
            print(">> 저장할 프레임이 없습니다.")

    def discard_episode(self):
        self.is_recording = False
        self.frame_count = 0
        print(">> [!] 에피소드 폐기됨. 다시 시작하세요.")

    def run(self):
        print("\n[작동 가이드]")
        print("  r : 녹화 시작")
        print("  s : 에피소드 저장 + 블록 리셋")
        print("  q : 종료")
        print("  GLFW 메인창: main + wrist PiP")
        print("  dataset 저장: observation.images.main / observation.images.wrist")

        for j in self.joint_mapping:
            self.leader.set_torque(j["id"], False)

        self.reset_block_position()

        try:
            while not glfw.window_should_close(self.window):
                time_prev = self.data.time

                # 1. 리더 읽기 + follower 제어
                action, valid_count = self.apply_teleop()

                # 2. 물리 엔진 업데이트
                while (self.data.time - time_prev) < (1.0 / 60.0):
                    mj.mj_step(self.model, self.data)

                # 3. state 수집
                state = self.get_state()

                # 4. 메인 뷰 렌더 + main image 캡처
                main_rgb = self.render_main_and_get_image()

                # 5. wrist PiP 렌더 + wrist image 캡처
                wrist_rgb = self.render_wrist_pip_and_get_image()

                # 6. 이제 GLFW 창에는 PiP가 올라간 상태
                glfw.swap_buffers(self.window)
                glfw.poll_events()

                # 7. 데이터 저장
                self.save_frame(main_rgb, wrist_rgb, state, action, valid_count)

                # 8. OpenCV 미리보기
                preview_main = cv2.cvtColor(main_rgb, cv2.COLOR_RGB2BGR)
                preview_wrist = cv2.cvtColor(wrist_rgb, cv2.COLOR_RGB2BGR)
                preview = cv2.hconcat([preview_main, preview_wrist])

                if self.is_recording:
                    cv2.putText(
                        preview, "REC", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2
                    )

                cv2.putText(
                    preview, f"Frames: {self.frame_count}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
                )

                cv2.imshow("MuJoCo ACT Collector (main | wrist)", preview)

                # 9. 키 입력 처리
                key = cv2.waitKey(1) & 0xFF
                if key == ord("r"):
                    if not self.is_recording:
                        self.is_recording = True
                        self.frame_count = 0
                        self.next_record_time = self.data.time
                        print(">> 녹화 시작")

                elif key == ord("s"):
                    if self.is_recording:
                        self.save_episode()
                        self.is_recording = False
                        self.reset_block_position()
                    else:
                        print(">> 현재 녹화 중이 아닙니다.")

                elif key == ord("d"): # 신규 기능
                    self.discard_episode()
                    self.reset_block_position()

                elif key == ord("q"):
                    print(">> 종료 요청됨...")
                    break

        except Exception as e:
            print(f"!! 런타임 에러 발생: {e}")

        finally:
            try:
                if hasattr(self, "dataset"):
                    print(">> 데이터셋 정리 중...")
                    if self.is_recording and self.frame_count > 0:
                        self.save_episode()

                    if hasattr(self.dataset, "finalize"):
                        self.dataset.finalize()

                    del self.dataset
                    print(">> 데이터셋 종료 완료")
            except Exception as e:
                print(f"!! 데이터셋 종료 중 에러: {e}")

            glfw.terminate()
            cv2.destroyAllWindows()
            print(">> 모든 리소스가 해제되었습니다.")


if __name__ == "__main__":
    recorder = MujocoTeleopRecorder(
        leader_port="/dev/ttyUSB0",
        repo_id="mujoco_pick_place_dualcam"
    )
    recorder.run()
