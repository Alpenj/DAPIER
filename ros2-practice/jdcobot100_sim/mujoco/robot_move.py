#!/usr/bin/env python3
"""pose_A / pose_B를 번갈아 목표로 주는 제어 예제 (교재 10장).

교재 코드와 달라진 점 하나: position actuator를 쓰면서 data.ctrl에
"PD로 계산한 토크"를 넣지 않는다. position actuator의 ctrl은 목표 위치이고
kp/kv는 XML의 sg90_act 클래스가 이미 갖고 있다(교재 15장).

교재 방식이 실제로 어떻게 되는지 보고 싶으면 --style torque 로 실행한다.
그러면 ctrl에 들어간 값이 ctrlrange로 잘려나가는 과정을 콘솔에 찍는다.
"""

from __future__ import annotations

import argparse
import os
import sys

import mujoco
import numpy as np
import glfw

HERE = os.path.dirname(os.path.abspath(__file__))
XML = os.path.join(HERE, "scene.xml")

# 관절 순서: dof_base, dof_shoulder, dof_elbow, dof_wrist_pitch
# 관절 범위가 +-0.5236 rad(+-30도)라 그 안에서 고른 자세다.
POSE_A = np.array([0.0, 0.0, 0.0, 0.0])
POSE_B = np.array([0.45, 0.35, -0.40, 0.30])


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--style", choices=["position", "torque"], default="position",
                    help="position=ctrl에 목표각(정석), torque=교재 10장 방식")
    ap.add_argument("--interval", type=float, default=2.0, help="자세 전환 주기(초)")
    ap.add_argument("--kp", type=float, default=500.0, help="--style torque 전용")
    ap.add_argument("--kd", type=float, default=10.0, help="--style torque 전용")
    args = ap.parse_args()

    model = mujoco.MjModel.from_xml_path(XML)
    data = mujoco.MjData(model)
    ctrl_lo = model.actuator_ctrlrange[:, 0].copy()
    ctrl_hi = model.actuator_ctrlrange[:, 1].copy()

    if not glfw.init():
        sys.exit("GLFW를 초기화할 수 없습니다. (DISPLAY 확인)")
    window = glfw.create_window(960, 720, f"jdcobot100 two-pose ({args.style})", None, None)
    glfw.make_context_current(window)
    glfw.swap_interval(1)

    scene = mujoco.MjvScene(model, maxgeom=2000)
    cam = mujoco.MjvCamera()
    opt = mujoco.MjvOption()
    ctx = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150.value)
    mujoco.mjv_defaultCamera(cam)
    mujoco.mjv_defaultOption(opt)
    cam.lookat[:] = [0.0, 0.0, 0.10]
    cam.distance = 0.55
    cam.azimuth = 135.0
    cam.elevation = -12.0

    last_pose = None
    while not glfw.window_should_close(window):
        frame_start = data.time
        while data.time - frame_start < 1.0 / 60.0:
            pose = POSE_A if (data.time // args.interval) % 2 == 0 else POSE_B

            if args.style == "position":
                data.ctrl[: model.nu] = pose[: model.nu]
            else:
                raw = (args.kp * (pose[: model.nu] - data.qpos[: model.nu])
                       + args.kd * (0.0 - data.qvel[: model.nu]))
                data.ctrl[: model.nu] = raw
                if last_pose is None or not np.array_equal(pose, last_pose):
                    clipped = np.clip(raw, ctrl_lo, ctrl_hi)
                    print(f"t={data.time:6.2f}s  파이썬이 계산한 ctrl={np.array2string(raw, precision=1)}")
                    print(f"            ctrlrange로 잘린 값={np.array2string(clipped, precision=3)} "
                          f"<- position actuator는 이걸 '목표각'으로 읽는다")

            last_pose = pose
            mujoco.mj_step(model, data)

        width, height = glfw.get_framebuffer_size(window)
        viewport = mujoco.MjrRect(0, 0, width, height)
        mujoco.mjv_updateScene(model, data, opt, None, cam,
                               mujoco.mjtCatBit.mjCAT_ALL, scene)
        mujoco.mjr_render(viewport, scene, ctx)
        glfw.swap_buffers(window)
        glfw.poll_events()

    glfw.terminate()


if __name__ == "__main__":
    main()
