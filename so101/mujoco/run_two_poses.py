#!/usr/bin/env python3
"""두 자세를 번갈아 목표로 준다 (교재 5단계 + 6단계 site 출력).

교재 예제와 다른 점:
  * SO101 6축 전체를 쓴다. 교재 예제는 4축이고 관절명도 다르다.
  * --interpolate로 선형 보간을 켤 수 있다. 교재가 "부드럽게 이동하고 싶을 때"
    로 언급만 하고 넘어간 부분이다. 계단 입력은 시작 순간 요구 토크가
    서보 한계의 수십 배라 항상 포화 구간을 지나간다.
  * 매 전환마다 ee_site 좌표를 찍는다 (교재 6단계).
"""

from __future__ import annotations

import argparse
import os

import mujoco
import mujoco.viewer
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
XML = os.path.join(HERE, "scene.xml")

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex",
          "wrist_flex", "wrist_roll", "gripper"]

# 관절 range 안에서 고른 자세. gripper는 열림(1.0)/닫힘(0.0)으로 같이 움직인다.
POSE_A = np.array([0.00, -1.20,  1.00, 0.30, 0.00, 0.80])   # 들어올린 자세, 그리퍼 열림
POSE_B = np.array([0.50,  0.60, -0.90, 0.50, 0.00, 1.00])   # 앞으로 내린 자세


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--interval", type=float, default=2.5, help="자세 전환 주기(초)")
    ap.add_argument("--interpolate", action="store_true",
                    help="계단 입력 대신 선형 보간으로 목표를 옮긴다")
    ap.add_argument("--ramp", type=float, default=1.2, help="보간에 쓸 시간(초)")
    ap.add_argument("--seconds", type=float, default=0.0)
    args = ap.parse_args()

    model = mujoco.MjModel.from_xml_path(XML)
    data = mujoco.MjData(model)

    act_ids = []
    for name in JOINTS:
        aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"act_{name}")
        if aid == -1:
            raise SystemExit(f"actuator act_{name}를 찾지 못했다")
        act_ids.append(aid)
    act_ids = np.array(act_ids)

    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "ee_site")
    lo = model.actuator_ctrlrange[act_ids, 0]
    hi = model.actuator_ctrlrange[act_ids, 1]
    for pose, label in ((POSE_A, "POSE_A"), (POSE_B, "POSE_B")):
        bad = np.where((pose < lo) | (pose > hi))[0]
        if bad.size:
            raise SystemExit(f"{label}의 {[JOINTS[i] for i in bad]}가 ctrlrange 밖이다")

    print(f"{'t':>6}  {'도달한 자세':<12}{'ee_site (x, y, z) m':<30}{'최대 추종오차':>14}")
    last_label = None
    prev_target = None

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            if args.seconds and data.time > args.seconds:
                break
            step_start = data.time
            while data.time - step_start < 1.0 / 60.0:
                phase = data.time % (2 * args.interval)
                going_to_b = phase < args.interval
                src, dst = (POSE_A, POSE_B) if going_to_b else (POSE_B, POSE_A)
                local = phase if going_to_b else phase - args.interval

                if args.interpolate:
                    alpha = min(local / args.ramp, 1.0)
                    target = src * (1.0 - alpha) + dst * alpha
                else:
                    target = dst

                data.ctrl[act_ids] = target
                mujoco.mj_step(model, data)

            label = "POSE_B" if going_to_b else "POSE_A"
            if label != last_label:
                # 목표가 막 바뀐 시점이다. 지금 qpos는 "직전 목표"에 도달한 결과이므로
                # 오차도 직전 목표 기준으로 재야 의미가 있다.
                if prev_target is not None:
                    ee = data.site_xpos[site_id]
                    reached = "POSE_A" if label == "POSE_B" else "POSE_B"
                    err = float(np.abs(prev_target - data.qpos[:6]).max())
                    print(f"{data.time:>6.2f}  {reached:<12}"
                          f"{f'({ee[0]:+.4f}, {ee[1]:+.4f}, {ee[2]:+.4f})':<30}"
                          f"{err:>10.5f} rad")
                last_label = label
                prev_target = dst
            viewer.sync()


if __name__ == "__main__":
    main()
