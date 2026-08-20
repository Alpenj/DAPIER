#!/usr/bin/env python3
"""관절 하나만 사인파로 움직인다 (교재 4단계).

교재 예제와 다른 점:
  * 관절명이 SO101 실제 이름이다. 교재 본문은 jdcobot100 이름을 쓴다.
  * 진폭을 ctrlrange 안으로 자동 클립한다. 교재 예제의 0.8 rad는
    gripper(-0.174~1.745)에는 맞지 않는다.
  * deadband를 옵션으로 뒀다. 교재는 backlash 근사로 deadband를 쓰는데,
    position actuator에 넣으면 목표가 계단처럼 끊겨서 오히려 떨림이 생긴다.
    --deadband로 직접 확인할 수 있다.
"""

from __future__ import annotations

import argparse
import math
import os
import time

import mujoco
import mujoco.viewer
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
XML = os.path.join(HERE, "scene.xml")

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex",
          "wrist_flex", "wrist_roll", "gripper"]


def apply_deadband(target: float, current: float, deadband: float) -> float:
    """교재 2단계의 deadband. 오차가 작으면 현재값을 유지한다."""
    if abs(target - current) < deadband:
        return current
    return target


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--joint", default="shoulder_lift", choices=JOINTS)
    ap.add_argument("--amplitude", type=float, default=0.8, help="사인파 진폭(rad)")
    ap.add_argument("--freq", type=float, default=0.4, help="주파수(Hz)")
    ap.add_argument("--deadband", type=float, default=0.0,
                    help="0보다 크면 교재식 deadband 적용")
    ap.add_argument("--seconds", type=float, default=0.0, help="0이면 창을 닫을 때까지")
    args = ap.parse_args()

    model = mujoco.MjModel.from_xml_path(XML)
    data = mujoco.MjData(model)

    act_name = f"act_{args.joint}"
    act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, act_name)
    jnt_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, args.joint)
    if act_id == -1 or jnt_id == -1:
        raise SystemExit(f"이름을 찾지 못했다: {act_name} / {args.joint} "
                         "(inspect_names.py로 확인할 것)")
    qadr = model.jnt_qposadr[jnt_id]

    lo, hi = model.actuator_ctrlrange[act_id]
    amp = min(args.amplitude, min(abs(lo), abs(hi)))
    if amp < args.amplitude:
        print(f"진폭을 {args.amplitude} -> {amp:.4f} rad로 줄였다 "
              f"({args.joint} ctrlrange {lo:+.4f}..{hi:+.4f})")

    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "ee_site")
    print(f"{args.joint} 제어 중 (Ctrl+C 또는 창 닫기로 종료)")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        t0 = time.time()
        last_print = 0.0
        while viewer.is_running():
            if args.seconds and time.time() - t0 > args.seconds:
                break
            step_start = data.time
            while data.time - step_start < 1.0 / 60.0:
                target = amp * math.sin(2 * math.pi * args.freq * data.time)
                if args.deadband > 0:
                    target = apply_deadband(target, float(data.qpos[qadr]), args.deadband)
                data.ctrl[act_id] = target
                mujoco.mj_step(model, data)

            if data.time - last_print > 0.5:
                last_print = data.time
                ee = data.site_xpos[site_id]
                print(f"t={data.time:6.2f}  ctrl={data.ctrl[act_id]:+.4f}  "
                      f"qpos={data.qpos[qadr]:+.4f}  "
                      f"오차={data.ctrl[act_id]-data.qpos[qadr]:+.4f} rad  "
                      f"ee=({ee[0]:+.3f}, {ee[1]:+.3f}, {ee[2]:+.3f})")
            viewer.sync()


if __name__ == "__main__":
    main()
