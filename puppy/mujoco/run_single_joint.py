#!/usr/bin/env python3
"""관절 하나만 사인파로 움직인다 (교재 4단계).

기립 자세를 유지한 채 지정한 관절 하나에만 진폭을 더한다. 나머지 7축을 0으로
두면 로봇이 그대로 주저앉기 때문이다(교재도 "나머지 관절은 기본 자세를
유지시키는 편이 안정적"이라고 안내한다).
"""

from __future__ import annotations

import argparse
import math
import os

import mujoco
import mujoco.viewer
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
XML = os.path.join(HERE, "scene.xml")
LEGS = ("lf", "rf", "lb", "rb")
JOINTS = [f"{leg}_joint{i}" for leg in LEGS for i in (1, 2)]


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--joint", default="lf_joint2", choices=JOINTS)
    ap.add_argument("--amplitude", type=float, default=0.25)
    ap.add_argument("--freq", type=float, default=0.5)
    ap.add_argument("--seconds", type=float, default=0.0)
    args = ap.parse_args()

    model = mujoco.MjModel.from_xml_path(XML)
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)          # 기립 자세에서 시작
    stand = model.key_ctrl[0].copy()

    act = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"act_{args.joint}")
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, args.joint)
    if act == -1 or jid == -1:
        raise SystemExit(f"이름을 찾지 못했다: {args.joint}")
    qadr = model.jnt_qposadr[jid]
    site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"{args.joint[:2]}_foot")
    base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")

    lo, hi = model.actuator_ctrlrange[act]
    print(f"{args.joint}: 기립값 {stand[act]:+.4f} rad, ctrlrange {lo:+.2f}..{hi:+.2f}")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        last = 0.0
        while viewer.is_running():
            if args.seconds and data.time > args.seconds:
                break
            step_start = data.time
            while data.time - step_start < 1.0 / 60.0:
                target = stand.copy()
                target[act] = np.clip(
                    stand[act] + args.amplitude * math.sin(2 * math.pi * args.freq * data.time),
                    lo, hi)
                data.ctrl[:] = target
                mujoco.mj_step(model, data)

            if data.time - last > 0.5:
                last = data.time
                foot = data.site_xpos[site]
                print(f"t={data.time:6.2f}  ctrl={data.ctrl[act]:+.4f}  "
                      f"qpos={data.qpos[qadr]:+.4f}  "
                      f"오차={data.ctrl[act]-data.qpos[qadr]:+.4f} rad  "
                      f"발 z={foot[2]:+.4f}  몸통 z={data.xpos[base][2]:.4f}")
            viewer.sync()


if __name__ == "__main__":
    main()
