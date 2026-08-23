#!/usr/bin/env python3
"""damping / armature를 측정으로 정한다.

kp와 forcerange는 derive_params.py가 계산으로 정한다. 반면 이 로봇은 서보
데이터시트가 주어지지 않아 damping과 armature를 같은 방식으로 유도할 수 없다.
대신 두 가지 기준으로 고른다.

  * armature : 수치 안정 조건.  위치 서보를 스프링으로 보면 고유주기가
               T = 2*pi*sqrt(M_eff/kp) 이고, 적분기가 이를 분해하려면
               timestep이 그보다 충분히 작아야 한다. M_eff가 너무 작으면
               같은 kp에서 발산한다.
  * damping  : 기립 정착까지의 진동이 사라지는 최소값.

두 값을 훑어 실제로 어디서 안정해지는지 잰다.
"""

from __future__ import annotations

import math
import os
import re

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
BASE = os.path.join(HERE, "puppy.xml")
SCENE = os.path.join(HERE, "scene.xml")
TMP = os.path.join(HERE, "build", "_gain_scene.xml")
TMP_ROBOT = os.path.join(HERE, "build", "_gain_puppy.xml")
LEGS = ("lf", "rf", "lb", "rb")
JOINTS = [f"{leg}_joint{i}" for leg in LEGS for i in (1, 2)]


def build(damping: float, armature: float) -> mujoco.MjModel:
    with open(BASE, encoding="utf-8") as handle:
        robot = handle.read()
    robot = re.sub(r'<joint damping="[^"]*" frictionloss="([^"]*)" armature="[^"]*"',
                   rf'<joint damping="{damping}" frictionloss="\1" armature="{armature}"',
                   robot)
    robot = robot.replace('meshdir="meshes/"', 'meshdir="../meshes/"')
    os.makedirs(os.path.dirname(TMP), exist_ok=True)
    with open(TMP_ROBOT, "w", encoding="utf-8") as handle:
        handle.write(robot)

    with open(SCENE, encoding="utf-8") as handle:
        scene = handle.read()
    scene = scene.replace('<include file="puppy.xml"/>',
                          f'<include file="{os.path.basename(TMP_ROBOT)}"/>')
    with open(TMP, "w", encoding="utf-8") as handle:
        handle.write(scene)
    return mujoco.MjModel.from_xml_path(TMP)


def measure(model: mujoco.MjModel) -> dict:
    data = mujoco.MjData(model)
    adr = {n: model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)]
           for n in JOINTS}
    base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    mujoco.mj_resetDataKeyframe(model, data, 0)

    n = int(4.0 / model.opt.timestep)
    max_qacc = 0.0
    settle = None
    heights = []
    blew = False
    for _ in range(n):
        data.ctrl[:] = model.key_ctrl[0]
        mujoco.mj_step(model, data)
        if not np.all(np.isfinite(data.qacc)):
            blew = True
            break
        max_qacc = max(max_qacc, float(np.abs(data.qacc).max()))
        heights.append(float(data.xpos[base][2]))
        if settle is None and float(np.abs(data.qvel).max()) < 1e-3:
            settle = data.time

    if blew:
        return {"blew": True}

    tail = np.array(heights[-int(0.5 / model.opt.timestep):])
    sag = max(abs(model.key_ctrl[0][i] - data.qpos[adr[n_]]) for i, n_ in enumerate(JOINTS))
    return {
        "blew": False,
        "max_qacc": max_qacc,
        "settle": settle,
        "ripple": float(tail.max() - tail.min()) * 1000,   # mm
        "sag_deg": math.degrees(sag),
    }


def report(label: str, res: dict) -> None:
    if res["blew"]:
        print(f"{label:<34}{'발산':>10}")
        return
    print(f"{label:<34}{res['max_qacc']:>10.1f}"
          f"{(f'{res['settle']:.2f}s' if res['settle'] else '미정착'):>10}"
          f"{res['ripple']:>12.4f}{res['sag_deg']:>12.4f}")


def main() -> None:
    print(f"{'설정':<34}{'max|qacc|':>10}{'정착':>10}{'잔진동(mm)':>12}{'처짐(deg)':>12}")
    print("-- armature 훑기 (damping 0.02 고정) --")
    for arm in (0.0, 0.0001, 0.0005, 0.0015, 0.005, 0.02):
        report(f"  armature={arm}", measure(build(0.02, arm)))
    print("-- damping 훑기 (armature 0.0015 고정) --")
    for damp in (0.0, 0.002, 0.01, 0.02, 0.05, 0.2):
        report(f"  damping={damp}", measure(build(damp, 0.0015)))

    model = mujoco.MjModel.from_xml_path(SCENE)
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    full = np.zeros((model.nv, model.nv))
    mujoco.mj_fullM(model, full, data.qM)
    kp = float(model.actuator_gainprm[0, 0])
    dt = float(model.opt.timestep)
    print(f"\n수치 안정 기준: 고유주기 T = 2*pi*sqrt(M_eff/kp) 가 timestep보다 충분히 커야 한다")
    print(f"  kp={kp:.0f}  timestep={dt}s")
    print(f"  {'joint':<14}{'M_eff':>12}{'T (s)':>10}{'T/timestep':>12}")
    for name in JOINTS[:4]:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        dof = model.jnt_dofadr[jid]
        m_eff = float(full[dof, dof])
        period = 2 * math.pi * math.sqrt(m_eff / kp)
        print(f"  {name:<14}{m_eff:>12.6f}{period:>10.4f}{period/dt:>12.1f}")

    for path in (TMP, TMP_ROBOT):
        if os.path.exists(path):
            os.remove(path)


if __name__ == "__main__":
    main()
