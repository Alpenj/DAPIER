#!/usr/bin/env python3
"""Puppy의 MJCF 파라미터를 계산으로 정한다.

로봇암(실습 3·5)과 달리 이 로봇은 서보 데이터시트가 주어지지 않았다. 대신
URDF가 관절 effort 한계를 명시하고 있고, 나머지는 모델 자체의 기구학·질량과
MuJoCo 접촉 모델의 정의에서 유도할 수 있다.

  forcerange <- URDF의 effort 한계 (추측하지 않고 모델이 가진 값)
  kp         <- 기립 시 관절 토크 / 허용 처짐
                관절 토크는 발 자코비안 전치로 정확히 구한다: tau = J^T f
  solref     <- MuJoCo 권장 timeconst >= 2 * timestep
  마찰계수    <- 기립 시 필요한 최소값을 접촉력 비로 계산
"""

from __future__ import annotations

import math
import os

import mujoco
import numpy as np

import leg_ik

HERE = os.path.dirname(os.path.abspath(__file__))
XML = os.path.join(HERE, "scene.xml")
LEGS = leg_ik.LEGS

SAG_LIMIT_DEG = 0.5      # 기립 자세에서 허용할 관절 처짐


def main() -> None:
    model = mujoco.MjModel.from_xml_path(XML)
    data = mujoco.MjData(model)
    g = abs(float(model.opt.gravity[2]))
    total_mass = float(model.body_mass.sum())

    print("=" * 70)
    print("0) 모델에서 읽은 값")
    print(f"   전체 질량 = {total_mass*1000:.1f} g   중력 = {g:.5f} m/s^2")
    print(f"   무게 = {total_mass*g:.4f} N,  발 4개면 발당 {total_mass*g/4:.4f} N")
    # jnt_actfrcrange[0]은 freejoint라 0이다. 다리 관절에서 읽는다.
    jid0 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "lf_joint1")
    effort = float(model.jnt_actfrcrange[jid0][1])
    print(f"   URDF effort 한계 = {effort:.3f} N.m  -> forcerange")

    print("\n1) 기립 자세 (닫힌 형태 역기구학)")
    legs = leg_ik.extract(model)
    lo, hi = leg_ik.stance_height_limits(legs)
    print(f"   몸통 높이 도달 범위 = {lo:.5f} ~ {hi:.5f} m")
    print(f"     L1 = {legs['lf'].l1:.5f} m,  Le = {legs['lf'].le:.5f} m")
    print(f"     앞 힙 z = {legs['lf'].hip[1]:+.6f},  뒤 힙 z = {legs['lb'].hip[1]:+.6f}"
          f"  (차이 {abs(legs['lb'].hip[1]-legs['lf'].hip[1])*1000:.3f} mm)")

    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)

    print("\n2) kp <- 관절 토크 / 허용 처짐")
    print("   발에 수직력 f를 주면 관절 토크는 tau = J^T f 다.")
    print("   J는 발 위치의 자코비안이며 MuJoCo가 정확히 계산해 준다.")
    normal = total_mass * g / 4.0
    force = np.array([0.0, 0.0, normal])
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))

    sag = math.radians(SAG_LIMIT_DEG)
    print(f"\n   {'joint':<14}{'tau (N.m)':>12}{'필요 kp':>12}{'effort 대비':>12}")
    needed = {}
    for leg in LEGS:
        gid = next(gg for gg in range(model.ngeom)
                   if model.geom_bodyid[gg] == mujoco.mj_name2id(
                       model, mujoco.mjtObj.mjOBJ_BODY, f"{leg}_link3")
                   and model.geom_type[gg] == mujoco.mjtGeom.mjGEOM_CYLINDER)
        mujoco.mj_jacGeom(model, data, jacp, jacr, gid)
        tau = jacp.T @ force
        for idx in (1, 2):
            name = f"{leg}_joint{idx}"
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            dof = model.jnt_dofadr[jid]
            t = abs(float(tau[dof]))
            needed[name] = t / sag
            print(f"   {name:<14}{t:>12.5f}{t/sag:>12.1f}{t/effort*100:>11.1f}%")

    kp = max(needed.values())
    print(f"\n   => kp = {kp:.0f}  (가장 큰 축 기준, 8축 공통)")
    print(f"   선형 구간 |오차| < forcerange/kp = {effort/kp:.5f} rad "
          f"({math.degrees(effort/kp):.3f} deg)")

    print("\n3) solref <- MuJoCo 권장 timeconst >= 2 * timestep")
    dt = float(model.opt.timestep)
    print(f"   timestep = {dt} s  ->  timeconst >= {2*dt:.4f} s")
    foot_gid = next(gg for gg in range(model.ngeom)
                    if model.geom_type[gg] == mujoco.mjtGeom.mjGEOM_CYLINDER)
    print(f"   현재 발 solref = {model.geom_solref[foot_gid]}  "
          f"({'만족' if model.geom_solref[foot_gid][0] >= 2*dt else '위반'})")

    print("\n4) 마찰계수 <- 기립 시 접촉력 비")
    mujoco.mj_resetDataKeyframe(model, data, 0)
    for _ in range(int(3.0 / dt)):
        data.ctrl[:] = model.key_ctrl[0]
        mujoco.mj_step(model, data)
    worst = 0.0
    for i in range(data.ncon):
        wrench = np.zeros(6)
        mujoco.mj_contactForce(model, data, i, wrench)
        normal_f = abs(wrench[0])
        tangent = math.hypot(wrench[1], wrench[2])
        if normal_f > 1e-9:
            worst = max(worst, tangent / normal_f)
    print(f"   기립 정착 후 필요한 최소 마찰계수 = {worst:.4f}")
    print(f"   현재 발 friction = {model.geom_friction[foot_gid][0]:.3f}  "
          f"(여유 {model.geom_friction[foot_gid][0]/max(worst,1e-9):.0f}배)")
    print("=" * 70)


if __name__ == "__main__":
    main()
