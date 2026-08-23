#!/usr/bin/env python3
"""leg_ik.py의 닫힌 형태 해가 MuJoCo의 실제 기구학과 일치하는지 검증한다.

세 가지를 잰다.
  1. 해석 FK  vs  MuJoCo mj_forward  (같은 관절각 -> 같은 발 위치인가)
  2. IK 왕복  : 임의 관절각 -> FK -> IK -> 원래 각으로 돌아오는가
  3. 기립 해  : solve_stance가 실제로 네 발을 같은 높이에 놓는가
"""

from __future__ import annotations

import os

import mujoco
import numpy as np

import leg_ik

HERE = os.path.dirname(os.path.abspath(__file__))
XML = os.path.join(HERE, "puppy.xml")
LEGS = leg_ik.LEGS


def main() -> None:
    model = mujoco.MjModel.from_xml_path(XML)
    data = mujoco.MjData(model)
    legs = leg_ik.extract(model)

    adr = {}
    for leg in LEGS:
        for i in (1, 2):
            name = f"{leg}_joint{i}"
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            adr[name] = model.jnt_qposadr[jid]
    dummy = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_dummy")
    foot_gid = {
        leg: next(g for g in range(model.ngeom)
                  if model.geom_bodyid[g] == mujoco.mj_name2id(
                      model, mujoco.mjtObj.mjOBJ_BODY, f"{leg}_link3")
                  and model.geom_type[g] == mujoco.mjtGeom.mjGEOM_CYLINDER)
        for leg in LEGS
    }

    def mujoco_feet(angles: dict[str, tuple[float, float]]) -> dict[str, np.ndarray]:
        """MuJoCo가 계산한 발 위치를 base_dummy 프레임의 (y, z)로 돌려준다."""
        qpos = np.zeros(model.nq)
        qpos[3] = 1.0
        for leg, (t1, t2) in angles.items():
            qpos[adr[f"{leg}_joint1"]] = t1
            qpos[adr[f"{leg}_joint2"]] = t2
        data.qpos[:] = qpos
        mujoco.mj_forward(model, data)
        origin = data.xpos[dummy]
        rot = data.xmat[dummy].reshape(3, 3)
        out = {}
        for leg in LEGS:
            local = rot.T @ (data.geom_xpos[foot_gid[leg]] - origin)
            out[leg] = local[1:]          # (y, z)
        return out

    rng = np.random.default_rng(0)

    print("1) 해석 FK vs MuJoCo mj_forward")
    worst = 0.0
    for _ in range(500):
        angles = {leg: (float(rng.uniform(-1.5, 1.5)), float(rng.uniform(-1.5, 1.5)))
                  for leg in LEGS}
        actual = mujoco_feet(angles)
        for leg in LEGS:
            predicted = legs[leg].hip + legs[leg].fk(*angles[leg])
            worst = max(worst, float(np.linalg.norm(predicted - actual[leg])))
    print(f"   무작위 500개 자세, 최대 오차 = {worst:.3e} m")

    print("\n2) IK 왕복 (각도 -> FK -> IK -> 각도)")
    worst_rt = 0.0
    fails = 0
    for _ in range(500):
        leg = legs[LEGS[int(rng.integers(4))]]
        t1 = float(rng.uniform(-1.5, 1.5))
        t2 = float(rng.uniform(-1.5, 1.5))
        target = leg.fk(t1, t2)
        try:
            for elbow in (+1, -1):
                s1, s2 = leg.ik(target, elbow)
                if abs(_wrap(s2 - t2)) < 1e-6:
                    worst_rt = max(worst_rt, abs(_wrap(s1 - t1)))
                    break
            else:
                fails += 1
        except ValueError:
            fails += 1
    print(f"   무작위 500회, 복원 실패 {fails}건, 최대 각도 오차 = {worst_rt:.3e} rad")

    print("\n3) 몸통 높이의 해석적 한계")
    lo, hi = leg_ik.stance_height_limits(legs)
    print(f"   가능 범위 = {lo:.5f} ~ {hi:.5f} m")
    for leg in LEGS:
        g = legs[leg]
        dy = 0.0
        top = np.sqrt(g.reach[1]**2 - dy**2) - g.hip[1]
        print(f"     {leg}: 단독 상한 {top:.5f} m")
    print(f"   -> 뒷다리 힙이 {abs(legs['lb'].hip[1]-legs['lf'].hip[1])*1000:.3f} mm 높아 "
          f"위쪽 한계를 뒷다리가 정한다")
    try:
        leg_ik.solve_stance(legs, hi + 0.001)
        print("   [경고] 한계 밖인데 해가 나왔다")
    except ValueError:
        print(f"   한계+1mm({hi+0.001:.5f})는 정상적으로 거부됨")

    print("\n4) solve_stance가 네 발을 수평으로 놓는가")
    print(f"   {'목표 높이':>10}{'발 z 최대 편차':>16}{'몸통 높이(실측)':>18}")
    for height in (0.070, 0.090, 0.100, round(hi, 4)):
        angles = leg_ik.solve_stance(legs, height)
        feet = mujoco_feet(angles)
        zs = [feet[leg][1] for leg in LEGS]
        spread = max(zs) - min(zs)
        print(f"   {height:>10.3f}{spread:>16.2e}{-min(zs):>18.5f}")


def _wrap(a: float) -> float:
    return (a + np.pi) % (2 * np.pi) - np.pi


if __name__ == "__main__":
    main()
