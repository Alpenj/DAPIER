#!/usr/bin/env python3
"""joint / actuator / site 이름을 찍어 본다 (교재 3단계).

교재가 문제 해결 2번에서 경고하는 "mj_name2id가 -1을 반환" 상황을 미리
막는 용도다. 실제로 교재 본문의 코드가 바로 그 함정에 빠져 있다 —
본문 예제는 SO101이 아니라 jdcobot100의 관절명(base_shoulder 등)을 쓴다.
"""

from __future__ import annotations

import os

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))


def main() -> None:
    model = mujoco.MjModel.from_xml_path(os.path.join(HERE, "scene.xml"))

    print(f"nq={model.nq} nv={model.nv} nu={model.nu} nsite={model.nsite} "
          f"nbody={model.nbody} ngeom={model.ngeom}")

    print("\n== joints ==")
    print(f"{'idx':>3}  {'name':<16}{'range (rad)':<24}{'range (deg)':<22}"
          f"{'damping':>8}{'armature':>9}")
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        lo, hi = model.jnt_range[i]
        dof = model.jnt_dofadr[i]
        print(f"{i:>3}  {name:<16}"
              f"{f'{lo:+.5f} .. {hi:+.5f}':<24}"
              f"{f'{np.degrees(lo):+7.1f} .. {np.degrees(hi):+7.1f}':<22}"
              f"{model.dof_damping[dof]:>8.3f}{model.dof_armature[dof]:>9.3f}")

    print("\n== actuators ==")
    print(f"{'idx':>3}  {'name':<20}{'joint':<16}{'ctrlrange':<24}{'forcerange':<18}")
    for i in range(model.nu):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        jnt = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT,
                                model.actuator_trnid[i, 0])
        lo, hi = model.actuator_ctrlrange[i]
        flo, fhi = model.actuator_forcerange[i]
        print(f"{i:>3}  {name:<20}{jnt:<16}"
              f"{f'{lo:+.5f} .. {hi:+.5f}':<24}{f'{flo:+.2f} .. {fhi:+.2f}':<18}")

    print("\n== sites ==")
    for i in range(model.nsite):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, i)
        body = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.site_bodyid[i])
        print(f"{i:>3}  {name:<16} (body: {body})")

    print("\n== 교재 본문이 쓰는 이름이 실제로 있는지 ==")
    for name in ("base_shoulder", "shoulder_arm1", "arm1_arm2", "arn2_end_arm"):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        print(f"   mj_name2id(JOINT, {name!r}) = {jid}"
              f"{'   <- 없음' if jid == -1 else ''}")


if __name__ == "__main__":
    main()
