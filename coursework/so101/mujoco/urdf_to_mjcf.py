#!/usr/bin/env python3
"""SO101 URDF를 MJCF(XML)로 변환한다 (교재 1단계).

jdcobot100(실습 3)과 달리 이 URDF는 손댈 게 거의 없다. LeRobot이 배포하는
`so101_new_calib.urdf`에는 이미
    <mujoco><compiler meshdir="meshes/" strippath="false" fusestatic="false"/></mujoco>
가 들어 있고 mesh filename도 `package://` 없이 파일명만 쓴다. 교재가 경고하는
`package://ros_arm/meshes/...` 문제는 이 파일에는 해당하지 않는다.

대신 확인해야 할 것은 관성이다. jdcobot100은 mass=1e-09이었지만 이 URDF는
실측 질량과 full inertia tensor를 갖고 있다. 그래서 실습 3에서 쓴
inertiafromgeom 우회가 여기서는 필요 없고, 오히려 쓰면 안 된다.
"""

from __future__ import annotations

import os

HERE = os.path.dirname(os.path.abspath(__file__))
BUILD = os.path.join(HERE, "build")
URDF = os.path.join(BUILD, "so101_new_calib.urdf")
RAW = os.path.join(BUILD, "so101_raw.xml")


def main() -> None:
    import mujoco
    import numpy as np

    if not os.path.exists(URDF):
        raise SystemExit("build/so101_new_calib.urdf가 없다. setup_assets.py를 먼저 실행할 것.")

    # meshdir="meshes/"가 URDF 위치 기준으로 풀리므로 그 디렉터리에서 컴파일한다.
    cwd = os.getcwd()
    os.chdir(HERE)
    try:
        model = mujoco.MjModel.from_xml_path(URDF)
        mujoco.mj_saveLastXML(RAW, model)
    finally:
        os.chdir(cwd)

    print(f"MJCF 저장: {os.path.relpath(RAW, HERE)}")
    print(f"  nq={model.nq} nv={model.nv} nu={model.nu} "
          f"nbody={model.nbody} ngeom={model.ngeom} nmesh={model.nmesh}")
    print(f"  전체 질량 = {model.body_mass.sum():.4f} kg")

    print("\n  관절 범위 (URDF에서 그대로 넘어온 값):")
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        lo, hi = model.jnt_range[i]
        print(f"    {name:<15} {lo:+.5f} .. {hi:+.5f} rad "
              f"({np.degrees(lo):+7.1f} .. {np.degrees(hi):+7.1f} deg)")

    print("\n  링크 질량:")
    for i in range(model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
        if model.body_mass[i] > 0:
            print(f"    {name:<28} {model.body_mass[i]*1000:7.1f} g")


if __name__ == "__main__":
    main()
