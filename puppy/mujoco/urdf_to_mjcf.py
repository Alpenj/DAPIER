#!/usr/bin/env python3
"""Puppy URDF를 MJCF(XML)로 변환한다 (교재 1단계).

교재는 `mujoco.MjModel.from_xml_path("puppy2.urdf")` 한 줄이면 된다고 하는데,
변환 결과를 그대로 쓰면 4족 로봇이 **허공에 매달린다.** URDF의 루트 링크
(`base_footprint`)가 월드에 고정 결합이라 MuJoCo가 fixed base로 컴파일하기
때문이다. 그 사실을 여기서 숫자로 확인하고, 실제 수정은 tune_mjcf.py에서 한다.
"""

from __future__ import annotations

import os

HERE = os.path.dirname(os.path.abspath(__file__))
BUILD = os.path.join(HERE, "build")
URDF = os.path.join(BUILD, "puppy2.urdf")
RAW = os.path.join(BUILD, "puppy_raw.xml")


def main() -> None:
    import mujoco
    import numpy as np

    if not os.path.exists(URDF):
        raise SystemExit("build/puppy2.urdf가 없다. setup_assets.py를 먼저 실행할 것.")

    model = mujoco.MjModel.from_xml_path(URDF)
    mujoco.mj_saveLastXML(RAW, model)

    print(f"MJCF 저장: {os.path.relpath(RAW, HERE)}")
    print(f"  nq={model.nq} nv={model.nv} nu={model.nu} "
          f"nbody={model.nbody} ngeom={model.ngeom} nmesh={model.nmesh}")
    print(f"  전체 질량 = {model.body_mass.sum()*1000:.1f} g")

    print("\n  관절:")
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        jtype = mujoco.mjtJoint(model.jnt_type[i]).name.replace("mjJNT_", "")
        lo, hi = model.jnt_range[i]
        print(f"    {name:<12} {jtype:<8} {lo:+.3f} .. {hi:+.3f} rad")

    free = [i for i in range(model.njnt)
            if model.jnt_type[i] == mujoco.mjtJoint.mjJNT_FREE]
    print(f"\n  free joint 개수 = {len(free)}")
    if not free:
        print("  -> 베이스가 월드에 고정되어 있다. 4족 로봇이 허공에 뜬 채로 다리만 움직인다.")
        print("     URDF 루트 base_footprint가 고정 결합이라 MuJoCo가 fixed base로 컴파일한다.")

    print("\n  링크 질량:")
    for i in range(model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
        if model.body_mass[i] > 0:
            print(f"    {name:<16} {model.body_mass[i]*1000:7.2f} g")


if __name__ == "__main__":
    main()
