#!/usr/bin/env python3
"""ROS용 jdcobot100 URDF를 MuJoCo가 읽을 수 있는 MJCF(XML)로 변환한다.

교재(위키독스 372030)는 URDF 원본을 직접 고치라고 하지만, 이 저장소의
``jdcobot100_sim/urdf/jdcobot100.urdf`` 는 RViz/Gazebo launch가 그대로 쓰는
파일이라 ``package://`` 경로를 지우면 ROS 쪽이 깨진다. 그래서 원본은 두고
MuJoCo 전용 사본(``build/jdcobot100_mjcf_input.urdf``)을 만들어 변환한다.

사본에 적용하는 두 가지 수정이 교재 3장(STL 경로 오류 해결)에 해당한다.
  1. ``<mujoco><compiler meshdir=... strippath="false"/></mujoco>`` 삽입
  2. ``package://jdcobot100_sim/meshes/base.stl`` -> ``base.stl``
"""

from __future__ import annotations

import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SRC_URDF = os.path.join(HERE, "..", "urdf", "jdcobot100.urdf")
MESH_DIR = os.path.join(HERE, "..", "meshes")
BUILD_DIR = os.path.join(HERE, "build")
PATCHED_URDF = os.path.join(BUILD_DIR, "jdcobot100_mjcf_input.urdf")
RAW_MJCF = os.path.join(BUILD_DIR, "jdcobot100_raw.xml")

# URDF 사본에 넣을 MuJoCo 전용 블록.
# meshdir 는 위 사본이 놓이는 build/ 기준 상대 경로다.
MUJOCO_BLOCK = """  <mujoco>
    <compiler meshdir="../../meshes/" strippath="false" balanceinertia="true" discardvisual="false"/>
  </mujoco>
"""

PACKAGE_RE = re.compile(r'filename="package://[^"]*/([^"/]+)"')


def patch_urdf() -> str:
    os.makedirs(BUILD_DIR, exist_ok=True)
    with open(SRC_URDF, encoding="utf-8") as handle:
        urdf = handle.read()

    patched, n_paths = PACKAGE_RE.subn(r'filename="\1"', urdf)
    if "<mujoco>" not in patched:
        patched = re.sub(r"(<robot[^>]*>\n)", r"\1" + MUJOCO_BLOCK, patched, count=1)

    with open(PATCHED_URDF, "w", encoding="utf-8") as handle:
        handle.write(patched)

    print(f"[1/3] URDF 사본 생성: {os.path.relpath(PATCHED_URDF, HERE)}")
    print(f"      package:// 경로 {n_paths}개를 파일명만 남기도록 수정")
    return patched


def check_meshes(patched: str) -> None:
    missing = sorted(
        {
            name
            for name in re.findall(r'<mesh filename="([^"]+)"', patched)
            if not os.path.exists(os.path.join(MESH_DIR, name))
        }
    )
    if missing:
        raise SystemExit(f"[오류] meshes/ 에 없는 STL: {missing}")
    print(f"[2/3] STL 경로 확인 완료 (meshdir={os.path.relpath(MESH_DIR, HERE)})")


def convert() -> None:
    import mujoco

    model = mujoco.MjModel.from_xml_path(PATCHED_URDF)
    mujoco.mj_saveLastXML(RAW_MJCF, model)
    print(f"[3/3] MJCF 저장: {os.path.relpath(RAW_MJCF, HERE)}")
    print(
        f"      nq={model.nq} nv={model.nv} nu={model.nu} "
        f"nbody={model.nbody} ngeom={model.ngeom} nmesh={model.nmesh}"
    )
    total_mass = float(sum(model.body_mass))
    print(f"      전체 질량 = {total_mass:.6g} kg  <-- 교재 7장에서 손볼 값")


if __name__ == "__main__":
    if not os.path.exists(SRC_URDF):
        sys.exit(f"URDF를 찾을 수 없음: {SRC_URDF}")
    check_meshes(patch_urdf())
    convert()
