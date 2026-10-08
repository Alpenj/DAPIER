#!/usr/bin/env python3
"""변환 직후 MJCF(build/jdcobot100_raw.xml)를 구동 가능한 모델로 다듬는다.

교재 6~9장에 해당하는 편집을 스크립트로 재현 가능하게 남긴 것이다.
geom이 126개라 손으로 고치면 재현이 안 되므로 후처리 방식을 썼다.

적용하는 편집:
  0. meshdir을 build/ 기준(../../meshes/)에서 이 파일 기준(../meshes/)으로 되돌림
  1. compiler에 inertiafromgeom="true" + geom density -> mass 1e-09 문제 해결
  2. option: implicitfast 적분기 (damping이 큰 서보 모델에 유리)
  3. default class 분리 (교재 14.4의 권장 형태)
       sg90_joint : joint 쪽 동역학 (damping/frictionloss/armature)
       sg90_act   : position actuator 쪽 게인 (kp/kv/forcerange)
  4. 최상위 body에 childclass="sg90_joint" -> 4개 joint가 전부 상속
  5. 로봇 geom 전체에 contype=0 conaffinity=0 (접촉 끄기)
       -> CAD에서 온 인접 링크 메시가 서로 파고들어 있어서 접촉력이
          액추에이터 토크를 그대로 상쇄한다. 실측으로 dof_base에
          actuator_force=+0.18 N.m 인데 qfrc_constraint=-0.1797 N.m 이
          걸려서 관절이 0.03 rad에서 멈췄다. 고정 베이스 4축 팔이라
          자체 충돌은 필요 없으므로 로봇 geom만 접촉에서 뺀다.
          (교재 8장 sg90 클래스의 <geom contype="0" conaffinity="0"/>와 같은 조치)
  6. 4개 joint에 position actuator 추가 (class="sg90_act")
"""

from __future__ import annotations

import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
RAW = os.path.join(HERE, "build", "jdcobot100_raw.xml")
OUT = os.path.join(HERE, "jdcobot100.xml")

JOINTS = ["dof_base", "dof_shoulder", "dof_elbow", "dof_wrist_pitch"]
ROOT_BODY = "shoulder_sub_assembly"

# 부품 재질은 3D 프린팅 플라스틱 + SG90 서보 하우징 수준으로 잡았다.
# 이 값에서 링크 질량이 13~53 g, 팔 전체 136 g으로 나온다.
DENSITY = 1000

HEADER = f"""  <compiler angle="radian" meshdir="../meshes/" inertiafromgeom="true" balanceinertia="true"/>

  <option timestep="0.002" integrator="implicitfast"/>

  <default>
    <!-- STL 형상에서 질량/관성을 뽑기 위한 밀도 (kg/m^3). URDF의 mass=1e-09를 대체한다.
         scene.xml의 floor까지 물들지 않도록 이름 없는 default에는 밀도만 둔다. -->
    <geom density="{DENSITY}"/>

    <!-- joint 쪽 동역학: body의 childclass로 상속시킨다. -->
    <default class="sg90_joint">
      <joint damping="0.05" frictionloss="0.002" armature="0.0005" limited="true"/>
    </default>

    <!-- actuator 쪽 게인: actuator 태그에 직접 지정한다. -->
    <default class="sg90_act">
      <position kp="15.0" kv="0.4" forcerange="-0.18 0.18"/>
    </default>
  </default>
"""

# 교재 6장 상태(질량/관성/댐핑 손대지 않고 actuator만 붙인 모델)를 재현하기 위한 블록.
# 이 모델은 반드시 발산한다. 비교 실험용으로만 만든다.
NAIVE_ACTUATOR = """  <actuator>
""" + "".join(
    f'    <position name="act_{j}" joint="{j}" ctrlrange="-3.14159 3.14159"/>\n'
    for j in JOINTS
) + """  </actuator>
"""

ACTUATOR = """  <actuator>
"""+ "".join(
    f'    <position class="sg90_act" name="act_{j}" joint="{j}" ctrlrange="-0.523599 0.523599"/>\n'
    for j in JOINTS
) + """  </actuator>
"""


NAIVE_OUT = os.path.join(HERE, "build", "jdcobot100_naive_actuator.xml")


def write_naive(raw: str) -> None:
    """교재 6장 상태 재현 모델 (mass=1e-09 그대로 + actuator만 추가)."""
    with open(NAIVE_OUT, "w", encoding="utf-8") as handle:
        handle.write(raw.replace("</mujoco>", NAIVE_ACTUATOR + "</mujoco>", 1))
    print(f"생성: {os.path.relpath(NAIVE_OUT, HERE)} (비교용 - 발산하는 게 정상)")


def main() -> None:
    with open(RAW, encoding="utf-8") as handle:
        xml = handle.read()

    write_naive(xml)

    xml = xml.replace('<mujoco model="jdcobot100_urdf">', '<mujoco model="jdcobot100">', 1)

    compiler_re = re.compile(r'  <compiler[^>]*/>\n')
    if not compiler_re.search(xml):
        raise SystemExit("compiler 태그를 찾지 못했다")
    xml = compiler_re.sub(HEADER, xml, count=1)

    root_re = re.compile(rf'(<body name="{ROOT_BODY}"[^>]*?)(/?>)')
    xml, n = root_re.subn(rf'\1 childclass="sg90_joint"\2', xml, count=1)
    if n != 1:
        raise SystemExit(f"최상위 body({ROOT_BODY})를 찾지 못했다")

    # 로봇 geom만 접촉에서 제외한다. <default> 안의 geom까지 바꾸면 그 설정이
    # scene.xml의 floor에도 상속되어 바닥이 통과 가능해지므로 worldbody 안에서만 치환한다.
    start = xml.index("<worldbody>")
    end = xml.index("</worldbody>") + len("</worldbody>")
    world, n_geom = re.subn(
        r'<geom (?![^>]*contype=)',
        '<geom contype="0" conaffinity="0" ',
        xml[start:end],
    )
    xml = xml[:start] + world + xml[end:]
    print(f"접촉 해제한 geom: {n_geom}개 (이미 contype 지정된 visual geom은 그대로)")

    xml = xml.replace("</mujoco>", ACTUATOR + "</mujoco>", 1)

    with open(OUT, "w", encoding="utf-8") as handle:
        handle.write(xml)
    print(f"생성: {os.path.relpath(OUT, HERE)}")


if __name__ == "__main__":
    main()
