#!/usr/bin/env python3
"""변환 직후 MJCF에 MuJoCo 전용 요소를 얹어 so101.xml을 만든다 (교재 2·6단계).

실습 3(jdcobot100)과 달리 질량·관성은 손대지 않는다. LeRobot이 배포하는
URDF에 실측값이 들어 있어서(총 632 g) 그대로 쓰는 게 맞다.
여기서 추가하는 것은 순수하게 "URDF가 표현하지 못하는 것"뿐이다.

  1. default class STS3215 - joint 동특성과 position actuator 게인
  2. 6개 joint에 position actuator (ctrlrange = 각 joint의 물리 range)
  3. end-effector site (교재 6단계)
  4. option: implicitfast 적분기

교재 2단계 예제 XML은 jdcobot100의 body tree와 관절명(base_shoulder 등)을
그대로 담고 있어서 SO101에는 쓸 수 없다. 실제 관절명 6개를 쓴다.
"""

from __future__ import annotations

import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
RAW = os.path.join(HERE, "build", "so101_raw.xml")
OUT = os.path.join(HERE, "so101.xml")

ROOT_BODY = "base_link"
EE_BODY = "gripper_frame_link"

# STS3215: 12 V에서 스톨 토크 30 kgf·cm = 2.94 N·m.
# URDF의 actuatorfrcrange="-10 10"은 실제 서보보다 3배 넉넉해서 그대로 두면
# 시뮬레이터가 실물보다 훨씬 힘센 팔이 된다.
STALL_TORQUE = 2.94

# 이 값들은 스윕으로 찾은 게 아니라 STS3215 데이터시트에서 유도했다.
# 전체 유도 과정과 검증은 derive_params.py / verify_derivation.py 참고.
#   forcerange 2.94 = 30 kgf.cm 스톨 토크
#   damping    0.62 = 스톨토크 / 무부하속도 (토크-속도 특성의 기울기)
#   armature   0.006 = N^2 * J_rotor (감속비 345, 로터를 강철 원기둥으로 근사)
#   kp         500  = 최대 중력토크 0.863 N.m / 허용 처짐 0.1 deg
#   kv         -> dampratio="1" (축마다 M_eff가 달라 상수 kv로는 못 맞춘다)
# frictionloss만 유도하지 못해 upstream LeRobot MJCF 값을 그대로 썼다.
PARAMS = dict(damping=0.62, frictionloss=0.052, armature=0.006, kp=500.0)

HEADER = """  <compiler angle="radian" meshdir="meshes/"/>

  <option timestep="0.002" integrator="implicitfast"/>

  <default>
    <!-- Feetech STS3215 한 종류로 6축이 전부 구동된다. joint 동특성과
         position actuator 게인을 한 클래스에 묶되, 적용 대상이 섞이지 않도록
         body는 childclass로, actuator는 class로 각각 참조한다. -->
    <default class="STS3215">
      <!-- contype=1 / conaffinity=0: 로봇끼리는 접촉하지 않고(1&0=0),
           바닥·큐브 등 외부 물체(1/1)와는 접촉한다(1&1=1).
           CAD 메쉬상 base_link와 shoulder_link가 22~28 mm 파고들어 있어서
           자체 충돌을 켜두면 shoulder_pan에 -187 N.m 구속력이 걸린다.
           교재 정답처럼 contype=0으로 전부 끄면 조용해지지만 바닥·물체 접촉도
           같이 죽어서 이후 pick/place 작업에 쓸 수 없다. -->
      <geom contype="1" conaffinity="0"/>
      <joint damping="{damping}" frictionloss="{frictionloss}" armature="{armature}"/>
      <position kp="{kp}" dampratio="1" forcerange="{fr_lo} {fr_hi}"/>
    </default>
  </default>
"""


def main() -> None:
    import mujoco

    with open(RAW, encoding="utf-8") as handle:
        xml = handle.read()

    xml = xml.replace('<mujoco model="so101_new_calib">', '<mujoco model="so101">', 1)

    header = HEADER.format(fr_lo=-STALL_TORQUE, fr_hi=STALL_TORQUE, **PARAMS)
    xml, n = re.subn(r'  <compiler[^>]*/>\n', header, xml, count=1)
    if n != 1:
        raise SystemExit("compiler 태그를 찾지 못했다")

    # 최상위 body에 childclass -> 6개 joint 전부가 STS3215 동특성을 상속
    xml, n = re.subn(rf'(<body name="{ROOT_BODY}")', rf'\1 childclass="STS3215"', xml, count=1)
    if n != 1:
        raise SystemExit(f"{ROOT_BODY}를 찾지 못했다")

    # end-effector site (교재 6단계). gripper_frame_link는 URDF상 그리퍼 기준
    # 프레임이라 원점에 두면 그대로 TCP 기준점이 된다.
    xml, n = re.subn(
        rf'(<body name="{EE_BODY}"[^>]*>)',
        r'\1\n          <site name="ee_site" pos="0 0 0" size="0.006" rgba="1 0.2 0.2 1"/>',
        xml,
        count=1,
    )
    if n != 1:
        raise SystemExit(f"{EE_BODY}를 찾지 못했다")

    # ctrlrange는 각 joint의 물리 range를 그대로 쓴다. 교재처럼 임의값(-1.57 등)을
    # 넣으면 wrist_roll(-2.74..2.84)처럼 범위가 넓은 축이 잘려버린다.
    model = mujoco.MjModel.from_xml_path(RAW)
    lines = []
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        lo, hi = model.jnt_range[i]
        lines.append(
            f'    <position class="STS3215" name="act_{name}" joint="{name}" '
            f'ctrlrange="{lo:.5f} {hi:.5f}"/>\n'
        )
    xml = xml.replace("</mujoco>", "  <actuator>\n" + "".join(lines) + "  </actuator>\n</mujoco>", 1)

    with open(OUT, "w", encoding="utf-8") as handle:
        handle.write(xml)
    print(f"생성: {os.path.relpath(OUT, HERE)}  (actuator {model.njnt}개, ee_site 1개)")


if __name__ == "__main__":
    main()
