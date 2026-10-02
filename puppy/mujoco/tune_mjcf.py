#!/usr/bin/env python3
"""변환 직후 MJCF에 MuJoCo 전용 요소를 얹어 puppy.xml을 만든다 (교재 2단계).

4족 로봇이라 앞선 실습(로봇암 2개)과 요구사항이 정반대다.

  * **freejoint가 필요하다.** URDF 루트가 월드에 고정 결합이라 그대로 두면
    로봇이 허공에 매달린 채 다리만 허우적거린다. 로봇암은 fixed base가
    정답이었지만 4족은 floating base여야 한다.
  * **발 접촉을 살려야 한다.** 실습 3·5에서는 자체 충돌을 껐지만, 여기서는
    발이 바닥을 딛는 것이 실습의 목적이다. 다리끼리만 충돌에서 빼고
    발-바닥 접촉은 그대로 둔다.
  * **서 있는 초기 자세가 필요하다.** freejoint를 붙이면 로봇이 원점에서
    시작해 바닥을 뚫거나 낙하한다. 기립 자세에서 발이 바닥에 닿는 높이를
    계산해 keyframe으로 넣는다.

추가로 URDF의 모든 geom이 rgba="0 0 0 1"(완전 검정)이라 뷰어에서 실루엣만
보인다. 몸통/다리/발 색을 나눠 준다.
"""

from __future__ import annotations

import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
RAW = os.path.join(HERE, "build", "puppy_raw.xml")
OUT = os.path.join(HERE, "puppy.xml")

ROOT_BODY = "base_footprint"
LEGS = ["lf", "rf", "lb", "rb"]
JOINTS = [f"{leg}_joint{i}" for leg in LEGS for i in (1, 2)]

# URDF가 명시한 관절 effort 한계. 서보 데이터시트를 추측하지 않고 모델이
# 실제로 갖고 있는 값을 쓴다.
EFFORT_LIMIT = 0.5

# 자세는 관절각이 아니라 **몸통 높이**로 지정하고, 관절각은 leg_ik.py의
# 닫힌 형태 역기구학으로 푼다.
#
# 교재 5단계는 네 다리에 같은 관절각(joint1=+0.25 / joint2=-0.90)을 준다.
# 그런데 이 URDF는 앞뒤 힙 부착 높이가 다르다.
#     lf/rf_link1 z = +0.120651
#     lb/rb_link1 z = +0.125988   (5.337 mm 높다)
# 같은 각도를 주면 뒷발이 뜨고 몸통이 pitch로 기운다(실측 -3.4도). 목표를
# '발 위치'로 주면 이 차이가 역기구학에서 자동으로 흡수된다.
#
# 부호도 교재와 반대다. 교재 값을 그대로 쓰면 다리가 몸통 쪽으로 접혀
# 몸통 높이가 0.017 m밖에 안 나온다.
#
# 상한은 해석적으로 0.11161 m(뒷다리가 결정)이다. 완전히 편 자세는 특이점에
# 가까워 무릎이 어느 쪽으로 굽을지 불안정하므로 상한의 90% 부근을 쓴다.
STAND_HEIGHT = 0.100      # 상한 0.11161 m의 89.6 %
SIT_HEIGHT = 0.070

# kp는 derive_params.py가 계산한 값이다.
#   기립 시 관절 토크(자코비안 전치) 0.0404 N.m / 허용 처짐 0.5도 -> kp = 4.6 -> 5
# damping / armature는 서보 사양이 주어지지 않아 유도하지 못했다. 대신
#   - armature는 수치 안정 조건 M_eff > kp*dt^2/4 를 만족하도록 잡고
#   - damping은 정착까지의 진동이 사라지는 최소값으로 측정해서 골랐다.
# 근거는 gain_sweep.py 출력과 README 참고.
PARAMS = dict(damping=0.02, frictionloss=0.002, armature=0.0015, kp=5.0)

# 발-바닥 접촉. solref의 timeconst는 timestep의 2배 이상이어야 한다(MuJoCo 권장).
FOOT_FRICTION = "1.0 0.02 0.001"
FOOT_SOLREF = "0.008 1"
FOOT_SOLIMP = "0.9 0.95 0.001"

HEADER = """  <compiler angle="radian" meshdir="meshes/"/>

  <option timestep="0.002" integrator="implicitfast"/>

  <default>
    <!-- 다리 관절 동특성. 최상위 body의 childclass로 8개 관절에 상속된다. -->
    <default class="puppy_joint">
      <joint damping="{damping}" frictionloss="{frictionloss}" armature="{armature}"/>
    </default>

    <!-- position actuator 게인 -->
    <default class="puppy_servo">
      <position kp="{kp}" dampratio="1" forcerange="{fr_lo} {fr_hi}"/>
    </default>

    <!-- 발 끝: 유일하게 바닥과 접촉하는 geom -->
    <default class="foot">
      <geom contype="1" conaffinity="1" friction="{foot_friction}"
            solref="{foot_solref}" solimp="{foot_solimp}" rgba="0.85 0.2 0.2 1"/>
    </default>
  </default>
"""


def main() -> None:
    import mujoco
    import numpy as np

    with open(RAW, encoding="utf-8") as handle:
        xml = handle.read()

    xml = xml.replace('<mujoco model="puppy">', '<mujoco model="puppy">', 1)

    header = HEADER.format(fr_lo=-EFFORT_LIMIT, fr_hi=EFFORT_LIMIT,
                           foot_friction=FOOT_FRICTION, foot_solref=FOOT_SOLREF,
                           foot_solimp=FOOT_SOLIMP, **PARAMS)
    xml, n = re.subn(r'  <compiler[^>]*/>\n', header, xml, count=1)
    if n != 1:
        raise SystemExit("compiler 태그를 찾지 못했다")

    # 1) freejoint + childclass
    xml, n = re.subn(
        rf'(<body name="{ROOT_BODY}">)',
        r'\1\n      <freejoint name="root"/>',
        xml, count=1)
    if n != 1:
        raise SystemExit(f"{ROOT_BODY}를 찾지 못했다")
    xml = xml.replace(f'<body name="{ROOT_BODY}">',
                      f'<body name="{ROOT_BODY}" childclass="puppy_joint">', 1)

    # 2) 다리·몸통 mesh geom은 서로 충돌시키지 않는다. CAD 메쉬가 관절부에서
    #    겹쳐 있어 접촉이 잡히면 다리가 제자리에서 튄다. 발(cylinder)만 남긴다.
    def strip_mesh_contact(m: re.Match) -> str:
        tag = m.group(0)
        if 'contype=' in tag:
            return tag
        return tag.replace("<geom ", '<geom contype="0" conaffinity="0" ', 1)

    xml = re.sub(r'<geom [^>]*type="mesh"[^>]*/>', strip_mesh_contact, xml)

    # 3) 발 cylinder에 class="foot" + site
    def foot_geom(m: re.Match) -> str:
        return m.group(0).replace("<geom ", '<geom class="foot" ', 1)

    xml, n_foot = re.subn(r'<geom [^>]*type="cylinder"[^>]*/>', foot_geom, xml)
    if n_foot != 4:
        raise SystemExit(f"발 geom이 4개가 아니라 {n_foot}개다")

    for leg in LEGS:
        xml, n = re.subn(
            rf'(<body name="{leg}_link3"[^>]*>)',
            rf'\1\n                  <site name="{leg}_foot" pos="0.0032 0 0.001" '
            rf'size="0.004" rgba="1 1 0 1"/>',
            xml, count=1)
        if n != 1:
            raise SystemExit(f"{leg}_link3를 찾지 못했다")

    # 4) 색 입히기. URDF가 전부 rgba="0 0 0 1"이라 그대로 두면 검은 실루엣이다.
    xml = xml.replace('rgba="0 0 0 1" mesh="base_link"', 'rgba="0.25 0.55 0.85 1" mesh="base_link"')
    xml = re.sub(r'rgba="0 0 0 1" (mesh="[lr][fb]_link1")', r'rgba="0.9 0.75 0.2 1" \1', xml)
    xml = re.sub(r'rgba="0 0 0 1" (mesh="[lr][fb]_link2")', r'rgba="0.75 0.78 0.82 1" \1', xml)

    # 5) actuator
    lines = "".join(
        f'    <position class="puppy_servo" name="act_{j}" joint="{j}" '
        f'ctrlrange="-2 2"/>\n'
        for j in JOINTS
    )
    xml = xml.replace("</mujoco>", "  <actuator>\n" + lines + "  </actuator>\n</mujoco>", 1)

    with open(OUT, "w", encoding="utf-8") as handle:
        handle.write(xml)

    # 6) 자세를 역기구학으로 풀어서 keyframe으로 넣는다.
    import leg_ik

    model = mujoco.MjModel.from_xml_path(OUT)
    data = mujoco.MjData(model)
    legs = leg_ik.extract(model)
    lo, hi = leg_ik.stance_height_limits(legs)
    print(f"  몸통 높이 해석적 한계 = {lo:.5f} ~ {hi:.5f} m (뒷다리가 상한을 정함)")

    adr = {n: model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)]
           for n in JOINTS}
    foot_gids = [g for g in range(model.ngeom)
                 if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_CYLINDER]

    def geom_bottom(gid: int) -> float:
        """실린더 geom이 닿는 가장 낮은 world z."""
        radius, half_len = model.geom_size[gid][0], model.geom_size[gid][1]
        axis_z = data.geom_xmat[gid].reshape(3, 3)[2, 2]
        drop = radius * np.sqrt(max(0.0, 1.0 - axis_z**2)) + half_len * abs(axis_z)
        return float(data.geom_xpos[gid][2] - drop)

    keys = []
    for label, height in (("stand", STAND_HEIGHT), ("sit", SIT_HEIGHT)):
        if not (lo <= height <= hi):
            raise SystemExit(f"{label} 높이 {height}가 도달 범위 {lo:.5f}~{hi:.5f} 밖이다")
        angles = leg_ik.solve_stance(legs, height, joint_range=(-2.0, 2.0))

        qpos = np.zeros(model.nq)
        qpos[3] = 1.0
        ctrl = []
        for name in JOINTS:
            leg, which = name[:2], name[-1]
            th1, th2 = angles[leg]
            value = th1 if which == "1" else th2
            qpos[adr[name]] = value
            ctrl.append(value)

        data.qpos[:] = qpos
        mujoco.mj_forward(model, data)
        bottom = min(geom_bottom(g) for g in foot_gids)
        qpos[2] = -bottom + 0.0002          # 발 표면이 바닥에 살짝 닿도록

        bottoms = [geom_bottom(g) for g in foot_gids]
        print(f"  {label:<6} 몸통 높이 {height:.4f} m  "
              f"발 표면 높이차 {max(bottoms)-min(bottoms):.1e} m  "
              f"관절각 j1={angles['lf'][0]:+.4f}/{angles['lb'][0]:+.4f} "
              f"j2={angles['lf'][1]:+.4f}/{angles['lb'][1]:+.4f} (앞/뒤)")
        keys.append((label, qpos, ctrl))

    key_xml = "  <keyframe>\n"
    for label, qpos, ctrl in keys:
        key_xml += (f'    <key name="{label}" '
                    f'qpos="{" ".join(f"{v:.6f}" for v in qpos)}"\n'
                    f'         ctrl="{" ".join(f"{v:.6f}" for v in ctrl)}"/>\n')
    key_xml += "  </keyframe>\n"
    xml = xml.replace("</mujoco>", key_xml + "</mujoco>", 1)
    with open(OUT, "w", encoding="utf-8") as handle:
        handle.write(xml)

    print(f"생성: {os.path.relpath(OUT, HERE)}")
    print(f"  freejoint 1개, actuator {len(JOINTS)}개, 발 site 4개, keyframe {len(keys)}개")


if __name__ == "__main__":
    main()
