#!/usr/bin/env python3
"""앉기 / 일어서기를 반복한다 (교재 5단계).

교재는 관절각 딕셔너리 두 개를 하드코딩하고 선형 보간한다. 여기서는 목표를
**몸통 높이**로 주고 매 프레임 leg_ik.py의 닫힌 형태 역기구학으로 관절각을
푼다. 그래서 앞뒤 힙 높이 차이(5.337 mm)가 자동으로 보정되고, 높이를 바꾸고
싶으면 숫자 하나만 고치면 된다.

--profile로 높이 궤적을 고른다.
  cosine : 부드러운 왕복 (기본)
  linear : 교재식 선형 보간
  step   : 계단 입력. 발이 미끄러지거나 튀는 걸 보려면 이걸로.
"""

from __future__ import annotations

import argparse
import math
import os

import mujoco
import mujoco.viewer
import numpy as np

import leg_ik

HERE = os.path.dirname(os.path.abspath(__file__))
XML = os.path.join(HERE, "scene.xml")
LEGS = leg_ik.LEGS
JOINTS = [f"{leg}_joint{i}" for leg in LEGS for i in (1, 2)]


def height_profile(t: float, period: float, lo: float, hi: float, mode: str) -> float:
    phase = (t % period) / period
    if mode == "step":
        return hi if phase < 0.5 else lo
    tri = 2 * phase if phase < 0.5 else 2 * (1 - phase)
    if mode == "linear":
        s = tri
    else:                                   # cosine
        s = 0.5 - 0.5 * math.cos(math.pi * tri)
    return hi + (lo - hi) * s


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--stand", type=float, default=0.100, help="선 자세 몸통 높이(m)")
    ap.add_argument("--sit", type=float, default=0.070, help="앉은 자세 몸통 높이(m)")
    ap.add_argument("--period", type=float, default=4.0, help="한 주기(초)")
    ap.add_argument("--profile", choices=["cosine", "linear", "step"], default="cosine")
    ap.add_argument("--seconds", type=float, default=0.0)
    args = ap.parse_args()

    model = mujoco.MjModel.from_xml_path(XML)
    data = mujoco.MjData(model)
    legs = leg_ik.extract(model)

    reach_lo, reach_hi = leg_ik.stance_height_limits(legs)
    for label, h in (("stand", args.stand), ("sit", args.sit)):
        if not (reach_lo <= h <= reach_hi):
            raise SystemExit(f"{label} 높이 {h}가 도달 범위 "
                             f"{reach_lo:.5f}~{reach_hi:.5f} m 밖이다")
    print(f"몸통 높이 도달 범위 = {reach_lo:.5f} ~ {reach_hi:.5f} m")

    act = {n: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"act_{n}")
           for n in JOINTS}
    adr = {n: model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)]
           for n in JOINTS}
    base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    floor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")

    # 목표 높이 -> 관절각. 매 프레임 풀어도 되지만 값이 반복되므로 캐시한다.
    cache: dict[int, dict[str, tuple[float, float]]] = {}

    def angles_for(height: float) -> dict[str, tuple[float, float]]:
        key = int(round(height * 1e5))
        if key not in cache:
            cache[key] = leg_ik.solve_stance(legs, key / 1e5, joint_range=(-2.0, 2.0))
        return cache[key]

    mujoco.mj_resetDataKeyframe(model, data, 0)
    print(f"{'t':>6}{'목표 높이':>11}{'실제 높이':>11}{'추종오차(deg)':>15}"
          f"{'pitch(deg)':>12}{'발접촉':>7}")
    last = -1.0

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            if args.seconds and data.time > args.seconds:
                break
            step_start = data.time
            while data.time - step_start < 1.0 / 60.0:
                h = height_profile(data.time, args.period, args.sit, args.stand,
                                   args.profile)
                sol = angles_for(h)
                for name in JOINTS:
                    th1, th2 = sol[name[:2]]
                    data.ctrl[act[name]] = th1 if name.endswith("1") else th2
                mujoco.mj_step(model, data)

            if data.time - last > 0.5:
                last = data.time
                h = height_profile(data.time, args.period, args.sit, args.stand,
                                   args.profile)
                err = max(abs(data.ctrl[act[n]] - data.qpos[adr[n]]) for n in JOINTS)
                w, x, y, z = data.qpos[3:7]
                pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))))
                nfoot = sum(1 for i in range(data.ncon)
                            if floor in (data.contact[i].geom1, data.contact[i].geom2))
                print(f"{data.time:>6.2f}{h:>11.4f}{data.xpos[base][2]:>11.4f}"
                      f"{math.degrees(err):>15.3f}{pitch:>12.3f}{nfoot:>7}")
            viewer.sync()


if __name__ == "__main__":
    main()
