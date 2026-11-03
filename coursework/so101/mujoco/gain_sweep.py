#!/usr/bin/env python3
"""게인 비교표를 만든다. README의 숫자가 전부 여기서 나온다.

측정 항목
  중력처짐 : ctrl=0으로 3초 유지했을 때 최대 |qpos|
  추종오차 : reach 자세로 3초 뒤 최대 |목표-qpos|
  오버슈트 : 이동 중 |qpos| 최대값이 |목표|를 넘은 비율
  포화비율 : actuator_force가 forcerange에 붙어 있던 시간 비율
"""

from __future__ import annotations

import os
import re

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
BASE = os.path.join(HERE, "so101.xml")
TMP = os.path.join(HERE, "build", "_gain_variant.xml")
REACH = np.array([0.90, -0.70, 1.10, 0.55, 0.80, 1.00])
SECONDS = 3.0

VARIANTS = [
    ("채택: 유도값",             'kp="500" dampratio="1"',   0.62, 0.052, 0.006),
    ("교재 정답 = upstream",     'kp="998.22" kv="2.731"',   0.60, 0.052, 0.028),
    ("교재 정답 + kv 15",        'kp="998.22" kv="15"',      0.60, 0.052, 0.028),
    ("유도값이되 armature 0.028", 'kp="500" dampratio="1"',   0.62, 0.052, 0.028),
    ("유도값이되 kp 150",         'kp="150" dampratio="1"',   0.62, 0.052, 0.006),
    ("유도값이되 kv 상수 2.731",   'kp="500" kv="2.731"',      0.62, 0.052, 0.006),
]


def build(pos_attr: str, damping: float, fric: float, arm: float) -> mujoco.MjModel:
    with open(BASE, encoding="utf-8") as handle:
        xml = handle.read()
    xml = re.sub(r'<joint damping="[^"]*" frictionloss="[^"]*" armature="[^"]*"',
                 f'<joint damping="{damping}" frictionloss="{fric}" armature="{arm}"', xml)
    xml = re.sub(r'<position [^/]*forcerange="([^"]*)"/>',
                 lambda m: f'<position {pos_attr} forcerange="{m.group(1)}"/>', xml)
    # meshdir이 so101.xml 기준이라 build/에 쓰면 한 단계 더 올라가야 한다.
    xml = xml.replace('meshdir="meshes/"', 'meshdir="../meshes/"')
    os.makedirs(os.path.dirname(TMP), exist_ok=True)
    with open(TMP, "w", encoding="utf-8") as handle:
        handle.write(xml)
    return mujoco.MjModel.from_xml_path(TMP)


def measure(model: mujoco.MjModel) -> tuple[float, float, float, float]:
    n = int(SECONDS / model.opt.timestep)

    hold = mujoco.MjData(model)
    for _ in range(n):
        hold.ctrl[:] = 0.0
        mujoco.mj_step(model, hold)
    sag = float(np.abs(hold.qpos[:6]).max())

    data = mujoco.MjData(model)
    peak = np.zeros(6)
    limit = model.actuator_forcerange[:, 1]
    saturated = 0
    for _ in range(n):
        data.ctrl[:] = REACH
        mujoco.mj_step(model, data)
        peak = np.maximum(peak, np.abs(data.qpos[:6]))
        saturated += int((np.abs(data.actuator_force[:6]) >= limit * 0.999).sum())
    err = float(np.abs(REACH - data.qpos[:6]).max())
    overshoot = float(((peak - np.abs(REACH)) / np.abs(REACH)).max() * 100)
    sat = saturated / (n * 6) * 100
    return sag, err, overshoot, sat


def main() -> None:
    print(f"{'설정':<28}{'중력처짐(rad)':>14}{'추종오차(rad)':>14}"
          f"{'오버슈트':>10}{'포화비율':>10}")
    for label, pos_attr, damping, fric, arm in VARIANTS:
        sag, err, ov, sat = measure(build(pos_attr, damping, fric, arm))
        print(f"{label:<28}{sag:>14.5f}{err:>14.5f}{ov:>9.1f}%{sat:>9.1f}%")
    os.remove(TMP)

    print("\nreach 자세 시작 시점의 요구 토크 (최대 오차 1.10 rad, forcerange 2.94 N.m):")
    for kp in (150, 500, 998.22):
        print(f"  kp={kp:>7}: {kp*1.10:>8.1f} N.m  = 한계의 {kp*1.10/2.94:>5.0f}배 "
              f"| 선형 구간 |오차| < {2.94/kp:.4f} rad")


if __name__ == "__main__":
    main()
