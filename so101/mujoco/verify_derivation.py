#!/usr/bin/env python3
"""derive_params.py가 유도한 값이 실제로 설계 목표를 만족하는지 확인한다.

유도의 핵심 주장은 두 개다.
  A. 정상상태에서  kp x 처짐 = 중력토크    -> 처짐을 예측할 수 있다
  B. 큰 오차 구간은 항상 포화              -> kp를 더 올려도 소용없다
둘 다 시뮬레이션으로 직접 재 본다.
"""

from __future__ import annotations

import math
import os

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
XML = os.path.join(HERE, "scene.xml")

# derive_params.py가 찾아낸, shoulder_lift 중력토크가 최대인 자세
WORST_POSE = np.array([0.8728, 1.3410, -1.3390, -0.0820, 1.3991, 0.5602])
SAG_LIMIT_DEG = 0.1


def main() -> None:
    model = mujoco.MjModel.from_xml_path(XML)
    names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
             for i in range(model.njnt)]
    kp = float(model.actuator_gainprm[0, 0])
    n = int(3.0 / model.opt.timestep)

    print(f"모델의 kp = {kp:.0f}, forcerange = {model.actuator_forcerange[0, 1]:.2f} N.m\n")

    print("주장 A: 정상상태 처짐 = 중력토크 / kp")
    print(f"{'joint':<16}{'중력토크':>12}{'예측 처짐':>13}{'실측 처짐':>13}{'차이':>10}")
    data = mujoco.MjData(model)
    for _ in range(n):
        data.ctrl[:] = WORST_POSE
        mujoco.mj_step(model, data)
    # 정착한 자세에서의 중력 항
    hold = mujoco.MjData(model)
    hold.qpos[: model.nq] = data.qpos[: model.nq]
    mujoco.mj_forward(model, hold)

    ok = True
    for i, name in enumerate(names):
        tau = abs(float(hold.qfrc_bias[i]))
        predicted = tau / kp
        actual = abs(float(WORST_POSE[i] - data.qpos[i]))
        diff = abs(predicted - actual)
        if diff > 3e-4:
            ok = False
        print(f"{name:<16}{tau:>10.4f} N.m{predicted:>13.6f}{actual:>13.6f}{diff:>10.6f}")

    worst_sag = float(np.abs(WORST_POSE - data.qpos[: model.nq]).max())
    limit = math.radians(SAG_LIMIT_DEG)
    print(f"\n  최대 처짐 {worst_sag:.6f} rad ({math.degrees(worst_sag):.4f} deg) "
          f"vs 목표 {limit:.6f} rad ({SAG_LIMIT_DEG} deg) -> "
          f"{'통과' if worst_sag <= limit else '초과'}")
    print(f"  예측과 실측 일치: {'그렇다' if ok else '아니다'}")

    print("\n주장 B: 큰 오차 구간은 포화라 kp가 의미 없다")
    reach = np.array([0.90, -0.70, 1.10, 0.55, 0.80, 1.00])
    limit_f = model.actuator_forcerange[:, 1]
    linear_band = float(limit_f[0] / kp)
    print(f"  선형 구간 폭 = forcerange/kp = {linear_band:.5f} rad "
          f"({math.degrees(linear_band):.3f} deg)")
    d2 = mujoco.MjData(model)
    saturated_steps = 0
    total = 0
    exit_time = None
    for _ in range(n):
        d2.ctrl[:] = reach
        mujoco.mj_step(model, d2)
        sat = np.abs(d2.actuator_force[:6]) >= limit_f * 0.999
        saturated_steps += int(sat.sum())
        total += 6
        if exit_time is None and not sat.any():
            exit_time = d2.time
    print(f"  이동 시작~정착까지 포화 상태였던 시간 비율 = "
          f"{saturated_steps/total*100:.1f}%")
    print(f"  마지막으로 포화가 풀린 시각 = {exit_time:.3f}s "
          f"(그 전까지는 kp와 무관하게 최대 토크)")


if __name__ == "__main__":
    main()
