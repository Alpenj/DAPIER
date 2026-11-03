#!/usr/bin/env python3
"""STS3215 데이터시트에서 MJCF 파라미터를 유도한다 (스윕 대신 계산).

damping / armature / forcerange / kp / kv는 감으로 고르거나 스윕으로 찾는 대신
서보 사양과 모델의 관성·중력 토크에서 직접 계산할 수 있다. 여기서 유도한 값을
gain_sweep.py의 측정값과 대조하면 유도가 맞는지 확인된다.

데이터시트 (Feetech STS3215, 12 V):
  스톨 토크   30 kgf.cm
  무부하 속도 0.222 s / 60도
  감속비      1:345
"""

from __future__ import annotations

import math
import os

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))

# ----- 데이터시트 -----
G = 9.80665
STALL_KGFCM = 30.0
NOLOAD_S_PER_60DEG = 0.222
GEAR_RATIO = 345.0

# 로터를 강철 원기둥으로 근사할 때의 치수 (이 부분만 추정값)
ROTOR_DIAMETER = 8e-3      # m
ROTOR_LENGTH = 15e-3       # m
ROTOR_DENSITY = 7800.0     # kg/m^3, 강철

# 설계 목표: 중력에 의한 정상상태 처짐 허용치
SAG_LIMIT_DEG = 0.1


def stall_torque() -> float:
    """30 kgf.cm -> N.m"""
    return STALL_KGFCM * G / 100.0


def noload_speed() -> float:
    """0.222 s/60deg -> rad/s (출력축 기준)"""
    return math.radians(60.0) / NOLOAD_S_PER_60DEG


def joint_damping() -> float:
    """점성 감쇠 b = 스톨토크 / 무부하속도.

    DC 모터의 토크-속도 선형 특성에서 나온다. 토크는 무부하 속도에서 0,
    정지 상태에서 스톨 토크이므로 기울기가 곧 점성 계수다.
    """
    return stall_torque() / noload_speed()


def rotor_inertia() -> float:
    """로터를 균질 원기둥으로 보고 J = 1/2 m r^2."""
    radius = ROTOR_DIAMETER / 2.0
    mass = math.pi * radius**2 * ROTOR_LENGTH * ROTOR_DENSITY
    return 0.5 * mass * radius**2, mass


def armature() -> float:
    """감속기를 지난 등가 회전 관성 J_eq = N^2 * J_rotor.

    감속비 N의 기어를 통해 보면 로터 관성이 N^2배로 확대되어 보인다.
    """
    j_rotor, _ = rotor_inertia()
    return GEAR_RATIO**2 * j_rotor


def max_gravity_torque(model: mujoco.MjModel, samples: int = 4000) -> np.ndarray:
    """작업 공간을 훑으며 관절별 최대 중력 토크를 구한다.

    정지 상태(qvel=0)에서 qfrc_bias는 순수 중력 항이다.
    """
    data = mujoco.MjData(model)
    lo = model.jnt_range[:, 0]
    hi = model.jnt_range[:, 1]
    rng = np.random.default_rng(0)
    worst = np.zeros(model.nq)
    worst_pose = np.zeros((model.nq, model.nq))
    for _ in range(samples):
        data.qpos[: model.nq] = rng.uniform(lo, hi)
        data.qvel[:] = 0.0
        mujoco.mj_forward(model, data)
        tau = np.abs(data.qfrc_bias[: model.nq])
        better = tau > worst
        worst_pose[better] = data.qpos[: model.nq]
        worst = np.maximum(worst, tau)
    return worst, worst_pose


def effective_inertia(model: mujoco.MjModel) -> np.ndarray:
    """홈 자세에서 질량행렬 대각 (armature 포함)."""
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    full = np.zeros((model.nv, model.nv))
    mujoco.mj_fullM(model, full, data.qM)
    return np.diag(full).copy()


def main() -> None:
    model = mujoco.MjModel.from_xml_path(os.path.join(HERE, "scene.xml"))
    names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
             for i in range(model.njnt)]

    tau = stall_torque()
    omega = noload_speed()
    b = joint_damping()
    j_rotor, rotor_mass = rotor_inertia()
    j_eq = armature()

    print("=" * 68)
    print("1) forcerange  <- 스톨 토크")
    print(f"   {STALL_KGFCM} kgf.cm x {G} / 100 = {tau:.4f} N.m")
    print(f"   => forcerange = \"-{tau:.2f} {tau:.2f}\"")

    print("\n2) damping  <- 토크-속도 특성의 기울기")
    print(f"   무부하 속도 = 60deg / {NOLOAD_S_PER_60DEG}s = {omega:.3f} rad/s")
    print(f"   b = {tau:.4f} / {omega:.3f} = {b:.4f} N.m.s/rad")
    print(f"   => damping = {b:.2f}")

    print("\n3) armature  <- 감속기를 통해 본 로터 관성")
    print(f"   로터 근사: d={ROTOR_DIAMETER*1e3:.0f}mm l={ROTOR_LENGTH*1e3:.0f}mm "
          f"강철 -> m={rotor_mass*1e3:.2f} g")
    print(f"   J_rotor = 1/2 m r^2 = {j_rotor:.3e} kg.m^2")
    print(f"   J_eq = N^2 J_rotor = {GEAR_RATIO:.0f}^2 x {j_rotor:.3e} = {j_eq:.4f} kg.m^2")
    print(f"   => armature = {j_eq:.3f}")

    print("\n4) kp  <- 허용 처짐과 최대 중력 토크")
    tau_g, worst_pose = max_gravity_torque(model)
    sag = math.radians(SAG_LIMIT_DEG)
    print(f"   position actuator 정상상태: kp * 처짐 = 중력토크")
    print(f"   허용 처짐 {SAG_LIMIT_DEG} deg = {sag:.6f} rad")
    print(f"   {'joint':<16}{'최대 중력토크':>14}{'필요 kp':>12}")
    kps = tau_g / sag
    for name, t, k in zip(names, tau_g, kps):
        print(f"   {name:<16}{t:>12.4f} N.m{k:>12.1f}")
    kp = float(kps.max())
    print(f"   => kp = {kp:.0f}  (가장 무거운 축 기준, 6축 공통)")

    print("\n   상한 확인: kp를 더 올려도 되는가")
    print(f"   선형 구간 |오차| < forcerange/kp = {tau/kp:.4f} rad "
          f"({math.degrees(tau/kp):.3f} deg)")
    print(f"   그보다 큰 오차에서는 항상 포화(bang-bang)라 kp가 의미 없다.")
    heaviest = int(np.argmax(kps))
    print(f"\n   최대 중력토크 자세 ({names[heaviest]} 기준):")
    print(f"   qpos = {np.array2string(worst_pose[heaviest], precision=4)}")
    print(f"   -> 검증용 목표 자세로 쓸 것 (verify_derivation.py)")

    print("\n5) kv  <- 임계감쇠")
    m_eff = effective_inertia(model)
    print(f"   kv = 2 sqrt(kp * M_eff) - damping   (zeta = 1)")
    print(f"   {'joint':<16}{'M_eff':>12}{'임계 kv':>12}")
    for name, mm in zip(names, m_eff):
        kv = 2.0 * math.sqrt(kp * mm) - b
        print(f"   {name:<16}{mm:>12.6f}{kv:>12.4f}")
    print("   축마다 M_eff가 다르므로 상수 kv 하나로는 전부 맞출 수 없다.")
    print("   => kv 대신 dampratio=\"1\"을 쓴다. MuJoCo가 축별 M_eff로 계산한다.")
    print("=" * 68)


if __name__ == "__main__":
    main()
