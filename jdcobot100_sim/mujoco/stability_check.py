#!/usr/bin/env python3
"""MJCF 안정성 헤드리스 점검기 (화면 없이 물리만 검증).

교재 6~7장의 "actuator를 붙이면 QACC가 발산한다"를 눈이 아니라 숫자로
확인하기 위한 도구다. 모델을 정해진 스텝만큼 굴리면서
  * qacc 최댓값 (발산 여부)
  * NaN/Inf 발생 시각
  * 목표각 대비 정착 오차
를 찍는다.

사용:
    python stability_check.py <model.xml> [--target 0,0.5,0,0] [--seconds 3]
"""

from __future__ import annotations

import argparse
import os

import mujoco
import numpy as np


def run(path: str, target: np.ndarray | None, seconds: float, ctrl_is_torque: bool,
        kp: float, kd: float) -> dict:
    model = mujoco.MjModel.from_xml_path(path)
    data = mujoco.MjData(model)

    n_steps = int(seconds / model.opt.timestep)
    max_qacc = 0.0
    max_qvel = 0.0
    blew_up_at = None

    for i in range(n_steps):
        if target is not None and model.nu:
            if ctrl_is_torque:
                # 교재 10장 코드 방식: 파이썬에서 PD 토크를 계산해 ctrl에 넣는다.
                err = target[: model.nu] - data.qpos[: model.nu]
                derr = -data.qvel[: model.nu]
                data.ctrl[: model.nu] = kp * err + kd * derr
            else:
                # position actuator 정석: ctrl = 목표 위치
                data.ctrl[: model.nu] = target[: model.nu]

        mujoco.mj_step(model, data)

        qacc = np.abs(data.qacc)
        qvel = np.abs(data.qvel)
        if not np.all(np.isfinite(qacc)) or not np.all(np.isfinite(data.qpos)):
            blew_up_at = data.time
            break
        max_qacc = max(max_qacc, float(qacc.max()))
        max_qvel = max(max_qvel, float(qvel.max()))

    warnings = [
        (mujoco.mjtWarning(i).name, int(data.warning[i].number))
        for i in range(len(data.warning))
        if data.warning[i].number
    ]

    result = {
        "model": os.path.basename(path),
        "warnings": warnings,
        "unstable": blew_up_at is not None or max_qacc > 1e5,
        "nq": model.nq,
        "nu": model.nu,
        "total_mass": float(model.body_mass.sum()),
        "timestep": model.opt.timestep,
        "max_qacc": max_qacc,
        "max_qvel": max_qvel,
        "blew_up_at": blew_up_at,
        "final_qpos": data.qpos[: model.nq].copy(),
    }
    if target is not None:
        result["tracking_error"] = np.abs(target[: model.nq] - data.qpos[: model.nq])
    return result


def report(res: dict) -> None:
    print(f"--- {res['model']}")
    print(f"    nq={res['nq']} nu={res['nu']} 총질량={res['total_mass']:.6g} kg "
          f"timestep={res['timestep']}")
    tag = "FAIL" if res["unstable"] else " OK "
    if res["blew_up_at"] is not None:
        print(f"    [{tag}] t={res['blew_up_at']:.4f}s 에서 NaN/Inf 발산")
    else:
        print(f"    [{tag}] max|qacc|={res['max_qacc']:.4g} "
              f"max|qvel|={res['max_qvel']:.4g}")
    if res["warnings"]:
        print(f"    MuJoCo 경고 = {res['warnings']}")
    print(f"    최종 qpos = {np.array2string(res['final_qpos'], precision=4)}")
    if "tracking_error" in res:
        print(f"    목표 오차 = {np.array2string(res['tracking_error'], precision=4)} rad")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("model")
    ap.add_argument("--target", default=None, help="쉼표로 구분한 목표 관절각(rad)")
    ap.add_argument("--seconds", type=float, default=3.0)
    ap.add_argument("--torque-style", action="store_true",
                    help="교재 10장처럼 파이썬에서 PD를 계산해 ctrl에 넣는다")
    ap.add_argument("--kp", type=float, default=500.0)
    ap.add_argument("--kd", type=float, default=10.0)
    args = ap.parse_args()

    tgt = np.array([float(x) for x in args.target.split(",")]) if args.target else None
    report(run(args.model, tgt, args.seconds, args.torque_style, args.kp, args.kd))
