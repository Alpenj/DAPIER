#!/usr/bin/env python3
"""화면 없이 자세별 스크린샷과 접지 상태를 남긴다 (검증/문서용)."""

from __future__ import annotations

import math
import os

import mujoco
import numpy as np

import leg_ik

HERE = os.path.dirname(os.path.abspath(__file__))
XML = os.path.join(HERE, "scene.xml")
OUT_DIR = os.path.join(HERE, "docs", "img")
JOINTS = [f"{leg}_joint{i}" for leg in leg_ik.LEGS for i in (1, 2)]

# 몸통 높이로만 지정한다. 관절각은 닫힌 형태 IK가 푼다.
POSES = {"pose_a_stand": 0.100, "pose_b_mid": 0.085, "pose_c_sit": 0.070}
SETTLE_SECONDS = 3.0


def save_png(pixels: np.ndarray, path: str) -> None:
    try:
        from PIL import Image
        Image.fromarray(pixels).save(path)
        return
    except ImportError:
        pass
    import struct
    import zlib

    h, w, _ = pixels.shape
    raw = b"".join(b"\x00" + pixels[y].tobytes() for y in range(h))

    def chunk(tag: bytes, payload: bytes) -> bytes:
        return (struct.pack(">I", len(payload)) + tag + payload
                + struct.pack(">I", zlib.crc32(tag + payload) & 0xFFFFFFFF))

    png = (b"\x89PNG\r\n\x1a\n"
           + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
           + chunk(b"IDAT", zlib.compress(raw, 6))
           + chunk(b"IEND", b""))
    with open(path, "wb") as handle:
        handle.write(png)


def main() -> None:
    os.makedirs(OUT_DIR, exist_ok=True)
    model = mujoco.MjModel.from_xml_path(XML)
    data = mujoco.MjData(model)
    legs = leg_ik.extract(model)
    renderer = mujoco.Renderer(model, 720, 960)

    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.lookat[:] = [0.0, 0.0, 0.055]
    cam.distance = 0.52
    cam.azimuth = 135.0
    cam.elevation = -16.0

    act = {n: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"act_{n}")
           for n in JOINTS}
    adr = {n: model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)]
           for n in JOINTS}
    base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    floor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")

    lo, hi = leg_ik.stance_height_limits(legs)
    print(f"몸통 높이 도달 범위 = {lo:.5f} ~ {hi:.5f} m")
    print(f"{'pose':<14}{'목표 높이':>10}{'실제 높이':>11}{'처짐(deg)':>11}"
          f"{'pitch(deg)':>12}{'발접촉':>7}")

    for name, height in POSES.items():
        sol = leg_ik.solve_stance(legs, height, joint_range=(-2.0, 2.0))
        mujoco.mj_resetData(model, data)
        # 목표 자세를 그대로 초기값으로 두고 발 표면을 바닥에 맞춘다.
        data.qpos[3] = 1.0
        for n in JOINTS:
            th1, th2 = sol[n[:2]]
            data.qpos[adr[n]] = th1 if n.endswith("1") else th2
        mujoco.mj_forward(model, data)
        bottoms = []
        for g in range(model.ngeom):
            if model.geom_type[g] != mujoco.mjtGeom.mjGEOM_CYLINDER:
                continue
            r, hl = model.geom_size[g][0], model.geom_size[g][1]
            az = data.geom_xmat[g].reshape(3, 3)[2, 2]
            bottoms.append(float(data.geom_xpos[g][2]
                                 - (r * math.sqrt(max(0.0, 1 - az * az)) + hl * abs(az))))
        data.qpos[2] = -min(bottoms) + 0.0002

        for _ in range(int(SETTLE_SECONDS / model.opt.timestep)):
            for n in JOINTS:
                th1, th2 = sol[n[:2]]
                data.ctrl[act[n]] = th1 if n.endswith("1") else th2
            mujoco.mj_step(model, data)

        sag = max(abs(data.ctrl[act[n]] - data.qpos[adr[n]]) for n in JOINTS)
        w, x, y, z = data.qpos[3:7]
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))))
        nfoot = sum(1 for i in range(data.ncon)
                    if floor in (data.contact[i].geom1, data.contact[i].geom2))
        print(f"{name:<14}{height:>10.4f}{data.xpos[base][2]:>11.5f}"
              f"{math.degrees(sag):>11.3f}{pitch:>12.3f}{nfoot:>7}")

        renderer.update_scene(data, camera=cam)
        path = os.path.join(OUT_DIR, f"{name}.png")
        save_png(renderer.render(), path)
        print(f"{'':14}저장: {os.path.relpath(path, HERE)}")

    renderer.close()


if __name__ == "__main__":
    main()
