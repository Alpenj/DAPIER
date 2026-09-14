#!/usr/bin/env python3
"""화면 없이 자세별 스크린샷과 추종 로그를 남긴다 (검증/문서용).

GLFW 창을 띄울 수 없는 환경(SSH, CI)에서도 모델이 실제로 목표 자세까지
가는지 눈으로 확인할 수 있게 PNG를 뽑는다.
"""

from __future__ import annotations

import os

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
XML = os.path.join(HERE, "scene.xml")
OUT_DIR = os.path.join(HERE, "docs", "img")

POSES = {
    "pose_a_home": np.array([0.0, 0.0, 0.0, 0.0]),
    "pose_b_reach": np.array([0.45, 0.35, -0.40, 0.30]),
    "pose_c_fold": np.array([-0.45, -0.30, 0.45, -0.25]),
}
SETTLE_SECONDS = 2.0


def main() -> None:
    os.makedirs(OUT_DIR, exist_ok=True)
    model = mujoco.MjModel.from_xml_path(XML)
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, 720, 960)
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.lookat[:] = [0.0, 0.0, 0.10]
    cam.distance = 0.55
    cam.azimuth = 135.0
    cam.elevation = -12.0

    try:
        from PIL import Image
    except ImportError:
        Image = None

    print(f"{'pose':<14}{'목표(rad)':<34}{'도달(rad)':<34}{'최대오차(rad)':>14}")
    for name, target in POSES.items():
        mujoco.mj_resetData(model, data)
        for _ in range(int(SETTLE_SECONDS / model.opt.timestep)):
            data.ctrl[: model.nu] = target[: model.nu]
            mujoco.mj_step(model, data)

        err = float(np.abs(target[: model.nq] - data.qpos[: model.nq]).max())
        print(f"{name:<14}{np.array2string(target, precision=2):<34}"
              f"{np.array2string(data.qpos[:model.nq], precision=3):<34}{err:>14.5f}")

        renderer.update_scene(data, camera=cam)
        pixels = renderer.render()
        path = os.path.join(OUT_DIR, f"{name}.png")
        if Image is not None:
            Image.fromarray(pixels).save(path)
        else:  # PIL 없이도 돌아가도록 최소 PNG 인코더 사용
            import struct
            import zlib

            h, w, _ = pixels.shape
            raw = b"".join(b"\x00" + pixels[y].tobytes() for y in range(h))

            def chunk(tag, payload):
                return (struct.pack(">I", len(payload)) + tag + payload
                        + struct.pack(">I", zlib.crc32(tag + payload) & 0xFFFFFFFF))

            png = (b"\x89PNG\r\n\x1a\n"
                   + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
                   + chunk(b"IDAT", zlib.compress(raw, 6))
                   + chunk(b"IEND", b""))
            with open(path, "wb") as handle:
                handle.write(png)
        print(f"{'':14}저장: {os.path.relpath(path, HERE)}")

    renderer.close()


if __name__ == "__main__":
    main()
