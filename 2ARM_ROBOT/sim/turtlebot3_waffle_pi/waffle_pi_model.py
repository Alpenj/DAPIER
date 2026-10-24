#!/usr/bin/env python3
"""Build and validate the stationary TurtleBot3 Waffle Pi MuJoCo model."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Sequence

import mujoco


PROJECT_DIR = Path(__file__).resolve().parent
BASE_URDF = PROJECT_DIR / "turtlebot3_waffle_pi_mujoco.urdf"
WHEEL_JOINT_NAMES = ("tb3_wheel_left_joint", "tb3_wheel_right_joint")
OFFICIAL_VISUAL_MESH_NAMES = {
    "waffle_pi_base",
    "left_tire",
    "right_tire",
    "lds",
}
EXPECTED_COLLISION_GEOMS = 7

_SCENE_XML = """
<mujoco model="dapier_turtlebot3_waffle_pi">
  <option timestep="0.002" gravity="0 0 -9.81"/>
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <global azimuth="135" elevation="-24"/>
  </visual>
  <worldbody>
    <light pos="0 0 3.5" dir="0 0 -1" directional="true"/>
    <geom name="floor" type="plane" size="0 0 0.05" pos="0 0 0"
          rgba="0.2 0.3 0.4 1"/>
  </worldbody>
</mujoco>
"""


def _configure_imported_geometry(base_spec: mujoco.MjSpec) -> None:
    """Keep official meshes visible and collision proxies contact-only."""

    visual_meshes = []
    collision_geoms = []
    for geom in base_spec.geoms:
        body_name = geom.parent.name
        if (
            geom.type == mujoco.mjtGeom.mjGEOM_MESH
            and geom.contype == 0
            and geom.conaffinity == 0
        ):
            geom.name = f"{body_name}_visual"
            visual_meshes.append(geom)
            continue
        if geom.contype or geom.conaffinity:
            geom.name = f"{body_name}_collision"
            geom.group = 3
            geom.rgba[3] = 0.0
            collision_geoms.append(geom)
            continue
        raise RuntimeError(
            f"unclassified Waffle Pi geometry on {body_name}: {geom.type}"
        )

    mesh_names = {geom.meshname for geom in visual_meshes}
    if mesh_names != OFFICIAL_VISUAL_MESH_NAMES:
        raise RuntimeError(f"official Waffle Pi visual meshes are incomplete: {mesh_names}")
    if len(collision_geoms) != EXPECTED_COLLISION_GEOMS:
        raise RuntimeError(
            "unexpected Waffle Pi collision geometry count: "
            f"{len(collision_geoms)}"
        )


def build_spec() -> mujoco.MjSpec:
    """Return an MJCF parent spec containing the official Waffle Pi URDF."""

    if not BASE_URDF.is_file():
        raise FileNotFoundError(f"missing Waffle Pi URDF: {BASE_URDF}")
    spec = mujoco.MjSpec.from_string(_SCENE_XML)
    base_mount = spec.worldbody.add_frame(name="tb3_mount")
    base_spec = mujoco.MjSpec.from_file(str(BASE_URDF))
    _configure_imported_geometry(base_spec)
    spec.attach(base_spec, prefix="tb3_", frame=base_mount)
    return spec


def build_model() -> mujoco.MjModel:
    return build_spec().compile()


def _names(
    model: mujoco.MjModel, object_type: mujoco.mjtObj, count: int
) -> tuple[str, ...]:
    return tuple(
        mujoco.mj_id2name(model, object_type, index) or ""
        for index in range(count)
    )


def validate_model(model: mujoco.MjModel, smoke_steps: int = 1000) -> dict[str, object]:
    if smoke_steps < 0:
        raise ValueError("smoke_steps must be non-negative")
    if (model.nq, model.nv, model.nu, model.njnt) != (2, 2, 0, 2):
        raise RuntimeError(
            "unexpected Waffle Pi dimensions: "
            f"nq={model.nq}, nv={model.nv}, nu={model.nu}, njnt={model.njnt}"
        )

    joints = _names(model, mujoco.mjtObj.mjOBJ_JOINT, model.njnt)
    bodies = _names(model, mujoco.mjtObj.mjOBJ_BODY, model.nbody)
    if joints != WHEEL_JOINT_NAMES:
        raise RuntimeError(f"unexpected wheel joints: {joints!r}")
    if "tb3_base_link" not in bodies:
        raise RuntimeError("MuJoCo conversion lost tb3_base_link")

    visual_mesh_geoms = [
        index
        for index in range(model.ngeom)
        if model.geom_type[index] == mujoco.mjtGeom.mjGEOM_MESH
        and model.geom_contype[index] == 0
        and model.geom_conaffinity[index] == 0
        and model.geom_rgba[index, 3] == 1.0
    ]
    hidden_collision_geoms = [
        index
        for index in range(model.ngeom)
        if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, index) or "").startswith("tb3_")
        and model.geom_contype[index] != 0
        and model.geom_conaffinity[index] != 0
        and model.geom_rgba[index, 3] == 0.0
        and model.geom_group[index] == 3
    ]
    expected_meshes = len(OFFICIAL_VISUAL_MESH_NAMES)
    if len(visual_mesh_geoms) != expected_meshes or model.nmesh != expected_meshes:
        raise RuntimeError("official Waffle Pi visual meshes did not survive MuJoCo compilation")
    if len(hidden_collision_geoms) != EXPECTED_COLLISION_GEOMS:
        raise RuntimeError("Waffle Pi collision proxies are not hidden contact geometry")

    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    for _ in range(smoke_steps):
        mujoco.mj_step(model, data)

    finite_state = all(math.isfinite(float(value)) for value in data.qpos) and all(
        math.isfinite(float(value)) for value in data.qvel
    )
    if not finite_state:
        raise RuntimeError("Waffle Pi simulation produced a non-finite state")
    return {
        "nq": model.nq,
        "nv": model.nv,
        "nu": model.nu,
        "nbody": model.nbody,
        "njnt": model.njnt,
        "ngeom": model.ngeom,
        "nmesh": model.nmesh,
        "official_visual_meshes": len(visual_mesh_geoms),
        "hidden_collision_geoms": len(hidden_collision_geoms),
        "wheel_joints": joints,
        "finite_state": finite_state,
        "smoke_steps": smoke_steps,
        "contacts_after_smoke": data.ncon,
        "max_abs_qvel": max((abs(float(value)) for value in data.qvel), default=0.0),
        "wheel_actuators_present": False,
    }


def _run_viewer(model: mujoco.MjModel) -> None:
    import mujoco.viewer

    data = mujoco.MjData(model)
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Validate the DAPIER TurtleBot3 Waffle Pi MuJoCo base."
    )
    parser.add_argument("--smoke-steps", type=int, default=1000)
    parser.add_argument("--viewer", action="store_true")
    args = parser.parse_args(argv)

    model = build_model()
    print(
        json.dumps(
            validate_model(model, smoke_steps=args.smoke_steps),
            ensure_ascii=False,
            indent=2,
        )
    )
    if args.viewer:
        _run_viewer(model)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
