from __future__ import annotations

import hashlib
import sys
import unittest
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

try:
    import mujoco
except ModuleNotFoundError as error:
    raise unittest.SkipTest("MuJoCo is not installed in this Python environment") from error

from waffle_pi_model import (
    BASE_URDF,
    EXPECTED_COLLISION_GEOMS,
    OFFICIAL_VISUAL_MESH_NAMES,
    WHEEL_JOINT_NAMES,
    build_model,
    validate_model,
)


def _git_blob_sha(path: Path) -> str:
    payload = path.read_bytes()
    return hashlib.sha1(f"blob {len(payload)}\0".encode() + payload).hexdigest()


class WafflePiModelTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.model = build_model()

    def test_portable_assets_exist(self) -> None:
        self.assertTrue(BASE_URDF.is_file())
        upstream = PROJECT_DIR / "upstream"
        self.assertTrue((upstream / "LICENSE").is_file())
        meshes = upstream / "turtlebot3_description" / "meshes"
        expected = (
            meshes / "bases" / "waffle_pi_base.stl",
            meshes / "wheels" / "left_tire.stl",
            meshes / "wheels" / "right_tire.stl",
            meshes / "sensors" / "lds.stl",
        )
        self.assertTrue(all(path.is_file() for path in expected))
        expected_blob_sha = {
            "waffle_pi_base.stl": "a2ff2c7ac8021dc7e51fdd07a3ae0f65d1adf9e8",
            "left_tire.stl": "29bfa039c30feb5f93a310fde2b39277d66596a8",
            "right_tire.stl": "43c27e67c96bbf60839a5db6792f73f2ee16590e",
            "lds.stl": "80a5fe33563aaa73d1cae521e82d0699ad525f4a",
        }
        self.assertEqual(
            {path.name: _git_blob_sha(path) for path in expected},
            expected_blob_sha,
        )
        official_urdf = (
            upstream
            / "turtlebot3_description"
            / "urdf"
            / "turtlebot3_waffle_pi.urdf"
        )
        self.assertEqual(
            _git_blob_sha(official_urdf),
            "ce39db97a6171fb3f0414fc84fc0daaa2d2bdf02",
        )

    def test_base_link_and_wheel_joints_survive_conversion(self) -> None:
        body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "tb3_base_link"
        )
        self.assertGreaterEqual(body_id, 0)
        joints = tuple(
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, index)
            for index in range(self.model.njnt)
        )
        self.assertEqual(joints, WHEEL_JOINT_NAMES)

    def test_official_meshes_render_and_collision_proxies_stay_active(self) -> None:
        mesh_names = {
            (
                mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_MESH, index)
                or ""
            ).removeprefix("tb3_")
            for index in range(self.model.nmesh)
        }
        self.assertEqual(mesh_names, OFFICIAL_VISUAL_MESH_NAMES)

        visual_geoms = []
        collision_geoms = []
        for index in range(self.model.ngeom):
            name = (
                mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, index)
                or ""
            )
            if name.startswith("tb3_") and name.endswith("_visual"):
                visual_geoms.append(index)
            if name.startswith("tb3_") and name.endswith("_collision"):
                collision_geoms.append(index)
        self.assertEqual(len(visual_geoms), len(OFFICIAL_VISUAL_MESH_NAMES))
        self.assertTrue(
            all(
                self.model.geom_type[index] == mujoco.mjtGeom.mjGEOM_MESH
                and self.model.geom_contype[index] == 0
                and self.model.geom_conaffinity[index] == 0
                and self.model.geom_rgba[index, 3] == 1.0
                for index in visual_geoms
            )
        )
        self.assertEqual(len(collision_geoms), EXPECTED_COLLISION_GEOMS)
        self.assertTrue(
            all(
                self.model.geom_contype[index] != 0
                and self.model.geom_conaffinity[index] != 0
                and self.model.geom_rgba[index, 3] == 0.0
                and self.model.geom_group[index] == 3
                for index in collision_geoms
            )
        )

    def test_no_wheel_actuator_is_added(self) -> None:
        self.assertEqual(self.model.nu, 0)

    def test_stationary_smoke_remains_finite(self) -> None:
        report = validate_model(self.model, smoke_steps=1000)
        self.assertTrue(report["finite_state"])
        self.assertFalse(report["wheel_actuators_present"])
        self.assertEqual(report["official_visual_meshes"], 4)
        self.assertEqual(report["hidden_collision_geoms"], 7)


if __name__ == "__main__":
    unittest.main()
