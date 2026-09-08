"""SIM-only integration checks for stock/right-only/both PGripper variants."""
import json
from pathlib import Path
import sys
import tempfile
import unittest

import mujoco
import numpy as np

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))
from mobile_dual_so101 import (ACTION_NAMES, HUMANOID_HOME_ACTION, apply_control_as_pose,
    build_model, resolve_so101_model, validate_model)
from compact_mobile_dual_so101 import build_compact_mobile_model, create_compact_mobile_data
from collision_guard import protected_geom_pairs
from pgripper import (MOTOR_MAX_RAD, JAW_METRES_PER_RAD, description, home_action,
    jaw_gap_m, require_stock_recording, selected_sides)


class PGripperIntegrationTest(unittest.TestCase):
    def test_mobile_variants_and_coupling(self):
        source = resolve_so101_model()
        parts, provenance = description()
        self.assertFalse(provenance["physical_mount_verified"])
        self.assertTrue(all(np.linalg.eigvalsh(p["inertia"]).min() > 0 for p in parts))
        with self.assertRaises(ValueError):
            selected_sides("unknown")
        stock, _ = build_model(arm_mount_height_m=.42, model_path=source)
        for variant in ("right", "both"):
            model, _ = build_model(arm_mount_height_m=.42, model_path=source, grippers=variant)
            sides = selected_sides(variant)
            self.assertEqual(model.nu, 12)
            self.assertEqual(model.neq, 2 * len(sides))
            self.assertEqual(tuple(model.actuator(i).name for i in range(model.nu)), ACTION_NAMES)
            self.assertFalse(np.any(model.eq_type == mujoco.mjtEq.mjEQ_WELD))
            protected = {i for pair in protected_geom_pairs(model) for i in pair}
            for side in sides:
                for suffix in ("pgripper_housing", "pgripper_pad_1", "pgripper_pad_2"):
                    geom = model.geom(f"{side}_{suffix}")
                    self.assertIn(geom.id, protected)
                    previous = int(geom.contype[0])
                    geom.contype[0] = 0
                    with self.assertRaisesRegex(RuntimeError, "collision geometry is missing"):
                        protected_geom_pairs(model)
                    geom.contype[0] = previous
            with self.assertRaisesRegex(ValueError, "stock recording"):
                require_stock_recording(model)
            data = mujoco.MjData(model)
            action = np.asarray(home_action(model, HUMANOID_HOME_ACTION))
            gaps, midpoints = [], []
            for fraction in np.linspace(0, 1, 9):
                for side in sides:
                    action[5 if side == "left" else 11] = MOTOR_MAX_RAD * fraction
                apply_control_as_pose(model, data, action)
                self.assertLess(np.max(np.abs(data.efc_pos[:model.neq])), 1e-10)
                gaps.append(jaw_gap_m(model, data, "right"))
                midpoints.append(data.site("right_cube_grasp").xpos.copy())
                displacement = JAW_METRES_PER_RAD * MOTOR_MAX_RAD * (1 - fraction)
                self.assertAlmostEqual(data.joint("right_pgripper_jaw_1_slide").qpos[0], displacement)
                self.assertAlmostEqual(data.joint("right_pgripper_jaw_2_slide").qpos[0], -displacement)
            self.assertGreater(gaps[0], .0003)  # nominal closed is not zero jaw gap
            self.assertLess(gaps[0], .0005)
            self.assertAlmostEqual(gaps[-1] - gaps[0], .0506644, places=6)
            self.assertTrue(np.allclose(midpoints, midpoints[0], atol=1e-12))
            if variant == "right":
                for index in range(stock.nbody):
                    name = stock.body(index).name
                    if name.startswith("left_"):
                        other = model.body(name).id
                        for field in ("body_pos", "body_quat", "body_mass", "body_inertia", "body_ipos"):
                            np.testing.assert_array_equal(getattr(stock, field)[index], getattr(model, field)[other])
                np.testing.assert_array_equal(model.actuator_ctrlrange[:6], stock.actuator_ctrlrange[:6])
            # Full mobile smoke uses physics, not pose writes after initialization.
            report = validate_model(model, source=source, smoke_steps=1000,
                                    initial_action=home_action(model, HUMANOID_HOME_ACTION))
            self.assertTrue(report["finite_state"])
            compact, _ = build_compact_mobile_model(model_path=source, grippers=variant)
            compact_data = create_compact_mobile_data(compact)
            self.assertTrue(np.isfinite(compact_data.qpos).all())

    def test_tabletop_cameras_data_guard_and_dynamic_jaws(self):
        try:
            from replay_recorded_episode import (build_tabletop, build_tabletop_spec, recorded_to_sim, sim_to_recorded)
            from lerobot.envs.so101_mujoco import camera_profiles
        except ModuleNotFoundError as error:
            self.skipTest(f"tabletop integration needs the existing LeRobot/Arrow environment: {error}")
        source = resolve_so101_model()
        profile = json.loads((PROJECT / "tabletop_replay.json").read_text())
        stock = build_tabletop(source, profile)
        for variant in ("right", "both"):
            model = build_tabletop(source, profile, grippers=variant)
            with tempfile.TemporaryDirectory() as directory:
                spec = build_tabletop_spec(source, profile, grippers=variant)
                spec.compile()
                xml = Path(directory) / "scene.xml"
                spec.to_file(str(xml))
                reloaded = mujoco.MjModel.from_xml_path(str(xml))
                self.assertEqual((reloaded.nq, reloaded.nu, reloaded.neq), (model.nq, model.nu, model.neq))
            np.testing.assert_array_equal(model.cam_pos, stock.cam_pos)
            np.testing.assert_array_equal(model.cam_quat, stock.cam_quat)
            np.testing.assert_array_equal(model.cam_fovy, stock.cam_fovy)
            for side in selected_sides(variant):
                self.assertEqual(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM,
                                                  f"{side}_dapier_fixed_finger_pad"), -1)
                self.assertGreater(model.geom(f"{side}_pgripper_pad_1").contype[0], 0)
                self.assertGreater(model.geom(f"{side}_pgripper_pad_2").contype[0], 0)
                self.assertEqual(model.geom(f"{side}_pgripper_pad_1").type[0], mujoco.mjtGeom.mjGEOM_MESH)
            for function in (recorded_to_sim, sim_to_recorded):
                with self.assertRaisesRegex(ValueError, "stock recording"):
                    function(model, np.zeros(12), profile)
            data = mujoco.MjData(model)
            action = np.tile(np.deg2rad([0, -35, 55, 35, 0, 30]), 2)
            action = np.asarray(home_action(model, action))
            apply_control_as_pose(model, data, action)
            open_gap = jaw_gap_m(model, data, "right")
            for fraction in np.linspace(1, 0, 1500):
                for side in selected_sides(variant):
                    data.ctrl[5 if side == "left" else 11] = fraction * MOTOR_MAX_RAD
                mujoco.mj_step(model, data)
            mujoco.mj_step(model, data, 500)
            closed_gap = jaw_gap_m(model, data, "right")
            self.assertGreater(open_gap - closed_gap, .049)
            self.assertGreater(closed_gap, 0)
            self.assertLess(closed_gap, .001)
            self.assertLess(np.max(np.abs(data.efc_pos[:model.neq])), .0001)
            self.assertTrue(np.isfinite(data.qpos).all())
            self.assertFalse(any(w.number for w in data.warning))


if __name__ == "__main__":
    unittest.main()
