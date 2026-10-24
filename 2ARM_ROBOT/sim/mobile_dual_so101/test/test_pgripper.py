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
    jaw_gap_m, require_stock_recording, selected_sides, replace_gripper)


class PGripperIntegrationTest(unittest.TestCase):
    def test_supplied_wrist_transform_and_rigid_camera(self):
        source = resolve_so101_model()
        arm = mujoco.MjSpec.from_file(str(source))
        parts, provenance = replace_gripper(arm)
        model = arm.compile()
        data = mujoco.MjData(model)
        basis = np.empty(9)
        mujoco.mju_quat2Mat(basis, parts[0]["mesh_quat"])
        basis = basis.reshape(3, 3)
        # Independent fixed-axis RPY calculation from the supplied URDF snapshot.
        r, p, y = [3.14151353, 1.52211682, -0.00007819]
        cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
        supplied = np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                             [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                             [-sp, cp*sr, cp*cr]])
        camera = provenance["wrist_camera"]
        local_camera = np.asarray(camera["position_m"])
        for angle in (-.5, 0, .5):
            data.joint("wrist_roll").qpos[0] = angle
            mujoco.mj_forward(model, data)
            wrist = data.body("wrist")
            gripper = data.body("gripper")
            parent_rotation = wrist.xmat.reshape(3, 3)
            root_rotation = gripper.xmat.reshape(3, 3)
            c, s = np.cos(angle), np.sin(angle)
            source_joint_rotation = np.array([[c, 0, -s], [0, 1, 0], [s, 0, c]])
            np.testing.assert_allclose(parent_rotation.T @ root_rotation @ basis,
                                       supplied @ source_joint_rotation, atol=1e-9)
            np.testing.assert_allclose(parent_rotation.T @ (gripper.xpos - wrist.xpos),
                                       [0, -.0611, .0181], atol=1e-12)
            cam = data.camera("wrist_cam")
            np.testing.assert_allclose(root_rotation.T @ (cam.xpos - gripper.xpos), local_camera, atol=1e-12)
            forward = -cam.xmat.reshape(3, 3)[:, 2]
            to_tip = data.site("cube_grasp").xpos - cam.xpos
            self.assertGreater(np.dot(forward, to_tip) / np.linalg.norm(to_tip),
                               np.cos(np.deg2rad(camera["vertical_fov_deg"] / 2)))
        compact, _ = build_compact_mobile_model(model_path=source, grippers="right")
        np.testing.assert_allclose(compact.camera("right_gripper_camera").pos, local_camera, atol=1e-12)
        np.testing.assert_allclose(compact.camera("right_gripper_camera").quat,
                                   model.camera("wrist_cam").quat, atol=1e-12)
        self.assertFalse(provenance["camera_mount_measured"])

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
        camera_position = description()[1]["wrist_camera"]["position_m"]
        for variant in ("right", "both"):
            model = build_tabletop(source, profile, grippers=variant)
            with tempfile.TemporaryDirectory() as directory:
                spec = build_tabletop_spec(source, profile, grippers=variant)
                spec.compile()
                xml = Path(directory) / "scene.xml"
                spec.to_file(str(xml))
                reloaded = mujoco.MjModel.from_xml_path(str(xml))
                self.assertEqual((reloaded.nq, reloaded.nu, reloaded.neq), (model.nq, model.nu, model.neq))
            for index in range(stock.ncam):
                name = stock.camera(index).name
                if name in {f"{side}_wrist_rgb" for side in selected_sides(variant)}:
                    np.testing.assert_allclose(model.camera(name).pos, camera_position)
                    continue
                other = model.camera(name).id
                for field in ("cam_pos", "cam_quat", "cam_fovy"):
                    np.testing.assert_array_equal(getattr(model, field)[other], getattr(stock, field)[index])
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
