import unittest
import tempfile
from pathlib import Path
from unittest.mock import patch

import mujoco
import numpy as np

from center_block_teacher import CenterBlockTeacher, run_with_viewer
from collision_guard import protected_geom_pairs
from integration_scenes import build_scene, task_env
from shoe_task import ShoeTaskEnv, ShoeTaskConfig, UnsafeActionError, block_metrics


class IntegrationTaskTest(unittest.TestCase):
    def test_viewer_observes_same_physics_state_as_headless(self):
        # Verify the observer on the valid baseline; never bypass desk's HOME gate.
        reference = CenterBlockTeacher(scene="legacy_tower")
        with patch.object(reference, "solve", side_effect=ValueError("stop at pregrasp")):
            expected = reference.run()
        teacher = CenterBlockTeacher(scene="legacy_tower")
        times = []
        overlays = []
        class Viewer:
            cam = type("Camera", (), {"lookat": np.zeros(3)})()
            opt = type("Option", (), {"geomgroup": np.ones(6, dtype=np.uint8)})()
            user_scn = None
            def lock(self):
                from contextlib import nullcontext
                return nullcontext()
            def __enter__(self): return self
            def __exit__(self, *args): return False
            def is_running(self): return True
            def sync(self): times.append(float(teacher.d.time))
            def set_texts(self, texts): overlays.append("\n".join(text[2] for text in texts))
        def launch(model, data, **kwargs):
            self.assertIs(model, teacher.m)
            self.assertIs(data, teacher.d)
            return Viewer()
        with tempfile.TemporaryDirectory() as directory, \
             patch("mujoco.viewer.launch_passive", side_effect=launch), \
             patch.object(teacher, "solve", side_effect=ValueError("stop at pregrasp")):
            report = run_with_viewer(teacher, Path(directory)/"report.json", hold_final=False)
        self.assertEqual(expected["transitions"], report["transitions"])
        self.assertEqual(teacher.d.time, reference.d.time)
        self.assertAlmostEqual(teacher.d.time, .2)
        self.assertGreater(max(times), 0)
        np.testing.assert_array_equal(teacher.d.qpos, reference.d.qpos)
        np.testing.assert_array_equal(teacher.d.qvel, reference.d.qvel)
        np.testing.assert_array_equal(teacher.d.ctrl, reference.d.ctrl)
        self.assertIsNone(teacher.env.physics_observer)
        self.assertTrue(any("Phase: SETTLE" in text for text in overlays))
        self.assertIn("STOPPED at PREGRASP", overlays[-1])
        self.assertIn("stop at pregrasp", overlays[-1])
        self.assertIn("SIM PHYSICS / LIVE TASK STATE", overlays[-1])
        self.assertIn("ctrl + mj_step", overlays[-1])

    def test_home_pair_diagnostic_preserves_real_clearance_rejection(self):
        from collision_diagnostics import describe_pair
        from mobile_dual_so101 import actuator_targets_from_qpos
        from collision_guard import check_bimanual_path
        teacher = CenterBlockTeacher()
        q = actuator_targets_from_qpos(teacher.m, teacher.d.qpos)
        guard = check_bimanual_path(teacher.m, q, q)
        self.assertTrue(guard.safe)
        before = (teacher.d.qpos.copy(), teacher.d.qvel.copy(), teacher.d.ctrl.copy())
        report = describe_pair(teacher.m, teacher.d, 48, 0, .03)
        self.assertEqual(report["geoms"][0]["mesh_asset"], "right_rotation_pitch_so101_v1")
        self.assertEqual(report["geoms"][0]["name"], None)  # truly unnamed geom
        self.assertEqual(report["geoms"][1]["name"], "table")
        self.assertEqual(report["geoms"][1]["type"], "mjGEOM_BOX")
        self.assertEqual(report["native_signed_distance_m"], 0.)
        self.assertAlmostEqual(report["final_distance_m"], .0162, places=7)
        self.assertTrue(report["mesh_box_certificate_eligible"])
        self.assertTrue(report["mesh_box_certificate_changed_result"])
        self.assertTrue(report["bounding_sphere_eligible"])
        self.assertFalse(report["bounding_sphere_changed_result"])
        self.assertFalse(report["mesh_certificate_eligible"])
        self.assertEqual(report["certified_separation_lower_bound_m"], 0.)
        self.assertTrue(report["protected_pair"])
        self.assertEqual(report["physical_clearance_class"], "B")
        self.assertEqual(report["distance_computation_class"], "C")
        for kind in ("compiled_raw", "compiled_hull"):
            evidence = report["independent_geometry"][kind]
            self.assertTrue(evidence["projection_inside_box_top"])
            self.assertAlmostEqual(evidence["exact_positive_distance_m"], .0162, places=7)
        self.assertEqual(report["pair_contacts"], [])
        for actual, expected in zip((teacher.d.qpos, teacher.d.qvel, teacher.d.ctrl), before):
            np.testing.assert_array_equal(actual, expected)
        self.assertTrue(check_bimanual_path(teacher.m, q, q).safe)

    def test_teacher_uses_approved_builder_and_passes_home_before_reset(self):
        with patch("shoe_task.build_mobile_spec", side_effect=AssertionError("old tower builder")):
            teacher = CenterBlockTeacher(scene="desk")
            before = teacher.d.qpos.copy()
            with patch("center_block_teacher.mujoco.mj_step", wraps=mujoco.mj_step) as step, \
                 patch.object(teacher.env, "reset", side_effect=ValueError("test stop after HOME")):
                report = teacher.run()
                step.assert_not_called()
        self.assertEqual(report["scene_id"], "integration_desk")
        self.assertEqual(report["failure"]["phase"], "RESET")
        self.assertEqual([r["phase"] for r in report["transitions"]], ["HOME", "RESET", "FAILURE"])
        self.assertFalse(report["success"])
        self.assertEqual(teacher.d.time, 0)
        np.testing.assert_array_equal(before, teacher.d.qpos)
        np.testing.assert_array_equal(teacher.d.ctrl, report["home"]["target_q"])
        self.assertEqual(teacher.m.site(teacher.site).name, "left_cube_grasp")
        self.assertEqual(set(teacher.fingers), {"jaw_1", "jaw_2"})
        self.assertEqual(len(report["provenance"]["model_sha256"]), 64)
        self.assertTrue(report["provenance"]["asset_sha256"])
        approved = build_scene("desk").compile()
        for field in ("body_pos", "body_quat", "geom_pos", "geom_quat", "geom_size",
                      "geom_type", "geom_contype", "geom_conaffinity",
                      "mesh_vert", "jnt_range", "actuator_trnid"):
            np.testing.assert_array_equal(getattr(approved, field), getattr(teacher.m, field))

    def test_table_stand_camera_and_fingers_fail_closed(self):
        env = task_env()
        pairs = protected_geom_pairs(env.model)
        for name in ("table", "stand_cad_bottom_1", "stand_cad_top_1",
                     "stand_cad_top_3", "os30a_enclosure_UNVERIFIED",
                     "left_pgripper_pad_1", "left_pgripper_pad_2", "right_pgripper_housing"):
            g = env.model.geom(name).id
            self.assertTrue(any(g in pair for pair in pairs), name)
            original = env.model.geom_contype[g]
            try:
                env.model.geom_contype[g] = 0
                with self.assertRaises(RuntimeError, msg=name):
                    protected_geom_pairs(env.model)
            finally:
                env.model.geom_contype[g] = original

    def test_failed_reset_cannot_settle_or_command(self):
        env = task_env()
        from dataclasses import replace
        env.config = replace(env.config, shoe_position_m=(.2,0.,0.))
        with self.assertRaises(ValueError):
            env.reset(seed=0)
        with self.assertRaisesRegex(ValueError, "validated"):
            env.settle()
        with self.assertRaisesRegex(ValueError, "validated"):
            env.apply_action(np.zeros(12), physics_steps=1)
        self.assertEqual(env.data.time, 0)
        self.assertFalse(block_metrics(env.model, env.data)["success"])
        with self.assertRaisesRegex(ValueError, "scene_id"):
            ShoeTaskEnv(ShoeTaskConfig(object_kind="block"), model=env.model)

    def test_mobile_has_no_silent_desk_or_tower_fallback(self):
        with self.assertRaisesRegex(ValueError, "UNVERIFIED"):
            task_env("mobile")

    def test_both_pgrippers_and_rgb_stands_preserve_confirmed_layout(self):
        from pgripper import MOTOR_MAX_RAD, jaw_gap_m
        from mobile_dual_so101 import apply_control_as_pose, actuator_targets_from_qpos
        for kind in ("desk", "mobile"):
            stock = build_scene(kind, grippers="stock").compile()
            model = build_scene(kind).compile()
            data = mujoco.MjData(model)
            mujoco.mj_forward(model, data)
            self.assertEqual(model.nu, 12)
            self.assertEqual(model.neq, 4)
            for name in ("left_base", "right_base", "os30a_plate_frame"):
                np.testing.assert_array_equal(model.body(name).pos, stock.body(name).pos)
                np.testing.assert_array_equal(model.body(name).quat, stock.body(name).quat)
            for name in ("stand_cad_bottom_1", "stand_cad_top_1", "stand_cad_top_3",
                         "os30a_enclosure_UNVERIFIED"):
                for field in ("pos", "quat", "size"):
                    np.testing.assert_array_equal(getattr(model.geom(name),field),
                                                  getattr(stock.geom(name),field))
            for side, index in (("left",5),("right",11)):
                self.assertGreater(model.geom(side+"_pgripper_camera_mount_visual").id, 0)
                self.assertGreaterEqual(model.camera(side+("_wrist_rgb" if kind=="desk" else "_wrist_cam")).id, 0)
                np.testing.assert_allclose(model.actuator_ctrlrange[index], [0,MOTOR_MAX_RAD])
                self.assertGreater(jaw_gap_m(model, data, side), .050)
            action = list(actuator_targets_from_qpos(model,data.qpos))
            action[5] = action[11] = 0
            apply_control_as_pose(model,data,action)  # isolated endpoint preview only
            for side in ("left","right"):
                self.assertGreater(jaw_gap_m(model,data,side), 0)
                self.assertLess(jaw_gap_m(model,data,side), .001)


if __name__ == "__main__":
    unittest.main()
