"""SIM-only contact contract, both handover directions, and causal negative controls."""
import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import pgripper_handover as handover
from mobile_dual_so101 import resolve_so101_model


class PGripperHandoverTest(unittest.TestCase):
    def test_replay_outputs_and_callback(self):
        args = SimpleNamespace(output=Path("/tmp/example-run"), viewer=True)
        seen = []
        def run(current, *, replay_requested):
            self.assertFalse(replay_requested.is_set())
            seen.append(current.output)
            if len(seen) < 3:
                replay_requested.set()
            return 0
        with patch.object(handover, "run", side_effect=run):
            self.assertEqual(handover.main(args), 0)
        self.assertEqual(seen, [args.output, Path("/tmp/example-run-replay-001"),
                                Path("/tmp/example-run-replay-002")])
        self.assertEqual(args.output, Path("/tmp/example-run"))
        with tempfile.TemporaryDirectory() as tmp, contextlib.redirect_stdout(io.StringIO()):
            args = SimpleNamespace(output=Path(tmp) / "interrupted", viewer=True,
                model=resolve_so101_model(), donor="left", handover_only=False,
                render=False, disable_recipient_contact=False)
            closed = []
            def launch(model, data, *, key_callback):
                return SimpleNamespace(cam=SimpleNamespace(lookat=np.zeros(3)),
                    is_running=lambda: True, set_texts=lambda texts: None,
                    sync=lambda: key_callback(ord("R")), close=lambda: closed.append(True))
            with patch("mujoco.viewer.launch_passive", side_effect=launch):
                self.assertEqual(handover.run(args), 2)
            report = json.loads((args.output / "report.json").read_text())
        self.assertTrue(report["interrupted_for_replay"])
        self.assertFalse(report["task_success"])
        self.assertFalse(report["donor_release_started"])
        self.assertEqual(closed, [True])

    def test_exact_contact_pairs_and_invalid_forces(self):
        contacts = [(0, 10), (11, 0), (10, 0), (0, 12), (13, 0), (99, 10), (12, 11), (0, 10)]
        data = SimpleNamespace(contact=[SimpleNamespace(geom1=a, geom2=b) for a, b in contacts])
        pairs = {frozenset((0, 10 + i)): i for i in range(4)}
        values = [.3, .8, .4, .9, 1.1, 100, 100, -1]
        def force(model, state, index, wrench):
            wrench[0] = values[index]
        with patch.object(mujoco, "mj_contactForce", side_effect=force):
            np.testing.assert_allclose(handover.target_pad_forces(None, data, pairs), [[.7, .8], [.9, 1.1]])
            model = SimpleNamespace(geom=lambda name: SimpleNamespace(id={"table": 10, "red_block_geom": 0}[name]))
            self.assertAlmostEqual(handover.table_support_force(model, data), .7)
            for bad in (float("nan"), float("inf")):
                values[0] = bad
                with self.assertRaisesRegex(ValueError, "non-finite"):
                    handover.target_pad_forces(None, data, pairs)
                with self.assertRaisesRegex(ValueError, "non-finite"):
                    handover.table_support_force(model, data)
        self.assertEqual(handover.finite_list([1, np.nan, np.inf]), [1, None, None])

    def test_physics_both_directions_and_contact_ablation(self):
        source = resolve_so101_model()
        original_initialize = handover.apply_control_as_pose
        original_plan = handover.plan_septic_joint_trajectory
        original_forces = handover.target_pad_forces
        for donor, fault in (("right", None), ("left", None), ("right", "no_contact"), ("right", "release_loss")):
            with self.subTest(donor=donor, fault=fault), tempfile.TemporaryDirectory() as tmp:
                output = Path(tmp) / "result"
                captured = []
                inject_loss = False
                def initialize(model, data, action):
                    captured.append((model, data))
                    original_initialize(model, data, action)
                def plan(model, start, goal, **kwargs):
                    nonlocal inject_loss
                    donor_grip = 5 if donor == "left" else 11
                    if fault == "release_loss" and start[donor_grip] < 1.9 and goal[donor_grip] > start[donor_grip] + .1:
                        inject_loss = True
                    return original_plan(model, start, goal, **kwargs)
                def forces(model, data, pairs):
                    result = original_forces(model, data, pairs)
                    if inject_loss:
                        result[0 if donor == "right" else 1] = 0
                    return result
                args = SimpleNamespace(model=source, output=output, donor=donor,
                    disable_recipient_contact=fault == "no_contact", viewer=False, render=False, handover_only=True)
                with patch.object(handover, "apply_control_as_pose", side_effect=initialize), \
                     patch.object(handover, "plan_septic_joint_trajectory", side_effect=plan), \
                     patch.object(handover, "target_pad_forces", side_effect=forces), contextlib.redirect_stdout(io.StringIO()):
                    code = handover.run(args)
                report = json.loads((output / "report.json").read_text())
                self.assertEqual(len(captured), 1, "runtime must initialize only once")
                self.assertEqual(code, 2 if fault else 0, report["failure"])
                self.assertEqual(report["handover_success"], fault is None)
                self.assertEqual(report["donor_released"], fault is None)
                self.assertFalse(report["object_pose_writes_or_attachments"])
                self.assertFalse(report["hardware_execution"])
                self.assertEqual(report["runtime_qpos_writes_after_initialization"], 0)
                self.assertFalse(any(report["warning_counts"]))
                self.assertTrue(report["source_hashes_unchanged"])
                self.assertLessEqual(report["deepest_contact"]["depth_m"], .001)
                if fault == "release_loss":
                    self.assertTrue(report["donor_release_started"])
                    self.assertEqual(report["failure"]["phase"], "donor release")
                    self.assertIn("force lost", report["failure"]["reason"])
                    self.assertEqual(report["recipient_only_hold_s"], 0)
                elif fault == "no_contact":
                    self.assertFalse(report["donor_release_started"])
                    self.assertEqual(report["failure"]["phase"], "recipient close")
                    self.assertIn("timeout", report["failure"]["reason"])
                    self.assertEqual(report["recipient_only_hold_s"], 0)
                else:
                    self.assertGreaterEqual(report["recipient_only_hold_s"], 3.0)
                    self.assertGreaterEqual(report["grip_confirmations"][report["recipient"]]["stable_for_s"], .33)
                    # Remove only contact AFTER the real handover, without changing object pose.
                    model, data = captured[0]
                    start_height = float(data.body("red_block").xpos[2])
                    for finger in (1, 2):
                        geom = model.geom(f'{report["recipient"]}_pgripper_pad_{finger}')
                        geom.contype = geom.conaffinity = 0
                    for _ in range(int(np.ceil(1 / model.opt.timestep))):
                        mujoco.mj_step(model, data)
                    self.assertGreater(start_height - data.body("red_block").xpos[2], .04)
                with self.assertRaisesRegex(ValueError, "new output"):
                    handover.run(args)

    def test_left_pick_handover_and_right_place(self):
        original_support = handover.table_support_force
        original_plan = handover.plan_septic_joint_trajectory
        original_forces = handover.target_pad_forces
        for fault in (None, "support_loss", "seat_grip_loss"):
            with self.subTest(fault=fault), tempfile.TemporaryDirectory() as tmp:
                output = Path(tmp) / "task"
                support_seen = False
                inject_loss = False
                def support(model, data):
                    nonlocal support_seen
                    if fault == "support_loss" and support_seen:
                        return 0.0
                    result = original_support(model, data)
                    support_seen = result >= .1
                    return result
                def plan(model, start, goal, **kwargs):
                    nonlocal inject_loss
                    if fault == "seat_grip_loss" and kwargs.get("minimum_duration_s") == 8.0:
                        inject_loss = True
                    return original_plan(model, start, goal, **kwargs)
                def forces(model, data, pairs):
                    result = original_forces(model, data, pairs)
                    if inject_loss:
                        result[1] = 0.0
                    return result
                args = SimpleNamespace(model=resolve_so101_model(), output=output, donor="left",
                    disable_recipient_contact=False, viewer=False, render=False, handover_only=False)
                with patch.object(handover, "table_support_force", side_effect=support), \
                     patch.object(handover, "plan_septic_joint_trajectory", side_effect=plan), \
                     patch.object(handover, "target_pad_forces", side_effect=forces), \
                     contextlib.redirect_stdout(io.StringIO()):
                    code = handover.run(args)
                report = json.loads((output / "report.json").read_text())
                self.assertEqual(code, 2 if fault else 0, report["failure"])
                self.assertTrue(report["handover_success"])
                self.assertEqual(report["task_success"], fault is None)
                self.assertEqual(report["place_release_started"], fault is None)
                self.assertGreaterEqual(report["recipient_only_hold_s"], 3)
                self.assertFalse(any(report["warning_counts"]))
                self.assertTrue(report["source_hashes_unchanged"])
                if fault:
                    self.assertEqual(report["failure"]["phase"],
                        "place support confirm" if fault == "support_loss" else "place seat")
                    self.assertIn("release forbidden" if fault == "support_loss" else "force lost",
                        report["failure"]["reason"])
                    self.assertEqual(report["table_supported_hold_s"], 0)
                else:
                    self.assertLess(report["simulation_seconds"], 100, "do not restore intermediate pauses/slow free closure")
                    self.assertGreaterEqual(report["table_supported_hold_s"], 3)
                    np.testing.assert_allclose(report["final_block_position_m"], [.22, -.08, .02], atol=.003)


if __name__ == "__main__":
    unittest.main()
