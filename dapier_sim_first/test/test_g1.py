from __future__ import annotations

import json
from pathlib import Path
import stat
import subprocess
import tempfile
import unittest

from dapier_sim_first.embodiment import so101_new_calibration_spec
from dapier_sim_first.g1 import (
    G1_EXECUTION_CONTRACT,
    G1_FRAMES,
    G1_RATE_HZ,
    G1_RECORD_ID,
    G1_SEED,
    G1_TASK_CONFIG,
    G1TraceFrame,
    evaluate_pick_lift,
    initialize_g1_manifest,
    scripted_g1_action,
    simulator_substeps,
)
from dapier_sim_first.gate import _parser


class ScriptedControllerContractTest(unittest.TestCase):
    def test_trace_is_exactly_300_bounded_deterministic_frames(self) -> None:
        embodiment = so101_new_calibration_spec("sha256:" + "2" * 64)
        first = [scripted_g1_action(index) for index in range(G1_FRAMES)]
        second = [scripted_g1_action(index) for index in range(G1_FRAMES)]

        self.assertEqual(first, second)
        self.assertEqual(len(first), G1_FRAMES)
        self.assertTrue(all(len(action) == 6 for action in first))
        for action in first:
            embodiment.action_to_sim(action)

        boundaries = G1_TASK_CONFIG["controller"]["phase_boundaries"]
        self.assertEqual(boundaries, [0, 30, 110, 170, 190, 260, 300])
        self.assertEqual(first[0], first[29])
        self.assertEqual(first[260], first[299])
        self.assertEqual(sum(simulator_substeps(index) for index in range(300)), 5000)

    def test_only_g0_and_g1_are_exposed(self) -> None:
        parser = _parser()
        subparser_action = next(
            action for action in parser._actions if action.dest == "command"
        )
        self.assertEqual(
            set(subparser_action.choices), {"init-g0", "g0", "init-g1", "g1"}
        )


class PickLiftEvaluatorTest(unittest.TestCase):
    @staticmethod
    def _trace(*, support_during_hold: bool = False) -> list[G1TraceFrame]:
        traces: list[G1TraceFrame] = []
        for index in range(G1_FRAMES):
            lifted = index >= 240
            traces.append(
                G1TraceFrame(
                    frame_index=index,
                    timestamp_ns=index,
                    measured_action_units=(0.0,) * 6,
                    commanded_action=(0.0,) * 6,
                    image=None,
                    cube_position=(0.2, 0.0, 0.08 if lifted else 0.05),
                    fixed_pad_contact=lifted,
                    moving_pad_contact=lifted,
                    support_contact=support_during_hold and index >= 270,
                )
            )
        return traces

    @staticmethod
    def _runtime() -> dict[str, object]:
        return {
            "initial_cube_position": [0.2, 0.0, 0.055],
            "configured_cube_xy": [0.2, 0.0],
            "total_substeps": 5000,
            "simulated_duration_s": 10.0,
        }

    def test_requires_lift_bilateral_contact_and_clear_support(self) -> None:
        result = evaluate_pick_lift(self._trace(), runtime=self._runtime())

        self.assertTrue(result["success"])
        self.assertEqual(result["status"], "PASS")
        self.assertAlmostEqual(result["object"]["minimum_lift_during_hold_m"], 0.03)
        self.assertEqual(result["contact"]["bilateral_pad_contact_hold_frames"], 30)
        self.assertEqual(result["contact"]["support_contact_hold_frames"], 0)
        self.assertFalse(result["integrity"]["weld_or_equality_grasp"])

    def test_support_contact_during_hold_fails(self) -> None:
        result = evaluate_pick_lift(
            self._trace(support_during_hold=True), runtime=self._runtime()
        )
        self.assertFalse(result["success"])
        self.assertEqual(result["status"], "FAIL")


class G1ManifestSafetyTest(unittest.TestCase):
    def test_manifest_is_fresh_read_only_and_scripted(self) -> None:
        repo_root = Path(__file__).resolve().parents[2]
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            model = root / "model.xml"
            calibration = root / "calibration.xml"
            model.write_text("<mujoco/>\n", encoding="utf-8")
            calibration.write_text("<mujoco/>\n", encoding="utf-8")
            lerobot_root = root / "lerobot"
            required = {
                "src/lerobot/envs/so101_mujoco/env.py": "# adapter\n",
                "src/lerobot/datasets/lerobot_dataset.py": "# writer\n",
                "pyproject.toml": '[project]\nname="lerobot"\nversion="0.6.0"\n',
            }
            for relative, content in required.items():
                path = lerobot_root / relative
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(content, encoding="utf-8")
            subprocess.run(["git", "init", "-q", str(lerobot_root)], check=True)
            subprocess.run(
                ["git", "-C", str(lerobot_root), "config", "user.name", "test"],
                check=True,
            )
            subprocess.run(
                [
                    "git",
                    "-C",
                    str(lerobot_root),
                    "config",
                    "user.email",
                    "test@example.invalid",
                ],
                check=True,
            )
            subprocess.run(["git", "-C", str(lerobot_root), "add", "."], check=True)
            subprocess.run(
                ["git", "-C", str(lerobot_root), "commit", "-qm", "fixture"],
                check=True,
            )
            run_root = root / "new-g1-run"

            manifest_path = initialize_g1_manifest(
                run_root=run_root,
                repo_root=repo_root,
                model_path=model,
                calibration_path=calibration,
                lerobot_root=lerobot_root,
            )
            payload = json.loads(manifest_path.read_text(encoding="utf-8"))

            self.assertEqual(payload["record_id"], G1_RECORD_ID)
            self.assertEqual(payload["gate"], "G1")
            self.assertEqual(payload["seed"], G1_SEED)
            self.assertEqual(payload["rate_hz"], G1_RATE_HZ)
            self.assertEqual(payload["frames"], G1_FRAMES)
            self.assertEqual(payload["execution_contract"], G1_EXECUTION_CONTRACT)
            self.assertEqual(
                payload["provenance_contract"],
                {"source": "scripted", "human_demo": False},
            )
            self.assertFalse(manifest_path.stat().st_mode & stat.S_IWUSR)

            with self.assertRaisesRegex(ValueError, "must not already exist"):
                initialize_g1_manifest(
                    run_root=run_root,
                    repo_root=repo_root,
                    model_path=model,
                    calibration_path=calibration,
                    lerobot_root=lerobot_root,
                )


if __name__ == "__main__":
    unittest.main()
