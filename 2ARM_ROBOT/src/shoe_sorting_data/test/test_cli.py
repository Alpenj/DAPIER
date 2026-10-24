import contextlib
import io
from pathlib import Path
import tempfile
import unittest

from shoe_sorting_data.cli import main


class CliTest(unittest.TestCase):
    def test_generate_validate_index_query_flow(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir) / "episodes"
            database = Path(temp_dir) / "manifest.sqlite3"
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                self.assertEqual(main(["generate", "--root", str(root), "--count", "2"]), 0)
                self.assertEqual(
                    main(["validate", "--manifest", str(root / "episode_000001" / "episode_manifest.json")]),
                    0,
                )
                self.assertEqual(main(["index", "--root", str(root), "--db", str(database)]), 0)
                self.assertEqual(main(["query", "--db", str(database), "--usable", "true"]), 0)
            self.assertIn('"generated": 2', output.getvalue())
            self.assertIn('"usable": true', output.getvalue())

    def test_faulty_episode_returns_nonzero(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir) / "episodes"
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(
                    main(["generate", "--root", str(root), "--count", "1", "--fault", "base_motion"]),
                    0,
                )
                self.assertEqual(
                    main(["validate", "--manifest", str(root / "episode_000001" / "episode_manifest.json")]),
                    1,
                )

    def test_act_export_and_verify_flow(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            episodes = root / "episodes"
            interchange = root / "act_interchange"
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                self.assertEqual(main(["generate", "--root", str(episodes), "--count", "5"]), 0)
                self.assertEqual(
                    main(["act-export", "--root", str(episodes), "--output", str(interchange)]),
                    0,
                )
                self.assertEqual(main(["act-verify", "--root", str(interchange)]), 0)
            rendered = output.getvalue()
            self.assertIn('"state_dim": 12', rendered)
            self.assertIn('"native_lerobot_ready": false', rendered)
            self.assertIn('"passed": true', rendered)

    def test_native_status_and_preflight_do_not_require_lerobot(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            episodes = root / "episodes"
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                self.assertEqual(main(["generate", "--root", str(episodes), "--count", "2", "--samples", "3", "--camera-payload"]), 0)
                self.assertEqual(main(["native-status"]), 0)
                self.assertEqual(
                    main(["native-preflight", "--root", str(episodes), "--depth-unit", "mm"]),
                    0,
                )
            rendered = output.getvalue()
            self.assertIn('"base_recorder_affected": false', rendered)
            self.assertIn('"episode_count": 2', rendered)
            self.assertIn('"depth_unit": "mm"', rendered)

    def test_offline_evaluator_fixture_cli_flow(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            fixture = root / "fixture"
            report = root / "report.json"
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                self.assertEqual(main(["offline-eval-fixture", "--root", str(fixture)]), 0)
                self.assertEqual(
                    main(
                        [
                            "offline-eval",
                            "--manifest",
                            str(fixture / "evaluation_manifest.json"),
                            "--output",
                            str(report),
                        ]
                    ),
                    0,
                )
            rendered = output.getvalue()
            self.assertIn('"synthetic_fixture_only": true', rendered)
            self.assertIn('"valid_timestep_count": 12', rendered)
            self.assertIn('"closed_loop_status": "NOT_MEASURED"', rendered)

    def test_rollout_safety_smoke_cli_never_publishes(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            output_path = Path(temp_dir) / "rollout_trace.json"
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                self.assertEqual(
                    main(["rollout-safety-smoke", "--output", str(output_path)]),
                    0,
                )
            rendered = output.getvalue()
            self.assertIn('"scenario_count": 6', rendered)
            self.assertIn('"reject_count": 5', rendered)
            self.assertIn('"published_command_count": 0', rendered)
            self.assertIn('"hardware_dispatch_authorized_count": 0', rendered)

    def test_pair_and_skill_exemplar_cli_flow(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            registry = root / "pair_registry.json"
            episode = root / "episodes" / "episode_000001"
            exemplars = root / "exemplars" / "skill_001"
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                self.assertEqual(
                    main(
                        [
                            "pair-add",
                            "--registry",
                            str(registry),
                            "--exemplar-id",
                            "shoe_a",
                            "--pair-id",
                            "pair_a",
                            "--object-instance-id",
                            "object_a",
                            "--embedding",
                            "1,0,0",
                            "--session-id",
                            "session_a",
                            "--background-id",
                            "background_a",
                        ]
                    ),
                    0,
                )
                self.assertEqual(
                    main(["pair-match", "--registry", str(registry), "--embedding", "0.99,0.01,0"]),
                    0,
                )
                self.assertEqual(main(["generate", "--root", str(root / "episodes"), "--count", "1"]), 0)
                self.assertEqual(
                    main(
                        [
                            "skill-register",
                            "--manifest",
                            str(episode / "episode_manifest.json"),
                            "--output",
                            str(exemplars / "skill_exemplar.json"),
                            "--exemplar-id",
                            "skill_001",
                            "--precondition",
                            "base_stopped",
                            "--postcondition",
                            "shoe_placed",
                            "--timeout-ms",
                            "15000",
                            "--tag",
                            "grid",
                        ]
                    ),
                    0,
                )
            self.assertIn('"decision": "match"', output.getvalue())
            self.assertIn('"exemplar_id": "skill_001"', output.getvalue())


if __name__ == "__main__":
    unittest.main()
