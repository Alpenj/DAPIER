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
