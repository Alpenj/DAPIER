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


if __name__ == "__main__":
    unittest.main()
