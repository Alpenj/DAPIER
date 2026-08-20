from pathlib import Path
import tempfile
import unittest

from shoe_sorting_data.index import build_index, query_index
from shoe_sorting_data.synthetic import generate_dataset, generate_episode


class EpisodeIndexTest(unittest.TestCase):
    def test_index_and_query_usable_validation_split(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir) / "episodes"
            generate_dataset(root, count=5)
            generate_episode(root / "episode_fault", fault="base_motion")
            database = Path(temp_dir) / "manifest.sqlite3"
            summary = build_index(root, database)
            self.assertEqual(summary, {"discovered": 6, "indexed": 6, "usable": 5, "invalid_manifest": 0})
            rows = query_index(database, usable=True, source_split="validation")
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]["episode_id"], "episode_000005")
            rejected = query_index(database, usable=False)
            self.assertEqual(len(rejected), 1)
            self.assertIn("base_interlock_violation", rejected[0]["issue_codes_json"])

    def test_reindex_is_idempotent(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir) / "episodes"
            generate_dataset(root, count=2)
            database = Path(temp_dir) / "manifest.sqlite3"
            build_index(root, database)
            build_index(root, database)
            self.assertEqual(len(query_index(database)), 2)

    def test_empty_root_and_missing_database_fail_cleanly(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir) / "episodes"
            root.mkdir()
            with self.assertRaisesRegex(ValueError, "no episode_manifest"):
                build_index(root, Path(temp_dir) / "manifest.sqlite3")
            with self.assertRaisesRegex(ValueError, "database not found"):
                query_index(Path(temp_dir) / "missing.sqlite3")


if __name__ == "__main__":
    unittest.main()
