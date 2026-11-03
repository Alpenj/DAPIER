import json
from pathlib import Path
import tempfile
import unittest

from casino_dealer.episode_manifest import (
    EPISODE_MANIFEST_SCHEMA_VERSION,
    build_manifest,
    load_manifest,
    save_manifest,
    validate_manifest,
)
from casino_dealer.episode_cli import main


class EpisodeManifestTest(unittest.TestCase):

    def make_manifest(self):
        return build_manifest(
            episode_id="episode_000001",
            task="Pick one card and place it on player_1.",
            skill="pick_and_place_card",
            source="lerobot",
            fps=30,
            cameras=["front"],
            arms=[
                {
                    "name": "right",
                    "follower_id": "so101_follower_main",
                    "leader_id": "so101_leader_main",
                }
            ],
        )

    def test_new_manifest_has_recorded_pending_outcome(self):
        manifest = self.make_manifest()

        self.assertEqual(
            manifest["schema_version"],
            EPISODE_MANIFEST_SCHEMA_VERSION,
        )
        self.assertEqual(manifest["outcome"]["status"], "recorded")
        self.assertIsNone(manifest["outcome"]["success"])
        validate_manifest(manifest)

    def test_failed_episode_requires_reason(self):
        manifest = self.make_manifest()
        manifest["outcome"] = {
            "status": "rejected",
            "success": False,
            "failure_reason": "card dropped",
        }
        validate_manifest(manifest)

        manifest["outcome"]["failure_reason"] = ""
        with self.assertRaisesRegex(ValueError, "failure_reason"):
            validate_manifest(manifest)

    def test_round_trip_and_cli_mark(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "episode_000001" / "episode_manifest.json"
            save_manifest(path, self.make_manifest())
            self.assertEqual(load_manifest(path)["task"]["skill"], "pick_and_place_card")

            exit_code = main(
                [
                    "mark",
                    "--path",
                    str(path),
                    "--status",
                    "accepted",
                    "--success",
                    "true",
                ]
            )
            self.assertEqual(exit_code, 0)
            marked = json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual(marked["outcome"]["status"], "accepted")
            self.assertTrue(marked["outcome"]["success"])


if __name__ == "__main__":
    unittest.main()
