from pathlib import Path
import tempfile
import unittest

from shoe_sorting_data.contract import load_manifest, save_manifest
from shoe_sorting_data.skill_exemplar import (
    audit_exemplar_leakage,
    build_skill_exemplar,
    retrieve_skill_exemplars,
    save_skill_exemplar,
)
from shoe_sorting_data.synthetic import generate_episode


class SkillExemplarTest(unittest.TestCase):
    def register(self, manifest_path: Path, output_path: Path, exemplar_id: str = "skill_exemplar_001"):
        exemplar = build_skill_exemplar(
            manifest_path,
            exemplar_id=exemplar_id,
            preconditions=["base_stopped", "camera_fresh", "target_reachable"],
            postconditions=["shoe_in_target", "gripper_released"],
            timeout_ms=15_000,
            tags=["grid", "side_by_side"],
            parameters={"target_zone": "grid_a"},
        )
        save_skill_exemplar(output_path, exemplar)
        return exemplar

    def test_register_and_retrieve_only_compatible_episode(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            source = generate_episode(root / "source" / "episode_000001", seed=10)
            query = generate_episode(root / "query" / "episode_000002", seed=11)
            exemplar_path = root / "exemplars" / "skill_exemplar_001" / "skill_exemplar.json"
            exemplar = self.register(source, exemplar_path)
            self.assertFalse(exemplar["execution_policy"]["control_authorized"])
            results = retrieve_skill_exemplars(
                root / "exemplars",
                query,
                skill_id="pair_and_place",
                tags=["grid"],
            )
            self.assertEqual([item["exemplar_id"] for item in results], ["skill_exemplar_001"])

            query_manifest = load_manifest(query)
            query_manifest["robot"]["calibration_version"] = "different_calibration"
            save_manifest(query, query_manifest)
            self.assertEqual(
                retrieve_skill_exemplars(root / "exemplars", query, skill_id="pair_and_place"),
                [],
            )

    def test_faulty_episode_cannot_be_registered(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            manifest = generate_episode(Path(temp_dir) / "episode_fault", fault="base_motion")
            with self.assertRaisesRegex(ValueError, "not usable"):
                build_skill_exemplar(
                    manifest,
                    exemplar_id="bad_exemplar",
                    preconditions=["base_stopped"],
                    postconditions=["shoe_placed"],
                    timeout_ms=10_000,
                )

    def test_validation_split_cannot_be_registered(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            manifest_path = generate_episode(Path(temp_dir) / "episode_validation", source_split="validation")
            with self.assertRaisesRegex(ValueError, "train or exemplar"):
                build_skill_exemplar(
                    manifest_path,
                    exemplar_id="leaky_exemplar",
                    preconditions=["base_stopped"],
                    postconditions=["shoe_placed"],
                    timeout_ms=10_000,
                )

    def test_leakage_audit_detects_object_session_and_background_overlap(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            source = generate_episode(root / "source" / "episode_000001", seed=20)
            evaluation = generate_episode(root / "evaluation" / "episode_000002", seed=21)
            exemplar_path = root / "exemplars" / "skill_exemplar_001" / "skill_exemplar.json"
            exemplar = self.register(source, exemplar_path)

            evaluation_manifest = load_manifest(evaluation)
            evaluation_manifest["provenance"]["object_instance_id"] = exemplar["leakage_keys"]["object_instance_id"]
            evaluation_manifest["provenance"]["session_id"] = exemplar["leakage_keys"]["session_id"]
            evaluation_manifest["provenance"]["background_id"] = exemplar["leakage_keys"]["background_id"]
            save_manifest(evaluation, evaluation_manifest)

            report = audit_exemplar_leakage(root / "exemplars", root / "evaluation")
            self.assertFalse(report["passed"])
            error_keys = {issue["key"] for issue in report["issues"] if issue["severity"] == "error"}
            warning_keys = {issue["key"] for issue in report["issues"] if issue["severity"] == "warning"}
            self.assertEqual(error_keys, {"object_instance_id", "session_id"})
            self.assertIn("background_id", warning_keys)


if __name__ == "__main__":
    unittest.main()
