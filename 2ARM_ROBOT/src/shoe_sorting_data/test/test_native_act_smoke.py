import importlib.util
from pathlib import Path
import tempfile
import unittest

from shoe_sorting_data.native_act_smoke import (
    EXPECTED_TAIL_MASKS,
    native_act_dependency_status,
    run_native_act_smoke,
)


class NativeActSmokeTest(unittest.TestCase):
    def test_dependency_probe_is_lazy_and_preserves_base_recorder(self):
        status = native_act_dependency_status()
        self.assertEqual(
            set(status["modules"]),
            {"lerobot", "torch", "torchvision", "numpy", "datasets", "pyarrow"},
        )
        self.assertFalse(status["base_recorder_affected"])
        self.assertEqual(
            EXPECTED_TAIL_MASKS,
            (
                (False, False, False),
                (False, False, True),
                (False, True, True),
            ),
        )

    @unittest.skipIf(importlib.util.find_spec("lerobot") is not None, "base-environment SKIP path only")
    def test_missing_optional_environment_is_recorded_as_skip(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            receipt = run_native_act_smoke(root, repo_id="local/test")
            self.assertEqual(receipt["status"], "SKIP")
            self.assertTrue((root / "dapier_act_roundtrip_receipt.json").is_file())

    def test_stage3_fixture_rejects_non_three_chunk(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            with self.assertRaisesRegex(ValueError, "chunk_size=3"):
                run_native_act_smoke(temp_dir, repo_id="local/test", chunk_size=4)


if __name__ == "__main__":
    unittest.main()
