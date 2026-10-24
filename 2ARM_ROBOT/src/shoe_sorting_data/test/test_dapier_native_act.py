import ast
import importlib.util
import inspect
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

import shoe_sorting_data.dapier_native_act as native_act
from shoe_sorting_data.dapier_native_act import ActionChunkQueue, dependency_status, run_smoke


class DAPIERNativeACTTest(unittest.TestCase):
    def test_checkpoint_load_never_retries_without_weights_only(self):
        torch = Mock()
        torch.load.side_effect = TypeError("weights_only unavailable")
        with patch.object(native_act, "_require_ml", return_value=(None, torch)):
            with self.assertRaisesRegex(TypeError, "weights_only unavailable"):
                native_act.load_checkpoint("untrusted.pt")
        torch.load.assert_called_once_with(Path("untrusted.pt"), map_location="cpu", weights_only=True)

    def test_dependency_boundary_and_stale_chunk_reset(self):
        self.assertNotIn("lerobot", dependency_status()["modules"])
        self.assertFalse(dependency_status()["lerobot_required"])
        tree = ast.parse(inspect.getsource(native_act))
        imported_roots = {
            alias.name.split(".")[0]
            for node in ast.walk(tree)
            if isinstance(node, (ast.Import, ast.ImportFrom))
            for alias in node.names
        }
        self.assertNotIn("lerobot", imported_roots)
        queue = ActionChunkQueue(2)
        queue.load([[1], [2], [3]])
        self.assertEqual(queue.pop(), [1])
        queue.reset()
        with self.assertRaisesRegex(RuntimeError, "fresh policy chunk"):
            queue.pop()

    @unittest.skipUnless(
        importlib.util.find_spec("torch") is not None and importlib.util.find_spec("numpy") is not None,
        "optional DAPIER-native ACT ML environment is not installed",
    )
    def test_full_independent_smoke(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            receipt = run_smoke(Path(temp_dir) / "smoke")
            self.assertEqual(receipt["status"], "PASS")
            self.assertFalse(receipt["lerobot_required"])
            self.assertFalse(receipt["lerobot_loaded_by_runtime"])
            self.assertTrue(receipt["checkpoint_roundtrip"])
            self.assertTrue(receipt["stale_chunk_blocked"])


if __name__ == "__main__":
    unittest.main()
