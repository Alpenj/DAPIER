"""An approved-looking IK JSON cannot bypass the native plan/profile boundary."""
from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import tempfile
import unittest

from so101.hardware_tools.motion.execute_bounded_pregrasp import main


class PregraspRefusalTest(unittest.TestCase):
    def test_success_shaped_input_is_refused_without_overwriting(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "ik.json"
            output = Path(directory) / "result.json"
            source.write_text(json.dumps({"ik_converged": True, "accepted_for_execution": True}))
            args = ["--ik-result", str(source), "--output", str(output)]
            with redirect_stdout(io.StringIO()):
                self.assertEqual(main(args), 1)
            report = json.loads(output.read_text())
            self.assertEqual(report["rejection_stage"], "IK_PLAN_AUDIT")
            self.assertFalse(report["motion_started"])
            self.assertEqual(sum(report["motor_writes"].values()), 0)
            before = output.read_bytes()
            with self.assertRaises(FileExistsError):
                main(args)
            self.assertEqual(output.read_bytes(), before)


if __name__ == "__main__":
    unittest.main()
