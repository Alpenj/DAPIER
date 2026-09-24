"""An approved-looking IK JSON cannot bypass the native plan/profile boundary."""
from contextlib import redirect_stdout
import io
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from so101.hardware_tools.motion.execute_bounded_pregrasp import main


class PregraspRefusalTest(unittest.TestCase):
    def test_explicit_budget_is_bound_to_generated_plan_only(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source, output, native = root/"ik.json", root/"result.json", root/"native"
            native.write_bytes(b"MOCK native binary identity")
            source.write_text(json.dumps({"schema_version":"dapier.offline-ik-candidate.v1"}))
            args = ["--ik-result",str(source),"--output",str(output),"--profile",str(root/"profile"),
                    "--native-executor",str(native),"--native-sha256",hashlib.sha256(native.read_bytes()).hexdigest(),
                    "--maximum-duration-s","40"]
            plan = {"schema_version":"dapier.bounded-pregrasp-plan.v1","maximum_duration_s":40.}
            with patch("dapier_research.real_sensor_ik_adapter.bounded_pregrasp_plan", return_value=plan) as prepare, \
                 patch("so101.hardware_tools.motion.execute_bounded_pregrasp.subprocess.run") as run:
                run.return_value.returncode = 1
                self.assertEqual(main(args), 1)
                self.assertEqual(prepare.call_args.kwargs["maximum_duration_s"], 40.)
                self.assertEqual(json.loads(output.with_suffix(".plan.json").read_text()), plan)
                self.assertIn("mock", run.call_args.args[0])
            # An already reviewed plan cannot silently acquire a different budget.
            source.write_text(json.dumps(plan))
            args[args.index("--output")+1] = str(root/"rejected.json")
            with patch("so101.hardware_tools.motion.execute_bounded_pregrasp.subprocess.run") as run, \
                 redirect_stdout(io.StringIO()):
                self.assertEqual(main(args), 1)
                run.assert_not_called()
            self.assertIn("cannot override", json.loads((root/"rejected.json").read_text())["reason"])

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
