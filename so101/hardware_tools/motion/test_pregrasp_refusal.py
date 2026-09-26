"""An approved-looking IK JSON cannot bypass the native plan/profile boundary."""
from contextlib import redirect_stderr, redirect_stdout
import io
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from so101.hardware_tools.motion.execute_bounded_pregrasp import main


class PregraspRefusalTest(unittest.TestCase):
    def test_carry_is_mock_only_and_keeps_endpoint_and_support_distinct(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source, native = root/"candidate.json", root/"native"
            native.write_bytes(b"MOCK native identity")
            candidate = {"schema_version":"dapier.offline-ik-candidate.v1",
                         "candidate_mode":"carry_endpoint_ik", "planning_phase":"LIFT",
                         "model":{"compiled_sha256":"MOCK"}}
            args = ["--ik-result",str(source),"--profile",str(root/"profile"),
                    "--previous-phase-trace",str(root/"prior"),
                    "--object-observation-binding",str(root/"binding"),
                    "--native-executor",str(native),"--native-sha256",hashlib.sha256(native.read_bytes()).hexdigest()]
            source.write_text(json.dumps(candidate))
            (root/"profile").write_text("MOCK profile identity")
            profile_sha = hashlib.sha256((root/"profile").read_bytes()).hexdigest()
            with patch("so101.hardware_tools.motion.execute_bounded_pregrasp.subprocess.run") as run, \
                 redirect_stdout(io.StringIO()):
                self.assertEqual(main(args+["--output",str(root/"hw.json"),"--transport","hardware"]), 1)
                run.assert_not_called()
            refusal = json.loads((root/"hw.json").read_text())
            self.assertIn("MOCK transport", refusal["reason"])
            self.assertEqual(sum(refusal["motor_writes"].values()), 0)
            # Substitute only native invocation/FK oracle here; the native loop
            # and actual FK semantics are covered by their separate regressions.
            for i, (phase, endpoint_ok, support_ok, bound, expected) in enumerate((
                    ("LIFT",True,False,True,0), ("LIFT",False,True,True,1),
                    ("PLACE",False,True,True,0), ("PLACE",True,False,True,1),
                    ("PLACE",False,True,False,1))):
                candidate["planning_phase"] = phase
                source.write_text(json.dumps(candidate))
                output = root/f"result-{i}.jsonl"
                def invoke(command, **_kwargs):
                    plan_sha = hashlib.sha256(Path(command[command.index("--plan")+1]).read_bytes()).hexdigest()
                    # Unbound case: a result from another plan (e.g. prior phase) is reused.
                    output.write_text(json.dumps({"event":"result","hardware_execution":False,
                        "plan_sha256":plan_sha if bound else "0"*64, "profile_sha256":profile_sha})+"\n")
                    return type("Completed", (), {"returncode":0})()
                endpoint = {"kinematic_endpoint_within_tolerance":endpoint_ok,
                            "model_supported_stop_verified":support_ok,
                            "cartesian_endpoint_verified":False,"task_success":False}
                with patch("dapier_research.real_sensor_ik_adapter.bounded_carry_plan",
                           return_value={"schema_version":"dapier.bounded-pregrasp-plan.v1"}), \
                     patch("so101.hardware_tools.motion.execute_bounded_pregrasp.subprocess.run", side_effect=invoke), \
                     patch("evaluate_single_shot_ik.check_native_feedback_endpoint", return_value=endpoint), \
                     redirect_stderr(io.StringIO()):
                    self.assertEqual(main(args+["--output",str(output)]), expected)
                    if bound:
                        self.assertEqual(json.loads(output.with_suffix(".endpoint.json").read_text()), endpoint)
                    else:
                        self.assertFalse(output.with_suffix(".endpoint.json").exists())

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
