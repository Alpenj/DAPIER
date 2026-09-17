import sys
from pathlib import Path
import unittest
from unittest.mock import patch
from types import SimpleNamespace
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from center_block_teacher import CenterBlockTeacher
from mobile_dual_so101 import actuator_targets_from_qpos


class CenterTeacherTest(unittest.TestCase):
    def test_failed_pregrasp_uses_settled_input_and_never_closes_or_lifts(self):
        teacher=CenterBlockTeacher(scene="legacy_tower")
        with patch.object(teacher,"solve",side_effect=ValueError("test unreachable pregrasp")):
            report=teacher.run()
        self.assertFalse(report["success"])
        self.assertEqual(report["failure"]["phase"],"PREGRASP")
        self.assertEqual([r["phase"]for r in report["transitions"]],
                         ["RESET","SETTLE","PREGRASP","FAILURE"])
        self.assertAlmostEqual(teacher.d.time,.2)
        self.assertLess(report["teacher_input"]["block_position_m"][2],.020)
        np.testing.assert_array_equal(report["teacher_input"]["block_position_m"],
                                      teacher.d.xpos[teacher.m.body("shoe").id])
        np.testing.assert_array_equal(report["teacher_input"]["measured_q"],
                                      actuator_targets_from_qpos(teacher.m,teacher.d.qpos))
        self.assertEqual(report["maximum_lift_above_settled_bottom_m"],0)

    def test_full_path_rejection_prevents_physics(self):
        teacher=CenterBlockTeacher(scene="legacy_tower")
        teacher.env.reset(seed=0)
        target=actuator_targets_from_qpos(teacher.m,teacher.d.qpos)
        before=teacher.d.qpos.copy()
        rejected=SimpleNamespace(safe=False,reason="test path collision",as_report=lambda:{"safe":False})
        with patch("center_block_teacher.check_bimanual_path",return_value=rejected):
            with self.assertRaisesRegex(ValueError,"full interpolated path rejected"):
                teacher.move(target)
        np.testing.assert_array_equal(before,teacher.d.qpos)
        self.assertEqual(teacher.d.time,0)


if __name__=="__main__":
    unittest.main()
