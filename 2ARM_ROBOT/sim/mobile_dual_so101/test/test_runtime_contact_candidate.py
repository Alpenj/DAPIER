from pathlib import Path
import copy
import json
import sys
import unittest
from unittest.mock import patch
import numpy as np
import mujoco
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from runtime_contact_candidate import RuntimeCandidateTeacher
from waypoint_block_teacher import WaypointBlockTeacher
from controlled_contact_audit import array_hashes, options

ROOT=Path(__file__).resolve().parents[1]


class RuntimeCandidateTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config=json.loads((ROOT/'config/runtime_contact_candidate.json').read_text())
        cls.fixture=json.loads((ROOT/'test/fixtures/runtime_candidate_gate.json').read_text())
        cls.t=RuntimeCandidateTeacher({},dict(provenance=cls.fixture['saved_provenance']),cls.config)

    def test_opt_in_changes_only_impratio_and_never_initial_state(self):
        t=self.t
        baseline=WaypointBlockTeacher(t.report['candidate'])
        self.assertEqual(array_hashes(t.m),array_hashes(baseline.m))
        a,b=options(baseline.m),options(t.m)
        self.assertEqual([k for k in a if a[k]!=b[k]],['impratio'])
        self.assertEqual((b['noslip_iterations'],b['impratio']),(0,100.))
        np.testing.assert_array_equal(t.d.qpos,baseline.d.qpos)
        np.testing.assert_array_equal(t.d.qvel,baseline.d.qvel)
        self.assertEqual(t.d.time,0.)

    def test_frozen_commands_and_a2_geometry_are_not_checkpoint_state(self):
        c=self.config
        self.assertFalse({'qpos','qvel','state','warmstart','act','settle'} & c.keys())
        self.assertEqual(len(c['close_commands_rad']),49)
        self.assertEqual(c['close_commands_rad'][-1],1.716112023423022)
        self.assertEqual(c['task_open_rad'],[2.2025746379457667]*2)
        T=np.asarray(c['target_TCP'])
        np.testing.assert_allclose(c['pregrasp_xyz_m'],T[:3,3]-.02*T[:3,0],atol=1e-15,rtol=0)
        old=json.loads((ROOT/'test/fixtures/global_impratio.json').read_text())
        self.assertEqual(c['close_commands_rad'],old['close_commands_rad'])
        self.assertEqual([p['phase'] for p in c['lift_commands']],['LIFT_5MM','LIFT_15MM','LIFT_30MM'])

    def test_saved_geometry_mismatch_stops_before_trial_or_physics(self):
        t=self.t
        t.staging_reference['provenance']['scene_id']=t.report['provenance']['scene_id']
        t.report['teacher_input']={}
        t.configure_task_open()
        before=t.d.qpos.copy()
        with patch.object(t,'evaluate_waypoint') as ik,patch.object(mujoco,'mj_step') as step:
            with self.assertRaisesRegex(ValueError,'saved staging geometry changed: model_sha256'):
                t.staging_plan(None,None,require_dynamic=True)
            ik.assert_not_called()
            step.assert_not_called()
        np.testing.assert_array_equal(t.d.qpos,before)

    def test_clone_observation_cannot_record_live_success(self):
        t=self.t;t.live_data=t.d;t.live_recorder=lambda obj: self.fail('clone contaminated live recorder')
        clone=copy.copy(t);clone.d=mujoco.MjData(t.m)
        with patch.object(WaypointBlockTeacher,'record_step') as inherited:
            clone.record_step()
            inherited.assert_called_once()

    def test_actual_first_failure_does_not_promote_copied_success(self):
        f=self.fixture
        self.assertTrue(f['home_pass'] and f['reset_pass'] and f['settle_pass'])
        self.assertEqual(f['live_physics_steps'],100)
        self.assertEqual(f['copied_preflight_steps'],0)
        self.assertEqual(f['failure']['phase'],'STAGING_DIAGNOSTIC')
        self.assertFalse(f['center_success'])
        self.assertEqual(f['actuator_waypoint_gates'],[])
        self.assertEqual(f['hold_duration_s'],0)
        self.assertEqual(f['final_forces_N'],[0,0])
        self.assertEqual(f['warnings'],0)
        self.assertFalse(f['saturation_any'])


if __name__=='__main__':unittest.main()
