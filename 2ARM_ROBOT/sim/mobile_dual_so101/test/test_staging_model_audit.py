from pathlib import Path
import copy
import json
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import sys
import mujoco
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from staging_model_audit import compare_models,refresh_reference
from runtime_contact_candidate import RuntimeCandidateTeacher

ROOT=Path(__file__).resolve().parents[1]
XML="""<mujoco><asset><mesh name="tetra" vertex="0 0 0 .1 0 0 0 .1 0 0 0 .1"/></asset>
<worldbody><body name="arm" pos="0 0 1"><joint name="j" range="-1 1"/>
<geom type="mesh" mesh="tetra" mass="1"/></body></worldbody>
<actuator><position joint="j" kp="10" ctrlrange="-1 1"/></actuator>
<equality><joint joint1="j" polycoef="0 1 0 0 0"/></equality></mujoco>"""


class StagingIdentityTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model=mujoco.MjModel.from_xml_string(XML)

    def test_native_serialization_does_not_create_a_physics_difference(self):
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory)/'model.mjb'
            mujoco.mj_saveModel(self.model,str(p),None)
            saved=mujoco.MjModel.from_binary_path(str(p))
        r=compare_models(saved,self.model)
        self.assertEqual(r['classification'],'A')
        self.assertEqual(r['physics_array_differences'],[])
        self.assertEqual(r['option_differences'],{})
        self.assertTrue(r['mesh_hashes_equal'])
        self.assertEqual(r['serialized_compilation_signatures'],[0,0])

    def test_physics_changes_cannot_refresh_identity(self):
        original=self.model
        mutations=(('body_pos',(1,0)),('body_mass',1),('body_inertia',(1,0)),
            ('geom_friction',(0,0)),('geom_solref',(0,0)),('jnt_range',(0,0)),
            ('actuator_gainprm',(0,0)),('actuator_gear',(0,0)),('eq_data',(0,0)),
            ('mesh_vert',(0,0)))
        for field,index in mutations:
            with self.subTest(field=field):
                changed=copy.copy(original);getattr(changed,field)[index]+=.0001
                self.assertEqual(compare_models(original,changed)['classification'],'B')
        changed=copy.copy(original);changed.opt.timestep*=2
        self.assertEqual(compare_models(original,changed)['classification'],'B')

    def test_refresh_preserves_parent_and_rejects_wrong_binary_or_physics(self):
        old=dict(provenance=dict(model_sha256='saved',scene_id='integration_desk',gripper_revision='both'),
            staging_search=dict(selected=dict(offset_m=.01)),saved_state=[1,2,3])
        before=copy.deepcopy(old);env=SimpleNamespace(model=self.model)
        current=dict(model_sha256='current',scene_id='integration_desk',gripper_revision='both')
        with patch('staging_model_audit.task_provenance',return_value=current):
            refreshed,_=refresh_reference(old,self.model,env,saved_raw_sha256='saved')
        self.assertEqual(old,before)
        self.assertEqual(refreshed['provenance'],current)
        self.assertEqual(refreshed['provenance_refresh']['original_provenance'],old['provenance'])
        self.assertNotIn('saved_state',refreshed)
        self.assertEqual(refreshed['staging_search']['selected'],dict(offset_m=.01))
        with self.assertRaisesRegex(ValueError,'does not match'):
            refresh_reference(old,self.model,env,saved_raw_sha256='wrong')
        changed=copy.copy(self.model);changed.opt.impratio=100
        with self.assertRaisesRegex(ValueError,'physical/model difference'):
            refresh_reference(old,changed,env,saved_raw_sha256='saved')

    def test_connection_stop_never_enters_close_or_claims_task_success(self):
        t=object.__new__(RuntimeCandidateTeacher);t.connection_only=True
        t.report=dict(success=False,live_task_success=False);t.transition=lambda phase:setattr(t,'phase',phase)
        with patch.object(RuntimeCandidateTeacher,'move') as move,patch.object(mujoco,'mj_step') as step:
            t.finish_task(None)
        move.assert_not_called();step.assert_not_called()
        self.assertEqual(t.phase,'A2_CONNECTION_READY')
        self.assertTrue(t.report['connection_pass'])
        self.assertFalse(t.report['success'] or t.report['live_task_success'])

    def test_actual_refreshed_gate_and_existing_collision_refusal(self):
        r=json.loads((ROOT/'test/fixtures/staging_identity_connection.json').read_text())
        self.assertEqual(r['classification'],'A')
        self.assertEqual(r['physics_array_differences'],[])
        self.assertEqual(r['option_differences'],{})
        self.assertTrue(r['mesh_hashes_equal'])
        s=r['connection']
        self.assertEqual(s['live_physics_steps'],100)
        self.assertFalse(s['center_success'])
        self.assertEqual(s['first_rejected_segment']['phase'],'SAFE_STAGE')
        guard=s['first_rejected_segment']['guard']
        self.assertFalse(guard['safe'])
        self.assertLess(guard['minimum_clearance_m'],guard['required_clearance_m'])
        self.assertEqual(guard['required_clearance_m'],.03)
        self.assertEqual([guard['first_geom_id'],guard['second_geom_id']],[36,74])
        self.assertEqual(s['waypoint_gates'],[])  # No arm motion executed.


if __name__=='__main__':unittest.main()
