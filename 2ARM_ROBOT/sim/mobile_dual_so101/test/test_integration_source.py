import copy
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
import mujoco
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from integration_scenes import build_scene, task_env, portable_model_sha256, preserve_desk_source_profile
from mobile_dual_so101 import resolve_so101_model


class IntegrationSourceTest(unittest.TestCase):
    def test_approved_desk_contract_and_stock_source_preserved(self):
        source=resolve_so101_model();before=source.read_bytes()
        stock=mujoco.MjSpec.from_file(str(source)).compile()
        model=build_scene('desk').compile()
        profile=json.loads(Path(__file__).parents[1].joinpath('integration_desk_source.json').read_text())
        for side in ('left','right'):
            np.testing.assert_array_equal(model.joint(side+'_shoulder_lift').range,profile['shoulder_lift_joint_range_rad'])
            np.testing.assert_array_equal(model.actuator(side+'_shoulder_lift').ctrlrange,profile['shoulder_lift_ctrlrange_rad'])
        self.assertEqual(source.read_bytes(),before)
        np.testing.assert_array_equal(mujoco.MjSpec.from_file(str(source)).compile().jnt_range,stock.jnt_range)
        fixture=json.loads(Path(__file__).with_name('fixtures').joinpath('post_contact_close.json').read_text())
        self.assertEqual(portable_model_sha256(task_env().model),fixture['portable_model_sha256'])

    def test_unverified_xml_changes_rejected(self):
        source=resolve_so101_model()
        with tempfile.TemporaryDirectory() as directory:
            changed=Path(directory)/'arm.xml'
            for text in (source.read_text()+'\n<!-- extra geometry revision -->',
                         source.read_text().replace('1.7453292519943366','1.7'),
                         source.read_text().replace('1.74533"','1.7"')):
                changed.write_text(text)
                with self.assertRaisesRegex(ValueError,'unverified'):
                    preserve_desk_source_profile(mujoco.MjSpec(),changed)

    def test_portable_identity_preserves_physics_and_rejects_mutations(self):
        # Native MJB retains asset paths; identical mesh bytes at a relocated path
        # must agree, while any actual compiled physics change remains visible.
        source=resolve_so101_model();original=build_scene('desk').compile()
        expected=portable_model_sha256(original)
        with tempfile.TemporaryDirectory() as directory:
            relocated=Path(directory)/source.name;relocated.write_bytes(source.read_bytes())
            (Path(directory)/'assets').symlink_to(source.parent/'assets',target_is_directory=True)
            with patch.dict(os.environ,DAPIER_SO101_MJCF=str(relocated)):
                self.assertEqual(portable_model_sha256(build_scene('desk').compile()),expected)
        for field,index in (('jnt_range',(1,0)),('mesh_vert',(0,0)),('actuator_gainprm',(0,0)),
                            ('geom_friction',(0,0)),('body_pos',(1,0)),('geom_size',(0,0))):
            model=copy.copy(original);getattr(model,field)[index]+=.0001
            self.assertNotEqual(portable_model_sha256(model),expected,field)
        model=copy.copy(original);model.opt.timestep+=.0001
        self.assertNotEqual(portable_model_sha256(model),expected)


if __name__=='__main__':unittest.main()
