"""Pixel-reference and visualization boundaries, with no physical positive claims."""
import copy
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import numpy as np

from .wrist_overlay import frame_binding, measure_wrist, render_wrist


def fixture(side='left'):
    im=np.full((240,320,3),255,np.uint8);im[80:160,100:210]=[0,0,255]
    f=dict(image_bgr=im,artifact_role='RAW_RGB',source_kind='TEST_FIXTURE',camera_key=side+'_wrist',image_identity='FRAME_1',source_sha256='FIXTURE')
    ref=dict(readable=True,inner_px=[20,280],midline_px=150.0,gap_px=260,gripper_tick=100,
        camera_key='left_wrist',source_kind='TEST_FIXTURE',source_sha256='REFERENCE_FIXTURE',reference_shape_hw=[240,320])
    context=dict(binding=frame_binding(f),scene=dict(wrist=dict(object=dict(stable=True))),plan=dict(tilt_deg=0),gripper_tick=100)
    return f,ref,context


class WristOverlayTests(unittest.TestCase):
    def test_observed_X_is_not_image_centre_or_a_2D_grasp_point(self):
        f,ref,c=fixture();r=measure_wrist(f,side='left',jaw_reference=ref,context=c)
        self.assertEqual(r['gripper']['observed_midline_u'],150)
        self.assertEqual(r['image_error_px'],dict(u=4.5,v=None))
        self.assertIsNone(r['gripper']['observed_grasp_centre_uv'])
        self.assertEqual(r['relative_angle']['status'],'UNKNOWN')
    def test_no_reference_never_defaults_to_screen_centre(self):
        f,_,_=fixture();r=measure_wrist(f,side='left')
        self.assertIsNone(r['gripper']['observed_midline_u']);self.assertIsNone(r['image_error_px']['u'])
    def test_current_context_and_model_cannot_use_another_raw_frame(self):
        f,ref,c=fixture();c['binding']['image_identity']='OTHER'
        self.assertIsNone(measure_wrist(f,side='left',jaw_reference=ref,context=c)['image_error_px']['u'])
        with self.assertRaisesRegex(ValueError,'BOUND_MODEL'):
            measure_wrist(f,side='left',model_projection=dict(binding=c['binding'],role='MODEL_PRIOR',centre_uv=[1,2]))
    def test_model_cross_and_error_do_not_become_observed(self):
        f,_,_=fixture('right');model=dict(binding=frame_binding(f),role='MODEL_PRIOR',centre_uv=[80,100],closing_line_uv=[[40,100],[120,100]])
        r=measure_wrist(f,side='right',model_projection=model)
        self.assertEqual(r['gripper']['model_grasp_centre_uv'],[80,100])
        self.assertIsNone(r['image_error_px']['u']);self.assertIsNotNone(r['model_relative_image_error_px']['u'])
        self.assertFalse(r['graph_requested']);self.assertIsNone(r['Correction'])
    def test_explicit_unoccluded_corresponding_edge_yields_image_angle_only(self):
        f,ref,c=fixture();e=dict(binding=frame_binding(f),unoccluded=True,correspondence_verified=True,points_uv=[[110,80],[200,80]])
        r=measure_wrist(f,side='left',jaw_reference=ref,context=c,edge_evidence=e)
        self.assertEqual(r['relative_angle']['status'],'AVAILABLE_IMAGE_ONLY')
        self.assertAlmostEqual(r['relative_angle']['value_deg'],-4.7)
        self.assertIsNone(r['relative_angle']['physical_3D_yaw'])
        for key in ('unoccluded','correspondence_verified'):
            n=copy.deepcopy(e);n[key]=False
            self.assertEqual(measure_wrist(f,side='left',jaw_reference=ref,context=c,edge_evidence=n)['relative_angle']['status'],'UNKNOWN')
    def test_invented_edge_and_overlay_input_are_rejected(self):
        f,ref,c=fixture();e=dict(binding=frame_binding(f),unoccluded=True,correspondence_verified=True,points_uv=[[1,1],[2,1]])
        self.assertEqual(measure_wrist(f,side='left',jaw_reference=ref,context=c,edge_evidence=e)['relative_angle']['status'],'UNKNOWN')
        overlay=render_wrist(f,measure_wrist(f,side='left'))
        with self.assertRaisesRegex(ValueError,'OVERLAY_IS_NOT_DETECTOR_INPUT'):measure_wrist(overlay,side='left')
    def test_configured_image_target_is_not_an_observed_gripper_direction(self):
        f,ref,c=fixture();e=dict(binding=frame_binding(f),unoccluded=True,correspondence_verified=True,points_uv=[[110,80],[200,80]])
        with patch('visual_servo.wrist_overlay._alignment',return_value=dict(ok=True,square_at_deg=12.25)):
            angle=measure_wrist(f,side='left',jaw_reference=ref,context=c,edge_evidence=e)['relative_angle']
        self.assertEqual(angle['role'],'OBSERVED_EDGE_VS_CONFIGURED_IMAGE_REFERENCE')
        self.assertAlmostEqual(angle['value_deg'],-12.25)
        self.assertIn('configured image target',angle['reference_source'])
        self.assertIsNone(angle['observed_gripper_direction_deg']);self.assertIsNone(angle['physical_3D_yaw'])
        f,_,_=fixture('right');e['binding']=frame_binding(f)
        model=dict(binding=frame_binding(f),role='MODEL_PRIOR',centre_uv=[80,100],closing_line_uv=[[40,100],[40,180]])
        angle=measure_wrist(f,side='right',edge_evidence=e,model_projection=model)['relative_angle']
        self.assertEqual(angle['role'],'OBSERVED_EDGE_VS_MODEL_PRIOR_DIRECTION')
        self.assertEqual(angle['reference_angle_deg'],90)
        self.assertIn('MODEL_PRIOR',angle['reference_source'])
        self.assertIsNone(angle['observed_gripper_direction_deg']);self.assertIsNone(angle['physical_3D_yaw'])
    def test_model_inside_does_not_clear_observed_clip_or_grant_a_FOV_pass(self):
        f,ref,c=fixture();f['image_bgr'][80:160,:210]=[0,0,255];c['binding']=frame_binding(f)
        n=dict(binding=frame_binding(f),role='MODEL_PRIOR',K=[[100,0,160],[0,100,120],[0,0,1]],D=[0]*5,
            T_world_camera=np.eye(4).tolist(),T_world_camera_next=np.eye(4).tolist(),plane_centre=[0,0,1],plane_normal=[0,0,1])
        r=measure_wrist(f,side='left',jaw_reference=ref,context=c,next_descent=n)
        self.assertEqual(r['fov']['next_descent']['status'],'MODEL_PRIOR_ENVELOPE_INSIDE')
        self.assertTrue(r['object']['widest_row_horizontal_clipped']);self.assertIsNone(r['image_error_px']['u'])
        self.assertFalse(r['fov']['next_descent']['valid_for_control']);self.assertFalse(r['executable'])
        self.assertNotIn('FOV_OK',str(r));self.assertIsNone(r['Correction'])
    def test_horizontal_clipping_invalidates_X_error_and_angle(self):
        f,ref,c=fixture();f['image_bgr'][80:160,:210]=[0,0,255];c['binding']=frame_binding(f)
        r=measure_wrist(f,side='left',jaw_reference=ref,context=c,next_descent=dict(binding=frame_binding(f),role='MODEL_PRIOR'))
        self.assertTrue(r['object']['frame_clipped']);self.assertIsNone(r['image_error_px']['u'])
        self.assertEqual(r['relative_angle']['status'],'UNKNOWN');self.assertEqual(r['fov']['next_descent']['status'],'UNKNOWN')
    def test_bottom_clipping_preserves_legacy_X_but_not_an_angle(self):
        f,ref,c=fixture();f['image_bgr'][80:,100:210]=[0,0,255];c['binding']=frame_binding(f)
        r=measure_wrist(f,side='left',jaw_reference=ref,context=c)
        self.assertTrue(r['object']['frame_clipped']);self.assertEqual(r['image_error_px']['u'],4.5)
        self.assertEqual(r['relative_angle']['status'],'UNKNOWN')
    def test_nominal_descent_projection_is_model_prior_and_can_exit_FOV(self):
        f,_,_=fixture('right');now=np.eye(4);future=np.eye(4);future[0,3]=3
        n=dict(binding=frame_binding(f),role='MODEL_PRIOR',K=[[100,0,160],[0,100,120],[0,0,1]],D=[0]*5,
            T_world_camera=now.tolist(),T_world_camera_next=future.tolist(),plane_centre=[0,0,1],plane_normal=[0,0,1],input_role='RECORDED_NOMINAL_TARGET_NOT_ACTUAL')
        r=measure_wrist(f,side='right',next_descent=n)
        self.assertEqual(r['fov']['next_descent']['status'],'MODEL_PRIOR_ENVELOPE_EXIT')
        self.assertFalse(r['fov']['next_descent']['physical_validated']);self.assertIsNone(r['Correction'])
        n['plane_centre']=[0,0,-1]
        self.assertEqual(measure_wrist(f,side='right',next_descent=n)['fov']['next_descent']['status'],'UNKNOWN')
    def test_render_preserves_source_pixels_and_binds_own_raw_frame(self):
        f,_,_=fixture();before=f['image_bgr'].copy();r=measure_wrist(f,side='left');v=render_wrist(f,r)
        np.testing.assert_array_equal(f['image_bgr'],before);self.assertFalse(np.shares_memory(v['image_bgr'],before))
        self.assertGreater(v['image_bgr'].shape[0],before.shape[0]);self.assertEqual(v['artifact_role'],'DEBUG_OVERLAY_ONLY')
        f['image_bgr'][0,0]=0
        with self.assertRaisesRegex(ValueError,'BINDING_MISMATCH'):render_wrist(f,r)
    def test_camera_side_cannot_be_relabelled_by_the_overlay_caller(self):
        f,_,_=fixture()
        with self.assertRaisesRegex(ValueError,'CAMERA_SIDE_MISMATCH'):measure_wrist(f,side='right')
    def test_contour_chord_and_invalid_A_alignment_profile_remain_unknown(self):
        f,ref,c=fixture();e=dict(binding=frame_binding(f),unoccluded=True,correspondence_verified=True,points_uv=[[110,80],[200,159]])
        r=measure_wrist(f,side='left',jaw_reference=ref,context=c,edge_evidence=e)
        self.assertEqual(r['relative_angle']['status'],'UNKNOWN')
        e['points_uv']=[[110,80],[200,80]];c['plan']['tilt_deg']=30
        r=measure_wrist(f,side='left',jaw_reference=ref,context=c,edge_evidence=e)
        self.assertFalse(r['legacy_alignment']['ok']);self.assertEqual(r['relative_angle']['status'],'UNKNOWN')
    def test_debug_output_cannot_modify_an_original_episode_directory(self):
        from .run_wrist_overlay import debug_output_path
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);original=root/'original';original.mkdir();bad=original/'overlay'
            with self.assertRaisesRegex(ValueError,'OUTSIDE_ORIGINAL_RUNS'):debug_output_path(bad,[original])
            self.assertFalse(bad.exists());self.assertEqual(debug_output_path(root/'debug',[original]),root/'debug')


if __name__=='__main__':unittest.main()
