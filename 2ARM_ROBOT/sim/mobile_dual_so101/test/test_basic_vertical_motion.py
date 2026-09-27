"""Noncontact SIM rehearsal preserves explicit hold channels and raw start state."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import json
import unittest
from unittest.mock import patch
import numpy as np
from basic_vertical_motion import plan
from integration_scenes import task_env
from collision_guard import check_bimanual_path
from mobile_dual_so101 import actuator_targets_from_qpos

class BasicVerticalMotionTest(unittest.TestCase):
    def test_plan_preserves_state_and_unoperated_commands(self):
        env = task_env('desk')
        env.reset(seed=0)
        env.data.qpos[5] -= 1e-5
        import mujoco
        mujoco.mj_forward(env.model, env.data)
        raw, ctrl = env.data.qpos.copy(), env.data.ctrl.copy()
        measured = actuator_targets_from_qpos(env.model, raw)
        with patch('basic_vertical_motion.check_bimanual_path', wraps=check_bimanual_path) as guard:
            _, start, target, _, trajectory, ik, result = plan(env)
        self.assertTrue(ik['converged'] and result['safe'])
        np.testing.assert_allclose(target-start, [0., 0., .02], atol=1e-15)
        np.testing.assert_array_equal(env.data.qpos, raw)
        np.testing.assert_array_equal(env.data.ctrl, ctrl)
        np.testing.assert_array_equal(trajectory.goal_rad[5:], ctrl[5:])
        np.testing.assert_array_equal(trajectory.start_rad, ctrl)
        np.testing.assert_array_equal(guard.call_args.args[1], measured)
        self.assertIs(guard.call_args.kwargs['reference_data'], env.data)
        self.assertEqual(guard.call_args.kwargs['task_phase'], 'SAFE_STAGE')
        self.assertEqual(guard.call_args.kwargs['required_clearance_m'], .03)
        np.testing.assert_allclose(trajectory.sample(trajectory.duration_s)[0], trajectory.goal_rad, atol=1e-15)

    def test_recorded_physics_is_noncontact_sim_only(self):
        r = json.loads((Path(__file__).parent/'fixtures/basic_vertical_motion.json').read_text())
        self.assertTrue(r['passed'])
        self.assertFalse(r['hardware_execution'])
        self.assertFalse(r['real_start_pose_verified'])
        self.assertEqual(r['solver']['noslip_iterations'], 0)
        np.testing.assert_array_equal(r['start_q'], r['return_q'])
        np.testing.assert_array_equal(r['target_q'][5:], r['start_q'][5:])
        self.assertEqual([p['phase'] for p in r['phases']], ['UP_20MM','NONCONTACT_HOLD','RETURN'])
        for p in r['phases']:
            self.assertLessEqual(p['position_error_m'], .0005)
            self.assertLessEqual(p['approach_error_deg'], 2.)
        self.assertGreaterEqual(r['observed_min_general_clearance_m'], .03)
        self.assertLessEqual(r['observed_max_tracking_error_rad'], .02)
        self.assertAlmostEqual(r['phases'][1]['sim_time_s']-r['phases'][0]['sim_time_s'], .5)

if __name__ == '__main__': unittest.main()
