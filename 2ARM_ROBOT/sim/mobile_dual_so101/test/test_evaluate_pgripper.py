"""Calibrate task ordering and the exact ACT observation boundary, without hardware."""
from pathlib import Path
import sys
import unittest
import numpy as np
import mujoco
from collections import deque
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from evaluate_pgripper import TaskProgress, LeftPickHoldProgress, act_observation, reference_ctrl, restore_frame_zero
from pgripper_learning import RGB_KEYS


class EvaluationTest(unittest.TestCase):
    def test_left_pick_requires_continuous_stable_unsupported_hold(self):
        def feed(p, seconds, forces=((2, 2), (0, 0)), height=.07, speed=0., support=0.):
            for i in range(round(seconds / .01)):
                p.update(.01, i * .01, forces, [.2, 0., height], speed, support)
        p = LeftPickHoldProgress()
        feed(p, 4., height=.02, support=.2)
        self.assertNotIn('left_held', p.events)
        feed(p, 2.)
        feed(p, .1, speed=.02)
        feed(p, 2.)
        self.assertNotIn('left_held', p.events)
        feed(p, 1.1)
        self.assertIn('left_held', p.events)
        self.assertNotIn('placed', p.events)
        for kwargs in ({'support': .2}, {'speed': .02}, {'forces': ((2, 2), (1, 1))}):
            bad = LeftPickHoldProgress()
            feed(bad, 4., **kwargs)
            self.assertNotIn('left_held', bad.events)
        dropped = LeftPickHoldProgress()
        feed(dropped, .5)
        feed(dropped, .01, forces=((0, 0), (0, 0)))
        feed(dropped, 4.)
        self.assertIsNotNone(dropped.failure)
        self.assertNotIn('left_held', dropped.events)

    def test_same_initial_integration_state_and_policy_queue_reset(self):
        model = mujoco.MjModel.from_xml_string('<mujoco><worldbody><body><joint type="slide"/><geom size=".1"/></body></worldbody></mujoco>')
        model.opt.timestep = .001
        left, right = mujoco.MjData(model), mujoco.MjData(model)
        state = np.empty(mujoco.mj_stateSize(model, mujoco.mjtState.mjSTATE_INTEGRATION))
        mujoco.mj_getState(model, left, state, mujoco.mjtState.mjSTATE_INTEGRATION)
        queue = deque([np.ones(12)])
        restore_frame_zero(model, left, state, SimpleNamespace(reset=queue.clear))
        restore_frame_zero(model, right, state)
        actual = []
        for data in (left, right):
            saved = np.empty_like(state)
            mujoco.mj_getState(model, data, saved, mujoco.mjtState.mjSTATE_INTEGRATION)
            actual.append(saved)
        np.testing.assert_array_equal(*actual)
        self.assertFalse(queue)
        state[0] = 1
        with self.assertRaises(ValueError):
            restore_frame_zero(model, left, state)

    def test_complete_sequence_off_goal_and_lost_support_are_not_placed(self):
        for position, support in (([.224, -.08, .02], .2), ([.22, -.08, .02], 0.)):
            progress = TaskProgress()
            stages = [([[2,2],[0,0]], [.2,0,.02], .2, .4),
                ([[2,2],[0,0]], [.27,0,.17], 0., .1),
                ([[2,2],[2,2]], [.27,0,.17], 0., .4),
                ([[0,0],[2,2]], [.27,0,.17], 0., 3.1),
                ([[0,0],[2,2]], [.22,-.08,.02], .2, .3),
                ([[0,0],[0,0]], position, support, 3.1)]
            timestamp = 0.
            for forces, pos, force, duration in stages:
                for _ in range(round(duration/.01)):
                    timestamp += .01
                    progress.update(.01, timestamp, forces, pos, 0., force)
            self.assertIn('table_support', progress.events)
            self.assertNotIn('placed', progress.events)

    def test_reference_clocks_hold_targets_and_trim_partial_interval(self):
        dense = np.tile(np.arange(44, dtype=float)[:, None], (1, 12))
        for substep in range(20):
            self.assertEqual(reference_ctrl(dense, 0, substep, 20, 'dense')[0], substep)
            self.assertEqual(reference_ctrl(dense, 0, substep, 20, 'first')[0], 0)
            self.assertEqual(reference_ctrl(dense, 0, substep, 20, 'end')[0], 19)
            self.assertEqual(reference_ctrl(dense, 5, substep, 20, 'first')[0], 20)
            self.assertEqual(reference_ctrl(dense, 5, substep, 20, 'end')[0], 39)
        self.assertEqual(reference_ctrl(dense, 5, 0, 20, 'dense')[0], 43)
        with self.assertRaises(ValueError):
            reference_ctrl(dense[:10], 0, 0, 20, 'end')

    def test_ordered_success_and_false_success_rejection(self):
        progress = TaskProgress()
        timestamp = 0.
        def feed(forces, position, support, seconds):
            nonlocal timestamp
            for _ in range(round(seconds / .01)):
                timestamp += .01
                progress.update(.01, timestamp, np.array(forces), position, 0., support)
        feed([[0, 0], [0, 0]], [.22, -.08, .02], .2, 4)
        self.assertNotIn('placed', progress.events, 'a cube initially on the table is not task success')
        feed([[2, 2], [0, 0]], [.2, 0, .02], .2, .4)
        feed([[2, 2], [0, 0]], [.27, 0, .17], 0., .1)
        feed([[2, 2], [2, 2]], [.27, 0, .17], 0., .4)
        feed([[0, 0], [2, 2]], [.27, 0, .17], 0., 3.1)
        feed([[0, 0], [2, 2]], [.22, -.08, .02], .2, .3)
        feed([[0, 0], [0, 0]], [.22, -.08, .02], 0., .02)
        self.assertIsNone(progress.failure, 'confirmed table ownership permits release and settling')
        feed([[0, 0], [0, 0]], [.22, -.08, .02], .2, 3.1)
        self.assertEqual(list(progress.events), ['left_grasp', 'left_lift', 'right_grasp', 'handover', 'table_support', 'placed'])
        self.assertIsNone(progress.failure)
        wrong = TaskProgress()
        for i in range(400):
            wrong.update(.01, i * .01, np.array([[0, 0], [2, 2]]), [.27, 0, .17], 0., 0.)
        self.assertNotIn('handover', wrong.events, 'right-only grasp cannot fabricate left pickup')
        with self.assertRaises(ValueError):
            wrong.update(.01, 4., np.full((2, 2), np.nan), [.2, 0, .02], 0., .2)

    def test_observation_has_no_labels_or_object_truth(self):
        stats = {'state_mean': np.ones(12, np.float32), 'state_std': np.full(12, 2., np.float32)}
        images = {key: np.zeros((240, 320, 3), np.uint8) for key in RGB_KEYS}
        obs = act_observation(np.ones(12, np.float32), images, stats, 'cpu')
        self.assertEqual(set(obs), {'observation.state', *RGB_KEYS})
        self.assertEqual(tuple(obs['observation.state'].shape), (1, 12))
        self.assertEqual(float(obs['observation.state'].abs().sum()), 0.)
        with self.assertRaises(ValueError):
            act_observation(np.ones(12), dict(images, object_truth=np.zeros(3)), stats, 'cpu')
        with self.assertRaises(ValueError):
            act_observation(np.full(12, np.nan), images, stats, 'cpu')


if __name__ == '__main__':
    unittest.main()
