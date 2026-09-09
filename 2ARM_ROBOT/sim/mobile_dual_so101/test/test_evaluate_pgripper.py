"""Calibrate task ordering and the exact ACT observation boundary, without hardware."""
from pathlib import Path
import sys
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from evaluate_pgripper import TaskProgress, act_observation
from pgripper_learning import RGB_KEYS


class EvaluationTest(unittest.TestCase):
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
