"""SIM-only recording boundary tests, including failure inside a policy frame."""
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import pgripper_learning as learning
import evaluate_pgripper as evaluation
from mobile_dual_so101 import apply_control_as_pose, actuator_targets_from_qpos, resolve_so101_model
from pgripper import home_action
from pgripper_execution import configure_physics
from replay_recorded_episode import build_tabletop_spec


class Renderer:
    def __init__(self, *args, **kwargs): pass
    def update_scene(self, *args, **kwargs): pass
    def render(self): return np.zeros((240, 320, 3), np.uint8)
    def close(self): pass


def scene():
    profile = json.loads(Path(learning.__file__).with_name('tabletop_replay.json').read_text())
    model = build_tabletop_spec(resolve_so101_model(), profile, grippers='both').compile()
    configure_physics(model, 2)
    data = mujoco.MjData(model)
    command = np.asarray(home_action(model, np.tile(np.deg2rad([0, -35, 55, 35, 0, 0]), 2)))
    apply_control_as_pose(model, data, command)
    return model, data


class RecordingTest(unittest.TestCase):
    def test_before_command_state_requested_sent_and_failed_archive(self):
        model, data = scene()
        measured = learning.contract_vector(model, actuator_targets_from_qpos(model, data.qpos))
        previous = data.ctrl.copy()
        requested = previous.copy(); requested[0] += .1
        sent = previous.copy(); sent[0] += .0012
        def failed_run(args, **kwargs):
            kwargs['command_observer'](model, data, requested, sent, 'test')
            np.testing.assert_array_equal(data.ctrl, previous)
            raise ValueError('injected collection failure')
        with tempfile.TemporaryDirectory() as tmp, patch.object(learning, 'run', failed_run), patch.object(mujoco, 'Renderer', Renderer):
            result = learning.collect_episode((tmp, 0, 4100, {'policy_target_hz': 25, 'physics_substeps': 2}))
            self.assertFalse(result['task_success'])
            with np.load(Path(tmp) / 'episode-000/episode.npz') as saved:
                np.testing.assert_array_equal(saved['state'][0], measured)
                np.testing.assert_array_equal(saved['action'][0], learning.contract_vector(model, requested))
                np.testing.assert_array_equal(saved['action_sent'][0], learning.contract_vector(model, sent))
                np.testing.assert_array_equal(saved['control_ranges'], model.actuator_ctrlrange)
                self.assertEqual(float(saved['command_dt_s']), .002)
                np.testing.assert_array_equal(saved['dense_ctrl'][0], requested)
                np.testing.assert_array_equal(saved['dense_sent_ctrl'][0], sent)
            self.assertIn('injected', result['failure'])

    def test_split_plan_preserved_when_collection_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            plan = {'train': [0, 1, 2, 3], 'holdout': [4, 5]}
            (root / 'split-plan.json').write_text(json.dumps(plan))
            rows = [{'episode': i, 'task_success': i != 1, 'canonical_full_task_success': i != 1} for i in range(6)]
            learning.finalize(SimpleNamespace(output=root), rows)
            manifest = json.loads((root / 'manifest.json').read_text())
            self.assertEqual(manifest['train'], [0, 2, 3])
            self.assertEqual(manifest['holdout'], [4, 5])
            self.assertEqual(json.loads((root / 'split-plan.json').read_text()), plan)

    def test_failure_inside_frame_preserves_current_evidence(self):
        model, data = scene()
        initial = np.empty(mujoco.mj_stateSize(model, mujoco.mjtState.mjSTATE_INTEGRATION))
        mujoco.mj_getState(model, data, initial, mujoco.mjtState.mjSTATE_INTEGRATION)
        steps = []
        original = evaluation.step_physics
        def fail_step(*args, **kwargs):
            if len(steps) == 2:
                raise ValueError('injected mid-frame failure')
            steps.append(1)
            original(*args, **kwargs)
        with tempfile.TemporaryDirectory() as tmp, patch.object(mujoco, 'Renderer', Renderer), patch.object(evaluation, 'step_physics', fail_step):
            root = Path(tmp)
            archive = root / 'source.npz'
            np.savez(archive, physics=initial[None], dense_ctrl=np.tile(data.ctrl, (20,1)),
                action=np.zeros((1,12)), state=np.zeros((1,12)))
            args = SimpleNamespace(mode='reference', episodes=1, seconds=1, reference_clock='first',
                checkpoint=None, dataset=None, output=root/'result', initial_episode=archive,
                physics_substeps=2, range_policy='project', seed=0, action_steps=1)
            evaluation.evaluate(args)
            trace = json.loads((args.output/'trial-00/trace.json').read_text())
            self.assertEqual(len(trace), 1)
            self.assertEqual(trace[0]['command_in_frame'], 2)
            self.assertAlmostEqual(trace[0]['time_s'], .004)
            self.assertEqual(trace[0]['failure'], 'injected mid-frame failure')
            for key in ('raw_action', 'projected_target', 'sent_ctrl', 'current_measured_state', 'forces_N'):
                self.assertIn(key, trace[0])


if __name__ == '__main__':
    unittest.main()
