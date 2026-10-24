"""Hardware-free negative controls for synthetic ACT data and grasp assistance."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest

import mujoco
import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mobile_dual_so101 import ACTION_NAMES
from pgripper_dataset import UNITS, file_sha256, validate_arrays, validate_dataset
from pgripper_execution import configure_physics, step_physics


def arrays():
    ranges = np.tile([-3., 3.], (12, 1))
    ranges[[5, 11]] = [0., 2.2028]
    dense = np.full((40, 12), .5)
    sent = dense / 2
    action, action_sent = dense[::20].copy(), sent[::20].copy()
    action[:, [5, 11]] /= 2.2028
    action_sent[:, [5, 11]] /= 2.2028
    return dict(state=np.full((2, 12), .1, np.float32), action=action,
        action_sent=action_sent, time=np.array([0., .04]), dense_ctrl=dense,
        dense_sent_ctrl=sent, control_ranges=ranges, command_dt_s=np.array(.002))


class SyntheticContractTest(unittest.TestCase):
    def test_command_provenance_not_next_state_and_both_arms(self):
        valid = arrays()
        self.assertEqual(validate_arrays(valid, action_source='requested_target'), 2)
        for key, mutation in (
                ('state', lambda a: a.__setitem__((0, 6), np.nan)),
                ('action', lambda a: a.__setitem__((0, 11), 100)),
                ('action', lambda a: a.__setitem__((0, 1), 90)),
                ('time', lambda a: a.__setitem__(1, .02)),
                ('action_sent', lambda a: a.__setitem__((0, 6), .5)),
                ('dense_sent_ctrl', lambda a: a.__setitem__((5, 6), 100))):
            bad = copy.deepcopy(valid)
            mutation(bad[key])
            with self.subTest(key=key), self.assertRaises(ValueError):
                validate_arrays(bad, action_source='requested_target')
        bad = copy.deepcopy(valid)
        bad['action'] = bad['state'].copy()
        with self.assertRaisesRegex(ValueError, 'dense command boundary'):
            validate_arrays(bad, action_source='requested_target')
        with self.assertRaises(ValueError):
            validate_arrays(valid, action_source='next_state')

    def test_dataset_decode_padding_split_and_corruption(self):
        from pgripper_learning import CHUNK, Demonstrations
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            entries = []
            for episode in range(2):
                folder = root / f'episode-{episode:03d}'
                folder.mkdir()
                values = arrays()
                values['state'] += episode * .1
                np.savez(folder / 'episode.npz', **values)
                for side in ('left', 'right'):
                    for frame in range(2):
                        Image.new('RGB', (320, 240), (episode * 100, 20, 10)).save(folder / f'{side}-{frame:05d}.jpg')
                entries.append(dict(episode=episode, frames=2, task_success=True,
                    canonical_full_task_success=True, episode_sha256=file_sha256(folder / 'episode.npz')))
            manifest = dict(fps=25, rgb_shape=[240, 320, 3], state_action_units=UNITS,
                action_names=list(ACTION_NAMES), action_source='requested_target',
                hardware_execution=False, train=[0], holdout=[1], episodes=entries)
            path = root / 'manifest.json'
            path.write_text(json.dumps(manifest))
            report = validate_dataset(root)
            self.assertTrue(report['passed'])
            self.assertTrue(all(e['dense_command_alignment_checked'] for e in report['episodes']))
            training = Demonstrations(root, [0])
            held = Demonstrations(root, [1], training.stats)
            batch = held[1]
            self.assertIs(held.stats, training.stats)
            self.assertEqual(tuple(batch['action'].shape), (CHUNK, 12))
            self.assertEqual(int(batch['action_is_pad'].sum()), CHUNK - 1)
            for key, value in (('holdout', [0]), ('holdout', []),
                    ('action_names', list(reversed(ACTION_NAMES))), ('hardware_execution', True)):
                bad = {**manifest, key: value}
                path.write_text(json.dumps(bad))
                with self.subTest(key=key), self.assertRaises(ValueError):
                    validate_dataset(root)
            bad = copy.deepcopy(manifest)
            bad['episodes'][1]['canonical_full_task_success'] = None
            path.write_text(json.dumps(bad))
            with self.assertRaisesRegex(ValueError, 'unverified'):
                validate_dataset(root)
            path.write_text(json.dumps(manifest))
            Image.new('RGB', (8, 8)).save(root / 'episode-001/right-00001.jpg')
            with self.assertRaisesRegex(ValueError, '320x240 RGB'):
                validate_dataset(root)
            self.assertTrue(validate_dataset(root, decode_images=False)['passed'])
            np.savez(root / 'episode-001/episode.npz', **arrays())
            with self.assertRaisesRegex(ValueError, 'hash mismatch'):
                validate_dataset(root, decode_images=False)

    def test_external_spring_and_active_attachment_cannot_pass_physics(self):
        model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
          <body name="red_block" pos="0 0 1"><freejoint/><geom size=".02" mass=".02"/></body>
          </worldbody><equality><weld body1="red_block" active="false"/></equality></mujoco>''')
        configure_physics(model, 2)
        data = mujoco.MjData(model)
        step_physics(model, data, 2)
        before = data.time
        for field, index in (('xfrc_applied', (1, 2)), ('qfrc_applied', 2)):
            values = getattr(data, field)
            values[index] = .1
            with self.subTest(field=field), self.assertRaisesRegex(ValueError, 'applied forces'):
                step_physics(model, data, 2)
            self.assertEqual(data.time, before)
            values[index] = 0
        data.eq_active[0] = True
        with self.assertRaisesRegex(ValueError, 'attachments'):
            step_physics(model, data, 2)
        self.assertEqual(data.time, before)


if __name__ == '__main__':
    unittest.main()
