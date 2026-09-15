"""Hardware-free checks for PGripper labels, RGB, split and source preservation."""
import io
import json
from pathlib import Path
import tempfile
import unittest

import h5py
import numpy as np
from PIL import Image

from convert_pgripper_act import convert, interval_actions, read_sapien, sha256, task_metadata


class ConversionTest(unittest.TestCase):
    def test_task_success_is_not_promoted(self):
        for attrs in (dict(success=True), dict(success=True, task_kind='vision_pick_only'),
                dict(success=True, task_kind='pgripper_handover', canonical_full_task_success=False)):
            result = task_metadata(attrs)
            self.assertTrue(result['source_task_success'])
            self.assertFalse(result['task_success'])
        for attrs in (dict(success=True, task_kind='vision_pick_only', canonical_full_task_success=True),
                dict(success=True, canonical_full_task_success='true')):
            with self.assertRaises(ValueError):
                task_metadata(attrs)

    def test_interval_end_and_invalid_clock(self):
        dense = np.tile(np.arange(44, dtype=float)[:, None] / 100, (1, 12))
        actions = interval_actions(dense, np.array([0, 20, 40]))
        np.testing.assert_allclose(actions[:, 0], [.19, .39])
        np.testing.assert_allclose(actions[:, 5], np.array([.19, .39]) / 2.2028)
        for bad in (np.array([1, 21]), np.array([0, 19]), np.array([0., 20.]), np.arange(4) * 20):
            with self.assertRaises(ValueError):
                interval_actions(dense, bad)

    def test_roundtrip_and_integrity(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / 'source'
            root.mkdir()
            entries = []
            rgb = io.BytesIO()
            Image.new('RGB', (320, 240), (230, 20, 10)).save(rgb, format='JPEG')
            encoded = np.frombuffer(rgb.getvalue(), dtype=np.uint8)
            for i in range(4):
                folder = root / f'episode_{i:03d}'
                folder.mkdir()
                path = folder / 'episode.hdf5'
                dense = np.tile(np.arange(44, dtype=float)[:, None] / 100, (1, 12))
                first = dense[[0, 20, 40]].copy()
                first[:, [5, 11]] /= 2.2028
                with h5py.File(path, 'w') as h:
                    h.attrs.update(schema='dapier.pgripper.sapien.v1', control_hz=500, observation_hz=25,
                        ordering='left5, left_gripper, right5, right_gripper', grippers='both NORMA PGripper',
                        hardware_execution=False, success=True, task_kind='pgripper_handover',
                        canonical_full_task_success=True, source_episode_id=str(i))
                    h['observation/state'] = np.full((3, 12), .1, np.float32)
                    h['action'] = first
                    h['physics_step'] = [0, 20, 40]
                    h['timestamp_s'] = [0., .04, .08]
                    h['expert/action_rad'] = dense
                    h['expert/contact_force'] = [np.nan]
                    for side in ('left', 'right'):
                        ds = h.create_dataset(f'observation/images/{side}_wrist_rgb', (3,), dtype=h5py.vlen_dtype(np.uint8))
                        for t in range(3):
                            ds[t] = encoded
                (folder / 'validation.json').write_text(json.dumps(dict(passed=True, hdf5_sha256=sha256(path))))
                entries.append(dict(folder=folder.name, frames=3, accepted=True))
            (root / 'index.json').write_text(json.dumps(dict(episodes=entries)))
            split_path = Path(tmp) / 'split.json'
            assignments = dict(schema='dapier.pgripper.source_split.v1', episodes=[
                dict(source_episode_id=e['folder'], sha256=sha256(root / e['folder'] / 'episode.hdf5'),
                    split='holdout' if i == 1 else 'train') for i, e in enumerate(entries)])
            split_path.write_text(json.dumps(assignments))
            def convert_fixed(destination, **kwargs):
                return convert(root, destination, split_manifest=split_path,
                    action_source='interval_end_actual', **kwargs)
            before = {p: sha256(p) for p in root.rglob('*') if p.is_file()}
            output = Path(tmp) / 'converted'
            manifest = convert_fixed(output)
            self.assertEqual(manifest['train'], [0, 2, 3])
            self.assertEqual(manifest['holdout'], [1])
            self.assertEqual(manifest['source_split_manifest'], assignments)
            self.assertTrue(manifest['episodes'][0]['canonical_full_task_success'])
            with np.load(output / 'episode-000/episode.npz') as ep:
                self.assertEqual(set(ep.files), {'state', 'action', 'time', 'action_interval_first', 'action_interval_end'})
                self.assertEqual(ep['action'].shape, (2, 12))
                np.testing.assert_allclose(ep['action'][:, 0], [.19, .39])
                self.assertFalse(np.array_equal(ep['action'], ep['state']))
            self.assertEqual((output / 'episode-000/left-00000.jpg').read_bytes(), rgb.getvalue())
            self.assertEqual(before, {p: sha256(p) for p in before})
            with self.assertRaises(FileExistsError):
                convert_fixed(output)
            with self.assertRaises(ValueError):
                convert_fixed(root / 'new')
            for kwargs in ({}, dict(mujoco_dataset=root)):
                with self.assertRaises(ValueError):
                    convert(root, Path(tmp) / 'rejected', **kwargs)
            self.assertFalse((Path(tmp) / 'rejected').exists())
            source = root / 'episode_000/episode.hdf5'
            first, _ = read_sapien(source, 'interval_first_actual')
            np.testing.assert_allclose(first['action'][:, 0], [0., .20])
            with self.assertRaisesRegex(ValueError, 'requested targets are unavailable'):
                read_sapien(source, 'requested_target')
            # Filtering/reordering changes output indices, never source split membership.
            entries[2]['accepted'] = False
            (root / 'index.json').write_text(json.dumps(dict(episodes=entries[::-1])))
            reduced = convert_fixed(Path(tmp) / 'reduced')
            self.assertEqual([reduced['episodes'][i]['source_episode_id'] for i in reduced['holdout']],
                ['episode_001'])
            assignments['episodes'][0]['sha256'] = '0' * 64
            split_path.write_text(json.dumps(assignments))
            with self.assertRaisesRegex(ValueError, 'ID/hash-bound split'):
                convert_fixed(Path(tmp) / 'stale')
            self.assertFalse((Path(tmp) / 'stale').exists())
            validation = root / 'episode_000/validation.json'
            with h5py.File(source, 'r+') as h:
                h.attrs['task_kind'] = 'vision_pick_only'
                del h.attrs['canonical_full_task_success']
                h.attrs['requested_target_units'] = 'arm rad; gripper 0..1'
                h['action_requested_target'] = np.full((3, 12), .7)
            validation.write_text(json.dumps(dict(passed=True, hdf5_sha256=sha256(source))))
            assignments['episodes'][0]['sha256'] = sha256(source)
            split_path.write_text(json.dumps(assignments))
            requested, _ = read_sapien(source, 'requested_target')
            np.testing.assert_allclose(requested['action'], .7)
            self.assertFalse(np.array_equal(requested['action'], requested['action_interval_end']))
            inspected = convert_fixed(Path(tmp) / 'inspection')
            pick = next(e for e in inspected['episodes'] if e['source_episode_id'] == 'episode_000')
            self.assertIsNone(pick['canonical_full_task_success'])
            self.assertTrue(pick['source_task_success'])
            self.assertFalse(pick['task_success'])
            self.assertNotIn(pick['episode'], inspected['train'] + inspected['holdout'])
            self.assertEqual(pick['source_split'], 'train')
            assignments['episodes'].append(assignments['episodes'][0])
            split_path.write_text(json.dumps(assignments))
            with self.assertRaisesRegex(ValueError, 'duplicate source split'):
                convert_fixed(Path(tmp) / 'duplicate')
            validation.write_text(json.dumps(dict(passed=True, hdf5_sha256='bad')))
            with self.assertRaises(ValueError):
                read_sapien(validation.with_name('episode.hdf5'))


if __name__ == '__main__':
    unittest.main()
