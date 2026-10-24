#!/usr/bin/env python3
"""Read-only PGripper SAPIEN HDF5 -> private two-wrist ACT staging (not PPO)."""
import argparse
import hashlib
import io
import json
from pathlib import Path

import h5py
import numpy as np
from PIL import Image

GRIP_RANGE = 2.2028
ACTION_SOURCES = ('requested_target', 'interval_first_actual', 'interval_end_actual')


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def interval_actions(dense, steps):
    dense, steps = np.asarray(dense), np.asarray(steps)
    if (dense.ndim != 2 or dense.shape[1] != 12 or not np.isfinite(dense).all()
            or steps.ndim != 1 or steps.dtype.kind not in 'iu'
            or not np.array_equal(steps, np.arange(len(steps)) * 20)
            or len(steps) and steps[-1] >= len(dense)):
        raise ValueError('expected finite 500Hz commands and contiguous 25Hz indices')
    complete = steps[steps + 19 < len(dense)]
    if not len(complete):
        raise ValueError('no complete control intervals')
    action = dense[complete + 19].copy()
    action[:, [5, 11]] /= GRIP_RANGE
    if (action[:, [5, 11]] < -1e-6).any() or (action[:, [5, 11]] > 1 + 1e-6).any():
        raise ValueError('PGripper command outside normalized range')
    return action.astype(np.float32)


def read_sapien(path, action_source='interval_end_actual'):
    if action_source not in ACTION_SOURCES:
        raise ValueError('an explicit supported action source is required')
    validation = json.loads(path.with_name('validation.json').read_text())
    digest = sha256(path)
    if validation['passed'] is not True or digest != validation['hdf5_sha256']:
        raise ValueError('source validation/hash mismatch')
    with h5py.File(path, 'r') as source:
        expected = dict(schema='dapier.pgripper.sapien.v1', control_hz=500, observation_hz=25,
            ordering='left5, left_gripper, right5, right_gripper', grippers='both NORMA PGripper',
            hardware_execution=False, success=True)
        if any(source.attrs.get(k) != v for k, v in expected.items()):
            raise ValueError('incompatible PGripper SIM schema')
        state, first = source['observation/state'][:], source['action'][:]
        steps, times = source['physics_step'][:], source['timestamp_s'][:]
        dense = source['expert/action_rad'][:]
        action = interval_actions(dense, steps)
        if (state.shape != (len(steps), 12) or first.shape != state.shape
                or not np.isfinite(state).all() or not np.isfinite(first).all()
                or not np.allclose(times, steps / 500, atol=1e-6, rtol=0)
                or (state[:, [5, 11]] < -1e-6).any() or (state[:, [5, 11]] > 1 + 1e-6).any()):
            raise ValueError('invalid measured state/action/time alignment')
        normalized_first = dense[steps].copy()
        normalized_first[:, [5, 11]] /= GRIP_RANGE
        if not np.allclose(first, normalized_first, atol=1e-6, rtol=0):
            raise ValueError('25Hz source command is not the interval first command')
        arrays = dict(state=state[:len(action)].astype(np.float32),
            action_interval_first=first[:len(action)].astype(np.float32),
            action_interval_end=action, time=times[:len(action)])
        if 'action_requested_target' in source:
            requested = source['action_requested_target'][:]
            if (source.attrs.get('requested_target_units') != 'arm rad; gripper 0..1'
                    or requested.shape != state.shape or not np.isfinite(requested).all()
                    or (requested[:, [5, 11]] < 0).any() or (requested[:, [5, 11]] > 1).any()):
                raise ValueError('invalid explicit requested target contract')
            arrays['action_requested_target'] = requested[:len(action)].astype(np.float32)
        key = dict(requested_target='action_requested_target',
            interval_first_actual='action_interval_first', interval_end_actual='action_interval_end')[action_source]
        if key not in arrays:
            raise ValueError('requested targets are unavailable; actual commands are not requested targets')
        arrays['action'] = arrays[key].copy()
        return arrays, digest


def task_metadata(attrs):
    task_kind = attrs.get('task_kind', 'unknown')
    canonical = attrs.get('canonical_full_task_success')
    source_success = attrs.get('source_task_success', attrs.get('success'))
    if (not isinstance(task_kind, str) or not task_kind
            or not isinstance(source_success, (bool, np.bool_))
            or canonical is not None and not isinstance(canonical, (bool, np.bool_))):
        raise ValueError('invalid task success metadata')
    if canonical and (task_kind != 'pgripper_handover' or not source_success):
        raise ValueError('canonical success requires explicit full handover task success')
    canonical = None if canonical is None else bool(canonical)
    return dict(task_kind=task_kind, source_task_success=bool(source_success),
        canonical_full_task_success=canonical, task_success=canonical is True)


def convert(dataset, output, mujoco_dataset=None, *, split_manifest=None, action_source=None):
    if mujoco_dataset is not None:
        raise ValueError('MuJoCo mixing disabled: legacy requested/actual command semantics need verification')
    if split_manifest is None or action_source not in ACTION_SOURCES:
        raise ValueError('explicit source-ID/hash split manifest and action_source are required')
    dataset, output = dataset.resolve(), output.absolute()
    protected = [dataset] + ([mujoco_dataset.resolve()] if mujoco_dataset else [])
    if any(output.resolve() == p or p in output.resolve().parents for p in protected):
        raise ValueError('output must be outside original datasets')
    accepted = [e for e in json.loads((dataset / 'index.json').read_text())['episodes'] if e['accepted']]
    assignments = json.loads(split_manifest.read_text())
    if assignments.get('schema') != 'dapier.pgripper.source_split.v1':
        raise ValueError('unsupported source split schema')
    splits = {}
    for assignment in assignments['episodes']:
        source_id = assignment['source_episode_id']
        if (not isinstance(source_id, str) or not source_id or source_id in splits
                or assignment['split'] not in ('train', 'holdout')
                or not isinstance(assignment['sha256'], str) or len(assignment['sha256']) != 64):
            raise ValueError('invalid or duplicate source split assignment')
        splits[source_id] = assignment
    sources, seen, seen_paths = [], set(), set()
    for entry in accepted:
        source_id = entry.get('source_episode_id', entry['folder'])
        source = (dataset / entry['folder'] / 'episode.hdf5').resolve()
        if dataset not in source.parents or source_id in seen or source in seen_paths:
            raise ValueError('duplicate source ID or episode path escapes dataset')
        seen.add(source_id)
        seen_paths.add(source)
        if source_id not in splits:
            raise ValueError('source episode has no matching ID/hash-bound split')
        arrays, digest = read_sapien(source, action_source)
        if digest != splits[source_id]['sha256']:
            raise ValueError('source episode has no matching ID/hash-bound split')
        with h5py.File(source, 'r') as archive:
            metadata = task_metadata(archive.attrs)
        sources.append((entry, source_id, source, arrays, digest, metadata))
    if not sources:
        raise ValueError('no accepted source episodes')
    output.mkdir(parents=True, exist_ok=False)
    manifest = dict(kind='SIM PGripper ACT staging; SAPIEN only, not PPO snapshots',
        fps=25, rgb_shape=[240, 320, 3], state_action_units='left6/right6; arm radians; gripper 0..1',
        action_source=action_source,
        action_semantics=dict(action=action_source,
            action_interval_first='interval_first_actual', action_interval_end='interval_end_actual',
            action_requested_target='requested_target; present only when explicitly recorded'),
        source_split_manifest=assignments, source_split_sha256=sha256(split_manifest),
        policy_inputs=['measured state', 'left wrist RGB', 'right wrist RGB'],
        top_camera='retained in original HDF5; not consumed by this two-wrist ACT',
        hardware_execution=False, train=[], holdout=[], episodes=[])
    for i, (entry, source_id, source, arrays, digest, metadata) in enumerate(sources):
        dest = output / f'episode-{i:03d}'
        dest.mkdir()
        with h5py.File(source, 'r') as archive:
            for side in ('left', 'right'):
                images = archive[f'observation/images/{side}_wrist_rgb']
                if len(images) != entry['frames']:
                    raise ValueError('image frame count mismatch')
                for t in range(len(arrays['state'])):
                    encoded = images[t].tobytes()
                    with Image.open(io.BytesIO(encoded)) as image:
                        if image.size != (320, 240) or image.mode != 'RGB':
                            raise ValueError('invalid wrist RGB')
                        image.load()
                    (dest / f'{side}-{t:05d}.jpg').write_bytes(encoded)
        np.savez_compressed(dest / 'episode.npz', **arrays)
        split = splits[source_id]['split']
        # Unknown or pick-only success is inspection data, not full-task training evidence.
        if metadata['canonical_full_task_success'] is True:
            manifest[split].append(i)
        manifest['episodes'].append(dict(episode=i, source=str(source), sha256=digest,
            source_episode_id=source_id, source_split=split, action_source=action_source,
            simulator='SAPIEN', frames=len(arrays['state']), source_frames=entry['frames'], **metadata))
        print(json.dumps(manifest['episodes'][-1]), flush=True)
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    return manifest


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--mujoco-dataset', type=Path)
    parser.add_argument('--split-manifest', type=Path, required=True)
    parser.add_argument('--action-source', choices=ACTION_SOURCES, required=True)
    args = parser.parse_args()
    print(json.dumps(convert(args.dataset, args.output, args.mujoco_dataset,
        split_manifest=args.split_manifest, action_source=args.action_source), indent=2))
