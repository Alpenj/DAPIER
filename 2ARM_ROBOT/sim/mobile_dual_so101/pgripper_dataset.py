#!/usr/bin/env python3
"""Read-only audit of two-wrist, 12-axis PGripper ACT staging, not HW readiness.

Inspired by JD-edu's post-conversion loader validation. This audits our native
NPZ/JPEG contract; it does not import JDcobot200 states, geometry or calibration.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image

FPS = 25
UNITS = 'left6/right6; arm radians; gripper 0..1'
ACTION_SOURCES = ('requested_target', 'interval_first_actual', 'interval_end_actual')
LEGACY_REQUESTED = 'requested simulator ctrl before mj_step, not next-state labels'


def file_sha256(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def validate_arrays(arrays, *, action_source):
    """Reject swapped contracts, corrupt vectors and misaligned control labels."""
    if action_source not in ACTION_SOURCES:
        raise ValueError('unsupported action source; next-state labels are not commands')
    state, action = np.asarray(arrays['state']), np.asarray(arrays['action'])
    if (state.ndim != 2 or state.shape[1] != 12 or not len(state)
            or action.shape != state.shape):
        raise ValueError('expected nonempty, aligned [frames, 12] state/action')
    for key, values in (('state', state), ('action', action)):
        if values.dtype.kind not in 'fiu' or not np.isfinite(values).all():
            raise ValueError(f'non-finite or non-numeric {key}')
        # Measured soft-joint overshoot is retained, never clipped into a label.
        tolerance = .02 if key == 'state' else 1e-6
        if ((values[:, [5, 11]] < -tolerance).any()
                or (values[:, [5, 11]] > 1 + tolerance).any()
                or np.abs(values[:, [0, 1, 2, 3, 4, 6, 7, 8, 9, 10]]).max() > 2 * np.pi):
            raise ValueError(f'{key} violates arm-rad/gripper-0..1 contract')
    times = np.asarray(arrays['time'])
    if (times.shape != (len(state),) or not np.isfinite(times).all()
            or not np.allclose(times, np.arange(len(state)) / FPS, atol=1e-6, rtol=0)):
        raise ValueError('observations must start at zero on the contiguous 25Hz clock')
    label_key = {'requested_target': 'action_requested_target',
        'interval_first_actual': 'action_interval_first', 'interval_end_actual': 'action_interval_end'}[action_source]
    if label_key in arrays and (arrays[label_key].shape != action.shape
            or not np.allclose(action, arrays[label_key], atol=1e-6, rtol=0)):
        raise ValueError('action differs from its declared source')
    if action_source != 'requested_target' and label_key not in arrays:
        raise ValueError('actual-command source array is missing')
    if 'phase' in arrays and np.shape(arrays['phase']) != (len(state),):
        raise ValueError('phase metadata must align with observations')
    if 'control_ranges' in arrays:
        ranges = np.asarray(arrays['control_ranges'])
        dt = np.asarray(arrays['command_dt_s'])
        if (ranges.shape != (12, 2) or not np.isfinite(ranges).all()
                or np.any(ranges[:, 0] >= ranges[:, 1]) or dt.shape != ()
                or not np.isfinite(dt) or dt <= 0):
            raise ValueError('invalid recorded control ranges/clock')
        interval = round(1 / FPS / float(dt))
        if interval < 1 or not np.isclose(interval * float(dt), 1 / FPS, atol=1e-12, rtol=0):
            raise ValueError('command clock does not divide observation clock')
        dense = np.asarray(arrays['dense_ctrl'])
        sent = np.asarray(arrays['dense_sent_ctrl'])
        if (dense.ndim != 2 or dense.shape[1] != 12 or sent.shape != dense.shape
                or not np.isfinite(dense).all() or not np.isfinite(sent).all()
                or (len(dense) + interval - 1) // interval != len(state)):
            raise ValueError('dense requested/sent commands do not align with observations')
        for values in (dense, sent):
            if np.any(values < ranges[:, 0] - 1e-6) or np.any(values > ranges[:, 1] + 1e-6):
                raise ValueError('dense control outside recorded actuator limits')
        for key, values in (('action', dense), ('action_sent', sent)):
            expected = values[::interval].copy()
            expected[:, [5, 11]] = ((expected[:, [5, 11]] - ranges[[5, 11], 0])
                / (ranges[[5, 11], 1] - ranges[[5, 11], 0]))
            if key not in arrays or np.shape(arrays[key]) != expected.shape or not np.allclose(
                    arrays[key], expected, atol=1e-6, rtol=0):
                raise ValueError(f'{key} does not match pre-step dense command boundary')
        if action_source != 'requested_target':
            raise ValueError('native dense recording uses requested targets, not actual-command labels')
    return len(state)


def validate_dataset(root, *, decode_images=True):
    root = Path(root).resolve()
    manifest_path = root / 'manifest.json'
    manifest = json.loads(manifest_path.read_text())
    if (manifest.get('fps') != FPS or manifest.get('hardware_execution') is not False
            or manifest.get('state_action_units') != UNITS
            or manifest.get('rgb_shape') != [240, 320, 3]):
        raise ValueError('incompatible two-wrist PGripper SIM manifest')
    if 'action_names' in manifest:
        from mobile_dual_so101 import ACTION_NAMES
        if manifest['action_names'] != list(ACTION_NAMES):
            raise ValueError('left-six/right-six action names or order mismatch')
    source = manifest.get('action_source')
    if source == LEGACY_REQUESTED:
        source = 'requested_target'
    if source not in ACTION_SOURCES:
        raise ValueError('explicit command action source is required')
    ids = manifest.get('train', []) + manifest.get('holdout', [])
    if (not manifest.get('train') or not manifest.get('holdout')
            or any(type(i) is not int or i < 0 for i in ids) or len(ids) != len(set(ids))):
        raise ValueError('nonempty, unique, episode-disjoint train/holdout splits required')
    entries = manifest['episodes']
    if (any(type(e.get('episode')) is not int or e['episode'] < 0 for e in entries)
            or len({e['episode'] for e in entries}) != len(entries)):
        raise ValueError('invalid or duplicate episode metadata')
    metadata = {e['episode']: e for e in entries}
    checked, seen_hashes = [], set()
    for episode in ids:
        entry = metadata.get(episode, {})
        if entry.get('task_success') is not True or entry.get('canonical_full_task_success') is not True:
            raise ValueError(f'episode {episode}: full-task physical success is unverified')
        directory = root / f'episode-{episode:03d}'
        archive = directory / 'episode.npz'
        if root not in archive.resolve().parents:
            raise ValueError('episode archive escapes dataset')
        digest = file_sha256(archive)
        if digest in seen_hashes or (entry.get('episode_sha256') is not None and entry['episode_sha256'] != digest):
            raise ValueError('duplicate episode content or archive hash mismatch')
        seen_hashes.add(digest)
        with np.load(archive, allow_pickle=False) as arrays:
            frames = validate_arrays(arrays, action_source=source)
            dense_checked = 'control_ranges' in arrays
        if entry.get('frames') != frames:
            raise ValueError(f'episode {episode}: frame count mismatch')
        if decode_images:
            for side in ('left', 'right'):
                for frame in range(frames):
                    path = directory / f'{side}-{frame:05d}.jpg'
                    if root not in path.resolve().parents:
                        raise ValueError('image path escapes dataset')
                    with Image.open(path) as image:
                        if image.mode != 'RGB' or image.size != (320, 240):
                            raise ValueError('wrist camera must decode to 320x240 RGB')
                        image.load()
        checked.append(dict(episode=episode, frames=frames, sha256=digest,
            dense_command_alignment_checked=dense_checked))
    return dict(passed=True, scope='SIM data integrity; not rollout or hardware success',
        manifest_sha256=file_sha256(manifest_path), action_source=source,
        train=manifest['train'], holdout=manifest['holdout'], images_decoded=decode_images,
        episodes=checked, hardware_execution=False)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(validate_dataset(args.dataset), indent=2))
