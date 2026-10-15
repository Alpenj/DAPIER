"""Evidence from saved runs only; no actuator or sensor interfaces."""
from datetime import datetime
import hashlib
import json
from pathlib import Path
from urllib.parse import quote
from zipfile import ZipFile, ZIP_DEFLATED
import os
import tempfile

from replay import CAMERAS, UNKNOWN, ReplaySession, finite, iso_time


def file_evidence(path, base):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return {'file': path.relative_to(base).as_posix(), 'sha256': digest.hexdigest(),
            'bytes': path.stat().st_size, 'mtime_ns': str(path.stat().st_mtime_ns)}


def reference_time(record, archive):
    errors = [e for e in (record.get('first_error'), archive.fault) if isinstance(e, dict)]
    timed = [e for e in errors if finite(e.get('time_s'))]
    first = min(timed, key=lambda e: e['time_s']) if timed else next(iter(errors), None)
    return first, first['time_s'] if first and finite(first.get('time_s')) else archive.trial_end


def inspect_run(directory, record, archive=None):
    # Import after records is loaded: both download formats share the same frame selector.
    from records import _failure_frames, load, lines
    directory = Path(directory).resolve()
    root = directory / 'session'
    archive = archive or ReplaySession(record['id'], root)
    first, at = reference_time(record, archive)
    snap = archive.snapshot(at)
    quality = archive.ep.get('data_quality', {})
    frames = _failure_frames(root, at)
    sources = {root / name for name in ('episode/EPISODE.json', 'episode/events.jsonl',
               'episode/joints.jsonl', 'episode/frames.jsonl', 'STATE.json', 'CURRENT_FEEDBACK.json')}
    sources.update(directory / name for name in ('APPROVAL.json', 'MANIFEST.json', 'SOURCE_SNAPSHOT.json'))
    cameras = []
    for camera in snap['cameras']:
        key = camera['id']
        coverage = quality.get('camera_coverage', {}).get('views', {}).get(CAMERAS[key][0], {})
        if key == 'top_depth':
            coverage = {'status': 'SAMPLED_DEPTH', 'saved_every_n_framesets': archive.ep.get('top_depth_saved_every_n_framesets', UNKNOWN),
                        'note': 'RGB sequence coverage is reported on top_rgb; depth uses its own saved-frame cadence.'}
        for item in frames[key].values():
            if item.get('file'):
                path = root / item['file']
                item.update(file_evidence(path, root))
                item['image_url'] = f'/api/media/{record["replay_session_id"] or record["id"]}/{quote(item["file"])}'
                if key == 'top_depth':
                    item['image_url'] += '?depth=1'
            else:
                item['sha256'], item['image_url'] = UNKNOWN, None
        cameras.append({**camera, 'coverage': coverage, 'frames': frames[key],
                        'freshness_basis': 'archived cadence; review only, no new control threshold'})
    receipts = [c['last_received_s'] for c in cameras if c['id'] != 'top_depth' and finite(c['last_received_s'])]
    joints = snap['joints']
    for joint in joints:
        joint['goal_error_ticks'] = joint['actual'] - joint['goal'] if finite(joint['actual']) and finite(joint['goal']) else UNKNOWN
    offsets = {side: {c['id']: next((j['feedback_time_s'] - c['last_received_s'] for j in joints
               if j['side'] == side and finite(j.get('feedback_time_s')) and finite(c['last_received_s'])), UNKNOWN)
               for c in cameras if c['id'] != 'top_depth'} for side in ('left', 'right')}
    stages = archive.stage_results(archive.trial_end)
    close = next((s for s in reversed(archive.stages) if s['name'] == 'close'), None)
    if close:
        close_path = root / 'stage_close.json'
        close_doc = load(close_path, [])
        if close_path.is_file():
            sources.add(close_path)
        stages.insert(1, {'name': 'CLOSE', 'status': close['result'], 'started_s': close['start'], 'ended_s': close['end'],
            'completion': 'COMPLETED' if close_doc.get('outcome', {}).get('command_completed') is True or close['result'] == 'GRIP_CONTACT_HOLDING'
                          else 'BLOCKED_NOT_COMPLETED' if close['result'] == 'REFUSED' else UNKNOWN,
            'detail': 'contact candidate only; actual object evidence is shown separately'})
    if first:
        for stage in stages:
            if stage['name'] == 'OBSERVE' and stage['status'] == 'FAILED':
                stage['ended_s'] = at
    holding = next((s for s in reversed(archive.stages) if s['end'] <= at and s['result'] in ('HOLDING', 'GRIP_CONTACT_HOLDING')), None)
    grasp_checks = [e for e in archive.events if e.get('kind') == 'trail' and e.get('step') in ('look', 'visual')
                    and e.get('gate') == 'held_after_lift' and e.get('ok') is True and e.get('t_host_s', float('inf')) <= at]
    # STATE evidence is retained separately when the recorder event only gives a trail index.
    held = next((e for e in archive.state.get('evidence', []) if e.get('what') == 'held_after_lift' and e.get('ok') is True
                 and finite(e.get('t_host_s', e.get('t'))) and e.get('t_host_s', e.get('t')) <= at), None)
    grasp = grasp_checks[-1] if grasp_checks else held
    grasp_time = grasp.get('t_host_s', grasp.get('t')) if grasp else None
    off_source, off_time, off = None, None, UNKNOWN
    for name in ('TORQUE_OFF_RESULT.json', 'TORQUE_OFF_OPERATOR_REQUEST.json'):
        path = root / 'review' / name
        if not path.is_file():
            continue
        sources.add(path)
        doc = load(path, [])
        if name == 'TORQUE_OFF_OPERATOR_REQUEST.json':
            measured = doc.get('sides', {})
            all_off = all(measured.get(side, {}).get('after', {}).get('answered') is True and
                          all(measured[side]['after'].get('state', {}).get('torque', {}).get(j) == 0
                              for j in ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper'))
                          for side in ('left', 'right'))
            off = True if all_off and doc.get('all_off_confirmed') is True else UNKNOWN
        # A left-only cleanup cannot establish both arms OFF.
        off_source, off_time = path.relative_to(root).as_posix(), iso_time(doc.get('finished') or doc.get('timestamp'))
    manual_path = root / 'review/MANUAL_RECOVERY_CONFIRMATION.json'
    manual = load(manual_path, [])
    if manual_path.is_file():
        sources.add(manual_path)
    request_path = root / 'OBSERVATION_MOVE_REQUEST.json'
    request = load(request_path, [])
    if request_path.is_file():
        sources.add(request_path)
    return_plan = request.get('plan', {}).get('normal_return', {})
    control_checks = [e for e in archive.events if e.get('kind') == 'observation_return_check'
                      and finite(e.get('t_host_s')) and e['t_host_s'] <= at]
    shutdown = {'normal_return_confirmed': record.get('normal_return_confirmed', UNKNOWN),
                'torque_off_confirmed': off, 'torque_off_source': off_source, 'torque_off_time_s': off_time,
                'safe_shutdown': record.get('safe_shutdown', UNKNOWN), 'manual_recovery': manual or UNKNOWN,
                'basis': 'Normal return, measured OFF, and operator recovery are separate evidence.'}
    # This is an export window, never a robot freshness or motion gate.
    lo, hi = at - 3, at + 3
    motor, events, frame_rows = [], [], []
    for row in archive.events:
        if finite(row.get('t_host_s')) and lo <= row['t_host_s'] <= hi:
            events.append(row)
    for relative in ['episode/joints.jsonl'] + sorted({r['source'] for rows in archive.feedback.values()
            for r in rows if ':' not in r['source'] and r['source'].endswith('/move.json')}):
        path = (root / relative).resolve()
        if not path.is_relative_to(root) or not path.is_file():
            continue
        data = load(path, []) if relative.endswith('move.json') else None
        rows = data.get('rows', []) if data is not None else lines(path, [])
        side = next((s['side'] for s in archive.stages if relative.startswith(s['name'] + '/')), None)
        for index, row in enumerate(rows, 1):
            t = row.get('t_host_s', row.get('t'))
            if finite(t) and lo <= t <= hi:
                motor.append({'source': relative, 'row': index, 'side': row.get('side', side), 'raw': row})
                sources.add(path)
    for index, row in enumerate(lines(root / 'episode/frames.jsonl', []), 1):
        if finite(row.get('recv_host_s')) and lo <= row['recv_host_s'] <= hi:
            frame_rows.append({'source': 'episode/frames.jsonl', 'row': index, 'raw': row})
    # No synthetic/interpolated samples: explicitly carry nearest evidence if the window has a gap.
    boundary = {side: next((r for r in reversed(rows) if r['t'] <= at), None) for side, rows in archive.feedback.items()}
    window = {'events': events, 'motor_samples': sorted(motor, key=lambda r: r['raw'].get('t_host_s', r['raw'].get('t', 0))),
              'frame_index': frame_rows, 'nearest_prior_feedback': boundary}
    unknowns = ['물리 촬영 시각의 3-view 동기화: UNKNOWN (host 수신 시각 차이만 측정)',
                '관절별 ACK가 같은 명령·시각에 대응하지 않으면 UNKNOWN; Goal readback은 물리 도달 확인과 다름']
    if first:
        unknowns.append('실패 시점 물체 지지/파지: 최신 검증 근거 없음은 물체 이탈의 증거가 아님')
    if shutdown['safe_shutdown'] == UNKNOWN:
        unknowns.append('safe_shutdown 미확인; 별도 OFF 또는 수동 전원 차단으로 정상 회수를 확정하지 않음')
    if not grasp:
        unknowns.append('실제 물체 파지 확인 자료 없음; HOLDING은 실행기 결과')
    if record.get('source_snapshot_status') != 'SAVED':
        unknowns.append('당시 실행 소스/Calibration/Profile 사본 일부 또는 전체 미보관; 기록된 SHA를 현재 파일로 대체하지 않음')
    boundary_ages = {side: at - row['t'] if row else UNKNOWN for side, row in boundary.items()}
    if any(row is None or row['t'] < lo for row in boundary.values()):
        unknowns.append('실패 전 3초 export 구간에 일부 팔의 12축 샘플 없음; 최근 과거 feedback age_s=' + json.dumps(boundary_ages))
    summary = {'schema': 'sweepick.inspection.v1', 'run_id': record['id'],
               'generated_at': datetime.now().astimezone().isoformat(), 'reference_time_s': at,
               'first_failure': first, 'stage_results': stages,
               'grasp': {'controller_holding': {'value': True if holding else UNKNOWN, 'stage': holding['name'] if holding else None,
                           'time_s': holding['end'] if holding else None, 'source': 'episode/EPISODE.json stages',
                           'meaning': 'executor pose/contact holding; does not establish object grasp'},
                         'object_grasp': {'value': True if grasp else UNKNOWN, 'time_s': grasp_time,
                           'source': 'episode/events.jsonl' if grasp_checks else 'STATE.json evidence' if held else None,
                           'basis': grasp or UNKNOWN, 'current_at_failure': UNKNOWN,
                           'handoff_grasp_confirmed': return_plan.get('grasp_confirmed', UNKNOWN),
                           'support_confirmed': return_plan.get('support_confirmed', UNKNOWN)},
                         'whole_task_success': {'value': False if first or str(record.get('outcome', '')).startswith('PARTIAL') else UNKNOWN,
                           'basis': record.get('outcome', UNKNOWN)}},
               'joints': joints, 'cameras': cameras,
               'control_observation': {'source': 'episode/events.jsonl observation_return_check',
                   'last_check': control_checks[-1] if control_checks else UNKNOWN,
                   'display_frame_source': 'episode/frames.jsonl; displayed nearest frame is not a new control judgment'},
               'time_alignment': {'basis': 'host receipt timestamps; not hardware exposure synchronization',
                 'host_receipt_spread_s': max(receipts) - min(receipts) if len(receipts) == 3 else UNKNOWN,
                 'physical_capture_sync': UNKNOWN, 'joint_camera_offsets_s': offsets},
               'recorder': {'status': record.get('recording', {}).get('status', UNKNOWN),
                  'errors': archive.ep.get('errors', []) + record.get('recording', {}).get('errors', []),
                  'recorder_errors': quality.get('recorder_errors', UNKNOWN),
                  'events_not_recorded': quality.get('events_not_recorded', UNKNOWN),
                  'coverage_status': quality.get('camera_coverage', {}).get('status', UNKNOWN)},
               'shutdown': shutdown,
               'readiness': {'limited_scope': {'scope': record.get('scope', UNKNOWN),
                   'status': 'NOT_READY' if first else UNKNOWN, 'basis': 'stored run result only; no current device preflight'},
                   'whole_mission': {'status': 'NOT_READY' if first else UNKNOWN,
                     'basis': 'RIGHT INSERT/CLOSE/RELEASE and complete mission not verified by this limited run'}},
               'window': {'start_s': lo, 'end_s': hi, 'event_count': len(events), 'motor_sample_count': len(motor),
                   'motor_before_count': sum(r['raw'].get('t_host_s', r['raw'].get('t', 0)) <= at for r in motor),
                   'motor_after_count': sum(r['raw'].get('t_host_s', r['raw'].get('t', 0)) > at for r in motor),
                   'nearest_prior_feedback_age_s': boundary_ages,
                   'selection_basis': 'failure/reference ±3s export only; exact raw samples, no interpolation'},
               'unknowns': unknowns, 'evidence_sources': [file_evidence(p, directory) for p in sorted(sources)
                   if p.resolve().is_relative_to(directory) and p.is_file()], 'device_access': False}
    return summary, window


def save_inspection(directory, record, archive=None):
    from records import atomic_json
    directory = Path(directory)
    summary, window = inspect_run(directory, record, archive)
    payloads = {'INSPECTION.json': json.dumps(summary, ensure_ascii=False, indent=2).encode() + b'\n'}
    for name, rows in window.items():
        payloads['window/' + name + ('.json' if isinstance(rows, dict) else '.jsonl')] = (
            json.dumps(rows, ensure_ascii=False).encode() + b'\n' if isinstance(rows, dict) else
            b''.join(json.dumps(row, ensure_ascii=False).encode() + b'\n' for row in rows))
    provenance = {}
    previous = directory / 'FAILURE_PACKET.zip'
    if record.get('dataset', {}).get('auto_bucket') == 'failed' and previous.is_file():
        with ZipFile(previous) as incoming:
            for name in incoming.namelist():
                payloads.setdefault(name, incoming.read(name))
    root = directory / 'session'
    for camera in summary['cameras']:
        for item in camera['frames'].values():
            if item.get('file'):
                payloads[item['file']] = (root / item['file']).read_bytes()
                provenance[item['file']] = {k: item.get(k) for k in ('timestamp_s', 'timestamp_kind', 'sequence', 'mtime_ns')}
    for item in summary['evidence_sources']:
        path = directory / item['file']
        # Large original indexes stay in place; their exact hashes and selected rows travel together.
        if path.stat().st_size <= 64 * 1024 and path.suffix == '.json':
            payloads.setdefault(item['file'], path.read_bytes())
            provenance[item['file']] = {'source_file': item['file'], 'mtime_ns': item['mtime_ns']}
    manifest = {'schema': 'sweepick.inspection-files.v1', 'run_id': record['id'],
                'reference_time_s': summary['reference_time_s'], 'generated_at': summary['generated_at'],
                'timestamps': 'host receipt/record times; mtime_ns is filesystem metadata, not capture time',
                'self_hash': 'FILE_SHA256.json excluded from its own hash list; verify remaining ZIP members byte for byte',
                'files': [{'file': name, 'sha256': hashlib.sha256(data).hexdigest(), 'bytes': len(data),
                           **provenance.get(name, {'derived_at': summary['generated_at']})}
                          for name, data in sorted(payloads.items())],
                'original_sources': summary['evidence_sources']}
    payloads['FILE_SHA256.json'] = json.dumps(manifest, ensure_ascii=False, indent=2).encode() + b'\n'
    atomic_json(directory / 'INSPECTION.json', summary)
    with tempfile.NamedTemporaryFile(dir=directory, suffix='.zip', delete=False) as stream:
        temporary = Path(stream.name)
    try:
        with ZipFile(temporary, 'w', ZIP_DEFLATED) as output:
            for name, data in payloads.items():
                output.writestr(name, data)
        os.replace(temporary, directory / 'INSPECTION_PACKAGE.zip')
    finally:
        temporary.unlink(missing_ok=True)
    return summary
