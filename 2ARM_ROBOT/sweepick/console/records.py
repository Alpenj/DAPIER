"""Preserve and summarize existing owner files. No robot or camera access."""
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import tempfile
from zipfile import ZIP_DEFLATED, ZipFile

from replay import CAMERAS, ROOTS, UNKNOWN, ReplaySession, clean, read_json, terminal_evidence, finite, stop_gate

SMALL_RAW_LIMIT = 64 * 1024


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, delete=False) as stream:
            temporary = Path(stream.name)
            json.dump(clean(value), stream, ensure_ascii=False, indent=2)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        temporary.replace(path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def capture_sources(files, store):
    """Keep exact approved inputs once per hash, including the large compiled model."""
    store = Path(store)
    store.mkdir(exist_ok=True)
    captured = {}
    for original, digest in files.items():
        source, target = Path(original), store / digest
        if not target.exists():
            with tempfile.NamedTemporaryFile(dir=store, delete=False) as stream:
                temporary = Path(stream.name)
                try:
                    with source.open('rb') as incoming:
                        shutil.copyfileobj(incoming, stream)
                    stream.flush()
                    os.fsync(stream.fileno())
                    if hashlib.sha256(temporary.read_bytes()).hexdigest() != digest:
                        raise ValueError(f'source changed during preparation: {original}')
                    temporary.chmod(0o444)
                    temporary.replace(target)
                finally:
                    temporary.unlink(missing_ok=True)
        elif hashlib.sha256(target.read_bytes()).hexdigest() != digest:
            raise ValueError(f'saved source hash mismatch: {target}')
        captured[original] = {'sha256': digest, 'file': str(target), 'bytes': target.stat().st_size}
    return captured


def load(path, errors):
    try:
        value = read_json(path)
        if not isinstance(value, dict):
            raise ValueError('expected JSON object')
        return value
    except (OSError, ValueError) as exc:
        errors.append(f'{path.name}: {type(exc).__name__}: {exc}')
        return {}


def lines(path, errors):
    if not path.exists():
        return
    try:
        with path.open() as stream:
            for number, line in enumerate(stream, 1):
                try:
                    row = json.loads(line)
                    if not isinstance(row, dict):
                        raise ValueError('expected JSON object')
                    yield row
                except ValueError:
                    errors.append(f'{path.name}:{number}: incomplete/malformed record')
    except OSError as exc:
        errors.append(f'{path.name}: {exc}')


# Run history: where a run stopped, for review. Read from the native trail (session/SESSION.json) and the first error; never a verdict.
STEP_LABEL = {'before_first_step': '첫 단계 전(세션·Recorder 시작)', 'observe': '초기 관측·집기 계획', 'full_chain_readiness': '전체 조작 준비 확인', 'visual': '손목 시각 정렬', 'look': '관측',
              'look_failed': '관측 실패', 'pregrasp': '집기 전 자세', 'align': '정렬', 'descend_a': '하강(A)', 'descend': '하강',
              'next_descent': '다음 하강', 'close': '그리퍼 닫기', 'lift': '들기', 'snapshot': '상태 기록', 'handoff': '인계',
              'handoff_observed': '인계 관측', 'handoff_not_entered': '인계 미진입', 'handoff_abandon': '인계 중단',
              'receive_approach_observed': '수신 접근 관측', 'receive_contact_finished': '수신 접촉 종료', 'plan_second_stop': '두 번째 정지 계획',
              'lower': '내리기', 'open': '그리퍼 열기', 'retreat': '후퇴', 'retreat_arm': '팔 후퇴', 'away': '이탈', 'home': '홈 복귀',
              'home_arm': '팔 홈 복귀', 'put_back_evidence': '되돌려 놓기 기록', 'limited_scope_terminal': '제한 범위 종료'}
_MOTOR_CHECK = re.compile(r"motor check failed on port '([^']+)'.*?Missing motor IDs:\s*((?:\s*- \d+[^\n]*\n?)+)", re.S)


def failure_reason(text):
    """A short Korean summary of a native error string; the full text stays in the record."""
    text = str(text or '')
    found = _MOTOR_CHECK.search(text)
    if found:
        ids = re.findall(r'- (\d+)', found.group(2))
        port = found.group(1).rsplit('/', 1)[-1]
        return f'{port} 모터 무응답 (ID {", ".join(ids)}) — 서보 전원·버스 케이블 확인'
    overload = re.search(r'ID=(\d+) error=0x20 \[RxPacketError\] Overload', text)
    if overload:
        return f'ID {overload.group(1)} 서보 과부하(Overload) 오류'
    if 'bus could not be opened' in text or 'bus_lost' in text:
        return 'Bus 열기 실패·연결 상실 (bus_lost)'
    first = next((line.strip() for line in text.splitlines() if line.strip()), '')
    return first[:200] or None


AFTER_STOP_STEPS = ('retreat', 'retreat_arm', 'away', 'lower', 'open', 'home', 'home_arm', 'put_back_evidence')


def native_gate(root):
    """The last native check of STATE.json evidence that came out false: {gate, look, why}. Read only."""
    try:
        evidence = read_json(Path(root) / 'STATE.json').get('evidence') or []
    except (OSError, ValueError, AttributeError):
        return None
    rows = [e for e in evidence if isinstance(e, dict)]
    row = rows[-1] if rows and rows[-1].get('ok') is False else None     # only the check the session ended on; an earlier one may have been re-aligned
    if row is None:
        return None
    return {'gate': f"{row.get('what') or UNKNOWN}=false", 'look': row.get('look'), 'why': row.get('why'), 'source': 'session/STATE.json evidence'}


def _owner_log(directory):
    """The JSON object the native owner printed last in owner.log, or {}."""
    try:
        lines = (Path(directory) / 'owner.log').read_text(errors='replace').splitlines()
        start = next((i for i, line in enumerate(lines) if line.startswith('{')), None)
        value = json.loads('\n'.join(lines[start:])) if start is not None else {}
        return value if isinstance(value, dict) else {}
    except (OSError, ValueError):
        return {}


def native_terminal(directory, ended):
    """The native result class (owner.log result) kept apart from the wrapper exit code."""
    result = _owner_log(directory).get('result')
    code = (ended or {}).get('exit_code')
    return {'result': result or UNKNOWN, 'exit_code': code,
            'exit_role': 'native non-success terminal class (not an executor crash by itself)' if code == 3 and result else
                         'normal end' if code == 0 else UNKNOWN if code is None else 'wrapper exit status'}


def recorder_errors(ep):
    """Episode recorder errors, kept apart from the first task gate. The exception chain is shown when the native recorded it."""
    rows = []
    for row in (ep.get('errors') or [])[:20] if isinstance(ep, dict) else []:
        if not isinstance(row, dict):
            continue
        exc = row.get('exception') if isinstance(row.get('exception'), dict) else None
        rows.append({'t_host_s': row.get('t_host_s'), 'what': ' '.join(str(row.get('what') or '').split())[:300],
                     'cause': (exc or {}).get('summary') or ((exc or {}).get('chain') or [{}])[-1].get('type') if exc else None,
                     'cause_status': 'RECORDED' if exc and len(exc.get('chain') or []) > 1 else
                                     'BUS_OPEN_CAUSE_UNCONFIRMED' if ': ConnectionError:' in str(row.get('what')) else 'MESSAGE_ONLY'})
    return rows


def torque_evidence(ep, feed):
    """Per arm: the final release the native certified (RETURNED_RELEASED move) apart from the last torque register a feed saw."""
    stages = [s for s in (ep.get('stages') or []) if isinstance(s, dict)] if isinstance(ep, dict) else []
    out = {}
    for side in ('left', 'right'):
        last = next((s for s in reversed(stages) if s.get('side') == side), None)
        seen = ((feed.get('sides') or {}).get(side) or {}).get('torque') if isinstance(feed, dict) else None
        values = (seen or {}).get('values') if isinstance(seen, dict) else None
        out[side] = {'final_stage': (last or {}).get('name') or (last or {}).get('stage'), 'final_result': (last or {}).get('result'),
                     'final_release_certified': bool(last and last.get('result') == 'RETURNED_RELEASED'),
                     'last_seen_torque_time_s': (seen or {}).get('time_s') if isinstance(seen, dict) else None,
                     'last_seen_torque_all_zero': bool(values) and all(v == 0 for v in values.values()) if isinstance(values, dict) else None,
                     'last_seen_role': 'LAST_OBSERVED_REGISTER; not a final torque-off check'}
    return out


def failure_point(root, first_error):
    """{stage, label, reason, detail, then, source} of the step a run stopped at, or None when nothing failed. The retreat/home
    steps the native runs after a stop are listed under `then`, not taken as the stopping step."""
    try:
        trail = read_json(Path(root) / 'SESSION.json').get('trail') or []
    except (OSError, ValueError, AttributeError):
        trail = []
    trail = [row for row in trail if isinstance(row, dict)]
    error = first_error if isinstance(first_error, dict) else {}
    failed = next((i for i in range(len(trail) - 1, -1, -1) if trail[i].get('ready') is False), None)
    if failed is None and not error:
        return None
    error_detail = error.get('detail')
    generic = not error_detail or bool(re.fullmatch(r'exit -?\d+', str(error_detail)))
    move_stage = re.match(r'([a-z_0-9]+)/run\d+/', str(error.get('source') or ''))
    if failed is not None:
        index, step, detail, source = failed, trail[failed].get('step'), trail[failed].get('why') or error_detail, 'SESSION.json trail (ready=false)'
    elif move_stage:
        step, detail, source = move_stage.group(1), error_detail, f"first_error ({error.get('source')})"
        index = next((i for i in range(len(trail) - 1, -1, -1) if trail[i].get('step') == step), None)
    elif str(error_detail) in STEP_LABEL:
        step, detail, source = str(error_detail), error.get('gate'), 'first_error (stage named by EPISODE.json)'
        index = next((i for i in range(len(trail) - 1, -1, -1) if trail[i].get('step') == step), None)
    else:
        index = next((i for i in range(len(trail) - 1, -1, -1) if trail[i].get('step') not in AFTER_STOP_STEPS), None)
        step = trail[index].get('step') if index is not None else None
        detail = (trail[index].get('why') if index is not None and generic else None) or error_detail
        source = 'SESSION.json trail (last step before retreat/home) + first_error'
    then = [STEP_LABEL.get(row.get('step'), row.get('step')) for row in trail[index + 1:]] if index is not None else []
    native = native_gate(root)
    if native and (generic or failed is None):                     # the native check that failed, not the wrapper's exit status
        error = dict(error, gate=native['gate'])
        if generic and native.get('why'):
            detail = native['why']
    if not trail and generic:                                       # nothing ran: the native's own reason (owner.log), e.g. a reader that did not open
        owner_why = _owner_log(Path(root).parent).get('why')
        if owner_why:
            step, detail, source = 'before_first_step', owner_why, 'owner.log why (no SESSION.json trail)'
    before = STEP_LABEL.get(trail[index - 1].get('step'), trail[index - 1].get('step')) if index else None
    return {'stage': step or UNKNOWN, 'label': STEP_LABEL.get(step, step or '단계 기록 없음'),
            'stage_failed': failed is not None, 'gate': error.get('gate'),
            'reason': failure_reason(detail) or error.get('gate') or UNKNOWN,
            'detail': str(detail)[:2000] if detail else None, 'after': before, 'then': then, 'source': source, 'native_gate': native}


def receive_summary(events):
    """The owner's own receive_contact_outcome, kept apart from the run category: a completed observation-only scope is
    not an insertion-ready view and not a hand-over."""
    row = next((e for e in reversed(events) if e.get('kind') == 'receive_contact_outcome'), None)
    if row is None:
        return None
    result = row.get('result')
    view = next((e for e in reversed(events) if e.get('kind') == 'observation_view_actual_consumed'), None)
    return {'result': result, 'time_s': row.get('t_host_s'), 'why': row.get('why'), 'normal_end': row.get('normal_end'),
            'diagnostic_scope_completed': result in ('RECEIVE_CONTACT_COLLECTED', 'RECEIVE_CONTACT_NO_CONTACT', 'RECEIVE_CONTACT_NOT_READY_RETURNED'),
            'insertion_ready': view.get('insertion_ready') if view else None, 'bimanual_handover_success': False,
            'basis': 'episode/events.jsonl receive_contact_outcome; this scope never releases the left hand'}


def run_record(directory, *, current=None, owner_alive=False, off_confirmed=False):
    directory = Path(directory).resolve()
    root, errors = directory / 'session', []
    job = (current if current and current.get('id') == directory.name else None)
    job = job or load(directory / 'CONTROL_FINAL.json', errors) or load(directory / 'JOB.json', errors)
    approval = load(directory / 'APPROVAL.json', errors)
    ep = load(root / 'episode/EPISODE.json', errors)
    feed = load(root / 'CURRENT_FEEDBACK.json', errors)
    events = list(lines(root / 'episode/events.jsonl', errors))
    lifecycle = list(lines(root / 'review/lifetime.jsonl', errors))
    ended = next((row for row in reversed(lifecycle) if row.get('event') == 'owner_end'), None)
    terminal = terminal_evidence(root)
    attempted = bool(job.get('consumed') or approval.get('approved') or root.exists())
    state = 'RUNNING' if owner_alive else 'ENDED' if attempted else 'PREPARED'
    error_candidates = [value for value in (job.get('first_error'), feed.get('first_error')) if isinstance(value, dict)]
    stopped = next((row for row in events if row.get('kind') in ('observation_plan_authorization_failed', 'owner_stop')), None)
    if stopped:
        detail = stopped.get('reason') or '; '.join(stopped.get('reasons', [])) or UNKNOWN
        error_candidates.append({'gate': stop_gate(detail), 'stop_event': 'owner_stop',
                                 'detail': detail, 'time_s': stopped.get('t_host_s'), 'source': 'episode/events.jsonl'})
    if terminal.get('error'):
        error_candidates.append({**terminal['error'], 'time_s': (terminal.get('failed_stage') or {}).get('t_host_s'),
                                 'source': terminal.get('move_file') or 'review/lifetime.jsonl'})
    limited = (job.get('execution_scope') == 'LEFT_START_TO_LIFT')
    limited_doc = load(root / 'LIMITED_SCOPE_TERMINAL.json', []) if limited and (root / 'LIMITED_SCOPE_TERMINAL.json').exists() else None
    # Console-17: a limited session's exit 3 is its terminal class (recorded in LIMITED_SCOPE_TERMINAL.json), not an executor error
    limited_exit = bool(limited_doc and ended and not ended.get('error') and ended.get('exit_code') in (0, 3))
    if ended and not limited_exit and (ended.get('error') or ended.get('exit_code') not in (0, None)):
        error_candidates.append({'gate': 'EXECUTOR_ERROR', 'detail': ended.get('error') or f"exit {ended.get('exit_code')}",
                                 'source': 'review/lifetime.jsonl'})
    if job.get('error'):
        error_candidates.append({'gate': 'EXECUTOR_ERROR', 'detail': job['error'], 'source': 'CONTROL.json'})
    error = min(error_candidates, key=lambda item: item.get('time_s') if isinstance(item.get('time_s'), (int, float)) else float('inf'), default=None)
    if job.get('state') == 'FAULT' or error:
        state = 'FAULT'
    if attempted and not owner_alive and not off_confirmed:
        state = 'FAULT'
    limited_kind = (limited_doc or {}).get('terminal')
    if limited and attempted and not owner_alive and not error and limited_kind in ('LEFT_LIFT_HELD_CONFIRMED', 'LEFT_LIFT_HOLD_UNKNOWN'):
        state = 'LIMITED_HOLDING'                                      # holding at the lift: neither success/HOME nor a fault record
    if not attempted:
        state = 'PREPARED'
    completed = [row for row in events if row.get('kind') == 'stage_record']
    progressed = any(row.get('stage') in ('lift', 'handoff_approach_right_approach') and row.get('result') == 'HOLDING'
                     for row in completed)
    outcome = ep.get('task_outcome', UNKNOWN)
    finished = {row.get('stage') for row in completed if row.get('result') in ('HOLDING', 'GRIP_CONTACT_HOLDING')}
    scope_done = (outcome == 'PARTIAL_OR_DIAGNOSTIC: RECEIVE_CONTACT_DONE_PUT_BACK' and
                  {'close', 'lift', 'handoff_approach_carry_right', 'handoff_approach_carry_left',
                   'handoff_approach_right_approach'} <= finished)
    category = ('success' if not owner_alive and off_confirmed and not error and scope_done
                else 'partial' if progressed else 'failure') if attempted else None
    auto_bucket = ('failed' if category == 'failure' or error or terminal.get('refused') or terminal.get('stopped_by') or terminal.get('exception')
                   else 'success_candidates' if category == 'success' else 'partial') if attempted else None
    missing = []
    frames = {key: 0 for key in CAMERAS}
    for row in lines(root / 'episode/frames.jsonl', errors):
        camera = 'top_rgb' if row.get('camera') == 'top' else row.get('camera')
        for key, field in ((camera, 'file'), ('top_depth', 'depth_file')):
            if not isinstance(key, str) or key not in frames or not row.get(field):
                continue
            if not isinstance(row[field], str):
                errors.append(f'frames.jsonl: invalid {field} path for {key}')
                continue
            relative = 'episode/' + row[field]
            path = (root / relative).resolve()
            if not path.is_relative_to(root) or not path.is_file() or path.stat().st_size == 0:
                missing.append(relative)
            else:
                frames[key] += 1
    if attempted and not owner_alive:
        for relative in ('episode/EPISODE.json', 'episode/events.jsonl', 'episode/frames.jsonl', 'episode/joints.jsonl', 'STATE.json'):
            path = root / relative
            if not path.is_file() or path.stat().st_size == 0:
                missing.append(relative)
        missing.extend(key + ' frames' for key, count in frames.items() if count == 0)
    quality = ep.get('data_quality', {})
    if not isinstance(quality, dict):
        errors.append('EPISODE.json: invalid data_quality')
        quality = {}
    coverage = quality.get('camera_coverage', {})
    if not isinstance(coverage, dict):
        errors.append('EPISODE.json: invalid camera_coverage')
        coverage = {}
    snapshot = load(directory / 'SOURCE_SNAPSHOT.json', errors)
    manifest = load(directory / 'MANIFEST.json', errors)
    snapshot_status = 'NOT_AVAILABLE'
    captured = snapshot.get('files')
    if isinstance(captured, dict) and captured:
        snapshot_status = 'SAVED'
        for original, item in captured.items():
            path = Path(item['file']) if isinstance(item, dict) and isinstance(item.get('file'), str) else None
            if path is None or not path.is_file() or path.stat().st_size != item.get('bytes'):
                snapshot_status = 'INCOMPLETE'
                errors.append(f'source snapshot missing/incomplete: {original}')
    recorder_failed = bool(ep.get('errors') or quality.get('recorder_errors') or quality.get('events_not_recorded'))
    incomplete_coverage = attempted and quality.get('complete_three_view_coverage') is not True
    data_status = ('RECORDING' if owner_alive else 'NOT_STARTED' if not attempted else
                   'INCOMPLETE' if missing or errors or recorder_failed or incomplete_coverage else 'SAVED')
    originals = []
    for name in ('APPROVAL.json', 'MANIFEST.json', 'SOURCE_SNAPSHOT.json', 'JOB.json', 'CONTROL_FINAL.json', 'owner.log'):
        path = directory / name
        if path.is_file():
            originals.append({'file': name, 'bytes': path.stat().st_size})
    if root.exists():
        for path in root.rglob('*'):
            if path.is_file() and not path.is_symlink():
                originals.append({'file': path.relative_to(directory).as_posix(), 'bytes': path.stat().st_size})
    replay_id = next((key for key, path in ROOTS.items() if path.resolve() == root), directory.name)
    replay_ready = not owner_alive and any(row.get('t_host_s') is not None for row in events)
    record = {'schema': 'sweepick.console-run.v1', 'id': directory.name, 'state': state, 'category': category,
              'verified_at': datetime.now().astimezone().isoformat(),
              'outcome': outcome, 'scope': approval.get('scope', UNKNOWN), 'source_session': job.get('source_session'),
              'execution_scope': job.get('execution_scope', 'FULL_CHAIN_RECEIVE_CONTACT'),
              'session_out': str(root), 'first_error': error, 'normal_return_confirmed': off_confirmed and error is None,
              'safe_shutdown': 'RECORDED_BOTH_ARMS_OFF' if off_confirmed and error is None else 'NOT_PERFORMED' if limited_kind in ('LEFT_LIFT_HELD_CONFIRMED', 'LEFT_LIFT_HOLD_UNKNOWN') else UNKNOWN,
              'receive_contact': receive_summary(events),
              'limited_terminal': None if not limited else (limited_doc or {'terminal': UNKNOWN, 'missing': 'LIMITED_SCOPE_TERMINAL.json'}),
              'recording': {'status': data_status, 'files': len(originals), 'bytes': sum(row['bytes'] for row in originals),
                            'camera_frames': frames, 'missing': sorted(set(missing)), 'errors': errors,
                            'coverage': coverage.get('status', UNKNOWN)},
              'source_snapshot_status': snapshot_status,
              'review_url': f'/api/runs/{directory.name}/review', 'review_file': str(directory / 'AI_REVIEW.json'),
              'failure_packet_url': f'/api/runs/{directory.name}/failure-packet' if auto_bucket == 'failed' else None,
              'dataset': {'auto_bucket': auto_bucket, 'capture_integrity': data_status,
                          'human_review': load(directory / 'DATA_REVIEW.json', []) or None},
              'terminal': terminal,
              'failure_point': failure_point(root, error),
              'native_terminal': native_terminal(directory, ended), 'recorder_errors': recorder_errors(ep),
              'torque_evidence': torque_evidence(ep, feed),
              'stage_results': [{'stage': s.get('stage'), 'side': s.get('side'), 'result': s.get('result')} for s in (ep.get('stages') or []) if isinstance(s, dict)][-12:],
              'replay_session_id': replay_id if replay_ready else None}
    return record, {'episode': ep, 'stages': completed, 'source_snapshot': snapshot,
                    'approval': approval, 'source_manifest': manifest,
                    'original_files': originals, 'native_data_quality': quality, 'lifecycle_end': ended}


def _bounded_json(path):
    size = path.stat().st_size
    if size <= SMALL_RAW_LIMIT:
        try:
            return {'truncated': False, 'bytes': size, 'value': read_json(path)}
        except (OSError, ValueError) as exc:
            return {'truncated': False, 'bytes': size, 'status': 'CORRUPT_INPUT',
                    'error': f'{type(exc).__name__}: {exc}', 'raw': path.read_text(errors='replace')}
    with path.open(errors='replace') as stream:
        prefix = stream.read(SMALL_RAW_LIMIT)
    return {'truncated': True, 'bytes': size, 'limit_bytes': SMALL_RAW_LIMIT, 'prefix': prefix}


def _failure_frames(root, at):
    rows, metadata = [], {}
    for row in lines(root / 'episode/frames.jsonl', []):
        camera = 'top_rgb' if row.get('camera') == 'top' else row.get('camera')
        if camera in CAMERAS and isinstance(row.get('file'), str):
            relative = 'episode/' + row['file']
            path = (root / relative).resolve()
            if path.is_relative_to(root.resolve()) and path.is_file():
                rows.append((camera, row.get('recv_host_s'), relative))
                metadata[(camera, relative)] = row
        if row.get('camera') == 'top' and isinstance(row.get('depth_file'), str):
            relative = 'episode/' + row['depth_file']
            path = (root / relative).resolve()
            if path.is_relative_to(root.resolve()) and path.is_file():
                rows.append(('top_depth', row.get('recv_host_s'), relative))
                metadata[('top_depth', relative)] = row
    selected = {}
    for camera in CAMERAS:
        candidates = [(t, f) for key, t, f in rows if key == camera and finite(t)] if finite(at) else []
        before = max(((t, f) for t, f in candidates if t <= at), default=None)
        after = min(((t, f) for t, f in candidates if t > at), default=None)
        selected[camera] = {
            'before': {'status': 'AVAILABLE', 'timestamp_s': before[0], 'file': before[1]} if before else {'status': 'UNKNOWN', 'timestamp_s': None, 'file': None},
            'after': {'status': 'AVAILABLE', 'timestamp_s': after[0], 'file': after[1]} if after else {'status': 'UNKNOWN', 'timestamp_s': None, 'file': None},
        }
        for item in selected[camera].values():
            row = metadata.get((camera, item['file']), {})
            item.update(timestamp_kind='host_receipt_s', sequence=row.get('n', UNKNOWN),
                        sdk_color_stamp_s=row.get('sdk_color_stamp_s', UNKNOWN),
                        sdk_depth_stamp_s=row.get('sdk_depth_stamp_s', UNKNOWN),
                        recv_monotonic_s=row.get('recv_monotonic_s', UNKNOWN))
    return selected


def save_failure_packet(directory, record, evidence):
    directory, root = Path(directory).resolve(), Path(directory).resolve() / 'session'
    terminal = record.get('terminal', {})
    stage = terminal.get('failed_stage') or {}
    if record.get('dataset', {}).get('auto_bucket') != 'failed':
        return None
    failure_at = stage.get('t_host_s')
    if not finite(failure_at):
        failure_at = next((row.get('time_s') for row in (record.get('first_error'),) if isinstance(row, dict) and finite(row.get('time_s'))), None)
    feed_early = load(root / 'CURRENT_FEEDBACK.json', [])
    feed_error = feed_early.get('first_error') if isinstance(feed_early, dict) and isinstance(feed_early.get('first_error'), dict) else None
    if not finite(failure_at) and feed_error and finite(feed_error.get('time_s')):
        failure_at = feed_error['time_s']                             # the recorded first servo/bus error of the owner feed
    if not finite(failure_at):
        failure_at = None                                             # Console-17: no time is invented (it was the last event, or 0)
    lifecycle_end = evidence.get('lifecycle_end') or {}
    try:
        end_wall = datetime.fromisoformat(lifecycle_end['timestamp']).timestamp() if isinstance(lifecycle_end.get('timestamp'), str) else None
    except ValueError:
        end_wall = None
    failure_sequence = [item for item in (
        {'kind': 'FIRST_SERVO_OR_BUS_ERROR', 'gate': feed_error.get('gate'), 'detail': feed_error.get('detail'), 'time_s': feed_error.get('time_s'),
         'source': 'session/CURRENT_FEEDBACK.json first_error'} if feed_error else None,
        {'kind': 'FAILED_STAGE', 'stage': stage.get('stage'), 'time_s': stage.get('t_host_s'), 'source': 'terminal failed_stage'} if stage else None,
        {'kind': 'HANDSHAKE_FAILURE' if 'motor check failed' in str(lifecycle_end.get('error')) else 'FINAL_EXCEPTION',
         'detail': lifecycle_end.get('error'), 'time_s': end_wall, 'time_basis': 'review/lifetime.jsonl owner_end timestamp (UTC wall clock of the launcher record)',
         'exit_code': lifecycle_end.get('exit_code'), 'source': 'review/lifetime.jsonl owner_end'} if lifecycle_end.get('error') else None) if item]
    move_relative = terminal.get('move_file')
    move_path = (root / move_relative).resolve() if isinstance(move_relative, str) else None
    if move_path is not None and (not move_path.is_relative_to(root) or not move_path.is_file()):
        move_path = None
    feed = load(root / 'CURRENT_FEEDBACK.json', [])
    snapshot = evidence.get('source_snapshot', {})
    files = snapshot.get('files', {}) if isinstance(snapshot, dict) else {}
    if not isinstance(files, dict) or not files:
        files = (evidence.get('source_manifest') or {}).get('files', {})
    files = files if isinstance(files, dict) else {}
    source_hashes = {name: value.get('sha256') if isinstance(value, dict) else value for name, value in files.items()}
    input_files, input_archives = [], set()
    for name, value in files.items():
        if not isinstance(value, dict) or not any(word in name.lower() for word in ('calibration', 'profile', 'candidate_q12', 'mapping')):
            continue
        captured = Path(value.get('file', ''))
        digest = value.get('sha256')
        if captured.is_file() and captured.stat().st_size <= SMALL_RAW_LIMIT and isinstance(digest, str):
            archive_file = f'inputs/{digest[:8]}_{Path(name).name}'
            if hashlib.sha256(captured.read_bytes()).hexdigest() == digest and archive_file not in input_archives:
                input_archives.add(archive_file)
                input_files.append({'source': name, 'sha256': digest, 'bytes': captured.stat().st_size,
                                    'captured_file': str(captured), 'archive_file': archive_file})
    reference = load(directory.parent / '20261014_155824_d4017e/SOURCE_SNAPSHOT.json', [])
    reference_files = reference.get('files', {}) if isinstance(reference, dict) else {}
    if not reference_files:
        reference_manifest = load(directory.parent / '20261014_155824_d4017e/MANIFEST.json', [])
        reference_files = reference_manifest.get('files', {}) if isinstance(reference_manifest, dict) else {}
    reference_hashes = {name: value.get('sha256') if isinstance(value, dict) else value for name, value in reference_files.items()}
    stage_path = (root / ('stage_' + str(stage.get('stage')) + '.json')).resolve() if stage.get('stage') else None
    if stage_path is not None and not stage_path.is_relative_to(root):
        stage_path = None
    current_move = load(move_path, []) if move_path and move_path.stat().st_size <= SMALL_RAW_LIMIT else {}
    reference_root = directory.parent / '20261014_155824_d4017e/session'
    reference_stage = load(reference_root / 'stage_close.json', [])
    reference_move = load(reference_root / 'close/run1/move.json', [])
    def close_inputs(move, stage_record):
        start = move.get('start', {}).get('ticks', {}) if isinstance(move, dict) else {}
        end = stage_record.get('end', {}) if isinstance(stage_record, dict) else {}
        acknowledgements = stage_record.get('writes') if isinstance(stage_record, dict) else None
        no_api_calls = isinstance(acknowledgements, dict) and acknowledgements and all(value == 0 for value in acknowledgements.values())
        return {'start_actual': start, 'end_actual': end.get('ticks', {}),
                'goal_register': end.get('goal_register', {}),
                'write_api_acknowledgements': acknowledgements,
                'sdk_wire_packet_attempts': 0 if no_api_calls else UNKNOWN}
    packet = {'schema': 'sweepick.failure-packet.v1', 'run_id': directory.name, 'run_outcome': record.get('outcome', UNKNOWN),
              'dataset': record.get('dataset'), 'act_approved': False,
              'failure_time_s': failure_at, 'failure_time_status': 'RECORDED' if failure_at is not None else 'UNKNOWN_NOT_RECORDED',
              'failure_sequence': failure_sequence, 'terminal': terminal, 'first_error': record.get('first_error'),
              'failed_stage': stage,
              'stage_record': _bounded_json(stage_path) if stage_path and stage_path.is_file() else {'status': 'UNKNOWN', 'reason': 'stage record unavailable'},
              'failed_move': _bounded_json(move_path) if move_path else {'status': 'UNKNOWN', 'reason': 'move file unavailable'},
              'motor_feedback': {'sides': feed.get('sides', {}), 'commands': feed.get('commands', {}),
                                 'last_write_s': feed.get('last_write_s'), 'first_error': feed.get('first_error')},
              'frames': _failure_frames(root, failure_at), 'source_sha256': source_hashes,
              'included_inputs': [{key: item[key] for key in ('source', 'sha256', 'bytes', 'archive_file')} for item in input_files],
              'source_diff_from_20261014_155824_d4017e': {
                  name: {'current_sha256': source_hashes.get(name), 'reference_sha256': reference_hashes.get(name)}
                  for name in sorted(set(source_hashes) & set(reference_hashes))
                  if source_hashes.get(name) != reference_hashes.get(name)},
              'source_inventory_changes': {'current_only': sorted(set(source_hashes) - set(reference_hashes)),
                                           'reference_only': sorted(set(reference_hashes) - set(source_hashes))},
              'close_input_comparison': {'current': close_inputs(current_move, load(stage_path, []) if stage_path and stage_path.is_file() else {}),
                                         'successful_stage_reference': close_inputs(reference_move, reference_stage)},
              'reference_context': {'run_id': '20261014_155824_d4017e',
                                    'role': 'SUCCESSFUL_STAGE_REFERENCE_NOT_WHOLE_RUN_SUCCESS'},
              'shutdown': {'normal_return_confirmed': record.get('normal_return_confirmed', False),
                           'safe_shutdown': record.get('safe_shutdown', UNKNOWN),
                           'lifecycle_end': evidence.get('lifecycle_end') or {'status': 'UNKNOWN'},
                           'records': {name: load(root / 'review' / name, []) if (root / 'review' / name).is_file()
                               else {'status': 'UNKNOWN', 'reason': 'record unavailable'} for name in
                               ('TORQUE_OFF_RESULT.json', 'TORQUE_OFF_OPERATOR_REQUEST.json', 'MANUAL_RECOVERY_CONFIRMATION.json')}}}
    small_context = []
    for path in (directory / 'owner.log', root / 'CAMERA_BINDING.json', root / 'ACTIVE_SELECTION.json'):
        resolved = path.resolve()
        if resolved.is_relative_to(directory) and resolved.is_file() and resolved.stat().st_size <= SMALL_RAW_LIMIT:
            small_context.append(resolved)
    packet['context_files'] = [{'file': path.relative_to(directory).as_posix(),
                                'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                                'bytes': path.stat().st_size} for path in small_context]
    atomic_json(directory / 'FAILURE_PACKET.json', packet)
    with tempfile.NamedTemporaryFile(dir=directory, suffix='.zip', delete=False) as stream:
        temporary = Path(stream.name)
    try:
        with ZipFile(temporary, 'w', ZIP_DEFLATED) as archive:
            archive.write(directory / 'FAILURE_PACKET.json', 'FAILURE_PACKET.json')
            for path in small_context:
                archive.write(path, path.relative_to(directory).as_posix())
            for per_view in packet['frames'].values():
                for item in per_view.values():
                    relative = item.get('file')
                    path = (root / relative).resolve() if relative else None
                    if path and path.is_relative_to(root) and path.is_file():
                        archive.write(path, relative)
            if move_path and move_path.stat().st_size <= SMALL_RAW_LIMIT:
                archive.write(move_path, move_relative)
            if stage_path and stage_path.is_file() and stage_path.stat().st_size <= SMALL_RAW_LIMIT:
                archive.write(stage_path, stage_path.relative_to(root).as_posix())
            for item in input_files:
                archive.write(item['captured_file'], item['archive_file'])
            episode = root / 'episode/EPISODE.json'
            if episode.is_file() and episode.stat().st_size <= SMALL_RAW_LIMIT:
                archive.write(episode, 'episode/EPISODE.json')
        temporary.replace(directory / 'FAILURE_PACKET.zip')
    finally:
        temporary.unlink(missing_ok=True)
    return directory / 'FAILURE_PACKET.zip'


def save_review(directory, record, evidence):
    archive = None
    originals = evidence.pop('original_files', [])
    evidence['original_files'] = {'count': len(originals), 'bytes': sum(row.get('bytes', 0) for row in originals),
        'top_level': sorted({row.get('file', '').split('/', 1)[0] for row in originals if row.get('file')}),
        'pointer': str(Path(directory) / 'session')}
    review = {'schema': 'sweepick.ai-review.v1', 'generated_at': datetime.now().astimezone().isoformat(),
              'run': record, **evidence,
              'review_request': '원본 경로를 열어 실행본·승인 범위·단계별 actual/goal/ACK·영상 시각을 대조하고 최초 오류와 저장 누락을 검증하라. UNKNOWN을 성공·미적용·안전 종료로 바꾸지 말라. 실물 명령을 보내지 말라.',
              'device_access': False}
    if record['replay_session_id']:
        try:
            archive = ReplaySession(record['id'], directory / 'session')
            archive.native_release_confirmed = record['normal_return_confirmed']
            record['dataset']['recorded_range'] = {'start_s': archive.start, 'end_s': archive.end}
            review['final_snapshot'] = archive.snapshot(archive.end)
            review['observation_views_settled_short'] = list(getattr(archive, 'settled_short_views', []))
            if archive.warnings:
                record['recording']['status'] = 'INCOMPLETE'
                record['recording']['errors'].extend(archive.warnings)
            if archive.fault and (record['first_error'] is None or
                    isinstance(archive.fault.get('time_s'), (int, float)) and
                    isinstance(record['first_error'].get('time_s'), (int, float)) and
                    archive.fault['time_s'] < record['first_error']['time_s']):
                record['first_error'] = archive.fault
                record['failure_point'] = failure_point(Path(directory) / 'session', archive.fault)
            if record['first_error']:
                record['normal_return_confirmed'], record['safe_shutdown'] = False, UNKNOWN
                if record['category'] == 'success':
                    record['category'] = 'partial'
                review['final_snapshot']['command']['safe_shutdown'] = UNKNOWN
        except (OSError, ValueError, KeyError, TypeError, AttributeError, IndexError) as exc:
            review['replay_error'] = f'{type(exc).__name__}: {exc}'
            record['replay_session_id'] = None
            record['recording']['status'] = 'INCOMPLETE'
            record['recording']['errors'].append(review['replay_error'])
    atomic_json(directory / 'AI_REVIEW.json', review)
    atomic_json(directory / 'RUN_RECORD.json', record)
    save_failure_packet(directory, record, evidence)
    if archive is not None:
        from inspection import save_inspection
        try:
            save_inspection(directory, record, archive)
        except (OSError, ValueError, TypeError, KeyError) as exc:
            atomic_json(directory / 'INSPECTION.json', {'schema': 'sweepick.inspection.v1', 'run_id': record['id'],
                'status': 'NOT_AVAILABLE', 'unknowns': [f'inspection failed: {type(exc).__name__}: {exc}'], 'device_access': False})
            (directory / 'INSPECTION_PACKAGE.zip').unlink(missing_ok=True)
    else:
        atomic_json(directory / 'INSPECTION.json', {'schema': 'sweepick.inspection.v1', 'run_id': record['id'],
            'status': 'NOT_AVAILABLE', 'unknowns': record['recording'].get('errors') or ['기록 시각·재생 근거 부족'],
            'device_access': False})
        (directory / 'INSPECTION_PACKAGE.zip').unlink(missing_ok=True)
    return directory / 'AI_REVIEW.json'
