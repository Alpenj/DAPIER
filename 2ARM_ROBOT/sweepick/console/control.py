"""Forward explicit operator requests to the existing managed A owner. Never open devices."""
from contextlib import contextmanager
from datetime import datetime
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import secrets
import re
import shlex
import subprocess
import threading

from replay import HOME, ROOTS, JOINTS, ReplaySession, read_json, terminal_evidence
from records import atomic_json, capture_sources, run_record, save_review

A = HOME / 'sweepick_261013_receive_alignment_restore'
PYTHON = HOME / 'DAPIER/so101_imitation_learning/.venv/bin/python'
SCOPE = 'Fresh LEFT PICK/LIFT/CARRY/RIGHT PREGRASP → 비접촉 관측 → 조건이 충족된 put-back/home/OFF'
MANAGED = 'sweepick_console_field'
FULL_CHAIN = 'FULL_CHAIN_RECEIVE_CONTACT'
LEFT_TO_LIFT = 'LEFT_START_TO_LIFT'
# Console-17: the native files that must carry the limited-scope contract before the console offers it. Read as text; never imported.
LIMITED_BACKEND_MARKERS = {'RUN_APPROVED_SESSION.py': ("def validate_limited_launch(", "LIMITED_APPROVAL_SCOPE = 'LEFT_START_TO_LIFT'"),
                           'revision/sweepick_pick01.py': ('LEFT_TO_LIFT = "LEFT_START_TO_LIFT"', 'def write_limited_terminal(', 'def limited_lift_end(')}
LIMITED_BOUND_SOURCES = ('RUN_APPROVED_SESSION.py', 'confirmed_wrist_routes.py', 'revision/sweepick_pick01.py', 'revision/sweepick_move_a.py',
                         'revision/sweepick_grasp.py', 'revision/sweepick_episode.py', 'revision/sweepick_full_chain.py',
                         'revision/sweepick_jaw_profile_left.json', 'revision/sweepick_progress_profile.json', 'revision/console_feedback.py')
LIMITED_TERMINALS = ('LEFT_LIFT_HELD_CONFIRMED', 'LEFT_LIFT_HOLD_UNKNOWN', 'LEFT_CLOSE_UNAVAILABLE', 'LEFT_LIFT_FAILED',
                     'OPERATOR_STOPPED', 'BUS_OR_SERVO_FAULT', 'ENDED_BEFORE_LIFT', 'NOT_STARTED')


TESTED_NATIVE = Path(__file__).resolve().parent / 'CONSOLE17_TESTED_NATIVE_SHA256.json'


def native_source_check():
    """Console-17: the active native files against the SHA of the source that passed the Fake regression. Read-only."""
    try:
        tested = read_json(TESTED_NATIVE).get('files') or {}
    except (OSError, ValueError, AttributeError):
        tested = {}
    rows = {}
    for name in LIMITED_BOUND_SOURCES:
        path = A / name
        active = hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None
        rows[name] = {'active_sha256': active, 'tested_sha256': tested.get(name), 'match': bool(active and active == tested.get(name))}
    return {'tested_record': str(TESTED_NATIVE), 'tested_record_present': bool(tested), 'files': rows,
            'all_match': bool(tested) and all(row['match'] for row in rows.values())}


def limited_backend():
    """(supported, reason) for LEFT_START_TO_LIFT: the contract markers AND the active native SHA equal to the tested source."""
    missing = []
    for name, markers in LIMITED_BACKEND_MARKERS.items():
        try:
            text = (A / name).read_text(errors='replace')
        except OSError:
            missing.append(f'{name}: not readable')
            continue
        missing.extend(f'{name}: {marker}' for marker in markers if marker not in text)
    if missing:
        return False, 'native 실행본에 Console-17 제한 종료 계약이 없음: ' + '; '.join(missing)
    check = native_source_check()
    if not check['all_match']:
        differs = [name for name, row in check['files'].items() if not row['match']]
        return False, ('시험 기록이 없음' if not check['tested_record_present'] else '활성 native SHA가 시험한 소스와 다름: ' + ', '.join(differs))
    return True, 'native 계약 표식 + 활성 native SHA = 시험한 소스 SHA'



class ControlError(Exception):
    pass


class Controller:
    def __init__(self, directory=None, run=subprocess.run):
        self.directory = Path(directory or HOME / 'sweepick_console_runs')
        self.directory.mkdir(exist_ok=True)
        self.state_file = self.directory / 'CONTROL.json'
        self.run = run
        self.csrf_token = secrets.token_urlsafe(32)
        self.event_lock = threading.RLock()
        self.records_lock = threading.RLock()
        self.event_key, self.event_offset, self.event_rows = None, 0, []

    def episode_events(self, root):
        path = Path(root) / 'episode/events.jsonl'
        if not path.exists():
            return []
        with self.event_lock:
            stat = path.stat()
            key = (str(path), stat.st_ino)
            if key != self.event_key or stat.st_size < self.event_offset:
                self.event_key, self.event_offset, self.event_rows = key, 0, []
            # Native journal is append-only; keep stage/error events when repetitive hold notes grow.
            with path.open('rb') as stream:
                stream.seek(self.event_offset)
                while line := stream.readline():
                    try:
                        row = json.loads(line)
                    except ValueError:
                        if not line.endswith(b'\n'):
                            break
                    else:
                        if isinstance(row, dict) and row.get('kind') in ('phase', 'stage_record', 'trail', 'owner_stop', 'episode_end',
                            'observation_plan_authorization_failed', 'awaiting_observation_move_approval',
                            'observation_move_approval_consumed', 'receive_contact_outcome'):
                            self.event_rows.append(row)
                    self.event_offset = stream.tell()
            return list(self.event_rows)

    @contextmanager
    def locked(self):
        with (self.directory / 'control.lock').open('a') as stream:
            fcntl.flock(stream, fcntl.LOCK_EX)
            yield

    def save(self, record):
        temporary = self.state_file.with_suffix('.tmp')
        temporary.write_text(json.dumps(record, ensure_ascii=False, indent=2))
        temporary.replace(self.state_file)

    def managed_alive(self):
        return self.run(['tmux', 'has-session', '-t', MANAGED], capture_output=True).returncode == 0

    def pane_pid(self):
        result = self.run(['tmux', 'display-message', '-p', '-t', MANAGED, '#{pane_pid}'], capture_output=True, text=True)
        return int(result.stdout.strip()) if result.returncode == 0 and result.stdout.strip().isdigit() else None

    @staticmethod
    def pid_start(pid):
        try:
            return Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()[19]
        except (OSError, IndexError):
            return None

    def owner_matches(self, job):
        pid = self.pane_pid()
        return bool(pid and pid == job.get('owner_pid') and self.pid_start(pid) == job.get('owner_start'))

    def status(self):
        job = read_json(self.state_file) or None
        alive = self.managed_alive()
        ours = bool(alive and job and self.owner_matches(job))
        state = 'IDLE'
        reason = None
        if job:
            terminal = terminal_evidence(job['session_out'])
            job['terminal'] = terminal
            limited = None
            if job.get('execution_scope') == LEFT_TO_LIFT:
                try:
                    limited = read_json(Path(job['session_out']) / 'LIMITED_SCOPE_TERMINAL.json') or None
                except (OSError, ValueError):
                    limited = {'terminal': 'UNKNOWN', 'error': 'LIMITED_SCOPE_TERMINAL.json unreadable'}
                job['limited_terminal'] = limited
            terminal_error = terminal.get('error')
            terminal_time = (terminal.get('failed_stage') or {}).get('t_host_s')
            existing_time = (job.get('first_error') or {}).get('time_s') if isinstance(job.get('first_error'), dict) else None
            if terminal_error and (not isinstance(existing_time, (int, float)) or
                    isinstance(terminal_time, (int, float)) and terminal_time < existing_time):
                job['first_error'] = {**terminal_error, 'time_s': terminal_time,
                                      'source': terminal.get('move_file') or 'review/lifetime.jsonl'}
            episode = self.episode_events(job['session_out'])
            failure = next((e for e in episode if e.get('kind') in ('observation_plan_authorization_failed', 'owner_stop')), None)
            episode_ended = any(e.get('kind') == 'episode_end' for e in episode)
            if job['state'] in ('STARTING', 'RUNNING', 'STOP_REQUESTED'):
                job['state'] = 'STOP_REQUESTED' if ours and job['state'] == 'STOP_REQUESTED' else 'RUNNING' if ours else 'ENDED'
                events = self.tail(Path(job['session_out']) / 'review/lifetime.jsonl')
                if events:
                    job['phase'] = events[-1].get('phase', 'UNKNOWN')
                    ended = next((e for e in reversed(events) if e.get('event') == 'owner_end'), None)
                    if ended:
                        job['exit_code'] = ended.get('exit_code')
                        if ended.get('error'):
                            job['error'] = ended['error']
                feed = read_json(Path(job['session_out']) / 'CURRENT_FEEDBACK.json')
                log = Path(job['session_out']).parent / 'owner.log'
                if not alive and not events and log.exists():
                    lines = log.read_text(errors='replace').splitlines()
                    if lines:
                        job['error'] = lines[-1]
                if not alive and job.get('execution_scope') == LEFT_TO_LIFT:
                    # Console-17: the native terminal decides the class; holding is never HOME/OFF and never allows the next run by itself
                    kind = (limited or {}).get('terminal')
                    # a recorded servo/bus/executor error of the same session wins over a holding terminal (same rule as records.run_record)
                    recorded_error = job.get('first_error') or job.get('error') or feed.get('first_error')
                    job['state'] = 'LIMITED_HOLDING' if kind in ('LEFT_LIFT_HELD_CONFIRMED', 'LEFT_LIFT_HOLD_UNKNOWN') and not recorded_error else 'FAULT'
                    reason = (f"LEFT_START_TO_LIFT 종료 {kind or 'UNKNOWN(종료 기록 없음)'} · 정상 HOME/OFF 아님 · 물체·팔 지지 후 수동 복구 확인 필요"
                              + (f" · 같은 세션 오류 우선: {((job.get('first_error') or feed.get('first_error') or {}) if isinstance(job.get('first_error') or feed.get('first_error'), dict) else {}).get('detail') or job.get('error')}" if recorded_error else ''))
                    job['block_reason'] = reason
                elif not alive and (job.get('first_error') or job.get('error') or feed.get('first_error') or not self.off_confirmed(job)):
                    job['state'] = 'FAULT'
                    reason = (job.get('first_error') or {}).get('detail') or job.get('error') or (feed.get('first_error') or {}).get('detail') or '종료 토크 상태가 확인되지 않음 · 기존 현장 복구 필요'
                    job['block_reason'] = reason
            if failure:
                detail = failure.get('reason') or '; '.join(failure.get('reasons', [])) or 'existing owner stopped'
                event_error = {'gate': 'LEFT_HOLD_VISION_UNCONFIRMED' if 'fresh left wrist' in detail else 'OWNER_STOP',
                               'detail': detail, 'time_s': failure.get('t_host_s'), 'source': 'episode/events.jsonl'}
                prior_time = (job.get('first_error') or {}).get('time_s') if isinstance(job.get('first_error'), dict) else None
                if not isinstance(prior_time, (int, float)) or (isinstance(event_error['time_s'], (int, float)) and event_error['time_s'] < prior_time):
                    job['first_error'] = event_error
                job['state'] = 'FAULT'
                job['block_reason'] = reason = detail + ' · 자동 재시작 금지; 정상 회수/종료와 별도 복구 확인을 구분'
            state = job['state']
            reason = reason or job.get('block_reason')
            request = Path(job['session_out']) / 'OBSERVATION_MOVE_REQUEST.json'
            decision = request.with_name('OBSERVATION_MOVE_APPROVAL.json')
            job['pending_view'] = None
            if ours and state == 'RUNNING' and not episode_ended and request.exists():
                try:
                    data = request.read_bytes()
                    digest = hashlib.sha256(data).hexdigest()
                    proposed = json.loads(data)
                    answered = read_json(decision)
                    if answered.get('request_sha256') != digest:
                        job['pending_view'] = {key: proposed.get(key) for key in ('scope', 'plan', 'candidate')}
                        job['pending_view']['request_sha256'] = digest
                except (OSError, ValueError):
                    job['pending_view_state'] = 'UNKNOWN · 요청 파일 수신 중'
        if alive and not ours:
            state, reason = 'BLOCKED', '기존 managed owner가 존재함'
        recovery = self.recovery_confirmation(job) if job and state in ('FAULT', 'LIMITED_HOLDING') else None
        if job:
            job['operator_recovery'] = recovery
        if recovery:
            reason = (reason or '원래 실패 기록 보존') + ' · 수동 복구 처리 확인됨; 새 준비와 이번 현장 승인 후 fresh 상태로 재검사'
        return {'csrf_token': self.csrf_token, 'execution_connected': True, 'state': state, 'scope': SCOPE, 'owner_alive': ours,
                'active': job, 'can_prepare': not alive and (state not in ('FAULT', 'LIMITED_HOLDING') or recovery is not None),
                'can_confirm_recovery': not alive and state in ('FAULT', 'LIMITED_HOLDING') and recovery is None,
                'can_start': not alive and state == 'PREPARED', 'can_stop': bool(ours and state != 'STOP_REQUESTED' and not job.get('stop_sent')),
                'block_reason': reason}

    def recovery_confirmation(self, job):
        try:
            record = read_json(Path(job['session_out']) / 'review/MANUAL_RECOVERY_CONFIRMATION.json')
        except (OSError, ValueError):
            return None
        if (record.get('job_id') == job['id'] and record.get('session_out') == job['session_out']
                and record.get('first_error') == job.get('first_error')
                and record.get('power_isolated') is True and record.get('workspace_reset') is True
                and record.get('source') == 'local_operator_manual_confirmation'):
            return record
        return None

    def confirm_recovery(self, job_id, *, power_isolated=False, workspace_reset=False):
        if power_isolated is not True or workspace_reset is not True:
            raise ControlError('팔·물체 지지 후 양팔 서보 전원 차단과 탁상 배치 복원 확인 필요')
        with self.locked():
            self.selected(job_id)
            status = self.status()
            if not status['can_confirm_recovery']:
                raise ControlError('owner 종료 뒤 현재 실패 세션에만 수동 복구 확인 가능')
            if self.owners():
                raise ControlError('장치 owner가 남아 있음; 중복 접근/새 실행 금지')
            job = status['active']
            record = {'job_id': job_id, 'session_out': job['session_out'], 'first_error': job.get('first_error'),
                      'timestamp': datetime.now().astimezone().isoformat(), 'source': 'local_operator_manual_confirmation',
                      'power_isolated': True, 'workspace_reset': True, 'torque_readback': 'UNKNOWN',
                      'normal_return_confirmed': False, 'motor_writes': 0}
            path = Path(job['session_out']) / 'review/MANUAL_RECOVERY_CONFIRMATION.json'
            path.parent.mkdir(parents=True, exist_ok=True)
            temporary = path.with_suffix('.tmp')
            temporary.write_text(json.dumps(record, ensure_ascii=False, indent=2))
            temporary.replace(path)
        return self.status()

    @staticmethod
    def tail(path, size=131072):
        if not path.exists():
            return []
        with path.open('rb') as stream:
            stream.seek(0, 2)
            offset = max(0, stream.tell() - size)
            stream.seek(offset)
            data = stream.read()
        lines = data.splitlines()
        if offset:
            lines = lines[1:]
        rows = []
        for line in lines:
            try:
                rows.append(json.loads(line))
            except ValueError:
                pass
        return rows

    def off_confirmed(self, job):
        # Final release is certified by the native executor ledger, never an optional/stale UI feed.
        root = Path(job['session_out']).resolve()
        try:
            ep = read_json(root / 'episode/EPISODE.json')
        except (OSError, ValueError):
            return False
        if not isinstance(ep, dict) or not isinstance(ep.get('stages'), list):
            return False
        last = {stage['side']: stage for stage in ep['stages'] if isinstance(stage, dict) and stage.get('side') in ('left', 'right')}
        for side in ('left', 'right'):
            stage = last.get(side, {})
            if stage.get('result') != 'RETURNED_RELEASED' or not isinstance(stage.get('folder'), str):
                return False
            folder = Path(stage['folder']).resolve()
            if not folder.is_relative_to(root):
                return False
            files = list(folder.glob('run*/move.json'))
            if not files:
                return False
            try:
                move = read_json(max(files, key=lambda path: path.stat().st_mtime_ns))
            except (OSError, ValueError):
                return False
            if (not isinstance(move, dict) or not isinstance(move.get('outcome'), dict)
                    or move.get('result') != 'RETURNED_RELEASED' or move['outcome'].get('torque_released') is not True):
                return False
            values = move.get('end', {})
            if not isinstance(values, dict):
                return False
            for register in ('torque', 'status', 'moving', 'velocity'):
                actual = values.get(register, {})
                if not isinstance(actual, dict) or set(actual) != set(JOINTS) or any(value != 0 for value in actual.values()):
                    return False
        return True

    @staticmethod
    def require_execution_scope(execution_scope):
        if execution_scope == LEFT_TO_LIFT:
            supported, reason = limited_backend()
            if not supported:
                raise ControlError('BACKEND_CHANGE_REQUIRED: ' + reason)
            return execution_scope
        if execution_scope != FULL_CHAIN:
            raise ControlError('지원되지 않는 execution_scope')
        return execution_scope

    def operation_contract(self):
        status = self.status()
        try:
            occupancy, query_error = self.owners(), None
        except (OSError, ValueError, KeyError, TypeError) as exc:
            occupancy, query_error = 'UNKNOWN', f'{type(exc).__name__}: {exc}'
        sources = []
        for name in ('RUN_APPROVED_SESSION.py', 'revision/sweepick_pick01.py', 'revision/sweepick_full_chain.py',
                     'revision/sweepick_move_a.py', 'revision/sweepick_episode.py'):
            path = A / name
            sources.append({'file': name, 'sha256': hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else 'UNKNOWN',
                            'bytes': path.stat().st_size if path.is_file() else None})
        launcher_available = (A / 'RUN_APPROVED_SESSION.py').is_file()
        limited_ok, limited_reason = limited_backend()
        return {'schema': 'sweepick.operation-contract.v1', 'checked_at': datetime.now().astimezone().isoformat(),
            'scopes': [
                {'id': FULL_CHAIN, 'label': '기존 Full-chain 실행', 'supported': launcher_available,
                 'backend_status': 'SUPPORTED_EXISTING_BACKEND' if launcher_available else 'BACKEND_NOT_AVAILABLE',
                 'can_prepare': launcher_available and status['can_prepare'] and occupancy == {},
                 'included_stages': ['START/OBSERVE', 'PICK/ALIGN/DESCEND', 'CLOSE', 'LIFT', 'CARRY', 'RIGHT PREGRASP',
                                     '승인 범위의 비접촉 관측', '조건이 충족된 기존 put-back/home/OFF'],
                 'excluded_stages': ['승인 없는 추가 관측 이동', 'RIGHT INSERT/CLOSE', '하중 이전/LEFT RELEASE/ACT'],
                 'reason': '기존 launcher의 full_chain=True, scope=RECEIVE_CONTACT 및 현재 비접촉 승인 범위를 유지합니다.',
                 'termination': '기존 정상 회수 또는 기존 중단·운영자 복구 계약'},
                {'id': LEFT_TO_LIFT, 'label': '왼팔 START → CLOSE → LIFT 제한 실행', 'supported': limited_ok,
                 'backend_status': 'SUPPORTED_CONSOLE17_LIMITED_BACKEND' if limited_ok else 'BACKEND_CHANGE_REQUIRED',
                 'can_prepare': limited_ok and status['can_prepare'] and occupancy == {},
                 'included_stages': ['현재 관측/계획', 'START/PREGRASP', 'ALIGN', 'DESCEND_A/DESCEND', 'CLOSE', 'ready_to_lift 판정', 'LIFT',
                                     'held_after_lift 판정', 'LIMITED_SCOPE_TERMINAL'],
                 'excluded_stages': ['CARRY', 'RIGHT PREGRASP/INSERT/CLOSE', '하중 이전', 'LEFT RELEASE',
                                     'LIFT 이후 자동 LOWER/OPEN/HOME/Torque OFF', 'ACT'],
                 'reason': limited_reason + ' · 기존 Full-chain 승인은 사용할 수 없고 이 범위의 새 승인·Manifest가 필요합니다.',
                 'termination': ('LIFT 뒤 LIMITED_SCOPE_TERMINAL.json 기록 후 owner 종료. 왼팔은 LIFT Goal로 Torque ON 유지 · 정상 HOME/RELEASE/safe shutdown 아님 · '
                                 '다음 실행은 수동 복구 확인 전 차단. LIFT 전 기존 거부/복귀 조건은 그대로.')}],
            'owner': {'state': status['state'], 'owner_alive': status['owner_alive'],
                      'active_run_id': (status['active'] or {}).get('id'), 'device_occupancy': occupancy,
                      'device_query_error': query_error}, 'sources': sources,
            'observation': {'status': 'NOT_CHECKED', 'basis': '새 reader/관절 조회 없음. 기존 owner의 실측 화면과 시각을 별도로 확인합니다.'},
            'calibration': {'status': 'NOT_CHECKED', 'basis': '장치 identity/EEPROM 및 Calibration 실측 대조 미수행. 기존 실행기 검사는 유지합니다.'},
            'limited_source_check': native_source_check(),
            'device_access': False}

    def prepare(self, source_session=None, *, execution_scope=FULL_CHAIN):
        self.require_execution_scope(execution_scope)
        if source_session is not None and (not isinstance(source_session, str)
                or (source_session not in ROOTS and not self.run_directory(source_session))):
            raise ControlError('알 수 없는 저장 세션')
        with self.locked():
            status = self.status()
            if not status['can_prepare']:
                raise ControlError(status['block_reason'] or '동시 세션 준비/실행 불가')
            previous = status['active']
            if previous:
                final = Path(previous['session_out']).parent / 'CONTROL_FINAL.json'
                if not final.exists():
                    final.write_text(json.dumps(previous, ensure_ascii=False, indent=2))
            job_id = datetime.now().strftime('%Y%m%d_%H%M%S_') + secrets.token_hex(3)
            directory = self.directory / job_id
            directory.mkdir()
            out = directory / 'session'
            manifest = read_json(A / 'PREPARED_SOURCE_MANIFEST.json')
            files = [Path(name) for name in manifest['files']]
            files.append(A / 'revision/console_feedback.py')
            limited = execution_scope == LEFT_TO_LIFT
            if limited:                                                  # this scope binds the current launcher/executors explicitly
                files.extend(A / name for name in LIMITED_BOUND_SOURCES)
            prepared = {'scope': LEFT_TO_LIFT if limited else 'RIGHT_RECEIVE_NONCONTACT_OBSERVATION', 'files': {
                str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in dict.fromkeys(files)}}
            if limited:
                prepared['execution_scope'] = LEFT_TO_LIFT
            try:
                atomic_json(directory / 'MANIFEST.json', prepared)
                console = Path(__file__).resolve().parent
                display_sources = {str(console / name): hashlib.sha256((console / name).read_bytes()).hexdigest()
                    for name in ('app.py', 'control.py', 'records.py', 'replay.py', 'inspection.py', 'live.py', 'static/app.js', 'static/review.js', 'static/field.js', 'static/index.html', 'static/style.css')}
                snapshot = capture_sources({**prepared['files'], **display_sources}, self.directory / 'source_store')
                atomic_json(directory / 'SOURCE_SNAPSHOT.json', {'files': snapshot})
            except (OSError, ValueError) as exc:
                raise ControlError(f'실행본 저장 준비 실패; START하지 않음: {exc}') from exc
            if limited:                                                  # never derived from the full-chain approval
                approval = {'scope': LEFT_TO_LIFT, 'permission_detail': 'One supervised fresh LEFT START/PREGRASP/ALIGN/DESCEND/CLOSE/LIFT. '
                            'Ends after held_after_lift with the left arm still powered. No CARRY, no RIGHT arm command, no automatic return after LIFT. No automatic next session.',
                            'right_arm_commands_permitted': False, 'automatic_return_after_lift': False, 'permission_to_open_devices': False,
                            'unchanged_wrist_mount_and_routes': False, 'policy_training_allowed': False}
            else:
                approval = read_json(A / 'NONCONTACT_START_APPROVAL.json')
            approval.update(approved=False, permission_to_execute=False, field_ready=False, session_out=str(out),
                            execution_scope=execution_scope,
                            prepared_manifest_sha256=hashlib.sha256((directory / 'MANIFEST.json').read_bytes()).hexdigest(),
                            approval_question_id='local_operator:' + job_id, current_instruction='fresh noncontact session via local operator console')
            (directory / 'APPROVAL.json').write_text(json.dumps(approval, ensure_ascii=False, indent=2))
            job = {'id': job_id, 'source_session': source_session, 'session_out': str(out),
                   'execution_scope': execution_scope,
                   'state': 'PREPARED', 'phase': 'awaiting_operator_approval', 'consumed': False,
                   'preparation': 'FRESH_SESSION' if source_session is None else 'ARCHIVE_REFERENCE'}
            atomic_json(directory / 'JOB.json', job)
            self.save(job)
        return self.status()

    def run_directory(self, run_id):
        if not isinstance(run_id, str) or not re.fullmatch(r'\d{8}_\d{6}_[0-9a-f]{6}', run_id):
            return None
        directory = self.directory / run_id
        if directory.is_symlink() or not directory.is_dir() or not (directory / 'APPROVAL.json').is_file():
            return None
        return directory

    def runs(self):
        status = self.status()
        records = []
        with self.records_lock:
            for directory in sorted(self.directory.iterdir(), reverse=True):
                if not self.run_directory(directory.name):
                    continue
                stamp = self.record_stamp(directory)
                try:
                    cached = read_json(directory / 'RUN_RECORD.json')
                except (OSError, ValueError):
                    cached = {}
                if not isinstance(cached, dict):
                    cached = {}
                current = status['active']
                alive = bool(current and current['id'] == directory.name and status['owner_alive'])
                if not alive and cached.get('index_signature') == stamp and (directory / 'AI_REVIEW.json').exists():
                    records.append(cached)
                    continue
                record, evidence, alive = self.run_evidence(directory, status)
                record['index_signature'] = stamp
                if not alive:
                    try:
                        save_review(directory, record, evidence)
                    except OSError as exc:
                        record['recording']['status'] = 'INCOMPLETE'
                        record['recording']['errors'].append(f'검증 기록 저장 실패: {exc}')
                records.append(record)
        return records

    @staticmethod
    def record_stamp(directory):
        files = ('APPROVAL.json', 'JOB.json', 'CONTROL_FINAL.json', 'SOURCE_SNAPSHOT.json', 'MANIFEST.json', 'owner.log',
                 'session/episode/EPISODE.json', 'session/episode/events.jsonl', 'session/episode/frames.jsonl',
                 'session/episode/joints.jsonl', 'session/STATE.json', 'session/CURRENT_FEEDBACK.json',
                 'session/review/lifetime.jsonl', 'session/review/TORQUE_OFF_OPERATOR_REQUEST.json',
                 'session/review/MANUAL_RECOVERY_CONFIRMATION.json', 'DATA_REVIEW.json')
        stamp = {name: [path.stat().st_mtime_ns, path.stat().st_size] if path.exists() else None
                 for name in files for path in [directory / name]}
        base = Path(__file__).resolve().parent
        stamp['reader_version'] = [hashlib.sha256((base / name).read_bytes()).hexdigest()
                                   for name in ('control.py', 'records.py', 'replay.py', 'inspection.py')]
        return stamp

    def run_evidence(self, directory, status):
        current = status['active']
        alive = bool(current and current['id'] == directory.name and status['owner_alive'])
        try:
            record, evidence = run_record(directory, current=current, owner_alive=alive,
                off_confirmed=False if alive else self.off_confirmed({'session_out': str(directory / 'session')}))
        except (OSError, ValueError, TypeError, AttributeError, KeyError) as exc:
            record = {'id': directory.name, 'state': 'UNKNOWN', 'category': None, 'outcome': 'UNKNOWN',
                'session_out': str(directory / 'session'), 'source_session': None, 'first_error': None,
                'normal_return_confirmed': False, 'safe_shutdown': 'UNKNOWN', 'source_snapshot_status': 'UNKNOWN',
                'recording': {'status': 'INCOMPLETE', 'files': None, 'bytes': None, 'missing': [], 'errors': [f'{type(exc).__name__}: {exc}']},
                'review_url': f'/api/runs/{directory.name}/review', 'review_file': str(directory / 'AI_REVIEW.json'),
                'replay_session_id': None}
            evidence = {'parse_error': record['recording']['errors'][0], 'originals_preserved_at': str(directory)}
        return record, evidence, alive

    def review_file(self, run_id):
        directory = self.run_directory(run_id)
        if directory is None:
            raise ControlError('알 수 없는 실행 기록')
        status = self.status()
        record, evidence, _ = self.run_evidence(directory, status)
        record['index_signature'] = self.record_stamp(directory)
        try:
            return save_review(directory, record, evidence)
        except OSError as exc:
            raise ControlError(f'검증 자료 저장 실패: {exc}') from exc

    def failure_packet(self, run_id):
        directory = self.run_directory(run_id)
        if directory is None:
            raise ControlError('알 수 없는 실행 기록')
        status = self.status()
        if status['owner_alive'] and (status.get('active') or {}).get('id') == run_id:
            raise ControlError('현재 owner 종료 후 실패 패킷을 받을 수 있음')
        self.review_file(run_id)
        path = directory / 'FAILURE_PACKET.zip'
        if not path.is_file():
            raise ControlError('이 실행에는 실패 패킷이 없음')
        return path

    def save_data_review(self, run_id, body):
        directory = self.run_directory(run_id)
        if directory is None:
            raise ControlError('알 수 없는 실행 기록')
        purpose = body.get('purpose')
        if purpose not in ('DIAGNOSTIC_SEGMENT', 'DIAGNOSTIC_ONLY', 'EXCLUDE', 'SUCCESS_CANDIDATE'):
            raise ControlError('허용되지 않은 데이터 리뷰 목적')
        record, _, alive = self.run_evidence(directory, self.status())
        if alive:
            raise ControlError('실행 종료 후 데이터 리뷰를 저장할 수 있음')
        if purpose == 'SUCCESS_CANDIDATE' and record.get('dataset', {}).get('auto_bucket') != 'success_candidates':
            raise ControlError('실패·거부·부분 실행은 SUCCESS_CANDIDATE로 저장할 수 없음')
        note, start, end = body.get('note'), body.get('start_s'), body.get('end_s')
        if purpose == 'DIAGNOSTIC_SEGMENT':
            if not isinstance(note, str) or not note.strip() or not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in (start, end)) or start >= end:
                raise ControlError('DIAGNOSTIC_SEGMENT에는 start_s < end_s와 메모가 필요함')
        archive = None
        try:
            archive = ReplaySession(run_id, directory / 'session')
            recorded_range = {'start_s': archive.start, 'end_s': archive.end}
        except (OSError, ValueError, TypeError, AttributeError, KeyError, IndexError) as exc:
            recorded_range = None
            if purpose == 'DIAGNOSTIC_SEGMENT' or start is not None or end is not None:
                raise ControlError(f'기록 시각 범위를 확인할 수 없음: {exc}') from exc
        supplied = [value for value in (start, end) if value is not None]
        if archive and any(not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value) or
                           value < archive.start or value > archive.end for value in supplied):
            raise ControlError(f'리뷰 시각은 기록 범위 {archive.start}..{archive.end} 안이어야 함')
        review = {'schema': 'sweepick.data-review.v1', 'run_id': run_id,
                  'saved_at': datetime.now().astimezone().isoformat(), 'purpose': purpose,
                  'interval': {'start_s': start, 'end_s': end} if start is not None or end is not None else None,
                  'note': note.strip() if isinstance(note, str) else '', 'human_confirmed': True,
                  'auto_bucket_at_review': record.get('dataset', {}).get('auto_bucket'),
                  'capture_integrity_at_review': record.get('recording', {}).get('status'),
                  'recorded_range': recorded_range, 'review_status': 'HUMAN_CONFIRMED',
                  'act_approved': False, 'original_outcome': record.get('outcome')}
        with self.records_lock:
            history = directory / 'DATA_REVIEW_HISTORY.jsonl'
            with history.open('a') as stream:
                stream.write(json.dumps(review, ensure_ascii=False) + '\n')
                stream.flush()
                os.fsync(stream.fileno())
            atomic_json(directory / 'DATA_REVIEW.json', review)
        return review

    def selected(self, job_id):
        job = read_json(self.state_file)
        if job.get('id') != job_id:
            raise ControlError('현재 준비/실행 중인 세션과 일치하지 않음')
        return job

    def owners(self):
        plan = read_json(A / 'RECEIVE_FIELD_PLAN.json')
        paths = set(plan['device_routes'].values())
        paths.update(('/dev/dapier/left_wrist_rgb', '/dev/dapier/right_wrist_rgb'))
        top_sysfs = Path(plan['accepted_usb_topology']['top_usb_sysfs'])
        if (top_sysfs / 'busnum').exists() and (top_sysfs / 'devnum').exists():
            paths.add(f'/dev/bus/usb/{int((top_sysfs / "busnum").read_text()):03d}/{int((top_sysfs / "devnum").read_text()):03d}')
        owners = {}
        for path in paths:
            result = self.run(['fuser', path], capture_output=True, text=True)
            if result.stdout.strip():
                owners[path] = result.stdout.strip()
        return owners

    def start(self, job_id, *, field_ready=False, mounts_unchanged=False, supervised=False, execution_scope=None):
        if execution_scope is not None:
            self.require_execution_scope(execution_scope)
        if not all(value is True for value in (field_ready, mounts_unchanged, supervised)):
            raise ControlError('배치/지지·감독·장치 결합에 대한 현장 승인 필요')
        with self.locked():
            job = self.selected(job_id)
            scope = self.require_execution_scope(job.get('execution_scope', FULL_CHAIN))
            if execution_scope is not None and execution_scope != scope:
                raise ControlError('요청 실행 범위가 준비된 작업의 범위와 다름')
            if job['consumed'] or not self.status()['can_start']:
                raise ControlError('승인이 소비되었거나 owner/종료 상태 때문에 START 불가')
            owners = self.owners()
            if owners:
                raise ControlError('장치 owner가 이미 존재함: ' + str(owners))
            directory = Path(job['session_out']).parent
            approval = read_json(directory / 'APPROVAL.json')
            expected = LEFT_TO_LIFT if scope == LEFT_TO_LIFT else 'RIGHT_RECEIVE_NONCONTACT_OBSERVATION'
            prepared_manifest = read_json(directory / 'MANIFEST.json') or {}
            manifest_scope = prepared_manifest.get('scope')
            if approval.get('scope') != expected or approval.get('execution_scope', FULL_CHAIN) != scope or manifest_scope != expected:
                raise ControlError('준비된 승인/Manifest 범위가 실행 범위와 다름; START하지 않음')
            if scope == LEFT_TO_LIFT:                                    # the approved manifest must bind exactly the tested native bytes
                tested = native_source_check()['files']
                bound = prepared_manifest.get('files') or {}
                if any(bound.get(str(A / name)) != row['tested_sha256'] for name, row in tested.items()):
                    raise ControlError('승인 Manifest가 시험한 native 소스 SHA와 다름; START하지 않음')
            approval.update(approved=True, field_ready=True, permission_to_execute=True, permission_to_open_devices=True,
                            unchanged_wrist_mount_and_routes=True, timestamp=datetime.now().astimezone().isoformat(),
                            field_confirmation='operator START click: setup/routes/support/supervision confirmed')
            (directory / 'APPROVAL.json').write_text(json.dumps(approval, ensure_ascii=False, indent=2))
            job.update(state='STARTING', consumed=True, phase='launch_requested')
            self.save(job)
            argv = ['env', 'SWEEPICK_START_APPROVAL_PATH=' + str(directory / 'APPROVAL.json'),
                    'SWEEPICK_SOURCE_MANIFEST_PATH=' + str(directory / 'MANIFEST.json'),
                    'SWEEPICK_SESSION_OUT=' + job['session_out'], 'SWEEPICK_MANAGED_SESSION=' + MANAGED,
                    'SWEEPICK_CONSOLE_FEED=1', str(PYTHON), str(A / 'RUN_APPROVED_SESSION.py'), '--execute', '--managed-tmux']
            command = 'exec ' + shlex.join(argv) + ' > ' + shlex.quote(str(directory / 'owner.log')) + ' 2>&1'
            result = self.run(['tmux', 'new-session', '-d', '-s', MANAGED, command], capture_output=True, text=True)
            if result.returncode:
                job.update(state='FAULT', error=result.stderr.strip(), block_reason='기존 실행기 시작 실패; 자동 재시도 없음')
                self.save(job)
                raise ControlError(job['block_reason'] + ': ' + job['error'])
            pid = self.pane_pid()
            job.update(owner_pid=pid, owner_start=self.pid_start(pid) if pid else None)
            self.save(job)
        return self.status()

    def stop(self, job_id):
        with self.locked():
            job = self.selected(job_id)
            if not self.status()['can_stop']:
                raise ControlError('이 콘솔의 실행 중 owner가 없음')
            Path(job['session_out'] + '.STOP').touch()
            result = self.run(['tmux', 'send-keys', '-t', MANAGED, 'C-c'], capture_output=True, text=True)
            if result.returncode:
                raise ControlError('STOP 전달 확인 불가: ' + result.stderr)
            job['state'] = 'STOP_REQUESTED'
            job['stop_sent'] = True
            self.save(job)
        return self.status()

    def view_decision(self, job_id, digest, approved):
        with self.locked():
            job = self.selected(job_id)
            pending = (self.status()['active'] or {}).get('pending_view')
            if not pending or pending['request_sha256'] != digest or not isinstance(approved, bool):
                raise ControlError('현재 actual 기반 관측 이동 요청과 일치하지 않음')
            record = {'approved': approved, 'request_sha256': digest, 'session_out': job['session_out'],
                      'scope': pending['scope'], 'operator_source': 'local_console_explicit_click'}
            path = Path(job['session_out']) / 'OBSERVATION_MOVE_APPROVAL.json'
            temporary = path.with_suffix('.tmp')
            temporary.write_text(json.dumps(record))
            temporary.replace(path)
        return self.status()
