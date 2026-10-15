"""Forward explicit operator requests to the existing managed A owner. Never open devices."""
import ast
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

from replay import HOME, ROOTS, JOINTS, ReplaySession, read_json, terminal_evidence, stop_gate
from records import atomic_json, capture_sources, run_record, save_review

A = HOME / 'sweepick_261013_receive_alignment_restore'
PYTHON = HOME / 'DAPIER/so101_imitation_learning/.venv/bin/python'
SCOPE = 'Fresh LEFT PICK/LIFT/CARRY/RIGHT PREGRASP → 비접촉 관측 → 조건이 충족된 put-back/home/OFF'
RIGHT_X_LABEL = 'Fresh LEFT PICK/LIFT/CARRY/RIGHT PREGRASP → T 관측 → RIGHT X F·V(이동마다 운영자 결정) → 기존 정상 역경로 → put-back/home/OFF'
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
# RIGHT_X_FINITE_METROLOGY: the existing chain, then T/F/V right X endpoints, then the existing normal return. Its own approval,
# manifest and tested native SHA; every F/V move is its own request digest and its own operator decision.
RIGHT_X = 'RIGHT_X_FINITE_METROLOGY'
RIGHT_X_MOVE = 'RIGHT_X_FINITE_METROLOGY_MOVE'
VIEW_MOVE = 'RIGHT_RECEIVE_NONCONTACT_OBSERVATION_MOVE'
RIGHT_X_BACKEND_MARKERS = {'RUN_APPROVED_SESSION.py': ("RIGHT_X = 'RIGHT_X_FINITE_METROLOGY'", 'rx.approval_errors(approval, out)', 'scope=rx.MOVE_SCOPE)'),
                           'revision/right_x_metrology.py': ("SCOPE = 'RIGHT_X_FINITE_METROLOGY'", "MOVE_SCOPE = 'RIGHT_X_FINITE_METROLOGY_MOVE'",
                                                             'def approval_errors(', 'DECLARED_WRITES = (', 'EXCLUDED = ('),
                           'revision/sweepick_full_chain.py': ('def run_right_x_metrology(', 'RIGHT_X_METROLOGY.json')}
RIGHT_X_BOUND_SOURCES = LIMITED_BOUND_SOURCES + ('revision/right_x_metrology.py', 'revision/current_view_candidate.py')
RIGHT_X_T_POSES = ('AFTER_RIGHT_PREGRASP', 'AFTER_ADOPTED_VIEW')
# P0-2: a finite operator input is not a physical permission. These are the facts only the field can confirm.
FIELD_REQUIRED = 'FIELD_PHYSICAL_CONFIRMATION_REQUIRED'
FIELD_ITEMS = ('direction_observed', 'start_pose_observed', 'noncontact_space_observed')


TESTED_NATIVE = Path(__file__).resolve().parent / 'CONSOLE17_TESTED_NATIVE_SHA256.json'
RIGHT_X_TESTED_NATIVE = Path(__file__).resolve().parent / 'RIGHT_X_TESTED_NATIVE_SHA256.json'


def native_source_check(names=None, record=None):
    """The active native files against the SHA of the source that passed the Fake regression. Read-only.
    Default: the Console-17 record and files; RIGHT_X passes its own."""
    names, record = names or LIMITED_BOUND_SOURCES, record or TESTED_NATIVE
    try:
        tested = read_json(record).get('files') or {}
    except (OSError, ValueError, AttributeError):
        tested = {}
    rows = {}
    for name in names:
        path = A / name
        active = hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None
        rows[name] = {'active_sha256': active, 'tested_sha256': tested.get(name), 'match': bool(active and active == tested.get(name))}
    return {'tested_record': str(record), 'tested_record_present': bool(tested), 'files': rows,
            'all_match': bool(tested) and all(row['match'] for row in rows.values())}


def right_x_source_check():
    return native_source_check(RIGHT_X_BOUND_SOURCES, RIGHT_X_TESTED_NATIVE)


def right_x_contract():
    """DECLARED_WRITES / EXCLUDED / scopes of the native right_x_metrology.py, read as literals (never imported)."""
    tree = ast.parse((A / 'revision/right_x_metrology.py').read_text())
    values = {}
    for node in tree.body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name) \
                and node.targets[0].id in ('SCOPE', 'MOVE_SCOPE', 'DECLARED_WRITES', 'EXCLUDED'):
            values[node.targets[0].id] = ast.literal_eval(node.value)
    if values.get('SCOPE') != RIGHT_X or values.get('MOVE_SCOPE') != RIGHT_X_MOVE \
            or not values.get('DECLARED_WRITES') or not values.get('EXCLUDED'):
        raise ValueError('right_x_metrology.py does not state the RIGHT_X scope/write contract')
    return {'declared_motor_writes': list(values['DECLARED_WRITES']), 'excluded': list(values['EXCLUDED'])}


def right_x_backend():
    """(supported, reason) for RIGHT_X_FINITE_METROLOGY: contract markers, readable write list AND active native SHA = tested source."""
    missing = []
    for name, markers in RIGHT_X_BACKEND_MARKERS.items():
        try:
            text = (A / name).read_text(errors='replace')
        except OSError:
            missing.append(f'{name}: not readable')
            continue
        missing.extend(f'{name}: {marker}' for marker in markers if marker not in text)
    if missing:
        return False, 'native 실행본에 RIGHT_X 측정 계약이 없음: ' + '; '.join(missing)
    try:
        right_x_contract()
    except (OSError, ValueError, SyntaxError) as exc:
        return False, f'native RIGHT_X write 목록을 읽을 수 없음: {exc}'
    check = right_x_source_check()
    if not check['all_match']:
        differs = [name for name, row in check['files'].items() if not row['match']]
        return False, ('RIGHT_X 시험 기록이 없음' if not check['tested_record_present'] else '활성 native SHA가 시험한 소스와 다름: ' + ', '.join(differs))
    return True, 'native RIGHT_X 계약 표식 + 활성 native SHA = 시험한 소스 SHA'


def finite_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def right_x_config(inputs):
    """The five operator inputs -> the native right_x_metrology object. Mirrors the native config check (which re-checks at
    launch). No default, recommended value or limit: a well-formed input is not a physical permission."""
    if not isinstance(inputs, dict):
        raise ControlError('RIGHT_X 운영자 입력 5개가 필요함')
    problems = []
    extra = set(inputs) - {'t_pose', 'x_basis_tool', 'x_basis_source', 'fit_displacement_m', 'verify_displacement_m'}
    if extra:
        problems.append(f'알 수 없는 입력 {sorted(extra)}')
    if inputs.get('t_pose') not in RIGHT_X_T_POSES:
        problems.append(f't_pose는 {RIGHT_X_T_POSES} 중 하나')
    basis = inputs.get('x_basis_tool')
    if not (isinstance(basis, list) and len(basis) == 3 and all(finite_number(v) for v in basis)):
        problems.append('x_basis_tool은 유한한 숫자 3개(right_cube_grasp CURRENT_TOOL 좌표)')
    elif not math.isclose(math.sqrt(sum(v * v for v in basis)), 1.0, rel_tol=0, abs_tol=1e-9):
        problems.append('x_basis_tool은 단위 벡터(방향만; 크기는 변위가 정함)')
    source = inputs.get('x_basis_source')
    if not isinstance(source, str) or not source.strip():
        problems.append('x_basis_source(축 근거 문장)가 필요함')
    for key in ('fit_displacement_m', 'verify_displacement_m'):
        if not finite_number(inputs.get(key)) or inputs.get(key) == 0:
            problems.append(f'{key}는 운영자가 정한 0이 아닌 유한한 부호 있는 m 값')
    if problems:
        raise ControlError('RIGHT_X 입력 오류: ' + '; '.join(problems))
    return {'t_pose': inputs['t_pose'], 'x_basis_tool': [float(v) for v in basis], 'x_basis_source': source.strip(),
            'fit_displacement_m': float(inputs['fit_displacement_m']), 'verify_displacement_m': float(inputs['verify_displacement_m']),
            'verify_from': 'F'}


def right_x_physical_basis(cfg):
    """P0-2: what each part of the plan rests on. The operator's numbers and the A model are not field evidence."""
    return {'status': FIELD_REQUIRED, 'operator_displacement_is_physical_permission': False,
            'direction': {'operator_stated': cfg['x_basis_source'], 'model': 'A model tool frame (candidate mapping, verified=false)',
                          'field_confirmed': False},
            'start_pose': {'model': 'each move is planned from the latest bus read at request time', 'field_confirmed': False},
            'noncontact_space': {'model': 'A model collision check with the held SEARCH_PRIOR; not a measured clearance', 'field_confirmed': False},
            'offline_probe_values': 'the ±5/10/20 mm offline probes covered two saved start states only; not a recommended value or a limit',
            'confirmed_by': 'operator field observation at each F/V request (direction, start pose, non-contact space)'}


def right_x_request_summary(proposed):
    """Display facts of one RIGHT_X move request (P0-1): plan start actual, approved displacement, target, collision, return."""
    candidate = proposed.get('candidate') if isinstance(proposed.get('candidate'), dict) else {}
    plan = proposed.get('plan') if isinstance(proposed.get('plan'), dict) else {}
    forward = plan.get('forward') if isinstance(plan.get('forward'), list) else []
    step = forward[0] if len(forward) == 1 and isinstance(forward[0], dict) else {}
    actual = proposed.get('actual_feedback') if isinstance(proposed.get('actual_feedback'), dict) else {}
    ticks = actual.get('ticks') if isinstance(actual.get('ticks'), dict) else {}
    start, target = step.get('start_ticks') or {}, step.get('target_ticks') or {}
    ident = str(candidate.get('id') or '')
    role = ident.rsplit('_', 1)[-1] if ident.startswith('RIGHT_X_METROLOGY_') and ident[-1:] in ('F', 'V') else None
    try:
        delta = {name: float(target[name]) - float(start[name]) for name in target}
        from_actual = bool(start) and isinstance(ticks.get('right'), dict) and all(
            round(float(start[name])) == round(float(ticks['right'][name])) for name in start)
    except (KeyError, TypeError, ValueError):
        delta, from_actual = None, False
    cert = step.get('certificate') if isinstance(step.get('certificate'), dict) else {}
    native = step.get('native_validation') if isinstance(step.get('native_validation'), dict) else {}
    ret = plan.get('normal_return') if isinstance(plan.get('normal_return'), dict) else {}
    return {'role': role, 'candidate_id': ident or None,
            'commanded_tool_displacement_m': candidate.get('commanded_tool_displacement_m'),
            'x_basis_tool': candidate.get('x_basis_tool'), 'x_basis_source': candidate.get('x_basis_source'),
            'planned_from': {'bus_seq': actual.get('bus_seq'), 't_bus_monotonic': actual.get('t_bus_monotonic'),
                             'right_ticks': ticks.get('right'), 'left_ticks': ticks.get('left'), 'plan_start_ticks': start or None},
            'plan_start_equals_request_actual': from_actual, 'target_ticks': target or None, 'tick_change': delta,
            'plan_ready': plan.get('ready'), 'step_count': len(forward), 'why': plan.get('why'),
            'collision': cert.get('nominal_verdict'), 'first_contact': cert.get('first_nominal_contact'),
            'native_executable': native.get('executable'), 'native_hard': native.get('hard'), 'native_provisional': native.get('provisional'),
            'normal_return_ready': ret.get('ready'), 'normal_return_why': ret.get('why'),
            'physical_status': FIELD_REQUIRED,
            'evidence': {'direction': 'OPERATOR_STATED + A_MODEL', 'start_pose': 'BUS_ACTUAL_AT_REQUEST', 'noncontact_space': 'A_MODEL_ONLY',
                         'field': 'NOT_CONFIRMED until the operator records it for this request'}}


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


class DeviceCheckError(ControlError):
    """The real device nodes could not be confirmed (alias missing/wrong, route changed, occupancy unknown)."""


WRIST_NAMES = ('left_wrist', 'right_wrist')


IDENTITY = 'WRIST_IDENTITY_CONFIRMATION.json'                      # written by wrist_cover_check.py after an operator cover test


def read_text(path):
    try:
        return Path(path).read_text().strip()
    except OSError:
        return None


def resolve_devices(plan, pairing, *, realpath=os.path.realpath, exists=os.path.exists, alias_root='/dev/dapier', sysfs_root='/sys/class/video4linux',
                    tty_root='/sys/class/tty', read=read_text, identity=None):
    """{role: canonical device node} for the arms, the top camera and both wrists, from the native plan and the
    operator-confirmed wrist pairing. Every wrist alias must resolve to its paired capture node on its paired USB route;
    anything else raises DeviceCheckError. Nothing is opened."""
    problems, nodes = [], {}
    routes = plan['device_routes']
    for role, route in routes.items():
        if role in WRIST_NAMES:
            continue
        if not exists(route):
            problems.append(f'{role}: {route} missing')
        else:
            nodes[role] = realpath(route)
    # arms: the alias must resolve to the USB adapter whose serial the pairing records for that arm (port/hub independent)
    for side in ('left', 'right'):
        expected = (pairing.get(f'{side}_wrist') or {}).get('paired_arm_serial')
        node = nodes.get(f'{side}_arm')
        if node and expected:
            serial = read(f"{realpath(f'{tty_root}/{Path(node).name}/device')}/../serial")
            if serial != expected:
                problems.append(f'{side}_arm: {node} has USB serial {serial}, the {side} arm is {expected}')
    for name in WRIST_NAMES:
        route, paired = routes.get(name), (pairing.get(name) or {})
        if paired.get('device') != route:
            problems.append(f'{name}: plan route {route} differs from the confirmed pairing {paired.get("device")}'); continue
        if not exists(route):
            problems.append(f'{name}: paired capture node {route} missing'); continue
        canonical = realpath(route); nodes[name] = canonical
        alias = f'{alias_root}/{name}_rgb'
        if not exists(alias):
            problems.append(f'{name}: alias {alias} missing')
        elif realpath(alias) != canonical:
            problems.append(f'{name}: alias {alias} -> {realpath(alias)}, paired node is {canonical}')
        usb = (paired.get('connection') or {}).get('usb_sysfs')
        device = realpath(f'{sysfs_root}/{Path(canonical).name}/device')
        if not usb or not device.startswith(realpath(usb).rstrip('/') + '/'):
            problems.append(f'{name}: {canonical} is not on the confirmed USB route {usb}')
    if nodes.get('left_wrist') and nodes.get('left_wrist') == nodes.get('right_wrist'):
        problems.append('both wrists resolve to the same capture node')
    # identical cameras without a serial are told apart by USB port only. The operator cover test confirms which port
    # carries which camera; it holds while the routes stay the same (replug into the same, marked port or a reboot keeps
    # it). A route change (port/hub move, discover) needs a new cover test. A swap of the two plugs between the same two
    # ports cannot be seen in software: the cable/port markings are the guard against that.
    if identity is not None:
        confirmed = (identity or {}).get('wrists') or {}
        if not confirmed:
            problems.append('wrist camera identity not confirmed by an operator cover test (run wrist_cover_check.py)')
        else:
            for name in WRIST_NAMES:
                usb = (pairing.get(name) or {}).get('connection', {}).get('usb_sysfs')
                row = confirmed.get(name) or {}
                if row.get('usb_sysfs') != usb:
                    problems.append(f'{name}: USB route {usb} differs from the last cover test ({row.get("usb_sysfs")}); run wrist_cover_check.py')
    if problems:
        raise DeviceCheckError('장치 경로 확인 실패: ' + '; '.join(problems))
    return nodes


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
                event_error = {'gate': stop_gate(detail), 'stop_event': 'owner_stop',
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
                    # an earlier decision (another digest, e.g. the F approval while V waits) never answers this request
                    if answered.get('request_sha256') != digest:
                        pending = {key: proposed.get(key) for key in ('scope', 'plan', 'candidate')}
                        pending['request_sha256'] = digest
                        allowed = (VIEW_MOVE, RIGHT_X_MOVE) if job.get('execution_scope') == RIGHT_X else (VIEW_MOVE,)
                        if proposed.get('session_out') != job['session_out'] or proposed.get('scope') not in allowed:
                            pending['approval_refused'] = '요청의 session/scope가 이 작업과 다름 · 거절만 가능'
                        if proposed.get('scope') == RIGHT_X_MOVE:
                            pending['right_x'] = right_x_request_summary(proposed)
                        job['pending_view'] = pending
                except (OSError, ValueError, AttributeError):
                    job['pending_view_state'] = 'UNKNOWN · 요청 파일 수신 중'
            if job.get('execution_scope') == RIGHT_X:
                try:
                    measured = read_json(Path(job['session_out']) / 'RIGHT_X_METROLOGY.json')
                    job['right_x_record'] = {'status': measured.get('status'), 'why': measured.get('why'),
                                             'endpoints': [e.get('role') for e in measured.get('endpoints') or [] if isinstance(e, dict)],
                                             'moves': [{k: m.get(k) for k in ('role', 'plan_ready', 'approved', 'result')}
                                                       for m in measured.get('moves') or [] if isinstance(m, dict)]} if measured else None
                except (OSError, ValueError, AttributeError):
                    job['right_x_record'] = {'status': 'UNKNOWN', 'why': 'RIGHT_X_METROLOGY.json unreadable'}
        if alive and not ours:
            state, reason = 'BLOCKED', '기존 managed owner가 존재함'
        recovery = self.recovery_confirmation(job) if job and state in ('FAULT', 'LIMITED_HOLDING') else None
        if job:
            job['operator_recovery'] = recovery
        if recovery:
            reason = (reason or '원래 실패 기록 보존') + ' · 수동 복구 처리 확인됨; 새 준비와 이번 현장 승인 후 fresh 상태로 재검사'
        label = RIGHT_X_LABEL if (job or {}).get('execution_scope') == RIGHT_X else SCOPE
        return {'csrf_token': self.csrf_token, 'execution_connected': True, 'state': state, 'scope': label, 'owner_alive': ours,
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
        if execution_scope == RIGHT_X:
            supported, reason = right_x_backend()
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
        except (OSError, ValueError, KeyError, TypeError, DeviceCheckError) as exc:
            occupancy, query_error = 'UNKNOWN', f'{type(exc).__name__}: {exc}'
        sources = []
        for name in ('RUN_APPROVED_SESSION.py', 'revision/sweepick_pick01.py', 'revision/sweepick_full_chain.py',
                     'revision/sweepick_move_a.py', 'revision/sweepick_episode.py'):
            path = A / name
            sources.append({'file': name, 'sha256': hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else 'UNKNOWN',
                            'bytes': path.stat().st_size if path.is_file() else None})
        launcher_available = (A / 'RUN_APPROVED_SESSION.py').is_file()
        limited_ok, limited_reason = limited_backend()
        right_x_ok, right_x_reason = right_x_backend()
        try:
            right_x_writes = right_x_contract()
        except (OSError, ValueError, SyntaxError):
            right_x_writes = {'declared_motor_writes': [], 'excluded': []}
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
                                 '다음 실행은 수동 복구 확인 전 차단. LIFT 전 기존 거부/복귀 조건은 그대로.')},
                {'id': RIGHT_X, 'label': '오른팔 X축 T/F/V 비접촉 측정', 'supported': right_x_ok,
                 'backend_status': 'SUPPORTED_RIGHT_X_METROLOGY_BACKEND' if right_x_ok else 'BACKEND_CHANGE_REQUIRED',
                 'can_prepare': right_x_ok and status['can_prepare'] and occupancy == {},
                 'included_stages': right_x_writes['declared_motor_writes'], 'excluded_stages': right_x_writes['excluded'],
                 'operator_inputs': [
                     {'key': 't_pose', 'choices': list(RIGHT_X_T_POSES), 'default': None},
                     {'key': 'x_basis_tool', 'kind': 'unit vector, right_cube_grasp CURRENT_TOOL frame', 'default': None},
                     {'key': 'x_basis_source', 'kind': 'evidence sentence for the axis', 'default': None},
                     {'key': 'fit_displacement_m', 'kind': 'signed non-zero metres (F)', 'default': None, 'limit': None},
                     {'key': 'verify_displacement_m', 'kind': 'signed non-zero metres (V, from the measured F endpoint)', 'default': None, 'limit': None}],
                 'physical_basis': {'status': FIELD_REQUIRED, 'operator_displacement_is_physical_permission': False,
                                    'offline_probe_values': '±5/10/20 mm 오프라인 계획값은 권장값·안전 한계가 아님'},
                 'reason': right_x_reason + ' · 기존 Full-chain·LEFT_START_TO_LIFT 승인과 START 사전 승인은 쓰지 않습니다. 이 범위 전용 승인·Manifest와 F/V 이동마다 별도 결정이 필요합니다.',
                 'termination': ('T·F·V 끝점 기록 뒤(또는 계획 불가·거절·관측 누락 시 그 자리에서) 기존 정상 역경로 → put-back/home/OFF. '
                                 'STOP·Servo/Bus fault·왼팔 보유 미확인은 기존 중단 계약: 자동 복귀 없음, 운영자 수동 회수.')}],
            'owner': {'state': status['state'], 'owner_alive': status['owner_alive'],
                      'active_run_id': (status['active'] or {}).get('id'), 'device_occupancy': occupancy,
                      'device_query_error': query_error}, 'sources': sources,
            'observation': {'status': 'NOT_CHECKED', 'basis': '새 reader/관절 조회 없음. 기존 owner의 실측 화면과 시각을 별도로 확인합니다.'},
            'calibration': {'status': 'NOT_CHECKED', 'basis': '장치 identity/EEPROM 및 Calibration 실측 대조 미수행. 기존 실행기 검사는 유지합니다.'},
            'limited_source_check': native_source_check(),
            'right_x_source_check': right_x_source_check(),
            'device_access': False}

    def auto_prepare_next(self):
        """After a normal FULL_CHAIN end (no error, native ledger: both arms RETURNED_RELEASED with torque/status 0, no
        operator STOP), prepare the next session once so the operator only re-checks the table and STARTs. The new job is
        unapproved like any prepared job; nothing is started. FAULT, LIMITED_HOLDING and stopped runs keep the manual path."""
        status = self.status()
        job = status['active']
        if (status['state'] != 'ENDED' or status['owner_alive'] or not status['can_prepare'] or not job or not job.get('consumed')
                or job.get('execution_scope', FULL_CHAIN) != FULL_CHAIN or job.get('stop_sent') or job.get('auto_next')):
            return status
        try:
            self.prepare(job.get('source_session'), execution_scope=FULL_CHAIN, auto_after=job['id'])
        except (ControlError, DeviceCheckError, OSError, ValueError, KeyError) as exc:
            with self.locked():
                current = read_json(self.state_file) or {}
                if current.get('id') == job['id'] and not current.get('auto_next'):
                    current['auto_next'] = {'prepared': None, 'error': f'{type(exc).__name__}: {exc}',
                                            'at': datetime.now().astimezone().isoformat()}
                    self.save(current)
        return self.status()

    def prepare(self, source_session=None, *, execution_scope=FULL_CHAIN, auto_after=None, right_x=None):
        self.require_execution_scope(execution_scope)
        if execution_scope == RIGHT_X:
            if source_session is not None or auto_after is not None:
                raise ControlError('RIGHT_X 측정은 현재 입력의 새 세션으로만, 운영자 입력과 함께 준비함')
            right_x = right_x_config(right_x)
        elif right_x is not None:
            raise ControlError('RIGHT_X 입력은 RIGHT_X_FINITE_METROLOGY 범위에만 해당')
        if source_session is not None and (not isinstance(source_session, str)
                or (source_session not in ROOTS and not self.run_directory(source_session))):
            raise ControlError('알 수 없는 저장 세션')
        with self.locked():
            status = self.status()
            if not status['can_prepare']:
                raise ControlError(status['block_reason'] or '동시 세션 준비/실행 불가')
            if auto_after is not None and ((status['active'] or {}).get('id') != auto_after or status['state'] != 'ENDED'
                                           or (status['active'] or {}).get('auto_next')):
                raise ControlError('다음 세션이 이미 준비됐거나 직전 실행이 정상 종료가 아님')
            occupied = self.owners()                                     # raises DeviceCheckError on an unconfirmed node/alias/route
            if occupied:
                raise ControlError('장치 owner가 이미 존재함: ' + str(occupied))
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
            measuring = execution_scope == RIGHT_X
            if limited:                                                  # this scope binds the current launcher/executors explicitly
                files.extend(A / name for name in LIMITED_BOUND_SOURCES)
            if measuring:
                files.extend(A / name for name in RIGHT_X_BOUND_SOURCES)
            prepared = {'scope': execution_scope if limited or measuring else 'RIGHT_RECEIVE_NONCONTACT_OBSERVATION', 'files': {
                str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in dict.fromkeys(files)}}
            if limited or measuring:
                prepared['execution_scope'] = execution_scope
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
            elif measuring:                                              # never derived from the full-chain approval
                contract = right_x_contract()
                approval = {'scope': RIGHT_X, 'permission_detail': 'One supervised fresh session: the existing LEFT PICK/LIFT/CARRY and RIGHT '
                            'approach, then the T observation and the right ARM5 X moves F and V, each by its own operator decision, then the '
                            'existing normal reverse, put-back, home and Torque OFF. No right gripper command, insertion, close or left release. '
                            'No automatic next session.',
                            'declared_motor_writes': contract['declared_motor_writes'], 'excluded': contract['excluded'],
                            'right_gripper_commands_permitted': False, 'insertion_permitted': False, 'close_permitted': False,
                            'left_release_permitted': False, 'right_x_metrology': right_x,
                            'right_x_physical_basis': right_x_physical_basis(right_x), 'writes_acknowledged': False,
                            'permission_to_open_devices': False, 'unchanged_wrist_mount_and_routes': False, 'policy_training_allowed': False}
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
            if auto_after is not None:
                job['auto_prepared_after'] = auto_after
            if measuring:
                job['right_x_metrology'] = right_x
                job['right_x_physical_basis'] = approval['right_x_physical_basis']
                job['right_x_declared_motor_writes'], job['right_x_excluded'] = approval['declared_motor_writes'], approval['excluded']
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
        """{canonical node: pids} of the real device nodes. Raises DeviceCheckError when a node, alias or route cannot be
        confirmed or fuser cannot answer; that is never reported as {} (free)."""
        plan = read_json(A / 'RECEIVE_FIELD_PLAN.json')
        pairing = read_json(A / 'WRIST_PAIRING.json').get('wrists') or {}
        try:
            identity = read_json(A / IDENTITY)
        except (OSError, ValueError):
            identity = {}
        paths = set(resolve_devices(plan, pairing, identity=identity or {}).values())
        top_sysfs = Path(plan['accepted_usb_topology']['top_usb_sysfs'])
        if (top_sysfs / 'busnum').exists() and (top_sysfs / 'devnum').exists():
            paths.add(f'/dev/bus/usb/{int((top_sysfs / "busnum").read_text()):03d}/{int((top_sysfs / "devnum").read_text()):03d}')
        owners = {}
        for path in sorted(paths):
            result = self.run(['fuser', path], capture_output=True, text=True)
            if result.returncode not in (0, 1) or (result.returncode == 1 and (result.stderr or '').strip()):
                raise DeviceCheckError(f'점유 확인 불가: fuser {path} rc={result.returncode} {(result.stderr or "").strip()}')
            if result.stdout.strip():
                owners[path] = result.stdout.strip()
        return owners

    def start(self, job_id, *, field_ready=False, mounts_unchanged=False, supervised=False, execution_scope=None, preapprove_view=False,
              writes_acknowledged=False):
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
            expected = scope if scope in (LEFT_TO_LIFT, RIGHT_X) else 'RIGHT_RECEIVE_NONCONTACT_OBSERVATION'
            prepared_manifest = read_json(directory / 'MANIFEST.json') or {}
            manifest_scope = prepared_manifest.get('scope')
            if approval.get('scope') != expected or approval.get('execution_scope', FULL_CHAIN) != scope or manifest_scope != expected \
                    or (scope == RIGHT_X and prepared_manifest.get('execution_scope') != RIGHT_X):
                raise ControlError('준비된 승인/Manifest 범위가 실행 범위와 다름; START하지 않음')
            if scope in (LEFT_TO_LIFT, RIGHT_X):                         # the approved manifest must bind exactly the tested native bytes
                tested = (native_source_check() if scope == LEFT_TO_LIFT else right_x_source_check())['files']
                bound = prepared_manifest.get('files') or {}
                if any(bound.get(str(A / name)) != row['tested_sha256'] for name, row in tested.items()):
                    raise ControlError('승인 Manifest가 시험한 native 소스 SHA와 다름; START하지 않음')
            if scope == RIGHT_X:
                if writes_acknowledged is not True:
                    raise ControlError('RIGHT_X: 전체 조작 단계와 제외 범위 확인 필요; START하지 않음')
                if preapprove_view is True:
                    raise ControlError('START 사전 승인은 RIGHT_X_FINITE_METROLOGY에 적용하지 않음; F/V는 이동마다 결정')
                contract = right_x_contract()
                if approval.get('declared_motor_writes') != contract['declared_motor_writes'] or approval.get('excluded') != contract['excluded']:
                    raise ControlError('준비된 write/제외 목록이 native 계약과 다름; START하지 않음')
                if right_x_config({k: v for k, v in (approval.get('right_x_metrology') or {}).items() if k != 'verify_from'}) \
                        != approval.get('right_x_metrology') or approval.get('right_x_metrology') != job.get('right_x_metrology'):
                    raise ControlError('준비된 RIGHT_X 입력이 작업 기록과 다름; START하지 않음')
                approval['writes_acknowledged'] = True
            if preapprove_view is True:
                if scope != FULL_CHAIN:
                    raise ControlError('관측 이동 사전 승인은 FULL_CHAIN_RECEIVE_CONTACT의 비접촉 관측에만 해당')
                # one explicit operator choice at START: the single existing non-contact view candidate, bound to this
                # session and prepared manifest; the launcher re-checks every fact of the generated path before using it
                approval['observation_view_preapproval'] = {
                    'approved': True, 'scope': 'RIGHT_RECEIVE_NONCONTACT_OBSERVATION_MOVE', 'candidate_id': 'CURRENT_RECEIVER_CLEAR_THEN_INNER_PAD_VIEW',
                    'candidate_count': 1, 'purpose': 'NONCONTACT_INNER_PAD_VIEW', 'insertion_permitted': False, 'close_permitted': False,
                    'session_out': job['session_out'], 'prepared_manifest_sha256': approval.get('prepared_manifest_sha256'),
                    'operator_source': 'local_console_start_option', 'granted_at': datetime.now().astimezone().isoformat(),
                    'requires': ['same session and manifest', 'candidate kind', 'planned from current measured ticks', 'every step certified CLEAR',
                                 'right arm only', 'Status 0 on both arms', 'current LEFT support/bus confirmed', 'single use']}
            approval.update(approved=True, field_ready=True, permission_to_execute=True, permission_to_open_devices=True,
                            unchanged_wrist_mount_and_routes=True, timestamp=datetime.now().astimezone().isoformat(),
                            field_confirmation='operator START click: setup/routes/support/supervision confirmed')
            (directory / 'APPROVAL.json').write_text(json.dumps(approval, ensure_ascii=False, indent=2))
            job.update(state='STARTING', consumed=True, phase='launch_requested', observation_view_preapproved=preapprove_view is True)
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

    def view_decision(self, job_id, digest, approved, field_confirmation=None):
        with self.locked():
            job = self.selected(job_id)
            pending = (self.status()['active'] or {}).get('pending_view')
            if not pending or pending['request_sha256'] != digest or not isinstance(approved, bool):
                raise ControlError('현재 actual 기반 관측 이동 요청과 일치하지 않음')
            if approved and pending.get('approval_refused'):
                raise ControlError(pending['approval_refused'])
            record = {'approved': approved, 'request_sha256': digest, 'session_out': job['session_out'],
                      'scope': pending['scope'], 'operator_source': 'local_console_explicit_click'}
            if pending['scope'] == RIGHT_X_MOVE:
                summary = pending.get('right_x') or {}
                record['role'] = summary.get('role')
                record['model_plan_basis'] = {k: summary.get(k) for k in ('commanded_tool_displacement_m', 'x_basis_tool', 'collision',
                                                                           'native_executable', 'normal_return_ready', 'plan_start_equals_request_actual')}
                record['operator_displacement_is_physical_permission'] = False
                if approved:
                    # P0-2: the approval of this one move rests on what the operator saw in the field now, not on the numbers
                    fc = field_confirmation if isinstance(field_confirmation, dict) else {}
                    note = fc.get('direction_note')
                    missing = [k for k in FIELD_ITEMS if fc.get(k) is not True] + ([] if isinstance(note, str) and note.strip() else ['direction_note'])
                    if missing:
                        raise ControlError(f'{FIELD_REQUIRED}: 이 요청의 현장 확인 없음 ({", ".join(missing)}); 승인하지 않음')
                    if summary.get('plan_ready') is not True or summary.get('step_count') != 1 or summary.get('collision') != 'CLEAR' \
                            or summary.get('native_executable') is not True or summary.get('normal_return_ready') is not True \
                            or summary.get('plan_start_equals_request_actual') is not True or summary.get('role') not in ('F', 'V'):
                        raise ControlError('요청 계획이 1개 X 목표·충돌 CLEAR·native 실행 가능·정상 역경로·실측 시작을 모두 갖추지 않음; 승인하지 않음')
                    record['field_physical_confirmation'] = {
                        **{k: True for k in FIELD_ITEMS}, 'direction_note': note.strip(),
                        'source': 'local_operator_field_observation', 'at': datetime.now().astimezone().isoformat(),
                        'scope': 'this request digest only'}
            path = Path(job['session_out']) / 'OBSERVATION_MOVE_APPROVAL.json'
            temporary = path.with_suffix('.tmp')
            temporary.write_text(json.dumps(record))
            temporary.replace(path)
        return self.status()
