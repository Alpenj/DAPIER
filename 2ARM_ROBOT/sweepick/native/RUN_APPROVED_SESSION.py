"""Launch the prepared single A session through the existing recorder/owner/finite executor."""
import hashlib
import json
import os
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

HERE = Path(__file__).resolve().parent
PLAN = HERE / 'RECEIVE_FIELD_PLAN.json'
APPROVAL = Path(os.environ.get('SWEEPICK_START_APPROVAL_PATH', HERE / 'NONCONTACT_START_APPROVAL.json'))
MANIFEST = Path(os.environ.get('SWEEPICK_SOURCE_MANIFEST_PATH', HERE / 'PREPARED_SOURCE_MANIFEST.json'))
REV = HERE / 'revision'
TMUX_SESSION = os.environ.get('SWEEPICK_MANAGED_SESSION', 'sweepick_right_receive_observation_09')
os.environ['SWEEPICK_RECEIVE_DIR'] = str(HERE / 'pin')
sys.path[:0] = [str(REV), str(REV.parent)]

FULL_CHAIN = 'FULL_CHAIN_RECEIVE_CONTACT'
LEFT_TO_LIFT = 'LEFT_START_TO_LIFT'
# Console-17: the limited scope has its own approval scope and manifest scope; a full-chain approval never opens it.
LIMITED_APPROVAL_SCOPE = 'LEFT_START_TO_LIFT'

_LIFETIME = None
_OUT = None
_PHASE = 'not_started'
_STOP_SIGNAL = None


def append_jsonl(path, record):
    """Append one complete durable record without replacing partial history."""
    data = (json.dumps(record, sort_keys=True, default=str) + '\n').encode()
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o644)
    try:
        while data:
            data = data[os.write(fd, data):]
        os.fsync(fd)
    finally:
        os.close(fd)


def lifecycle(event, phase=None, **fields):
    if _LIFETIME is None:
        return
    append_jsonl(_LIFETIME, dict(timestamp=datetime.now(timezone.utc).isoformat(), event=event,
                                phase=phase or _PHASE, attempt=1, pid=os.getpid(), ppid=os.getppid(),
                                pgid=os.getpgid(0), **fields))


def set_phase(phase, event='phase_start', **fields):
    global _PHASE
    _PHASE = phase
    lifecycle(event, phase, **fields)


def handle_signal(signum, _frame):
    global _STOP_SIGNAL
    _STOP_SIGNAL = signum
    if _OUT is not None:
        Path(str(_OUT) + '.STOP').touch(exist_ok=True)
    lifecycle('signal', signal=signal.Signals(signum).name, sender='UNKNOWN', stop_requested=True)


def stop_checkpoint():
    if _STOP_SIGNAL is not None:
        raise RuntimeError(f'{signal.Signals(_STOP_SIGNAL).name} requested STOP during {_PHASE}')


def authorize_observation_plan(owner, proposed, raw, candidate):
    """One operator decision on this actual-bound plan; no movement while waiting."""
    proposal = dict(scope='RIGHT_RECEIVE_NONCONTACT_OBSERVATION_MOVE', session_out=str(owner.out),
                    candidate=candidate, actual_feedback=raw, plan=proposed,
                    insertion_permitted=False, close_permitted=False,
                    wait_policy='operator decision with fresh support/bus/STOP monitoring',
                    issued_monotonic_s=owner.clock())
    content = json.dumps(proposal, sort_keys=True, default=str).encode()
    digest = hashlib.sha256(content).hexdigest()
    request = owner.out / 'OBSERVATION_MOVE_REQUEST.json'
    request.write_bytes(content)
    owner.note('awaiting_observation_move_approval', request=str(request), sha256=digest)
    decision_path = owner.out / 'OBSERVATION_MOVE_APPROVAL.json'
    # Human approval is a supervised pause, not the software computation budget.
    cadence = json.loads((REV / 'sweepick_jaw_profile_left.json').read_text())['creep_period_s']['value']
    frame_wait_started = None
    while True:
        stop_checkpoint()
        current = owner.raw()
        supported, why = owner.recovery_ready('waiting for observation move approval', observation=current)
        if not supported:
            check = owner.last_recovery_check
            frame = check.get('left_wrist_frame') or {}
            # Match the existing bounded reader wait; a stale image never confirms support or consumes approval.
            delayed = (check.get('register_checks_ok') is True
                       and frame.get('reader_reason') == "frame exceeds the reader's measured-cadence freshness limit")
            if not delayed or (frame_wait_started is not None and owner.clock() - frame_wait_started >= owner.idle_limit_s):
                raise RuntimeError('observation approval wait lost current support/bus: ' + str(why))
            if frame_wait_started is None:
                frame_wait_started = owner.clock()
                owner.note('observation_approval_frame_wait', camera=frame, support='UNCONFIRMED',
                           budget_s=owner.idle_limit_s, basis=owner.idle_basis)
            owner.sleep(float(cadence))
            continue
        if frame_wait_started is not None:
            owner.note('observation_approval_frame_restored', waited_s=owner.clock() - frame_wait_started,
                       camera=owner.last_recovery_check['left_wrist_frame'])
            frame_wait_started = None
        if decision_path.exists():
            decision = json.loads(decision_path.read_text())
            approved = (decision.get('approved') is True and decision.get('request_sha256') == digest
                        and decision.get('session_out') == str(owner.out)
                        and decision.get('scope') == proposal['scope'])
            owner.note('observation_move_approval_consumed', approved=approved, request_sha256=digest)
            return approved
        owner.sleep(float(cadence))


def execution_scope(approval):
    """The approved execution scope. A missing field is the existing full chain; anything else must match exactly."""
    scope = approval.get('execution_scope', FULL_CHAIN)
    if scope not in (FULL_CHAIN, LEFT_TO_LIFT):
        raise RuntimeError(f'unsupported execution_scope {scope!r}; no device/SDK/motor is opened')
    return scope


def validate_limited_launch(approval, plan):
    """LEFT_START_TO_LIFT: its own approval scope and manifest scope. The RIGHT RECEIVE observation/contact plan contract is
    not a precondition of this scope; the session's own LEFT observation, grasp and collision checks are unchanged."""
    out = Path(plan['session_out'])
    if approval.get('scope') != LIMITED_APPROVAL_SCOPE or approval.get('session_out') != str(out):
        raise RuntimeError('LEFT_START_TO_LIFT approval does not match this scope/session output')
    if approval.get('right_arm_commands_permitted') is not False or approval.get('automatic_return_after_lift') is not False:
        raise RuntimeError('LEFT_START_TO_LIFT approval must state right_arm_commands_permitted=false and automatic_return_after_lift=false')
    if out.exists():
        raise RuntimeError(f'session output already exists; preserve the ended attempt: {out}')
    if Path(str(out) + '.STOP').exists():
        raise RuntimeError(f'existing session STOP latch requires an explicit new-session recovery decision: {out}.STOP')
    if approval.get('prepared_manifest_sha256') != hashlib.sha256(MANIFEST.read_bytes()).hexdigest():
        raise RuntimeError('LEFT_START_TO_LIFT approval is bound to a different prepared source manifest')
    manifest = json.loads(MANIFEST.read_text())
    if manifest.get('scope') != LIMITED_APPROVAL_SCOPE or manifest.get('execution_scope') != LEFT_TO_LIFT:
        raise RuntimeError('prepared source manifest is not a LEFT_START_TO_LIFT manifest')
    for name, digest in manifest['files'].items():
        if hashlib.sha256(Path(name).read_bytes()).hexdigest() != digest:
            raise RuntimeError(f'prepared source/profile/plan changed before launch: {name}')
    for required in (Path(__file__).resolve(), REV / 'sweepick_pick01.py', REV / 'sweepick_move_a.py'):
        if str(required) not in manifest['files']:
            raise RuntimeError(f'LEFT_START_TO_LIFT manifest does not bind {required.name}')
    return approval, plan, out, manifest


def validate_launch():
    """Validate the one-session approval and frozen inputs without importing device owners."""
    approval = json.loads(APPROVAL.read_text())
    required = ('approved', 'field_ready', 'permission_to_execute', 'permission_to_open_devices')
    missing = [name for name in required if approval.get(name) is not True]
    if missing:
        raise RuntimeError('noncontact observation approval incomplete (' + ', '.join(missing) + '); no device/SDK/motor is opened')
    plan = json.loads(PLAN.read_text())
    if os.environ.get('SWEEPICK_SESSION_OUT'):
        plan['session_out'] = os.environ['SWEEPICK_SESSION_OUT']
    if execution_scope(approval) == LEFT_TO_LIFT:
        return validate_limited_launch(approval, plan)
    if plan.get('observation_only') is not True or plan.get('observation_candidate_policy') != 'ADOPTED_RECEIVER_INNER_PAD_VIEW':
        raise RuntimeError('noncontact-only plan scope is not bound')
    out = Path(plan['session_out'])
    if approval.get('scope') != 'RIGHT_RECEIVE_NONCONTACT_OBSERVATION' or approval.get('session_out') != str(out):
        raise RuntimeError('noncontact observation approval does not match the prepared RECEIVE_CONTACT output')
    if out.exists():
        raise RuntimeError(f'session output already exists; preserve the ended attempt: {out}')
    if Path(str(out) + '.STOP').exists():
        raise RuntimeError(f'existing session STOP latch requires an explicit new-session recovery decision: {out}.STOP')
    if approval.get('prepared_manifest_sha256') != hashlib.sha256(MANIFEST.read_bytes()).hexdigest():
        raise RuntimeError('noncontact observation approval is bound to a different prepared source manifest')
    manifest = json.loads(MANIFEST.read_text())
    for name, digest in manifest['files'].items():
        if hashlib.sha256(Path(name).read_bytes()).hexdigest() != digest:
            raise RuntimeError(f'prepared source/profile/plan changed before launch: {name}')
    return approval, plan, out, manifest


def managed_tmux_owner():
    pane = os.environ.get('TMUX_PANE')
    if not os.environ.get('TMUX') or not pane:
        return False
    current = subprocess.run(['/usr/bin/tmux', 'display-message', '-p', '-t', pane, '#S'], text=True,
                             stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, check=False)
    return current.returncode == 0 and current.stdout.strip() == TMUX_SESSION


def first_right_wrist_frame(owner, fc):
    """Use the resident reader's first actual frame within the existing observation budget."""
    started = owner.clock()
    readers = owner.ep.wrists
    reader = readers.get('right_wrist') or readers.get('right')
    cadence_s = 1.0 / float(reader.info['driver_fps'])
    while True:
        stop_checkpoint()
        frames = owner._frames()
        frame = frames.get('right_wrist') or {}
        if 'frame' in frame and fc.camera_valid(frame):
            return frames
        reason = frame.get('reader_reason', 'resident right wrist has not delivered its first frame')
        if any(word in reason for word in ('error', 'ended', 'malformed')) or owner.clock()-started >= owner.idle_limit_s:
            raise RuntimeError('optical startup has no current right wrist frame: ' + reason)
        owner.sleep(cadence_s)


def preload_sources(out=None):
    # Resident readers require canonical field dependencies before the selected executor is bound.
    tools = Path.home() / 'DAPIER/.local-artifacts/sweepick-field-tools'
    sys.path.insert(0, str(tools))
    import sweepick_local_loop
    if Path(sweepick_local_loop.__file__).resolve().parent != tools:
        raise RuntimeError('canonical field readers were shadowed before any device access')
    sys.path[:0] = [str(REV), str(HERE)]
    # Canonical readers retain their own helper; the selected executor binds its revised trajectory helper explicitly.
    import importlib.util
    if Path(sys.modules['sweepick_trajectory'].__file__).resolve().parent != REV:
        spec = importlib.util.spec_from_file_location('sweepick_trajectory', REV / 'sweepick_trajectory.py')
        selected = importlib.util.module_from_spec(spec)
        sys.modules['sweepick_trajectory'] = selected
        spec.loader.exec_module(selected)
    import sweepick_trajectory
    import sweepick_move_a
    import sweepick_commission
    import sweepick_261009_offsite_relative_features
    import current_rgbd_faces
    import inner_pad_face_contract
    import native_surface_candidates
    import current_view_candidate
    import sweepick_261010_observed_robot_mask
    import sweepick_full_chain as fc
    import right_wrist_relative_geometry
    import right_wrist_optical_calibration
    import model_assisted_receive_plan
    import sweepick_pick01 as p1
    for module in (fc, p1, sweepick_commission, sweepick_move_a, sweepick_trajectory, sweepick_261009_offsite_relative_features,
                   current_rgbd_faces, inner_pad_face_contract, native_surface_candidates, current_view_candidate,
                   sweepick_261010_observed_robot_mask, right_wrist_relative_geometry, right_wrist_optical_calibration, model_assisted_receive_plan):
        if Path(module.__file__).resolve().parent != REV:
            raise RuntimeError(f'prepared revision was shadowed before any device access: {module.__file__}')
    if Path(fc.codex()["receive_entry"].__file__).resolve().parent != HERE / "pin":
        raise RuntimeError("prepared receive pin was shadowed before device access")
    if out is None:
        out = Path(json.loads(PLAN.read_text())['session_out'])
    p1.SESSION_WRIST_PAIRING = Path(out) / 'CAMERA_BINDING.json'
    return fc, p1

def main():
    global _LIFETIME, _OUT
    if sys.argv[1:] != ['--execute', '--managed-tmux']:
        print('After operator approval: START_FIELD_SESSION.py --execute')
        return 2
    if not managed_tmux_owner():
        print('NOT STARTED: physical owner requires the dedicated managed tmux session', file=sys.stderr)
        return 2
    approval, plan, out, manifest = validate_launch()
    scope = execution_scope(approval)
    out.mkdir(parents=True, exist_ok=False)  # atomic claim: an ended or concurrent attempt is never reused
    (out / 'review').mkdir()
    _OUT = out
    _LIFETIME = out / 'review/lifetime.jsonl'
    lifecycle('owner_start', 'validation')
    old_handlers = {sig: signal.signal(sig, handle_signal) for sig in (signal.SIGTERM, signal.SIGINT)}
    try:
        set_phase('preparation')
        stop_checkpoint()
        # The resident readers prepend the shared tools path; bind the selected owner before that import.
        fc, p1 = preload_sources(out)
        stop_checkpoint()
        if scope == FULL_CHAIN:                                     # RIGHT RECEIVE face geometry belongs to the full chain only
            from inner_pad_face_contract import prepare_face_geometry
            model_started = time.perf_counter(); face_model = p1.model()
            model_s = time.perf_counter() - model_started; geometry_started = time.perf_counter()
            fc.PREPARED_FACE_GEOMETRY = prepare_face_geometry(face_model, model_source_path=p1.SCENE,
                source_model_path=plan["face_geometry_source_xml"], calibration_paths=[p1.CAND, p1.CAL, p1.ASM],
                config_paths=[PLAN, MANIFEST, Path(p1.__file__), Path(fc.__file__),
                              *sorted(REV.glob("*profile*.json")), *[REV / n for n in
                              ("inner_pad_face_contract.py", "current_rgbd_faces.py", "sweepick_261010_observed_robot_mask.py")]],
                config_values=[plan.get("top_camera_reference"), plan.get("planner_options")])
            fc.PREPARED_FACE_GEOMETRY_TIMINGS = dict(model_prepare_s=model_s, cad_prepare_s=time.perf_counter() - geometry_started)
        stop_checkpoint()
        from sweepick_episode import Episode
        from sweepick_local_loop import WristCam, SdkTop
        import sweepick_local_loop as loop
        from sweepick_commission import real_bus
        from confirmed_wrist_routes import bind_wrist_readers
        binding = bind_wrist_readers(loop, HERE, out, approval)
        lifecycle('wrist_binding', **binding)
        stop_checkpoint()

        def open_readonly_bus(side):
            stop_checkpoint()
            bus = loop.f5.ReadOnlyBus(real_bus('/dev/dapier/' + side + '_arm'))
            bus.connect()
            if _STOP_SIGNAL is not None:
                bus.disconnect()
                stop_checkpoint()
            return bus

        loop.open_readonly_bus = open_readonly_bus

        def episode():
            opened = []
            try:
                wrists = {}
                for name in ('left_wrist', 'right_wrist'):
                    stop_checkpoint()
                    wrists[name] = WristCam(name, '/dev/dapier/' + name + '_rgb'); opened.append(wrists[name])
                    stop_checkpoint()
                top = SdkTop(out / f'episode/sdk_{int(time.time())}', 30 * 60 * 20); opened.append(top)
                stop_checkpoint()
                ep = Episode(out, wrists=wrists, top=top, open_arm=open_readonly_bus, mode='RECEIVE_CONTACT' if scope == FULL_CHAIN else LEFT_TO_LIFT, arm_feedback=True,
                             identifiers=dict(prepared_manifest=manifest, wrist_route_basis=plan['wrist_route_basis']))
                ep.owned = opened
                if _STOP_SIGNAL is not None:
                    ep.close(task_outcome='ENDED_BY_OPERATOR_SIGNAL_BEFORE_SESSION')
                    stop_checkpoint()
                return ep
            except Exception:
                for reader in reversed(opened):
                    reader.close()
                raise

        from bus_packet_ring import install
        original_build = fc.build
        def optical_build(*args, **kwargs):
            handoff, owner = original_build(*args, **kwargs)
            policy = plan['model_assisted_optical_contract']
            if owner.scope != 'contact' or owner.record_kind != 'REAL' or policy['source_kind'] != 'REAL':
                raise RuntimeError('the approved optical contract is for REAL RECEIVE_CONTACT only')
            camera_binding = json.loads((out / 'CAMERA_BINDING.json').read_text())
            calibration = json.loads(Path(policy['calibration_path']).read_text())
            current_mount = dict(calibration['binding'], physical_pairing=camera_binding['wrists']['right_wrist'])
            owner.last_contact_frames = first_right_wrist_frame(owner, fc)
            owner.configure_right_wrist_optical(policy['calibration_path'], current_mount)
            owner.model_assisted_live_enabled = False
            owner.authorize_observation_plan = lambda proposed, raw, candidate: authorize_observation_plan(owner, proposed, raw, candidate)
            owner.note('noncontact_current_measurement_contract', policy=policy,
                       source_role='current REAL RGB pixels + estimated pose + CAD/model priors',
                       physical_error_bound_m=None, full_release_permitted=False)
            return handoff, owner
        fc.build = optical_build
        packet_hook = sys.modules['bus_packet_ring']
        if Path(packet_hook.__file__).resolve().parent != REV:
            raise RuntimeError('packet hook revision shadowed before device access')
        ring = install(HERE / 'live_bus_diag')
        if os.environ.get('SWEEPICK_CONSOLE_FEED') == '1':
            from console_feedback import install as install_console_feed
            install_console_feed(out)
        stop_checkpoint()
        lifecycle('phase_end', phase='preparation', status='ready')
        set_phase('session')
        lifecycle('execution_scope', scope=scope)
        try:
            if scope == LEFT_TO_LIFT:
                fc.build = original_build                           # no hand-over owner is built in this scope
                code = p1.run_session(out, execute=True, record=True, full_chain=False, limit=p1.LEFT_TO_LIFT, episode_fn=episode)
            else:
                code = p1.run_session(out, execute=True, record=True, full_chain=True, scope='RECEIVE_CONTACT',
                                      contact_plan=plan, episode_fn=episode)
        finally:
            ring.flush()
        if _STOP_SIGNAL is not None and code == 0:
            code = 128 + _STOP_SIGNAL
        lifecycle('phase_end', exit_code=code, stop_requested=_STOP_SIGNAL is not None)
        set_phase('ended', 'owner_end', exit_code=code, stop_requested=_STOP_SIGNAL is not None)
        return code
    except Exception as exc:
        code = 128 + _STOP_SIGNAL if _STOP_SIGNAL is not None else 1
        set_phase('ended', 'owner_end', exit_code=code, stop_requested=_STOP_SIGNAL is not None,
                  error=f'{type(exc).__name__}: {exc}')
        if _STOP_SIGNAL is not None:
            return code
        raise
    finally:
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)

if __name__ == '__main__':
    raise SystemExit(main())
