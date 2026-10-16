'use strict';

const CAMERA_ORDER = [
  ['top_rgb', 'OS30A RGB'],
  ['top_depth', 'OS30A Depth'],
  ['left_wrist', '왼손목 RGB'],
  ['right_wrist', '오른손목 RGB'],
];
const STAGE_ORDER = ['PICK', 'LIFT', 'CARRY', 'PREGRASP', 'OBSERVE'];
const CATEGORY_LABEL = { success: '성공', failure: '실패', partial: '부분성공' };
const OWNER_ACTIVE_STATES = ['STARTING', 'RUNNING', 'STOP_REQUESTED'];
const OWNER_TERMINAL_STATES = ['ENDED', 'FAULT', 'LIMITED_HOLDING'];
const state = {
  sessions: [], session: null, snapshot: null, plan: null, prepareSource: undefined, category: 'all',
  runs: [], runBucket: 'all', runsBusy: false, runsRefreshPending: false, lastTerminalKey: null,
  time: 0, playing: false, speed: 1, timer: null, lastTick: 0,
  snapshotGeneration: 0, snapshotRevision: 0, seekRevision: 0,
  snapshotBusy: false, queuedSnapshot: null, snapshotError: null,
  snapshotController: null, planController: null,
  control: null, controlBusy: false, controlPollBusy: false, controlTimer: null,
  controlActionError: null, controlStatusError: null,
  liveMode: false, liveSnapshotBusy: false, liveSnapshotPending: false,
  liveSnapshotReceivedAt: null, liveSnapshotError: null,
  recoveryJobId: null,
  dataReviewRunId: null, dataReviewRange: null,
};

const $ = (id) => document.getElementById(id);
const display = (value) => value === null || value === undefined || value === '' ? 'UNKNOWN' : String(value);
const finiteNumber = (value) => value !== null && value !== undefined && value !== ''
  && typeof value !== 'boolean' && Number.isFinite(Number(value));
const seconds = (value) => finiteNumber(value) ? `${Number(value).toFixed(2)} s` : 'UNKNOWN';
const recordedTime = (value) => finiteNumber(value)
  ? new Intl.DateTimeFormat('ko-KR', { dateStyle: 'short', timeStyle: 'medium', hour12: false }).format(new Date(Number(value) * 1000))
  : 'UNKNOWN';
const fps = (value) => finiteNumber(value) ? `${Number(value).toFixed(Math.abs(Number(value)) >= 10 ? 1 : 2)} FPS` : display(value);
const compactSource = (value) => display(value).split('/').pop();
const jsonText = (value) => value && typeof value === 'object' && Object.keys(value).length ? JSON.stringify(value, null, 2) : '—';
const proposalText = (value) => value === null || value === undefined ? '—' : typeof value === 'string' ? value : JSON.stringify(value, null, 2);

function cameraDisplayEvidence(camera, mode, now, forceStale) {
  const rawStatus = camera?.status || 'UNKNOWN';
  const liveAge = mode === 'LIVE' && finiteNumber(camera?.last_received_s) && now >= Number(camera.last_received_s)
    ? now - Number(camera.last_received_s) : null;
  const freshnessLimit = finiteNumber(camera?.freshness_limit_s) && Number(camera.freshness_limit_s) > 0
    ? Number(camera.freshness_limit_s) : null;
  let status = rawStatus;
  if (mode === 'LIVE' && rawStatus === 'LIVE') {
    if (liveAge === null) status = 'UNKNOWN';
    else if (forceStale) status = 'STALE';
    else if (freshnessLimit === null) status = 'UNKNOWN';
    else if (liveAge > freshnessLimit) status = 'STALE';
  }
  return { rawStatus, status, liveAge, freshnessLimit };
}

function setError(message = '') {
  $('errorBanner').textContent = message;
  $('errorBanner').hidden = !message;
}

async function parseJsonResponse(response) {
  const text = await response.text();
  let data;
  try { data = JSON.parse(text); } catch { data = null; }
  if (!response.ok) {
    const detail = (data?.detail ?? text.trim()) || response.statusText;
    throw new Error(`HTTP ${response.status}: ${typeof detail === 'string' ? detail : JSON.stringify(detail)}`);
  }
  if (!data || typeof data !== 'object') throw new Error('JSON 응답 형식이 올바르지 않습니다.');
  return data;
}

async function getJson(url, signal) {
  return parseJsonResponse(await fetch(url, { signal, headers: { Accept: 'application/json' } }));
}

async function postControl(path, body) {
  const token = state.control?.csrf_token;
  if (!token) throw new Error('제어 토큰이 없습니다. 상태를 다시 불러오십시오.');
  const response = await fetch(path, {
    method: 'POST',
    headers: { Accept: 'application/json', 'Content-Type': 'application/json', 'X-Sweepick-Token': token },
    body: JSON.stringify(body),
  });
  return parseJsonResponse(response);
}

function renderControlErrors() {
  const message = [state.controlActionError, state.controlStatusError, state.liveSnapshotError]
    .filter(Boolean).join(' · ');
  $('controlError').textContent = message;
  $('controlError').hidden = !message;
}

function controlOwnerActive(control = state.control) {
  return Boolean(control?.active) && (control.owner_alive === true || OWNER_ACTIVE_STATES.includes(control.state));
}

function liveViewAvailable(control = state.control) {
  return Boolean(control?.active?.consumed)
    && [...OWNER_ACTIVE_STATES, ...OWNER_TERMINAL_STATES].includes(control.state);
}

function applyControl(control) {
  const hadControl = Boolean(state.control);
  const previousJob = state.control?.active?.id;
  state.control = control;
  if (control.active?.id !== previousJob) {
    $('fieldReady').checked = false;
    $('recoveryPower').checked = false; $('recoveryWorkspace').checked = false;
    state.recoveryJobId = null;
    if ($('recoveryDialog').open) $('recoveryDialog').close();
  }
  renderControl();
  syncControlTimer();
  if (state.liveMode && !control.active) exitLiveMode();
  const terminalKey = OWNER_TERMINAL_STATES.includes(control?.state) && control?.active?.id && control.owner_alive !== true
    ? `${control.active.id}:${control.state}` : null;
  if (hadControl && terminalKey && terminalKey !== state.lastTerminalKey) {
    void refreshRuns();
    void refreshSessions(true);
  }
  state.lastTerminalKey = terminalKey;
}

async function refreshControl() {
  if (state.controlPollBusy) return;
  state.controlPollBusy = true;
  try {
    applyControl(await getJson('/api/control'));
    state.controlStatusError = null;
    renderControlErrors();
  } catch (error) {
    const message = `실행 상태를 불러오지 못했습니다: ${error.message}`;
    state.control = {
      ...state.control, execution_connected: null, state: 'UNKNOWN',
      can_prepare: false, can_start: false, can_stop: false, block_reason: message,
    };
    renderControl();
    state.controlStatusError = message;
    renderControlErrors();
  } finally {
    state.controlPollBusy = false;
  }
}

function syncControlTimer() {
  const shouldPoll = controlOwnerActive();
  if (shouldPoll && state.controlTimer === null) {
    state.controlTimer = window.setInterval(async () => {
      await refreshControl();
      if (state.liveMode) await fetchLiveSnapshot();
    }, 1000);
  } else if (!shouldPoll && state.controlTimer !== null) {
    window.clearInterval(state.controlTimer);
    state.controlTimer = null;
  }
}

function renderControl() {
  const control = state.control;
  const active = control?.active;
  const connected = control?.execution_connected === true;
  const terminal = OWNER_TERMINAL_STATES.includes(control?.state);
  const liveAvailable = liveViewAvailable(control);
  $('systemControlState').textContent = connected ? '실행 제어 연결'
    : control?.execution_connected === false ? '저장 재생 전용' : control ? '실행 제어 UNKNOWN' : '실행 제어 확인 중';
  $('systemControlDetail').textContent = connected ? '장치는 기존 owner만 접근' : '브라우저 장치 직접 접근 없음';
  $('cameraModeLabel').textContent = state.liveMode ? terminal ? `${control.state} OWNER RECORD` : connected ? 'LIVE OWNER STREAMS' : 'OWNER FEED · CONNECTION UNKNOWN' : 'ARCHIVED CAMERA STREAMS';
  $('cameraModeKey').textContent = state.liveMode ? terminal ? `${control.state} · 마지막 owner 파일 기록 · 자동 갱신 없음` : connected ? 'LIVE = 기존 owner file feed' : '연결 상태 UNKNOWN · 이전 표시 보존' : 'REPLAY_FRESH = 보관 프레임';
  $('executionState').textContent = display(control?.state);
  $('executionState').className = `badge ${control?.state === 'FAULT' || control?.state === 'BLOCKED' || control?.state === 'LIMITED_HOLDING' ? 'danger' : control?.state === 'RUNNING' ? 'success' : 'neutral'}`;
  $('executionScope').textContent = display(active?.scope ?? control?.scope);
  $('activeJob').textContent = active ? `${display(active.id)} · ${sourceLabel(active.source_session)}` : '없음';
  $('activePhase').textContent = active ? `${display(active.state)} · ${display(active.phase)} · ${display(active.session_out)}` : '—';
  $('blockReason').textContent = control?.block_reason ? `실행 차단: ${control.block_reason}` : '';
  $('blockReason').hidden = !control?.block_reason;
  $('liveView').hidden = !liveAvailable || state.liveMode;
  $('liveView').disabled = !connected || !liveAvailable || state.controlBusy;
  $('liveView').textContent = terminal ? 'FULL_CHAIN 마지막 실물 세션 기록 보기' : 'FULL_CHAIN 현재 실물 세션 보기';
  $('archiveView').hidden = !state.liveMode;
  $('controlStop').disabled = !connected || control?.can_stop !== true || !active || state.controlBusy;
  $('recoveryControl').hidden = control?.can_confirm_recovery !== true;
  $('recoveryControl').disabled = !connected || state.controlBusy;
  $('confirmRecovery').disabled = !connected || control?.can_confirm_recovery !== true || state.controlBusy
    || state.recoveryJobId !== active?.id || !$('recoveryPower').checked || !$('recoveryWorkspace').checked;
  $('prepareControl').disabled = !connected || control?.can_prepare !== true || !state.plan
    || state.prepareSource === undefined || state.controlBusy;
  $('rerunRecovery').hidden = !['FAULT', 'LIMITED_HOLDING'].includes(control?.state);
  $('rerunBlockReason').textContent = control?.can_confirm_recovery === true
    ? '이전 실패 작업의 수동 복구 확인이 없어 새 준비가 막혀 있습니다. 아래에서 팔·물체 지지 후 전원 차단과 배치 복원 처리를 확인하십시오. 이미 사용한 START 승인은 재사용할 수 없습니다.'
    : control?.can_prepare === true
      ? '원래 실패 기록은 보존됩니다. 수동 복구 확인이 기록됐으므로 위의 새 실물 세션 준비를 누를 수 있습니다.'
      : '기존 owner 종료 또는 상태 확인이 필요합니다. ' + display(control?.block_reason);
  $('rerunRecoveryControl').hidden = control?.can_prepare === true;
  $('rerunRecoveryControl').disabled = !connected || control?.can_confirm_recovery !== true || state.controlBusy;
  $('preparedControl').hidden = !active;
  $('nextSession').textContent = active?.state === 'PREPARED' && !active.consumed && active.auto_prepared_after
    ? 'FULL_CHAIN 다음 실물 세션 · 준비됨, 확인 후 START' : 'FULL_CHAIN 다음 실물 세션';
  $('preparedJob').textContent = display(active?.id);
  $('preparedOutput').textContent = display(active?.session_out);
  $('preparedScope').textContent = display(active?.scope ?? control?.scope);
  $('fieldReady').disabled = !connected || !active || active.state !== 'PREPARED' || control?.can_start !== true || state.controlBusy;
  if (active?.consumed === true) $('fieldReady').checked = true;
  $('startControl').disabled = $('fieldReady').disabled || !$('fieldReady').checked;
  $('preapproveView').disabled = $('fieldReady').disabled || active?.execution_scope === 'LEFT_START_TO_LIFT';
  if ($('preapproveView').disabled) $('preapproveView').checked = false;
  const scopeName = active?.execution_scope === 'LEFT_START_TO_LIFT' ? 'LEFT_START_TO_LIFT' : 'FULL_CHAIN';
  $('startControl').textContent = control?.state === 'LIMITED_HOLDING' ? 'LEFT_START_TO_LIFT 보유 종료 · 수동 복구 전 새 실행 차단'
    : control?.state === 'FAULT' ? `${scopeName} 중단됨 · 새 실행 차단`
    : active?.consumed ? `${scopeName} 이번 세션 START 이미 전달됨` : `${scopeName} 현장 승인 후 실물 START`;
  $('limitedTerminal').hidden = active?.execution_scope !== 'LEFT_START_TO_LIFT';
  $('limitedTerminal').textContent = limitedTerminalText(active);
  $('fieldApprovalInfo').textContent = active?.consumed && active.auto_next?.error
    ? `다음 작업 자동 준비 실패: ${active.auto_next.error} · 위의 새 실물 세션 준비로 직접 준비하십시오.`
    : active?.consumed
    ? `이 작업의 승인은 이미 사용됐습니다. 현재 ${display(control.state)} · 중복 승인/START는 비활성입니다.`
    : !connected ? '제어 연결 확인 전에는 승인할 수 없습니다.'
      : active?.state === 'PREPARED' && active.auto_prepared_after
        ? `직전 실행(${active.auto_prepared_after})이 정상 종료되어 다음 작업이 자동으로 준비됐습니다. 물체·팔 배치를 다시 확인한 뒤 체크하고 START를 누르십시오.`
      : active?.state === 'PREPARED' ? '이번 세션의 현장 조건을 확인한 뒤 체크하고 START를 누르십시오.'
      : '새 작업 준비 후에 현장 승인을 입력할 수 있습니다.';
  renderPlanApproval();
  renderPendingView(active?.pending_view);
  if (state.liveMode && state.snapshot?.mode === 'LIVE') renderCameras(state.snapshot.cameras, 'LIVE');
  window.fieldPanel?.update(state.control, state.snapshot, state.liveSnapshotError);
}

function limitedTerminalText(active) {
  const doc = active?.limited_terminal;
  if (!doc) return 'LEFT_START_TO_LIFT 종료 기록 없음 · 종료 전이거나 UNKNOWN';
  const actual = doc.last_actual || {};
  const torque = actual.torque && typeof actual.torque === 'object' ? Object.entries(actual.torque).map(([k, v]) => `${k}=${v}`).join(' ') : 'UNKNOWN';
  return [`LIMITED_SCOPE_TERMINAL ${display(doc.terminal)}`, `마지막 stage ${display(doc.last_stage?.name)} · ${display(doc.last_stage?.controller_result)}`,
    `Torque(마지막 실행기 읽기) ${torque}`, `정상 HOME ${display(doc.normal_home)} · Torque OFF 확인 ${display(doc.torque_off_confirmed)} · safe shutdown ${display(doc.safe_shutdown)}`,
    `운영자 회수 필요 ${display(doc.operator_required)} · 다음 실행 자동 허용 ${display(doc.next_run_allowed)} · 프로세스 종료는 모터 정지/OFF 증거가 아님`].join(' · ');
}

function renderPlanApproval() {
  if (!state.plan) return;
  const used = state.control?.active?.source_session === state.plan.source_session && state.control.active.consumed === true;
  $('approvedBadge').textContent = used ? '이번 작업 승인 사용됨' : '새 세션 승인 대기';
  $('approvedBadge').className = `badge ${used ? 'success' : 'danger'}`;
  $('planApproval').textContent = used ? `${state.control.active.id} · START 승인 소비 · ${state.control.state}` : display(state.plan.approval_state);
}

function sourceLabel(source) {
  return source === null ? '현재 입력 · FULL_CHAIN 새 실물 세션' : display(source);
}

function renderPendingView(pending) {
  $('pendingView').hidden = !pending;
  if (!pending) return;
  $('pendingViewScope').textContent = display(pending.scope);
  $('pendingViewPlan').textContent = proposalText(pending.plan);
  $('pendingViewCandidate').textContent = proposalText(pending.candidate);
  $('approveView').disabled = state.control?.execution_connected !== true || state.controlBusy;
  $('declineView').disabled = state.control?.execution_connected !== true || state.controlBusy;
}

function filteredSessions() {
  return state.category === 'all' ? state.sessions : state.sessions.filter((session) => session.category === state.category);
}

function renderSessionOptions(preferredId, preferredTime) {
  const sessions = filteredSessions();
  const select = $('sessionSelect');
  select.replaceChildren();
  if (!sessions.length) {
    select.append(new Option('선택한 결과의 세션이 없습니다', ''));
    select.disabled = true;
    selectSession(null);
    return;
  }
  sessions.forEach((session) => select.append(new Option(`${session.label} · ${CATEGORY_LABEL[session.category] || display(session.category)}`, session.id)));
  const next = sessions.find((session) => session.id === preferredId) || sessions[0];
  select.value = next.id;
  select.disabled = false;
  if (state.session?.id !== next.id) selectSession(next, preferredTime);
  else renderSessionSummary();
}

function renderSessionSummary() {
  const session = state.session;
  const owner = state.liveMode ? state.control?.active : null;
  $('rawOutcome').textContent = owner ? `${owner.id} · ${display(state.control.state)}` : display(session?.outcome);
  $('classificationScope').textContent = owner ? display(state.control.scope) : display(session?.classification_scope);
  const badge = $('categoryBadge');
  badge.textContent = owner ? '실물 작업 기록' : CATEGORY_LABEL[session?.category] || display(session?.category);
  badge.className = `badge ${owner ? state.control.state === 'FAULT' ? 'danger' : 'neutral' : session?.category || 'neutral'}`;
  $('sessionWarning').textContent = owner ? '' : session?.warning || '';
  $('sessionWarning').hidden = Boolean(owner) || !session?.warning;
  $('successReplay').disabled = !state.sessions.some((item) => item.category === 'success');
  $('rerunPlan').disabled = !session;
  if (owner) $('bookmarkList').replaceChildren();
  else renderBookmarks();
}

function hostClock(seconds) {
  if (!finiteNumber(seconds)) return 'UNKNOWN';
  const date = new Date(Number(seconds) * 1000);
  const two = (value) => String(value).padStart(2, '0');
  return `${two(date.getHours())}:${two(date.getMinutes())}:${two(date.getSeconds())}.${String(date.getMilliseconds()).padStart(3, '0')}`;
}

function runEvidenceLines(run) {
  // Native terminal class, recorder errors and per-arm torque evidence, each on its own line (never merged into one verdict).
  const lines = [];
  const terminal = run?.native_terminal;
  if (terminal && (terminal.exit_code ?? null) !== null && terminal.exit_code !== 0) {
    lines.push(`Native 종료: ${display(terminal.result)} · exit ${display(terminal.exit_code)}${terminal.exit_code === 3 && terminal.result !== 'UNKNOWN' ? ' (비성공 종료 분류, 실행기 충돌 아님)' : ''}`);
  }
  const stages = Array.isArray(run?.stage_results) ? run.stage_results.slice(-3) : [];
  if (stages.length && run?.failure_point) lines.push(`단계 결과: ${stages.map((s) => `${display(s.stage)} ${display(s.result)}`).join(' → ')}`);
  (Array.isArray(run?.recorder_errors) ? run.recorder_errors : []).slice(0, 3).forEach((row) => {
    const status = row.cause_status === 'BUS_OPEN_CAUSE_UNCONFIRMED' ? '원인 미확정(BUS_OPEN_CAUSE_UNCONFIRMED)' : row.cause ? `원인 ${row.cause}` : '메시지만 기록';
    lines.push(`Recorder 오류(작업 Gate와 별개) ${hostClock(row.t_host_s)}: ${display(row.what).slice(0, 120)} · ${status}`);
  });
  const torque = run?.torque_evidence;
  if (torque && run?.failure_point) {
    ['left', 'right'].forEach((side) => {
      const row = torque[side] || {};
      const arm = side === 'left' ? '왼팔' : '오른팔';
      if (row.final_release_certified) lines.push(`${arm}: 최종 Torque 해제 확인 (${display(row.final_stage)} RETURNED_RELEASED)`);
      else if (finiteNumber(row.last_seen_torque_time_s)) lines.push(`${arm}: 최종 해제 확인 없음 · 마지막 관측 ${hostClock(row.last_seen_torque_time_s)} Torque ${row.last_seen_torque_all_zero ? '0' : '켜짐/일부'} (종료 확인 아님)`);
      else lines.push(`${arm}: 최종 해제 확인 없음 · Torque 관측 기록 없음`);
    });
  }
  return lines;
}

function renderRuns() {
  const rows = $('runRows');
  rows.replaceChildren();
  if (!state.runs.length) {
    const row = document.createElement('tr');
    const cell = document.createElement('td');
    cell.colSpan = 5; cell.className = 'muted'; cell.textContent = '저장된 실물 실행 이력이 없습니다.';
    row.append(cell); rows.append(row); return;
  }
  state.runs.filter((run) => state.runBucket === 'all' || run?.dataset?.auto_bucket === state.runBucket).forEach((run) => {
    const row = document.createElement('tr');
    row.dataset.runId = display(run?.id);

    const identity = document.createElement('td');
    const id = document.createElement('strong'); id.className = 'mono'; id.textContent = display(run?.id);
    const runState = document.createElement('small'); runState.textContent = display(run?.state);
    identity.append(id, runState);

    const result = document.createElement('td');
    const badge = document.createElement('span');
    const category = ['success', 'partial', 'failure'].includes(run?.category) ? run.category : 'neutral';
    badge.className = `badge ${category}`;
    badge.textContent = CATEGORY_LABEL[run?.category] || display(run?.category);
    const outcome = document.createElement('small');
    outcome.textContent = `${display(run?.outcome)} · dataset ${display(run?.dataset?.auto_bucket)} · human ${display(run?.dataset?.human_review?.purpose)} / ${display(run?.dataset?.human_review?.review_status)} · ACT ${run?.dataset?.human_review?.act_approved === true ? 'APPROVED' : 'NOT_APPROVED'}`;
    result.append(badge);
    const point = run?.failure_point;
    if (point && typeof point === 'object') {
      const where = document.createElement('div');
      where.className = 'failure-point';
      const stage = document.createElement('strong');
      stage.textContent = `멈춘 단계: ${display(point.label)}${point.after ? ` (직전: ${point.after})` : ''}`;
      const reason = document.createElement('small');
      reason.textContent = `${point.gate ? `${point.gate} · ` : ''}${display(point.reason)}`;
      if (point.detail) reason.title = point.detail;
      where.append(stage, reason);
      if (Array.isArray(point.then) && point.then.length) {
        const then = document.createElement('small');
        then.className = 'muted';
        then.textContent = `이후: ${point.then.join(' → ')}`;
        where.append(then);
      }
      result.append(where);
    }
    const extra = runEvidenceLines(run);
    if (extra.length) {
      const box = document.createElement('div');
      box.className = 'run-evidence';
      extra.forEach((text) => { const line = document.createElement('small'); line.textContent = text; box.append(line); });
      result.append(box);
    }
    result.append(outcome);

    const recording = document.createElement('td');
    const recordingStatus = document.createElement('strong');
    recordingStatus.textContent = display(run?.recording?.status);
    const files = Array.isArray(run?.recording?.files) ? run.recording.files.length : display(run?.recording?.files);
    const bytes = finiteNumber(run?.recording?.bytes) ? `${Number(run.recording.bytes).toLocaleString('ko-KR')} B` : 'UNKNOWN';
    const recordingDetail = document.createElement('small');
    recordingDetail.textContent = `files ${files} · bytes ${bytes} · 확인 ${display(run?.verified_at)}`;
    const recordingProblems = [...(Array.isArray(run?.recording?.missing) ? run.recording.missing : []),
      ...(Array.isArray(run?.recording?.errors) ? run.recording.errors : [])];
    recording.append(recordingStatus, recordingDetail);
    if (recordingProblems.length) {
      const problem = document.createElement('small'); problem.className = 'history-problem';
      problem.textContent = recordingProblems.map(display).join(' · '); recording.append(problem);
    }

    const source = document.createElement('td');
    const sourceName = document.createElement('span'); sourceName.textContent = sourceLabel(run?.source_session);
    const snapshot = document.createElement('small'); snapshot.textContent = `snapshot ${display(run?.source_snapshot_status)}`;
    source.append(sourceName, snapshot);

    const actions = document.createElement('td'); actions.className = 'history-actions';
    if (typeof run?.replay_session_id === 'string' && run.replay_session_id) {
      const replay = document.createElement('a');
      replay.className = 'button secondary'; replay.textContent = '저장 데이터 재생';
      replay.href = `/?${new URLSearchParams({ session: run.replay_session_id })}`;
      actions.append(replay);
    } else {
      const unavailable = document.createElement('span'); unavailable.className = 'muted'; unavailable.textContent = '재생 불가';
      actions.append(unavailable);
    }
    if (typeof run?.review_url === 'string' && run.review_url.startsWith('/api/runs/')) {
      const review = document.createElement('a');
      review.className = 'button secondary'; review.textContent = 'AI 검토 JSON'; review.href = run.review_url;
      review.download = typeof run.review_file === 'string' && run.review_file ? run.review_file : `${display(run?.id)}-review.json`;
      actions.append(review);
    } else {
      const unknown = document.createElement('span'); unknown.className = 'muted'; unknown.textContent = '검토 파일 UNKNOWN';
      actions.append(unknown);
    }
    if (typeof run?.failure_packet_url === 'string' && run.failure_packet_url.startsWith('/api/runs/')) {
      const packet = document.createElement('a'); packet.className = 'button secondary';
      packet.textContent = '실패 패킷 ZIP'; packet.href = run.failure_packet_url; packet.download = `${run.id}_FAILURE_PACKET.zip`;
      actions.append(packet);
    }
    if (typeof run?.id === 'string' && run.recording?.status !== 'RECORDING') {
      const reviewData = document.createElement('button'); reviewData.className = 'button secondary';
      reviewData.type = 'button'; reviewData.textContent = run?.dataset?.human_review ? '데이터 리뷰 수정' : '데이터 리뷰';
      reviewData.addEventListener('click', () => openDataReview(run)); actions.append(reviewData);
    }
    if (typeof run?.id === 'string' && run.recording?.status !== 'RECORDING') {
      const data = document.createElement('a');
      data.className = 'button secondary'; data.textContent = '원본 데이터 TAR';
      data.href = `/api/runs/${encodeURIComponent(run.id)}/data`;
      data.download = `${run.id}_data.tar`;
      data.title = '저장된 원본 프레임·Depth·관절·단계·로그를 묶어서 다운로드';
      actions.append(data);
    }
    [identity, result, recording, source, actions].forEach((cell, index) => {
      cell.dataset.label = ['Run ID / 상태', '결과', '데이터 저장 상태', '원본 / snapshot', '재생 / 다운로드'][index];
    });
    row.append(identity, result, recording, source, actions); rows.append(row);
  });
}

function openDataReview(run) {
  state.dataReviewRunId = run.id;
  const saved = run?.dataset?.human_review || {};
  const range = run?.dataset?.recorded_range || saved.recorded_range || {};
  $('dataReviewRun').textContent = `${run.id} · auto ${display(run?.dataset?.auto_bucket)} · capture ${display(run?.dataset?.capture_integrity)}`;
  state.dataReviewRange = range;
  const duration = finiteNumber(range.start_s) && finiteNumber(range.end_s) ? Number(range.end_s) - Number(range.start_s) : null;
  $('dataReviewRange').textContent = duration === null ? '기록 시간 범위 UNKNOWN' : `검수 가능한 범위: 0 ~ ${duration.toFixed(3)}초 · ${recordedTime(range.start_s)} ~ ${recordedTime(range.end_s)}`;
  $('dataReviewPurpose').value = saved.purpose || 'DIAGNOSTIC_ONLY';
  $('dataReviewPurpose').querySelector('option[value="SUCCESS_CANDIDATE"]').disabled = run?.dataset?.auto_bucket !== 'success_candidates';
  for (const id of ['dataReviewStart', 'dataReviewEnd']) { $(id).min = 0; $(id).max = duration ?? ''; }
  $('dataReviewStart').value = finiteNumber(saved.interval?.start_s) && finiteNumber(range.start_s) ? saved.interval.start_s - range.start_s : '';
  $('dataReviewEnd').value = finiteNumber(saved.interval?.end_s) && finiteNumber(range.start_s) ? saved.interval.end_s - range.start_s : '';
  $('dataReviewNote').value = saved.note || '';
  $('dataReviewError').hidden = true; $('dataReviewDialog').showModal();
}

async function saveDataReview() {
  const value = (id) => $('dataReview' + id).value;
  const optionalNumber = (id) => value(id) === '' ? null : Number(value(id)) + Number(state.dataReviewRange?.start_s);
  try {
    await postControl(`/api/runs/${encodeURIComponent(state.dataReviewRunId)}/data-review`, {
      purpose: value('Purpose'), start_s: optionalNumber('Start'), end_s: optionalNumber('End'), note: value('Note'),
    });
    $('dataReviewDialog').close(); await refreshRuns();
  } catch (error) {
    $('dataReviewError').textContent = error.message; $('dataReviewError').hidden = false;
  }
}

async function refreshRuns() {
  if (state.runsBusy) { state.runsRefreshPending = true; return; }
  state.runsBusy = true; $('refreshRuns').disabled = true;
  try {
    const data = await getJson('/api/runs');
    if (!Array.isArray(data.runs)) throw new Error('runs 배열이 없습니다.');
    state.runs = data.runs;
    $('runHistoryError').hidden = true;
    renderRuns();
    window.inspectionPanel?.setRuns(state.runs, state.session?.id);
  } catch (error) {
    $('runHistoryError').textContent = `실행 이력을 불러오지 못했습니다: ${error.message}`;
    $('runHistoryError').hidden = false;
  } finally {
    state.runsBusy = false; $('refreshRuns').disabled = false;
    if (state.runsRefreshPending) { state.runsRefreshPending = false; void refreshRuns(); }
  }
}

async function refreshSessions(silent = false, requestedId, preferredTime) {
  try {
    const data = await getJson('/api/sessions');
    if (!Array.isArray(data.sessions)) throw new Error('sessions 배열이 없습니다.');
    const previousId = state.session?.id;
    const previousTime = state.time;
    state.sessions = data.sessions;
    const refreshedSelection = state.sessions.find((session) => session.id === previousId);
    if (refreshedSelection) state.session = refreshedSelection;
    const requestedExists = requestedId && state.sessions.some((session) => session.id === requestedId);
    const preferredId = (requestedExists ? requestedId : null) ?? previousId
      ?? state.sessions.find((session) => session.id === 'session_18')?.id;
    renderSessionOptions(preferredId, preferredTime ?? (previousId ? previousTime : undefined));
    return true;
  } catch (error) {
    if (!silent) {
      $('sessionSelect').replaceChildren(new Option('세션 로드 실패', ''));
      $('sessionSelect').disabled = true;
      setError(`세션 목록을 불러오지 못했습니다: ${error.message}`);
    }
    return false;
  }
}

function selectSession(session, preferredTime) {
  pause();
  state.liveMode = false;
  setError('');
  state.snapshotGeneration += 1;
  state.seekRevision += 1;
  state.snapshotController?.abort();
  state.queuedSnapshot = null;
  state.session = session;
  window.inspectionPanel?.selectForReplay(session?.id);
  state.snapshot = null;
  state.snapshotError = null;
  clearSnapshot();
  if (!session) {
    $('playPause').disabled = true;
    $('timeSlider').disabled = true;
    renderSessionSummary();
    renderSnapshotState();
    renderControl();
    return;
  }
  const initialTime = finiteNumber(preferredTime) ? Number(preferredTime)
    : finiteNumber(session.default_t_s) ? Number(session.default_t_s) : Number(session.start_s);
  state.time = Math.min(Number(session.end_s), Math.max(Number(session.start_s), initialTime));
  const slider = $('timeSlider');
  slider.min = session.start_s;
  slider.max = session.end_s;
  slider.value = state.time;
  slider.disabled = false;
  $('playPause').disabled = false;
  $('speedSelect').disabled = false;
  renderSessionSummary();
  renderTime();
  queueSnapshot(state.time);
  renderControl();
}

function renderBookmarks() {
  const list = $('bookmarkList');
  list.replaceChildren();
  const bookmarks = state.session?.bookmarks || [];
  if (!bookmarks.length) {
    const empty = document.createElement('span');
    empty.className = 'muted'; empty.textContent = '없음'; list.append(empty); return;
  }
  bookmarks.forEach((bookmark) => {
    const button = document.createElement('button');
    button.type = 'button'; button.className = 'bookmark';
    button.textContent = `${bookmark.name} · ${relativeTime(Number(bookmark.t_s) - Number(state.session.start_s))}`;
    button.addEventListener('click', () => seek(Number(bookmark.t_s)));
    list.append(button);
  });
}

function relativeTime(value) {
  const safe = Math.max(0, Number(value) || 0);
  const minutes = Math.floor(safe / 60);
  return `+${String(minutes).padStart(2, '0')}:${(safe % 60).toFixed(1).padStart(4, '0')}`;
}

function renderTime() {
  $('relativeTime').textContent = state.session ? relativeTime(state.time - Number(state.session.start_s)) : '+00:00.0';
  $('wallTime').textContent = state.session ? `기록 ${recordedTime(state.time)}` : '기록 시각 —';
  if (state.session) $('timeSlider').value = state.time;
}

function seek(value, continuous = false) {
  if (!state.session || !Number.isFinite(value)) return;
  state.time = Math.min(Number(state.session.end_s), Math.max(Number(state.session.start_s), value));
  if (!continuous) {
    state.seekRevision += 1;
    state.snapshot = null;
    clearSnapshot();
  }
  state.snapshotError = null;
  renderTime();
  queueSnapshot(state.time);
}

function play() {
  if (!state.session || state.playing) return;
  if (state.time >= Number(state.session.end_s)) seek(Number(state.session.start_s));
  state.playing = true;
  state.lastTick = performance.now();
  $('playPause').textContent = '❚❚ 저장 시연 일시정지';
  state.timer = window.setInterval(() => {
    const now = performance.now();
    const elapsed = (now - state.lastTick) / 1000;
    state.lastTick = now;
    seek(state.time + elapsed * state.speed, true);
    if (state.time >= Number(state.session.end_s)) pause();
  }, 200);
}

function pause() {
  state.playing = false;
  if (state.timer !== null) window.clearInterval(state.timer);
  state.timer = null;
  $('playPause').textContent = '▶ 저장 시연 재생';
  renderSnapshotState();
}

function queueSnapshot(time) {
  if (!state.session) return;
  state.snapshotRevision += 1;
  state.queuedSnapshot = {
    id: state.session.id, time, generation: state.snapshotGeneration,
    revision: state.snapshotRevision, seekRevision: state.seekRevision, mode: 'REPLAY',
  };
  if (!state.snapshotBusy) fetchQueuedSnapshot();
  renderSnapshotState();
}

function snapshotResponseIsRenderable(request, snapshotTime, current = {
  generation: state.snapshotGeneration, sessionId: state.session?.id,
  revision: state.snapshotRevision, seekRevision: state.seekRevision,
  time: state.time, playing: state.playing,
}) {
  const responseTime = finiteNumber(snapshotTime) ? Number(snapshotTime) : Number(request.time);
  const sameSeek = request.generation === current.generation && request.id === current.sessionId
    && request.seekRevision === current.seekRevision;
  const latestRequest = request.revision === current.revision;
  const forwardPlaybackLag = current.playing && responseTime <= current.time;
  return sameSeek && responseTime <= current.time && (latestRequest || forwardPlaybackLag);
}

function renderSnapshotState() {
  if (state.liveMode) {
    const terminal = OWNER_TERMINAL_STATES.includes(state.control?.state);
    const connected = state.control?.execution_connected === true;
    const responseAge = finiteNumber(state.liveSnapshotReceivedAt)
      ? Math.max(0, Date.now() / 1000 - Number(state.liveSnapshotReceivedAt)) : null;
    $('snapshotState').textContent = state.liveSnapshotError && state.snapshot?.mode === 'LIVE'
      ? `LIVE snapshot 오류 · 마지막 응답 ${seconds(responseAge)} 전 · 표시 프레임 STALE 재평가`
      : state.snapshot?.mode === 'LIVE'
        ? terminal ? `${state.control.state} 종료 기록 · owner snapshot ${recordedTime(state.snapshot.time_s)} · 자동 갱신 없음`
          : connected ? `LIVE · 기존 owner snapshot ${recordedTime(state.snapshot.time_s)} · 마지막 응답 ${seconds(responseAge)} 전${state.liveSnapshotBusy ? ' · 갱신 중' : ''}`
            : `연결 UNKNOWN · 마지막 owner snapshot ${recordedTime(state.snapshot.time_s)}`
      : terminal ? `${state.control.state} 종료 snapshot 없음`
        : state.liveSnapshotBusy ? 'LIVE snapshot 조회 중 · 표시 자료 없음' : 'LIVE snapshot 없음';
    return;
  }
  const shownTime = state.snapshot?.time_s;
  const pending = state.snapshotBusy || state.queuedSnapshot;
  if (!finiteNumber(shownTime)) {
    $('snapshotState').textContent = state.snapshotError
      ? '조회 실패 · 표시 자료 없음'
      : pending ? '스냅샷 조회 중 · 표시 자료 없음' : '표시 스냅샷 없음';
    return;
  }
  const lag = Math.max(0, state.time - Number(shownTime));
  $('snapshotState').textContent = lag > 0.01
    ? `표시 ${recordedTime(shownTime)} · 커서보다 ${lag.toFixed(2)} s 이전${pending ? ' · 조회 중' : ''}`
    : `표시 스냅샷 ${recordedTime(shownTime)}${pending ? ' · 다음 조회 중' : ''}`;
}

async function fetchLiveSnapshot() {
  if (!state.liveMode || state.control?.execution_connected !== true || !liveViewAvailable()) return;
  if (state.liveSnapshotBusy) {
    state.liveSnapshotPending = true;
    if (state.snapshot?.mode === 'LIVE') renderCameras(state.snapshot.cameras, 'LIVE');
    renderSnapshotState();
    return;
  }
  const jobId = state.control.active.id;
  const generation = state.snapshotGeneration;
  state.liveSnapshotBusy = true;
  renderSnapshotState();
  try {
    const snapshot = await getJson('/api/control/snapshot');
    if (!state.liveMode || generation !== state.snapshotGeneration || state.control?.active?.id !== jobId) return;
    if (snapshot.mode !== 'LIVE') throw new Error(`예상하지 않은 snapshot mode: ${display(snapshot.mode)}`);
    state.liveSnapshotReceivedAt = Date.now() / 1000;
    state.liveSnapshotError = null;
    state.snapshot = snapshot;
    renderSnapshot(snapshot);
    renderControlErrors();
  } catch (error) {
    if (!state.liveMode || generation !== state.snapshotGeneration) return;
    state.liveSnapshotError = `LIVE snapshot을 불러오지 못했습니다: ${error.message}`;
    if (state.snapshot?.mode === 'LIVE') renderCameras(state.snapshot.cameras, 'LIVE');
    window.fieldPanel?.update(state.control, state.snapshot, state.liveSnapshotError);
    renderControlErrors();
  } finally {
    const retryForCurrentView = state.liveMode && state.control?.execution_connected === true && liveViewAvailable()
      && (state.liveSnapshotPending || generation !== state.snapshotGeneration || state.control?.active?.id !== jobId);
    state.liveSnapshotPending = false;
    state.liveSnapshotBusy = false;
    if (retryForCurrentView) void fetchLiveSnapshot();
    else renderSnapshotState();
  }
}

async function enterLiveMode() {
  if (state.control?.execution_connected !== true || !liveViewAvailable()) return;
  pause();
  state.snapshotGeneration += 1;
  state.snapshotController?.abort();
  state.queuedSnapshot = null;
  state.liveMode = true;
  state.liveSnapshotReceivedAt = null;
  state.liveSnapshotError = null;
  state.snapshot = null;
  clearSnapshot();
  renderControlErrors();
  $('playPause').disabled = true;
  $('timeSlider').disabled = true;
  $('speedSelect').disabled = true;
  renderControl();
  renderSnapshotState();
  await fetchLiveSnapshot();
}

function exitLiveMode() {
  if (!state.liveMode) return;
  const session = state.session;
  const time = state.time;
  state.liveMode = false;
  state.liveSnapshotReceivedAt = null;
  state.liveSnapshotError = null;
  renderControlErrors();
  selectSession(session, time);
}

async function fetchQueuedSnapshot() {
  const request = state.queuedSnapshot;
  if (!request) return;
  state.queuedSnapshot = null;
  state.snapshotBusy = true;
  const controller = new AbortController();
  state.snapshotController = controller;
  try {
    const snapshot = await getJson(`/api/sessions/${encodeURIComponent(request.id)}/snapshot?t=${encodeURIComponent(request.time)}`, controller.signal);
    if (!state.liveMode && request.mode === 'REPLAY' && snapshot.mode === 'REPLAY'
        && snapshotResponseIsRenderable(request, snapshot.time_s)) {
      state.snapshot = snapshot;
      state.snapshotError = null;
      renderSnapshot(snapshot);
      setError('');
    }
  } catch (error) {
    if (error.name !== 'AbortError' && request.generation === state.snapshotGeneration
        && request.seekRevision === state.seekRevision) {
      state.snapshotError = error.message;
      setError(`스냅샷을 불러오지 못했습니다: ${error.message}`);
    }
  } finally {
    state.snapshotBusy = false;
    if (state.snapshotController === controller) state.snapshotController = null;
    if (state.queuedSnapshot) fetchQueuedSnapshot();
    renderSnapshotState();
  }
}

function createCameraCards() {
  const grid = $('cameraGrid');
  CAMERA_ORDER.forEach(([id, label], index) => {
    const fragment = $('cameraTemplate').content.cloneNode(true);
    const card = fragment.querySelector('.camera-card');
    card.dataset.cameraId = id;
    card.querySelector('.camera-index').textContent = `0${index + 1}`;
    card.querySelector('h3').textContent = label;
    const image = card.querySelector('img');
    image.alt = `${label} 저장 프레임`;
    image.addEventListener('error', () => {
      image.dataset.failedSrc = image.dataset.src || '';
      image.hidden = true;
      const placeholder = card.querySelector('.camera-placeholder');
      placeholder.hidden = false;
      placeholder.textContent = '프레임을 표시할 수 없음';
    });
    image.addEventListener('load', () => {
      delete image.dataset.failedSrc;
      image.hidden = false;
      card.querySelector('.camera-placeholder').hidden = true;
    });
    grid.append(fragment);
  });
}

function renderCameras(cameras = [], mode) {
  const byId = new Map(cameras.map((camera) => [camera.id, camera]));
  const now = Date.now() / 1000;
  CAMERA_ORDER.forEach(([id]) => {
    const camera = byId.get(id);
    const card = document.querySelector(`[data-camera-id="${id}"]`);
    const nonLiveRecord = mode === 'LIVE' && (OWNER_TERMINAL_STATES.includes(state.control?.state)
      || state.control?.execution_connected !== true);
    const liveRefreshFailed = mode === 'LIVE' && Boolean(state.liveSnapshotError);
    const { rawStatus, status, liveAge, freshnessLimit } = cameraDisplayEvidence(
      camera, mode, now, nonLiveRecord || liveRefreshFailed,
    );
    const badge = card.querySelector('.camera-status');
    badge.textContent = status === 'REPLAY_FRESH' ? '보관 재생 · 프레임 있음'
      : status === 'LIVE' && mode === 'LIVE' ? `LIVE · ${display(camera?.connection)}` : display(status);
    badge.className = `camera-status badge ${status === 'REPLAY_FRESH' ? 'archive' : status === 'LIVE' && mode === 'LIVE' ? 'success' : status === 'STALE' ? 'danger' : 'neutral'}`;
    const image = card.querySelector('img');
    const placeholder = card.querySelector('.camera-placeholder');
    if (camera?.image_url) {
      if (image.dataset.src !== camera.image_url) {
        delete image.dataset.failedSrc;
        image.src = camera.image_url;
        image.dataset.src = camera.image_url;
      }
      image.hidden = image.dataset.failedSrc === camera.image_url;
      placeholder.hidden = !image.hidden;
      if (image.hidden) placeholder.textContent = '프레임을 표시할 수 없음';
    } else {
      if (image.dataset.src) { image.removeAttribute('src'); delete image.dataset.src; }
      image.hidden = true; placeholder.hidden = false;
      placeholder.textContent = camera?.reason || '프레임 없음';
    }
    const stale = card.querySelector('.stale-overlay');
    stale.hidden = status !== 'STALE';
    stale.querySelector('span').textContent = status === 'STALE' ? `${seconds(liveAge ?? camera?.age_s)} 경과` : '';
    const fpsField = card.querySelector('[data-field="fps"]');
    fpsField.textContent = id === 'top_depth' && camera?.saved_fps !== undefined
      ? `수신 ${fps(camera?.fps)} · 저장 ${fps(camera.saved_fps)}`
      : fps(camera?.fps);
    card.querySelector('[data-field="connection"]').textContent = display(camera?.connection);
    card.querySelector('[data-field="last"]').textContent = recordedTime(camera?.last_received_s);
    card.querySelector('[data-field="age"]').textContent = seconds(liveAge ?? camera?.age_s);
    const freshnessReason = liveRefreshFailed ? 'LIVE snapshot 갱신 실패 · 마지막 파일 프레임 유지'
      : rawStatus === 'LIVE' && status === 'STALE' && freshnessLimit !== null && liveAge > freshnessLimit
        ? `freshness ${seconds(freshnessLimit)} 초과`
        : rawStatus === 'LIVE' && status === 'UNKNOWN' ? 'freshness 근거 없음' : null;
    card.querySelector('[data-field="reason"]').textContent = freshnessReason || camera?.reason
      || (status === 'REPLAY_FRESH' ? '—' : display(camera?.reason));
    const range = camera?.depth_range_mm;
    card.querySelector('[data-field="depth"]').textContent = Array.isArray(range)
      ? `${range[0]}–${range[1]} mm`
      : range && typeof range === 'object'
        ? `${display(range.min_mm ?? range.min)}–${display(range.max_mm ?? range.max)} mm`
        : range ?? '—';
  });
}

function renderJoints(joints = []) {
  const body = $('jointRows');
  body.replaceChildren();
  const rows = joints.slice(0, 12);
  while (rows.length < 12) rows.push(null);
  rows.forEach((joint, index) => {
    const row = document.createElement('tr');
    const axis = joint ? `${joint.side === 'left' ? 'L' : joint.side === 'right' ? 'R' : display(joint.side)}${display(joint.id)} · ${display(joint.name)}` : `— · 미수신 축 ${index + 1}`;
    [axis, joint?.actual, joint?.goal, joint?.requested_goal, joint?.torque, joint?.moving, joint?.status].forEach((value, cellIndex) => {
      const cell = document.createElement(cellIndex === 0 ? 'th' : 'td');
      if (cellIndex === 0) cell.scope = 'row';
      cell.textContent = display(value);
      row.append(cell);
    });
    const feedback = document.createElement('td');
    feedback.className = 'evidence-cell';
    feedback.append(document.createTextNode(`${joint?.feedback_status === 'POST_RECOVERY_READ_AT_OR_BEFORE_RECORD_TIME' ? '승인 후 OFF 조회' : display(joint?.feedback_status)} · ${seconds(joint?.age_s)}`));
    const feedbackMeta = document.createElement('small');
    feedbackMeta.textContent = `${recordedTime(joint?.feedback_time_s)} · ${compactSource(joint?.source)}`;
    feedbackMeta.title = `전체 source: ${display(joint?.source)}`;
    feedback.append(feedbackMeta);
    const ack = document.createElement('td');
    ack.className = 'evidence-cell';
    ack.textContent = display(joint?.ack);
    ack.title = `ACK 증거 기록시각: ${recordedTime(joint?.ack_time_s)}\n출처: ${display(joint?.ack_source)}`;
    const historicalAck = joint?.last_acknowledged;
    if (historicalAck && typeof historicalAck === 'object') {
      const ackMeta = document.createElement('small');
      ackMeta.textContent = `과거 ACK 증거: ${display(historicalAck.ack)} · goal ${display(historicalAck.requested_goal)} · ${display(historicalAck.stage)} · ${recordedTime(historicalAck.recorded_at_s)}`;
      ack.append(ackMeta);
    }
    const application = document.createElement('td');
    application.textContent = display(joint?.command_state ?? joint?.application);
    application.title = `legacy application: ${display(joint?.application)} · proof: ${display(joint?.application_proof)}`;
    row.append(feedback, ack, application);
    body.append(row);
  });
}

function renderStages(results = []) {
  const target = $('stageResults');
  target.replaceChildren();
  const byName = new Map(results.map((result) => [result.name, result]));
  STAGE_ORDER.forEach((name, index) => {
    const result = byName.get(name);
    const item = document.createElement('div'); item.className = 'stage-item';
    const number = document.createElement('span'); number.className = 'stage-number'; number.textContent = String(index + 1).padStart(2, '0');
    const copy = document.createElement('div');
    const title = document.createElement('strong'); title.textContent = result?.label || name;
    const detail = document.createElement('p'); detail.textContent = display(result?.detail);
    const badge = document.createElement('span'); badge.className = result?.status === 'REFUSED' ? 'badge danger' : 'badge neutral'; badge.textContent = display(result?.status);
    copy.append(title, detail); item.append(number, copy, badge); target.append(item);
  });
}

function renderCommand(command = {}) {
  const error = command.first_error && typeof command.first_error === 'object' ? command.first_error : null;
  const attempt = command.last_attempt && typeof command.last_attempt === 'object' ? command.last_attempt : null;
  const id = error?.id ?? error?.servo_id ?? error?.device_id ?? error?.actuator_id
    ?? attempt?.id ?? attempt?.servo_id ?? attempt?.device_id ?? command.id ?? 'UNKNOWN';
  const ack = display(error?.ack ?? command.ack);
  const application = display(command.state ?? error?.application ?? command.application);
  const safety = display(error?.safety ?? error?.safe_shutdown ?? command.safe_shutdown);
  $('commandSummary').textContent = `ID${display(id)} ACK${ack}/application${application}/safety${safety}`;
  $('commandAck').textContent = display(command.ack);
  $('commandApplication').textContent = display(command.state ?? command.application) + (command.application_proof?.length ? ' · ' + command.application_proof.join(', ') : '');
  $('parentExit').textContent = display(command.terminal?.parent_exit_code);
  $('childExit').textContent = display(command.terminal?.child_exit_code);
  $('safeShutdown').textContent = display(command.safe_shutdown);
  $('firstErrorRecord').classList.toggle('empty', !error);
  $('errorGate').textContent = error ? display(error.gate ?? error.name) : 'FIRST_ERROR 없음';
  $('errorDetail').textContent = error
    ? display(error.detail ?? error.message)
    : '이 시각까지 최초 오류 기록 없음';
  const recovery = command.later_recovery && typeof command.later_recovery === 'object' ? command.later_recovery : null;
  $('laterRecovery').hidden = !recovery;
  $('recoveryTime').textContent = recovery ? `기록 시각 ${recordedTime(recovery.time_s)}` : 'UNKNOWN';
  $('recoveryDetail').textContent = recovery
    ? [recovery.scope, `left off: ${display(recovery.all_left_off)}`, `right: ${display(recovery.right)}`, recovery.source].filter(Boolean).join(' · ')
    : '—';
}

function renderWarnings(warnings = []) {
  const target = $('snapshotWarnings'); target.replaceChildren();
  warnings.forEach((warning) => {
    const item = document.createElement('p'); item.textContent = typeof warning === 'string' ? warning : JSON.stringify(warning); target.append(item);
  });
}

function renderSnapshot(snapshot) {
  renderSessionSummary();
  if (snapshot.mode === 'LIVE') {
    $('relativeTime').textContent = '실물 작업 기록';
    $('wallTime').textContent = `화면 조회 ${recordedTime(snapshot.time_s)}`;
  }
  renderCameras(snapshot.cameras, snapshot.mode);
  renderJoints(snapshot.joints);
  renderStages(snapshot.stage_results);
  $('jointTime').textContent = `스냅샷 기준 ${recordedTime(snapshot.time_s)} · 축별 측정시각은 Feedback 열`;
  $('currentStage').textContent = display(snapshot.current_stage);
  $('ikStatus').textContent = display(snapshot.ik?.status);
  $('ikDetails').textContent = jsonText(snapshot.ik?.details);
  $('collisionStatus').textContent = display(snapshot.collision?.status);
  $('collisionClass').textContent = display(snapshot.collision?.classification);
  $('collisionDetails').textContent = jsonText(snapshot.collision?.details);
  renderCommand(snapshot.command);
  renderWarnings(snapshot.warnings);
  window.fieldPanel?.update(state.control, snapshot, state.liveSnapshotError);
}

function clearSnapshot() {
  renderCameras([]); renderJoints([]); renderStages([]); renderCommand({}); renderWarnings([]);
  $('jointTime').textContent = '스냅샷 기준 —'; $('currentStage').textContent = 'UNKNOWN';
  $('ikStatus').textContent = 'UNKNOWN'; $('ikDetails').textContent = '—';
  $('collisionStatus').textContent = 'UNKNOWN'; $('collisionClass').textContent = '—'; $('collisionDetails').textContent = '—';
  window.fieldPanel?.update(state.control, null);
}

function listItems(target, items) {
  target.replaceChildren();
  (items || []).forEach((value) => { const item = document.createElement('li'); item.textContent = display(value); target.append(item); });
}

async function openRerunPlan(sourceSession = state.session?.id, scope = 'FULL_CHAIN_RECEIVE_CONTACT') {
  if (sourceSession !== null && !sourceSession) return;
  const planUrl = sourceSession === null
    ? '/api/control/plan'
    : `/api/sessions/${encodeURIComponent(sourceSession)}/rerun-plan`;
  const dialog = $('rerunDialog');
  $('rerunLoading').hidden = false; $('rerunContent').hidden = true; $('rerunError').hidden = true;
  $('downloadJson').disabled = true; $('downloadText').disabled = true;
  state.planController?.abort();
  state.planController = new AbortController();
  state.plan = null;
  state.prepareSource = sourceSession;
  state.prepareScope = scope;
  const limited = scope === 'LEFT_START_TO_LIFT';
  $('rerunTitle').textContent = limited ? 'LEFT_START_TO_LIFT 제한 실물 세션 준비' : sourceSession === null ? 'FULL_CHAIN 다음 실물 세션 준비' : 'FULL_CHAIN 저장 세션 기반 재실행 준비';
  $('prepareDescription').textContent = limited
    ? 'LEFT_START_TO_LIFT · 현재 입력으로 새 출력과 이 범위 전용 승인·Manifest만 준비합니다. LIFT 뒤 왼팔은 Torque ON으로 남고 자동 HOME/OFF는 없습니다.'
    : sourceSession === null
    ? 'FULL_CHAIN_RECEIVE_CONTACT · 저장 기록과 독립된 현재 입력으로 새 출력 디렉터리와 실행 작업만 준비합니다.'
    : `FULL_CHAIN_RECEIVE_CONTACT · 저장 세션 ${sourceSession}을 원본으로 새 출력 디렉터리와 실행 작업만 준비합니다.`;
  dialog.showModal();
  renderControl();
  void refreshControl();
  try {
    const plan = await getJson(planUrl, state.planController.signal);
    if (state.prepareSource !== sourceSession) return;
    if (plan.source_session !== sourceSession) throw new Error('계획의 입력 컨텍스트가 요청과 일치하지 않습니다.');
    state.plan = plan;
    $('planSource').textContent = sourceLabel(plan.source_session);
    $('planScope').textContent = display(plan.scope);
    $('planApproval').textContent = display(plan.approval_state);
    $('planLauncher').textContent = display(plan.existing_launcher);
    $('approvedBadge').textContent = plan.approved === true ? '승인됨' : '승인 대기';
    $('approvedBadge').className = `badge ${plan.approved === true ? 'success' : 'danger'}`;
    $('connectionBadge').textContent = plan.execution_connected === true ? '실행 연결됨' : 'START 미연결';
    $('connectionBadge').className = `badge ${plan.execution_connected === true ? 'success' : 'danger'}`;
    listItems($('planRequirements'), plan.requirements);
    listItems($('planProhibited'), plan.prohibited);
    $('rerunLoading').hidden = true; $('rerunContent').hidden = false;
    renderControl();
    $('downloadJson').disabled = false; $('downloadText').disabled = false;
  } catch (error) {
    if (error.name === 'AbortError') return;
    $('rerunLoading').hidden = true; $('rerunError').hidden = false;
    $('rerunError').textContent = `재실행 준비 정보를 불러오지 못했습니다: ${error.message}`;
  }
}

function downloadPlan(format) {
  if (!state.plan) return;
  const content = format === 'json' ? JSON.stringify(state.plan, null, 2) : [
    `입력 컨텍스트: ${sourceLabel(state.plan.source_session)}`,
    `범위: ${display(state.plan.scope)}`,
    `승인 상태: ${display(state.plan.approval_state)}`,
    `실행 연결: ${state.plan.execution_connected === true ? '연결됨' : 'START 미연결'}`,
    '', '필수 검증:', ...(state.plan.requirements || []).map((item) => `- ${item}`),
    '', '금지 사항:', ...(state.plan.prohibited || []).map((item) => `- ${item}`),
  ].join('\n');
  const url = URL.createObjectURL(new Blob([content], { type: format === 'json' ? 'application/json' : 'text/plain' }));
  const link = document.createElement('a'); link.href = url; link.download = `rerun-plan-${state.plan.source_session || 'current-input'}.${format}`; link.click();
  URL.revokeObjectURL(url);
}

async function runControlAction(path, body, failureLabel) {
  if (state.controlBusy) return;
  state.controlBusy = true;
  state.controlActionError = null;
  renderControlErrors();
  renderControl();
  try {
    applyControl(await postControl(path, body));
    state.controlActionError = null;
    renderControlErrors();
    $('rerunError').hidden = true;
  } catch (error) {
    const message = `${failureLabel}: ${error.message}`;
    state.controlActionError = message;
    await refreshControl();
    renderControlErrors();
    if ($('rerunDialog').open) {
      $('rerunError').textContent = message;
      $('rerunError').hidden = false;
    }
  } finally {
    state.controlBusy = false;
    renderControl();
  }
}

async function prepareControl() {
  if (!state.plan || state.prepareSource === undefined || state.control?.execution_connected !== true || state.control.can_prepare !== true) return;
  await runControlAction('/api/control/prepare', { source_session: state.prepareSource, execution_scope: state.prepareScope || 'FULL_CHAIN_RECEIVE_CONTACT' }, '실물 세션 준비 실패');
}

async function startControl() {
  const active = state.control?.active;
  if (!active || state.control?.execution_connected !== true || state.control.can_start !== true || !$('fieldReady').checked) return;
  await runControlAction('/api/control/start', {
    job_id: active.id, field_ready: true, mounts_unchanged: true, supervised: true,
    execution_scope: active.execution_scope || 'FULL_CHAIN_RECEIVE_CONTACT',
    preapprove_view: active.execution_scope !== 'LEFT_START_TO_LIFT' && $('preapproveView').checked === true,
  }, '실물 START 실패');
  if (state.control?.active?.id === active.id && state.control.active.consumed && liveViewAvailable()) {
    if ($('rerunDialog').open) $('rerunDialog').close();
    await enterLiveMode();
  }
}

async function stopControl() {
  const active = state.control?.active;
  if (!active || state.control?.execution_connected !== true || state.control.can_stop !== true) return;
  await runControlAction('/api/control/stop', { job_id: active.id }, '실물 STOP 실패');
}

async function confirmRecovery() {
  const active = state.control?.active;
  const jobId = state.recoveryJobId;
  if (!active || active.id !== jobId || $('confirmRecovery').disabled) return;
  $('recoveryError').hidden = true;
  await runControlAction('/api/control/confirm-recovery', {
    job_id: jobId, power_isolated: $('recoveryPower').checked, workspace_reset: $('recoveryWorkspace').checked,
  }, '수동 복구 기록 실패');
  if (state.control?.active?.id === jobId && state.control.active.operator_recovery) $('recoveryDialog').close();
  else {
    $('recoveryError').textContent = state.controlActionError || '수동 복구 확인을 기록하지 못했습니다.';
    $('recoveryError').hidden = false;
  }
}

async function decidePendingView(approved) {
  const active = state.control?.active;
  const pending = active?.pending_view;
  if (!active || !pending || state.control?.execution_connected !== true) return;
  await runControlAction('/api/control/view-decision', {
    job_id: active.id, request_sha256: pending.request_sha256, approved,
  }, '관측 이동 결정 실패');
}

async function init() {
  createCameraCards(); clearSnapshot();
  renderControl();
  void refreshControl();
  void refreshRuns();
  const query = new URLSearchParams(window.location.search);
  const requestedId = query.get('session');
  if (query.get('prepare') === '1') { await refreshControl(); await openRerunPlan(null); }
  const sessionsLoaded = await refreshSessions(false, requestedId || undefined, query.get('t') || undefined);
  if (sessionsLoaded && state.session) {
    if (query.get('play') === '1') {
      const requestedSpeed = query.get('speed');
      if ([...$('speedSelect').options].some(option => option.value === requestedSpeed)) {
        state.speed = Number(requestedSpeed); $('speedSelect').value = requestedSpeed;
      }
      if (!query.has('t')) seek(Number(state.session.start_s));
      play();
    }
  }
  if (query.get('live') === '1') { await refreshControl(); await enterLiveMode(); }
}

$('categoryFilters').addEventListener('change', (event) => { if (event.target.name === 'category') { if (state.liveMode) exitLiveMode(); state.category = event.target.value; renderSessionOptions(state.session?.id); } });
$('sessionSelect').addEventListener('change', (event) => selectSession(state.sessions.find((session) => session.id === event.target.value) || null));
$('timeSlider').addEventListener('input', (event) => seek(Number(event.target.value)));
$('speedSelect').addEventListener('change', (event) => { state.speed = Number(event.target.value); });
$('playPause').addEventListener('click', () => state.playing ? pause() : play());
$('successReplay').addEventListener('click', () => {
  const session = state.sessions.find((item) => item.category === 'success');
  if (!session) return;
  if (state.liveMode) exitLiveMode();
  state.category = 'success';
  document.querySelector('input[name="category"][value="success"]').checked = true;
  renderSessionOptions(session.id);
  if (state.session?.id === session.id) { seek(Number(session.start_s)); play(); }
});
$('nextSession').addEventListener('click', () => openRerunPlan(null));
$('rerunPlan').addEventListener('click', () => openRerunPlan(state.session?.id));
$('refreshRuns').addEventListener('click', refreshRuns);
$('runBucketFilter').addEventListener('change', (event) => { state.runBucket = event.target.value; renderRuns(); });
$('saveDataReview').addEventListener('click', saveDataReview);
$('liveView').addEventListener('click', enterLiveMode);
$('archiveView').addEventListener('click', exitLiveMode);
$('prepareControl').addEventListener('click', prepareControl);
$('fieldReady').addEventListener('change', renderControl);
$('startControl').addEventListener('click', startControl);
$('controlStop').addEventListener('click', stopControl);
$('recoveryControl').addEventListener('click', () => {
  state.recoveryJobId = state.control?.active?.id;
  $('recoveryPower').checked = false; $('recoveryWorkspace').checked = false;
  $('recoveryError').hidden = true;
  $('recoveryJob').textContent = display(state.control?.active?.id);
  renderControl(); $('recoveryDialog').showModal();
});
$('rerunRecoveryControl').addEventListener('click', () => {
  if ($('rerunRecoveryControl').disabled) return;
  $('rerunDialog').close();
  $('recoveryControl').click();
});
$('recoveryPower').addEventListener('change', renderControl);
$('recoveryWorkspace').addEventListener('change', renderControl);
$('confirmRecovery').addEventListener('click', confirmRecovery);
$('recoveryDialog').addEventListener('close', () => {
  if ($('recoveryDialog').open) return;
  state.recoveryJobId = null;
  $('recoveryPower').checked = false; $('recoveryWorkspace').checked = false;
  renderControl();
});
$('approveView').addEventListener('click', () => decidePendingView(true));
$('declineView').addEventListener('click', () => decidePendingView(false));
$('downloadJson').addEventListener('click', () => downloadPlan('json'));
$('downloadText').addEventListener('click', () => downloadPlan('txt'));
$('rerunDialog').addEventListener('close', () => {
  if ($('rerunDialog').open) return;
  state.planController?.abort();
  state.plan = null;
  state.prepareSource = undefined;
});

window.fieldPanel?.setOnPrepare((scope) => openRerunPlan(null, scope || 'FULL_CHAIN_RECEIVE_CONTACT'));

window.inspectionPanel?.setReplaySelector((run) => {
  const session = state.sessions.find((item) => item.id === run?.replay_session_id);
  if (!session || state.session?.id === session.id) return;
  if (state.liveMode) exitLiveMode();
  state.category = 'all';
  document.querySelector('input[name="category"][value="all"]').checked = true;
  renderSessionOptions(session.id);
});

// Minimal contract self-check; failures stay visible in development consoles.
console.assert(CAMERA_ORDER.length === 4 && STAGE_ORDER.length === 5, '고정 운영 패널 구성이 손상되었습니다.');
async function selfCheckReverseSeek() {
  let release;
  const delayedResponse = new Promise((resolve) => { release = resolve; });
  const request = { id: 'test', time: 20, generation: 1, revision: 1, seekRevision: 1 };
  let current = { sessionId: 'test', time: 20, generation: 1, revision: 1, seekRevision: 1, playing: false };
  const rendered = delayedResponse.then((time) => snapshotResponseIsRenderable(request, time, current));
  current = { ...current, time: 10, revision: 2, seekRevision: 2 };
  release(20);
  console.assert(!await rendered, '역방향 seek 뒤 늦은 미래 스냅샷을 폐기해야 합니다.');
}
void selfCheckReverseSeek();
init();
