'use strict';

(() => {
  const LIMITED = 'LEFT_START_TO_LIFT';
  const FULL = 'FULL_CHAIN_RECEIVE_CONTACT';
  const RIGHT_X = 'RIGHT_X_FINITE_METROLOGY';
  const panelState = { catalog: null, control: null, snapshot: null, snapshotError: null, onPrepare: null, busy: false };
  const byId = (id) => document.getElementById(id);
  const shown = (value) => value === null || value === undefined || value === '' ? 'UNKNOWN' : String(value);
  const valueSummary = (value) => {
    if (value === null || value === undefined || value === '') return 'UNKNOWN';
    if (Array.isArray(value)) return value.length ? value.slice(0, 8).map(shown).join(', ') : '없음';
    if (typeof value !== 'object') return String(value);
    const rows = Object.entries(value).slice(0, 8).map(([key, item]) => `${key}=${typeof item === 'object' ? '[provided]' : shown(item)}`);
    return rows.length ? rows.join(' · ') : '없음';
  };
  const recordedTime = (value) => typeof value === 'number' && Number.isFinite(value)
    ? new Intl.DateTimeFormat('ko-KR', { dateStyle: 'short', timeStyle: 'medium', hour12: false }).format(new Date(value * 1000))
    : shown(value);

  function fillList(target, values) {
    target.replaceChildren();
    const items = Array.isArray(values) && values.length ? values : ['없음'];
    items.forEach((value) => {
      const item = document.createElement('li');
      item.textContent = shown(value);
      target.append(item);
    });
  }

  function scopeById(id) {
    return panelState.catalog?.scopes?.find((scope) => scope?.id === id) || null;
  }

  function prepareAllowed(scope) {
    return [FULL, LIMITED, RIGHT_X].includes(scope?.id) && scope.supported === true && scope.can_prepare === true
      && panelState.control?.execution_connected === true && panelState.control?.can_prepare === true;
  }

  function renderScope() {
    const selected = byId('fieldScopeSelect').value;
    const scope = scopeById(selected);
    const limited = selected === LIMITED;
    const measuring = selected === RIGHT_X;
    const supported = scope?.supported === true;
    byId('fieldScopeStatus').textContent = scope
      ? `${shown(scope.backend_status)} · ${supported ? 'SUPPORTED' : 'UNSUPPORTED'}`
      : limited || measuring ? 'BACKEND_CHANGE_REQUIRED · UNSUPPORTED' : 'UNKNOWN';
    byId('fieldScopeReason').textContent = scope?.reason || (limited
      ? 'native fixed full-chain · no hold terminal · ownership continuous contract'
      : '범위 계약 응답에 이 ID가 없습니다.');
    byId('fieldTermination').textContent = `종료 의미 ${valueSummary(scope?.termination)}`;
    fillList(byId('fieldIncluded'), scope?.included_stages);
    fillList(byId('fieldExcluded'), scope?.excluded_stages);

    const allowed = prepareAllowed(scope);
    const button = byId('fieldPrepare');
    button.disabled = !allowed;
    button.textContent = limited ? (allowed ? 'LEFT_START_TO_LIFT 제한 준비 흐름 열기' : supported ? 'LEFT_START_TO_LIFT 준비 조건 미충족' : '현재 제한 범위는 준비할 수 없음')
      : measuring ? (allowed ? 'RIGHT_X 측정 준비 흐름 열기 · 운영자 입력 5개' : supported ? 'RIGHT_X 준비 조건 미충족' : 'RIGHT_X 측정 범위는 준비할 수 없음')
      : allowed ? 'FULL_CHAIN 기존 준비 흐름 열기' : 'FULL_CHAIN 준비 조건 미충족';
    byId('fieldScopeMessage').textContent = measuring
      ? (supported ? `RIGHT_X_FINITE_METROLOGY는 이 범위 전용 승인·Manifest로만 준비합니다. 축·변위는 운영자가 직접 입력하며 기본값·상한이 없고, 입력값은 실물 허가가 아닙니다(${shown(scope?.physical_basis?.status)}). F/V 이동마다 계획 시작 실측·목표·충돌검사·정상 복귀를 보고 따로 결정합니다. START 사전 승인은 쓰지 않습니다. ${scope?.reason || ''}`
        : `RIGHT_X_FINITE_METROLOGY는 활성 native가 이 범위를 시험한 소스와 같지 않아 준비할 수 없습니다. ${scope?.reason || ''}`)
      : limited
      ? (supported ? `LEFT_START_TO_LIFT는 이 범위 전용 승인·Manifest로만 준비합니다. LIFT 뒤 보유 상태로 종료하며 다음 실행은 수동 복구 확인 전까지 막힙니다. ${scope?.reason || ''}`
        : `LEFT_START_TO_LIFT는 현재 backend가 지원하지 않아 실행·준비할 수 없습니다. ${scope?.reason || 'native 실행기는 fixed full-chain이며 LIFT/HOLDING 종료와 범위별 ownership 계약이 없습니다.'}`)
      : supported
        ? 'FULL_CHAIN_RECEIVE_CONTACT는 기존 backend와 기존 현장 승인 흐름만 사용합니다. 이 버튼은 START를 직접 보내지 않습니다.'
        : 'FULL_CHAIN_RECEIVE_CONTACT 지원 상태를 확인할 수 없어 준비를 열지 않습니다.';
  }

  function renderSources(sources) {
    const body = byId('fieldSources');
    body.replaceChildren();
    const rows = Array.isArray(sources) && sources.length ? sources : [null];
    rows.forEach((source) => {
      const row = document.createElement('tr');
      [source?.file, source?.sha256, source?.bytes].forEach((value, index) => {
        const cell = document.createElement('td');
        cell.textContent = source ? shown(value) : index === 0 ? '원본 정보 없음' : 'UNKNOWN';
        row.append(cell);
      });
      body.append(row);
    });
  }

  function renderCatalog() {
    const data = panelState.catalog;
    if (!data) {
      renderScope();
      return;
    }
    const owner = data.owner || {};
    byId('fieldCatalogState').textContent = `${shown(data.schema)} · checked_at ${shown(data.checked_at)} · GET 조회 전용`;
    byId('fieldDeviceAccess').textContent = `Device access ${data.device_access === false ? 'false' : shown(data.device_access)}`;
    byId('fieldObservation').textContent = `Observation ${shown(data.observation?.status)} · ${valueSummary(data.observation?.basis)}`;
    byId('fieldCalibration').textContent = `Calibration ${shown(data.calibration?.status)} · ${valueSummary(data.calibration?.basis)}`;
    byId('fieldOwner').textContent = `${shown(owner.state)} · alive ${shown(owner.owner_alive)} · run ${shown(owner.active_run_id)}`;
    byId('fieldOccupancy').textContent = `Device occupancy ${valueSummary(owner.device_occupancy)}`;
    byId('fieldOwnerError').textContent = `Device query error ${owner.device_query_error === null ? '없음' : shown(owner.device_query_error)}`;
    renderSources(data.sources);
    renderScope();
  }

  function compactAttempt(attempt) {
    if (!attempt || typeof attempt !== 'object') return 'UNKNOWN';
    const keys = ['id', 'servo_id', 'device_id', 'side', 'stage', 'joint', 'requested_tick', 'requested_goal', 'goal', 'acknowledgement', 'ack', 'result'];
    const values = keys.filter((key) => attempt[key] !== undefined).map((key) => `${key}=${typeof attempt[key] === 'object' ? JSON.stringify(attempt[key]) : shown(attempt[key])}`);
    return values.length ? values.join(' · ') : valueSummary(attempt);
  }

  function activeEvidence(snapshot) {
    return snapshot && snapshot.mode !== 'LIVE' ? {} : panelState.control?.active || {};
  }

  function lastCompletedStage(snapshot) {
    const explicit = snapshot?.last_completed_stage ?? snapshot?.command?.last_completed_stage
      ?? activeEvidence(snapshot).last_completed_stage;
    if (explicit !== undefined && explicit !== null && explicit !== '') return shown(explicit);
    const stages = Array.isArray(snapshot?.stage_results) ? snapshot.stage_results : [];
    for (let index = stages.length - 1; index >= 0; index -= 1) {
      const status = String(stages[index]?.status || '').toUpperCase();
      if (stages[index]?.completion !== 'BLOCKED_NOT_COMPLETED' && ['SUCCESS', 'SUCCEEDED', 'COMPLETE', 'COMPLETED', 'HOLDING', 'GRIP_CONTACT_HOLDING', 'RETURNED_RELEASED'].includes(status)) {
        return shown(stages[index]?.name ?? stages[index]?.label);
      }
    }
    return 'UNKNOWN';
  }

  function firstFailure(snapshot) {
    const failure = snapshot?.first_failure ?? snapshot?.command?.first_error ?? activeEvidence(snapshot).first_error;
    if (!failure || typeof failure !== 'object') return shown(failure);
    const gate = failure.gate ?? failure.name ?? failure.stage ?? failure.code;
    const detail = failure.detail ?? failure.message ?? failure.result;
    return [shown(gate), detail === undefined ? null : shown(detail)].filter(Boolean).join(' · ');
  }

  function renderReadback(snapshot) {
    const body = byId('fieldReadback');
    body.replaceChildren();
    const joints = Array.isArray(snapshot?.joints) ? snapshot.joints.slice(0, 12) : [];
    if (!joints.length) {
      const row = document.createElement('tr');
      const cell = document.createElement('td'); cell.colSpan = 5; cell.textContent = 'Snapshot 증거 없음';
      row.append(cell); body.append(row); return;
    }
    joints.forEach((joint, index) => {
      const historical = joint?.last_acknowledged && typeof joint.last_acknowledged === 'object' ? joint.last_acknowledged : {};
      const row = document.createElement('tr');
      const axis = `${joint?.side === 'left' ? 'L' : joint?.side === 'right' ? 'R' : shown(joint?.side)}${shown(joint?.id ?? index + 1)} · ${shown(joint?.name)}`;
      const values = [axis, joint?.requested_goal ?? historical.requested_goal, joint?.ack ?? historical.ack,
        joint?.actual, joint?.goal];
      values.forEach((value, cellIndex) => {
        const cell = document.createElement(cellIndex === 0 ? 'th' : 'td');
        if (cellIndex === 0) cell.scope = 'row';
        cell.textContent = shown(value);
        row.append(cell);
      });
      body.append(row);
    });
  }

  function renderCameras(snapshot) {
    const cameras = Array.isArray(snapshot?.cameras) ? snapshot.cameras : [];
    const target = byId('fieldCameras'); target.replaceChildren();
    (cameras.length ? cameras : [null]).forEach((camera) => {
      const item = document.createElement('li');
      const forceStale = panelState.snapshotError || panelState.control?.execution_connected !== true
        || !['STARTING', 'RUNNING', 'STOP_REQUESTED'].includes(panelState.control?.state);
      const display = typeof window.cameraDisplayEvidence === 'function'
        ? window.cameraDisplayEvidence(camera, snapshot?.mode, Date.now() / 1000, forceStale) : {};
      item.textContent = camera
        ? `${shown(camera.label ?? camera.id)} · ${shown(display.status)} · connection ${shown(camera.connection)} · age ${shown(display.liveAge ?? camera.age_s)} s · last ${recordedTime(camera.last_received_s)}`
        : 'Snapshot 증거 없음';
      target.append(item);
    });
  }

  function renderEvidence() {
    const control = panelState.control;
    const snapshot = panelState.snapshot;
    const mode = snapshot?.mode;
    const runId = mode === 'LIVE' ? control?.active?.id : snapshot?.session?.id;
    byId('fieldEvidenceContext').textContent = !snapshot ? `snapshot 없음 · controller ${shown(control?.state)}`
      : mode === 'LIVE'
        ? `LIVE · run ${shown(runId)} · snapshot ${recordedTime(snapshot.time_s)}`
        : `REPLAY · 저장 run ${shown(runId)} · snapshot ${recordedTime(snapshot.time_s)} · 현재 실물 actual 아님`;
    const active = activeEvidence(snapshot);
    const currentStage = snapshot?.current_stage ?? active.phase;
    const currentStageResult = Array.isArray(snapshot?.stage_results)
      ? snapshot.stage_results.find((stage) => stage?.name === currentStage) : null;
    byId('fieldCurrentStage').textContent = `${shown(currentStage)}${currentStageResult?.status ? ` · ${shown(currentStageResult.status)}` : ''}`;
    byId('fieldLastCompleted').textContent = `Last completed stage ${lastCompletedStage(snapshot)}`;
    byId('fieldControllerStatus').textContent = `현재 controller ${shown(control?.state)} · run ${shown(control?.active?.id)} · owner alive ${shown(control?.owner_alive)} · 표시 snapshot command ${shown(snapshot?.command?.state)}`;
    byId('fieldLastAttempt').textContent = `Last attempt ${compactAttempt(snapshot?.command?.last_attempt ?? active.last_attempt)}`;
    byId('fieldFirstFailure').textContent = firstFailure(snapshot);

    const joints = Array.isArray(snapshot?.joints) ? snapshot.joints : [];
    const torque = joints.map((joint) => joint?.torque).filter((value) => value !== undefined && value !== null);
    const torqueSummary = torque.length === joints.length && joints.length
      ? `${torque.length}/${joints.length} provided · ${[...new Set(torque.map(String))].join('/')}` : 'UNKNOWN';
    const grasp = snapshot?.grasp?.status ?? snapshot?.grasp?.value ?? active.grasp?.status ?? active.grasp;
    byId('fieldTorqueGrasp').textContent = `Torque ${torqueSummary} · Grasp ${shown(grasp)}${grasp === undefined ? ' (HOLDING은 grasp 증거가 아님)' : ''}`;
    const normalReturn = snapshot?.normal_return_confirmed ?? snapshot?.command?.terminal?.normal_return_confirmed
      ?? active.normal_return_confirmed;
    const safeShutdown = snapshot?.safe_shutdown ?? snapshot?.command?.safe_shutdown ?? active.safe_shutdown;
    byId('fieldNormalReturn').textContent = `Normal return ${shown(normalReturn)}`;
    byId('fieldSafeShutdown').textContent = `Safe shutdown ${shown(safeShutdown)} · STOP 요청과 별도`;
    renderReadback(snapshot);
    renderCameras(snapshot);

    document.querySelectorAll('[data-field-target]').forEach((button) => {
      const target = byId(button.dataset.fieldTarget);
      button.disabled = !target || target.disabled || target.hidden;
    });
  }

  function renderScopes(data) {
    if (!data || typeof data !== 'object' || !Array.isArray(data.scopes)) throw new Error('범위 응답의 scopes 형식이 올바르지 않습니다.');
    panelState.catalog = data;
    byId('fieldError').hidden = true;
    renderCatalog();
  }

  async function refreshScopes() {
    if (panelState.busy) return;
    panelState.busy = true;
    byId('fieldRefresh').disabled = true;
    try {
      const response = await fetch('/api/control/scopes', { headers: { Accept: 'application/json' } });
      const text = await response.text();
      let data;
      try { data = JSON.parse(text); } catch { data = null; }
      if (!response.ok) throw new Error(`HTTP ${response.status}: ${data?.detail || text.trim() || response.statusText}`);
      renderScopes(data);
    } catch (error) {
      panelState.catalog = null;
      byId('fieldError').textContent = `실행 범위 정보를 불러오지 못했습니다: ${error.message}`;
      byId('fieldError').hidden = false;
      byId('fieldCatalogState').textContent = '범위 계약 조회 실패 · 실행 준비 비활성';
      renderScope();
    } finally {
      panelState.busy = false;
      byId('fieldRefresh').disabled = false;
    }
  }

  byId('fieldScopeSelect').addEventListener('change', renderScope);
  byId('fieldRefresh').addEventListener('click', refreshScopes);
  byId('fieldPrepare').addEventListener('click', () => {
    const scope = scopeById(byId('fieldScopeSelect').value);
    if (prepareAllowed(scope)) panelState.onPrepare?.(scope.id);
  });
  document.querySelectorAll('[data-field-target]').forEach((button) => {
    button.addEventListener('click', () => {
      const target = byId(button.dataset.fieldTarget);
      if (target && !target.disabled && !target.hidden) target.click();
    });
  });

  window.fieldPanel = Object.freeze({
    setOnPrepare(callback) { panelState.onPrepare = typeof callback === 'function' ? callback : null; },
    update(control, snapshot, snapshotError = null) { panelState.control = control ?? null; panelState.snapshot = snapshot ?? null; panelState.snapshotError = snapshotError; renderScope(); renderEvidence(); },
    refresh: refreshScopes,
    renderScopes,
    selectedScope() { return scopeById(byId('fieldScopeSelect').value); },
  });

  renderScope();
  renderEvidence();
  void refreshScopes();
})();
