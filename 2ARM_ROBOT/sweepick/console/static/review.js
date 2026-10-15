'use strict';

(() => {
  const CAMERA_SLOTS = [
    ['top_rgb', 'OS30A RGB'],
    ['top_depth', 'OS30A Depth'],
    ['left_wrist', '왼손목 RGB'],
    ['right_wrist', '오른손목 RGB'],
  ];
  const inspection = {
    runs: [], selectedRunId: null, controller: null, generation: 0, selectReplay: null,
  };
  const byId = (id) => document.getElementById(id);
  const shown = (value) => value === null || value === undefined || value === '' ? 'UNKNOWN' : String(value);
  const finite = (value) => value !== null && value !== undefined && value !== ''
    && typeof value !== 'boolean' && Number.isFinite(Number(value));
  const number = (value, digits = 3) => finite(value) ? Number(value).toFixed(digits) : 'UNKNOWN';
  const time = (value) => {
    if (!finite(value)) return 'UNKNOWN';
    const numeric = Number(value);
    return numeric > 1e9
      ? `${new Intl.DateTimeFormat('ko-KR', { dateStyle: 'short', timeStyle: 'medium', hour12: false }).format(new Date(numeric * 1000))} (${numeric.toFixed(3)} s)`
      : `${numeric.toFixed(3)} s`;
  };
  const truth = (value, yes, no) => value === true ? yes : value === false ? no : shown(value);
  const values = (value) => Array.isArray(value) ? value.map(shown).join(' · ') || '없음' : shown(value);
  const objectText = (value) => value && typeof value === 'object' ? JSON.stringify(value) : shown(value);

  function statusClass(value) {
    const status = String(value ?? '').toUpperCase();
    if (['FAIL', 'ERROR', 'REFUSED', 'MISSING', 'UNSAFE', 'NOT_HOLDING', 'NOT_VERIFIED', 'NOT_CONFIRMED'].some((word) => status.includes(word))) return 'danger';
    if (['PARTIAL', 'STALE', 'PROVISIONAL', 'INCOMPLETE'].some((word) => status.includes(word))) return 'partial';
    if (['SUCCESS', 'SUCCEEDED', 'PASS', 'PASSED', 'COMPLETE', 'COMPLETED', 'OK', 'VERIFIED', 'HOLDING', 'SAFE'].some((word) => status.includes(word))) return 'success';
    return 'neutral';
  }

  function text(id, value) {
    byId(id).textContent = shown(value);
  }

  function statusElement(value) {
    const element = document.createElement('span');
    element.className = `badge ${statusClass(value)}`;
    element.textContent = shown(value);
    return element;
  }

  function detailCard(label, value, detail) {
    const card = document.createElement('article');
    card.className = 'inspection-card';
    const heading = document.createElement('span');
    heading.className = 'meta-label'; heading.textContent = label;
    const strong = document.createElement('strong'); strong.append(statusElement(value));
    const paragraph = document.createElement('p'); paragraph.textContent = shown(detail);
    card.append(heading, strong, paragraph);
    return card;
  }

  function tableCell(value, heading = false) {
    const cell = document.createElement(heading ? 'th' : 'td');
    if (heading) cell.scope = 'row';
    cell.textContent = shown(value);
    return cell;
  }

  function safeImageUrl(value) {
    if (typeof value !== 'string' || !value) return null;
    try {
      const url = new URL(value, window.location.href);
      return url.origin === window.location.origin && ['http:', 'https:'].includes(url.protocol) ? url.href : null;
    } catch { return null; }
  }

  function renderStages(stages) {
    const body = byId('inspectionStages'); body.replaceChildren();
    if (!Array.isArray(stages) || !stages.length) {
      const row = document.createElement('tr'); const cell = tableCell('UNKNOWN · 단계 결과 없음');
      cell.colSpan = 6; row.append(cell); body.append(row); return;
    }
    stages.forEach((stage) => {
      const row = document.createElement('tr');
      const status = document.createElement('td'); status.append(statusElement(stage?.status));
      row.append(tableCell(stage?.name, true), status, tableCell(stage?.completion), tableCell(time(stage?.started_s)),
        tableCell(time(stage?.ended_s)), tableCell(stage?.detail));
      body.append(row);
    });
  }

  function jointLabel(joint, index) {
    if (!joint) return `미수신 축 ${index + 1}`;
    const side = joint.side === 'left' ? 'L' : joint.side === 'right' ? 'R' : shown(joint.side);
    return `${side}${shown(joint.id)} · ${shown(joint.name)}`;
  }

  function renderJoints(joints) {
    const body = byId('inspectionJoints'); body.replaceChildren();
    const rows = Array.isArray(joints) ? joints.slice(0, 12) : [];
    while (rows.length < 12) rows.push(null);
    rows.forEach((joint, index) => {
      const feedback = `${shown(joint?.feedback_status)} · age ${number(joint?.age_s)} s · ${time(joint?.feedback_time_s)} · ${shown(joint?.source)}`;
      const row = document.createElement('tr');
      const previous = joint?.last_acknowledged;
      const ack = previous
        ? `${shown(joint?.ack)} · 과거 ${shown(previous.ack)} goal ${shown(previous.requested_goal)} · ${shown(previous.stage)} · ${time(previous.recorded_at_s)}` : shown(joint?.ack);
      const ackCell = tableCell(ack); ackCell.className = 'inspection-ack';
      row.append(tableCell(jointLabel(joint, index), true), tableCell(joint?.actual), tableCell(joint?.goal),
        tableCell(joint?.requested_goal), tableCell(joint?.goal_error_ticks), ackCell, tableCell(feedback));
      body.append(row);
    });
  }

  function renderFrame(camera, frame, position) {
    const figure = document.createElement('figure'); figure.className = 'inspection-frame';
    const caption = document.createElement('figcaption');
    const label = document.createElement('strong'); label.textContent = `${position.toUpperCase()} · ${shown(frame?.status)}`;
    const stamp = document.createElement('span'); stamp.textContent = time(frame?.timestamp_s);
    caption.append(label, stamp);
    const viewport = document.createElement('div'); viewport.className = 'inspection-frame-viewport';
    const imageUrl = safeImageUrl(frame?.image_url);
    const placeholder = document.createElement('span'); placeholder.textContent = imageUrl ? '이미지 불러오는 중…' : 'UNKNOWN · 저장 프레임 없음';
    viewport.append(placeholder);
    if (imageUrl) {
      const image = document.createElement('img'); image.alt = `${shown(camera?.label)} ${position} 저장 오류 기준 프레임`;
      image.addEventListener('load', () => placeholder.hidden = true);
      image.addEventListener('error', () => { image.remove(); placeholder.hidden = false; placeholder.textContent = 'UNKNOWN · 이미지 로드 실패'; });
      image.src = imageUrl; viewport.prepend(image);
    }
    const metadata = document.createElement('p');
    metadata.textContent = `seq ${shown(frame?.sequence)} · file ${shown(frame?.file)} · sha256 ${shown(frame?.sha256)}`;
    if (imageUrl) {
      const link = document.createElement('a'); link.href = imageUrl; link.target = '_blank'; link.rel = 'noopener';
      link.textContent = ' 원본 크게 보기'; metadata.append(link);
    }
    figure.append(caption, viewport, metadata); return figure;
  }

  function renderCameras(cameras) {
    const target = byId('inspectionCameras'); target.replaceChildren();
    const supplied = Array.isArray(cameras) ? cameras : [];
    CAMERA_SLOTS.forEach(([id, fallbackLabel], index) => {
      const camera = supplied.find((item) => item?.id === id) || supplied[index] || { id, label: fallbackLabel };
      const coverage = camera?.coverage || {};
      const card = document.createElement('article'); card.className = 'inspection-camera-card';
      const header = document.createElement('header');
      const title = document.createElement('h4'); title.textContent = `${index + 1}. ${shown(camera?.label || fallbackLabel)}`;
      header.append(title, statusElement(camera?.status));
      const facts = document.createElement('p'); facts.className = 'inspection-camera-facts';
      const count = (rows) => Array.isArray(rows) ? rows.length : 'UNKNOWN';
      facts.textContent = `age ${number(camera?.age_s)} s · freshness limit ${number(camera?.freshness_limit_s)} s · last ${time(camera?.last_received_s)} · n ${shown(camera?.n)} · coverage ${shown(coverage.status)} · saved every N framesets ${shown(coverage.saved_every_n_framesets ?? camera?.saved_every_n_framesets)} · cadence allowance ${number(coverage.cadence_allowance_s)} s · internal gaps ${count(coverage.internal_gaps)} · sequence gaps ${count(coverage.sequence_gaps)} · missing seq ${values(coverage.missing_sequence_numbers)} · longest gap ${number(coverage.longest_internal_gap_s)} s`;
      const gaps = document.createElement('details');
      const gapTitle = document.createElement('summary'); gapTitle.textContent = '프레임 공백 시각·원본 기준';
      const gapText = document.createElement('pre'); gapText.textContent = JSON.stringify(coverage, null, 2);
      gaps.append(gapTitle, gapText);
      const frames = document.createElement('div'); frames.className = 'inspection-frame-pair';
      frames.append(renderFrame(camera, camera?.frames?.before, 'before'), renderFrame(camera, camera?.frames?.after, 'after'));
      card.append(header, facts, gaps, frames); target.append(card);
    });
  }

  function renderGrasp(grasp) {
    const target = byId('inspectionGrasp'); target.replaceChildren();
    const holding = grasp?.controller_holding || {};
    const object = grasp?.object_grasp || {};
    const whole = grasp?.whole_task_success || {};
    target.append(
      detailCard('Controller HOLDING · 해당 단계 기록', truth(holding.value, 'HOLDING', 'NOT_HOLDING'), `stage ${shown(holding.stage)} · ${time(holding.time_s)} · source ${shown(holding.source)}`),
      detailCard('Historical object evidence · 현재/handoff 상태 아님', truth(object.value, 'HELD_CONFIRMED', 'NOT_VERIFIED'), `${time(object.time_s)} · source ${shown(object.source)} · basis ${objectText(object.basis)} · current at failure ${shown(object.current_at_failure)} · handoff grasp confirmed ${truth(object.handoff_grasp_confirmed, 'CONFIRMED', 'NOT_CONFIRMED')} · support confirmed ${truth(object.support_confirmed, 'CONFIRMED', 'NOT_CONFIRMED')}`),
      detailCard('Whole task success · 별도 판정', truth(whole.value, 'SUCCESS', 'FAILURE'), `basis ${shown(whole.basis)}`),
    );
  }

  function renderShutdown(shutdown) {
    const target = byId('inspectionShutdown'); target.replaceChildren();
    const recovery = shutdown?.manual_recovery;
    target.append(
      detailCard('Normal return confirmed', truth(shutdown?.normal_return_confirmed, 'CONFIRMED', 'NOT_CONFIRMED'), '상위 실행의 정상 반환 증거'),
      detailCard('Torque off confirmed', truth(shutdown?.torque_off_confirmed, 'CONFIRMED', 'NOT_CONFIRMED'), `${shown(shutdown?.torque_off_source)} · ${time(shutdown?.torque_off_time_s)}`),
      detailCard('Safe shutdown', truth(shutdown?.safe_shutdown, 'SAFE', 'UNSAFE'), 'normal return·torque off와 별도 판정'),
      detailCard('Manual recovery', recovery && typeof recovery === 'object' ? 'OPERATOR_RECORDED' : 'UNKNOWN', objectText(recovery)),
    );
  }

  function renderReadiness(readiness) {
    const target = byId('inspectionReadiness'); target.replaceChildren();
    const limited = readiness?.limited_scope || {};
    const mission = readiness?.whole_mission || {};
    target.append(
      detailCard(`Limited scope · ${shown(limited.scope)}`, limited.status, `basis ${shown(limited.basis)}`),
      detailCard('Whole mission', mission.status, `basis ${shown(mission.basis)}`),
    );
  }

  function renderList(id, items, emptyText) {
    const list = byId(id); list.replaceChildren();
    const safeItems = Array.isArray(items) && items.length ? items : [emptyText];
    safeItems.forEach((item) => { const element = document.createElement('li'); element.textContent = shown(item); list.append(element); });
  }

  function renderSources(sources) {
    const body = byId('inspectionSources'); body.replaceChildren();
    if (!Array.isArray(sources) || !sources.length) {
      const row = document.createElement('tr'); const cell = tableCell('UNKNOWN · 증거 파일 목록 없음');
      cell.colSpan = 4; row.append(cell); body.append(row); return;
    }
    sources.forEach((source) => {
      const row = document.createElement('tr');
      row.append(tableCell(source?.file, true), tableCell(source?.sha256), tableCell(source?.mtime_ns), tableCell(source?.bytes)); body.append(row);
    });
  }

  function renderInspection(data) {
    text('inspectionSelectedRun', `선택된 저장 run: ${shown(data.run_id)}`);
    text('inspectionReference', `실패/기준 시각: ${time(data.reference_time_s)} · 저장 증거 (LIVE 아님)`);
    const failure = data.first_failure;
    text('inspectionFirstFailure', failure ? `${shown(failure.gate)} · ${time(failure.time_s)}` : 'FIRST FAILURE 없음');
    text('inspectionFirstFailureDetail', failure ? `${shown(failure.detail)} · source ${shown(failure.source)}` : '기록된 task failure 없음');
    text('inspectionRecorderStatus', data.recorder?.status);
    text('inspectionRecorderDetail', `coverage ${shown(data.recorder?.coverage_status)} · errors ${values(data.recorder?.errors)} · recorder errors ${values(data.recorder?.recorder_errors)} · events not recorded ${objectText(data.recorder?.events_not_recorded)}`);
    text('inspectionWindow', `${time(data.window?.start_s)} → ${time(data.window?.end_s)}`);
    text('inspectionWindowDetail', `schema ${shown(data.schema)} · events ${shown(data.window?.event_count)} · motors ${shown(data.window?.motor_sample_count)} (전 ${shown(data.window?.motor_before_count)} / 후 ${shown(data.window?.motor_after_count)}) · ${shown(data.window?.selection_basis)}`);
    text('inspectionDeviceAccess', truth(data.device_access, 'TRUE', 'FALSE'));
    byId('inspectionDeviceAccess').className = data.device_access === false ? 'inspection-safe' : 'inspection-warning';
    renderGrasp(data.grasp);
    renderStages(data.stage_results);
    renderJoints(data.joints);
    renderCameras(data.cameras);
    const alignment = data.time_alignment || {};
    text('inspectionAlignment', `Host receipt spread ${number(alignment.host_receipt_spread_s)} s`);
    text('inspectionAlignmentDetail', `basis ${shown(alignment.basis)} · physical capture sync ${shown(alignment.physical_capture_sync)} · joint↔camera offsets ${objectText(alignment.joint_camera_offsets_s)}`);
    const observation = data.control_observation || {};
    const lastCheck = observation.last_check;
    byId('inspectionControlObservation').hidden = !lastCheck || typeof lastCheck !== 'object';
    if (lastCheck && typeof lastCheck === 'object') {
      const wristFrame = lastCheck.last_left_wrist_frame ?? lastCheck.left_wrist_frame;
      text('inspectionControlCheck', `last check ${time(lastCheck.t_host_s ?? lastCheck.time_s ?? lastCheck.timestamp_s)} · source ${shown(lastCheck.source ?? observation.source)}`);
      text('inspectionControlCheckDetail', `display frame source ${shown(lastCheck.display_frame_source ?? observation.display_frame_source)} · ok ${shown(lastCheck.ok)} · why ${objectText(lastCheck.why)} · left wrist frame ${objectText(wristFrame)}`);
    }
    renderShutdown(data.shutdown || {});
    renderReadiness(data.readiness || {});
    renderList('inspectionUnknowns', data.unknowns, '없음');
    renderSources(data.evidence_sources);
  }

  async function readJson(response) {
    const raw = await response.text();
    let data;
    try { data = JSON.parse(raw); } catch { data = null; }
    if (!response.ok) throw new Error(`HTTP ${response.status}: ${shown(data?.detail || raw.trim() || response.statusText)}`);
    if (!data || typeof data !== 'object' || Array.isArray(data)) throw new Error('검사 JSON 응답 형식이 올바르지 않습니다.');
    return data;
  }

  function updateLinks(runId) {
    const encoded = runId ? encodeURIComponent(runId) : null;
    [['inspectionJson', 'inspection'], ['inspectionPackage', 'inspection-package']].forEach(([id, resource]) => {
      const link = byId(id);
      if (!encoded) { link.removeAttribute('href'); link.setAttribute('aria-disabled', 'true'); return; }
      link.href = `/api/runs/${encoded}/${resource}`; link.removeAttribute('aria-disabled');
      if (resource === 'inspection-package') link.download = `${runId}_inspection.zip`;
    });
  }

  async function loadRun(runId) {
    inspection.controller?.abort();
    inspection.selectedRunId = runId || null;
    inspection.generation += 1;
    const generation = inspection.generation;
    updateLinks(runId);
    text('inspectionSelectedRun', `선택된 저장 run: ${shown(runId)}`);
    text('inspectionReference', '실패/기준 시각: 불러오는 중… · 저장 증거 (LIVE 아님)');
    byId('inspectionError').hidden = true;
    byId('inspectionContent').hidden = true;
    byId('inspectionLoading').hidden = false;
    byId('inspectionLoading').textContent = runId ? '저장 검사 증거를 불러오는 중…' : '검사할 Run ID를 선택하십시오.';
    if (!runId) return;
    const controller = new AbortController(); inspection.controller = controller;
    try {
      const data = await readJson(await fetch(`/api/runs/${encodeURIComponent(runId)}/inspection`, {
        signal: controller.signal, headers: { Accept: 'application/json' },
      }));
      if (generation !== inspection.generation || inspection.selectedRunId !== runId) return;
      if (data.run_id !== runId) throw new Error(`응답 run_id 불일치: ${shown(data.run_id)}`);
      if (data.status === 'NOT_AVAILABLE') throw new Error(values(data.unknowns));
      renderInspection(data);
      byId('inspectionLoading').hidden = true; byId('inspectionContent').hidden = false;
    } catch (error) {
      if (error.name === 'AbortError' || generation !== inspection.generation) return;
      byId('inspectionLoading').hidden = true;
      byId('inspectionError').textContent = `저장 검사를 불러오지 못했습니다: ${error.message}`;
      byId('inspectionError').hidden = false;
    }
  }

  function populateRuns(runs, preferredRunId) {
    const select = byId('inspectionRunSelect'); select.replaceChildren();
    if (!runs.length) {
      select.append(new Option('저장된 실행 없음', '')); select.disabled = true; void loadRun(null); return;
    }
    runs.forEach((run) => select.append(new Option(`${shown(run.id)} · ${shown(run.outcome ?? run.state)}`, run.id)));
    const current = runs.find((run) => run.id === inspection.selectedRunId);
    const replayMatch = runs.find((run) => run.replay_session_id === preferredRunId);
    const next = current || replayMatch || runs[0];
    select.value = next.id; select.disabled = false; void loadRun(next.id);
  }

  function setRuns(runs, replaySessionId) {
    inspection.runs = Array.isArray(runs) ? runs.filter((run) => typeof run?.id === 'string' && run.id) : [];
    populateRuns(inspection.runs, replaySessionId);
  }

  function selectForReplay(sessionId) {
    const match = inspection.runs.find((run) => run.replay_session_id === sessionId);
    if (!match || match.id === inspection.selectedRunId) return;
    byId('inspectionRunSelect').value = match.id; void loadRun(match.id);
  }

  byId('inspectionRunSelect').addEventListener('change', (event) => {
    const run = inspection.runs.find((item) => item.id === event.target.value);
    void loadRun(run?.id || null);
    if (run) inspection.selectReplay?.(run);
  });

  window.inspectionPanel = {
    setRuns,
    selectForReplay,
    setReplaySelector(callback) { inspection.selectReplay = typeof callback === 'function' ? callback : null; },
  };

  let documentGeneration = 0;
  async function openAnalysisFile(review, file) {
    const generation = ++documentGeneration;
    byId('analysisError').hidden = true;
    byId('analysisDocument').hidden = false;
    byId('analysisDocument').scrollIntoView({behavior: 'smooth', block: 'start'});
    text('analysisDocumentTitle', `${review.id} · ${file.name}`);
    text('analysisDocumentMeta', `SHA-256 ${file.sha256} · ${file.bytes} bytes · mtime_ns ${file.mtime_ns}`);
    text('analysisDocumentText', '문서 불러오는 중…');
    byId('analysisDownload').href = file.url;
    byId('analysisDownload').download = file.name;
    try {
      const response = await fetch(file.url);
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      const bytes = await response.arrayBuffer();
      const digest = Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256', bytes)))
        .map((value) => value.toString(16).padStart(2, '0')).join('');
      if (digest !== file.sha256) throw new Error('파일 SHA가 목록과 달라졌습니다. 페이지를 새로고침해 현재 원문을 확인하십시오.');
      const body = new TextDecoder().decode(bytes);
      if (generation !== documentGeneration) return;
      text('analysisDocumentText', body);
      text('analysisDocumentMeta', `SHA-256 VERIFIED ${digest} · ${file.bytes} bytes · mtime_ns ${file.mtime_ns}`);
    } catch (error) {
      if (generation === documentGeneration) text('analysisDocumentText', `UNKNOWN · 문서 조회 실패: ${error.message}`);
    }
  }

  async function loadAnalyses() {
    try {
      const data = await readJson(await fetch('/api/analyses'));
      const catalog = byId('analysisCatalog'); catalog.replaceChildren();
      for (const review of data.analyses || []) {
        const card = detailCard(review.title, review.verification?.software_status ?? review.verification?.status,
          `실물 재현 ${review.physical_reproduction} · 분석 기준 Run ${review.source_run_id} · 비교 Run ${review.reference_run_id}`);
        const link = document.createElement('a'); link.href = `/?session=${encodeURIComponent(review.source_run_id)}#inspectionPanel`;
        link.textContent = '분석 기준 실물 Run 보기'; card.append(link);
        const files = document.createElement('div'); files.className = 'analysis-files';
        for (const file of review.files || []) {
          const button = document.createElement('button'); button.type = 'button'; button.className = 'button secondary';
          button.textContent = file.name; button.disabled = !file.available;
          button.addEventListener('click', () => openAnalysisFile(review, file)); files.append(button);
        }
        card.append(files); catalog.append(card);
      }
    } catch (error) {
      text('analysisError', `검수 목록 조회 실패: ${error.message}`); byId('analysisError').hidden = false;
    }
  }
  void loadAnalyses();

  console.assert(truth(true, 'YES', 'NO') === 'YES' && truth(false, 'YES', 'NO') === 'NO', '검사 boolean 표시 규칙이 손상되었습니다.');
})();
