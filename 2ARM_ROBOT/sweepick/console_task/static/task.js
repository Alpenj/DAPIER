// Task panel (2026-10-18): run the taught pick on one arm, record the taught poses, watch the steps and the log.
(() => {
  const $ = (id) => document.getElementById(id);
  const POSE_LABEL = { grasp: '① 집기 자세', above: '② 블록 바로 위', pregrasp: '③ pregrasp(6–7 cm)', carry: '④ 운반 자세(블록 쥔 채 접기)' };
  const STEP_LABEL = { pregrasp: 'pregrasp', above: '블록 바로 위', place_check: '블록 위치 확인', grasp: '집기 자세', close: '그리퍼 닫기', lift: '들기', carry: '운반 자세', carry_hold: '운반 중 쥠 감시', back_over_place: '놓을 자리 위로', lower: '내려놓기', open: '그리퍼 열기', retreat: '후퇴', home: '복귀·해제' };
  const OK = new Set(['HOLDING', 'TARGET_NOT_REACHED_HOLDING', 'GRIP_CONTACT_HOLDING', 'RETURNED_RELEASED', 'PLANNED', 'HELD']);
  let token = null;
  async function tok() {
    if (token) return token;
    const r = await fetch('/api/control'); const j = await r.json(); token = j.csrf_token; return token;
  }
  async function post(path, body) {
    const r = await fetch(path, { method: 'POST', headers: { 'content-type': 'application/json', 'x-sweepick-token': await tok() }, body: JSON.stringify(body || {}) });
    const j = await r.json().catch(() => ({}));
    if (!r.ok) throw new Error(j.detail || r.statusText);
    return j;
  }
  function msg(text, bad) { const m = $('taskMessage'); m.textContent = text; m.hidden = !text; m.className = 'banner ' + (bad ? 'error' : 'info'); }
  async function run(side) {
    const msgText = side === 'auto' ? '영상 정렬 자동 실행: 헤드로 블록을 찾고 성공 가능성이 높은 팔을 골라, 손목 화면을 가르친 화면에 맞춰 집습니다.\n블록은 아무 곳(팔이 닿는 범위)에 두면 됩니다. 비상정지가 준비됐나요?' : `${side === 'left' ? '왼팔' : '오른팔'} task를 실행합니다.\n블록이 기준 자세를 가르친 자리에 있고, 비상정지(전원선)가 준비됐나요?`;
    if (!confirm(msgText)) return;
    try { await post('/api/task/start', { side, record: $('taskRecord').checked }); msg(''); } catch (e) { msg(e.message, true); }
    refresh();
  }
  async function teach(side, pose) {
    if (!confirm(`${side === 'left' ? '왼팔' : '오른팔'} ${POSE_LABEL[pose]}를 기록합니다.\n토크가 꺼진 팔을 손으로 그 자세에 잡고 있나요?`)) return;
    try { const r = await post('/api/task/teach', { side, pose }); msg(`기록됨: ${side} ${POSE_LABEL[pose]}`); } catch (e) { msg(e.message, true); }
    refreshRuns();
  }
  function stepRow(s) {
    const good = OK.has(s.result); const cls = s.result ? (good ? 'ok' : 'bad') : '';
    return `<li class="task-step ${cls}"><b>${STEP_LABEL[s.step] || s.step}</b> <code>${s.result || '…'}</code>${s.note ? `<small>${s.note.replace(/</g, '&lt;')}</small>` : ''}</li>`;
  }
  async function refresh() {
    try {
      const s = await (await fetch('/api/task/status')).json();
      const running = !!s.running;
      ['taskLeft', 'taskRight', 'taskAuto'].forEach((id) => { $(id).disabled = running; });
      $('taskStop').disabled = !running;
      $('taskState').textContent = s.job ? (running ? `실행 중 · ${s.job.side} · ${Math.round(s.job.elapsed_s)} s` : `끝남 · ${s.job.side} · ${s.result || '결과 없음'}`) : '대기';
      $('taskState').className = 'badge ' + (running ? 'warn' : (s.result && /RETURNED_RELEASED/.test(s.result) ? 'ok' : (s.job ? 'neutral' : 'neutral')));
      $('taskGripped').textContent = s.job ? (s.gripped ? '잡음' : (running ? '…' : '못 잡음')) : '—';
      $('taskSteps').innerHTML = (s.steps || []).map(stepRow).join('');
      $('taskLog').textContent = (s.log || []).join('\n');
    } catch (e) { /* page keeps the last view */ }
  }
  async function refreshRuns() {
    try {
      const r = await (await fetch('/api/task/runs')).json();
      $('taskDisk').textContent = `로컬 여유 ${r.disk.free_gb} GB (${r.disk.used_pct}% 사용)`;
      $('taskRuns').innerHTML = r.runs.map((x) => `<tr><td>${x.name}</td><td>${x.result || '—'}</td><td>${x.gripped ? '예' : '아니오'}</td><td>${x.size_mb} MB</td><td>${x.analyzed ? '분석됨' : '대기'}</td><td>${x.offloaded ? 'work6' : '로컬'}</td></tr>`).join('') || '<tr><td colspan="6">아직 실행 기록 없음</td></tr>';
      const pending = r.runs.filter((x) => !x.offloaded).length;
      $('taskPending').textContent = pending ? `분석·work6 전송 대기 ${pending}건 — Claude에게 "task 데이터 분석해줘"라고 요청하세요` : '대기 중인 데이터 없음';
      for (const side of ['left', 'right']) for (const pose of ['grasp', 'above', 'pregrasp', 'carry']) {
        const b = $(`teach_${side}_${pose}`); if (b) b.classList.toggle('done', !!(r.refs[side] || {})[pose]);
      }
    } catch (e) { /* keep */ }
  }
  document.addEventListener('DOMContentLoaded', () => {
    $('taskLeft').onclick = () => run('left');
    $('taskRight').onclick = () => run('right');
    $('taskAuto').onclick = () => run('auto');
    $('taskStop').onclick = async () => { try { await post('/api/task/stop', {}); msg('정지 요청을 보냈습니다. 팔은 그 자리에서 버팁니다.'); } catch (e) { msg(e.message, true); } };
    for (const side of ['left', 'right']) for (const pose of ['grasp', 'above', 'pregrasp', 'carry']) {
      const b = $(`teach_${side}_${pose}`); if (b) b.onclick = () => teach(side, pose);
    }
    refresh(); refreshRuns(); setInterval(refresh, 1000); setInterval(refreshRuns, 5000);
  });
})();
