// Console-17 browser checks against a console copy on PORT. /api/control and /api/control/scopes are answered by this
// script (fixtures); every POST is blocked and recorded. No real prepare/start/stop reaches any server.
import {spawn} from 'node:child_process';
import {readFile, writeFile, mkdir} from 'node:fs/promises';
import {setTimeout as delay} from 'node:timers/promises';
import assert from 'node:assert/strict';

const PORT = process.env.PORT || '8775';
const directory = new URL('.', import.meta.url).pathname;
const profile = directory + 'chromium_qa_profile';
await mkdir(profile, {recursive: true});
const chrome = spawn('/snap/bin/chromium', ['--headless=new', '--no-sandbox', '--disable-gpu', '--disable-dev-shm-usage',
  '--remote-debugging-address=127.0.0.1', '--remote-debugging-port=0', '--user-data-dir=' + profile, '--window-size=1366,1000', 'about:blank'],
  {detached: true, stdio: ['ignore', 'ignore', 'pipe']});
let diagnostics = ''; chrome.stderr.on('data', c => { diagnostics += c; });
const errors = [], results = [], posts = [];
const limitedScope = (supported) => ({id: 'LEFT_START_TO_LIFT', label: 'LEFT', supported, can_prepare: supported,
  backend_status: supported ? 'SUPPORTED_CONSOLE17_LIMITED_BACKEND' : 'BACKEND_CHANGE_REQUIRED', included_stages: ['LIFT', 'LIMITED_SCOPE_TERMINAL'],
  excluded_stages: ['CARRY'], reason: 'fixture', termination: 'fixture'});
const fullScope = {id: 'FULL_CHAIN_RECEIVE_CONTACT', label: 'FULL', supported: true, can_prepare: true, backend_status: 'SUPPORTED_EXISTING_BACKEND', included_stages: [], excluded_stages: [], reason: 'fixture', termination: 'fixture'};
let scopes = {schema: 'sweepick.operation-contract.v1', checked_at: 'fixture', scopes: [fullScope, limitedScope(true)], owner: {state: 'IDLE'}, sources: [], observation: {}, calibration: {}, device_access: false};
const idle = {csrf_token: 'qa', execution_connected: true, state: 'IDLE', scope: 'fixture', owner_alive: false, active: null, can_prepare: true, can_confirm_recovery: false, can_start: false, can_stop: false, block_reason: null};
const holding = {...idle, state: 'LIMITED_HOLDING', can_prepare: false, can_confirm_recovery: true, block_reason: 'LEFT_START_TO_LIFT 종료 LEFT_LIFT_HELD_CONFIRMED · 정상 HOME/OFF 아님',
  active: {id: 'QA_LIMITED', execution_scope: 'LEFT_START_TO_LIFT', scope: 'LEFT_START_TO_LIFT', state: 'LIMITED_HOLDING', consumed: true, phase: 'ended', session_out: '/qa/session',
    limited_terminal: {terminal: 'LEFT_LIFT_HELD_CONFIRMED', last_stage: {name: 'lift', controller_result: 'HOLDING'}, last_actual: {torque: {gripper: 1, elbow_flex: 1}},
      normal_home: false, torque_off_confirmed: false, safe_shutdown: 'NOT_PERFORMED', operator_required: true, next_run_allowed: false}}};
let control = idle;
let ws;
try {
  let port;
  for (let i = 0; i < 200; i++) { try { port = (await readFile(profile + '/DevToolsActivePort', 'utf8')).split('\n')[0]; break; } catch { await delay(100); } }
  assert(port, 'Chromium debug port missing: ' + diagnostics);
  const target = (await (await fetch('http://127.0.0.1:' + port + '/json/list')).json()).find(t => t.type === 'page');
  ws = new WebSocket(target.webSocketDebuggerUrl);
  await new Promise((resolve, reject) => { ws.addEventListener('open', resolve, {once: true}); ws.addEventListener('error', reject, {once: true}); });
  let serial = 0; const pending = new Map();
  const send = (method, params = {}) => new Promise((resolve, reject) => { const id = ++serial; pending.set(id, {resolve, reject}); ws.send(JSON.stringify({id, method, params})); });
  const answer = (requestId, body) => send('Fetch.fulfillRequest', {requestId, responseCode: 200, responseHeaders: [{name: 'Content-Type', value: 'application/json'}], body: Buffer.from(JSON.stringify(body)).toString('base64')});
  ws.addEventListener('message', event => {
    const m = JSON.parse(event.data);
    if (m.id) { const t = pending.get(m.id); pending.delete(m.id); m.error ? t.reject(m.error) : t.resolve(m.result); }
    if (m.method === 'Runtime.exceptionThrown') errors.push(m.params.exceptionDetails.exception?.description || m.params.exceptionDetails.text);
    if (m.method === 'Fetch.requestPaused') {
      const {request, requestId} = m.params; const path = new URL(request.url).pathname;
      if (request.method !== 'GET') { posts.push({path, body: request.postData ? JSON.parse(request.postData) : null}); void send('Fetch.failRequest', {requestId, errorReason: 'BlockedByClient'}); }
      else if (path === '/api/control') void answer(requestId, control);
      else if (path === '/api/control/scopes') void answer(requestId, scopes);
      else void send('Fetch.continueRequest', {requestId});
    }
  });
  const evaluate = async e => { const r = await send('Runtime.evaluate', {expression: e, returnByValue: true, awaitPromise: true}); if (r.exceptionDetails) throw Error(JSON.stringify(r.exceptionDetails)); return r.result.value; };
  const wait = async e => { for (let i = 0; i < 200; i++) { if (await evaluate(e)) return; await delay(100); } throw Error('UI timeout ' + e); };
  const shot = async name => { const r = await send('Page.captureScreenshot', {format: 'png'}); await writeFile(directory + name + '.png', Buffer.from(r.data, 'base64')); };
  await send('Runtime.enable'); await send('Page.enable');
  await send('Fetch.enable', {patterns: [{urlPattern: `http://localhost:${PORT}/api/*`, requestStage: 'Request'}]});
  await send('Page.navigate', {url: `http://localhost:${PORT}/`});
  await wait(`!!window.fieldPanel && document.querySelector('#fieldCatalogState').textContent.includes('fixture')`);

  await evaluate(`document.querySelector('#fieldScopeSelect').value='LEFT_START_TO_LIFT'; document.querySelector('#fieldScopeSelect').dispatchEvent(new Event('change'))`);
  await wait(`!document.querySelector('#fieldPrepare').disabled`);
  assert(await evaluate(`document.querySelector('#fieldPrepare').textContent.includes('LEFT_START_TO_LIFT 제한 준비')`));
  assert(await evaluate(`document.querySelector('#fieldScopeMessage').textContent.includes('전용 승인')`));
  results.push({check: 'limited_scope_prepare_enabled_only_when_backend_supported'});

  await evaluate(`document.querySelector('#fieldPrepare').click()`);
  await wait(`document.querySelector('#rerunDialog').open && document.querySelector('#rerunTitle').textContent.includes('LEFT_START_TO_LIFT')`);
  await wait(`!document.querySelector('#prepareControl').disabled`);
  await evaluate(`document.querySelector('#prepareControl').click()`);
  await wait(`${JSON.stringify('x')} && true`); await delay(500);
  const prepare = posts.find(p => p.path === '/api/control/prepare');
  assert(prepare && prepare.body.execution_scope === 'LEFT_START_TO_LIFT', JSON.stringify(posts));
  results.push({check: 'limited_prepare_request_carries_its_scope (POST blocked)'});
  await evaluate(`document.querySelector('#rerunDialog').close()`);

  scopes = {...scopes, scopes: [fullScope, limitedScope(false)]};
  await evaluate(`window.fieldPanel.refresh()`); await delay(400);
  await evaluate(`document.querySelector('#fieldScopeSelect').value='LEFT_START_TO_LIFT'; document.querySelector('#fieldScopeSelect').dispatchEvent(new Event('change'))`);
  await wait(`document.querySelector('#fieldPrepare').disabled && document.querySelector('#fieldPrepare').textContent.includes('준비할 수 없음')`);
  results.push({check: 'limited_scope_disabled_when_backend_missing'});

  control = holding;
  await evaluate(`refreshControl()`);
  await wait(`document.querySelector('#executionState').textContent.includes('LIMITED_HOLDING')`);
  const view = await evaluate(`({badge: document.querySelector('#executionState').className, terminal: document.querySelector('#limitedTerminal').textContent,
    terminalHidden: document.querySelector('#limitedTerminal').hidden, start: document.querySelector('#startControl').textContent, startDisabled: document.querySelector('#startControl').disabled,
    prepareDisabled: document.querySelector('#prepareControl').disabled, recoveryShown: !document.querySelector('#rerunRecovery').hidden})`);
  assert(view.badge.includes('danger') && !view.terminalHidden && view.terminal.includes('LEFT_LIFT_HELD_CONFIRMED') && view.terminal.includes('Torque OFF 확인 false'), JSON.stringify(view));
  assert(view.terminal.includes('정상 HOME false') && view.terminal.includes('모터 정지/OFF 증거가 아님'), JSON.stringify(view));
  assert(view.start.includes('수동 복구 전 새 실행 차단') && view.startDisabled && view.prepareDisabled && view.recoveryShown, JSON.stringify(view));
  results.push({check: 'limited_holding_terminal_shown_and_next_run_blocked'});
  for (const width of [420, 1366]) {
    await send('Emulation.setDeviceMetricsOverride', {width, height: 1000, deviceScaleFactor: 1, mobile: width < 600});
    await delay(300);
    assert(await evaluate(`document.documentElement.scrollWidth <= window.innerWidth + 1`), 'horizontal overflow at ' + width);
    await shot(`console17_limited_holding_${width}`);
    results.push({check: 'limited_holding_fits_viewport', width});
  }
  assert.deepEqual(errors, []);
  assert(posts.every(p => p.path === '/api/control/prepare'), JSON.stringify(posts));
  await writeFile(directory + 'CONSOLE17_BROWSER_QA.json', JSON.stringify({passed: results.length, results, errors, blocked_posts: posts, real_control_posts: 0, port: PORT}, null, 2));
  console.log(JSON.stringify({passed: results.length, errors, blocked_posts: posts.length}));
} finally {
  try { ws?.close(); } catch {}
  try { process.kill(-chrome.pid); } catch {}
}
