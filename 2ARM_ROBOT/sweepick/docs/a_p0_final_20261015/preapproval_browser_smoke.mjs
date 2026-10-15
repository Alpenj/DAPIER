// START pre-approval checkbox smoke test against a console copy on PORT. /api/control is a fixture; every POST is
// blocked and its body recorded. No prepare/start reaches a server.
import {spawn} from 'node:child_process';
import {readFile, writeFile, mkdir, rm} from 'node:fs/promises';
import {setTimeout as delay} from 'node:timers/promises';
import assert from 'node:assert/strict';
const PORT = process.env.PORT || '8777';
const directory = new URL('.', import.meta.url).pathname; const profile = directory + 'chromium_qa_profile';
await mkdir(profile, {recursive: true}); await rm(profile + '/DevToolsActivePort', {force: true});
const chrome = spawn('/snap/bin/chromium', ['--headless=new', '--no-sandbox', '--disable-gpu', '--disable-dev-shm-usage', '--remote-debugging-address=127.0.0.1',
  '--remote-debugging-port=0', '--user-data-dir=' + profile, '--window-size=1366,1000', 'about:blank'], {detached: true, stdio: ['ignore', 'ignore', 'pipe']});
const errors = [], results = [], posts = [];
const job = (scope) => ({id: 'QA_ONLY', execution_scope: scope, scope, state: 'PREPARED', consumed: false, phase: 'awaiting_operator_approval', session_out: '/qa/session'});
let control = {csrf_token: 'qa', execution_connected: true, state: 'PREPARED', scope: 'fixture', owner_alive: false, active: job('FULL_CHAIN_RECEIVE_CONTACT'),
  can_prepare: false, can_confirm_recovery: false, can_start: true, can_stop: false, block_reason: null};
let ws;
try {
  let port; for (let i = 0; i < 200; i++) { try { port = (await readFile(profile + '/DevToolsActivePort', 'utf8')).split('\n')[0]; break; } catch { await delay(100); } }
  assert(port, 'Chromium debug port missing');
  const target = (await (await fetch('http://127.0.0.1:' + port + '/json/list')).json()).find(t => t.type === 'page');
  ws = new WebSocket(target.webSocketDebuggerUrl); await new Promise((r, j) => { ws.addEventListener('open', r, {once: true}); ws.addEventListener('error', j, {once: true}); });
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
      else void send('Fetch.continueRequest', {requestId});
    }
  });
  const evaluate = async e => { const r = await send('Runtime.evaluate', {expression: e, returnByValue: true, awaitPromise: true}); if (r.exceptionDetails) throw Error(JSON.stringify(r.exceptionDetails)); return r.result.value; };
  const wait = async e => { for (let i = 0; i < 200; i++) { if (await evaluate(e)) return; await delay(100); } throw Error('UI timeout ' + e); };
  await send('Runtime.enable'); await send('Fetch.enable', {patterns: [{urlPattern: `http://localhost:${PORT}/api/*`, requestStage: 'Request'}]});
  await send('Page.navigate', {url: `http://localhost:${PORT}/`});
  await wait(`(()=>{try{return state.control?.state==='PREPARED' && !document.querySelector('#fieldReady').disabled}catch(e){return false}})()`);
  const startWith = async (preapprove) => {
    posts.length = 0;
    await evaluate(`(()=>{const r=document.querySelector('#fieldReady'); if(!r.checked) r.click(); const p=document.querySelector('#preapproveView'); if(p.checked!==${preapprove}) p.click(); })()`);
    await wait(`!document.querySelector('#startControl').disabled`);
    await evaluate(`document.querySelector('#startControl').click()`); await wait(`${'true'}`); await delay(600);
    return posts.find(p => p.path === '/api/control/start');
  };
  let s = await startWith(false);
  assert(s && s.body.preapprove_view === false && s.body.execution_scope === 'FULL_CHAIN_RECEIVE_CONTACT', JSON.stringify(posts));
  results.push({check: 'unchecked_sends_preapprove_view_false (existing manual approval)'});
  s = await startWith(true);
  assert(s && s.body.preapprove_view === true, JSON.stringify(posts)); results.push({check: 'checked_sends_preapprove_view_true'});
  control = {...control, active: job('LEFT_START_TO_LIFT')}; await evaluate(`refreshControl()`); await delay(500);
  assert(await evaluate(`document.querySelector('#preapproveView').disabled && !document.querySelector('#preapproveView').checked`));
  posts.length = 0; await evaluate(`document.querySelector('#startControl').click()`); await delay(600);
  s = posts.find(p => p.path === '/api/control/start');
  assert(s && s.body.preapprove_view === false && s.body.execution_scope === 'LEFT_START_TO_LIFT', JSON.stringify(posts));
  results.push({check: 'limited_scope_disables_and_never_sends_preapproval'});
  assert.deepEqual(errors, []);
  await writeFile(directory + 'PREAPPROVAL_BROWSER_SMOKE.json', JSON.stringify({passed: results.length, results, errors, blocked_posts: posts.length, real_control_posts: 0, port: PORT}, null, 2));
  console.log(JSON.stringify({passed: results.length, errors}));
} finally { try { ws?.close(); } catch {} try { process.kill(-chrome.pid); } catch {} }
