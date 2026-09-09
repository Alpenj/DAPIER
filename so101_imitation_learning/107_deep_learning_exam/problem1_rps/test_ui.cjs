// Run: node test_ui.cjs — no server, camera, model, or npm packages required.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const elements = new Map();
function element() {
  return {textContent: '', disabled: false, style: {}, children: [],
    classList: {toggle() {}}, addEventListener() {}, add() {},
    append(...children) { this.children.push(...children); },
    replaceChildren(...children) { this.children = children; },
    querySelector() { return this.small ||= element(); }};
}
const get = id => {
  if (!elements.has(id)) elements.set(id, element());
  return elements.get(id);
};
let clock = 0;
const context = vm.createContext({
  document: {getElementById: get, createElement: element, addEventListener() {}},
  window: {addEventListener() {}}, performance: {now: () => clock},
  AbortController, AbortSignal, URL, setTimeout,
  // Unexpected network or device access must fail rather than touch real data.
  fetch() { throw new Error('UI unit check must not access the network'); },
  navigator: {mediaDevices: {getUserMedia() { throw new Error('Camera forbidden'); }}},
});
const source = fs.readFileSync(path.join(__dirname, 'studio.html'), 'utf8');
const script = source.match(/<script>([\s\S]*?)<\/script>/)[1];
const startup = 'renderControls();pollStatus();setInterval(pollStatus,2000);';
assert.equal(script.split(startup).length, 2, 'Locate explicit startup before isolating UI logic');
vm.runInContext(script.replace(startup, ''), context);
const run = code => vm.runInContext(code, context);
function reset() { run('clearPrediction(); state.mode="game";'); }
function predict(label, time, confidence = .91, probabilities) {
  clock = time;
  const classes = ['scissors', 'rock', 'paper', 'none'];
  const result = {label, confidence, latency_ms: 5,
    probabilities: probabilities || classes.map(c => c === label ? .91 : .03)};
  run(`showPrediction(${JSON.stringify(result)},10)`);
}

for (const [hand, counter] of [['scissors', '✊ 바위'], ['rock', '✋ 보'], ['paper', '✌ 가위']]) {
  reset();
  predict(hand, 0); predict(hand, 400);
  assert.equal(run('state.frozen'), false);
  predict(hand, 700);
  assert.equal(get('computer').textContent, counter);
  assert.equal(run('state.frozen'), true);
}

reset();
predict('scissors', 0); predict('scissors', 700);
assert.equal(run('state.frozen'), false, '700 ms alone must not bypass 3-frame minimum');
predict('scissors', 701);
assert.equal(run('state.frozen'), true);
reset();
predict('rock', 0); predict('rock', 1); predict('rock', 2);
assert.equal(run('state.frozen'), false, '3 frames alone must not bypass 700 ms minimum');

reset();
predict('none', 0); predict('none', 350); predict('none', 700);
assert.equal(get('player').textContent, '없음');
assert.equal(get('computer').textContent, '대기');
assert.equal(run('state.frozen'), false, 'Stable no-hand must keep waiting, not finish a round');
predict('rock', 800, .4, [.3, .4, .2, .1]);
assert.equal(get('player').textContent, '판단 중', 'Uncertainty must clear stale no-hand display');
assert.equal(run('state.candidate'), null);
assert.equal(run('state.frames'), 0);

reset();
predict('paper', 0); predict('paper', 400);
predict('paper', 690, .69, [.1, .1, .69, .11]);
predict('paper', 700); predict('paper', 1050);
assert.equal(run('state.frozen'), false, 'Low confidence must restart the stability clock');
predict('paper', 1400);
assert.equal(run('state.frozen'), true);
reset();
predict('scissors', 0, .71, [.51, .4, .06, .03]);
assert.equal(run('state.candidate'), null, 'Small probability margin must reject uncertain response');
reset();
predict('scissors', 0); predict('scissors', 400); predict('rock', 700);
assert.equal(run('state.frames'), 1, 'Changing hand must restart stability accumulation');
assert.equal(get('computer').textContent, '대기');
run('stopWork()');
assert.equal(run('state.mode'), 'idle');
assert.equal(get('player').textContent, '—');
assert.equal(get('computer').textContent, '대기');

async function checkCrop() {
  const video = get('video');
  const calls = [];
  get('cropCanvas').getContext = () => ({
    setTransform(...args) { calls.push(['transform', ...args]); },
    drawImage(...args) { calls.push(['draw', ...args]); },
  });
  get('cropCanvas').toBlob = (callback, type, quality) => {
    assert.equal(type, 'image/jpeg'); assert.equal(quality, .92); callback({mock: true});
  };
  for (const [w, h] of [[1280, 720], [720, 1280]]) {
    Object.assign(video, {videoWidth: w, videoHeight: h, readyState: 2});
    run('state.stream={getVideoTracks:()=>[{readyState:"live",muted:false}]};updateROI()');
    calls.length = 0;
    await run('frameBlob()');
    const side = Math.min(w, h) * .7;
    const near = (a, b) => assert.ok(Math.abs(a-b) < 1e-8, `${a} != ${b}`);
    near(parseFloat(get('roi').style.width) / 100 * w, side);
    near(parseFloat(get('roi').style.height) / 100 * h, side);
    near(parseFloat(get('roi').style.left) / 100 * w, (w-side)/2);
    near(parseFloat(get('roi').style.top) / 100 * h, (h-side)/2);
    assert.deepEqual(calls[0], ['transform', -1, 0, 0, 1, 224, 0]);
    assert.deepEqual(calls[1], ['draw', video, (w-side)/2, (h-side)/2, side, side, 0, 0, 224, 224]);
    assert.deepEqual(calls[2], ['transform', 1, 0, 0, 1, 0, 0]);
  }
  run('state.stream=null');
  await assert.rejects(() => run('frameBlob()'), /카메라 프레임/);
}
checkCrop().then(() => console.log('PASS: counter mapping, none/uncertain reset, time+frame stability, mirrored shared ROI — no camera/network'))
  .catch(error => { console.error(error); process.exitCode = 1; });
