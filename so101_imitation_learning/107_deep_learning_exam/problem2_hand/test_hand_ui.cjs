// Run: node test_hand_ui.cjs — mocked DOM/canvas only, no camera/network/training.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const elements = new Map(), canvasCalls = [];
function element() { return {textContent:'',disabled:false,checked:false,style:{},children:[],
  classList:{toggle(){}},addEventListener(){},add(){},
  append(...c){this.children.push(...c);},replaceChildren(...c){this.children=c;}}; }
const get = id => {if(!elements.has(id))elements.set(id,element());return elements.get(id);};
const canvas = new Proxy({}, {get:(_,name)=>(...args)=>canvasCalls.push([name,...args]),set:()=>true});
get('handCanvas').getContext=()=>canvas;
get('cropCanvas').getContext=()=>canvas;
get('cropCanvas').toDataURL=(type,quality)=>{assert.equal(type,'image/jpeg');assert.equal(quality,.92);return 'data:image/jpeg;base64,MOCK';};
let clock=0, scheduled=0, onSleep=()=>{};
const context=vm.createContext({document:{getElementById:get,createElement:element,addEventListener(){}},
  window:{addEventListener(){}},performance:{now:()=>clock},AbortController,AbortSignal,URL,
  setTimeout(callback,ms){if(++scheduled>100)throw Error("Mock loop exceeded 100 steps");clock+=ms;onSleep(ms);queueMicrotask(callback);},
  fetch(){throw Error('Network forbidden in UI checks');},
  navigator:{mediaDevices:{getUserMedia(){throw Error('Camera forbidden');}}}});
const source=fs.readFileSync(path.join(__dirname,'hand.html'),'utf8');
const script=source.match(/<script>([\s\S]*?)<\/script>/)[1];
const startup='drawHand();renderControls();pollStatus();setInterval(pollStatus,2000);';
assert.equal(script.split(startup).length,2);
vm.runInContext(script.replace(startup,''),context);
const run=code=>vm.runInContext(code,context);
context.Option=function(text,value){return {...element(),textContent:text,value};};
const joints=()=>Array.from(run('state.joints'));
const result=()=>({label:'rock',confidence:.9,probabilities:[.03,.9,.03,.04],latency_ms:4,z_mode:'zero',
  actions:Array.from({length:8},(_,i)=>Array(10).fill(i<4?1:1.4))});
function validate(value){context.prediction=value;run('validatePrediction(prediction)');}
validate(result());
const bounds=result();bounds.actions[0][0]=0;bounds.actions[0][1]=1.4;validate(bounds);
for(const mutate of [r=>r.actions.pop(),r=>r.actions[0].pop(),r=>r.actions[0][0]=-.01,
  r=>r.actions[0][0]=1.4001,r=>r.actions[0][0]=NaN,r=>r.confidence=Infinity,
  r=>r.confidence=1.1,r=>r.label='invalid',r=>r.z_mode='posterior',r=>r.probabilities[0]=-1]){
  const bad=result();mutate(bad);assert.throws(()=>validate(bad));
}
run('drawHand()');
assert.equal(get('jointRows').children.length,5);
assert.equal(get('jointRows').children.reduce((n,r)=>n+r.children.length-1,0),10);
assert.equal(canvasCalls.filter(c=>c[0]==='arc').length,20,'Ten joint circles, each with inner/outer arc');
run('state.joints=Array(10).fill(.2);applyAction(Array(10).fill(1))');
assert.ok(joints().every(q=>Math.abs(q-.48)<1e-12),'q must use .35 response rate');

function prepare(){
  onSleep=()=>{};scheduled=0;
  run('resetHand();state.online=true;state.run="mock-run";state.data={training:{running:false},ready_to_train:false};state.stream={getVideoTracks:()=>[{readyState:"live",muted:false}]};renderControls();');
  Object.assign(get('video'),{videoWidth:1280,videoHeight:720,readyState:2});
}
async function checks(){
  run('state.data={source_sessions:10,ready_to_train:true,training:{running:false},models:[{run:"newest"},{run:"best"},{run:"oldest"}],runs:[{id:"newest",act_metrics:{val_mse:.3}},{id:"best",act_metrics:{val_mse:.1}},{id:"oldest",act_metrics:{val_mse:.2}}]};state.run="";renderStatus()');
  assert.equal(run('state.run'),'best','Initial selection must minimize validation MSE, not choose oldest run');
  run('state.run="oldest";renderStatus()');
  assert.equal(run('state.run'),'oldest','Polling must preserve an existing user choice');
  run('state.run="removed";renderStatus()');
  assert.equal(run('state.run'),'best','Missing selection must fall back to best available validation MSE');
  run('state.data.models=[{run:"newest",teacher_version:"mixed-open-start-v2"},{run:"best",teacher_version:"random-start-v1"},{run:"oldest",teacher_version:"mixed-open-start-v2"}];state.run="";renderStatus()');
  assert.equal(run('state.run'),'oldest','Default must minimize MSE only within newest teacher version');
  run('state.run="best";renderStatus()');
  assert.equal(run('state.run'),'best','Explicit legacy-model choice must be preserved');
  run('state.data.models=[{run:"newest"},{run:"best"},{run:"oldest"}];state.run="";renderStatus()');
  assert.equal(run('state.run'),'best','Missing teacher version must fall back to one legacy cohort');
  prepare();
  let requests=[];
  context.mockAPI=async(_,options)=>{requests.push(JSON.parse(options.body));if(requests.length===2)run('stopMotion()');return result();};
  run('api=(...args)=>mockAPI(...args)');await run('startMotion()');
  const expected=1-Math.pow(.65,4);
  assert.equal(requests.length,2,'Four actions must be followed by a new observation');
  assert.ok(requests[0].joints.every(q=>q===0));
  assert.ok(requests[1].joints.every(q=>Math.abs(q-expected)<1e-12),'Second input must contain updated q, not action targets');
  assert.ok(joints().every(q=>Math.abs(q-expected)<1e-12),'Only first four of eight actions may execute');

  for(const [label,confidence] of [['none',.95],['paper',.69]]){
    prepare();let count=0,held;
    context.mockAPI=async()=>{count++;const r=result();if(count===2){r.label=label;r.confidence=confidence;held=joints();}return r;};
    onSleep=ms=>{if(ms===400){assert.match(get('motionStatus').textContent,/WAIT/);assert.deepEqual(joints(),held);run('stopMotion()');}};
    await run('startMotion()');assert.equal(count,2);assert.deepEqual(joints(),held);
    assert.equal(get('observed').textContent,'—');assert.equal(get('actionRows').children.length,0);
  }

  prepare();let deliver;
  context.mockAPI=()=>new Promise(resolve=>{deliver=resolve;});
  const pending=run('startMotion()');run('resetHand()');deliver(result());await pending;
  assert.ok(joints().every(q=>q===0),'Late response must not move a reset hand');
  assert.equal(run('state.running'),false);assert.equal(get('observed').textContent,'—');
  assert.equal(get('actionRows').children.length,0);

  prepare();let steps=0;
  context.mockAPI=async()=>result();
  onSleep=ms=>{if(ms===100 && ++steps===2)run('stopMotion()');};
  await run('startMotion()');
  assert.ok(joints().every(q=>Math.abs(q-.35)<1e-12),'Stop between commands must cancel the pending next action');

  for(const [w,h] of [[1280,720],[720,1280]]){
    prepare();Object.assign(get('video'),{videoWidth:w,videoHeight:h});canvasCalls.length=0;
    run('updateROI();frameImage()');const side=Math.min(w,h)*.7;
    const near=(a,b)=>assert.ok(Math.abs(a-b)<1e-8);
    near(parseFloat(get('roi').style.width)*w/100,side);near(parseFloat(get('roi').style.height)*h/100,side);
    near(parseFloat(get('roi').style.left)*w/100,(w-side)/2);near(parseFloat(get('roi').style.top)*h/100,(h-side)/2);
    assert.deepEqual(canvasCalls,[['setTransform',-1,0,0,1,224,0],['drawImage',get('video'),(w-side)/2,(h-side)/2,side,side,0,0,224,224],['setTransform',1,0,0,1,0,0]]);
  }
  run('state.stream=null');assert.throws(()=>run('frameImage()'),/카메라 프레임/);
  console.log('PASS: bounded 8x10 actions, ten joints, .35 lag, four-action reobserve, WAIT, stop/reset stale cancellation, mirrored ROI — no camera/network');
}
checks().catch(error=>{console.error(error);process.exitCode=1;});
