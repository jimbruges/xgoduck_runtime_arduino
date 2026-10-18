const $=id=>document.getElementById(id);
const MOTORS=[10,11,12,13,14,20,21,22,23,24,30,31,32,33,34];
// Positive direction of each joint in the Microduck model (MJCF joint axes at the zero pose,
// robot frame x forward, y left, z up). The policy expects measured angles to increase.
const JOINTS=[
  [10,'Left hip yaw','Turn the left foot so its toe points inward, toward the right leg (seen from above).'],
  [11,'Left hip roll','Swing the left foot outward, away from the body.'],
  [12,'Left hip pitch','Swing the left leg backward: the foot moves behind the hip.'],
  [13,'Left knee','Bend the left knee so the foot moves forward.'],
  [14,'Left ankle','Tip the left toe down.'],
  [20,'Right hip yaw','Turn the right foot so its toe points outward, away from the left leg.'],
  [21,'Right hip roll','Swing the right foot inward, toward the left leg.'],
  [22,'Right hip pitch','Swing the right leg forward: the foot moves ahead of the hip.'],
  [23,'Right knee','Bend the right knee so the foot moves backward.'],
  [24,'Right ankle','Tip the right toe up.'],
  [30,'Neck pitch','Tilt the neck backward: its top moves toward the tail.'],
  [31,'Head pitch','Tip the head nose-down.'],
  [32,'Head yaw','Turn the head to the robot\'s left.'],
  [33,'Head roll','Roll the head so its left side goes down.'],
  [34,'Mouth','Open the beak. (Not a policy joint; the mouth command opens with positive degrees.)'],
];
// IMU checks: value from status, the sign the policy expects, and the threshold.
const IMU=[
  ['grav-x','Pitch nose-down','Tilt the whole robot nose-down by about 30° and hold.','gravity',0,+1,0.3],
  ['grav-y','Roll left side down','Tilt the whole robot so its left side is lower by about 30°, and hold.','gravity',1,+1,0.3],
  ['gyro-y','Pitch rate','Tip the robot nose-down quickly (the peak rate is used).','gyro',1,+1,40],
  ['gyro-x','Roll rate','Roll the robot left side down quickly.','gyro',0,-1,40],
  ['gyro-z','Yaw rate','Turn the robot to its left (anticlockwise seen from above) quickly.','gyro',2,+1,40],
];
const state={};   // key -> {base, peak, result}
let status=null, busy=false;

function row(key,title,text){
  const joint=key.startsWith('j');
  return `<div class="check-row${joint?' joint':''}" id="row-${key}"><div><b>${title}</b><span>${text}</span>${joint?`<em class="check-dir" id="dir-${key}">direction —</em>`:''}</div>
    <code id="val-${key}">—</code><span class="check-result" id="res-${key}">not run</span>
    <div class="check-btns"><button data-start="${key}">Start</button>${joint?`<button data-flip="${key.slice(1)}">Flip direction</button>`:''}</div></div>`;
}
let flipped=new Set();
async function postJson(path,body){
  const r=await fetch('/api/'+path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body||{})});
  const x=await r.json().catch(()=>({}));if(!r.ok)throw new Error(x.detail||'request failed');return x;
}
function renderSigns(js){
  if(!js)return;
  flipped=new Set(js.flipped_ids);
  for(const [id] of JOINTS){const el=$(`dir-j${id}`);if(el){el.textContent=flipped.has(id)?'direction flipped':'direction default';el.classList.toggle('flipped',flipped.has(id));}}
  $('signs-state').textContent=(js.flipped_ids.length?`Flipped: ${js.flipped_ids.join(', ')}`:'All joints at default direction')+(js.synced?'':' · applying to MCU…');
}
$('imu').innerHTML=IMU.map(([k,t,x])=>row(k,t,x)).join('');
$('joints').innerHTML=JOINTS.map(([id,t,x])=>row('j'+id,`ID ${id} · ${t}`,x)).join('');

function current(key){
  if(!status)return null;
  if(key.startsWith('j')){const i=MOTORS.indexOf(+key.slice(1));return status.angles?status.angles[i]:null;}
  const c=IMU.find(r=>r[0]===key);return status[c[3]]?status[c[3]][c[4]]:null;
}
document.addEventListener('click',async e=>{
  const f=e.target.closest('[data-flip]');
  if(f){
    const id=+f.dataset.flip, next=!flipped.has(id);
    if(!confirm(`${next?'Flip':'Restore'} the direction of ID ${id}? Torque must be off.`))return;
    try{renderSigns(await postJson('joint_sign',{id,flipped:next}));delete state['j'+id];
      $(`res-j${id}`).textContent='not run · press Start to recheck';$(`row-j${id}`).className='check-row joint';
      $('message').textContent=`ID ${id} direction ${next?'flipped':'restored'}. Run its check again.`;}
    catch(err){$('message').textContent=err.message;}
    return;
  }
  const b=e.target.closest('[data-start]');if(!b)return;
  const key=b.dataset.start, v=current(key);
  if(v==null){$('message').textContent='No live data yet.';return;}
  if(status&&status.enabled){$('message').textContent='Torque is on. Press Torque off first.';return;}
  state[key]={base:key.startsWith('gyro')?0:v,peak:0,result:null};
  $(`res-${key}`).textContent='move it now…';$(`row-${key}`).className='check-row active';
});
function evaluate(key){
  const s=state[key];if(!s||s.result)return;
  const v=current(key);if(v==null)return;
  let delta, thr, sign=+1;
  if(key.startsWith('j')){delta=v-s.base;thr=10;}
  else{const c=IMU.find(r=>r[0]===key);delta=v-s.base;sign=c[5];thr=c[6];}
  if(Math.abs(delta)>Math.abs(s.peak))s.peak=delta;
  $(`val-${key}`).textContent=(key.startsWith('j')?`Δ ${delta.toFixed(1)}°`:key.startsWith('gyro')?`${v.toFixed(0)}°/s · peak ${s.peak.toFixed(0)}`:`${v.toFixed(2)} (Δ ${delta.toFixed(2)})`);
  if(Math.abs(s.peak)>=thr){
    s.result=Math.sign(s.peak)===sign?'ok':'reversed';
    $(`res-${key}`).textContent=s.result==='ok'?'✓ matches the model':'✗ reversed';
    $(`row-${key}`).className='check-row '+(s.result==='ok'?'pass':'fail');
    summary();
  }
}
function summary(){
  const done=Object.entries(state).filter(([,s])=>s.result);
  const bad=done.filter(([,s])=>s.result==='reversed').map(([k])=>k.startsWith('j')?'ID '+k.slice(1):k);
  $('message').textContent=`${done.length} checked · ${bad.length?'reversed: '+bad.join(', '):'none reversed so far'}`;
}
$('reset-signs').onclick=async()=>{
  if(!confirm('Reset every joint to its default direction? Torque must be off.'))return;
  try{renderSigns(await postJson('joint_sign/reset'));for(const k of Object.keys(state))if(k.startsWith('j'))delete state[k];
    document.querySelectorAll('.check-row.joint').forEach(r=>{r.className='check-row joint';});$('message').textContent='All directions reset to default.';}
  catch(err){$('message').textContent=err.message;}
};
$('torque-off').onclick=async()=>{try{await fetch('/api/mode',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({mode:'off'})});$('message').textContent='Torque off';}catch(e){$('message').textContent=e.message;}};
setInterval(async()=>{
  if(busy||document.hidden)return;busy=true;
  try{
    status=await (await fetch('/api/status')).json();
    renderSigns(status.joint_signs);
    for(const [id] of JOINTS){const k='j'+id;if(!state[k])$(`val-${k}`).textContent=current(k)==null?'—':`${current(k).toFixed(1)}°`;}
    for(const [k] of IMU)if(!state[k]){const v=current(k);$(`val-${k}`).textContent=v==null?'—':v.toFixed(k.startsWith('gyro')?0:2);}
    Object.keys(state).forEach(evaluate);
    if(status.enabled&&!Object.keys(state).length)$('message').textContent='Torque is on. Press Torque off before moving joints by hand.';
    else if(!Object.keys(state).length)$('message').textContent='Ready. Torque is off.';
  }catch(_){}
  finally{busy=false;}
},50);
