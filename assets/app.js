const $=id=>document.getElementById(id);
let statusBusy=false, commandBusy=false, commandPending=false;
const HEAD_LABELS=['Neck pitch 30','Head pitch 31','Head yaw 32','Head roll 33'];
for(let i=0;i<4;i++) $('head-controls').insertAdjacentHTML('beforeend',`<label>${HEAD_LABELS[i]}<input id="head${i}" type="range" min="-1" max="1" value="0" step="0.02"></label>`);
$('head-controls').insertAdjacentHTML('beforeend','<label>Mouth 34 (°)<input id="mouth" type="range" min="0" max="30" value="0" step="1"></label>');
async function post(path, body){const r=await fetch('/api/'+path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});const x=await r.json();if(!r.ok)throw new Error(x.detail||'request failed');return x;}
function showError(e){$('message').textContent=e.message;}
async function setMode(mode){try{await post('mode',{mode});$('message').textContent='Mode updated';}catch(e){showError(e);}}
document.querySelectorAll('[data-mode]').forEach(b=>b.onclick=()=>setMode(b.dataset.mode));
$('pick').onclick=async()=>{try{const r=await post('pick',{});$('message').textContent=`Pick started · ${r.model}`;}catch(e){showError(e);}};
$('roulade').onclick=async()=>{try{const r=await post('roulade',{});$('message').textContent=`Roulade · ${r.model}`;}catch(e){showError(e);}};
$('stop').onclick=()=>{zero();setMode('off')};
const keys=new Set(), sticks={left:{x:0,y:0},right:{x:0,y:0}}, resetSticks=[], knobs={};
let padDriving=false;
let motion=[0,0,0];
const deadzone=v=>Math.abs(v)<.1?0:v;
function updateMotion(){
  const key=(a,b)=>keys.has(a)-keys.has(b);
  motion=[keys.has('w')||keys.has('s')?key('w','s')*.2:deadzone(sticks.left.y)*.4,
    keys.has('a')||keys.has('d')?key('a','d')*.15:deadzone(-sticks.left.x)*.3,
    keys.has('q')||keys.has('e')?key('q','e')*.5:deadzone(-sticks.right.x)];
  ['vx','vy','wz'].forEach((id,i)=>$(id+'Value').textContent=motion[i].toFixed(2));
}
async function sendCommand(){
  if(commandBusy){commandPending=true;return;}
  commandBusy=true;commandPending=false;
  try{await post('command',{twist:[...motion],head:[0,1,2,3].map(i=>+$('head'+i).value),mouth:+$('mouth').value});}
  catch(e){showError(e);}
  finally{commandBusy=false;if(commandPending)sendCommand();}
}
function joystick(id,side){
  const zone=$(id),knob=zone.firstElementChild;
  let pointer=null;
  function reset(){
    const old=pointer;pointer=null;
    sticks[side]={x:0,y:0};knob.style.transform='translate(0px,0px)';
    if(old!==null&&zone.hasPointerCapture(old))zone.releasePointerCapture(old);
  }
  resetSticks.push(reset);
  knobs[side]=(x,y)=>{const radius=(zone.clientWidth-knob.offsetWidth)/2;knob.style.transform=`translate(${x*radius}px,${-y*radius}px)`;};
  function move(e){
    const b=zone.getBoundingClientRect(),radius=(zone.clientWidth-knob.offsetWidth)/2;
    let dx=e.clientX-b.left-b.width/2,dy=e.clientY-b.top-b.height/2;
    const distance=Math.hypot(dx,dy);
    if(distance>radius){dx*=radius/distance;dy*=radius/distance;}
    knob.style.transform=`translate(${dx}px,${dy}px)`;
    sticks[side]={x:dx/radius,y:-dy/radius};updateMotion();
  }
  zone.onpointerdown=e=>{if(padDriving||pointer!==null||e.button!==0)return;e.preventDefault();pointer=e.pointerId;zone.setPointerCapture(pointer);move(e);sendCommand();};
  zone.onpointermove=e=>{if(e.pointerId===pointer)move(e);};
  const release=e=>{if(e.pointerId!==pointer)return;reset();updateMotion();sendCommand();};
  zone.onpointerup=zone.onpointercancel=zone.onlostpointercapture=release;
}
joystick('joy-left','left');joystick('joy-right','right');
function zero(){keys.clear();resetSticks.forEach(reset=>reset());updateMotion();sendCommand();}
$('zero').onclick=zero;
['leg-alpha','head-alpha'].forEach(id=>$(id).onchange=()=>post('filter',{leg:+$('leg-alpha').value,head:+$('head-alpha').value}).catch(showError));
function keyCommand(){updateMotion();sendCommand();}
window.addEventListener('keydown',e=>{if(e.code==='Space'){e.preventDefault();$('stop').click();return;}const k=e.key.toLowerCase();if('wasdqe'.includes(k)&&k.length===1){keys.add(k);keyCommand();}});
window.addEventListener('keyup',e=>{const k=e.key.toLowerCase();if(keys.delete(k))keyCommand();});
window.addEventListener('blur',()=>{keys.clear();zero();});
document.addEventListener('visibilitychange',()=>{if(document.hidden){keys.clear();zero();post('mode',{mode:'off'}).catch(()=>{});}});
window.addEventListener('pagehide',()=>navigator.sendBeacon('/api/mode',new Blob([JSON.stringify({mode:'off'})],{type:'application/json'})));
setInterval(()=>{if(!document.hidden&&!commandBusy)sendCommand();},50);
setInterval(async()=>{if(statusBusy||document.hidden)return;statusBusy=true;try{const r=await fetch('/api/status');if(!r.ok)throw new Error('status read failed');const s=await r.json();const modeLabel={shadow:'inference only',off:'torque off',hold:'default pose',policy:'walk',calibrate:'calibrate',servo_debug:'servo setup'};$('rx').textContent=s.feedback_hz.toFixed(1)+' Hz';$('rl').textContent=s.inference_hz.toFixed(1)+' Hz';$('servos').textContent=(s.servo_ids?.length||0)+'/15';$('imu').textContent=s.imu_valid?'online':s.imu_ok?'no data':'offline';$('imu').classList.toggle('warn-text',!s.imu_valid);$('mode').textContent=(modeLabel[s.mode]||s.mode)+(s.enabled?' · enabled':' · torque off');$('message').textContent=s.last_error||`Bridge ${(s.bridge_baud/1e6).toFixed(0)} Mbps · state age ${(s.feedback_age_ms||0).toFixed(1)} ms`;
document.querySelectorAll('[data-mode="hold"],[data-mode="policy"]').forEach(b=>b.disabled=!s.control_ready);
const recovery=s.recovery, pick=s.pick, roulade=s.roulade;
const upright=(s.mode==='shadow'||s.mode==='policy')&&recovery&&recovery.active&&recovery.phase==='walk';
$('pick').disabled=!upright||(pick&&pick.active)||(roulade&&roulade.active);
$('roulade').disabled=$('pick').disabled;
if(recovery){const phaseLabel=roulade&&roulade.active?'roulade':pick&&pick.active?'pick':({walk:'walk',default_pose:'get-up · action = 0',getup:'get-up'}[recovery.phase]||recovery.phase);
$('recovery-phase').textContent=recovery.active?phaseLabel:'idle';$('active-model').textContent=s.active_model||'—';$('tilt-angle').textContent=recovery.angle_deg==null?'—':recovery.angle_deg.toFixed(1)+'°';
$('recovery-timer').textContent=!recovery.active?'—':roulade&&roulade.active?`roulade ${roulade.remaining_s.toFixed(2)} / ${roulade.seconds.toFixed(2)} s`:pick&&pick.active?`pick φ ${pick.phi.toFixed(2)} / 1.00`:recovery.phase==='default_pose'?`hold ${recovery.hold_remaining_s.toFixed(2)} s left`:recovery.phase==='getup'?`upright ${recovery.stand_elapsed_s.toFixed(2)} / 1.00 s`:`fallen ${recovery.fall_elapsed_s.toFixed(2)} / 0.15 s`;
if(pick&&pick.active)$('mouth').value=Math.round(s.mouth||0);}
for(const [id,values]of [['acc-values',s.acc],['gyro-values',s.gyro],['gravity-values',s.gravity]])$(id).textContent=values?values.map(x=>x.toFixed(3)).join(' / '):'—';
$('command-health').textContent=`sent ${s.sent} · MCU echo ${s.mcu_command_seq??'—'} · command age ${s.mcu_command_age_us<1e9?(s.mcu_command_age_us/1000).toFixed(1)+' ms':'—'} · gaps ${s.sequence_gaps} · rejected ${s.mcu_invalid_commands??0}`;
renderTuning(s);
const pad=s.pad||{};
// While a controller drives motion, show the commands actually in effect.
const driving=!!(pad.connected&&pad.active);
if(driving){const [vx,vy,wz]=s.command||[0,0,0];
  knobs.left&&knobs.left(-vy/.3,vx/.4);knobs.right&&knobs.right(-wz/1,0);
  ['vx','vy','wz'].forEach((id,i)=>$(id+'Value').textContent=(s.command||[0,0,0])[i].toFixed(2));
  (s.head||[]).forEach((v,i)=>$('head'+i).value=v);$('mouth').value=Math.round(s.mouth||0);}
else if(padDriving){resetSticks.forEach(r=>r());updateMotion();}
padDriving=driving;document.querySelectorAll('.joy,#head-controls input').forEach(el=>el.classList.toggle('pad-driven',driving));
document.querySelectorAll('#head-controls input').forEach(el=>el.disabled=driving);$('pad-note').textContent=pad.connected?`Controller: ${pad.name||'connected'}${pad.active?' · its sticks drive motion; these pads are ignored':''}${pad.error?' · '+pad.error:''}`:'No Bluetooth controller connected. Pair one on the Controller page.';
if(s.angles)$('joints').innerHTML=s.motors.map((id,i)=>`<tr class="${s.servo_ids.includes(id)?'':'missing'}"><td>${id}</td><td>${s.angles[i].toFixed(1)}</td><td>${s.targets[i].toFixed(1)}</td></tr>`).join('');
}catch(e){showError(e);}finally{statusBusy=false;}},250);

// Walk tuning
let tuneBusy=false;
function renderTuning(s){
  const b=s.battery_v;
  $('battery').textContent=b==null?'battery —':`battery ${b.toFixed(1)} V · max ${s.servo_temp_max??'—'} °C`;
  $('battery').className=b!=null&&b<7.0?'warn-text':'';
  const t=s.tuning;if(!t||tuneBusy)return;
  document.querySelectorAll('[data-home]').forEach(x=>x.classList.toggle('active',x.dataset.home===t.home_source));
  if(document.activeElement!==$('cmd-tau')){$('cmd-tau').value=t.cmd_tau_s;$('cmd-tau-v').textContent=t.cmd_tau_s?t.cmd_tau_s.toFixed(2)+' s':'off';}
  $('imu-bias').textContent=`roll ${t.imu_bias_deg[0].toFixed(1)}° · pitch ${t.imu_bias_deg[1].toFixed(1)}°`;
  if(document.activeElement!==$('home-lean')){$('home-lean').value=t.home_lean_deg??0;$('home-lean-v').textContent=leanLabel(t.home_lean_deg??0);}
  const g=s.servo_gains;
  if(g){for(const k of ['kp','kd'])if(document.activeElement!==$('servo-'+k)){$('servo-'+k).value=g[k];$(`servo-${k}-v`).textContent=g[k];}
    $('gain-state').textContent=g.synced?'· applied':'· applying…';}
  const h=s.deployment&&s.deployment.home_deg;if(h)$('home-detail').textContent=`walk home now: hip pitch ${h[2].toFixed(1)}° / ankle ${h[4].toFixed(1)}° · knee ${h[3].toFixed(2)}°`;
}
async function setTuning(body,label){
  tuneBusy=true;
  try{await post('tuning',body);$('message').textContent=label;}catch(e){showError(e);}finally{tuneBusy=false;}
}
document.querySelectorAll('[data-home]').forEach(x=>x.onclick=()=>setTuning({home_source:x.dataset.home},`Home pose: ${x.textContent}`));
$('cmd-tau').oninput=()=>{const v=+$('cmd-tau').value;$('cmd-tau-v').textContent=v?v.toFixed(2)+' s':'off';};
$('cmd-tau').onchange=()=>setTuning({cmd_tau_s:+$('cmd-tau').value},'Command smoothing updated');
$('level-imu').onclick=async()=>{
  if(!confirm('Is the body held level, checked with a spirit level on its flat top, and still? (Not Default pose.)'))return;
  tuneBusy=true;$('message').textContent='Levelling IMU… keep still';
  try{const r=await post('tuning/level',{seconds:1});$('message').textContent=`IMU levelled: roll ${r.tuning.imu_bias_deg[0]}°, pitch ${r.tuning.imu_bias_deg[1]}° (${r.samples} samples)`;}
  catch(e){showError(e);}finally{tuneBusy=false;}
};
$('reset-imu').onclick=()=>setTuning({imu_bias_deg:[0,0,0]},'IMU level reset');

['kp','kd'].forEach(k=>{
  $('servo-'+k).oninput=()=>{$(`servo-${k}-v`).textContent=$('servo-'+k).value;};
  $('servo-'+k).onchange=()=>setTuning({['servo_'+k]:+$('servo-'+k).value},`Servo ${k.toUpperCase()} set to ${$('servo-'+k).value}`);
});
async function refreshWalk(){
  try{
    const r=await (await fetch('/api/log/summary')).json();
    if(!r.walk){$('walk-summary').textContent='No walk recorded yet.';return;}
    const rows=['forward','backward','turn','still'].filter(k=>r.groups[k]).map(k=>{const x=r.groups[k];
      return `<tr><td>${k}</td><td>${x.legs_ratio??'—'}</td><td>${x.legs_lag_ms??'—'} ms</td><td>${k==='still'?'':x.peak_vx}</td></tr>`;}).join('');
    $('walk-summary').innerHTML=`<p>KP ${r.gains.kp} · KD ${r.gains.kd} · ${r.seconds} s walking in the last ${r.window_s} s${r.falls?` · <b class="warn-text">${r.falls} fall${r.falls>1?'s':''}</b>`:' · no falls'}${r.still_pitch_deg!=null?` · resting pitch ${r.still_pitch_deg}°`:''}</p>
      <table><thead><tr><th>command</th><th>follow</th><th>lag</th><th>peak vx</th></tr></thead><tbody>${rows}</tbody></table>`;
  }catch(e){$('walk-summary').textContent=e.message;}
}
$('walk-refresh').onclick=refreshWalk;
$('walk-clear').onclick=async()=>{try{await post('log/clear',{});$('walk-summary').textContent='Cleared. Walk, then Refresh.';}catch(e){showError(e);}};
refreshWalk();
setInterval(()=>{if(!document.hidden)refreshWalk();},2000);

function leanLabel(v){v=+v;return v?`${v>0?'+':''}${v}° · body ~${Math.abs(v*1.3).toFixed(0)} mm ${v>0?'forward':'back'}`:'0°';}
$('home-lean').oninput=()=>{$('home-lean-v').textContent=leanLabel($('home-lean').value);};
$('home-lean').onchange=()=>setTuning({home_lean_deg:+$('home-lean').value},`Home lean set to ${$('home-lean').value}°`);
