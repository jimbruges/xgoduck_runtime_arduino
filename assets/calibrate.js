const $=id=>document.getElementById(id);
const MOTORS=[10,11,12,13,14,20,21,22,23,24,30,31,32,33,34];
let statusBusy=false, targetBusy=false, targetPending=false, active=false, lastStatus=null, finishing=false;
const targets=Array(15).fill(null); // null: joint not energized yet

async function post(path, body={}){
  const r=await fetch('/api/'+path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
  const x=await r.json().catch(()=>({}));
  if(!r.ok) throw new Error(x.detail||'request failed');
  return x;
}
function showError(e){$('message').textContent=typeof e==='string'?e:(e.message||String(e));}

const list=$('joints');
MOTORS.forEach((id,i)=>{
  list.insertAdjacentHTML('beforeend',`<div class="cal-row" id="row${i}">
    <img class="cal-thumb" src="${CalGuide.JOINTS[i].img}" alt="" loading="lazy">
    <div class="cal-meta"><b>ID ${id} · ${CalGuide.JOINTS[i].name}</b><span id="raw${i}">—</span><span id="zero${i}">zero —</span></div>
    <input id="sl${i}" type="range" min="0" max="4095" value="2047">
    <div class="cal-actions"><span id="val${i}">off</span><button data-finish="${i}">Finish</button></div>
  </div>`);
  $(`row${i}`).addEventListener('pointerdown',()=>select(i));
  $(`sl${i}`).addEventListener('focus',()=>select(i));
  $(`sl${i}`).oninput=()=>setTarget(i,+$(`sl${i}`).value);
});

// Row and guide sliders drive the same target.
function setTarget(i,v){
  if(!active) return;
  targets[i]=v;
  $(`sl${i}`).value=v;
  $(`val${i}`).textContent=v;
  if(i===selected){$('guide-sl').value=v;$('guide-val').textContent=v;}
  queueTargets();
}
function syncGuide(){
  const i=selected;
  $('guide-raw').textContent=$(`raw${i}`).textContent;
  $('guide-raw').style.color=$(`raw${i}`).style.color;
  $('guide-sl').disabled=$(`sl${i}`).disabled;
  $('guide-sl').value=$(`sl${i}`).value;
  $('guide-val').textContent=$(`val${i}`).textContent;
}
$('guide-sl').oninput=()=>setTarget(selected,+$('guide-sl').value);

let selected=-1;
function select(i){
  if(i===selected)return;
  if(selected>=0)$(`row${selected}`).classList.remove('selected');
  selected=i;
  $(`row${i}`).classList.add('selected');
  const j=CalGuide.JOINTS[i];
  $('guide-id').textContent=`ID ${MOTORS[i]}`;
  $('guide-name').textContent=j.name;
  $('guide-text').textContent=j.text;
  $('guide-view').textContent=j.view;
  $('guide-img').src=j.img;
  $('guide-img').alt=`CAD render of ${j.name} at 0°, ${j.view.toLowerCase()}`;
  syncGuide();
}
$('guide-prev').onclick=()=>select((selected+14)%15);
$('guide-next').onclick=()=>select((selected+1)%15);
select(0);

function queueTargets(){
  if(!active||finishing) return;
  if(targetBusy){targetPending=true;return;}
  targetBusy=true;targetPending=false;
  post('cal/targets',{targets:[...targets]}).catch(showError).finally(()=>{
    targetBusy=false;
    if(targetPending) queueTargets();
  });
}

const sleep=ms=>new Promise(r=>setTimeout(r,ms));
// Finish re-centres the servo so this pose reads 2048. Stop sending targets until it is
// done, so no target from the old encoder frame reaches the joint, then hold it at 2048.
async function finish(i){
  if(finishing)return;
  finishing=true;
  const held=targets[i]!==null;
  $('message').textContent=`Saving ID ${MOTORS[i]} zero…`;
  try{
    while(targetBusy)await sleep(20);
    targets[i]=null;
    const r=await post('cal/finish',{index:i});
    if(held){targets[i]=r.zero_pos;$(`sl${i}`).value=r.zero_pos;$(`val${i}`).textContent=r.zero_pos;}
    else $(`val${i}`).textContent='off';
    $(`row${i}`).classList.add('done');
    $(`zero${i}`).textContent='zero '+r.zero_pos;
    $('message').textContent=r.needs_power_cycle
      ?`Joint ${MOTORS[i]} zero saved. The servo still counts ${Math.abs(r.turns)} turn${Math.abs(r.turns)>1?'s':''} from before, so it reads out of range until its power is cycled: when calibration is done, turn the servo power off and on.`
      :`Joint ${MOTORS[i]} zero saved as ${r.zero_pos}`;
    if(i<14)select(i+1);
  }catch(e){showError(e);}
  finally{finishing=false;queueTargets();}
}
document.querySelectorAll('[data-finish]').forEach(btn=>btn.onclick=()=>finish(+btn.dataset.finish));
$('guide-finish').onclick=()=>finish(selected);

async function enter(){
  try{
    await post('cal/enter',{});
    active=true;
    targets.fill(null);
    MOTORS.forEach((_,i)=>$(`val${i}`).textContent='off');
    syncGuide();
    $('message').textContent='Calibration active: torque off. A joint powers up (KP = 3, KD = 0) when you first move its slider, then follows it slowly.';
  }catch(e){showError(e);}
}
async function exit(){
  try{
    await post('cal/exit',{});
    active=false;
    $('message').textContent='Calibration closed. Position modes are available from the control page.';
  }catch(e){showError(e);}
}
// Hold every responding joint at its saved zero for fine-tuning. The MCU still energizes
// one joint per tick and slews each at CAL_STEP_PER_TICK, so this is not a jump.
async function moveAllToZero(){
  if(!confirm('Move every responding joint to its saved zero and hold it there? Only do this after calibrating, with the robot supported.'))return;
  if(!active){await enter(); if(!active)return;}
  const s=lastStatus;
  if(!s||!s.zero_pos||!s.raw_pos){showError('No servo status yet; try again in a moment.');return;}
  const missing=new Set(s.missing_joint_ids||[]), skipped=[];
  MOTORS.forEach((id,i)=>{
    const z=s.zero_pos[i], v=s.raw_pos[i];
    if(missing.has(id)||!(z>=0&&z<=4095)||!(v>=0&&v<=4095)){skipped.push(id);return;}
    targets[i]=z;
    $(`sl${i}`).value=z;
    $(`val${i}`).textContent=z;
  });
  syncGuide();
  queueTargets();
  $('message').textContent='Moving to saved zeros and holding (KP = 3). Fine-tune with a slider, then press Finish.'+
    (skipped.length?` Skipped (missing or out of range): ${skipped.join(', ')}.`:'');
}
$('start').onclick=enter;
$('all-zero').onclick=moveAllToZero;
$('exit').onclick=exit;
$('exit2').onclick=exit;
window.addEventListener('pagehide',()=>navigator.sendBeacon('/api/cal/exit',new Blob([JSON.stringify({})],{type:'application/json'})));
document.addEventListener('visibilitychange',()=>{if(document.hidden&&active)exit();});

setInterval(async()=>{
  if(statusBusy||document.hidden)return;
  statusBusy=true;
  try{
    const r=await fetch('/api/status');
    if(!r.ok)throw new Error('status read failed');
    const s=await r.json();
    lastStatus=s;
    active=s.mode==='calibrate';
    $('mode').textContent=s.mode+(s.mcu_calibrating?' · MCU':'');
    if(s.last_error)$('message').textContent=s.last_error;
    if(s.cal_done)s.cal_done.forEach((d,i)=>$(`row${i}`).classList.toggle('done',!!d));
    if(s.raw_pos)s.raw_pos.forEach((v,i)=>{
      // Sign-magnitude: bit 15 marks a reading below 0. Out of range means the servo
      // counted a turn since power-up; Finish still sets the right offset.
      const inRange=v>=0&&v<=4095, signed=v&0x8000?-(v&0x7FFF):v;
      $(`raw${i}`).textContent=inRange?'encoder '+v:`encoder ${signed}: out of range (the servo counted a turn). Slider locked; Finish at zero still works, then cycle servo power`;
      $(`raw${i}`).style.color=inRange?'':'#f1ca86';
      // The MCU will not power a joint whose reading is past the encoder range.
      $(`sl${i}`).disabled=!inRange;
      // Unpowered joints: keep the slider on the present position so the first drag starts there.
      if(targets[i]===null&&inRange)$(`sl${i}`).value=v;
    });
    if(s.zero_pos)s.zero_pos.forEach((v,i)=>$(`zero${i}`).textContent='zero '+v);
    // A servo whose offset no longer matches its calibration: flag it until it is redone.
    const oc=s.offset_check, lost=new Set(oc&&oc.state==='mismatch'?oc.mismatch.map(m=>m.id):[]);
    $('offset-warn').hidden=!lost.size;
    if(lost.size)$('offset-warn').textContent=oc.message+' Finish each one, then Exit calibration to save.';
    MOTORS.forEach((id,i)=>$(`row${i}`).classList.toggle('lost',lost.has(id)));
    syncGuide();
    if(active){
      // Keep the calibration session alive. Status reads do not refresh its heartbeat.
      // Through queueTargets, so Finish waits for it and it never follows a Finish.
      if(!targetBusy&&!finishing) queueTargets();
    }
  }catch(e){showError(e);}
  finally{statusBusy=false;}
},250);
