const $=id=>document.getElementById(id);
const esc=s=>String(s??'').replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));
const BUTTONS=['south','east','west','north','l1','r1','l2','r2','select','start','mode','thumbl','thumbr','dpad_up','dpad_down','dpad_left','dpad_right'];
const AXES=['lx','ly','rx','ry','lz','rz','gas','brake'];
let meta=null, config=null, dirty=false, busy=false;

async function api(path,body,timeout=8000){
  const opts={signal:AbortSignal.timeout(timeout)};
  if(body!==undefined)Object.assign(opts,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
  const r=await fetch('/api/'+path,opts);
  const x=await r.json().catch(()=>({}));
  if(!r.ok)throw new Error(x.detail||'request failed');
  return x;
}
function show(m){$('message').textContent=m;}

// Bluetooth
async function refreshBt(){
  try{
    const s=await api('bt/status');
    $('bt-state').textContent=(s.powered?'adapter on':'adapter off')+(s.scanning?` · scanning ${s.scan_left_s}s`:'')+(s.power_note?` · last power event ${s.power_note}`:'');
    $('scan').disabled=busy||s.scanning;
    const ap=s.autopair||{};
    $('autopair').disabled=busy||ap.active;
    $('autopair-msg').textContent=ap.message||'';
    $('devices').innerHTML=s.devices.length?s.devices.map(d=>`<div class="pad-device">
      <div><b>${esc(d.alias||d.name||'Unknown device')}</b><span>${esc(d.mac)}${d.rssi!=null?` · ${d.rssi} dBm`:''}${d.icon?` · ${esc(d.icon)}`:''}</span>
      <span class="pad-tags">${d.connected?'<em class="on">connected</em>':''}${d.paired?'<em>paired</em>':''}${d.trusted?'<em>trusted</em>':''}</span></div>
      <div class="buttons">${!d.paired?`<button data-act="pair" data-mac="${esc(d.mac)}">Pair &amp; connect</button>`:
        d.connected?`<button data-act="disconnect" data-mac="${esc(d.mac)}">Disconnect</button>`:`<button data-act="connect" data-mac="${esc(d.mac)}">Connect</button>`}
        ${d.paired?`<button data-act="remove" data-mac="${esc(d.mac)}">Forget</button>`:''}</div></div>`).join('')
      :'<p class="hint">No devices yet. Scan with the controller in pairing mode.</p>';
    if(s.hidden_unnamed)$('devices').insertAdjacentHTML('beforeend',`<p class="hint">${s.hidden_unnamed} unnamed nearby devices hidden.</p>`);
    if(s.helper_error)show('Helper: '+s.helper_error);
  }catch(e){$('bt-state').textContent='unavailable';show(e.message);}
}
$('devices').onclick=async e=>{
  const b=e.target.closest('button[data-act]');
  if(!b||busy)return;
  const {act,mac}=b.dataset;
  if(act==='remove'&&!confirm(`Forget ${mac}? It will need pairing again.`))return;
  busy=true;b.disabled=true;
  show({pair:'Pairing… keep the controller in pairing mode (up to 30 s).',connect:'Connecting…',disconnect:'Disconnecting…',remove:'Forgetting…'}[act]);
  try{await api('bt/'+act,{mac},act==='pair'?65000:30000);show(`${act} ${mac}: done`);}
  catch(e){show(`${act} ${mac}: ${e.message}`);}
  finally{busy=false;refreshBt();}
};
$('autopair').onclick=async()=>{try{await api('bt/autopair',{seconds:60});show('Now put the controller in pairing mode.');refreshBt();}catch(e){show(e.message);}};
$('scan').onclick=async()=>{try{await api('bt/scan',{seconds:20});show('Scanning for 20 s…');refreshBt();}catch(e){show(e.message);}};
$('refresh').onclick=refreshBt;

// Mapping editor
function options(map,value){return Object.entries(map).map(([k,v])=>`<option value="${esc(k)}"${k===value?' selected':''}>${esc(v)}</option>`).join('');}
function renderMap(){
  const buttons=[...new Set([...BUTTONS,...Object.keys(config.buttons)])];
  $('map-buttons').innerHTML=buttons.map(b=>`<label class="pad-row" data-button="${esc(b)}"><span>${esc(b)}</span>
    <select data-map-button="${esc(b)}">${options(meta.actions,config.buttons[b]||'')}</select></label>`).join('');
  const axes=[...new Set([...AXES,...Object.keys(config.axes)])];
  $('map-axes').innerHTML=axes.map(a=>{const s=config.axes[a]||{target:'',invert:false,trigger:false};
    return `<div class="pad-row" data-axis="${esc(a)}"><span>${esc(a)}</span>
    <select data-map-axis="${esc(a)}">${options(meta.axis_targets,s.target)}</select>
    <label><input type="checkbox" data-invert="${esc(a)}"${s.invert?' checked':''}> Invert</label>
    <label><input type="checkbox" data-trigger="${esc(a)}"${s.trigger?' checked':''}> Trigger</label></div>`;}).join('');
  $('deadzone').value=config.deadzone;
}
function collect(){
  const buttons={},axes={};
  document.querySelectorAll('[data-map-button]').forEach(s=>{if(s.value)buttons[s.dataset.mapButton]=s.value;});
  document.querySelectorAll('[data-map-axis]').forEach(s=>{const a=s.dataset.mapAxis;if(s.value)axes[a]={target:s.value,
    invert:document.querySelector(`[data-invert="${CSS.escape(a)}"]`).checked,trigger:document.querySelector(`[data-trigger="${CSS.escape(a)}"]`).checked};});
  return {deadzone:+$('deadzone').value||0,buttons,axes};
}
document.addEventListener('change',e=>{if(e.target.closest('.pad-map')||e.target.id==='deadzone'){dirty=true;show('Unsaved mapping changes');}});
$('save').onclick=async()=>{try{const r=await api('pad/config',{config:collect()});config=r.config;dirty=false;renderMap();show('Mapping saved');}catch(e){show(e.message);}};
$('defaults').onclick=()=>{if(!confirm('Replace the mapping with the defaults? Save to keep it.'))return;config=structuredClone(meta.defaults);renderMap();dirty=true;show('Defaults loaded. Save to keep them.');};

// Live input from /api/status
function renderLive(p){
  const connected=p&&p.connected, pressed=new Set(connected?p.buttons:[]), axes=connected?p.axes||{}:{};
  $('pad-name').textContent=connected?`${p.name||'controller'}${p.active?'':' · idle'}`:'no controller';
  const names=[...new Set([...BUTTONS,...pressed])];
  $('pad-buttons').innerHTML=names.map(b=>`<span class="pad-chip${pressed.has(b)?' on':''}">${esc(b)}</span>`).join('');
  $('pad-axes').innerHTML=Object.keys(axes).sort().map(a=>{const v=axes[a];return `<div class="pad-axis"><span>${esc(a)}</span>
    <div class="pad-bar"><i style="left:${50+Math.min(0,v)*50}%;width:${Math.abs(v)*50}%"></i></div><code>${v.toFixed(2)}</code></div>`;}).join('')
    ||'<p class="hint">Axes appear when a controller is connected.</p>';
  document.querySelectorAll('[data-button]').forEach(r=>r.classList.toggle('on',pressed.has(r.dataset.button)));
  document.querySelectorAll('[data-axis]').forEach(r=>r.classList.toggle('on',Math.abs(axes[r.dataset.axis]||0)>0.5&&!(r.querySelector('[data-trigger]')?.checked&&(axes[r.dataset.axis]||0)<-0.5)));
  if(p&&p.error)show(p.error);
}
let statusBusy=false;
setInterval(async()=>{
  if(statusBusy||document.hidden)return;statusBusy=true;
  try{const s=await api('status');renderLive(s.pad);}catch(_){}
  finally{statusBusy=false;}
},100);
setInterval(()=>{if(!document.hidden&&!busy)refreshBt();},1500);

(async()=>{
  try{meta=await api('pad/config');config=meta.config;renderMap();show('Ready');}
  catch(e){show(e.message);}
  refreshBt();
})();
window.addEventListener('beforeunload',e=>{if(dirty){e.preventDefault();}});
