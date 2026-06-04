/* Auto-generated from zone_setup.html -- do not edit directly */
R"ZONEHTML(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>irrigoto · Zone Setup</title>
<script>
(function(){
  try{var t=localStorage.getItem('irrigoto_theme');
    if(t==='light'||t==='dark')document.documentElement.dataset.theme=t;}catch(e){}
  fetch('/api/theme').then(function(r){return r.json();}).then(function(d){
    var t2=d.dark?'dark':'light';
    if(document.documentElement.dataset.theme!==t2){
      document.documentElement.dataset.theme=t2;
      try{localStorage.setItem('irrigoto_theme',t2);}catch(e){}}
  }).catch(function(){});
})();
</script>
<style>
*{box-sizing:border-box;margin:0;padding:0;-webkit-tap-highlight-color:transparent;}
:root,:root[data-theme="dark"]{
  --bg:#060c10;--bg2:#0b1820;--bg3:#0f2030;
  --green:#00e87a;--green-dim:#0a3020;--green-glow:rgba(0,232,122,.25);
  --orange:#ff8c00;--orange-dim:rgba(255,140,0,.12);
  --text:#c8e0d4;--text-dim:#2e5040;--text-mid:#608070;
  --border:#0c2a1e;
  --btn:#0b1c28;--btn-hover:#0f2535;
  --radius:12px;--radius-sm:8px;
  /* Shared with landing_html.h .zone-radar — keep in sync. The canvas
     draw() reads this via getComputedStyle for the radar background. */
  --radar-bg:#0a1a12;
  /* Range-ring stroke for the zone-setup radar. Tuned to sit just
     barely above the radar bg so the rings are very faint guidelines. */
  --radar-grid:#15281e;
}
:root[data-theme="light"]{
  --bg:#f7f9f8;--bg2:#ffffff;--bg3:#eef3f0;
  --green:#00913f;--green-dim:#cfeede;--green-glow:rgba(0,145,63,.20);
  --orange:#c95400;--orange-dim:rgba(201,84,0,.12);
  --text:#0e1a14;--text-dim:#cfdcd5;--text-mid:#5a7468;
  --border:#cee0d7;
  --btn:#f0f5f2;--btn-hover:#e3ece7;
  --radar-bg:#E8F4FE;  /* light blue, shared with landing radar */
  --radar-grid:#cfdcd5;  /* light grid; current --text-dim looked right */
}
html,body{height:100%;overflow:hidden;background:var(--bg);}
body{
  color:var(--text);
  font-family:-apple-system,BlinkMacSystemFont,'SF Pro Text','Helvetica Neue',sans-serif;
  display:flex;flex-direction:column;height:100dvh;
  max-width:480px;margin:0 auto;
  -webkit-user-select:none;user-select:none;-webkit-touch-callout:none;
}
/* Control panel: disable text selection everywhere (kills stray selection of
   labels/buttons and the selection box that spanned the d-pad on mobile),
   except in editable fields like the zone-name input. */
input,textarea{-webkit-user-select:text;user-select:text;}

/* ── Header ── */
header{
  display:flex;align-items:center;justify-content:space-between;
  padding:14px 16px 10px;flex-shrink:0;
  border-bottom:1px solid var(--border);
}
.back{color:var(--green);text-decoration:none;font-size:20px;line-height:1;}
.logo{
  font-family:'Courier New',Courier,monospace;
  font-size:20px;font-weight:700;letter-spacing:.15em;
  color:var(--green);text-shadow:0 0 12px var(--green-glow);
}
.header-mid{
  font-size:10px;letter-spacing:.18em;color:var(--text-mid);
  text-transform:uppercase;font-weight:500;
}
/* ── Name row ── */
#name-row{
  flex-shrink:0;display:flex;align-items:center;gap:10px;
  padding:6px 16px 4px;border-top:1px solid var(--border);
}
#name-row label{
  font-size:9px;letter-spacing:.16em;text-transform:uppercase;
  color:var(--text-dim);font-family:'Courier New',monospace;white-space:nowrap;
}
#zone-name{
  flex:1;background:transparent;border:none;border-bottom:1px solid var(--border);
  color:var(--text);font-size:14px;font-weight:600;font-family:inherit;
  padding:2px 4px;outline:none;transition:border-color .15s;
}
#zone-name:focus{border-bottom-color:var(--green);}
#zone-name::placeholder{color:var(--text-dim);font-weight:400;}
#conn{display:flex;align-items:center;gap:6px;font-size:11px;color:var(--text-mid);}
#conn-dot{width:7px;height:7px;border-radius:50%;background:var(--text-dim);transition:all .4s;}
#conn-dot.ok{background:var(--green);box-shadow:0 0 8px var(--green);}
#conn-dot.err{background:#e84040;box-shadow:0 0 8px rgba(232,64,64,.5);}

/* ── Radar map ── */
#map-wrap{
  flex:1;display:flex;align-items:center;justify-content:center;
  padding:12px 16px;min-height:0;position:relative;
}
#heat-legend{
  display:none;flex-direction:column;align-items:flex-start;
  gap:4px;position:absolute;right:10px;top:50%;transform:translateY(-50%);
}
.hl-bar{
  width:14px;border-radius:3px;
  background:linear-gradient(to bottom,
    rgba(255,90,40,0.95) 0%,rgba(255,195,40,0.9) 28%,
    rgba(40,210,80,0.95) 52%,rgba(60,200,180,0.9) 76%,
    rgba(60,140,255,0.95) 100%);
  border:0.5px solid rgba(255,255,255,0.2);
}
.hl-ticks{
  display:flex;flex-direction:column;justify-content:space-between;
  font-size:8px;color:rgba(200,224,212,0.7);line-height:1.3;
  padding-left:4px;
}
#hl-label{font-size:7px;color:rgba(200,224,212,0.4);white-space:nowrap;margin-top:2px;}
#map{border-radius:50%;display:block;touch-action:none;}

/* ── Readout bar ── */
#readout{
  display:grid;grid-template-columns:repeat(3,1fr);
  flex-shrink:0;border-top:1px solid var(--border);
  border-bottom:1px solid var(--border);
}
.rcel{
  text-align:center;padding:8px 0;
  border-right:1px solid var(--border);
}
.rcel:last-child{border-right:none;}
.rlabel{
  font-size:8px;letter-spacing:.16em;text-transform:uppercase;
  color:var(--text-dim);font-family:'Courier New',monospace;margin-bottom:3px;
  display:flex;align-items:center;justify-content:center;gap:5px;
}
.rval{
  font-size:19px;font-weight:700;font-family:'Courier New',monospace;
  color:var(--text);line-height:1;
}

/* ── D-Pad ── */
#dpad-section{
  flex-shrink:0;display:flex;align-items:center;
  justify-content:center;gap:16px;padding:12px 16px;
}
#dpad{
  display:grid;
  grid-template-columns:56px 56px 56px;
  grid-template-rows:56px 56px 56px;
  gap:4px;
}
.dp{
  display:flex;align-items:center;justify-content:center;
  background:var(--btn);border:1px solid var(--border);
  border-radius:var(--radius-sm);color:var(--text-mid);
  font-size:20px;cursor:pointer;
  -webkit-touch-callout:none;-webkit-user-select:none;user-select:none;
  transition:background .1s,color .1s,box-shadow .1s;
  touch-action:manipulation;
}
.dp:active,.dp.held{
  background:var(--green-dim);color:var(--green);
  box-shadow:0 0 14px var(--green-glow);border-color:#1a5035;
}
.dp-empty{background:transparent;border:none;pointer-events:none;}
#btn-add{
  background:transparent;
  border:1px solid #1a5035 !important;
  color:var(--green);font-size:30px;font-weight:300;
  border-radius:var(--radius-sm);
}
#btn-add:active{background:var(--green-dim);box-shadow:0 0 18px var(--green-glow);}

/* ── Secondary actions ── */
#secondary{
  flex-shrink:0;display:flex;gap:8px;padding:0 16px 10px;
}
.sbtn{
  flex:1;padding:11px 0;
  background:var(--btn);border:1px solid var(--border);
  border-radius:var(--radius-sm);color:var(--text-mid);
  font-size:12px;letter-spacing:.04em;cursor:pointer;
  font-family:inherit;transition:all .1s;
}
.sbtn:active{background:var(--bg3);color:var(--text);}
.dp.at-limit{border-color:#ff8c00 !important;color:#ff8c00 !important;}
.dp.at-limit:active{background:var(--orange-dim);box-shadow:0 0 14px rgba(255,140,0,.4);}
#btn-trim{
  margin-top:4px;padding:4px 10px;font-size:10px;letter-spacing:.06em;
  background:var(--bg3);border:1px solid #1a5035;
  border-radius:5px;color:#3a8060;cursor:pointer;
  font-family:"Courier New",monospace;white-space:nowrap;
  transition:all .15s;display:block;width:calc(100% - 16px);margin-inline:8px;
}
#btn-trim:not(.water-off):active{
  background:var(--green-dim);color:var(--green);
  border-color:var(--green);box-shadow:0 0 12px var(--green-glow);
}
/* Water off: visible but clearly inactive */
#btn-trim.water-off{color:var(--text-dim);border-color:var(--border);background:var(--btn);cursor:default;}
#btn-trim.busy{opacity:.6;pointer-events:none;}

#btn-water{border-color:#1a5035 !important;color:#3a8060;}
#btn-water.on{
  background:var(--green-dim) !important;
  color:var(--green) !important;
  box-shadow:0 0 12px var(--green-glow);
}

/* ── Footer ── */
#footer{
  flex-shrink:0;display:flex;gap:10px;
  padding:8px 16px 24px;
}
#btn-cancel{
  flex:1;padding:15px 0;
  background:var(--btn);border:1px solid var(--border);
  border-radius:var(--radius);color:var(--text-mid);
  font-size:15px;cursor:pointer;font-family:inherit;
  transition:all .1s;
}
#btn-cancel:active{background:var(--bg3);}
#btn-save{
  flex:2;padding:15px 0;
  background:var(--green);border-radius:var(--radius);
  color:#060c10;font-size:15px;font-weight:700;cursor:pointer;
  font-family:inherit;
  box-shadow:0 2px 24px rgba(0,232,122,.35);
  transition:all .1s;
}
#btn-save:disabled{
  background:var(--green-dim);color:var(--text-dim);
  box-shadow:none;cursor:default;
}
#btn-save:not(:disabled):active{opacity:.82;}
/* Light-theme override: solid bright green on white is too loud — match
   the landing "Water Zone" .btn-primary look (light green bg, darker
   green text, subtle border, no glow). */
:root[data-theme="light"] #btn-save{
  background:var(--green-dim);color:var(--green);
  border:1px solid #1a5035;box-shadow:none;
}
:root[data-theme="light"] #btn-save:disabled{
  background:var(--bg3);color:var(--text-mid);
  border-color:var(--border);
}

/* ── Toast ── */
#toast{
  position:fixed;bottom:90px;left:50%;transform:translateX(-50%);
  background:rgba(0,232,122,.92);color:#040c08;
  padding:10px 22px;border-radius:20px;
  font-size:13px;font-weight:700;letter-spacing:.04em;
  font-family:'Courier New',monospace;
  opacity:0;transition:opacity .3s;pointer-events:none;
  white-space:nowrap;
}
</style>
</head>
<body>

<header>
  <a class="back" href="/">&#8592;</a>
  <div class="logo">Irrigoto</div>
  <div class="header-mid">Zone Setup</div>
  <div id="conn"><div id="conn-dot"></div><span id="conn-label">connecting</span></div>
</header>

<div id="map-wrap"><canvas id="map"></canvas>
  <div id="heat-legend">
    <div style="display:flex;gap:5px;align-items:stretch">
      <div class="hl-bar" style="height:90px"></div>
      <div class="hl-ticks" style="height:90px"><span>200%</span><span>target<br>3.2mm</span><span>50%</span></div>
    </div>
    <div id="hl-label"></div>
  </div>
</div>

<div id="readout">
  <div class="rcel"><div class="rlabel">Bearing</div><div class="rval" id="v-bearing">--.-°</div></div>
  <div class="rcel" style="position:relative;"><div class="rlabel">Throw</div><div class="rval" id="v-throw">-- ft</div><button id="btn-trim" class="water-off" onclick="doTrim()" title="Trim to actual">⊙ trim</button></div>
  <div class="rcel"><div class="rlabel">Points</div><div class="rval" id="v-points">0 / 36</div></div>
</div>

<div id="dpad-section">
  <div id="dpad">
    <div class="dp-empty"></div>
    <button class="dp" id="btn-up"
      onpointerdown="holdStart('pres_up',this)" onpointerup="holdStop()" onpointerleave="holdStop()">▲</button>
    <div class="dp-empty"></div>

    <button class="dp" id="btn-left"
      onpointerdown="holdStart('nozzle_ccw',this)" onpointerup="holdStop()" onpointerleave="holdStop()">◀</button>
    <button class="dp" id="btn-add" onclick="doAct('add_pt')">+</button>
    <button class="dp" id="btn-right"
      onpointerdown="holdStart('nozzle_cw',this)" onpointerup="holdStop()" onpointerleave="holdStop()">▶</button>

    <div class="dp-empty"></div>
    <button class="dp" id="btn-dn"
      onpointerdown="holdStart('pres_dn',this)" onpointerup="holdStop()" onpointerleave="holdStop()">▼</button>
    <div class="dp-empty"></div>
  </div>
</div>

<div id="secondary">
  <button class="sbtn" id="btn-water" onclick="doAct('water_toggle')">💧 Water</button>
  <button class="sbtn" onclick="doAct('undo')">⌫ Undo</button>
  <button class="sbtn" onclick="doAct('clear')">✕ Clear</button>
  <button class="sbtn" id="btn-path" onclick="togglePath()">◎ Path</button>
  <button class="sbtn" id="btn-heatmap" onclick="toggleHeatmap()">🌡 Depth</button>
  <button class="sbtn" id="btn-edit" onclick="toggleEdit()">✎ Edit</button>
</div>
<!-- b400: edit-mode toolbar (drag/delete interior points). Shown only while
     editing; the d-pad/secondary/footer hide. Save writes via /zone/import. -->
<div id="edit-bar" style="display:none;gap:6px;padding:8px 16px;align-items:center;justify-content:center">
  <button class="sbtn" onclick="deleteSelPoint()">🗑 Delete pt</button>
  <button class="sbtn" id="btn-edit-save" onclick="saveEdits()">✓ Save edits</button>
  <button class="sbtn" onclick="cancelEdit()">✕ Done</button>
</div>

<div id="name-row">
  <label for="zone-name">Name</label>
  <input id="zone-name" type="text" maxlength="31" placeholder="Zone name" autocomplete="off" autocorrect="off" spellcheck="false">
</div>

<div id="footer">
  <button id="btn-cancel" onclick="doAct('cancel')">Cancel</button>
  <button id="btn-save" id="btn-save" onclick="doAct('save')" disabled>Save Zone</button>
</div>

<div id="toast"></div>

<script>
// ── Canvas ──────────────────────────────────────────────────────
const CV = document.getElementById('map');
const ctx = CV.getContext('2d');
let ST = {bearing:0, throw_mm:4267, throw_ft:14, pressure_pct:50, water:false, points:[], act_max_throw:10058, act_min_throw:0};
// b400: on-device point editor state. _cx/_cy/_maxR are captured each draw() so
// the canvas pointer handlers can hit-test / invert the projection.
let _editMode=false, _selPt=-1, _dragging=false, _cx=0, _cy=0, _maxR=1;
// Display scale (mm at canvas rim): device's reported max throw (calibrated
// reach, or live throw if pressure exceeds the cal table) + 3ft (914mm) buffer.
// Tracks calibration, so the radar stays correctly scaled at any water pressure.
function _edScale(){ return (ST.act_max_throw||10058) + 914; }
const TRAIL_MAX = 120;
let waterTrail = [];
let lastWaterBearing = null;
let showPath = false;
let heatPasses = 0;
let lastRun = null;
let waterCSVRows = null;
const _zoneIdParam = new URLSearchParams(window.location.search).get('id')||'0';
fetch('/zone/last_water?id='+_zoneIdParam).then(r=>r.json()).then(d=>{
  if(d.num_rings>0){lastRun=d; if(heatPasses===2) draw();}
}).catch(()=>{});
(function loadCSV(){
  fetch('/zone/water_csv?id='+_zoneIdParam).then(r=>{
    if(!r.ok) throw new Error('csv '+r.status);
    return r.text();
  }).then(txt=>{
    const lines=txt.trim().split('\n');
    if(lines.length<2){waterCSVRows=[];if(heatPasses===2)draw();return;}
    const hdr=lines[0].split(',');
    const idx=k=>hdr.indexOf(k);
    if(idx('nozzle_deg_actual')<0){waterCSVRows=[];if(heatPasses===2)draw();return;}
    waterCSVRows=lines.slice(1).map(l=>{
      const c=l.split(',');
      const pt=+c[idx('pass_type')];
      return {ring:+c[idx('ring')],sector:+c[idx('sector')],
              actDeg:+c[idx('nozzle_deg_actual')],tgtDeg:+c[idx('nozzle_deg_target')],
              actMm:+c[idx('throw_mm_actual')],tgtMm:+c[idx('throw_mm_target')],
              psiAct:+c[idx('pressure_psi_actual')],psiTgt:+c[idx('pressure_psi_target')],
              passType:pt,
              // Aggregate records (passType=255): time_s encodes total depth_mm × 10
              totalDepth: pt===255 ? +c[idx('time_s')]/10 : 0};
    }).filter(r=>r.actMm>100);
    if(heatPasses===2) draw();
  }).catch(e=>{console.warn('CSV load failed:',e);waterCSVRows=[];if(heatPasses===2)draw();});
})();

function togglePath(){
  showPath = !showPath;
  const btn = document.getElementById('btn-path');
  btn.style.color = showPath ? 'var(--green)' : '';
  btn.style.borderColor = showPath ? 'var(--green)' : '';
  draw();
}

function toggleHeatmap(){
  heatPasses = (heatPasses + 1) % 3;
  const btn = document.getElementById('btn-heatmap');
  btn.innerHTML = ['🌡 Depth','🌡 Predict','🌡 Actual'][heatPasses];
  btn.style.color = heatPasses ? 'var(--green)' : '';
  btn.style.borderColor = heatPasses ? 'var(--green)' : '';
  const leg=document.getElementById('heat-legend');
  const hlLbl=document.getElementById('hl-label');
  if(heatPasses>0){
    leg.style.display='flex';
    hlLbl.textContent=heatPasses===2?'actual run':'predicted';
  } else {
    leg.style.display='none';
  }
  resizeCanvas();
}

function drawHeatMap(W, H, cx, cy, maxR) {
  if (!heatPasses || ST.points.length < 3) return;
  const passes = heatPasses;

  // Walk-order sorted polygon for pip test
  const wpts=[...ST.points].sort((a,b)=>((a.widx??99)-(b.widx??99))||a.deg-b.deg);
  function pip(bearing, r_mm) {
    const px=r_mm*Math.sin(bearing*Math.PI/180), py=r_mm*Math.cos(bearing*Math.PI/180);
    let c=0;
    for(let i=0;i<wpts.length;i++){
      const j=(i+1)%wpts.length,pi=wpts[i],pj=wpts[j];
      const x1=pi.throw_mm*Math.sin(pi.deg*Math.PI/180)-px,y1=pi.throw_mm*Math.cos(pi.deg*Math.PI/180)-py;
      const x2=pj.throw_mm*Math.sin(pj.deg*Math.PI/180)-px,y2=pj.throw_mm*Math.cos(pj.deg*Math.PI/180)-py;
      if((y1>0)!==(y2>0)){const t=y1/(y1-y2);if(x1+t*(x2-x1)>0)c++;}
    }
    return c%2===1;
  }

  // Zone arc bounds
  const sdegs=ST.points.map(p=>p.deg).sort((a,b)=>a-b);
  let mg=0,gi=0;
  for(let i=0;i<sdegs.length;i++){const nxt=i<sdegs.length-1?sdegs[i+1]:sdegs[0]+360;if(nxt-sdegs[i]>mg){mg=nxt-sdegs[i];gi=i;}}
  let arcStart=sdegs[(gi+1)%sdegs.length];
  let arcSpan=((sdegs[gi]-arcStart+360)%360)||360;

  // Ring generation (mirrors firmware)
  const actMax=ST.act_max_throw||10058;
  const actMin=(ST.act_min_throw&&ST.act_min_throw>50)?ST.act_min_throw:Math.min(...ST.points.map(p=>p.throw_mm));
  const zoneMax=Math.max(...ST.points.map(p=>p.throw_mm));
  const zoneThrowMin=Math.min(...ST.points.map(p=>p.throw_mm));
  // Sprinkler-in-center: all walk points at same throw => gap in path is not an exclusion
  const sprinklerInCenter=zoneMax>0&&(zoneMax-zoneThrowMin)/zoneMax<0.05;
  if(sprinklerInCenter){arcStart=0;arcSpan=360;}
  // pip(0,1): test 1mm north — if true, sprinkler is inside the polygon.
  // Force 360 deg arc so coverage is polygon-clipped, not bearing-gap-clipped.
  // (The gap-detection arc is wrong when the sprinkler is inside the polygon.)
  const originInside=pip(0,1);
  if(!sprinklerInCenter&&originInside){arcStart=0;arcSpan=360;}
  const rings=[];let t=zoneMax;
  while(t>=actMin&&rings.length<36){rings.push(t);t-=Math.max(700*(t/actMax),80);}
  if(!rings.length)return;
  // Inner rings below actMin for sprinkler-inside-polygon zones.
  const WATER_MIN_THROW_JS=461;
  const innerRingStart=rings.length;
  if(!sprinklerInCenter&&originInside&&actMin>WATER_MIN_THROW_JS){
    while(t>WATER_MIN_THROW_JS&&rings.length<36){rings.push(t);t-=Math.max(700*(t/actMax),80);}
  }

  const MIN_ELLIPSE=1829, SPLASH_R=300, STEP=1.0;

  // Per-ring arc bounds from zone polygon circle intersection.
  // Returns {s, e} where s=CW arc start, e=CW arc end (e may exceed 360).
  // Falls back to full zone arc if < 2 intersections found.
  function ringArcBounds(r_mm) {
    const bearings=[];
    for(let i=0;i<wpts.length;i++){
      const j=(i+1)%wpts.length,pi=wpts[i],pj=wpts[j];
      const x1=pi.throw_mm*Math.sin(pi.deg*Math.PI/180);
      const y1=pi.throw_mm*Math.cos(pi.deg*Math.PI/180);
      const x2=pj.throw_mm*Math.sin(pj.deg*Math.PI/180);
      const y2=pj.throw_mm*Math.cos(pj.deg*Math.PI/180);
      const dx=x2-x1,dy=y2-y1;
      const a=dx*dx+dy*dy;
      if(a<1)continue;
      const b2=2*(x1*dx+y1*dy);
      const c=x1*x1+y1*y1-r_mm*r_mm;
      const disc=b2*b2-4*a*c;
      if(disc<0)continue;
      const sq=Math.sqrt(disc);
      for(const sg of[1,-1]){
        const t=(-b2+sg*sq)/(2*a);
        if(t<-0.001||t>1.001)continue;
        const ix=x1+t*dx,iy=y1+t*dy;
        const bear=(Math.atan2(ix,iy)*180/Math.PI+360)%360;
        const cwDist=(bear-arcStart+360)%360;
        if(cwDist<=arcSpan+0.5)bearings.push(bear);
      }
    }
    if(bearings.length<2)return{s:arcStart,e:arcStart+arcSpan};
    bearings.sort((a,b)=>((a-arcStart+360)%360)-((b-arcStart+360)%360));
    let e=bearings[bearings.length-1];
    if(e<bearings[0])e+=360;
    return{s:bearings[0],e};
  }

  function dcolor(ratio){
    if(ratio<0.6)  return 'rgba(60,140,255,0.55)';
    if(ratio<0.88) return 'rgba(60,200,180,0.55)';
    if(ratio<1.12) return 'rgba(40,210,80,0.65)';
    if(ratio<1.5)  return 'rgba(255,195,40,0.65)';
    return               'rgba(255,90,40,0.65)';
  }

  rings.forEach((rt,ri)=>{
    let rO,rI;
    if(rt<MIN_ELLIPSE){rO=rt+SPLASH_R;rI=Math.max(0,rt-SPLASH_R);}
    else{rO=ri===0?rt:(rings[ri-1]+rt)/2;rI=ri===rings.length-1?(ri>=innerRingStart?WATER_MIN_THROW_JS:actMin):(rt+rings[ri+1])/2;}
    const rOPx=rO/_edScale()*maxR, rIPx=Math.max(0,rI/_edScale()*maxR);
    if(rOPx<=rIPx+0.5)return;

    // Count active sectors (10-deg steps = WATER_SECTOR_DEG)
    let activeSectors=0;
    for(let o=0;o<=arcSpan;o+=10)if(pip((arcStart+o)%360,rt))activeSectors++;
    if(activeSectors<1)return;
    const arcFrac=activeSectors/36;

    const rO_m=rO/1000,rI_m=rI/1000,r_m=rt/1000;
    const areaPerDeg=r_m*(Math.PI/180)*(rO_m-rI_m);
    let depth,targetDepth;
    if (passes===2) {
      return;  // actual mode handled separately below
    } else {
      // Predicted: firmware model (pressPct from ring_throw/actMax)
      targetDepth=3.175*passes;
      const pressPct=rt/actMax*100;
      const flowLpm=0.12*pressPct+1.0;
      const area_m2=Math.PI*(rO_m*rO_m-rI_m*rI_m)*arcFrac;
      const timeMins=targetDepth*area_m2/flowLpm;
      const dps=(activeSectors*10)/(timeMins*60);
      depth=(flowLpm/60/1000)/Math.max(dps,0.01)/areaPerDeg*1000;
    }
    const col=dcolor(depth/targetDepth);

    // Draw polygon-clipped arc using pip sweep (handles complex zone shapes)
    {
      const HSTEP=1.5;const hspans=[];let hss=null;
      for(let o=0;o<=arcSpan+HSTEP;o+=HSTEP){
        const ins=pip((arcStart+o)%360,rt);
        if(ins&&hss===null)hss=o;
        if(!ins&&hss!==null){hspans.push({lo:(arcStart+hss+360)%360,sp:o-hss});hss=null;}
      }
      if(hss!==null)hspans.push({lo:(arcStart+hss+360)%360,sp:arcSpan-hss});
      hspans.forEach(hs=>{
        let he=(hs.lo+hs.sp-90)*Math.PI/180;
        const hl=(hs.lo-90)*Math.PI/180;
        if(he<=hl)he+=2*Math.PI;
        ctx.beginPath();ctx.arc(cx,cy,rOPx,hl,he);ctx.arc(cx,cy,rIPx,he,hl,true);
        ctx.closePath();ctx.fillStyle=col;ctx.fill();
      });
    }

    // Depth label if band wide enough and mid-arc is inside zone
    if(rOPx-rIPx>11){
      const mb=(arcStart+arcSpan*0.5)%360;
      if(pip(mb,(rO+rI)/2)){
        const mr=(rO+rI)/2/_edScale()*maxR, ma=(mb-90)*Math.PI/180;
        ctx.font='8px sans-serif';ctx.textAlign='center';ctx.textBaseline='middle';
        ctx.fillStyle='rgba(255,255,255,0.80)';
        ctx.fillText(depth.toFixed(1),cx+Math.cos(ma)*mr,cy+Math.sin(ma)*mr);
      }
    }
  });

  // ---- Actual-mode rendering: cell-based from CSV measurements ----
  // Each CSV row is drawn as an individual sector patch at the actual
  // nozzle bearing x actual throw, colored by actual water depth.
  // This shows spatial variation in both bearing offset and throw error.
  if (passes===2) {
    if (!lastRun) {
      ctx.save();
      ctx.font='bold 13px sans-serif';ctx.textAlign='center';ctx.textBaseline='middle';
      ctx.fillStyle='rgba(200,224,212,0.55)';
      ctx.fillText('No watering data yet',cx,cy-14);
      ctx.font='11px sans-serif';ctx.fillStyle='rgba(200,224,212,0.35)';
      ctx.fillText('Run a watering cycle first',cx,cy+6);
      ctx.restore(); return;
    }
    if (waterCSVRows === null) {
      ctx.save();
      ctx.font='bold 13px sans-serif';ctx.textAlign='center';ctx.textBaseline='middle';
      ctx.fillStyle='rgba(200,224,212,0.55)';
      ctx.fillText('Loading watering data...',cx,cy);
      ctx.restore(); return;
    }
    if (!waterCSVRows.length) {
      ctx.save();
      ctx.font='bold 13px sans-serif';ctx.textAlign='center';ctx.textBaseline='middle';
      ctx.fillStyle='rgba(200,224,212,0.55)';
      ctx.fillText('No CSV data -- water zone to generate',cx,cy);
      ctx.restore(); return;
    }
    try {
    const LR = lastRun.rings;
    const SECTOR_DEG = 10;
    const targetDepth = 3.175; // mm (1/8 inch per pass)

    // Build sorted list of ring average throws from CSV for radial bounds
    const ringThrowMap = {};
    waterCSVRows.forEach(r=>{
      if(!ringThrowMap[r.ring]) ringThrowMap[r.ring]={sum:0,n:0};
      ringThrowMap[r.ring].sum += r.actMm;
      ringThrowMap[r.ring].n++;
    });
    const sortedRings = Object.keys(ringThrowMap).map(Number).sort((a,b)=>a-b);
    const ringAvgMm = {};
    sortedRings.forEach(r=>{ ringAvgMm[r]=ringThrowMap[r].sum/ringThrowMap[r].n; });

    // Rasterize each measurement as an exact annular sector onto the depth buffer.
    // No Gaussian splat = no angular over-counting. CW + CCW passes that land
    // on the same pixel genuinely accumulate (double coverage = double depth).
    const LOG_DEG = 2.0;
    const BW=Math.round(W), BH=Math.round(H);
    const depthBuf = new Float32Array(BW*BH);

    waterCSVRows.forEach(row=>{
      const {ring, actDeg, actMm, psiAct, passType, totalDepth} = row;
      const isAggr = passType===255;
      // Aggregate records cover a full 10° sector and carry pre-computed total depth.
      // Per-pass records cover a 2° sample and derive depth from flow model.
      const patchDeg = isAggr ? 10.0 : LOG_DEG;
      const ri = sortedRings.indexOf(ring);
      const rawO = ri===0 ? actMm*1.08 : (ringAvgMm[sortedRings[ri-1]]+actMm)/2;
      const rawI = ri===sortedRings.length-1 ? actMm*0.92 : (actMm+ringAvgMm[sortedRings[ri+1]])/2;
      let depth;
      if (isAggr) {
        depth = totalDepth;  // stored directly as mm of water across all passes
      } else {
        const dps = (LR && LR[ring-1] && LR[ring-1].dps) ? LR[ring-1].dps : 1.0;
        const flowLpm = 0.12*(psiAct/5.034*100)+1.0;
        const r_m=actMm/1000, dr_m=(rawO-rawI)/1000;
        const area_m2 = Math.PI*((r_m+dr_m/2)**2-(r_m-dr_m/2)**2)*(LOG_DEG/360);
        depth = (flowLpm/60/1000)*(LOG_DEG/dps)/Math.max(area_m2,1e-6)*1000;
      }

      // Sector bearing span
      const bearLo = actDeg - patchDeg/2;
      const bearHi = actDeg + patchDeg/2;
      const rOPx = rawO/_edScale()*maxR, rIPx = Math.max(0, rawI/_edScale()*maxR);

      // Tight bounding box around this sector
      const cRad = (actDeg-90)*Math.PI/180;
      const midRPx = (rOPx+rIPx)/2;
      const halfRad = (rOPx-rIPx)/2 + 2;
      const halfAng = rIPx*Math.sin(patchDeg/2*Math.PI/180) + 2;
      const bcx=cx+midRPx*Math.cos(cRad), bcy=cy+midRPx*Math.sin(cRad);
      const box=halfRad+halfAng;
      const x0=Math.max(0,Math.floor(bcx-box)), x1=Math.min(BW-1,Math.ceil(bcx+box));
      const y0=Math.max(0,Math.floor(bcy-box)), y1=Math.min(BH-1,Math.ceil(bcy+box));

      for(let px2=x0;px2<=x1;px2++){
        for(let py2=y0;py2<=y1;py2++){
          const dx=px2-cx, dy=py2-cy;
          const r2=Math.sqrt(dx*dx+dy*dy);
          if(r2<rIPx||r2>rOPx) continue;
          const bear=((Math.atan2(dx,-dy)*180/Math.PI)+360)%360;
          let inSec;
          if(bearHi>=360) inSec=bear>=bearLo||bear<=bearHi-360;
          else if(bearLo<0) inSec=bear>=bearLo+360||bear<=bearHi;
          else inSec=bear>=bearLo&&bear<=bearHi;
          if(inSec) depthBuf[py2*BW+px2]+=depth;
        }
      }
    });

    // Color-map accumulated depth buffer and blit onto canvas
    const imgData=ctx.createImageData(BW,BH);
    const d32=imgData.data;
    for(let i=0;i<BW*BH;i++){
      const dep=depthBuf[i];
      if(dep<0.005) continue;
      const ratio=dep/targetDepth;
      // Color ramp: blue(<0.6) teal(0.6-0.88) green(0.88-1.12) amber(1.12-1.5) red(>1.5)
      let r=0,g=0,b2=0,a=0;
      if(ratio<0.60){r=40;g=90+Math.round(ratio/0.60*80);b2=200;a=Math.round(ratio/0.60*200);}
      else if(ratio<0.88){const t=(ratio-0.60)/0.28;r=40;g=170+Math.round(t*50);b2=Math.round(200*(1-t));a=200;}
      else if(ratio<1.12){const t=(ratio-0.88)/0.24;r=Math.round(40+t*120);g=220;b2=0;a=220;}
      else if(ratio<1.50){const t=(ratio-1.12)/0.38;r=160+Math.round(t*80);g=Math.round(220*(1-t*0.5));b2=0;a=230;}
      else{r=240;g=Math.max(0,80-Math.round((ratio-1.5)*40));b2=0;a=240;}
      const base=i*4;
      d32[base]=r; d32[base+1]=g; d32[base+2]=b2; d32[base+3]=Math.min(255,a);
    }
    // Render to temp canvas then blur onto main canvas for organic spread.
    // The sector rasterization gives accurate depth; blur represents physical
    // spray spread (elliptical at each pause, smear while moving).
    const tmpC = document.createElement('canvas');
    tmpC.width = BW; tmpC.height = BH;
    tmpC.getContext('2d').putImageData(imgData, 0, 0);
    ctx.save();
    ctx.filter = 'blur(3px)';
    ctx.drawImage(tmpC, 0, 0);
    ctx.restore();

    // Redraw zone polygon outline on top so boundary is always visible
    if(ST.points.length>=3){
      const pts=ST.points;
      ctx.save();
      ctx.beginPath();
      pts.forEach((p,i)=>{
        const r=p.throw_mm/_edScale()*maxR;
        const a=(p.deg-90)*Math.PI/180;
        i?ctx.lineTo(cx+Math.cos(a)*r,cy+Math.sin(a)*r)
          :ctx.moveTo(cx+Math.cos(a)*r,cy+Math.sin(a)*r);
      });
      ctx.closePath();
      ctx.strokeStyle='rgba(0,255,160,0.6)';
      ctx.lineWidth=1.5;
      ctx.stroke();
      ctx.restore();
    }

    // Legend -- gradient bar with depth labels
    return;
  } catch(e){ console.error('Actual heatmap error:',e); } }

}

function drawPath(W, H, cx, cy, maxR) {
  if (ST.points.length < 2) return;
  function pointInZone(bearing, r_mm) {
    const px=r_mm*Math.sin(bearing*Math.PI/180),py=r_mm*Math.cos(bearing*Math.PI/180);
    let crosses=0;
    const n=ST.points.length;
    for(let i=0;i<n;i++){
      const j=(i+1)%n,pi=ST.points[i],pj=ST.points[j];
      const x1=pi.throw_mm*Math.sin(pi.deg*Math.PI/180)-px,y1=pi.throw_mm*Math.cos(pi.deg*Math.PI/180)-py;
      const x2=pj.throw_mm*Math.sin(pj.deg*Math.PI/180)-px,y2=pj.throw_mm*Math.cos(pj.deg*Math.PI/180)-py;
      if((y1>0)!==(y2>0)){const t=y1/(y1-y2);if(x1+t*(x2-x1)>0)crosses++;}
    }
    return(crosses%2===1);
  }
  const sdegs=ST.points.map(p=>p.deg).sort((a,b)=>a-b);
  let mg=0,gi=0;
  for(let i=0;i<sdegs.length;i++){const n=i<sdegs.length-1?sdegs[i+1]:sdegs[0]+360;if(n-sdegs[i]>mg){mg=n-sdegs[i];gi=i;}}
  let arcStart=sdegs[(gi+1)%sdegs.length],arcEnd=sdegs[gi];
  let arcSpan=((arcEnd-arcStart+360)%360)||360;
  const actMax=ST.act_max_throw||10058;
  const zoneMax=Math.max(...ST.points.map(p=>p.throw_mm));
  const zoneMin=(ST.act_min_throw&&ST.act_min_throw>50)?ST.act_min_throw:Math.min(...ST.points.map(p=>p.throw_mm));
  const zoneThrowMin2=Math.min(...ST.points.map(p=>p.throw_mm));
  // Sprinkler-in-center: all walk points at same throw => gap is not an exclusion
  const pathSIC=zoneMax>0&&(zoneMax-zoneThrowMin2)/zoneMax<0.05;
  if(pathSIC){arcStart=0;arcSpan=360;}
  // Origin-inside-polygon: force 360 deg arc so polygon boundary clips coverage
  const pathOriginInside=pointInZone(0,1);
  if(!pathSIC&&pathOriginInside){arcStart=0;arcSpan=360;}
  const rings=[];let t=zoneMax;
  while(t>=zoneMin&&rings.length<36){rings.push(t);t-=Math.max(700*(t/actMax),80);}
  if(!rings.length)return;
  // Inner rings below zoneMin for sprinkler-inside-polygon zones
  if(!pathSIC&&pathOriginInside&&zoneMin>461){
    while(t>461&&rings.length<36){rings.push(t);t-=Math.max(700*(t/actMax),80);}
  }
  const STEP=0.5;
  for(let ri=0;ri<rings.length;ri++){
    const thr=rings[ri],r=(thr/_edScale())*maxR;
    const cw=(ri%2===0),al=0.70-0.30*(ri/(rings.length-1||1));
    const spans=[];let spanStart=null;
    for(let o=0;o<=arcSpan+STEP;o+=STEP){
      const bearing=(arcStart+o)%360,inside=pointInZone(bearing,thr);
      if(inside&&spanStart===null)spanStart=o;
      if(!inside&&spanStart!==null){spans.push({lo:(arcStart+spanStart+360)%360,span:o-spanStart});spanStart=null;}
    }
    if(spanStart!==null)spans.push({lo:(arcStart+spanStart+360)%360,span:arcSpan-spanStart});
    for(const {lo,span} of spans){
      if(span<0.5)continue;
      const sa=(lo-90)*Math.PI/180;
      ctx.beginPath();ctx.arc(cx,cy,r,sa,sa+span*Math.PI/180,false);
      ctx.strokeStyle=cw?'rgba(80,180,255,'+al+')':'rgba(255,160,60,'+al+')';
      ctx.lineWidth=1.5;ctx.setLineDash([]);ctx.stroke();
    }
  }
}

function resizeCanvas(){
  const wrap = document.getElementById('map-wrap');
  const sz = Math.min(wrap.clientWidth - 32, wrap.clientHeight - 16, 330) | 0;
  if(CV.width !== sz){ CV.width = sz; CV.height = sz; }
  draw();
}

function draw(){
  const W = CV.width, H = CV.height, cx = W>>1, cy = H>>1;
  const maxR = cx - 4;
  _cx=cx; _cy=cy; _maxR=maxR;   // b400: for the point-editor hit-test/drag

  ctx.clearRect(0,0,W,H);

  // Background — solid fill from the CSS --radar-bg variable so the
  // radar matches the landing-page thumbnails and follows the theme.
  // Only the inside of the outermost circle is filled; the corners
  // stay transparent and show the page bg.
  const radarBg = getComputedStyle(document.documentElement)
                  .getPropertyValue('--radar-bg').trim() || '#0a1a12';
  ctx.beginPath(); ctx.arc(cx,cy,maxR,0,Math.PI*2);
  ctx.fillStyle = radarBg; ctx.fill();

  // Range rings + labels. Rings are stroked in --radar-grid (theme-
  // tuned to sit just barely above the radar bg). Labels use
  // --text-mid (more legible) with the foot symbol "5'" / "10'".
  const css        = getComputedStyle(document.documentElement);
  const ringColor  = css.getPropertyValue('--radar-grid').trim() || '#15281e';
  const labelColor = css.getPropertyValue('--text-mid').trim() || '#608070';
  ctx.strokeStyle = ringColor;
  ctx.lineWidth   = 0.7;
  // Range rings + labels — tick list derived from the domain so the grid
  // adapts to any calibrated max throw (and thus any supply pressure).
  const _maxFt = (ST.act_max_throw||10058)/304.8;
  const _step  = _maxFt > 40 ? 10 : 5;
  const _ft = [];
  for(let f=_step; f<=_maxFt+0.01; f+=_step) _ft.push(f);
  _ft.forEach(ft => {
    const r = (ft*304.8/_edScale()) * maxR;
    ctx.beginPath(); ctx.arc(cx, cy, r, 0, Math.PI*2); ctx.stroke();
  });
  ctx.fillStyle  = labelColor;
  ctx.font       = '9px "Courier New",monospace';
  ctx.textAlign  = 'left';
  ctx.textBaseline = 'middle';
  _ft.forEach(ft => {
    const r = (ft*304.8/_edScale()) * maxR;
    ctx.fillText(ft + "'", cx + r + 3, cy - 5);
  });

  // Zone polygon
  if(ST.points.length >= 2){
    ctx.beginPath();
    ST.points.forEach((p,i)=>{
      const rr=(p.throw_mm/_edScale())*maxR;
      const rad=(p.deg-90)*Math.PI/180;
      i===0 ? ctx.moveTo(cx+Math.cos(rad)*rr, cy+Math.sin(rad)*rr)
            : ctx.lineTo(cx+Math.cos(rad)*rr, cy+Math.sin(rad)*rr);
    });
    ctx.closePath();
    ctx.fillStyle='rgba(255,140,0,.09)'; ctx.fill();
    ctx.strokeStyle='rgba(255,140,0,.55)'; ctx.lineWidth=1.5; ctx.stroke();
  }

  // Current throw ring (dashed)
  const throwR=(ST.throw_mm/_edScale())*maxR;
  ctx.beginPath(); ctx.arc(cx,cy,throwR,0,Math.PI*2);
  ctx.strokeStyle='rgba(0,232,122,.12)'; ctx.lineWidth=1;
  ctx.setLineDash([3,5]); ctx.stroke(); ctx.setLineDash([]);

  // Overlays (drawn before nozzle line so line stays on top)
  drawHeatMap(W, H, cx, cy, maxR);
  if (showPath) drawPath(W, H, cx, cy, maxR);

  // Water trail dots
  const actualThrowMm = ST.actual_throw_mm || 0;
  if (waterTrail.length > 1) {
    for (let i = 0; i < waterTrail.length; i++) {
      const age = i / waterTrail.length;
      const tr = (waterTrail[i].r/_edScale())*maxR;
      const tbrad = (waterTrail[i].b - 90)*Math.PI/180;
      ctx.beginPath(); ctx.arc(cx+Math.cos(tbrad)*tr, cy+Math.sin(tbrad)*tr, 2.5, 0, Math.PI*2);
      ctx.fillStyle=`rgba(0,232,122,${(age*0.55).toFixed(2)})`; ctx.fill();
    }
  }

  // Nozzle bearing line
  const brad=(ST.bearing-90)*Math.PI/180;
  const bx=cx+Math.cos(brad)*maxR, by=cy+Math.sin(brad)*maxR;
  ctx.beginPath(); ctx.moveTo(cx,cy); ctx.lineTo(bx,by);
  ctx.strokeStyle='rgba(0,232,122,.55)'; ctx.lineWidth=1.5;
  ctx.shadowBlur=14; ctx.shadowColor='#00e87a'; ctx.stroke(); ctx.shadowBlur=0;

  // Throw indicator dot — at actual throw when valve open, else slider position
  const actualThrowR = actualThrowMm > 100 ? (actualThrowMm/_edScale())*maxR : throwR;
  const dotIsActual = actualThrowMm > 100;
  const tx=cx+Math.cos(brad)*actualThrowR, ty=cy+Math.sin(brad)*actualThrowR;
  ctx.beginPath(); ctx.arc(tx,ty,5.5,0,Math.PI*2);
  ctx.fillStyle = dotIsActual ? '#00e87a' : 'rgba(0,232,122,.5)';
  ctx.shadowBlur = dotIsActual ? 22 : 18;
  ctx.shadowColor='#00e87a'; ctx.fill(); ctx.shadowBlur=0;
  if (dotIsActual) {
    ctx.beginPath(); ctx.arc(tx,ty,9,0,Math.PI*2);
    ctx.strokeStyle='rgba(0,232,122,.35)'; ctx.lineWidth=1.5; ctx.stroke();
  }

  // Perimeter points
  ST.points.forEach((p,i)=>{
    const rr=(p.throw_mm/_edScale())*maxR;
    const rad=(p.deg-90)*Math.PI/180;
    const x=cx+Math.cos(rad)*rr, y=cy+Math.sin(rad)*rr;
    const sel=(_editMode && i===_selPt);           // b400: highlight selected pt
    ctx.beginPath(); ctx.arc(x,y, sel?8:(_editMode?6:5), 0,Math.PI*2);
    ctx.fillStyle = sel ? '#00e87a' : '#ff8c00';
    ctx.shadowBlur = sel?16:10; ctx.shadowColor = sel?'#00e87a':'#ff8c00';
    ctx.fill(); ctx.shadowBlur=0;
    // Point index number
    ctx.fillStyle='#060c10';
    ctx.font='bold 8px -apple-system,sans-serif';
    ctx.textAlign='center'; ctx.textBaseline='middle';
    ctx.fillText(i+1, x, y);
  });

  // Sprinkler
  ctx.beginPath(); ctx.arc(cx,cy,7,0,Math.PI*2);
  ctx.fillStyle='#ffd700';
  ctx.shadowBlur=18; ctx.shadowColor='#ffd700'; ctx.fill(); ctx.shadowBlur=0;
  ctx.beginPath(); ctx.arc(cx,cy,2.5,0,Math.PI*2);
  ctx.fillStyle='#060c10'; ctx.fill();
}

// ── State update ────────────────────────────────────────────────
function applyState(s){
  // Record water trail from actual throw
  const athr = s.actual_throw_mm || 0;
  if (athr > 100) {
    if (lastWaterBearing === null || Math.abs(s.bearing - lastWaterBearing) > 1.5) {
      waterTrail.push({b: s.bearing, r: athr});
      if (waterTrail.length > TRAIL_MAX) waterTrail.shift();
      lastWaterBearing = s.bearing;
    }
  } else {
    if (waterTrail.length > 0) waterTrail = [];
    lastWaterBearing = null;
  }
  if (s.act_max_throw !== undefined) s.act_max_throw = s.act_max_throw;
  if (s.act_min_throw !== undefined) s.act_min_throw = s.act_min_throw;
  ST=s;
  // Populate name field on first load (don't overwrite while user is typing)
  const nameEl=document.getElementById('zone-name');
  if(s.name && nameEl !== document.activeElement && !nameEl.dataset.edited)
    nameEl.value=s.name;
  document.getElementById('v-bearing').textContent=s.bearing.toFixed(1)+'°';
  document.getElementById('v-points').textContent=s.points.length+' / 36';
  document.getElementById('btn-water').classList.toggle('on',s.water);
  document.getElementById('btn-save').disabled=(s.points.length<3);
  document.getElementById('btn-dn').classList.toggle('at-limit', !!s.at_min);
  document.getElementById('btn-up').classList.toggle('at-limit', !!s.at_max);
  const limHint = s.at_min ? ' MIN' : s.at_max ? ' MAX' : '';
  document.getElementById('v-throw').textContent = s.throw_ft.toFixed(1)+'ft'+limHint;
  document.getElementById('btn-trim').classList.toggle('water-off', !s.water);
  draw();
}

// ── API ─────────────────────────────────────────────────────────
async function doTrim(){
  if(!ST.water) return;  // guard: only trim when water is on
  const btn = document.getElementById("btn-trim");
  btn.classList.add("busy");
  btn.textContent = "⊙ trimming...";
  try{
    const r=await fetch("/zone/act?cmd=trim_pres",{method:"POST"});
    if(!r.ok) throw new Error(r.status);
    applyState(await r.json());
    setConn(true);
    toast("Trimmed ✓");
  }catch(e){ setConn(false); }
  btn.classList.remove("busy");
  btn.textContent = "⊙ trim";
}

async function doAct(cmd){
  try{
    let url='/zone/act?cmd='+cmd;
    if(cmd==='save'){
      const name=document.getElementById('zone-name').value.trim()||'Zone';
      url+='&name='+encodeURIComponent(name);
    }
    const r=await fetch(url,{method:'POST'});
    if(!r.ok) throw new Error(r.status);
    applyState(await r.json());
    setConn(true);
    if(cmd==='save'){
      toast('Zone saved ✓');
      // Zone geometry changed -- clear cached heatmap data so stale coverage
      // is not shown.  Files were deleted server-side; clear JS state too.
      lastRun=null; waterCSVRows=null; draw();
    }
    if(cmd==='add_pt') toast('Point '+ST.points.length+' added');
    if(cmd==='cancel') window.history.back();
  }catch(e){ setConn(false); }
}

// Hold-repeat for dpad.
// Valve buttons use a two-phase strategy:
//   Step 1        : full pres_up/dn (motor + 300ms PSI settle) so throw updates on a single tap.
//   Steps 2+      : pres_up/dn_move (motor only, ~250ms) with acceleration 0.5->1.0->1.5 deg.
//   After release : poll() fires after settle delay to refresh the throw display.
// Nozzle buttons: sequential, same command every repeat.
let _hTimer=null, _hBtn=null, _hActive=false, _hStep=0, _hIsValve=false;
function _holdDeg(n){ return n<=4?0.5:n<=8?1.0:1.5; }
function holdStart(cmd,el){
  holdStop();
  _hBtn=el; el.classList.add('held');
  _hActive=true; _hStep=0;
  _hIsValve=(cmd==='pres_up'||cmd==='pres_dn');
  (async function loop(){
    if(!_hActive) return;
    _hStep++;
    if(_hStep>180){ holdStop(); toast('hold auto-stopped'); return; }  // ~11s hard cap vs a stuck button
    var act;
    if(_hIsValve){
      if(_hStep===1) act=cmd+'&deg=0.5';           // full settle -- immediate throw feedback
      else           act=cmd+'_move&deg='+_holdDeg(_hStep); // motor-only -- fast repeat
    } else {
      act=cmd;
    }
    await doAct(act).catch(()=>{});
    if(_hActive) _hTimer=setTimeout(loop,60);
  })();
}
function holdStop(){
  var wasValve=_hIsValve;
  _hActive=false; _hStep=0; _hIsValve=false;
  if(_hTimer){clearTimeout(_hTimer);_hTimer=null;}
  if(_hBtn){_hBtn.classList.remove('held');_hBtn=null;}
  // After releasing a valve button: wait for the valve to settle then poll for
  // updated PSI/throw.  The move-only hold commands skip settle, so we need this.
  if(wasValve) setTimeout(function(){ fetch('/zone/state').then(r=>r.json()).then(applyState).catch(()=>{}); }, 350);
}
// Safety net: a missed pointerup/leave on mobile (pointercancel fires for
// gestures/long-press) must never leave a button stuck hammering the device
// with valve commands (the TaskWDT crash during zone setup). ANY pointer
// release/cancel or focus loss stops the hold-repeat.
['pointerup','pointercancel'].forEach(function(ev){ window.addEventListener(ev, holdStop); });
window.addEventListener('blur', holdStop);
document.addEventListener('visibilitychange', function(){ if(document.hidden) holdStop(); });

// ── Connection indicator ────────────────────────────────────────
function setConn(ok){
  const d=document.getElementById('conn-dot');
  const l=document.getElementById('conn-label');
  d.className=ok?'ok':'err';
  l.textContent=ok?'live':'offline';
}

// ── Toast ───────────────────────────────────────────────────────
function toast(msg){
  const t=document.getElementById('toast');
  t.textContent=msg; t.style.opacity='1';
  clearTimeout(t._t);
  t._t=setTimeout(()=>t.style.opacity='0',2000);
}

// ── Poll ────────────────────────────────────────────────────────
// Sequential: schedule next poll only after current fetch completes.
// ── b400: on-device point editor (drag / delete interior points) ─────
function _setEditUI(on){
  document.getElementById('edit-bar').style.display     = on?'flex':'none';
  document.getElementById('dpad-section').style.display = on?'none':'';
  document.getElementById('secondary').style.display    = on?'none':'';
  document.getElementById('footer').style.display       = on?'none':'';
  CV.style.touchAction = on?'none':'';   // let the canvas own touch drags in edit mode
}
function toggleEdit(){
  if(_editMode){ cancelEdit(); return; }
  if(!ST.points || ST.points.length<1){ toast('No points to edit'); return; }
  _editMode=true; _selPt=-1; _dragging=false; _setEditUI(true);
  toast('Edit: drag a point; tap + Delete pt to remove');
  draw();
}
function exitEdit(){ _editMode=false; _selPt=-1; _dragging=false; _setEditUI(false); }
function cancelEdit(){  // discard local edits, reload authoritative device state
  exitEdit();
  fetch('/zone/state').then(r=>r.json()).then(applyState).catch(()=>{});
}
function deleteSelPoint(){
  if(_selPt<0 || _selPt>=ST.points.length){ toast('Tap a point first'); return; }
  ST.points.splice(_selPt,1); _selPt=-1; draw();
}
function _cxy(e){
  const r=CV.getBoundingClientRect();
  return {x:(e.clientX-r.left)*(CV.width/r.width), y:(e.clientY-r.top)*(CV.height/r.height)};
}
function _hitPoint(x,y){
  let best=-1, bd=1e9; const thr=Math.max(16,_maxR*0.06);
  ST.points.forEach((p,i)=>{
    const rr=(p.throw_mm/_edScale())*_maxR, a=(p.deg-90)*Math.PI/180;
    const d=Math.hypot(_cx+Math.cos(a)*rr-x, _cy+Math.sin(a)*rr-y);
    if(d<bd){ bd=d; best=i; }
  });
  return bd<=thr ? best : -1;
}
CV.addEventListener('pointerdown',e=>{
  if(!_editMode) return;
  const {x,y}=_cxy(e); const h=_hitPoint(x,y);
  _selPt=h; _dragging=(h>=0);
  if(_dragging){ try{CV.setPointerCapture(e.pointerId);}catch(_){} }
  draw();
});
CV.addEventListener('pointermove',e=>{
  if(!_editMode||!_dragging||_selPt<0) return;
  const {x,y}=_cxy(e); const dx=x-_cx, dy=y-_cy;
  let r=Math.hypot(dx,dy)/_maxR*_edScale(); if(r>_edScale()) r=_edScale(); if(r<0) r=0;
  let deg=Math.atan2(dy,dx)*180/Math.PI+90; deg=((deg%360)+360)%360;
  ST.points[_selPt].throw_mm=r; ST.points[_selPt].deg=deg; draw();
});
function _endDrag(){ _dragging=false; }
CV.addEventListener('pointerup',_endDrag);
CV.addEventListener('pointercancel',_endDrag);
window.addEventListener('pointerup',_endDrag);   // safety: release if it ends off-canvas
async function saveEdits(){
  if(!ST.points || ST.points.length<2){ toast('Need at least 2 points'); return; }
  if(ST.points.length<3 && !confirm('Fewer than 3 points — save anyway?')) return;
  const zid=(_zoneIdParam==='new')?'new':_zoneIdParam;
  const nm=(document.getElementById('zone-name').value||ST.name||'Zone').slice(0,31);
  const body=JSON.stringify({ id:zid, name:nm, num_points:ST.points.length,
    points:ST.points.map(p=>({nozzle_deg:+(+p.deg).toFixed(2), throw_mm:+(+p.throw_mm).toFixed(1),
                              pressure:+(+(p.psi||0)).toFixed(4), walk_idx:+(p.widx||0)})) });
  const btn=document.getElementById('btn-edit-save'); if(btn) btn.disabled=true;
  try{
    const r=await fetch('/zone/import?id='+encodeURIComponent(zid)+'&recompute=1',
                        {method:'POST',body}).then(x=>x.json());
    if(r&&r.ok){ toast('Saved zone '+r.id+' ('+r.num_points+' pts)'); exitEdit();
      fetch('/zone/state').then(x=>x.json()).then(applyState).catch(()=>{}); }
    else toast('Save failed: '+((r&&r.error)||'?'));
  }catch(e){ toast('Save error'); }
  finally{ if(btn) btn.disabled=false; }
}

var _pollTimer=null;
async function poll(){
  if(_editMode){ _pollTimer=setTimeout(poll,900); return; }  // b400: don't clobber edits
  try{
    const r=await fetch('/zone/state');
    if(!r.ok) throw new Error();
    applyState(await r.json());
    setConn(true);
  }catch(e){ setConn(false); }
  _pollTimer=setTimeout(poll,900);
}

// ── Init ────────────────────────────────────────────────────────
window.addEventListener('resize',resizeCanvas);
resizeCanvas();
// Mark name field edited when user types so polling won't overwrite it
document.getElementById('zone-name').addEventListener('input',function(){
  this.dataset.edited='1';
});
poll();
</script>
</body>
</html>
)ZONEHTML"
