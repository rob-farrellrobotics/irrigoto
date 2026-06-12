/* Auto-generated from schedule.html -- edit the .html, then run regen.py */
R"SCHEDHTML(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>Schedule &mdash; irrigoto</title>
<script>
// Apply theme before body renders to avoid flash. localStorage = instant
// cache; /api/theme = source of truth (NVS-backed).
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
  --green:#00e87a;--green-dim:#0a3020;--green-glow:rgba(0,232,122,.22);
  --orange:#ff8c00;--text:#c8e0d4;--text-mid:#608070;--text-dim:#1e3828;
  --border:#0c2a1e;--btn:#0b1c28;--radius:12px;--radius-sm:8px;--red:#f87171;
}
:root[data-theme="light"]{
  --bg:#f7f9f8;--bg2:#ffffff;--bg3:#eef3f0;
  --green:#00913f;--green-dim:#cfeede;--green-glow:rgba(0,145,63,.18);
  --orange:#c95400;--text:#0e1a14;--text-mid:#5a7468;--text-dim:#cfdcd5;
  --border:#cee0d7;--btn:#f0f5f2;--red:#c43030;
}
html,body{min-height:100%;background:var(--bg);color:var(--text);
  font-family:-apple-system,BlinkMacSystemFont,'Helvetica Neue',sans-serif;}
body{display:flex;flex-direction:column;max-width:560px;margin:0 auto;padding-bottom:24px;}
header{display:flex;align-items:center;gap:12px;padding:14px 20px 10px;
  border-bottom:1px solid var(--border);}
.back{color:var(--green);text-decoration:none;font-size:20px;line-height:1;}
.hdr-title{font-family:'Courier New',monospace;font-size:16px;font-weight:700;
  letter-spacing:.1em;color:var(--green);flex:1;}
.page-logo{display:flex;justify-content:center;padding:12px 0 2px;}
.logo{font-family:'Courier New',monospace;font-size:20px;font-weight:700;
  letter-spacing:.15em;color:var(--green);text-shadow:0 0 12px var(--green-glow);}
main{padding:16px;}
.sec-hdr{display:flex;align-items:center;justify-content:space-between;
  margin:0 2px 10px;}
.sec-title{font-size:11px;letter-spacing:.18em;text-transform:uppercase;
  color:var(--text-mid);font-weight:500;}
.sec-btn{font-size:12px;color:var(--green);background:none;
  padding:4px 10px;border-radius:var(--radius-sm);cursor:pointer;
  border:1px solid var(--border);font-family:inherit;}
.sec-btn:active{background:var(--green-dim);}
.card{background:var(--bg2);border:1px solid var(--border);
  border-radius:var(--radius);padding:14px 16px;margin-bottom:10px;}
.empty{color:var(--text-mid);font-size:13px;padding:4px 0;text-align:center;}
.status-card{font-size:12px;color:var(--text-mid);margin-bottom:14px;
  background:var(--bg2);border:1px solid var(--border);
  border-radius:var(--radius);padding:10px 14px;}
.status-row{display:flex;justify-content:space-between;align-items:center;
  padding:3px 0;}
.status-row .lbl{color:var(--text-mid);}
.status-row .val{color:var(--text);font-family:'Courier New',monospace;
  font-size:12px;text-align:right;}
.status-row .val.warn{color:var(--orange);}
.status-row .val.err{color:var(--red);}

/* Entry card layout */
.entry{position:relative;}
.entry.disabled{opacity:.55;}
.entry-hdr{display:flex;align-items:center;justify-content:space-between;
  margin-bottom:10px;}
.entry-num{font-family:'Courier New',monospace;font-size:11px;color:var(--text-mid);
  letter-spacing:.1em;}
.entry-del{background:none;border:1px solid var(--border);color:var(--red);
  font-size:13px;padding:4px 10px;border-radius:var(--radius-sm);cursor:pointer;}
.entry-del:active{background:rgba(248,113,113,.1);}

.row{display:flex;gap:8px;margin-bottom:10px;flex-wrap:wrap;}
.row > .col{flex:1;min-width:0;}
.col label{display:block;font-size:10px;letter-spacing:.12em;
  text-transform:uppercase;color:var(--text-mid);margin-bottom:4px;}
select,input[type="time"]{background:var(--bg3);border:1px solid var(--border);
  border-radius:var(--radius-sm);color:var(--text);font-size:14px;
  padding:8px 10px;width:100%;font-family:inherit;}
select:focus,input[type="time"]:focus{outline:1px solid var(--green);}

/* Days picker — 7 round chips */
.days{display:flex;gap:6px;justify-content:space-between;}
.day{flex:1;text-align:center;font-size:11px;font-weight:600;
  padding:8px 0;border-radius:var(--radius-sm);border:1px solid var(--border);
  background:var(--btn);color:var(--text-mid);cursor:pointer;
  font-family:inherit;letter-spacing:.04em;}
.day.sel{background:var(--green-dim);border-color:#1a5035;color:var(--green);}
.day-presets{display:flex;gap:6px;margin-top:6px;flex-wrap:wrap;}
.day-preset{font-size:11px;color:var(--text-mid);background:none;
  border:1px solid var(--border);padding:3px 8px;border-radius:6px;
  cursor:pointer;font-family:inherit;}
.day-preset:hover{color:var(--green);}

/* Enabled toggle */
.toggle-row{display:flex;justify-content:space-between;align-items:center;
  padding:4px 0 0;}
.toggle-row .toggle-lbl{font-size:13px;color:var(--text);}
.theme-switch{position:relative;display:inline-block;width:44px;height:22px;}
.theme-switch input{opacity:0;width:0;height:0;}
.theme-slider{position:absolute;cursor:pointer;inset:0;background:var(--bg3);
  transition:.3s;border-radius:22px;border:1px solid var(--border);}
.theme-slider:before{position:absolute;content:"";height:14px;width:14px;
  left:3px;top:3px;background:var(--text-mid);transition:.3s;border-radius:50%;}
.theme-switch input:checked + .theme-slider{background:var(--green-dim);border-color:var(--green);}
.theme-switch input:checked + .theme-slider:before{transform:translateX(22px);background:var(--green);}

/* Footer save bar */
.save-bar{display:flex;gap:10px;margin-top:8px;}
.btn{padding:10px 16px;border-radius:var(--radius-sm);border:1px solid var(--border);
  background:var(--btn);color:var(--text);font-size:14px;cursor:pointer;
  font-family:inherit;flex:1;text-align:center;}
.btn-primary{background:var(--green-dim);border-color:#1a5035;color:var(--green);
  font-weight:600;}
.btn-primary:disabled{opacity:.4;cursor:not-allowed;}
.btn-danger{color:var(--red);}
#save-status{font-size:12px;color:var(--text-mid);text-align:center;
  margin-top:8px;min-height:18px;}
#save-status.ok{color:var(--green);}
#save-status.err{color:var(--red);}

/* Add-entry button */
.add-btn{display:block;width:100%;padding:14px;margin-bottom:16px;
  background:var(--bg2);border:1px dashed var(--border);
  border-radius:var(--radius);color:var(--green);font-size:14px;
  cursor:pointer;font-family:inherit;}
.add-btn:hover{border-color:var(--green);background:var(--green-dim);}
.add-btn:disabled{opacity:.4;cursor:not-allowed;}

.hint{font-size:11px;color:var(--text-mid);margin-top:4px;line-height:1.4;}
</style>
</head>
<body>
<header>
  <a class="back" href="/" title="Back">&larr;</a>
  <span class="hdr-title">SCHEDULE</span>
</header>
<div class="page-logo"><span class="logo">Irrigoto</span></div>

<main>
  <div class="status-card">
    <div class="status-row"><span class="lbl">Device time</span><span class="val" id="st-time">&mdash;</span></div>
    <div class="status-row"><span class="lbl">Next run</span><span class="val" id="st-next">&mdash;</span></div>
    <div class="status-row"><span class="lbl">Last update</span><span class="val" id="st-last">&mdash;</span></div>
  </div>

  <div class="sec-hdr">
    <span class="sec-title">Rain Delay</span>
    <span class="sec-title" id="delay-state">off</span>
  </div>
  <div class="card">
    <div style="display:grid;grid-template-columns:repeat(4,1fr);gap:6px;">
      <button class="btn" onclick="setDelay(6)">6 h</button>
      <button class="btn" onclick="setDelay(12)">12 h</button>
      <button class="btn" onclick="setDelay(24)">24 h</button>
      <button class="btn" onclick="setDelay(48)">48 h</button>
    </div>
    <div style="display:flex;gap:8px;margin-top:8px;align-items:center;">
      <input type="number" id="delay-custom" min="1" max="336" placeholder="hours"
             style="flex:1;background:var(--bg3);border:1px solid var(--border);
                    border-radius:var(--radius-sm);color:var(--text);font-size:14px;
                    padding:8px 10px;font-family:inherit;">
      <button class="btn" onclick="setDelayCustom()" style="flex:0 0 auto;">Set</button>
      <button class="btn btn-danger" id="delay-cancel-btn" onclick="setDelay(0)"
              style="flex:0 0 auto;display:none;">Cancel</button>
    </div>
    <div id="delay-status" class="hint" style="margin-top:8px;min-height:14px;"></div>
  </div>

  <div class="sec-hdr">
    <span class="sec-title">Entries</span>
    <span class="sec-title" id="ent-count">0 of 32</span>
  </div>
  <div id="entries"><div class="empty">Loading&hellip;</div></div>

  <button class="add-btn" id="add-btn" onclick="addEntry()">+ Add entry</button>

  <div class="save-bar">
    <button class="btn btn-danger" onclick="clearAll()" title="Remove all entries">Clear all</button>
    <button class="btn btn-primary" id="save-btn" onclick="saveAll()">Save</button>
  </div>
  <div id="save-status"></div>

  <div class="hint" style="margin-top:18px;">
    Schedule entries fire at the device's local time. Time comes from Home Assistant
    (no NTP). Overlapping entries on shared days are rejected at save time.
  </div>
</main>

<script>
const DAY_LABELS = ['Sun','Mon','Tue','Wed','Thu','Fri','Sat'];
const MODE_LABELS = ['Pulse','Gentle','Smooth','Serpentine'];
// Depth is eighths of an inch, indexed by the stored value (1..8); index 0 is
// an unused placeholder. Deeper targets are achieved as multiple 1/8" passes
// (same as the other modes), so Smooth is no longer 1/8"-only.
const DEPTH_LABELS = ['—','1/8&Prime;','1/4&Prime;','3/8&Prime;','1/2&Prime;','5/8&Prime;','3/4&Prime;','7/8&Prime;','1&Prime;'];

let state = {
  zones: [],         // [{id, name}]
  entries: [],       // working copy (edited locally)
  maxEntries: 32,
  fwBuild: 0,
  lastStatus: '',
  nextRun: null,
  deviceTime: 0,
  tzOffsetMin: 0,
  delayUntil: 0,     // epoch; 0 = no delay
};

// ── Initial load ─────────────────────────────────────────────────────────────
async function init() {
  try {
    const r = await fetch('/api/schedule', {cache:'no-store'});
    if (!r.ok) throw new Error('HTTP ' + r.status);
    const d = await r.json();
    state.zones = d.zones || [];
    state.entries = (d.entries || []).map(normalizeEntry);
    state.maxEntries = d.max_entries || 32;
    state.fwBuild = d.fw_build || 0;
    state.lastStatus = d.last_status || '';
    state.nextRun = d.next_run || null;
    state.deviceTime = d.now || 0;
    state.tzOffsetMin = (typeof d.tz_offset_min === 'number') ? d.tz_offset_min : 0;
    state.delayUntil = d.delay_until || 0;
    renderAll();
  } catch (e) {
    document.getElementById('entries').innerHTML =
      '<div class="empty" style="color:var(--red)">Load failed: ' + (e.message||e) + '</div>';
  }
}

function normalizeEntry(e) {
  return {
    zone: +e.zone || 1,
    mode: +e.mode || 0,
    depth: +e.depth || 1,   // eighths (1..8); 0/missing -> 1/8"
    hour: +e.hour || 0,
    minute: +e.minute || 0,
    days_mask: +e.days_mask || 0,
    enabled: e.enabled ? 1 : 0,
  };
}

// ── Render ───────────────────────────────────────────────────────────────────
function renderAll() {
  renderStatus();
  renderDelay();
  renderEntries();
  document.getElementById('ent-count').textContent =
    state.entries.length + ' of ' + state.maxEntries;
  document.getElementById('add-btn').disabled =
    state.entries.length >= state.maxEntries;
}

function renderDelay() {
  const lbl  = document.getElementById('delay-state');
  const btn  = document.getElementById('delay-cancel-btn');
  const note = document.getElementById('delay-status');
  if (state.delayUntil > 1700000000 && state.delayUntil > (state.deviceTime || 0)) {
    const until = formatLocal(state.delayUntil);
    lbl.textContent = 'active until ' + until;
    lbl.style.color = 'var(--orange)';
    btn.style.display = '';
    note.textContent = 'Schedule entries that fall within this window will be skipped.';
  } else {
    lbl.textContent = 'off';
    lbl.style.color = '';
    btn.style.display = 'none';
    note.textContent = '';
  }
}

function renderStatus() {
  const t = document.getElementById('st-time');
  const n = document.getElementById('st-next');
  const l = document.getElementById('st-last');
  if (state.deviceTime > 1700000000) {
    t.textContent = formatLocal(state.deviceTime);
    t.classList.remove('warn');
  } else {
    t.textContent = 'not synced';
    t.classList.add('warn');
  }
  if (state.nextRun && state.nextRun.epoch > 1700000000) {
    const zname = lookupZoneName(state.nextRun.zone);
    n.textContent = zname + ' → ' + formatLocal(state.nextRun.epoch);
  } else if (state.entries.length === 0) {
    n.textContent = 'no entries';
  } else if (state.deviceTime <= 1700000000) {
    n.textContent = 'waiting for time';
  } else {
    n.textContent = 'no upcoming runs';
  }
  l.textContent = state.lastStatus || '—';
  if (state.lastStatus.indexOf('rejected') === 0 ||
      state.lastStatus.indexOf('parse failed') === 0 ||
      state.lastStatus.indexOf('out-of-range') === 0) {
    l.classList.add('err'); l.classList.remove('warn');
  } else if (state.lastStatus.indexOf('ok') === 0) {
    l.classList.remove('err'); l.classList.remove('warn');
  } else {
    l.classList.remove('err'); l.classList.add('warn');
  }
}

function lookupZoneName(zoneOneBased) {
  // schedule entries use 1-based zone numbers; zones[] uses 0-based ids.
  // Fallback label shows the 0-based "#id" (display convention everywhere).
  const z = state.zones.find(z => z.id === (zoneOneBased - 1));
  return z ? z.name : ('Zone #' + (zoneOneBased - 1));
}

function formatLocal(epochSec) {
  // Device epoch is already local-wall-clock if we trust the device clock.
  // For display, render it using device's offset so it always matches what
  // the user typed (independent of the browser's TZ).
  const d = new Date(epochSec * 1000);
  // Use UTC pieces, then apply tzOffsetMin to convert to device-local.
  const t = epochSec + state.tzOffsetMin * 60;
  const u = new Date(t * 1000);
  const Y = u.getUTCFullYear();
  const M = String(u.getUTCMonth()+1).padStart(2,'0');
  const D = String(u.getUTCDate()).padStart(2,'0');
  const h = String(u.getUTCHours()).padStart(2,'0');
  const m = String(u.getUTCMinutes()).padStart(2,'0');
  return Y + '-' + M + '-' + D + ' ' + h + ':' + m;
}

function renderEntries() {
  const el = document.getElementById('entries');
  if (!state.entries.length) {
    el.innerHTML = '<div class="card empty">No entries yet. Tap "Add entry" below to schedule a watering run.</div>';
    return;
  }
  el.innerHTML = '';
  state.entries.forEach((e, idx) => el.appendChild(renderEntry(e, idx)));
}

// Compute the next fire instant for an entry given the current device
// time. Mirrors the firmware's next_run walker so per-entry previews
// match what HA's Next Scheduled Run will say after Save. Returns a
// short human-readable label ("Today 17:30", "Sun 5/17 17:30",
// "Sat 5/23 17:30") and the unix epoch.
function entryNextFire(e, deviceEpoch, tzOffMin) {
  if (!e.enabled || !e.days_mask) {
    return { label: 'never (disabled)', epoch: 0, today: false };
  }
  if (!deviceEpoch || deviceEpoch < 1700000000) {
    return { label: 'waiting for time sync', epoch: 0, today: false };
  }
  // Compute device-local wall-clock fields from epoch + tz offset.
  // We work in a "local-shifted UTC" — epoch + tzOff*60, then read
  // UTC pieces. That gives the same calendar fields the device sees.
  function localFields(epoch) {
    const u = new Date((epoch + tzOffMin*60) * 1000);
    return { y: u.getUTCFullYear(), mo: u.getUTCMonth()+1, d: u.getUTCDate(),
             h: u.getUTCHours(), mi: u.getUTCMinutes(), wday: u.getUTCDay() };
  }
  // Walk forward up to 14 days, matching the firmware.
  for (let off = 0; off < 15; off++) {
    const cand = deviceEpoch + off * 86400;
    const f = localFields(cand);
    // Build the candidate fire moment: same Y-M-D as cand (in device
    // local), with the entry's H:M:00. Inverse of localFields:
    //   epoch = Date.UTC(...) / 1000 - tzOff*60
    const fireEpoch = Math.floor(Date.UTC(f.y, f.mo-1, f.d, e.hour, e.minute, 0) / 1000) - tzOffMin*60;
    if (fireEpoch <= deviceEpoch) continue;       // past for today
    const fireFields = localFields(fireEpoch);
    if (!(e.days_mask & (1 << fireFields.wday))) continue;
    // Format label.
    const today    = localFields(deviceEpoch);
    const isToday  = (fireFields.y === today.y && fireFields.mo === today.mo && fireFields.d === today.d);
    const tomorrow = localFields(deviceEpoch + 86400);
    const isTomorrow = (fireFields.y === tomorrow.y && fireFields.mo === tomorrow.mo && fireFields.d === tomorrow.d);
    const t = String(fireFields.h).padStart(2,'0') + ':' + String(fireFields.mi).padStart(2,'0');
    const wdayLbl = ['Sun','Mon','Tue','Wed','Thu','Fri','Sat'][fireFields.wday];
    let label;
    if (isToday)         label = 'Today ' + t;
    else if (isTomorrow) label = 'Tomorrow ' + t;
    else                 label = wdayLbl + ' ' + fireFields.mo + '/' + fireFields.d + ' ' + t;
    return { label, epoch: fireEpoch, today: isToday };
  }
  return { label: 'no upcoming fire', epoch: 0, today: false };
}

function renderEntry(e, idx) {
  const card = document.createElement('div');
  card.className = 'card entry' + (e.enabled ? '' : ' disabled');

  // Build the zone <option> list, anchored on whatever zones the API
  // returned plus the entry's current zone number (in case the user
  // deleted that zone after creating this entry).
  const zoneOpts = buildZoneOptions(e.zone);
  const timeStr = String(e.hour).padStart(2,'0') + ':' + String(e.minute).padStart(2,'0');
  // Per-entry next fire preview (computed client-side; matches the
  // firmware's next_run walker so what you see is what HA will see).
  const nf = entryNextFire(e, state.deviceTime, state.tzOffsetMin);
  const nfCol = nf.today ? 'var(--green)'
              : nf.epoch ? 'var(--text-mid)'
              : 'var(--orange)';

  card.innerHTML =
    '<div class="entry-hdr">' +
      '<span class="entry-num">ENTRY #' + (idx+1) + '</span>' +
      '<span class="next-preview" style="font-size:11px;color:' + nfCol + ';font-family:monospace;">' +
        '&rarr; ' + nf.label +
      '</span>' +
      '<button class="entry-del" onclick="delEntry(' + idx + ')">Delete</button>' +
    '</div>' +
    '<div class="row">' +
      '<div class="col" style="flex:2"><label>Zone</label>' +
        '<select data-k="zone">' + zoneOpts + '</select></div>' +
      '<div class="col"><label>Time</label>' +
        '<input type="time" data-k="time" value="' + timeStr + '"></div>' +
    '</div>' +
    '<div class="row">' +
      '<div class="col"><label>Mode</label>' +
        '<select data-k="mode">' +
          '<option value="0"' + (e.mode===0?' selected':'') + '>Pulse</option>' +
          '<option value="1"' + (e.mode===1?' selected':'') + '>Gentle</option>' +
          '<option value="2"' + (e.mode===2?' selected':'') + '>Smooth</option>' +
          '<option value="3"' + (e.mode===3?' selected':'') + '>Serpentine</option>' +
        '</select></div>' +
      '<div class="col"><label>Depth</label>' +
        '<select data-k="depth">' +
          [1,2,3,4,5,6,7,8].map(v =>
            '<option value="' + v + '"' + (((+e.depth)||1)===v ? ' selected' : '') + '>' +
            DEPTH_LABELS[v] + '</option>'
          ).join('') +
        '</select></div>' +
    '</div>' +
    '<div><label style="font-size:10px;letter-spacing:.12em;text-transform:uppercase;color:var(--text-mid);display:block;margin-bottom:4px;">Days</label>' +
      '<div class="days" data-k="days">' +
        DAY_LABELS.map((lbl, i) =>
          '<button class="day' + (e.days_mask & (1<<i) ? ' sel' : '') + '" ' +
          'data-bit="' + i + '">' + lbl + '</button>'
        ).join('') +
      '</div>' +
      '<div class="day-presets">' +
        '<button class="day-preset" data-preset="127">Every day</button>' +
        '<button class="day-preset" data-preset="62">Weekdays</button>' +
        '<button class="day-preset" data-preset="65">Weekends</button>' +
        '<button class="day-preset" data-preset="42">MWF</button>' +
        '<button class="day-preset" data-preset="20">Tu/Th</button>' +
        '<button class="day-preset" data-preset="0">None</button>' +
      '</div>' +
    '</div>' +
    '<div class="toggle-row" style="margin-top:12px;">' +
      '<span class="toggle-lbl">Enabled</span>' +
      '<label class="theme-switch">' +
        '<input type="checkbox" data-k="enabled"' + (e.enabled ? ' checked' : '') + '>' +
        '<span class="theme-slider"></span>' +
      '</label>' +
    '</div>';

  // Wire up change handlers — keep state.entries in sync immediately so
  // saveAll() always sends the visible state.
  card.querySelector('[data-k="zone"]').addEventListener('change', ev => {
    state.entries[idx].zone = +ev.target.value || 1;
  });
  card.querySelector('[data-k="time"]').addEventListener('change', ev => {
    const [h,m] = ev.target.value.split(':').map(Number);
    state.entries[idx].hour = h|0;
    state.entries[idx].minute = m|0;
    updateEntryPreview(card, state.entries[idx]);
  });
  card.querySelector('[data-k="mode"]').addEventListener('change', ev => {
    state.entries[idx].mode = +ev.target.value;
    // Re-render this card so the depth-disabled state updates.
    rerenderEntry(idx);
  });
  card.querySelector('[data-k="depth"]').addEventListener('change', ev => {
    state.entries[idx].depth = +ev.target.value;
  });
  card.querySelector('[data-k="enabled"]').addEventListener('change', ev => {
    state.entries[idx].enabled = ev.target.checked ? 1 : 0;
    card.classList.toggle('disabled', !ev.target.checked);
    updateEntryPreview(card, state.entries[idx]);
  });
  card.querySelectorAll('.day').forEach(btn => {
    btn.addEventListener('click', () => {
      const bit = +btn.dataset.bit;
      state.entries[idx].days_mask ^= (1 << bit);
      btn.classList.toggle('sel');
      updateEntryPreview(card, state.entries[idx]);
    });
  });
  card.querySelectorAll('.day-preset').forEach(btn => {
    btn.addEventListener('click', () => {
      const mask = +btn.dataset.preset;
      state.entries[idx].days_mask = mask;
      card.querySelectorAll('.day').forEach(d => {
        const bit = +d.dataset.bit;
        d.classList.toggle('sel', !!(mask & (1<<bit)));
      });
      updateEntryPreview(card, state.entries[idx]);
    });
  });

  return card;
}

function rerenderEntry(idx) {
  const el = document.getElementById('entries');
  const old = el.children[idx];
  const fresh = renderEntry(state.entries[idx], idx);
  if (old) el.replaceChild(fresh, old);
}

// Cheap in-place refresh of just the "→ next: ..." chip on an entry
// card. Avoids the full rerenderEntry flash when the user toggles a
// day chip or scrubs the time picker.
function updateEntryPreview(card, e) {
  const chip = card.querySelector('.next-preview');
  if (!chip) return;
  const nf = entryNextFire(e, state.deviceTime, state.tzOffsetMin);
  chip.textContent = '→ ' + nf.label;
  chip.style.color = nf.today ? 'var(--green)'
                  : nf.epoch ? 'var(--text-mid)'
                  : 'var(--orange)';
}

function buildZoneOptions(currentZone) {
  // Build option list anchored on known zones; also include the current
  // zone number even if it's no longer in the device's zone list, so we
  // don't silently mutate the user's value just by rendering.
  const seen = new Set();
  let opts = '';
  state.zones.forEach(z => {
    seen.add(z.id + 1);
    const sel = (z.id + 1 === currentZone) ? ' selected' : '';
    // value = 1-based wire zone number (functional); label "(#id)" = 0-based
    // storage id (display convention shared with HA's schedule tab).
    opts += '<option value="' + (z.id+1) + '"' + sel + '>' +
            escapeHtml(z.name) + ' (#' + z.id + ')</option>';
  });
  if (!seen.has(currentZone)) {
    opts = '<option value="' + currentZone + '" selected>Zone #' + (currentZone-1) +
           ' (missing)</option>' + opts;
  }
  return opts;
}

function escapeHtml(s) {
  return String(s).replace(/[&<>"']/g, c =>
    ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
}

// ── Entry add / delete ───────────────────────────────────────────────────────
function addEntry() {
  if (state.entries.length >= state.maxEntries) return;
  // Sensible defaults: first available zone, Smooth 1/8", 6:00 AM, every day.
  const firstZone = state.zones.length ? state.zones[0].id + 1 : 1;
  state.entries.push({
    zone: firstZone, mode: 2, depth: 1,
    hour: 6, minute: 0,
    days_mask: 127, enabled: 1,
  });
  renderAll();
  // Scroll the new entry into view.
  const el = document.getElementById('entries');
  const last = el.lastChild;
  if (last && last.scrollIntoView) last.scrollIntoView({behavior:'smooth', block:'center'});
}

function delEntry(idx) {
  state.entries.splice(idx, 1);
  renderAll();
}

// ── Save / clear ─────────────────────────────────────────────────────────────
function entriesToText(entries) {
  return entries.map(e =>
    [e.zone, e.mode, e.depth, e.hour, e.minute, e.days_mask, e.enabled].join(',')
  ).join(';');
}

async function saveAll() {
  const btn = document.getElementById('save-btn');
  const st = document.getElementById('save-status');
  btn.disabled = true; btn.textContent = 'Saving…';
  st.className = ''; st.textContent = '';
  try {
    const text = entriesToText(state.entries);
    const r = await fetch('/api/schedule', {
      method:'POST',
      headers:{'Content-Type':'application/x-www-form-urlencoded'},
      body: 'text=' + encodeURIComponent(text),
    });
    const d = await r.json();
    state.lastStatus = d.status || (d.ok ? 'ok' : 'failed');
    state.nextRun = d.next_run || null;
    state.deviceTime = d.now || state.deviceTime;
    renderStatus();
    if (d.ok) {
      st.className = 'ok';
      st.textContent = 'Saved ✓ ' + state.lastStatus;
    } else {
      st.className = 'err';
      st.textContent = state.lastStatus;
    }
  } catch (e) {
    st.className = 'err';
    st.textContent = 'Save failed: ' + (e.message||e);
  } finally {
    btn.disabled = false; btn.textContent = 'Save';
  }
}

async function setDelay(hours) {
  const st = document.getElementById('save-status');
  st.className = ''; st.textContent = '';
  try {
    const r = await fetch('/api/schedule/delay', {
      method: 'POST',
      headers: {'Content-Type': 'application/x-www-form-urlencoded'},
      body: 'hours=' + (hours | 0),
    });
    const d = await r.json();
    if (!d.ok) throw new Error(d.error || 'delay failed');
    state.delayUntil = d.delay_until || 0;
    // The server's next_run may have shifted past the delay window —
    // re-fetch to keep the page consistent.
    await init();
    st.className = 'ok';
    st.textContent = hours > 0
      ? ('Rain delay set for ' + hours + ' h.')
      : 'Rain delay cancelled.';
  } catch (e) {
    st.className = 'err';
    st.textContent = 'Delay failed: ' + (e.message || e);
  }
}

function setDelayCustom() {
  const inp = document.getElementById('delay-custom');
  const h = parseInt(inp.value, 10);
  if (!h || h < 1) return;
  setDelay(Math.min(h, 336));   // server clamps to 14 d but match here too
  inp.value = '';
}

async function clearAll() {
  if (!state.entries.length) return;
  if (!confirm('Remove all ' + state.entries.length + ' schedule entries?')) return;
  const btn = document.getElementById('save-btn');
  const st = document.getElementById('save-status');
  st.className = ''; st.textContent = '';
  try {
    const r = await fetch('/api/schedule/clear', {method:'POST'});
    const d = await r.json();
    if (!d.ok) throw new Error(d.error || 'clear failed');
    state.entries = [];
    state.lastStatus = 'cleared';
    state.nextRun = null;
    renderAll();
    st.className = 'ok';
    st.textContent = 'Schedule cleared.';
  } catch (e) {
    st.className = 'err';
    st.textContent = 'Clear failed: ' + (e.message||e);
  }
}

init();
</script>
</body>
</html>
)SCHEDHTML"
