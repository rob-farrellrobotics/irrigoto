/* fs_html.h -- OtO filesystem browser page */
R"FSHTML(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>Filesystem &mdash; irrigoto</title>
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
  --green:#00e87a;--green-dim:#0a3020;
  --text:#c8e0d4;--text-mid:#608070;--text-dim:#1e3828;
  --border:#0c2a1e;--btn:#0b1c28;--radius:12px;--radius-sm:8px;
  --red:#f87171;--orange:#ff8c00;
}
:root[data-theme="light"]{
  --bg:#f7f9f8;--bg2:#ffffff;--bg3:#eef3f0;
  --green:#00913f;--green-dim:#cfeede;
  --text:#0e1a14;--text-mid:#5a7468;--text-dim:#cfdcd5;
  --border:#cee0d7;--btn:#f0f5f2;
  --red:#c43030;--orange:#c95400;
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
  letter-spacing:.15em;color:var(--green);
  text-shadow:0 0 12px rgba(0,232,122,.22);}
.usage{font-size:11px;color:var(--text-mid);font-family:'Courier New',monospace;}
main{padding:16px;}
.upload-card{background:var(--bg2);border:1px solid var(--border);
  border-radius:var(--radius);padding:14px 16px;margin-bottom:16px;}
.upload-title{font-size:11px;letter-spacing:.18em;text-transform:uppercase;
  color:var(--text-mid);font-weight:500;margin-bottom:10px;}
.upload-row{display:flex;gap:8px;align-items:center;}
.file-inp{flex:1;background:var(--bg3);border:1px solid var(--border);
  border-radius:var(--radius-sm);color:var(--text-mid);font-size:12px;
  padding:8px 10px;cursor:pointer;}
.file-inp::file-selector-button{background:var(--btn);border:1px solid var(--border);
  border-radius:6px;color:var(--text);font-size:12px;padding:4px 10px;
  cursor:pointer;margin-right:8px;}
.path-inp{flex:1;background:var(--bg3);border:1px solid var(--border);
  border-radius:var(--radius-sm);color:var(--text);font-size:12px;
  padding:8px 10px;font-family:'Courier New',monospace;}
.path-inp:focus{outline:1px solid var(--green);}
.btn{padding:8px 14px;border-radius:var(--radius-sm);border:1px solid var(--border);
  background:var(--btn);color:var(--text);font-size:13px;cursor:pointer;
  display:inline-flex;align-items:center;gap:5px;white-space:nowrap;text-decoration:none;}
.btn-green{background:var(--green-dim);border-color:#1a5035;color:var(--green);}
.btn:active{opacity:.7;}
.upload-status{font-size:12px;color:var(--text-mid);margin-top:8px;min-height:16px;}
/* Directory tree */
.tree{font-family:'Courier New',monospace;font-size:12px;}
.dir-row{display:flex;align-items:center;gap:0;padding:2px 0;
  border-bottom:1px solid rgba(12,42,30,.4);}
.dir-row:last-child{border-bottom:none;}
.indent{display:inline-block;width:16px;flex-shrink:0;}
.dir-icon{color:var(--orange);font-size:11px;margin-right:4px;flex-shrink:0;}
.file-icon{color:var(--text-dim);font-size:11px;margin-right:4px;flex-shrink:0;}
.dir-name{color:var(--orange);font-weight:600;}
.file-name{color:var(--text);flex:1;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;}
.file-size{color:var(--text-mid);font-size:10px;margin-left:8px;flex-shrink:0;}
.file-actions{display:flex;gap:4px;margin-left:8px;flex-shrink:0;}
.act-btn{padding:3px 8px;border-radius:4px;border:1px solid var(--border);
  background:var(--bg3);color:var(--text-mid);font-size:10px;cursor:pointer;
  text-decoration:none;}
.act-btn:hover{border-color:var(--green);color:var(--green);}
.act-btn.del:hover{border-color:var(--red);color:var(--red);}
.empty{color:var(--text-dim);font-size:12px;padding:8px 0;}
.card{background:var(--bg2);border:1px solid var(--border);
  border-radius:var(--radius);padding:14px 16px;margin-bottom:12px;}
.sec-title{font-size:11px;letter-spacing:.18em;text-transform:uppercase;
  color:var(--text-mid);font-weight:500;margin-bottom:10px;}
#refresh-btn{font-size:12px;color:var(--green);background:none;border:none;cursor:pointer;
  padding:2px 8px;border-radius:4px;border:1px solid var(--border);}
.sec-hdr{display:flex;align-items:center;justify-content:space-between;margin-bottom:10px;}
</style>
</head>
<body>
<div class="page-logo"><span class="logo">Irrigoto</span></div>
<header>
  <a class="back" href="/">&#8592;</a>
  <span class="hdr-title">Filesystem</span>
  <span class="usage" id="usage">&mdash;</span>
</header>
<main>

  <!-- Upload -->
  <div class="upload-card">
    <div class="upload-title">Upload File</div>
    <div class="upload-row" style="margin-bottom:8px">
      <input class="file-inp" type="file" id="upload-file">
    </div>
    <div class="upload-row">
      <span style="font-size:12px;color:var(--text-mid);width:44px">Path:</span>
      <input class="path-inp" id="upload-path" placeholder="/lfs/zones/zone_000.bin" value="/lfs/">
      <button class="btn btn-green" onclick="uploadFile()">&#8593; Upload</button>
    </div>
    <div class="upload-status" id="upload-status"></div>
  </div>

  <!-- Directory tree -->
  <div class="card">
    <div class="sec-hdr">
      <span class="sec-title">Files</span>
      <button id="refresh-btn" onclick="loadTree()">&#8635; Refresh</button>
    </div>
    <div id="tree" class="tree"><div class="empty">Loading&hellip;</div></div>
  </div>

</main>
<script>
function fmtSize(b){
  if(b<1024) return b+'B';
  if(b<1048576) return (b/1024).toFixed(1)+'KB';
  return (b/1048576).toFixed(2)+'MB';
}
// .wbin files are binary water logs served as CSV -- show and download as .csv
function displayName(n){ return n.endsWith('.wbin')?n.slice(0,-5)+'.csv':n; }

function sumEntries(entries){
  if(!entries) return 0;
  return entries.reduce((s,e)=>s+(e.type==='file'?e.size:sumEntries(e.children)),0);
}

async function loadTree(){
  document.getElementById('tree').innerHTML='<div class="empty">Loading&hellip;</div>';
  try{
    const d=await fetch('/fs/list',{cache:'no-store'}).then(r=>r.json());
    const dataBytes=sumEntries(d.entries);
    document.getElementById('usage').textContent=
      fmtSize(dataBytes)+' files · '+fmtSize(d.used)+' / '+fmtSize(d.total)+' blocks';
    renderTree(d.entries, document.getElementById('tree'), 0);
  }catch(e){
    document.getElementById('tree').innerHTML='<div class="empty">Could not load &mdash; is storage mounted?</div>';
  }
}

function renderTree(entries, container, depth){
  if(depth===0) container.innerHTML='';
  if(!entries||!entries.length){
    if(depth===0) container.innerHTML='<div class="empty">Empty</div>';
    return;
  }
  entries.forEach(e=>{
    const row=document.createElement('div');
    row.className='dir-row';
    const indent='<span class="indent" style="width:'+(depth*16)+'px"></span>';
    if(e.type==='dir'){
      row.innerHTML=indent+
        '<span class="dir-icon">&#128193;</span>'+
        '<span class="dir-name">'+e.name+'/</span>';
      container.appendChild(row);
      if(e.children&&e.children.length)
        renderTree(e.children, container, depth+1);
    } else {
      row.innerHTML=indent+
        '<span class="file-icon">&#128196;</span>'+
        '<span class="file-name">'+displayName(e.name)+'</span>'+
        '<span class="file-size">'+fmtSize(e.size)+'</span>'+
        '<span class="file-actions">'+
          '<a class="act-btn" href="/fs/download?path='+encodeURIComponent(e.path)+'" download="'+displayName(e.name)+'">&#8595; dl</a>'+
          '<button class="act-btn del" onclick="deleteFile(\''+e.path+'\')">del</button>'+
        '</span>';
      container.appendChild(row);
    }
  });
}

// Pre-fill upload path when file selected
document.getElementById('upload-file').addEventListener('change', function(){
  if(this.files[0]){
    const cur=document.getElementById('upload-path').value;
    const dir=cur.endsWith('/')?cur:cur.replace(/\/[^/]*$/,'/');
    document.getElementById('upload-path').value=dir+this.files[0].name;
  }
});

async function uploadFile(){
  const fileInp=document.getElementById('upload-file');
  const path=document.getElementById('upload-path').value.trim();
  const st=document.getElementById('upload-status');
  if(!fileInp.files[0]){st.textContent='Select a file first.';return;}
  if(!path.startsWith('/lfs/')){st.textContent='Path must start with /lfs/';return;}
  st.style.color='var(--text-mid)'; st.textContent='Uploading\u2026';
  const fd=new FormData();
  fd.append('path',path);
  fd.append('file',fileInp.files[0]);
  try{
    const r=await fetch('/fs/upload',{method:'POST',body:fd});
    const d=await r.json();
    if(d.ok){
      st.style.color='var(--green)';
      st.textContent='Uploaded '+fmtSize(d.size)+' to '+path;
      loadTree();
    } else {
      st.style.color='var(--red)'; st.textContent=d.error||'Upload failed.';
    }
  }catch(e){ st.style.color='var(--red)'; st.textContent='Error: '+e.message; }
}

async function deleteFile(path){
  if(!confirm('Delete '+path+'?')) return;
  try{
    const r=await fetch('/fs/delete',{method:'POST',
      body:'path='+encodeURIComponent(path),
      headers:{'Content-Type':'application/x-www-form-urlencoded'}});
    const text=await r.text();
    let d; try{d=JSON.parse(text);}catch(e){alert('Server error: '+text.substring(0,120));return;}
    if(d.ok) loadTree();
    else alert(d.error||'Delete failed.');
  }catch(e){ alert('Error: '+e.message); }
}

loadTree();
</script>
</body>
</html>
)FSHTML"
