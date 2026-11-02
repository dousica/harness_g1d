"""标定页面的 HTML（独立页面，风格与主控制台一致）。"""

PAGE = r"""<!doctype html><html lang="zh"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>标定 · G1-D</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
:root{--c1:#6366F1;--c2:#818CF8;--c3:#E0E7FF;--c4:#F8FAFC;--c5:#1E293B;
 --ink:#1E293B;--slate:#64748B;--line:#E2E8F0;--ok:#059669;--warn:#D97706;--err:#DC2626;
 --mono:ui-monospace,"SF Mono","JetBrains Mono",Menlo,monospace}
body{background:var(--c4);color:var(--ink);font:15px/1.6 system-ui,-apple-system,"PingFang SC",sans-serif}
header{display:flex;align-items:center;gap:12px;padding:13px 24px;
 background:linear-gradient(100deg,#6366F1,#818CF8);color:#fff;box-shadow:0 6px 22px rgba(99,102,241,.28)}
header a{color:#fff;text-decoration:none;opacity:.85;font-size:14px}
header a:hover{opacity:1}
h1{font-size:16px;font-weight:700}
main{display:grid;grid-template-columns:minmax(0,1fr) 420px;gap:18px;padding:18px 24px 30px;align-items:start}
@media(max-width:1150px){main{grid-template-columns:1fr}}
.card{background:#fff;border:1px solid var(--line);border-radius:14px;padding:16px;
 box-shadow:0 2px 6px rgba(30,41,59,.05),0 12px 28px rgba(99,102,241,.08)}
.card h2{font-size:12px;letter-spacing:.12em;text-transform:uppercase;color:var(--c1);
 margin-bottom:12px;display:flex;align-items:center;gap:9px;font-weight:800}
.card h2::before{content:"";width:4px;height:14px;border-radius:2px;background:linear-gradient(180deg,var(--c1),var(--c2))}
.card h2::after{content:"";flex:1;height:1px;background:linear-gradient(90deg,var(--c3),transparent)}
.tabs{display:flex;gap:6px;flex-wrap:wrap;margin-bottom:12px}
.tab{padding:7px 14px;border-radius:9px;border:1px solid var(--line);background:#fff;
 cursor:pointer;font-size:14px;font-weight:500}
.tab:hover{background:var(--c3);border-color:var(--c2)}
.tab.on{background:linear-gradient(135deg,var(--c1),var(--c2));color:#fff;border-color:transparent}
.view{position:relative;background:#0F172A;border-radius:11px;overflow:hidden;aspect-ratio:16/9}
.view img{width:100%;height:100%;object-fit:contain;display:block}
.bar{height:8px;border-radius:5px;background:#EEF2FF;overflow:hidden;margin-top:5px}
.bar i{display:block;height:100%;background:linear-gradient(90deg,var(--c1),var(--c2));border-radius:5px}
.metric{margin-bottom:11px;font-size:13px}
.metric .row{display:flex;justify-content:space-between}
.metric b{font-family:var(--mono);font-weight:700}
button{appearance:none;border:1px solid var(--line);background:#fff;color:var(--ink);
 padding:9px 16px;border-radius:9px;font-size:14px;font-weight:500;cursor:pointer;transition:.15s}
button:hover{background:var(--c3);border-color:var(--c2);color:#4338CA}
button.primary{background:linear-gradient(135deg,#6366F1,#818CF8);border-color:transparent;color:#fff;
 box-shadow:0 6px 18px rgba(99,102,241,.35)}
button.primary:hover{background:linear-gradient(135deg,#4F46E5,#6366F1);color:#fff}
button:disabled{opacity:.45;cursor:not-allowed}
.row{display:flex;gap:8px;flex-wrap:wrap;align-items:center}
.hint{color:#94A3B8;font-size:12.5px;line-height:1.7;margin-top:10px}
pre{background:#F8FAFC;border:1px solid var(--line);border-radius:9px;padding:10px;
 font-family:var(--mono);font-size:12.5px;max-height:220px;overflow:auto;white-space:pre-wrap}
.pill{display:inline-flex;align-items:center;gap:6px;padding:4px 11px;border-radius:999px;
 font-size:12.5px;background:#ECFDF5;color:var(--ok);border:1px solid #A7F3D0}
.pill.bad{background:#FEF2F2;color:var(--err);border-color:#FECACA}
.pill.warn{background:#FFFBEB;color:var(--warn);border-color:#FCD34D}
</style></head><body>
<header>
  <h1>视觉标定</h1>
  <span id="conn" style="padding:3px 10px;border-radius:999px;background:rgba(255,255,255,.2);font-size:13px">连接中…</span>
  <span id="camInfo" style="opacity:.9;font-size:14px"></span>
  <span style="flex:1"></span>
  <a href="/">← 返回控制台</a>
</header>
<main>
  <section class="card">
    <h2>相机</h2>
    <div class="tabs" id="tabs"></div>
    <div class="view"><img id="stream" src=""></div>
    <div class="row" style="margin-top:12px">
      <button class="primary" onclick="capture()">采集这一张</button>
      <button id="autoBtn" onclick="toggleAuto()">自动采集：关</button>
      <button onclick="calibrate()">一键标定</button>
      <button onclick="resetAll()">清空样本</button>
      <button onclick="depthTest()">深度测试</button>
      <span id="status" class="pill warn">未采集</span>
    </div>
    <div class="hint">
      把标定板放在画面中央，缓慢转动倾斜（15°~75°）。四项指标全绿才允许采集；
      重复位姿会被自动拒绝。建议 20 张以上，覆盖画面四角。
    </div>
  </section>

  <section>
    <div class="card">
      <h2>实时质量</h2>
      <div id="metrics"></div>
    </div>
    <div class="card" style="margin-top:16px">
      <h2>标定结果</h2>
      <pre id="result">还没有标定</pre>
    </div>
  </section>
</main>
<script>
let CAM="head_left", METRIC=null;
const CAMS=[["head_left","头部左目"],["head_right","头部右目"],
            ["stereo","双目（一次出三份）"],["lwrist","左腕"],["rwrist","右腕"],["handeye","手眼标定"]];
function buildTabs(){
  document.getElementById("tabs").innerHTML=CAMS.map(([k,n])=>
    `<div class="tab${k===CAM?' on':''}" onclick="sel('${k}')">${n}</div>`).join('');
}
const STREAM_CAM={handeye:"head_left"};
function sel(k){CAM=k;AUTO=false;stopAutoLoop();buildTabs();
 document.getElementById("autoBtn").textContent="自动采集：关";
 document.getElementById("autoBtn").style.background="#fff";
 document.getElementById("autoBtn").style.color="#1E293B";
 document.getElementById("stream").src="/calib/stream?cam="+(STREAM_CAM[k]||k)+"&v="+Math.random();refresh();}
function bar(name,val,ok,text){
  return `<div class="metric"><div class="row"><span>${name}</span><b>${text}</b></div>
   <div class="bar"><i style="width:${Math.max(0,Math.min(100,val*100))}%;${ok?'':'background:#DC2626'}"></i></div></div>`;
}
async function refresh(){
  if(CAM==="handeye"){
    try{
      const r=await fetch("/calib/he/state"); const st=await r.json();
      document.getElementById("metrics").innerHTML=
        `<div class="metric"><div class="row"><span>已记录位姿</span><b>${st.samples}</b></div></div>`+
        `<div class="hint">把板子装在夹爪上，让头部相机看得见；每换一个姿态点一次「采集这一张」。\n需要 12 个以上、绕两个不平行的轴转动。</div>`;
    }catch(e){}
    return;
  }
  try{
    const r=await fetch("/calib/score?cam="+CAM); const m=await r.json();
    METRIC=m;
    const ok=m.ok;
    document.getElementById("metrics").innerHTML=
      bar("角点数", m.corner_ratio, m.corner_count>0 && m.corner_ratio>=0.55, `${m.corner_count} 点`) +
      bar("覆盖面积", m.area_ratio/0.4, m.area_ratio>=0.10, `${(m.area_ratio*100).toFixed(0)}%`) +
      bar("清晰度", Math.min(1,m.sharpness/500), m.sharpness>=100, m.sharpness.toFixed(0)) +
      (m.reasons.length? `<div class="hint" style="color:#DC2626">${m.reasons.join(" · ")}</div>`
                       : `<div class="hint" style="color:#059669">✅ 可以采集</div>`) +
      `<div class="hint">已采集 <b>${m.samples||0}</b> ${CAM==="stereo"?"对":"张"}${m.auto?" · 自动采集中":""}</div>`;
    const st=document.getElementById("status");
    st.className="pill "+(ok?"":"bad"); st.textContent=ok?"可采集":"不可用";
  }catch(e){}
}
let AUTO=false;
let AUTO_TIMER=null;
function stopAutoLoop(){ if(AUTO_TIMER){clearInterval(AUTO_TIMER); AUTO_TIMER=null;} }
async function toggleAuto(){
  AUTO=!AUTO;
  const b=document.getElementById("autoBtn");
  b.textContent="自动采集："+(AUTO?"开":"关");
  b.style.background=AUTO?"#059669":"#fff"; b.style.color=AUTO?"#fff":"#1E293B";
  b.style.borderColor=AUTO?"#059669":"#E2E8F0";
  stopAutoLoop();
  if(CAM==="handeye"){
    if(AUTO){
      AUTO_TIMER=setInterval(async()=>{
        try{
          const r=await fetch("/calib/he/capture?hand=left",{method:"POST"});
          const j=await r.json();
          if(j.ok){ toast("已记录 "+j.count+" 个位姿"); }
        }catch(e){}
      }, 3000);
    }
    return;
  }
  await fetch(`/calib/auto?cam=${CAM}&on=${AUTO?1:0}`,{method:"POST"});
}
async function capture(){
  if(CAM==="handeye"){
    const r=await fetch("/calib/he/capture?hand=left",{method:"POST"}); const j=await r.json();
    document.getElementById("status").className="pill "+(j.ok?"":"bad");
    document.getElementById("status").textContent=j.ok?("已记录 "+j.count+" 个位姿"):("拒绝："+j.why);
    return;
  }
  const r=await fetch("/calib/capture?cam="+CAM,{method:"POST"}); const j=await r.json();
  document.getElementById("status").className="pill "+(j.ok?"":"bad");
  document.getElementById("status").textContent=j.ok?("已保存 "+j.count):("拒绝："+j.why);
  refresh();
}
async function calibrate(){
  if(CAM==="handeye"){
    document.getElementById("result").textContent="解算中…";
    const r=await fetch("/calib/he/solve",{method:"POST"}); const j=await r.json();
    document.getElementById("result").textContent = j.ok
      ? `样本 ${j.samples} | 有效旋转对 ${j.pairs}\n残差: 平移 ${j.rms_mm.toFixed(2)} mm | 旋转 ${j.rms_deg.toFixed(3)}°\n\nT_torso_camera:\n` +
        j.T.map(r=>r.map(v=>v.toFixed(4).padStart(9)).join(" ")).join("\n") +
        `\n\n已保存: ${j.path}`
      : ("失败: "+(j.error||""));
    return;
  }
  document.getElementById("result").textContent="计算中…";
  const r=await fetch("/calib/run?cam="+CAM,{method:"POST"}); const j=await r.json();
  document.getElementById("result").textContent = j.ok
    ? `RMS 重投影误差: ${j.rms.toFixed(3)} px\n样本: ${j.samples} 张\n相机矩阵:\n`+
      j.K.map(r=>r.map(v=>v.toFixed(2).padStart(10)).join(" ")).join("\n")+
      `\n\n已保存: ${j.path}`
    : ("失败: "+(j.error||""));
}
async function depthTest(){
  const el=document.getElementById("result");
  el.textContent="测量中…（板子要同时在左右目画面里）";
  const r=await fetch("/calib/depth"); const j=await r.json();
  if(!j.ok){ el.textContent="失败: "+(j.why||""); return; }
  const pnp=j.pnp_depth_mm||{};
  el.textContent =
    `测量点数: ${j.points}   基线: ${j.baseline_mm} mm   fx(校正)=${j.fx_rect}\n` +
    `视差: ${j.disparity_px.mean} px (σ=${j.disparity_px.std})\n` +
    `深度(视差): ${j.depth_mm.mean} mm  σ=${j.depth_mm.std} mm  [${j.depth_mm.min}~${j.depth_mm.max}]\n` +
    `深度(PnP) : ${pnp.mean ?? "-"} mm  σ=${pnp.std ?? "-"} mm\n` +
    `两者差  : ${j.diff_vs_pnp_mm ?? "-"} mm\n` +
    `理论: 1px 视差误差 → ${j.theoretical_sigma_mm_per_px} mm 深度误差`;
}
async function resetAll(){
  await fetch("/calib/reset?cam="+CAM,{method:"POST"}); refresh();
}
async function ping(){
  const el=document.getElementById("conn");
  try{
    const r=await fetch("/calib/he/state",{cache:"no-store"});
    if(!r.ok) throw 0;
    await r.json();
    el.textContent="✅ 服务器已连接";
    el.style.background="rgba(255,255,255,.9)"; el.style.color="#059669";
  }catch(e){
    el.textContent="⛔ 连不上服务器";
    el.style.background="rgba(255,255,255,.9)"; el.style.color="#DC2626";
  }
}
buildTabs(); sel(CAM); ping(); setInterval(ping, 2000);
setInterval(refresh,800);
</script></body></html>"""
