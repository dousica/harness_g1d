"""G1-D 控制台 v2 — Web 层与界面。逻辑在 console_core.py。

    bash run_console.sh            # 真机
    bash run_console.sh --sim      # 无硬件预览
"""
from __future__ import annotations

import argparse
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import numpy as np

import console_core as C
import calib_page
import calib_service

RUNCFG = {"config": "device.local.json", "sim": False}
BUILD = time.strftime("%H%M%S")
STREAM_SLOTS = threading.Semaphore(3)      # 最多 3 路视频流，防止线程堆积
# 每类流只保留最新的一条：新连接进来时，旧的会自动退出
STREAM_GEN = {"grid": 0, "calib": 0, "cam": 0}
STREAM_LOCK = threading.Lock()


def _new_generation(kind):
    with STREAM_LOCK:
        STREAM_GEN[kind] += 1
        return STREAM_GEN[kind]


def _is_current(kind, gen):
    with STREAM_LOCK:
        return STREAM_GEN[kind] == gen

PAGE = r"""<!doctype html><html lang="zh"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>G1-D Console</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
:root{
 --c1:#6366F1; --c2:#818CF8; --c3:#E0E7FF; --c4:#F8FAFC; --c5:#1E293B;
 --ink:#1E293B; --slate:#64748B; --line:#E2E8F0;
 --ok:#059669; --warn:#D97706; --err:#DC2626;
 --mono:ui-monospace,"SF Mono","JetBrains Mono",Menlo,monospace;
}
html,body{height:100%}
body{background:
   radial-gradient(900px 420px at 8% -8%,#E0E7FF 0%,transparent 62%),
   radial-gradient(760px 380px at 100% 0%,#E0E7FF 0%,transparent 58%),
   var(--c4);
 color:var(--ink);font:15.5px/1.62 system-ui,-apple-system,"PingFang SC","Microsoft YaHei",sans-serif;
 -webkit-font-smoothing:antialiased}
header{position:sticky;top:0;z-index:20;display:flex;align-items:center;gap:12px;
 padding:14px 26px;background:linear-gradient(100deg,#6366F1 0%,#7C7CF5 55%,#818CF8 100%);
 box-shadow:0 6px 22px rgba(99,102,241,.28)}
.brand{display:flex;align-items:center;gap:10px;font-weight:700;font-size:16.5px;color:#fff;letter-spacing:.2px}
.logo{width:26px;height:26px;border-radius:8px;background:rgba(255,255,255,.22);
 border:1px solid rgba(255,255,255,.45);box-shadow:inset 0 0 12px rgba(255,255,255,.35)}
header .chip{background:rgba(255,255,255,.16);border:1px solid rgba(255,255,255,.32);color:#fff}
header .chip b{color:#fff}
header .chip.ok{background:rgba(255,255,255,.92);color:#059669;border-color:#fff}
header .chip.ok b{color:#059669}
header .chip.bad{background:rgba(255,255,255,.92);color:#DC2626;border-color:#fff}
header .chip.bad b{color:#DC2626}
.chip{display:inline-flex;align-items:center;gap:6px;padding:6px 13px;border-radius:999px;
 font-size:13px;color:var(--slate);background:#fff;border:1px solid var(--line)}
.chip b{color:var(--ink);font-weight:600}
header button{background:rgba(255,255,255,.16);border-color:rgba(255,255,255,.36);color:#fff}
header button:hover{background:rgba(255,255,255,.28);border-color:#fff;color:#fff}
.spacer{flex:1}
main{display:grid;grid-template-columns:minmax(0,1fr) 470px;gap:18px;padding:20px 24px 32px;align-items:start}
@media(max-width:1150px){main{grid-template-columns:1fr}}
.col{display:flex;flex-direction:column;gap:16px;min-width:0}
.card{background:#fff;border:1px solid var(--line);border-radius:16px;padding:16px;
 box-shadow:0 2px 6px rgba(30,41,59,.05),0 12px 28px rgba(99,102,241,.09)}
.card h2{font-size:12.5px;letter-spacing:.14em;text-transform:uppercase;color:var(--c1);
 margin-bottom:12px;display:flex;align-items:center;gap:9px;font-weight:800}
.card h2::before{content:"";width:4px;height:14px;border-radius:2px;
 background:linear-gradient(180deg,var(--c1),var(--c2))}
.card h2::after{content:"";flex:1;height:1px;background:linear-gradient(90deg,var(--c3),transparent)}
/* 任务卡：白底 + 顶部靛蓝渐变条 */
.card.accent{position:relative;overflow:hidden}
.card.accent::before{content:"";position:absolute;inset:0 0 auto 0;height:3px;
 background:linear-gradient(90deg,var(--c1),var(--c2),var(--c3))}
/* 示教卡：浅靛蓝底 */
.card.tint{background:linear-gradient(180deg,#F5F6FF,#EDF0FF);border-color:#D6DCFF}
.card.tint .item{background:#fff}
/* 关节卡：深色，制造对比 */
.card.dark{background:linear-gradient(180deg,#1E293B,#182233);border-color:#26344A;
 box-shadow:0 12px 32px rgba(30,41,59,.28)}
.card.dark h2{color:var(--c2)}
.card.dark h2::before{background:linear-gradient(180deg,var(--c2),var(--c3))}
.card.dark h2::after{background:linear-gradient(90deg,#33425C,transparent)}
.card.dark .jrow:hover{background:rgba(129,140,248,.12)}
.card.dark .jrow{border-bottom:1px solid rgba(129,140,248,.14)}
.card.dark .jrow .nm{color:#94A3B8}
.card.dark .jrow .val{color:#A5B4FC}
.card.dark button{background:#25324A;border-color:#33425C;color:#E0E7FF}
.card.dark button:hover{background:#31405C;border-color:var(--c2);color:#fff}
.card.dark .hint{color:#7C8AA5}
.cams{display:grid;grid-template-columns:1fr 1fr;gap:12px}
.tile{position:relative;border-radius:11px;overflow:hidden;background:var(--c5);
 border:1px solid #33425C;aspect-ratio:16/9}
.tile img{width:100%;height:100%;object-fit:cover;display:block}
.tile .cap{position:absolute;left:9px;top:8px;font-size:12px;letter-spacing:.04em;
 color:#E0E7FF;background:rgba(30,41,59,.74);padding:3px 8px;border-radius:6px;
 border:1px solid rgba(129,140,248,.35)}
.tile .live{position:absolute;right:9px;top:9px;width:8px;height:8px;border-radius:50%;
 background:#F87171;box-shadow:0 0 0 3px rgba(248,113,113,.18)}
.tile .live.on{background:#34D399;box-shadow:0 0 0 3px rgba(52,211,153,.22)}
.row{display:flex;gap:8px;flex-wrap:wrap;align-items:center}
button{appearance:none;border:1px solid var(--line);background:#fff;color:var(--ink);
 padding:8px 15px;border-radius:9px;font-size:14.5px;font-weight:500;cursor:pointer;transition:.15s}
button:hover{background:var(--c3);border-color:var(--c2);color:#4338CA}
button:active{transform:translateY(1px)}
button.primary{background:linear-gradient(135deg,#6366F1,#818CF8);border-color:transparent;
 color:#fff;box-shadow:0 6px 18px rgba(99,102,241,.35)}
button.primary:hover{background:linear-gradient(135deg,#4F46E5,#6366F1);color:#fff}
button.soft{background:var(--c3);border-color:#C7D2FE;color:#4338CA}
button.soft:hover{background:#D2DBFF;border-color:var(--c2);color:#3730A3}
button.mini{padding:4px 11px;font-size:13px;border-radius:7px}
button.danger{color:var(--err);border-color:#FECACA;background:#FEF2F2}
button.danger:hover{background:#FEE2E2;border-color:#FCA5A5;color:var(--err)}
button:disabled{opacity:.45;cursor:not-allowed;transform:none}
input[type=text],textarea{width:100%;background:#fff;border:1px solid var(--line);
 border-radius:10px;color:var(--ink);padding:9px 11px;font:inherit;outline:none;transition:.15s}
input[type=text]::placeholder,textarea::placeholder{color:#A3AEC2}
input[type=text]:focus,textarea:focus{border-color:var(--c2);
 box-shadow:0 0 0 3px rgba(129,140,248,.22)}
textarea{min-height:104px;resize:vertical;line-height:1.65}
.jlist{max-height:300px;overflow:auto;margin-top:4px}
.jrow{display:grid;grid-template-columns:20px 1fr 76px 34px 34px;gap:7px;align-items:center;
 padding:6px 5px;border-radius:7px;font-size:14px}
.jrow .nm{color:var(--slate);overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
.jrow .val{font-family:var(--mono);color:#4338CA;text-align:right;font-size:14px;font-weight:700}
.jrow button{padding:2px 0;width:32px;border-radius:6px;font-size:13px}
input[type=checkbox]{accent-color:var(--c1);width:15px;height:15px}
.lib{max-height:150px;overflow:auto;margin-top:10px}
.item{display:flex;justify-content:space-between;align-items:center;gap:8px;
 padding:8px 10px;border:1px solid #D6DCFF;border-radius:10px;margin-bottom:6px;background:#fff}
.item .nm{font-family:var(--mono);font-size:14px;color:#4338CA;font-weight:700}
.item .meta{color:var(--slate);font-size:12.5px;margin-left:6px;font-family:var(--mono)}
.log{height:250px;overflow:auto;background:linear-gradient(180deg,#F8FAFC,#F1F4FF);
 border:1px solid var(--line);border-radius:11px;padding:9px 11px;
 font-family:var(--mono);font-size:13.5px;line-height:1.75}
.log div{padding:1px 0 1px 9px;border-left:2px solid #CBD5E1;white-space:pre-wrap;word-break:break-word}
.k-think{color:#475569;border-color:#CBD5E1}
.k-move{color:#047857;border-color:#6EE7B7}
.k-grip{color:#1D4ED8;border-color:#93C5FD}
.k-look{color:#6D28D9;border-color:#C4B5FD}
.k-finish{color:#047857;border-color:#059669;font-weight:700}
.k-warn{color:var(--warn);border-color:#FCD34D}
.k-error{color:var(--err);border-color:#FCA5A5}
.k-dry,.k-stop{color:#94A3B8;border-color:#E2E8F0}
.k-api{color:#0369A1;border-color:#7DD3FC}
.k-info{color:#475569;border-color:#CBD5E1}
.k-step{color:#4338CA;border-color:#A5B4FC;font-weight:600}
.hint{color:#8B97AC;font-size:12.5px;line-height:1.65;margin-top:9px}
.sep{height:1px;background:linear-gradient(90deg,#D6DCFF,transparent);margin:13px 0}
.estop{background:linear-gradient(180deg,#EF4444,#DC2626);color:#fff;border:none;
 font-weight:800;letter-spacing:.04em;padding:9px 18px;border-radius:10px;
 box-shadow:0 6px 18px rgba(220,38,38,.45)}
.estop:hover{background:linear-gradient(180deg,#DC2626,#B91C1C);color:#fff}
.estopbar{display:none;align-items:center;gap:10px;padding:10px 24px;
 background:#FEF2F2;border-bottom:2px solid #FCA5A5;color:#B91C1C;
 font-weight:700;font-size:14px}
.estopbar.on{display:flex}
.toast{position:fixed;right:22px;bottom:22px;background:var(--c5);color:#E0E7FF;
 padding:11px 16px;border-radius:11px;font-size:13px;opacity:0;transform:translateY(8px);
 transition:.2s;pointer-events:none;z-index:99;box-shadow:0 14px 34px rgba(30,41,59,.35);
 border:1px solid #33425C}
.toast.on{opacity:1;transform:none}
</style></head><body>
<header>
  <div class="brand"><span class="logo"></span> G1-D Console</div>
  <span class="chip" id="cLink"><b>●</b> 连接中</span>
  <span class="chip" id="cMode"><b>●</b> 只读</span>
  <span class="chip" id="cErr">误差 <b>0.000</b></span>
  <span class="spacer"></span>
  <button class="mini" id="engBtn" onclick="toggleEngage()" style="font-weight:700">接管手臂</button>
  <button class="mini" onclick="goZero()">回零位</button>
  <button class="mini" onclick="goDangle()">回下垂位</button>
  <button class="mini" onclick="reconnect()">重连手臂</button>
  <button class="mini" id="hbBtn" onclick="toggleHb()">看门狗：关</button>
  <button class="estop" id="estopBtn" onclick="doEstop()">急停 ESC</button>
</header>
<div class="estopbar" id="estopbar">⚠ 已急停 · <span id="estopReason"></span>
  <button class="mini" onclick="doResume()" style="margin-left:auto">解除急停</button></div>

<main>
  <div class="col">
    <div class="card">
      <h2>视觉</h2>
      <div class="row" style="margin-bottom:10px">
        <button class="mini" id="camBtn" onclick="toggleCam()">画面：开</button>
        <button class="mini" onclick="depthNow()">测深度</button>
        <span class="hint" id="depthOut" style="margin:0;font-family:var(--mono)"></span>
      </div>
      <div class="tile" style="aspect-ratio:16/9">
        <img id="camimg" src="/cam/grid" style="object-fit:contain">
        <span class="live" id="l-grid"></span>
      </div>
    </div>

    <div class="card tint">
      <h2>示教 · 轨迹库</h2>
      <div class="row">
        <input type="text" id="wpname" placeholder="航点名，如 tape_above" style="flex:1">
        <button class="primary" onclick="saveWp()">记录当前姿态</button>
      </div>
      <div class="lib" id="wplist"></div>
      <div class="sep"></div>
      <div class="row">
        <input type="text" id="seqname" placeholder="轨迹名，如 pick_tape" style="flex:1">
        <input type="text" id="seqlist" placeholder="航点顺序：a, b, c" style="flex:1.3">
        <button class="primary" onclick="saveSeq()">存轨迹</button>
      </div>
      <div class="lib" id="seqs"></div>
      <div class="hint">点「记录」把当前关节角存成航点；多个航点串成轨迹后可一键执行。GPT 优先调用航点，再微调。</div>
    </div>
  </div>

  <div class="col">
    <div class="card accent">
      <h2>任务 · GPT-6 Astra</h2>
      <textarea id="task" placeholder="例如：用 tape_above 到黑胶带上方 → tape_grasp 夹起 → blue_above → blue_insert 放进孔里。夹住后先用 look_gripper 确认。"></textarea>
      <div class="row" style="margin-top:10px">
        <button class="primary" id="runbtn" onclick="runTask()">执行任务</button>
        <button class="soft" onclick="stopTask()">中止</button>
        <label class="chip" style="cursor:pointer"><input type="checkbox" id="allow" checked style="margin-right:6px">允许运动</label>
      </div>
    </div>

    <div class="card">
      <h2>日志</h2>
      <div class="log" id="log"></div>
    </div>

    <div class="card dark">
      <h2>关节</h2>
      <div class="row" style="margin-bottom:8px">
        <button class="mini" onclick="sel('left')">左臂</button>
        <button class="mini" onclick="sel('right')">右臂</button>
        <button class="mini" onclick="sel('none')">清空</button>
        <span class="hint" style="margin:0">勾选后一起动</span>
      </div>
      <div class="row" style="margin-bottom:4px">
        <button class="mini" onclick="jogSel(-0.05)">−0.05</button>
        <button class="mini" onclick="jogSel(0.05)">+0.05</button>
        <button class="mini" onclick="jogSel(-0.15)">−0.15</button>
        <button class="mini" onclick="jogSel(0.15)">+0.15</button>
        <span class="hint" style="margin:0">开口</span>
        <input type="range" id="gripCm" min="0" max="4.5" step="0.1" value="0"
               style="flex:1;max-width:150px" oninput="setGripCm(this.value)">
        <span class="hint" style="margin:0;font-family:var(--mono)" id="gripCmVal">0.0 cm</span>
        <button class="mini" onclick="setGripFull()">全开距离</button>
        <button class="mini" id="gpBtn" onclick="toggleGripPower()">夹爪断电</button>
        <button class="mini" onclick="probeGrip()">测量行程</button>
        <span class="hint" id="gripInfo" style="margin:0;font-family:var(--mono)">夹爪 --</span>
      </div>
      <div class="jlist" id="jlist"></div>
    </div>
  </div>
</main>
<div class="toast" id="toast"></div>
<script>
const NAMES=NAMES_JSON, selSet=new Set();
let ENGAGED=false, ESTOPPED=false; const PAGE_BUILD='__BUILD__';
function toast(t){const e=document.getElementById('toast');e.textContent=t;e.classList.add('on');
 clearTimeout(e._t);e._t=setTimeout(()=>e.classList.remove('on'),1600)}
function build(){
 document.getElementById('jlist').innerHTML=NAMES.map((n,i)=>
  `<div class="jrow"><input type="checkbox" data-i="${i}" onchange="tog(${i},this.checked)">
   <span class="nm" title="${n}">${n.replace(/_joint$/,'')}</span>
   <span class="val" id="v${i}">0.000</span>
   <button onclick="jogOne(${i},-0.05)">−</button><button onclick="jogOne(${i},0.05)">+</button></div>`).join('');
}
function tog(i,on){on?selSet.add(i):selSet.delete(i)}
function sel(k){selSet.clear();document.querySelectorAll('#jlist input').forEach(c=>{
 const n=NAMES[+c.dataset.i];const on=k==='none'?false:(k==='left'?n.startsWith('left'):n.startsWith('right'));
 c.checked=on;if(on)selSet.add(+c.dataset.i)})}
async function post(u,b){return fetch(u,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(b||{})})}
function needEngage(){if(!ENGAGED){toast('先点右上角「接管手臂」');return true}return false}
function jogOne(i,d){if(needEngage())return;const o={};o[NAMES[i]]=d;post('/jog_multi',{deltas:o})}
function jogSel(d){if(needEngage())return;if(!selSet.size)return toast('先勾选关节');const o={};selSet.forEach(i=>o[NAMES[i]]=d);
 post('/jog_multi',{deltas:o}).then(()=>toast('已下发 '+selSet.size+' 个关节'))}
function grip(a){if(needEngage())return;post('/grip',{action:a})}
async function probeGrip(){
  if(!ENGAGED) return toast('先接管手臂');
  if(!confirm('夹爪会慢慢关到底再开到底，靠力矩判断限位。\n确认夹爪里没有东西、周围无人？')) return;
  toast('正在测量行程，约 10 秒…');
  const r=await fetch('/grip_probe',{method:'POST'}); const j=await r.json();
  toast(j.ok ? ('行程: '+j.min.toFixed(3)+' ~ '+j.max.toFixed(3)) : ('失败: '+j.why));
}
async function toggleHb(){
  HBON=!HBON;
  await post('/heartbeat_enable?on='+(HBON?1:0));
  const b=document.getElementById('hbBtn');
  b.textContent='看门狗：'+(HBON?'开':'关');
  b.style.background=HBON?'#059669':'#fff';
  b.style.color=HBON?'#fff':'#1E293B';
  toast(HBON?'看门狗已开启：页面失联 8 秒自动急停':'看门狗已关闭');
}
let HBON=false, GPOWER=true, GRIPCMSUPPRESS=false;
async function setGripCm(v){
  if(GRIPCMSUPPRESS) return;
  await post('/grip_cm',{cm:parseFloat(v)});
  document.getElementById('gripCmVal').textContent=parseFloat(v).toFixed(1)+' cm';
}
async function setGripFull(){
  const v = prompt('全张开时两指之间的实际距离（厘米）？\n用尺子量一下全开的状态。', '4.5');
  if(v===null) return;
  await post('/grip_full_cm',{cm:parseFloat(v)});
  const sl=document.getElementById('gripCm'); sl.max=v;
  toast('全开行程已设为 '+v+' cm');
}
async function toggleGripPower(){
  GPOWER=!GPOWER;
  await post('/grip_power?on='+(GPOWER?1:0));
  const b=document.getElementById('gpBtn');
  b.textContent=GPOWER?'夹爪断电':'夹爪通电';
  b.style.background=GPOWER?'#fff':'#D97706';
  b.style.color=GPOWER?'#1E293B':'#fff';
  toast(GPOWER?'夹爪已通电':'夹爪已断电，两指可自由扳动');
}
function home(){if(needEngage())return;post('/home');toast('正在回启动位（半速）')}
function resetAsk(){
  if(needEngage())return;
  if(confirm('复位 = 手臂盲走到启动位，不做避障。\n确认周围无遮挡、急停在手边？')) home();
}
function freeze(){post('/freeze');toast('已冻结')}
async function doEstop(){ await post('/estop',{reason:'界面按钮'}); toast('已急停'); tick(); }
async function doResume(){ await post('/resume'); toast('急停已解除'); tick(); }
document.addEventListener('keydown',e=>{
  if(e.key==='Escape'){ e.preventDefault(); if(!ESTOPPED) doEstop(); }
});
setInterval(()=>{if(document.visibilityState==='visible')post('/heartbeat')},2000);
async function precheckThen(target, title, ask){
  const r = await post('/precheck',{target}); const j = await r.json();
  const head = j.ok ? '✅ 碰撞检测：通过' : '⛔ 碰撞检测：拒绝';
  const body = j.why && j.why !== 'ok' ? ('\n原因：'+j.why) : '';
  if(!confirm(head + body + '\n\n' + ask)) return false;
  return true;
}
let CONNECT={state:"idle",attempt:0,total:4,error:null};
let CAMON=true;
const BLANK='data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7';
function toggleCam(){
  CAMON=!CAMON;
  const img=document.getElementById('camimg');
  const b=document.getElementById('camBtn');
  if(CAMON){ img.src='/cam/grid?v='+Date.now(); }
  else { img.src=BLANK; }          // 置空会断掉 MJPEG 连接，释放并发
  b.textContent='画面：'+(CAMON?'开':'关');
  b.style.background=CAMON?'#fff':'#D97706';
  b.style.color=CAMON?'#1E293B':'#fff';
  toast(CAMON?'画面已开启':'画面已关闭（已释放视频连接）');
}
async function depthNow(){
  const el=document.getElementById('depthOut');
  el.textContent='测量中…';
  try{
    const r=await fetch('/calib/depth'); const j=await r.json();
    if(!j.ok){ el.textContent='深度：'+(j.why||'失败'); return; }
    const pnp=j.pnp_depth_mm||{};
    el.innerHTML='深度 <b>'+j.depth_mm.mean+' mm</b>（σ '+j.depth_mm.std+
      '，PnP '+pnp.mean+' mm，差 '+j.diff_vs_pnp_mm+' mm）';
  }catch(e){ el.textContent='深度：请求失败'; }
}
async function toggleEngage(){
  if(ENGAGED){ await post('/release'); toast('已释放，控制权交回机器人'); tick(); return; }
  if(!confirm('接管手臂（不会移动，只在当前姿态接管）。\n\n确认周围无遮挡、急停在手边？')) return;
  toast('正在连接机器人，请稍候…');
  await post('/takeover');
}
function renderConnect(){
  const c=CONNECT||{}; const el=document.getElementById('cErr');
  if(c.state==='connecting'){
    el.innerHTML='<b style="color:#D97706">连接中 '+c.attempt+'/'+c.total+'</b>';
  } else if(c.state==='failed'){
    el.innerHTML='<b style="color:#DC2626">连接失败</b>';
  }
}
async function goZero(){
  if(!ENGAGED) return toast('先点「进入工作状态」');
  if(!await precheckThen('zero','回零位','两臂将缓慢移动到【官方零位】。确认继续？')) return;
  const r = await post('/pose',{name:'zero'}); const j = await r.json();
  toast(j.ok ? '正在回零位' : ('被拒绝：'+j.why));
}
async function goDangle(){
  if(!ENGAGED) return toast('先点「进入工作状态」');
  if(!await precheckThen('dangle','回下垂位','两臂将缓慢下降到【自然下垂位】。确认继续？')) return;
  const r = await post('/pose',{name:'dangle'}); const j = await r.json();
  toast(j.ok ? '正在下降到下垂位' : ('被拒绝：'+j.why));
}
async function reconnect(){toast('正在重连…');const r=await post('/reset_arm');const j=await r.json();toast(j.ok?'已重连':('重连失败 '+j.error))}
async function saveWp(){const n=document.getElementById('wpname').value.trim();
 if(!n)return toast('给航点起个名字');await post('/waypoint',{name:n});
 document.getElementById('wpname').value='';toast('已记录 '+n);loadLib()}
function delWp(n){post('/wp_delete',{name:n}).then(loadLib)}
function gotoWp(n){post('/goto',{name:n});toast('回放 '+n)}
async function saveSeq(){const n=document.getElementById('seqname').value.trim();
 const raw=document.getElementById('seqlist').value.trim();
 if(!n||!raw)return toast('轨迹名和航点顺序都要填');
 await post('/seq_save',{name:n,waypoints:raw.split(/[,，>\s]+/).filter(Boolean)});
 document.getElementById('seqname').value='';document.getElementById('seqlist').value='';
 toast('已存轨迹 '+n);loadLib()}
function playSeq(n){post('/seq_play',{name:n});toast('执行轨迹 '+n)}
function delSeq(n){post('/seq_delete',{name:n}).then(loadLib)}
async function loadLib(d){
 if(!d) d=await (await fetch('/lib')).json();
 document.getElementById('wplist').innerHTML=Object.entries(d.waypoints).map(([k,v])=>
  `<div class="item"><span><span class="nm">${k}</span><span class="meta">grip ${v.grip}</span></span>
   <span><button class="mini" onclick="gotoWp('${k}')">回放</button>
   <button class="mini danger" onclick="delWp('${k}')">删</button></span></div>`).join('')||'<div class="hint">还没有航点</div>';
 document.getElementById('seqs').innerHTML=Object.entries(d.sequences).map(([k,v])=>
  `<div class="item"><span><span class="nm">${k}</span><span class="meta">${v.waypoints.join(' → ')}</span></span>
   <span><button class="mini primary" onclick="playSeq('${k}')">执行</button>
   <button class="mini danger" onclick="delSeq('${k}')">删</button></span></div>`).join('')||'<div class="hint">还没有轨迹</div>';
}
async function tick(s){
 try{
  if(!s) s=await (await fetch('/state',{cache:'no-store'})).json();
  document.getElementById('cLink').className='chip ok';document.getElementById('cLink').innerHTML='<b>●</b> 已连接';
  if(s.build && s.build!==PAGE_BUILD){
    document.getElementById('cLink').className='chip bad';
    document.getElementById('cLink').innerHTML='<b>●</b> 页面已过期，请 Ctrl+F5';
    return;
  }
  ENGAGED=!!s.engaged; ESTOPPED=!!s.estopped;
  if(s.connect){ const before=CONNECT.state; CONNECT=s.connect; renderConnect();
    if(before!==CONNECT.state){
      if(CONNECT.state==='ready') toast('已接管，手臂保持不动');
      else if(CONNECT.state==='failed') toast('接管失败：'+(CONNECT.error||''));
    }
  }
  if(typeof s.hb==='boolean' && s.hb!==HBON){
    HBON=s.hb; const b=document.getElementById('hbBtn');
    b.textContent='看门狗：'+(HBON?'开':'关');
    b.style.background=HBON?'#059669':'#fff'; b.style.color=HBON?'#fff':'#1E293B';
  }
  const bar=document.getElementById('estopbar');
  bar.className='estopbar'+(ESTOPPED?' on':'');
  if(ESTOPPED) document.getElementById('estopReason').textContent=s.estop_reason||'';
  const b=document.getElementById('engBtn');
  b.textContent=ENGAGED?'释放控制':'接管手臂';
  b.style.background=ENGAGED?'rgba(255,255,255,.92)':'rgba(255,255,255,.16)';
  b.style.color=ENGAGED?'#DC2626':'#fff';
  document.getElementById('cMode').className='chip '+(ENGAGED?'ok':'bad');
  document.getElementById('cMode').innerHTML='<b>●</b> '+(ENGAGED?'已接管':'未接管');
  document.getElementById('cErr').innerHTML='误差 <b>'+s.busy.toFixed(3)+'</b>';
  if(s.grip){const g=s.grip;
    const sl=document.getElementById('gripCm');
    if(sl && typeof g.cm==='number' && document.activeElement!==sl){
      GRIPCMSUPPRESS=true; sl.value=g.cm; sl.max=g.full_cm||4.5; GRIPCMSUPPRESS=false;
      document.getElementById('gripCmVal').textContent=g.cm.toFixed(1)+' cm';
    }
    document.getElementById('gripInfo').innerHTML=
      '夹爪 <b>'+g.cmd.toFixed(2)+'</b> → '+g.goal.toFixed(2)+
      ' | τ '+g.peak_tau.toFixed(2)+' N·m'+(g.grasped?' <b style="color:#059669">夹住</b>':'');
  }
  s.q.forEach((v,i)=>{const e=document.getElementById('v'+i);if(e)e.textContent=v.toFixed(3)});
 }catch(e){document.getElementById('cLink').className='chip bad';
  document.getElementById('cLink').innerHTML='<b>●</b> 断开'}
}
async function tickLog(a){
 if(!a) a=await (await fetch('/agent',{cache:'no-store'})).json();
 const el=document.getElementById('log');
 el.innerHTML=a.log.map(l=>`<div class="k-${l.kind}">[${l.t}] ${l.text}</div>`).join('');
 el.scrollTop=el.scrollHeight;
 document.getElementById('runbtn').disabled=a.running;
}
function runTask(){const t=document.getElementById('task').value.trim();
 if(!t)return toast('写一句任务');
 post('/run',{task:t,allow:document.getElementById('allow').checked});toast('任务已下发')}
function stopTask(){post('/stop_agent');toast('已请求中止')}
async function camStatus(d){
 try{
  if(!d) d=await (await fetch('/camstat',{cache:'no-store'})).json();
  const e=document.getElementById('l-grid');
  if(e) e.className='live '+((d.head&&d.lwrist&&d.rwrist)?'on':'');
 }catch(e){}
}
async function refreshAll(){
  let d;
  try{
    const r=await fetch('/status',{cache:'no-store'});
    if(!r.ok) throw 0;
    d=await r.json();
  }catch(e){
    document.getElementById('cLink').className='chip bad';
    document.getElementById('cLink').innerHTML='<b>●</b> 断开';
    return;
  }
  tick(d.state); tickLog(d.agent); loadLib(d.lib); camStatus(d.cam);
}
build(); refreshAll();
setInterval(refreshAll, 600);      // 一次请求拿全部状态，原来要 4 个请求
</script></body></html>"""


class Server(ThreadingHTTPServer):
    """默认监听队列只有 5，浏览器一开多个连接就会溢出丢连接。"""
    request_queue_size = 128
    daemon_threads = True
    allow_reuse_address = True


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.0"

    def log_message(self, *a):
        pass

    def _json(self, obj, code=200):
        data = json.dumps(obj, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _body(self):
        n = int(self.headers.get("Content-Length") or 0)
        if not n:
            return {}
        try:
            return json.loads(self.rfile.read(n) or b"{}")
        except json.JSONDecodeError:
            return {}

    # ---------------------------------------------------------------- GET
    def do_GET(self):
        path = urlparse(self.path).path
        arm = C.S.arm
        if path == "/":
            html = (PAGE.replace("NAMES_JSON", json.dumps(list(C.ARM_JOINT_NAMES)))
                        .replace("__BUILD__", BUILD))
            data = html.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
        elif path == "/state":
            self._json({"q": arm.q().tolist(), "busy": arm.busy(), "fault": arm.fault,
                        "motion": C.MOTION["allow"],
                        "engaged": bool(getattr(arm, "engaged", False)),
                        "estopped": bool(getattr(arm, "estopped", False)),
                        "estop_reason": getattr(arm, "estop_reason", None),
                        "mode": getattr(arm, "mode", "?"),
                        "grip": arm.grip_state() if hasattr(arm, "grip_state") else {},
                        "collision": bool(getattr(arm, "collision", None) is not None),
                        "build": BUILD,
                        "hb": bool(getattr(arm, "hb_enabled", False))})
        elif path == "/lib":
            with C.S.lock:
                self._json({"waypoints": C.S.waypoints, "sequences": C.S.sequences})
        elif path == "/agent":
            with C.S.lock:
                self._json({"running": C.S.agent["running"], "log": C.S.agent["log"]})
        elif path == "/calib":
            data = calib_page.PAGE.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
        elif path == "/calib/stream":
            qs = parse_qs(urlparse(self.path).query)
            self._calib_stream((qs.get("cam") or ["head_left"])[0])
        elif path == "/calib/depth":
            self._json(calib_service.depth_live())
        elif path == "/calib/he/state":
            self._json(calib_service.handeye_state())
        elif path == "/calib/score":
            qs = parse_qs(urlparse(self.path).query)
            self._json(calib_service.score((qs.get("cam") or ["head_left"])[0]))
        elif path == "/status":
            with C.S.lock:
                agent = {"running": C.S.agent["running"], "log": C.S.agent["log"][-80:]}
                lib = {"waypoints": C.S.waypoints, "sequences": C.S.sequences}
            self._json({
                "state": {"q": arm.q().tolist(), "busy": arm.busy(), "fault": arm.fault,
                          "motion": C.MOTION["allow"], "engaged": bool(getattr(arm, "engaged", False)),
                          "estopped": bool(getattr(arm, "estopped", False)),
                          "estop_reason": getattr(arm, "estop_reason", None),
                          "mode": getattr(arm, "mode", "?"),
                          "grip": arm.grip_state() if hasattr(arm, "grip_state") else {},
                          "collision": bool(getattr(arm, "collision", None) is not None),
                          "build": BUILD, "hb": bool(getattr(arm, "hb_enabled", False)),
                          "connect": getattr(arm, "connect_state",
                                             {"state": "idle", "attempt": 0, "error": None})},
                "agent": agent, "lib": lib,
                "cam": {k: cam.jpeg() is not None for k, cam in C.CAMS.items()},
            })
        elif path == "/camstat":
            self._json({k: cam.jpeg() is not None for k, cam in C.CAMS.items()})
        elif path == "/cam/grid":
            self._grid_stream()
        elif path.startswith("/cam/"):
            self._stream(path.split("/", 2)[2])
        else:
            self._json({"error": "not found"}, 404)

    @staticmethod
    def _decode_fast(jpg, reduced=4):
        """直接以 1/reduced 尺寸解码，省掉一次大图 resize。"""
        import cv2
        import numpy as np
        flag = {1: cv2.IMREAD_COLOR, 2: cv2.IMREAD_REDUCED_COLOR_2,
                4: cv2.IMREAD_REDUCED_COLOR_4, 8: cv2.IMREAD_REDUCED_COLOR_8}.get(
                    reduced, cv2.IMREAD_COLOR)
        return cv2.imdecode(np.frombuffer(jpg, np.uint8), flag)

    @staticmethod
    def _shrink(jpg, max_w=800, quality=70):
        """把大图压成适合网页传输的小图（不改变标定/检测用的原图）。"""
        try:
            import cv2
            import numpy as np
            img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                return jpg
            h, w = img.shape[:2]
            if w <= max_w:
                ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, quality])
                return buf.tobytes() if ok else jpg
            nh = int(max_w * h / w)
            img = cv2.resize(img, (max_w, nh), interpolation=cv2.INTER_AREA)
            ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, quality])
            return buf.tobytes() if ok else jpg
        except Exception:
            return jpg

    _grid_cache = {"key": None, "data": None}

    @staticmethod
    def _grid_jpeg():
        """把四路相机合成一张 2x2 画面：只占浏览器 1 个连接。"""
        import cv2
        import numpy as np
        key = tuple(id(cam.jpeg()) for cam in
                    (C.CAMS["head"], C.CAMS["head"], C.CAMS["lwrist"], C.CAMS["rwrist"]))
        cache = Handler._grid_cache
        if cache["key"] == key and cache["data"] is not None:
            return cache["data"]
        tiles = []
        for name, (src, half) in (
                ("头部左目", (C.CAMS["head"], "left")),
                ("头部右目", (C.CAMS["head"], "right")),
                ("左腕", (C.CAMS["lwrist"], None)),
                ("右腕", (C.CAMS["rwrist"], None))):
            jpg = src.jpeg() if src is not None else None
            if jpg is None:
                tiles.append(None)
                continue
            img = Handler._decode_fast(jpg, reduced=8 if half is not None else 4)
            if img is None:
                tiles.append(None)
                continue
            if half is not None:
                # head 是 3840x1080，1/8 解码后 480x135，再切左右
                w = img.shape[1]
                img = img[:, : w // 2] if half == "left" else img[:, w // 2:]
            if img.shape[1] != 480 or img.shape[0] != 270:
                import cv2 as _cv
                img = _cv.resize(img, (480, 270))
            tiles.append(img)
        if not any(t is not None for t in tiles):
            return None
        blank = np.full((270, 480, 3), 18, np.uint8)
        tiles = [t if t is not None else blank for t in tiles]
        for t, label in zip(tiles, ("HEAD-L", "HEAD-R", "WRIST-L", "WRIST-R")):
            cv2.rectangle(t, (0, 0), (150, 26), (0, 0, 0), -1)
            cv2.putText(t, label, (8, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 220, 255), 1)
        top = np.hstack([tiles[0], tiles[1]])
        bot = np.hstack([tiles[2], tiles[3]])
        grid = np.vstack([top, bot])
        ok, buf = cv2.imencode(".jpg", grid, [cv2.IMWRITE_JPEG_QUALITY, 72])
        data = buf.tobytes() if ok else None
        Handler._grid_cache = {"key": key, "data": data}
        return data

    def _stream(self, name):
        gen = _new_generation("cam")
        if not STREAM_SLOTS.acquire(blocking=False):
            self._json({"error": "too many video streams"}, 503)
            return
        try:
            self._stream_body(name, gen)
        finally:
            STREAM_SLOTS.release()

    def _stream_body(self, name, gen):
        try:
            self.connection.settimeout(5.0)
        except Exception:
            pass
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        last = None
        while _is_current("cam", gen):
            if name == "head_left":
                jpg = C.eye_jpeg("left")
            elif name == "head_right":
                jpg = C.eye_jpeg("right")
            else:
                cam = C.CAMS.get(name)
                jpg = cam.jpeg() if cam else None
            if jpg is not None and jpg is not last:
                last = jpg
                jpg = self._shrink(jpg, max_w=640, quality=62)
                try:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                     + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
                except Exception:
                    return
            time.sleep(1 / 4)

    def _grid_stream(self):
        gen = _new_generation("grid")
        if not STREAM_SLOTS.acquire(blocking=False):
            self._json({"error": "too many video streams"}, 503)
            return
        try:
            self._grid_stream_body(gen)
        finally:
            STREAM_SLOTS.release()

    def _grid_stream_body(self, gen):
        try:
            self.connection.settimeout(5.0)
        except Exception:
            pass
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        last = None
        while _is_current("grid", gen):
            jpg = self._grid_jpeg()
            if jpg is not None and jpg is not last:
                last = jpg
                try:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                     + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
                except Exception:
                    return
            time.sleep(1 / 5)

    def _calib_stream(self, cam):
        gen = _new_generation("calib")
        if not STREAM_SLOTS.acquire(blocking=False):
            self._json({"error": "too many video streams"}, 503)
            return
        try:
            self._calib_stream_body(cam, gen)
        finally:
            STREAM_SLOTS.release()

    def _calib_stream_body(self, cam, gen):
        try:
            self.connection.settimeout(5.0)
        except Exception:
            pass
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        while _is_current("calib", gen):
            jpg = calib_service.stream_frame(cam)
            if jpg is None:
                time.sleep(0.3)
                continue
            try:
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                 + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
            except Exception:
                return
            time.sleep(0.33)     # ~3 fps，检测要时间

    # --------------------------------------------------------------- POST
    def do_POST(self):
        path = urlparse(self.path).path
        body = self._body()
        arm = C.S.arm
        if path == "/jog_multi":
            self._json({"moved": arm.jog(body.get("deltas") or {})})
        elif path == "/grip":
            if "value" in body:
                val = float(body["value"])
            else:
                val = (arm.grip_open if body.get("action") == "open"
                       else arm.grip_close)
            arm.grip(val)
            self._json({"grip": val})
        elif path == "/home":
            ok, why = arm.go_zero()
            self._json({"ok": ok, "why": why}, 200 if ok else 409)
        elif path == "/freeze":
            with arm.lock:
                arm.target = arm.current.copy()
            self._json({"ok": True})
        elif path == "/waypoint":
            name = str(body.get("name") or "").strip()
            if not name:
                self._json({"error": "name required"}, 400); return
            end = time.time() + 2.0
            while time.time() < end and arm.busy() > 0.01:
                time.sleep(0.05)
            with C.S.lock:
                C.S.waypoints[name] = {"q": arm.q().tolist(),
                                       "grip": getattr(arm, "grip_val", C.GRIP_OPEN),
                                       "ts": time.strftime("%H:%M:%S")}
            C.S.save()
            self._json({"ok": True})
        elif path == "/wp_delete":
            with C.S.lock:
                C.S.waypoints.pop(str(body.get("name")), None)
            C.S.save(); self._json({"ok": True})
        elif path == "/goto":
            with C.S.lock:
                wp = C.S.waypoints.get(str(body.get("name")))
            if not wp:
                self._json({"error": "no such waypoint"}, 404); return
            ok, why = arm.goto(wp["q"])
            if ok:
                arm.grip(wp.get("grip", C.GRIP_OPEN))
            self._json({"ok": ok, "why": why}, 200 if ok else 409)
        elif path == "/seq_save":
            name = str(body.get("name") or "").strip()
            wps = [str(x) for x in body.get("waypoints") or [] if x]
            if not name or not wps:
                self._json({"error": "name and waypoints required"}, 400); return
            with C.S.lock:
                C.S.sequences[name] = {"waypoints": wps}
            C.S.save(); self._json({"ok": True})
        elif path == "/seq_delete":
            with C.S.lock:
                C.S.sequences.pop(str(body.get("name")), None)
            C.S.save(); self._json({"ok": True})
        elif path == "/seq_play":
            threading.Thread(target=C.play_sequence, args=(str(body.get("name")),),
                             daemon=True).start()
            self._json({"ok": True})
        elif path == "/run":
            if C.S.agent["running"]:
                self._json({"error": "agent already running"}, 409); return
            if not getattr(C.S.arm, "engaged", False):
                self._json({"error": "手臂未接管，请先点「接管手臂」"}, 409); return
            C.MOTION["allow"] = bool(body.get("allow", True))
            with C.S.lock:
                C.S.agent.update({"running": True, "log": [], "done": False})
            threading.Thread(target=C.run_agent,
                             args=(str(body.get("task", "")), 30, C.MOTION["allow"]),
                             daemon=True).start()
            self._json({"ok": True})
        elif path == "/calib/depth":
            self._json(calib_service.depth_live())
        elif path == "/calib/he/state":
            self._json(calib_service.handeye_state())
        elif path == "/calib/he/capture":
            qs = parse_qs(urlparse(self.path).query)
            self._json(calib_service.handeye_capture((qs.get("hand") or ["left"])[0]))
        elif path == "/calib/he/solve":
            self._json(calib_service.handeye_solve())
        elif path == "/calib/he/reset":
            self._json(calib_service.handeye_reset())
        elif path == "/calib/auto":
            qs = parse_qs(urlparse(self.path).query)
            cam = (qs.get("cam") or ["head_left"])[0]
            on = (qs.get("on") or ["1"])[0] not in ("0", "false", "")
            calib_service.set_auto(cam, on)
            self._json({"ok": True, "on": on})
        elif path in ("/calib/capture", "/calib/run", "/calib/reset"):
            qs = parse_qs(urlparse(self.path).query)
            cam = (qs.get("cam") or ["head_left"])[0]
            if cam not in calib_service.CAMS:
                self._json({"ok": False, "error": f"未知相机 {cam}"}, 400); return
            if path == "/calib/capture":
                ok, why, n = calib_service.capture(cam)
                self._json({"ok": ok, "why": why, "count": n})
            elif path == "/calib/run":
                self._json(calib_service.run(cam))
            else:
                calib_service.reset(cam)
                self._json({"ok": True})
        elif path == "/estop":
            C.S.arm.estop(str(body.get("reason") or "界面急停"))
            C.S.agent["running"] = False
            self._json({"ok": True})
        elif path == "/resume":
            C.S.arm.resume()
            self._json({"ok": True})
        elif path == "/grip_cm":
            cm = float(body.get("cm", 0))
            self._json(arm.grip_cm(cm) or {"ok": False})
        elif path == "/grip_full_cm":
            self._json(arm.set_grip_full_cm(float(body.get("cm", 4.5))))
        elif path == "/grip_probe":
            self._json(arm.probe_gripper())
        elif path == "/grip_power":
            qs = parse_qs(urlparse(self.path).query)
            on = (qs.get("on") or ["1"])[0] not in ("0", "false", "")
            self._json(arm.set_grip_power(on))
        elif path == "/heartbeat_enable":
            qs = parse_qs(urlparse(self.path).query)
            on = (qs.get("on") or ["0"])[0] not in ("0", "false", "")
            self._json(arm.set_heartbeat(on))
        elif path == "/heartbeat":
            C.S.arm.heartbeat()
            self._json({"ok": True})
        elif path == "/precheck":
            name = str(body.get("target") or "zero")
            ok, why = arm.precheck(name)
            self._json({"ok": ok, "why": why, "target": name})
        elif path == "/enter_work":
            try:
                mode = arm.engage()
            except Exception as exc:
                self._json({"error": f"接管失败：{type(exc).__name__}: {exc}"}, 500); return
            ok, why = arm.go_zero()
            self._json({"ok": ok, "mode": mode, "why": why})
        elif path == "/pose":
            ok, why = arm.go_pose(str(body.get("name") or "zero"))
            self._json({"ok": ok, "why": why}, 200 if ok else 409)
        elif path == "/takeover":
            self._json(arm.engage_async())
        elif path == "/release":
            C.S.arm.release()
            self._json({"ok": True})
        elif path == "/reset_arm":
            try:
                C.S.arm.shutdown()
            except Exception:
                pass
            time.sleep(1.0)
            try:
                C.S.arm = C.SimArm() if RUNCFG["sim"] else C.Arm(RUNCFG["config"])
                self._json({"ok": True})
            except Exception as exc:
                self._json({"error": f"{type(exc).__name__}: {exc}"}, 500)
        elif path == "/stop_agent":
            C.S.agent["running"] = False
            self._json({"ok": True})
        else:
            self._json({"error": "not found"}, 404)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="device.local.json")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--sim", action="store_true", help="无硬件预览")
    args = ap.parse_args(argv)

    if not C.API_KEY:
        print("[console] warning: ASTRA_API_KEY 未设置，任务框会失败")

    RUNCFG.update({"config": args.config, "sim": bool(args.sim)})
    if args.sim:
        C.S.arm = C.SimArm()
        print("[console] SIM 模式：无 DDS、无运动", flush=True)
    else:
        C.S.arm = C.Arm(args.config)
        print(f"[console] 已连接（未接管，不会发指令），当前姿态 {np.round(C.S.arm.home[:7], 3).tolist()}", flush=True)

    threading.Thread(target=C.S.arm.watchdog_loop, daemon=True).start()
    srv = Server((args.host, args.port), Handler)
    print(f"[console] 打开 http://127.0.0.1:{args.port}", flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        C.S.arm.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
