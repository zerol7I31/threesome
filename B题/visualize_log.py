# -*- coding: utf-8 -*-
"""
行为日志可视化 —— 读取机器狗指令日志（logs/*.jsonl），生成单文件交互式 HTML。

用法：
    python visualize_log.py                         # 可视化 logs/ 下最新的日志
    python visualize_log.py logs/live-xxx.jsonl     # 指定日志
    python visualize_log.py logs/mock-s1000-xxx.jsonl
        若日志旁边存在 同名-truth.json（mock 局自动保存的真值侧车），自动叠加真值图层。

输出：与日志同目录、同主文件名的 .html。浏览器打开后可：
    滚轮缩放 / 拖拽平移；悬停任意测站看该站全部指令明细；
    悬停清除点显示 20 m 清除半径参考圆；按频道过滤；图层开关；
    底部时间轴回放整局过程（虚拟时间轴）。

也被 p3_robot.py 自动调用：mock --viz（每局）与 live（每局结束默认）都会生成。
"""
import argparse
import glob
import json
import math
import os
import re
import time

TARGET_R = 1800.0


# ----------------------------------------------------------------------
# 日志解析（只依赖日志本身，不需要真值）
# ----------------------------------------------------------------------
def load_log(path):
    acts = []
    prev_t = 0.0
    prev_pos = (0.0, 0.0)
    with open(path, encoding="utf-8") as f:
        for ln in f:
            ln = ln.strip()
            if not ln:
                continue
            try:
                r = json.loads(ln)
            except ValueError:
                continue
            pth = r.get("path")
            p = r.get("payload") or {}
            resp = r.get("resp") or {}
            ok = resp.get("accepted") is True
            pos = p.get("position")
            pos = [float(pos["x"]), float(pos["y"])] if pos else None
            rec = {"i": len(acts), "path": pth, "ok": ok,
                   "req": p.get("request_id"), "ch": p.get("channel")}
            if ok:
                t = float(resp.get("virtual_time_s", 0.0))
                rec.update(t=t, dt=max(0.0, t - prev_t), pos=pos,
                           prev=[prev_pos[0], prev_pos[1]],
                           move=(math.dist(prev_pos, pos) if pos else 0.0))
                if pth == "/measure":
                    rec["result"] = resp.get("measure_result")
                    rec["svd"] = resp.get("svd_deg")
                elif pth == "/clear":
                    rec["result"] = resp.get("clear_result")
                elif pth == "/exit":
                    rec["result"] = resp.get("exit_reason")
                if pos:
                    prev_pos = (pos[0], pos[1])
                prev_t = t
            else:
                rec["t"] = None
                rec["http"] = resp.get("_http") or resp.get("http_status")
            acts.append(rec)
    return acts


def build_stats(acts):
    cam = 1          # 测向机初始频道（附件2：初始为 1）
    st = {"T": 0.0, "measures": 0, "clears": 0, "hits": 0,
          "dirs": 0, "nosig": 0, "nears": 0, "rejected": 0,
          "move_m": 0.0, "move_s": 0.0, "detect_s": 0.0,
          "switch_s": 0.0, "clear_s": 0.0}
    for a in acts:
        if not a["ok"]:
            st["rejected"] += 1
            continue
        st["T"] = max(st["T"], a["t"] or 0.0)
        st["move_m"] += a.get("move") or 0.0
        if a["path"] == "/measure":
            st["measures"] += 1
            st["detect_s"] += 5.0
            if a["ch"] != cam:
                st["switch_s"] += 1.0
            cam = a["ch"]
            r = a.get("result")
            if r == "direction":
                st["dirs"] += 1
            elif r == "no_signal":
                st["nosig"] += 1
            elif r == "near":
                st["nears"] += 1
        elif a["path"] == "/clear":
            st["clears"] += 1
            st["clear_s"] += 5.0 if a.get("result") == "success" else 3.0
            if a.get("result") == "success":
                st["hits"] += 1
    st["move_s"] = st["move_m"] / 5.0
    return st


def build_stations(acts):
    """连续同位置（<1 m）的 /measure 聚成"静止测站"，供悬停聚合显示。"""
    stations, cur = [], None
    for a in acts:
        if not (a["ok"] and a["path"] == "/measure" and a["pos"]):
            cur = None
            continue
        if cur and math.dist(cur["pos"], a["pos"]) < 1.0:
            cur["items"].append(a["i"])
        else:
            cur = {"pos": list(a["pos"]), "items": [a["i"]]}
            stations.append(cur)
    for s in stations:
        s["n"] = len(s["items"])
    return stations


def build_channels(acts):
    chs = {}
    for a in acts:
        if not a["ok"] or a["ch"] is None:
            continue
        c = chs.setdefault(str(a["ch"]), {"measures": 0, "dirs": 0, "nosig": 0,
                                          "nears": 0, "clears": 0, "hits": 0,
                                          "clear_t": None})
        if a["path"] == "/measure":
            c["measures"] += 1
            r = a.get("result")
            if r == "direction":
                c["dirs"] += 1
            elif r == "no_signal":
                c["nosig"] += 1
            elif r == "near":
                c["nears"] += 1
        elif a["path"] == "/clear":
            c["clears"] += 1
            if a.get("result") == "success":
                c["hits"] += 1
                c["clear_t"] = a["t"]
    return chs


def load_truth(path):
    """path 可以是真值 json 本身，也可以是日志路径（自动推导 同名-truth.json）。"""
    cand = path
    if cand.endswith(".jsonl"):
        cand = re.sub(r"\.jsonl$", "-truth.json", cand)
    if not os.path.exists(cand):
        return None
    try:
        with open(cand, encoding="utf-8") as f:
            d = json.load(f)
    except Exception:
        return None
    srcs = {}
    for k, s in (d.get("sources") or {}).items():
        try:
            srcs[str(int(k))] = {"pos": [float(s["pos"][0]), float(s["pos"][1])],
                                 "R": float(s["R"]),
                                 "dir": (None if s.get("dir") is None else float(s["dir"]))}
        except Exception:
            continue
    desc = "seed %s · %s%s" % (d.get("seed"), d.get("dist"),
                               " · 含定向源" if d.get("directional") else " · 全向源")
    return {"sources": srcs, "desc": desc}


def build_data(log_path, acts, truth):
    return {
        "meta": {"log": os.path.basename(log_path),
                 "generated": time.strftime("%Y-%m-%d %H:%M:%S"),
                 "truth_desc": (truth or {}).get("desc")},
        "stats": build_stats(acts),
        "actions": acts,
        "stations": build_stations(acts),
        "channels": build_channels(acts),
        "truth": ({"sources": truth["sources"]} if truth else None),
    }


# ----------------------------------------------------------------------
# HTML 模板（__TITLE__ / __DATA_JSON__ 两个占位符；其余原样输出）
# ----------------------------------------------------------------------
TEMPLATE = r"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<title>__TITLE__</title>
<style>
:root{--bg:#f3f5f7;--card:#ffffff;--ink:#1f2329;--sub:#6b7280;--line:#e3e6ea;--accent:#2563eb}
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:"Segoe UI","Microsoft YaHei",system-ui,sans-serif;background:var(--bg);color:var(--ink);font-size:14px}
header{padding:14px 24px;background:#111827;color:#fff}
header h1{font-size:17px;font-weight:600}
header .sub{color:#9ca3af;font-size:12px;margin-top:4px}
.wrap{display:flex;gap:14px;padding:14px 24px;align-items:flex-start}
.left{flex:1;min-width:0}
.cards{display:flex;flex-wrap:wrap;gap:10px;margin-bottom:12px}
.card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:10px 14px;min-width:128px}
.card .v{font-size:19px;font-weight:700}
.card .k{font-size:12px;color:var(--sub);margin-top:3px}
.panel{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:12px 14px;margin-bottom:12px}
.ptitle{font-size:13px;font-weight:600;margin-bottom:8px}
.panel .k{font-size:12px;color:var(--sub);margin-bottom:6px}
#tbar{display:flex;height:18px;border-radius:6px;overflow:hidden;background:#eef0f2}
#tbar>div{height:100%}
#tbarlegend{margin-top:8px;font-size:12px;color:var(--sub);display:flex;flex-wrap:wrap;gap:12px}
.lg i{display:inline-block;width:10px;height:10px;border-radius:2px;margin-right:5px}
#mapbox{position:relative;background:#fff;border:1px solid var(--line);border-radius:10px;overflow:hidden}
#map{display:block;width:100%;aspect-ratio:1/1;cursor:grab}
#map:active{cursor:grabbing}
#maphint{position:absolute;right:10px;bottom:8px;font-size:11px;color:#9ca3af;background:rgba(255,255,255,.85);padding:2px 8px;border-radius:6px;pointer-events:none}
#tooltip{position:absolute;display:none;pointer-events:none;background:rgba(17,24,39,.94);color:#fff;font-size:12px;line-height:1.6;padding:8px 10px;border-radius:8px;max-width:360px;max-height:320px;overflow:hidden;z-index:5;white-space:pre-line;font-family:Consolas,monospace}
#replay{display:flex;align-items:center;gap:12px}
#replay button{background:var(--accent);color:#fff;border:0;border-radius:8px;padding:7px 16px;font-size:13px;cursor:pointer;flex:none}
#replay input[type=range]{flex:1}
#clock{font-size:12px;color:var(--sub);flex:none;font-family:Consolas,monospace;white-space:nowrap}
.side{width:350px;flex:none}
.side label{display:block;font-size:13px;padding:3px 0;cursor:pointer}
.side select{width:100%;padding:6px 8px;border:1px solid var(--line);border-radius:8px;font-size:13px;margin-top:4px}
table{width:100%;border-collapse:collapse;font-size:12px}
th,td{padding:5px 6px;text-align:left;border-bottom:1px solid #f0f1f3;white-space:nowrap}
th{color:var(--sub);font-weight:600}
tr[data-ch]{cursor:pointer}
tr[data-ch]:hover{background:#f5f8ff}
.chip{display:inline-block;width:10px;height:10px;border-radius:3px;margin-right:5px}
.ok{color:#15803d;font-weight:600}
.warn{color:#b45309;font-weight:600}
.mut{color:#9ca3af}
.rej{font-size:12px;color:#b91c1c;padding:2px 0;font-family:Consolas,monospace}
.legend2{font-size:12px;color:var(--sub);line-height:1.9}
</style>
</head>
<body>
<header>
  <h1>机器狗行为可视化 · __TITLE__</h1>
  <div class="sub" id="subtitle"></div>
</header>
<div class="wrap">
  <div class="left">
    <div class="cards" id="cards"></div>
    <div class="panel">
      <div class="k">虚拟时间分解（占虚拟总时间比例）</div>
      <div id="tbar"></div>
      <div id="tbarlegend"></div>
    </div>
    <div id="mapbox">
      <svg id="map" viewBox="-1900 -1900 3800 3800"></svg>
      <div id="tooltip"></div>
      <div id="maphint">滚轮缩放 · 拖拽平移 · 悬停看详情 ｜ 坐标：东 +x，北 +y，单位 m；虚线圆环 r=600/1200，实线圆 r=1800</div>
    </div>
    <div class="panel" id="replay">
      <button id="play">▶ 播放</button>
      <input type="range" id="slider" min="0" max="1" step="any" value="0">
      <span id="clock"></span>
    </div>
  </div>
  <aside class="side">
    <div class="panel">
      <div class="ptitle">图层</div>
      <label><input type="checkbox" data-layer="path" checked> 行驶路径</label>
      <label><input type="checkbox" data-layer="ray" checked> 示向度方位线</label>
      <label><input type="checkbox" data-layer="wedge" checked> ±1° 误差楔形</label>
      <label><input type="checkbox" data-layer="mark" checked> 测站 / 清除点</label>
      <label id="truthcb"><input type="checkbox" data-layer="truth" checked> 真值叠加（mock）</label>
      <label><input type="checkbox" data-layer="grid" checked> 网格 / 边界</label>
      <div class="ptitle" style="margin-top:10px">频道过滤</div>
      <select id="chsel"><option value="all">全部频道</option></select>
    </div>
    <div class="panel">
      <div class="ptitle">图例</div>
      <div class="legend2">
        ● 彩色圆点 = 测得示向度（颜色=频道）<br>
        ● 灰点 = no_signal　● 橙点 = near（距离≤5 m）<br>
        ★ 星 = 清除成功　✕ 红叉 = 未命中（悬停显示 20 m 清除半径）<br>
        ◆ 菱形 = 真值干扰源：虚线圆=有效接收半径 R，扇形=定向 180° 覆盖区<br>
        — 方位线按 1500 m 或场地边界截断；浅色楔形为 ±1° 误差范围
      </div>
    </div>
    <div class="panel">
      <div class="ptitle">逐频道明细（点击行 = 过滤该频道）</div>
      <table id="chtable"></table>
    </div>
    <div class="panel" id="rejpanel" style="display:none"></div>
  </aside>
</div>
<script>const DATA = __DATA_JSON__;</script>
<script>
(function(){
'use strict';
const NS='http://www.w3.org/2000/svg';
const D=DATA, A=D.actions, ST=D.stats, R=1800;
const $=id=>document.getElementById(id);
const svg=$('map');
function el(n,at,p){const e=document.createElementNS(NS,n);for(const k in at)e.setAttribute(k,at[k]);if(p)p.appendChild(e);return e;}
const CHC=c=>'hsl('+((c*137)%360)+',70%,42%)';
const fmt=(n,d)=>(n==null||isNaN(n))?'—':Number(n).toFixed(d==null?1:d);
const fmt0=n=>fmt(n,0);

$('subtitle').textContent='日志 '+D.meta.log+' ｜ 生成于 '+D.meta.generated+' ｜ 指令 '+A.length+' 条（被拒 '+ST.rejected+'）'
  +(D.truth?(' ｜ 真值：'+D.meta.truth_desc):' ｜ 无真值（真实模拟器日志）');

/* ---------- 顶部统计卡片 ---------- */
const cards=[
 ['虚拟总时间', fmt(ST.T)+' s'],
 ['清除成功', D.truth? ST.hits+' / '+Object.keys(D.truth.sources).length : String(ST.hits)],
 ['平均定位清除时间', ST.hits? fmt(ST.T/ST.hits)+' s/个':'—'],
 ['检测 /measure', ST.measures+' 次（有方向 '+ST.dirs+'）'],
 ['清除 /clear', ST.clears+' 次（未命中 '+(ST.clears-ST.hits)+'）'],
 ['移动距离', fmt(ST.move_m/1000,2)+' km'],
 ['被拒指令', String(ST.rejected)],
];
$('cards').innerHTML=cards.map(c=>'<div class="card"><div class="v">'+c[1]+'</div><div class="k">'+c[0]+'</div></div>').join('');

/* ---------- 时间分解条 ---------- */
(function(){
 const segs=[['移动',ST.move_s,'#4C78A8'],['检测',ST.detect_s,'#F58518'],['切换频道',ST.switch_s,'#9D755D'],['清除动作',ST.clear_s,'#54A24B']];
 const tot=segs.reduce((s,x)=>s+x[1],0)||1;
 $('tbar').innerHTML=segs.map(s=>'<div style="width:'+(100*s[1]/tot)+'%;background:'+s[2]+'" title="'+s[0]+' '+fmt(s[1])+' s"></div>').join('');
 $('tbarlegend').innerHTML=segs.map(s=>'<span class="lg"><i style="background:'+s[2]+'"></i>'+s[0]+' '+fmt(s[1])+' s（'+fmt(100*s[1]/(ST.T||tot))+'%）</span>').join('')
   +'<span class="lg">分解合计 '+fmt(tot)+' s ｜ 虚拟总时间 '+fmt(ST.T)+' s</span>';
})();

/* ---------- 视图状态（缩放/平移） ---------- */
let VW=3800,VCX=0,VCY=0;
const mkScale=()=>VW/950;
function updateView(){
 svg.setAttribute('viewBox',(VCX-VW/2)+' '+(VCY-VW/2)+' '+VW+' '+VW);
 svg.querySelectorAll('g.mk').forEach(g=>{g.setAttribute('transform','translate('+g._x+' '+g._y+') scale('+mkScale()+')');});
}
svg.addEventListener('wheel',function(e){
 e.preventDefault();
 const r=svg.getBoundingClientRect();
 const fx=(e.clientX-r.left)/r.width, fy=(e.clientY-r.top)/r.height;
 const wx=VCX-VW/2+fx*VW, wy=VCY-VW/2+fy*VW;
 VW=Math.min(4200,Math.max(60,VW*(e.deltaY>0?1.15:1/1.15)));
 VCX=wx-(fx-0.5)*VW; VCY=wy-(fy-0.5)*VW;
 updateView();
},{passive:false});
let drag=null;
svg.addEventListener('mousedown',e=>{drag={x:e.clientX,y:e.clientY,cx:VCX,cy:VCY};});
window.addEventListener('mousemove',e=>{
 if(!drag)return;
 const r=svg.getBoundingClientRect();
 VCX=drag.cx-(e.clientX-drag.x)/r.width*VW;
 VCY=drag.cy-(e.clientY-drag.y)/r.height*VW;
 updateView();
});
window.addEventListener('mouseup',()=>{drag=null;});

/* ---------- 图层骨架 ---------- */
const world=el('g',{transform:'scale(1,-1)'},svg);   // y 轴翻转为"北向上"
const gGrid=el('g',{},world), gTruth=el('g',{},world), gWedge=el('g',{},world),
      gRay=el('g',{},world), gPath=el('g',{},world), gMark=el('g',{},world);
el('circle',{cx:0,cy:0,r:R,fill:'#fbfdfc',stroke:'#1f2937','stroke-width':2,'vector-effect':'non-scaling-stroke'},gGrid);
[600,1200].forEach(rr=>el('circle',{cx:0,cy:0,r:rr,fill:'none',stroke:'#e5e7eb','stroke-width':1.2,'vector-effect':'non-scaling-stroke','stroke-dasharray':'4 5'},gGrid));
el('line',{x1:-R,y1:0,x2:R,y2:0,stroke:'#e5e7eb','stroke-width':1.2,'vector-effect':'non-scaling-stroke'},gGrid);
el('line',{x1:0,y1:-R,x2:0,y2:R,stroke:'#e5e7eb','stroke-width':1.2,'vector-effect':'non-scaling-stroke'},gGrid);
el('circle',{cx:0,cy:0,r:14,fill:'#111827'},gGrid);

/* ---------- 可见性注册表（时间轴 + 频道过滤统一裁决） ---------- */
const registry=[];
function reg(e,t,ch){registry.push({e:e,t:(t==null?null:+t),ch:(ch==null?null:+ch)});}
let curT=ST.T||0, selCh='all';
const layers={grid:true,truth:true,wedge:true,ray:true,path:true,mark:true};
function applyVis(){
 const gg={grid:gGrid,truth:gTruth,wedge:gWedge,ray:gRay,path:gPath,mark:gMark};
 for(const k in gg) gg[k].style.display=layers[k]?'':'none';
 registry.forEach(o=>{
   const tOK=o.t==null||o.t<=curT+1e-9;
   const cOK=selCh=='all'||o.ch==null||o.ch==selCh;
   o.e.style.display=tOK&&cOK?'':'none';
 });
 updateClock();
}
function updateClock(){
 let done=0,hits=0;
 A.forEach(a=>{if(a.ok&&a.t!=null&&a.t<=curT+1e-9){done++;if(a.path=='/clear'&&a.result=='success')hits++;}});
 $('clock').textContent='t = '+fmt(curT)+' s ｜ 已执行 '+done+'/'+A.length+' 条 ｜ 已清除 '+hits+' 个';
}

/* ---------- 测站索引（悬停聚合） ---------- */
const stByAction={};
D.stations.forEach(s=>s.items.forEach(i=>{stByAction[i]=s;}));
function stationTip(s){
 const lines=['测站 ('+fmt0(s.pos[0])+', '+fmt0(s.pos[1])+') · 连续检测 '+s.items.length+' 次'];
 s.items.forEach(i=>{const a=A[i];
   lines.push('#'+(i+1)+' ch'+a.ch+' '+a.result+(a.svd!=null?'  svd='+fmt(a.svd,2)+'°':'')+'  t='+fmt(a.t)+'s');
 });
 return lines.join('\n');
}

/* ---------- 标记（缩放时保持屏幕尺寸不变） ---------- */
const starPts=(function(){const p=[];for(let i=0;i<10;i++){const r=i%2?5.5:13,a=-Math.PI/2+i*Math.PI/5;p.push((r*Math.cos(a)).toFixed(2)+','+(r*Math.sin(a)).toFixed(2));}return p.join(' ');})();
function marker(x,y,ch,t,tip,parent){
 const g=el('g',{class:'mk'},parent||gMark);
 g._x=x; g._y=y; g._tip=tip;
 el('circle',{r:26,fill:'#000','fill-opacity':0},g);
 reg(g,t,ch);
 return g;
}
function rayLen(pos,th){
 const a=th*Math.PI/180,ux=Math.cos(a),uy=Math.sin(a),x=pos[0],y=pos[1];
 const b=x*ux+y*uy, disc=b*b-(x*x+y*y-R*R);
 const t=disc>=0?(-b+Math.sqrt(disc)):1500;
 return Math.max(1,Math.min(1500,t));
}

/* ---------- 真值图层（仅 mock 局有侧车文件时） ---------- */
if(D.truth){
 Object.keys(D.truth.sources).forEach(k=>{
   const s=D.truth.sources[k], c=CHC(+k), x=s.pos[0], y=s.pos[1];
   if(s.dir!=null){
     const pts=[[x,y]];
     for(let a=s.dir-90;a<=s.dir+90+1e-6;a+=8){const r=a*Math.PI/180;pts.push([x+s.R*Math.cos(r),y+s.R*Math.sin(r)]);}
     reg(el('polygon',{points:pts.map(p=>p[0].toFixed(1)+','+p[1].toFixed(1)).join(' '),fill:c,'fill-opacity':.06,stroke:'none'},gTruth),null,+k);
     reg(el('line',{x1:x,y1:y,x2:x+s.R*0.35*Math.cos(s.dir*Math.PI/180),y2:y+s.R*0.35*Math.sin(s.dir*Math.PI/180),stroke:c,'stroke-width':2,'vector-effect':'non-scaling-stroke','stroke-opacity':.75},gTruth),null,+k);
   }
   reg(el('circle',{cx:x,cy:y,r:s.R,fill:'none',stroke:c,'stroke-width':1.3,'vector-effect':'non-scaling-stroke','stroke-dasharray':'8 6','stroke-opacity':.45},gTruth),null,+k);
   const tip='真值 ch'+k+'\n位置 ('+fmt0(x)+', '+fmt0(y)+')\n'+(s.dir!=null?'定向源，发射方向 '+fmt(s.dir,1)+'°（覆盖 ±90°）':'全向源')+'\n有效接收半径 R = '+fmt0(s.R)+' m';
   const g=marker(x,y,+k,null,tip,gTruth);
   el('polygon',{points:'0,-13 13,0 0,13 -13,0',fill:c,stroke:'#111827','stroke-width':2,'vector-effect':'non-scaling-stroke'},g);
 });
}

/* ---------- 路径 + 方位线/楔形 + 测站/清除标记 ---------- */
A.forEach(a=>{
 if(!a.ok) return;
 if(a.pos&&a.prev&&a.move>0.05){
   reg(el('line',{x1:a.prev[0],y1:a.prev[1],x2:a.pos[0],y2:a.pos[1],stroke:'#2563eb','stroke-width':2.2,'vector-effect':'non-scaling-stroke','stroke-opacity':.7},gPath),a.t,null);
 }
 if(!a.pos) return;
 if(a.path=='/measure'){
   let hl=null;
   if(a.result=='direction'&&a.svd!=null){
     const L=rayLen(a.pos,a.svd), rd=a.svd*Math.PI/180;
     const r1=(a.svd-1)*Math.PI/180, r2=(a.svd+1)*Math.PI/180;
     const w=el('polygon',{points:a.pos[0]+','+a.pos[1]+' '+(a.pos[0]+L*Math.cos(r1)).toFixed(1)+','+(a.pos[1]+L*Math.sin(r1)).toFixed(1)+' '+(a.pos[0]+L*Math.cos(r2)).toFixed(1)+','+(a.pos[1]+L*Math.sin(r2)).toFixed(1),fill:CHC(a.ch),'fill-opacity':.07,stroke:'none'},gWedge);
     const l=el('line',{x1:a.pos[0],y1:a.pos[1],x2:a.pos[0]+L*Math.cos(rd),y2:a.pos[1]+L*Math.sin(rd),stroke:CHC(a.ch),'stroke-width':1.6,'vector-effect':'non-scaling-stroke','stroke-opacity':.5},gRay);
     reg(w,a.t,a.ch); reg(l,a.t,a.ch);
     hl=on=>{w.setAttribute('fill-opacity',on?.3:.07);l.setAttribute('stroke-opacity',on?.95:.5);l.setAttribute('stroke-width',on?3:1.6);};
   }
   const st=stByAction[a.i];
   const g=marker(a.pos[0],a.pos[1],a.ch,a.t,st?stationTip(st):('ch'+a.ch+' '+a.result));
   if(hl) g._hl=hl;
   const fill=a.result=='direction'?CHC(a.ch):(a.result=='near'?'#f59e0b':'#9ca3af');
   el('circle',{r:a.result=='near'?11:8,fill:fill,stroke:'#fff','stroke-width':1.6,'vector-effect':'non-scaling-stroke'},g);
 }else if(a.path=='/clear'){
   const ok=a.result=='success';
   const tip='#'+(a.i+1)+' /clear ch'+a.ch+'\n位置 ('+fmt0(a.pos[0])+', '+fmt0(a.pos[1])+')  t='+fmt(a.t)+' s\n结果 '+(ok?'success（已清除）':'no_target_in_range（未命中：20 m 内无该频道目标）');
   const g=marker(a.pos[0],a.pos[1],a.ch,a.t,tip);
   if(ok) el('polygon',{points:starPts,fill:CHC(a.ch),stroke:'#111827','stroke-width':2,'vector-effect':'non-scaling-stroke'},g);
   else el('path',{d:'M -9 -9 L 9 9 M -9 9 L 9 -9',stroke:'#dc2626','stroke-width':4,'vector-effect':'non-scaling-stroke','stroke-linecap':'round'},g);
   g._ring=el('circle',{cx:a.pos[0],cy:a.pos[1],r:20,fill:'none',stroke:ok?CHC(a.ch):'#dc2626','stroke-width':2,'vector-effect':'non-scaling-stroke','stroke-dasharray':'5 4',display:'none'},gMark);
 }
});

/* ---------- 悬停 tooltip / 清除半径 / 方位高亮 ---------- */
const tip=$('tooltip'), mapbox=$('mapbox');
let hoverG=null;
function clearHover(){
 if(hoverG){if(hoverG._ring)hoverG._ring.style.display='none';if(hoverG._hl)hoverG._hl(false);hoverG=null;}
}
svg.addEventListener('mousemove',e=>{
 const g=(e.target&&e.target.closest)?e.target.closest('g.mk'):null;
 if(g!=hoverG){clearHover();hoverG=g;if(g){if(g._ring)g._ring.style.display='';if(g._hl)g._hl(true);}}
 if(g&&g._tip&&g.style.display!='none'){
   tip.style.display='block'; tip.textContent=g._tip;
   const r=mapbox.getBoundingClientRect();
   tip.style.left=Math.min(e.clientX-r.left+14,Math.max(0,r.width-370))+'px';
   tip.style.top=Math.min(e.clientY-r.top+14,Math.max(0,r.height-330))+'px';
 }else tip.style.display='none';
});
svg.addEventListener('mouseleave',()=>{tip.style.display='none';clearHover();});

/* ---------- 图层开关 + 频道过滤 ---------- */
document.querySelectorAll('input[data-layer]').forEach(cb=>cb.addEventListener('change',()=>{layers[cb.dataset.layer]=cb.checked;applyVis();}));
if(!D.truth){$('truthcb').style.display='none';layers.truth=false;}
const sel=$('chsel');
Object.keys(D.channels).map(Number).sort((a,b)=>a-b).forEach(c=>{
 const o=document.createElement('option');
 o.value=c; o.textContent='频道 '+c+(D.channels[c].hits?'（已清除）':'');
 sel.appendChild(o);
});
sel.addEventListener('change',()=>{selCh=sel.value=='all'?'all':+sel.value;applyVis();});

/* ---------- 逐频道明细表 ---------- */
(function(){
 const hasT=!!D.truth;
 let h='<tr><th>频道</th><th>检测</th><th>方向</th><th>无信号</th><th>清除(中/试)</th><th>状态</th>'+(hasT?'<th>真值位置</th><th>类型 / R</th>':'')+'</tr>';
 Object.keys(D.channels).map(Number).sort((a,b)=>a-b).forEach(c=>{
   const d=D.channels[c];
   const st=d.hits?'<span class="ok">已清除@'+fmt0(d.clear_t)+'s</span>':(d.dirs?'<span class="warn">有方位未清除</span>':(d.measures?'<span class="mut">仅 no_signal</span>':'<span class="mut">未测</span>'));
   let tc='';
   if(hasT){const s=D.truth.sources[c];
     tc=s?'<td>('+fmt0(s.pos[0])+','+fmt0(s.pos[1])+')</td><td>'+(s.dir!=null?'定向 '+fmt0(s.dir)+'°':'全向')+' / '+fmt0(s.R)+'</td>':'<td>—</td><td>—</td>';}
   h+='<tr data-ch="'+c+'"><td><i class="chip" style="background:'+CHC(c)+'"></i>ch'+c+'</td><td>'+d.measures+'</td><td>'+d.dirs+'</td><td>'+d.nosig+'</td><td>'+d.hits+'/'+d.clears+'</td><td>'+st+'</td>'+tc+'</tr>';
 });
 $('chtable').innerHTML=h;
 $('chtable').querySelectorAll('tr[data-ch]').forEach(tr=>tr.addEventListener('click',()=>{sel.value=tr.dataset.ch;sel.dispatchEvent(new Event('change'));}));
})();

/* ---------- 被拒指令 ---------- */
if(ST.rejected){
 const items=A.filter(a=>!a.ok);
 $('rejpanel').style.display='';
 $('rejpanel').innerHTML='<div class="ptitle">被拒指令（'+items.length+'）</div>'
   +items.map(a=>'<div class="rej">#'+(a.i+1)+' '+a.path+' req='+(a.req||'?')+(a.http?' http='+a.http:'')+'</div>').join('');
}

/* ---------- 时间轴 / 回放 ---------- */
const slider=$('slider');
slider.max=ST.T||1; slider.step=(ST.T||1)/2000; slider.value=curT;
slider.addEventListener('input',()=>{curT=+slider.value;applyVis();});
let playing=null;
$('play').addEventListener('click',()=>{
 if(playing){clearInterval(playing);playing=null;$('play').textContent='▶ 播放';return;}
 if(curT>=ST.T) curT=0;
 $('play').textContent='⏸ 暂停';
 playing=setInterval(()=>{
   curT+=(ST.T||1)/750;
   if(curT>=ST.T){curT=ST.T;clearInterval(playing);playing=null;$('play').textContent='▶ 播放';}
   slider.value=curT; applyVis();
 },40);
});

updateView();
applyVis();
})();
</script>
</body>
</html>
"""


# ----------------------------------------------------------------------
# 渲染入口
# ----------------------------------------------------------------------
def render_html(data, title):
    payload = json.dumps(data, ensure_ascii=False, separators=(",", ":"))
    payload = payload.replace("</", "<\\/")          # 防止数据里出现 </script>
    return (TEMPLATE
            .replace("__DATA_JSON__", payload)
            .replace("__TITLE__", title))


def render_file(log_path, truth=None, out_path=None):
    """读日志 → 生成 HTML，返回输出路径。truth 可传 None（自动找侧车）、路径或 dict。"""
    acts = load_log(log_path)
    if truth is None:
        truth = load_truth(log_path)
    elif isinstance(truth, str):
        truth = load_truth(truth)
    data = build_data(log_path, acts, truth)
    html = render_html(data, os.path.basename(log_path))
    if not out_path:
        out_path = re.sub(r"\.jsonl$", "", log_path) + ".html"
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(html)
    return out_path


def main():
    ap = argparse.ArgumentParser(description="把 logs/*.jsonl 行为日志渲染成单文件交互式 HTML")
    ap.add_argument("log", nargs="?", help="日志路径（缺省取 logs/ 下最新修改的 .jsonl）")
    ap.add_argument("-o", "--out", help="输出 html 路径（缺省与日志同名同目录）")
    ap.add_argument("--truth", help="真值侧车 json（缺省自动找 同名-truth.json）")
    args = ap.parse_args()
    log = args.log
    if not log:
        cands = glob.glob(os.path.join("logs", "*.jsonl"))
        if not cands:
            print("logs/ 下没有 .jsonl 日志")
            return
        log = max(cands, key=os.path.getmtime)
        print("未指定日志，自动取最新：%s" % log)
    out = render_file(log, truth=args.truth, out_path=args.out)
    n = sum(1 for _ in open(log, encoding="utf-8"))
    print("已生成：%s（指令 %d 条，可用浏览器直接打开）" % (out, n))


if __name__ == "__main__":
    main()
