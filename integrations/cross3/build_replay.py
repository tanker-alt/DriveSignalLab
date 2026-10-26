#!/usr/bin/env python3
"""Create a self-contained schematic replay from actual CARLA trajectories."""
import argparse
import json
from pathlib import Path

TEMPLATE='''<!doctype html><meta charset="utf-8"><title>Cross3 CARLA replay</title>
<style>body{background:#101821;color:#dce8ef;font:16px system-ui;margin:24px}canvas{background:#182a27;border:1px solid #486170;border-radius:12px;width:min(78vh,90vw)}button,input{vertical-align:middle}button{padding:8px 20px}#info{margin:12px 0}small{color:#a2b5c2}</style>
<h2>Cross3 · CARLA 实测轨迹回放</h2><p>蓝色：LMDrive 主车　金色：背景车　白点：原生信号灯位置</p>
<button id="play">播放 / 暂停</button> <input id="slider" type="range" min="0" value="0" style="width:45vw"><div id="info"></div><canvas id="c" width="900" height="900"></canvas>
<p><small>这是道路示意与真实车辆位置回放，不是 CARLA 渲染截图，也不是完整闯红灯认证。信号显示来自每帧实际灯态。</small></p>
<script>const D=__DATA__;const c=document.getElementById('c'),ctx=c.getContext('2d'),slider=document.getElementById('slider');slider.max=D.frames.length-1;
let playing=false;document.getElementById('play').onclick=()=>playing=!playing;slider.oninput=()=>draw(+slider.value);
const scale=2.4, X=x=>450+x*scale,Y=y=>450+y*scale;
const poses={};for(const r of D.vehicles)(poses[r.actor]??=[]).push(r);
const roles=Object.fromEntries(D.summary.vehicles.map(v=>[v.actor_id,v.role]));
function line(x,y,a,b,color,width=1){ctx.strokeStyle=color;ctx.lineWidth=width;ctx.beginPath();ctx.moveTo(X(x),Y(y));ctx.lineTo(X(a),Y(b));ctx.stroke()}
function draw(i){const f=D.frames[i];ctx.clearRect(0,0,900,900);ctx.fillStyle='#36414a';ctx.fillRect(X(-172),Y(-10.5),344*scale,21*scale);ctx.fillRect(X(-10.5),Y(-172),21*scale,344*scale);ctx.fillRect(X(-22),Y(-22),44*scale,44*scale);
for(const sign of [-1,1]){for(const v of [-7,-3.5,3.5,7]){ctx.setLineDash([7,8]);line(sign*22,v,sign*172,v,'#a9b4bd');line(v,sign*22,v,sign*172,'#a9b4bd')}ctx.setLineDash([]);line(sign*22,0,sign*172,0,'#e0bc4f',2);line(0,sign*22,0,sign*172,'#e0bc4f',2)}
for(const [sid,cfg] of Object.entries(D.signals)){const lane=-cfg.incoming_lanes[0],lat=(lane-.5)*3.5;let [x,y]=cfg.road_id===1?[-28,lat]:cfg.road_id===2?[lat,28]:cfg.road_id===3?[28,-lat]:[-lat,-28];ctx.fillStyle=({Red:'#ff5d69',Yellow:'#ffcf5b',Green:'#57e8a7'})[f.actual[sid]]??'#ccc';ctx.beginPath();ctx.arc(X(x),Y(y),5,0,7);ctx.fill()}
for(const [id,rows] of Object.entries(poses)){let r=null;for(const p of rows){if(p.frame>f.frame)break;r=p}if(!r||f.frame>rows.at(-1).frame+10)continue;ctx.fillStyle=roles[id]?.startsWith('hero')?'#5cb6ff':'#f2bc59';ctx.beginPath();ctx.arc(X(r.x),Y(r.y),5,0,7);ctx.fill()}
document.getElementById('info').textContent=`仿真 ${f.time.toFixed(1)} s · ${f.phase??'CLEARANCE'} / ${f.stage} · CARLA frame ${f.frame}`}
setInterval(()=>{if(playing){slider.value=(+slider.value+2)%(+slider.max+1);draw(+slider.value)}},60);draw(0);</script>'''


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('episode',type=Path)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    upstream=Path(__file__).resolve().parents[1]/'carla_handoff/upstream_20261023_v4'
    manifest=json.loads((upstream/'map_dataset/junction_toolkit/maps/cross3/manifest.json').read_text())
    frames=[json.loads(l) for l in (a.episode/'signals.jsonl').open()]
    frames=frames[::5]+frames[-1:]
    data=dict(signals=manifest['signals'],frames=frames,
        vehicles=[json.loads(l) for l in (a.episode/'vehicles.jsonl').open()],
        summary=json.loads((a.episode/'summary.json').read_text()))
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(TEMPLATE.replace('__DATA__',json.dumps(data).replace('<','\\u003c')))
    print(a.output)


if __name__=='__main__':main()
