"""Browse explicit connections retained in a failed PNR's public .vo file.

The output is a logical connectivity view, never a failed-route list. It uses
the actual device library for pin directions and rejects aliases/unknown pins
on the selected nets. No database/protected implementation is decoded.
"""
import argparse
from array import array
from bisect import bisect_right
from collections import Counter
from datetime import datetime
import html
import json
from pathlib import Path
import re

from audit_global_muxes import bits
from audit_pin_loads import CELLS, PORTS
from audit_routing_topology import primitive_directions

LIBRARY = Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v')


def inspect(build, targets):
    build = Path(build).resolve()
    netlist = build/'impl/pnr/g2_top.vo'
    report_path = build/'impl/pnr/g2_top.rpt.txt'
    text = netlist.read_text(encoding='utf-8')
    assert not re.search(r'^\s*assign\s', text, re.M), 'explicit alias resolution required'
    assert len(re.findall(r'^module\s', text, re.M)) == 1, 'flat physical netlist required'
    directions = primitive_directions(LIBRARY)
    report = report_path.read_text(encoding='utf-8')
    section = report.split('6. Global Clock Signals', 1)[1].split('7. Pinout', 1)[0]
    resources = {line.split('|')[0].strip():line.split('|')[1].strip()
                 for line in section.splitlines()
                 if '| PRIMARY' in line or '| LW ' in line}
    rows = {n:dict(net=n, reported_global=resources.get(n), drivers=[], sinks=[],
                   route_status='unknown', failed_route_identity_verified=False)
            for n in targets}
    offsets = array('I', (m.start() for m in re.finditer('\n', text)))
    scanned = 0
    for match in CELLS.finditer(text):
        kind, raw_name, body = match.groups()
        name = raw_name.lstrip('\\')
        ports = dict(PORTS.findall(body))
        scanned += 1
        for pin, expression in ports.items():
            expanded = bits(expression)
            for position, net in enumerate(expanded):
                if net not in rows:
                    continue
                direction = directions.get(kind, {}).get(pin)
                assert direction in ('input','output','inout'), (name, kind, pin)
                endpoint = dict(cell=name, kind=kind, pin=pin,
                                bit_position_from_msb=position,
                                expression=expression,
                                line=bisect_right(offsets, match.start())+1)
                if direction in ('input','inout'):
                    rows[net]['sinks'].append(endpoint)
                if direction in ('output','inout'):
                    rows[net]['drivers'].append(dict(endpoint, ports=ports))
    for row in rows.values():
        assert row['sinks'] and row['drivers'], ('unresolved selected net', row['net'])
        row['sink_count'] = len(row['sinks'])
        row['sink_pins'] = dict(Counter(x['pin'] for x in row['sinks']))
        row['sink_groups'] = dict(Counter('/'.join(x['cell'].split('/')[:3])
                                          for x in row['sinks']))
    return dict(build=str(build), netlist=str(netlist), report=str(report_path),
                generated_at=datetime.now().astimezone().isoformat(),
                cells_scanned=scanned, nets=list(rows.values()),
                route_status_available=False, congestion_available=False,
                coordinates_available=False,
                scope='Direct public .vo connectivity and reported global assignments only; selected high-load nets are not identified failed nets.')


PAGE = r'''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<title>连接结构审阅</title>
<style>body{font:16px system-ui;margin:32px auto;max-width:1120px;padding:0 20px;color:#203047;background:#f7f9fc}h1{font-size:27px}header,p{line-height:1.65}.notice{background:#fff3da;border-left:4px solid #c58b20;padding:14px 18px}select,input,button{font:inherit;padding:9px;border:1px solid #b7c4d5;border-radius:5px}select{max-width:100%;width:100%}input{width:60%}table{border-collapse:collapse;width:100%;background:white;font-size:13px}th,td{text-align:left;padding:9px;border-bottom:1px solid #dce3ed;word-break:break-all}th{background:#e9eef6}pre{white-space:pre-wrap;word-break:break-all;background:white;padding:16px;font-size:13px}.meta{font-size:13px;color:#52617a}.flow{display:flex;gap:12px;align-items:center;margin:18px 0;flex-wrap:wrap}.box{background:white;border:1px solid #cbd6e6;padding:14px;border-radius:6px;flex:1;min-width:150px}.arrow{font-size:24px}#summary{line-height:1.8}details{margin:15px 0}button{cursor:pointer;margin:12px 6px 12px 0}</style>
<h1>保留下来的连接结构</h1>
<p class="notice"><b>这是连接结构入口，不是失败网络名单。</b>下列网络按实际负载选取；是否属于未布通网络，当前无法确定。图中没有实际线段、坐标或拥塞数据。</p>
<p class="meta">来源：__SOURCE__<br>点击网名选择驱动和负载；输入模块名或引脚筛选全部端点。</p>
<select id="pick"></select><div id="summary"></div>
<div class="flow"><div class="box" id="sourceBox"></div><span class="arrow">→</span><div class="box" id="netBox"></div><span class="arrow">→</span><div class="box" id="sinkBox"></div></div>
<details><summary>源单元的完整端口连接</summary><pre id="drivers"></pre></details>
<details><summary>负载按模块汇总</summary><pre id="groups"></pre></details>
<input id="filter" placeholder="筛选，例如 snapshot、u_nav、CLEAR"><button id="download">导出本网络全部端点</button>
<p id="count" class="meta"></p><table><thead><tr><th>单元</th><th>类型</th><th>引脚</th><th>.vo 行号</th></tr></thead><tbody id="rows"></tbody></table>
<button id="prev">上一页</button><button id="next">下一页</button>
<script id="data" type="application/json">__DATA__</script>
<script>
const data=JSON.parse(document.getElementById('data').textContent),$=id=>document.getElementById(id);let page=0;
for(const [i,n]of data.nets.entries()){const o=document.createElement('option');o.value=i;o.textContent=n.net+'  ·  '+n.sink_count+' 个输入引脚';$('pick').append(o)}
function render(){const n=data.nets[+$('pick').value],q=$('filter').value.toLowerCase();
const hits=n.sinks.filter(s=>(s.cell+' '+s.kind+' '+s.pin).toLowerCase().includes(q));page=Math.min(page,Math.max(0,Math.ceil(hits.length/75)-1));
$('summary').textContent='报告全局资源：'+(n.reported_global||'未列入 PRIMARY/LW 表')+'；实际输入引脚：'+n.sink_count+'；布线状态：未知。';
$('sourceBox').textContent=n.drivers.map(d=>d.kind+' '+d.cell+'.'+d.pin).join(' / ');$('netBox').textContent=n.net;$('sinkBox').textContent=Object.entries(n.sink_pins).map(([k,v])=>k+' × '+v).join('；');
$('drivers').textContent=JSON.stringify(n.drivers,null,2);$('groups').textContent=Object.entries(n.sink_groups).sort((a,b)=>b[1]-a[1]).map(([k,v])=>v+'  '+k).join('\n');
$('rows').replaceChildren();for(const s of hits.slice(page*75,(page+1)*75)){const tr=document.createElement('tr');for(const v of[s.cell,s.kind,s.pin,s.line]){const td=document.createElement('td');td.textContent=v;tr.append(td)}$('rows').append(tr)}
$('count').textContent='匹配 '+hits.length+' / '+n.sinks.length+' 个端点；第 '+(page+1)+' / '+Math.max(1,Math.ceil(hits.length/75))+' 页，每页75条。';$('prev').disabled=page===0;$('next').disabled=(page+1)*75>=hits.length;}
$('pick').onchange=()=>{page=0;render()};$('filter').oninput=()=>{page=0;render()};$('prev').onclick=()=>{page--;render()};$('next').onclick=()=>{page++;render()};
$('download').onclick=()=>{const n=data.nets[+$('pick').value],b=new Blob([JSON.stringify(n,null,2)],{type:'application/json'}),u=URL.createObjectURL(b),a=document.createElement('a');a.href=u;a.download='net_connections.json';a.click();setTimeout(()=>URL.revokeObjectURL(u),1000)};render();
</script></html>'''


def main():
    p=argparse.ArgumentParser()
    p.add_argument('build',type=Path)
    p.add_argument('--net',action='append',required=True)
    p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();result=inspect(a.build,a.net)
    a.out.mkdir(parents=True,exist_ok=False)
    (a.out/'connections.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    encoded=json.dumps(result,ensure_ascii=False).replace('<','\\u003c')
    (a.out/'index.html').write_text(PAGE.replace('__SOURCE__',html.escape(result['netlist']))
                                  .replace('__DATA__',encoded),encoding='utf-8')
    print(json.dumps(dict(cells_scanned=result['cells_scanned'],nets=[
        dict(net=n['net'],sinks=n['sink_count'],global_resource=n['reported_global'])
        for n in result['nets']]),ensure_ascii=False))


if __name__=='__main__':main()
