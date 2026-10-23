"""Package a cancelled frozen PNR input and bounded portable runner locally.

No tool run, upload, source minimization or hash validation is performed.
"""
import argparse,json,re,shutil,zipfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
RUNNER='''import argparse,json
from pathlib import Path
from bounded_tool import run_tool
from collect_pnr_result import finalize_gate
p=argparse.ArgumentParser()
p.add_argument('--tool',type=Path,required=True,help='absolute path to gw_sh.exe')
p.add_argument('--minutes',type=float,default=90)
p.add_argument('--silent-minutes',type=float,default=75)
p.add_argument('--telemetry',action='store_true',help='requires psutil on Windows')
a=p.parse_args();root=Path(__file__).resolve().parent
if not(0<a.minutes<=90 and 0<a.silent_minutes<=75):p.error('budgets must be positive, at most90/75 minutes')
if(root/'impl').exists():p.error('implementation exists; preserve it and extract a fresh copy before another run')
r=run_tool([str(a.tool.resolve()),'run.tcl'],root,max_runtime_s=a.minutes*60,max_no_progress_s=a.silent_minutes*60,capture_system_telemetry=a.telemetry)
finalize_gate(root)
raise SystemExit(r['tool_exit'] or r['cancelled'])
'''


def main():
    ap=argparse.ArgumentParser();ap.add_argument('build',type=Path);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
    source=a.build.resolve();out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
    manifest=json.loads((source/'manifest.json').read_text())
    assert manifest['frozen_netlist'] and manifest['cdc_reference_audit']
    for name in ('g2_top.vg','g2.cst','g2.sdc','cdc_reference_audit.json','effective_options.tcl'):
        shutil.copyfile(source/name,out/name)
    lines=(source/'run.tcl').read_text().splitlines();portable=['set repro_root [file dirname [file normalize [info script]]]','cd $repro_root']
    types={'netlist':'g2_top.vg','cst':'g2.cst','sdc':'g2.sdc'}
    for line in lines:
        match=re.match(r'add_file -type (netlist|cst|sdc) ',line)
        if match:line='add_file -type '+match[1]+' [file join $repro_root "'+types[match[1]]+'"]'
        portable.append(line)
    (out/'run.tcl').write_text('\n'.join(portable)+'\n',encoding='utf-8',newline='\n')
    original_manifest=manifest.copy();manifest.update(diagnostic_only=True,programmed=False,
        scope='local reproducibility package only; no G2/deployment acceptance')
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    (out/'original_manifest.json').write_text(json.dumps(original_manifest,indent=2),encoding='utf-8')
    for name in ('bounded_tool.py','collect_pnr_result.py','audit_cdc_first_stages.py','audit_pin_loads.py'):
        shutil.copyfile(ROOT/'integration/g2'/name,out/name)
    (out/'run_bounded.py').write_text(RUNNER,encoding='utf-8',newline='\n')
    previous=out/'previous_run';previous.mkdir()
    for relative in ('tool_runtime.json','resource_cancellation.json','g2_gate.json','gw_sh.log','impl/pnr/g2_top.log','impl/pnr/cmd.do','impl/pnr/device.cfg'):
        file=source/relative
        if file.exists():shutil.copyfile(file,previous/relative.replace('/','_'))
    readme='''# 冻结布线复现包

此包只包含复现所需的综合网表、引脚/时钟约束和工具选项，以及旧取消记录。它尚未按逻辑删减成最小故障电路，不能称为已定位的厂商bug。没有上传到外部服务。

原运行：SOURCE
工具：Gowin V1.9.12.04，GW5AT-LV138PG484AC1/I0 revision C。
核心50MHz、接收域100MHz，26条hold-only和26条最大延迟候选；place1/route2、复制1、宽拆分1、timing_driven1。

在Windows上解压到新目录，安装/使用Python与同版已授权Gowin工具。执行：

    python run_bounded.py --tool "D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/bin/gw_sh.exe" --telemetry

--telemetry需要psutil；可省略，仍执行运行预算。单次上限90分钟、无日志/产物进展75分钟，可用参数缩短。CPU/系统观测不会重置预算。已有impl目录时拒绝覆盖，保留全部结果后从新解压目录重试。

run.tcl可供厂商直接检查设置；直接执行Tcl会绕过Python预算，不能因此无限等待。旧.p/db为受保护/未验证格式，未装入新实现，也不作为拥塞定位。

manifest强制diagnostic_only。即使产生位流或工具返回0，仍不得接受或部署。请保留最终rpt.txt、tr、vo、完整日志和tool_runtime/system_telemetry记录；缺报告即未验证。没有硬件下载动作。
'''.replace('SOURCE',str(source))
    (out/'README.md').write_text(readme,encoding='utf-8')
    files=[p for p in out.rglob('*')if p.is_file()]
    archive=out.with_suffix('.zip')
    assert not archive.exists(),archive
    with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED,compresslevel=6)as z:
        for file in files:z.write(file,file.relative_to(out))
    result=dict(source=str(source),directory=str(out),archive=str(archive),files=len(files),archive_bytes=archive.stat().st_size,
        scope='frozen standalone PNR input package, not a logic-minimized reproducer or proven vendor defect',uploaded=False,programmed=False)
    (out/'package_result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result))


if __name__=='__main__':main()
