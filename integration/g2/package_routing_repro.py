"""Package frozen PNR inputs and a bounded portable runner locally.

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
p.add_argument('--minutes',type=float,default=120)
p.add_argument('--silent-minutes',type=float,default=0)
p.add_argument('--telemetry',action='store_true',help='requires psutil on Windows')
a=p.parse_args();root=Path(__file__).resolve().parent
if not(0<a.minutes<=120 and 0<=a.silent_minutes<=120):p.error('total budget must be positive and at most120min; idle budget0 disables idle cancellation')
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
    from collect_pnr_result import has_real_sfp
    sfp=has_real_sfp(manifest)
    for name in ('g2_top.vg','g2.cst','g2.sdc','cdc_reference_audit.json','effective_options.tcl'):
        shutil.copyfile(source/name,out/name)
    for name in ('sfp_cdc_reference.json','clock_resource_plan.json'):
        if (source/name).exists():shutil.copyfile(source/name,out/name)
    if sfp:
        assert (source/'sfp_cdc_reference.json').exists(),'missing complete host CDC reference'
        assert (source/'ip/serdes.csr').exists(),'frozen SFP requires its exact CSR'
        (out/'ip').mkdir();shutil.copyfile(source/'ip/serdes.csr',out/'ip/serdes.csr')
    lines=(source/'run.tcl').read_text().splitlines();portable=['set repro_root [file dirname [file normalize [info script]]]','cd $repro_root']
    types={'netlist':'g2_top.vg','cst':'g2.cst','sdc':'g2.sdc'}
    for line in lines:
        match=re.match(r'add_file -type (netlist|cst|sdc) ',line)
        if match:line='add_file -type '+match[1]+' [file join $repro_root "'+types[match[1]]+'"]'
        if line.startswith('set_csr '):
            assert sfp,'unexpected CSR outside real SFP manifest'
            line='set_csr [file join $repro_root "ip" "serdes.csr"]'
        portable.append(line)
    (out/'run.tcl').write_text('\n'.join(portable)+'\n',encoding='utf-8',newline='\n')
    original_manifest=manifest.copy();manifest.update(diagnostic_only=True,programmed=False,
        scope='local reproducibility package only; no G2/deployment acceptance')
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    (out/'original_manifest.json').write_text(json.dumps(original_manifest,indent=2),encoding='utf-8')
    for name in ('bounded_tool.py','collect_pnr_result.py','audit_cdc_first_stages.py','audit_pin_loads.py',
                 'audit_sfp_cdc.py','audit_global_muxes.py','synthesis_connectivity.py','audit_netlist_memories.py'):
        shutil.copyfile(ROOT/'integration/g2'/name,out/name)
    (out/'run_bounded.py').write_text(RUNNER,encoding='utf-8',newline='\n')
    previous=out/'previous_run';previous.mkdir()
    for relative in ('tool_runtime.json','resource_cancellation.json','g2_gate.json','gw_sh.log','impl/pnr/g2_top.log',
                     'impl/pnr/cmd.do','impl/pnr/device.cfg','impl/pnr/g2_top.rpt.txt','impl/pnr/g2_top.tr',
                     'impl/pnr/g2_top.vo','impl/pnr/g2_top.posp','impl/pnr/g2_top.p','impl/pnr/g2_top.db'):
        file=source/relative
        if file.exists():shutil.copyfile(file,previous/relative.replace('/','_'))
    readme='''# 冻结布线复现包

此包包含复现所需的综合网表、引脚/时钟约束、SerDes CSR（启用真实SFP时）与工具选项，以及原失败/取消报告和布局现场。它尚未按逻辑删减成最小故障电路，不能称为已定位的厂商bug。没有上传到外部服务。

原运行：SOURCE
工具：Gowin V1.9.12.04，GW5AT-LV138PG484AC1/I0 revision C。
实际约束与选项见g2.sdc、g2.cst和run.tcl。启用真实SFP时包含50/100/125/125MHz、26条雷达与51条主机首级路径的154条候选约束；厂商PCS/状态/复位/IO完整审查仍未通过。请不要移用旧的整域false-path。

在Windows上解压到新目录，安装/使用Python与同版已授权Gowin工具。执行：

    python run_bounded.py --tool "D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/bin/gw_sh.exe" --telemetry

--telemetry需要psutil；可省略，仍执行运行预算。默认单次总上限120分钟，无静默终止；可用参数缩短。CPU/系统观测不会重置预算。已有impl目录时拒绝覆盖，保留全部结果后从新解压目录重试。

run.tcl可供直接检查设置；直接执行Tcl会绕过Python预算。previous_run中的.p/db仅是原失败现场：当前没有验证读取器，不作为新实现输入或拥塞定位结论。

manifest强制diagnostic_only。即使产生位流或工具返回0，仍不得接受或部署。请保留最终rpt.txt、tr、vo、完整日志和tool_runtime/system_telemetry记录；缺报告即未验证。没有硬件下载动作。

需要工具级诊断的具体交付：失败网络名称、源/目标引脚及物理位置；逐网未布通原因或拥塞区域；失败PNR时导出这些信息和BSRAM/DSP布局的受支持方法。资源未超限和高扇出计数都不能代替这些证据。
'''.replace('SOURCE',str(source))
    (out/'README.md').write_text(readme,encoding='utf-8')
    files=[p for p in out.rglob('*')if p.is_file()]
    archive=out.with_suffix('.zip')
    assert not archive.exists(),archive
    with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED,compresslevel=6)as z:
        for file in files:z.write(file,file.relative_to(out))
    result=dict(source=str(source),directory=str(out),archive=str(archive),files=len(files),archive_bytes=archive.stat().st_size,real_sfp=sfp,
        scope='frozen standalone PNR input package, not a logic-minimized reproducer or proven vendor defect',uploaded=False,programmed=False)
    (out/'package_result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result))


if __name__=='__main__':main()
