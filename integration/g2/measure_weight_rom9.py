"""Compare only real old/new weight ROMs with common launch/sink registers."""
import argparse,html,json,re,shutil,subprocess
import xml.etree.ElementTree as ET
from pathlib import Path
from optimize_storage import ROOT,prepare
from preflight import validate


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--variant',choices=['both','bank16','packed9'],default='both');a=ap.parse_args()
    out=a.out.resolve();out.mkdir(parents=True,exist_ok=False);rows=[]
    tool=Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/bin/gw_sh.exe')
    variants=(False,True) if a.variant=='both' else (a.variant=='packed9',)
    for packed in variants:
        run=out/('packed9' if packed else 'bank16');run.mkdir()
        shutil.copytree(ROOT/'control/fpga/src/mem',run/'src/mem');layout=prepare(run,directed=True,packed9=packed)
        (run/'wrapper.v').write_text('''module weight_rom_timing(input clk,en,input[15:0]addr,output reg signed[15:0]value);
reg en_q;reg[15:0]addr_q;wire signed[15:0]word_q;
always@(posedge clk)begin en_q<=en;addr_q<=addr;value<=word_q;end
fixed4500_weight_rom u_rom(clk,en_q,addr_q,word_q);
endmodule
''')
        (run/'clock.sdc').write_text('create_clock -name clk50 -period 20 [get_ports {clk}]\n')
        settings=['set_device -device_version C GW5AT-LV138PG484AC1/I0',
            f'add_file -type verilog {{{(run/"wrapper.v").as_posix()}}}',
            f'add_file -type verilog {{{(run/"fixed4500_weight_rom.v").as_posix()}}}',
            f'add_file -type sdc {{{(run/"clock.sdc").as_posix()}}}',
            'set_option -top_module weight_rom_timing','set_option -output_base_name weight_rom_timing',
            'set_option -verilog_std v2001','set_option -global_freq 50',
            'set_option -looplimit 100000','set_option -convert_sdp32_36_to_sdp16_18 1']
        effective=validate(tool,run,settings,50,1)
        (run/'run.tcl').write_text('\n'.join(settings+['run syn','exit'])+'\n')
        with (run/'gw_sh.log').open('w')as log:p=subprocess.run([str(tool),'run.tcl'],cwd=run,stdout=log,stderr=subprocess.STDOUT)
        s=(run/'gw_sh.log').read_text(encoding='utf-8',errors='replace')
        row=dict(variant=run.name,build=str(run),effective_options=effective,tool_exit=p.returncode,
            errors=re.findall(r'^.*ERROR\s*\(.*$',s,re.M),layout=layout)
        syn=run/'impl/gwsynthesis';report=syn/'weight_rom_timing_syn.rpt.html'
        if p.returncode==0 and not row['errors'] and report.exists():
            row['synthesis_resources']=ET.parse(syn/'weight_rom_timing_syn_rsc.xml').getroot().attrib
            report=report.read_text(encoding='utf-8')
            def cells(section):return [html.unescape(re.sub('<[^>]+>','',x)).strip() for x in re.findall(r'<td[^>]*>(.*?)</td>',section,re.S)]
            values=cells(report.split('<a name="performance">',1)[1].split('</table>',1)[0])
            fields=cells(report.split('Path Summary:',1)[1].split('</table>',1)[0])
            row.update(estimated_fmax_mhz=float(values[3].replace('(MHz)','')),first_path=dict(zip(fields[::2],fields[1::2])))
        rows.append(row);print(json.dumps(row),flush=True)
        (out/'result.json').write_text(json.dumps(dict(runs=rows,measurement_registers=33,programmed=False,
            scope='isolated ROM synthesis with launch and sink; not joint PNR, MAC timing or deployment acceptance'),indent=2),encoding='utf-8')
    return any(r['tool_exit']!=0 or r['errors'] for r in rows)


if __name__=='__main__':raise SystemExit(main())
