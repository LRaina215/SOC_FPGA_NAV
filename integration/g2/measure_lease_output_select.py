"""Compare timed input-launch/feedback cones of the original and factored guard."""
import argparse,html,json,re,subprocess
import xml.etree.ElementTree as ET
from pathlib import Path
from prepare_lease_output_select import generate

ROOT=Path(__file__).resolve().parents[2]
WRAPPER='''module lease_timing(input clk,reset_n,input [167:0] raw,output [147:0] result);
 reg[167:0] launched;
 always@(posedge clk)launched<=raw;
 wire[1:0] owner,source;wire[31:0]session,psession;wire[15:0]seq;
 wire valid,fault,estop,feedback;wire signed[23:0]v,w;wire signed[15:0]mv,mw;
 assign {owner,source,session,psession,seq,valid,fault,estop,v,w,feedback,mv,mw}=launched;
 wire signed[23:0]ov,ow;wire armed,expired,stopped,balance;wire[31:0]age,accepted,rejected;
 command_lease_guard dut(clk,reset_n,owner,source,session,psession,seq,valid,fault,estop,v,w,
 feedback,mv,mw,ov,ow,armed,expired,stopped,age,accepted,rejected,balance);
 assign result={ov,ow,armed,expired,stopped,age,accepted,rejected,balance};
 endmodule
'''


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
    out=a.out.resolve();out.mkdir(parents=True,exist_ok=False);rows=[]
    for variant in ('original','output_select'):
        p=out/variant;p.mkdir();rtl=p/'guard.v'
        rtl.write_bytes((ROOT/'integration/g2/command_lease_guard.v').read_bytes())
        if variant=='output_select':generate(rtl)
        (p/'top.v').write_text(WRAPPER)
        (p/'clock.sdc').write_text('create_clock -name clk50 -period 20 [get_ports {clk}]\n')
        (p/'run.tcl').write_text(f'''set_device -device_version C GW5AT-LV138PG484AC1/I0
add_file -type verilog {{{rtl.as_posix()}}}
add_file -type verilog {{{(p/'top.v').as_posix()}}}
add_file -type sdc {{{(p/'clock.sdc').as_posix()}}}
set_option -top_module lease_timing
set_option -output_base_name lease_timing
set_option -global_freq 50
run syn
exit
''')
        with(p/'gw_sh.log').open('w')as log:
            proc=subprocess.run(['D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/bin/gw_sh.exe','run.tcl'],cwd=p,
                                stdout=log,stderr=subprocess.STDOUT,timeout=180,creationflags=subprocess.CREATE_NO_WINDOW)
        assert proc.returncode==0 and 'ERROR'not in(p/'gw_sh.log').read_text()
        syn=p/'impl/gwsynthesis';report=(syn/'lease_timing_syn.rpt.html').read_text()
        def cells(section):return [html.unescape(re.sub('<[^>]+>','',x)).strip()for x in re.findall(r'<td[^>]*>(.*?)</td>',section,re.S)]
        values=cells(report.split('<a name="performance">',1)[1].split('</table>',1)[0])
        fields=cells(report.split('Path Summary:',1)[1].split('</table>',1)[0])
        rows.append(dict(variant=variant,estimated_fmax_mhz=float(values[3].replace('(MHz)','')),
                         first_path=dict(zip(fields[::2],fields[1::2])),
                         resources=ET.parse(syn/'lease_timing_syn_rsc.xml').getroot().attrib))
    result=dict(runs=rows,input_launch_registers=168,scope='paired module SYN estimates, not joint physical timing',programmed=False)
    (out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))


if __name__=='__main__':main()
