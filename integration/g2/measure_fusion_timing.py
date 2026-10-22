"""Isolate selected control arithmetic from the large joint design."""
import argparse,html,importlib.util,json,re,shutil,subprocess
import xml.etree.ElementTree as ET
from pathlib import Path
from preflight import validate
ROOT=Path(__file__).resolve().parents[2]


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--module',choices=['fusion','action-map'],default='fusion');a=ap.parse_args()
    out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
    tool=Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/bin/gw_sh.exe')
    module='imu_fusion' if a.module=='fusion' else 'policy_action_map'
    top=module if a.module=='fusion' else 'action_map_timing'
    generator_name='g2_fusion_pipeline.py' if a.module=='fusion' else 'g2_action_map_widths.py'
    spec=importlib.util.spec_from_file_location('controlgen',ROOT/'control/tools'/generator_name)
    generator=importlib.util.module_from_spec(spec);spec.loader.exec_module(generator)
    rows=[]
    for variant in ('original','retimed' if a.module=='fusion' else 'exact_widths'):
        run=out/variant;run.mkdir();source=run/'fusion.v'
        if variant=='original':shutil.copyfile(ROOT/'control/fpga/src'/(module+'.v'),source)
        else:generator.generate(source)
        sources=[source]
        if a.module=='action-map':
            # These are launch registers for measurement, not a product change.
            # Without them, un-delayed input ports have no timed launch clock
            # and the report only measures unrelated self-feedback paths.
            wrapper=run/'launch_wrapper.v'
            wrapper.write_text('''module action_map_timing(
 input clk,rst_n,run,action_valid,input[143:0] actions,input signed[11:0] ff_lh,ff_lk,ff_rh,ff_rk,
 output signed[31:0] left_mmps,right_mmps,output[15:0] lh_us,lk_us,rh_us,rk_us,
 output[143:0] last_actions,output have_action);
 reg[143:0] actions_q;reg[47:0] ff_q;reg run_q,valid_q;
 always@(posedge clk)begin actions_q<=actions;ff_q<={ff_rk,ff_rh,ff_lk,ff_lh};run_q<=run;valid_q<=action_valid;end
 policy_action_map u_map(clk,rst_n,run_q,valid_q,actions_q,ff_q[11:0],ff_q[23:12],ff_q[35:24],ff_q[47:36],
  left_mmps,right_mmps,lh_us,lk_us,rh_us,rk_us,last_actions,have_action);
 endmodule
''')
            sources.append(wrapper)
        for name in (('imu_isqrt.v','imu_recip.v') if a.module=='fusion' else ()):
            shutil.copyfile(ROOT/'control/fpga/src'/name,run/name);sources.append(run/name)
        (run/'clock.sdc').write_text('create_clock -name clk50 -period 20 [get_ports {clk}]\n')
        settings=['set_device -device_version C GW5AT-LV138PG484AC1/I0']
        settings += [f'add_file -type verilog {{{p.as_posix()}}}' for p in sources]
        settings += [f'add_file -type sdc {{{(run/"clock.sdc").as_posix()}}}',
            'set_option -top_module '+top,'set_option -output_base_name '+top,
            'set_option -global_freq 50','set_option -convert_sdp32_36_to_sdp16_18 0',
            'set_option -verilog_std v2001','set_option -looplimit 100000']
        effective=validate(tool,run,settings,50,0)
        (run/'run.tcl').write_text('\n'.join(settings+['run syn'])+'\n')
        with (run/'gw_sh.log').open('w') as log:
            process=subprocess.run([str(tool),'run.tcl'],cwd=run,stdout=log,stderr=subprocess.STDOUT)
        text=(run/'gw_sh.log').read_text(encoding='utf-8',errors='replace')
        row=dict(variant=variant,build=str(run),effective_options=effective,tool_exit=process.returncode,
                 errors=re.findall(r'^.*ERROR\s*\(.*$',text,re.M))
        row['synthesis_succeeded']=process.returncode==0 and not row['errors']
        syn=run/'impl/gwsynthesis';report=syn/(top+'_syn.rpt.html')
        if row['synthesis_succeeded'] and report.exists():
            row['synthesis_resources']=ET.parse(syn/(top+'_syn_rsc.xml')).getroot().attrib
            report=report.read_text(encoding='utf-8')
            def cells(section):return [html.unescape(re.sub('<[^>]+>','',x)).strip() for x in re.findall(r'<td[^>]*>(.*?)</td>',section,re.S)]
            values=cells(report.split('<a name="performance">',1)[1].split('</table>',1)[0])
            fields=cells(report.split('Path Summary:',1)[1].split('</table>',1)[0])
            row.update(estimated_fmax_mhz=float(values[3].replace('(MHz)','')),first_path=dict(zip(fields[::2],fields[1::2])))
        rows.append(row);print(json.dumps(row),flush=True)
        (out/'result.json').write_text(json.dumps(dict(module=module,measurement_top=top,input_launch_registers=a.module=='action-map',runs=rows,programmed=False,
            scope='isolated control module synthesis diagnosis/estimates; not joint PNR or vehicle acceptance'),indent=2))
    return not all(row['synthesis_succeeded'] for row in rows)


if __name__=='__main__':raise SystemExit(main())
