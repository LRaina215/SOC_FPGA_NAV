"""Small isolated synthesis comparison; never a joint resource/timing signoff."""
import argparse,html,json,re,shutil,subprocess,xml.etree.ElementTree as ET
from pathlib import Path
from preflight import validate
from prepare_coverage_timing import ROOT,generate


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
    out=a.out.resolve();out.mkdir(parents=True,exist_ok=False);rows=[]
    tool=Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/bin/gw_sh.exe')
    for variant in ('original','static_bounds'):
        run=out/variant;run.mkdir();coverage=run/'coverage.v'
        if variant=='original':shutil.copyfile(ROOT/'rtl/map/support_coverage.v',coverage)
        else:generate(coverage)
        shutil.copyfile(ROOT/'integration/g2/nav_ram_sdp_directed.v',run/'memory.v')
        (run/'clock.sdc').write_text('create_clock -name clk50 -period 20.000 [get_ports {clk}]\n')
        settings=['set_device -device_version C GW5AT-LV138PG484AC1/I0',
            f'add_file -type verilog {{{coverage.as_posix()}}}',f'add_file -type verilog {{{(run/"memory.v").as_posix()}}}',
            f'add_file -type sdc {{{(run/"clock.sdc").as_posix()}}}',
            'set_option -top_module support_coverage','set_option -output_base_name support_coverage',
            'set_option -global_freq 50','set_option -convert_sdp32_36_to_sdp16_18 0',
            'set_option -verilog_std v2001','set_option -looplimit 100000']
        effective=validate(tool,run,settings,50,0)
        (run/'run.tcl').write_text('\n'.join(settings+['run syn'])+'\n')
        with (run/'gw_sh.log').open('w') as log:
            subprocess.run([str(tool),'run.tcl'],cwd=run,stdout=log,stderr=subprocess.STDOUT,check=True)
        text=(run/'gw_sh.log').read_text();assert not re.search(r'ERROR\s*\(',text)
        syn=run/'impl/gwsynthesis';rsc=ET.parse(syn/'support_coverage_syn_rsc.xml').getroot()
        report=(syn/'support_coverage_syn.rpt.html').read_text()
        def cells(section):return [html.unescape(re.sub('<[^>]+>','',x)).strip() for x in re.findall(r'<td[^>]*>(.*?)</td>',section,re.S)]
        values=cells(report.split('<a name="performance">',1)[1].split('</table>',1)[0])
        fields=cells(report.split('Path Summary:',1)[1].split('</table>',1)[0])
        rows.append(dict(variant=variant,build=str(run),effective_options=effective,synthesis_resources=rsc.attrib,
            estimated_fmax_mhz=float(values[3].replace('(MHz)','')),first_path=dict(zip(fields[::2],fields[1::2]))))
    result=dict(runs=rows,programmed=False,scope='isolated module synthesis estimates, not joint PNR, vehicle throughput or deployment acceptance')
    (out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))


if __name__=='__main__':main()
