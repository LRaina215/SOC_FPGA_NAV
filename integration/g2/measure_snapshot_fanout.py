"""Paired scheduler SYN diagnoses data-mux fanout, not real core performance."""
import argparse,collections,importlib.util,json,re,subprocess
import xml.etree.ElementTree as ET
from pathlib import Path
from preflight import validate
ROOT=Path(__file__).resolve().parents[2]
CELL=re.compile(r'^\s{2}(\w+)\s+(\\\S+|[^\s(]+)\s*\((.*?)\);',re.M|re.S)
PORT=re.compile(r'\.([A-Z][A-Z0-9_]*)\s*\(([^()]*)\)')
MODULE=re.compile(r'^module\s+(\S+)\s*\((.*?)^endmodule',re.M|re.S)


def logic_loads(text, module='rl_policy_tick_v5', limit=20):
    # Local names may repeat in the core. Never merge loads across modules.
    modules=dict(MODULE.findall(text))
    assert module in modules,module
    counts=collections.Counter();drivers={}
    for kind,name,body in CELL.findall(modules[module]):
        ports=dict(PORT.findall(body))
        if kind.startswith('LUT'):
            for pin,net in ports.items():
                net=net.replace('\\','').strip()
                if pin.startswith('I'):counts[net]+=1
                elif pin=='F':drivers[net]=dict(kind=kind,name=name,inputs={k:v for k,v in ports.items()if k.startswith('I')})
    return [dict(module=module,net=n,loads=v,driver=drivers.get(n))for n,v in counts.most_common(limit)]


def main():
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True)
    p.add_argument('--only',choices=('original','fanout128'));a=p.parse_args()
    out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
    spec=importlib.util.spec_from_file_location('snapshot_generator',ROOT/'control/tools/g2_snapshot_fanout.py')
    generator=importlib.util.module_from_spec(spec);spec.loader.exec_module(generator)
    stub=(ROOT/'integration/g2/tb_snapshot_fanout.sv').read_text().split('module fc_v4_top',1)[1].split('endmodule',1)[0]
    stub=('module fc_v4_top'+stub+'endmodule\n').replace('tb_snapshot_fanout.mode',"3'd0")
    results=[];tool=Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/bin/gw_sh.exe')
    for variant in ((a.only,)if a.only else ('original','fanout128')):
        run=out/variant;run.mkdir();source=run/'policy.v'
        source.write_bytes((ROOT/'control/fpga/src/rl_policy_tick_v5.v').read_bytes())
        if variant!='original':generator.generate(source)
        (run/'core_stub.v').write_text(stub)
        (run/'clock.sdc').write_text('create_clock -name core_clk -period 20 [get_ports {clk}]\n')
        settings=['set_device -device_version C GW5AT-LV138PG484AC1/I0']
        settings += [f'add_file -type verilog {{{(run/f).as_posix()}}}'for f in ('policy.v','core_stub.v')]
        settings += [f'add_file -type sdc {{{(run/"clock.sdc").as_posix()}}}','set_option -top_module rl_policy_tick_v5',
          'set_option -output_base_name snapshot','set_option -global_freq 50','set_option -convert_sdp32_36_to_sdp16_18 0',
          'set_option -verilog_std v2001','set_option -looplimit 100000']
        effective=validate(tool,run,settings,50,0)
        (run/'run.tcl').write_text('\n'.join(settings+['run syn','exit'])+'\n')
        with(run/'gw_sh.log').open('w')as log:
            process=subprocess.run([str(tool),'run.tcl'],cwd=run,stdout=log,stderr=subprocess.STDOUT,timeout=300)
        log=(run/'gw_sh.log').read_text(errors='replace');errors=re.findall(r'^.*ERROR\s*\(.*$',log,re.M)
        row=dict(variant=variant,tool_exit=process.returncode,errors=errors,effective_options=effective)
        if not errors and process.returncode==0:
            syn=run/'impl/gwsynthesis';row['resources']=ET.parse(syn/'snapshot_syn_rsc.xml').getroot().attrib
            row['top_direct_lut_loads']=logic_loads((syn/'snapshot.vg').read_text())
        results.append(row)
        (out/'result.json').write_text(json.dumps(dict(runs=results,scope='isolated scheduler with identical functional core stub, no physical routing or full inference claim',programmed=False),indent=2))
        print(json.dumps(row),flush=True)
        if errors or process.returncode:raise SystemExit(2)


if __name__=='__main__':main()
