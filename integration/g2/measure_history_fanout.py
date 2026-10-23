"""Paired module SYN to verify the history-data fanout attribute takes effect."""
import argparse,importlib.util,json,re,subprocess
from collections import Counter,defaultdict
from pathlib import Path
from audit_trig_routing import CELLS
from audit_pin_loads import PORTS,INPUTS
from audit_cdc_first_stages import bits

ROOT=Path(__file__).resolve().parents[2]


def main():
    ap=argparse.ArgumentParser();ap.add_argument('frozen',type=Path);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
    out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
    spec=importlib.util.spec_from_file_location('history_fanout',ROOT/'control/tools/g2_history_fanout.py')
    gen=importlib.util.module_from_spec(spec);spec.loader.exec_module(gen)
    source=(a.frozen/'control/rl_observation_v5.v').read_bytes();rows=[]
    for variant in ('original','fanout128'):
        p=out/variant;p.mkdir();rtl=p/'observation.v';rtl.write_bytes(source)
        if variant=='fanout128':gen.generate(rtl)
        (p/'clock.sdc').write_text('create_clock -name clk -period 20 [get_ports {clk}]\n')
        (p/'run.tcl').write_text(f'''set_device -device_version C GW5AT-LV138PG484AC1/I0
add_file -type verilog {{{rtl.as_posix()}}}
add_file -type sdc {{{(p/'clock.sdc').as_posix()}}}
set_option -top_module rl_observation_v5
set_option -output_base_name observation
set_option -global_freq 50
set_option -looplimit 100000
run syn
exit
''')
        with(p/'gw_sh.log').open('w')as f:
            r=subprocess.run(['D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/bin/gw_sh.exe','run.tcl'],cwd=p,
                stdout=f,stderr=subprocess.STDOUT,timeout=180,creationflags=subprocess.CREATE_NO_WINDOW)
        assert r.returncode==0 and 'ERROR'not in(p/'gw_sh.log').read_text(),variant
        cells={n.lstrip('\\'):(k,dict(PORTS.findall(b)))for k,n,b in CELLS.findall((p/'impl/gwsynthesis/observation.vg').read_text())}
        counts=Counter();loads=defaultdict(Counter)
        for n,(k,ports) in cells.items():
            counts[k]+=1
            for pin,e in ports.items():
                if pin in INPUTS:
                    for net in bits(e):loads[net][pin]+=1
        regs=[]
        for n,(k,ports) in cells.items():
            if k.startswith('DFF') and n.startswith('initialized'):
                regs.append(dict(name=n,ports=ports,loads=dict(loads[bits(ports['Q'])[0]])))
        assert regs,variant
        rows.append(dict(variant=variant,primitives=dict(counts),initialized_registers=regs,
            max_initialized_load=max(sum(r['loads'].values())for r in regs)))
    result=dict(frozen=str(a.frozen),module_results=rows,attribute_only=True,
        scope='module SYN connectivity only; no change to algorithm/RTL operations, no joint physical benefit assumed')
    result['fanout_reduced']=rows[1]['max_initialized_load']<=128 and rows[1]['max_initialized_load']<rows[0]['max_initialized_load']
    (out/'result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(dict(fanout_reduced=result['fanout_reduced'],loads=[r['max_initialized_load']for r in rows])))
    return not result['fanout_reduced']


if __name__=='__main__':raise SystemExit(main())
