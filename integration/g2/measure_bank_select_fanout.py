"""Bounded paired module SYN for the actual 64x1024x18 map geometry.

Does not start a joint build, change a running PNR, or program hardware.
Only use full-design PNR to judge any routability/timing improvement.
"""
import argparse
from collections import Counter, defaultdict
from datetime import datetime
import json
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET

import psutil

from audit_netlist_memories import BSRAM
from audit_routing_topology import primitive_directions
from preflight import validate
from prepare_bank_select_fanout import generate
from retry_frozen_synthesis import memory_status
from synthesis_connectivity import read

TOOL = Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/bin/gw_sh.exe')
LIBRARY = Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v')
WRAPPER = '''module g2_top(
 input clk,we,re,input [15:0] waddr,raddr,input [17:0] wdata,
 output [17:0] rdata);
 nav_ram_sdp #(.DW(18),.AW(16),.DEPTH(65536)) u_ram
 (.clk(clk),.we(we),.re(re),.waddr(waddr),.raddr(raddr),.wdata(wdata),.rdata(rdata));
endmodule
'''


def run_module(out):
    memory = memory_status()
    # This is one small module, capped below768MiB, not a full joint SYN.
    # Full joint SYN still requires the existing2/4GiB startup reserves.
    assert memory.avail_phys >= 1024**3 and memory.avail_page >= 3*1024**3, 'small probe memory reserve unavailable'
    state = dict(started_at=datetime.now().astimezone().isoformat(), max_runtime_s=180,
                 max_owned_working_set_bytes=768*1024**2, cancelled=False,
                 start_free_bytes=memory.avail_phys, start_commit_reserve_bytes=memory.avail_page,
                 scope='one bounded64-bank module SYN; other tool processes never terminated')
    started = time.monotonic()
    with (out/'gw_sh.log').open('w', encoding='utf-8') as stream:
        process = subprocess.Popen([str(TOOL), 'run.tcl'], cwd=out, stdout=stream,
                                   stderr=subprocess.STDOUT, creationflags=subprocess.CREATE_NO_WINDOW)
        state['pid'] = process.pid
        peak = 0
        try:
            while process.poll() is None:
                owner = psutil.Process(process.pid)
                owned = [owner, *owner.children(recursive=True)]
                used = 0
                for owned_process in owned:
                    try:
                        used += owned_process.memory_info().rss
                    except psutil.NoSuchProcess:
                        continue
                peak = max(peak, used)
                memory = memory_status()
                reason = ('module180s budget' if time.monotonic()-started > 180 else
                          'module working set cap' if used > 768*1024**2 else
                          'host memory reserve' if memory.avail_phys < 256*1024**2 or memory.avail_page < 1024**3 else None)
                if reason:
                    state.update(cancelled=True, reason=reason)
                    for child in reversed(owned):
                        if child.is_running():
                            child.kill()
                    process.wait()
                    break
                time.sleep(.5)
        except psutil.NoSuchProcess:
            process.wait()
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()
            state.update(tool_exit=process.returncode, peak_observed_owned_rss=peak,
                         elapsed_s=time.monotonic()-started,
                         ended_at=datetime.now().astimezone().isoformat())
            (out/'module_runtime.json').write_text(json.dumps(state, indent=2), encoding='utf-8')
    assert not state['cancelled'] and process.returncode == 0, state
    return state


def inventory(netlist):
    cells = read(netlist)
    directions = primitive_directions(LIBRARY)
    loads = defaultdict(Counter)
    for c in cells.values():
        for pin, nets in c['ports'].items():
            if directions[c['kind']][pin] in ('input', 'inout'):
                for net in nets:
                    loads[net][pin] += 1
    registers = [dict(name=n, kind=c['kind'], ports=c['ports'],
                      direct_loads=dict(loads[c['ports']['Q'][0]]))
                 for n, c in cells.items() if c['kind'].startswith('DFF') and 'selected' in n]
    assert registers
    counts = Counter(c['kind'] for c in cells.values())
    return dict(primitives=dict(counts), bsram=sum(counts[t] for t in BSRAM),
                ff=sum(n for t,n in counts.items() if t.startswith('DFF')),
                max_selected_load=max(sum(r['direct_loads'].values()) for r in registers),
                selected_registers=registers)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('source', type=Path)
    p.add_argument('--out', type=Path, required=True)
    a = p.parse_args()
    source, out = a.source.resolve(), a.out.resolve()
    out.mkdir(parents=True, exist_ok=False)
    original = (source/'nav/rtl/common/nav_ram_sdp.v').read_bytes()
    result = dict(source=str(source), build=str(out), geometry=dict(DW=18, AW=16, DEPTH=65536, banks=64),
                  attribute_only=True, programmed=False, full_joint_synthesis=False, runs=[],
                  scope='module SYN data connectivity only; no full PNR benefit or G2 acceptance assumed')
    for variant in ('original', 'fanout128'):
        folder = out/variant
        folder.mkdir()
        (folder/'ram.v').write_bytes(original)
        (folder/'top.v').write_text(WRAPPER, encoding='utf-8')
        if variant != 'original':
            generate(folder/'ram.v')
        (folder/'clock.sdc').write_text('create_clock -name core_clk -period 20 [get_ports {clk}]\n', encoding='utf-8')
        settings = ['set_device -device_version C GW5AT-LV138PG484AC1/I0']
        settings += [f'add_file -type verilog {{{(folder/n).as_posix()}}}' for n in ('ram.v','top.v')]
        settings += [f'add_file -type sdc {{{(folder/"clock.sdc").as_posix()}}}',
                     'set_option -top_module g2_top', 'set_option -output_base_name g2_top',
                     'set_option -global_freq 50', 'set_option -convert_sdp32_36_to_sdp16_18 0',
                     'set_option -looplimit 100000']
        effective = validate(TOOL, folder, settings, 50, 0)
        (folder/'run.tcl').write_text('\n'.join(settings+['run syn','exit'])+'\n', encoding='utf-8')
        runtime = run_module(folder)
        row = dict(variant=variant, effective_options=effective, runtime=runtime,
                   resources=ET.parse(folder/'impl/gwsynthesis/g2_top_syn_rsc.xml').getroot().attrib,
                   inventory=inventory(folder/'impl/gwsynthesis/g2_top.vg'))
        result['runs'].append(row)
        (out/'result.json').write_text(json.dumps(result, indent=2), encoding='utf-8', newline='\n')
        print(json.dumps(dict(variant=variant, load=row['inventory']['max_selected_load'],
                             bsram=row['inventory']['bsram'], ff=row['inventory']['ff'])), flush=True)
    before, after = [r['inventory'] for r in result['runs']]
    result['fanout_reduced'] = after['max_selected_load'] <= 128 and after['max_selected_load'] < before['max_selected_load']
    result['bsram_unchanged'] = before['bsram'] == after['bsram'] == 64
    assert result['bsram_unchanged']
    from verify_bank_select_replication import verify
    result['selector_transition_proofs'] = [verify(out/v/'impl/gwsynthesis/g2_top.vg')
                                           for v in ('original','fanout128')]
    (out/'result.json').write_text(json.dumps(result, indent=2), encoding='utf-8', newline='\n')
    print(json.dumps(dict(fanout_reduced=result['fanout_reduced'], bsram_unchanged=result['bsram_unchanged'])))
    return not result['fanout_reduced']


if __name__ == '__main__':
    raise SystemExit(main())
