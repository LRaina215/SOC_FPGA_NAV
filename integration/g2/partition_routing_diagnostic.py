"""One frozen-netlist ablation to screen navigation routing pressure.

Only the unique slf_match definition is replaced by constant outputs. The
remaining explicit cells, original control/weights, CSR, pins and clocks stay
unchanged at input. PNR may prune downstream logic and move the whole design:
this experiment cannot identify individual failed nets or prove congestion in
the removed module. Its output is permanently diagnostic-only.
"""
import argparse
from collections import Counter
from datetime import datetime
import json
from pathlib import Path
import re
import shutil
import time

from audit_netlist_memories import MODULE, INST
from bounded_tool import run_tool
from collect_pnr_result import finalize_gate
from preflight import validate
from synthesis_connectivity import read, DECL

TARGET = 'u_nav/u_match'
REPORTS = ('report_route_congestion -max_grids 200',
           'report_high_fanout_nets -max_nets 100 -min_fanout 100')


def stub_definition(kind, body):
    # Gowin closes some module headers as ")\n;", not ");".
    # A literal split would swallow declarations and the first real cell.
    end = re.search(r'\)\s*;', body)
    assert end and not DECL.search(body[:end.start()]), 'invalid module header'
    port_header = body[:end.start()]
    declarations = [m[0] for m in DECL.finditer(body) if m[1] in ('input', 'output', 'inout')]
    assert not any(m[1] == 'inout' for m in DECL.finditer(body)), 'bidirectional boundary needs a separate design'
    outputs = [(hi, lo, name) for d, hi, lo, name in DECL.findall(body) if d == 'output']
    assert outputs
    replacement = ['module '+kind+' ('+port_header+');', *declarations]
    output_bits = 0
    for hi, lo, name in outputs:
        bits = [None] if not hi else range(int(lo), int(hi)+1)
        for bit in bits:
            net = name+' ' if bit is None else name+' ['+str(bit)+']'
            replacement.append('  GND diagnostic_zero_'+str(output_bits)+' (\n    .G('+net+')\n);')
            output_bits += 1
    replacement.append('endmodule')
    return '\n'.join(replacement), output_bits


def prepare(source, out):
    source, out = Path(source).resolve(), Path(out).resolve()
    assert out != source and source not in out.parents and out not in source.parents
    report = (source/'impl/pnr/g2_top.rpt.txt').read_text(encoding='utf-8')
    assert re.search(r'PR0004\).*total 273 unrouted nets', report), 'requires the frozen273 baseline'
    original = (source/'g2_top.vg').read_text(encoding='utf-8')
    matches = list(MODULE.finditer(original))
    modules = {m[1]: m[2] for m in matches}
    references = []

    def walk(kind, prefix=''):
        for child, name in INST.findall(modules[kind]):
            path = (prefix+'/'+name.lstrip('\\')).strip('/')
            if child in modules:
                references.append((path, child))
                walk(child, path)
    walk('g2_top')
    target_type, = [kind for path, kind in references if path == TARGET]
    assert target_type == 'slf_match'
    assert sum(kind == target_type for _, kind in references) == 1, 'shared definition would alter other instances'
    target, = [m for m in matches if m[1] == target_type]
    replacement, output_bits = stub_definition(target_type, target[2])
    changed = original[:target.start()]+replacement+original[target.end():]
    out.mkdir(parents=True, exist_ok=False)
    (out/'g2_top.vg').write_text(changed, encoding='utf-8', newline='\n')
    before, after = read(source/'g2_top.vg'), read(out/'g2_top.vg')
    added = [c for n, c in after.items() if n.startswith(TARGET+'/')]
    assert len(added) == output_bits and all(c['kind'] == 'GND' for c in added), 'stub retained old logic'
    outside = lambda data: {n: c for n, c in data.items() if not n.startswith(TARGET+'/')}
    assert outside(before) == outside(after), 'unexpected cell/pin change outside partition'
    control_before = {n: c for n, c in before.items() if n.startswith('u_control/')}
    assert control_before and control_before == {n: c for n, c in after.items() if n.startswith('u_control/')}
    assert original[:target.start()] == changed[:target.start()]
    for file in ('g2.cst', 'g2.sdc', 'cdc_reference_audit.json', 'sfp_cdc_reference.json'):
        shutil.copyfile(source/file, out/file)
    (out/'ip').mkdir()
    shutil.copyfile(source/'ip/serdes.csr', out/'ip/serdes.csr')
    assert (source/'ip/serdes.csr').read_bytes() == (out/'ip/serdes.csr').read_bytes()
    cst = (out/'g2.cst').read_text(encoding='utf-8')
    removed = [line for line in cst.splitlines() if line.startswith('CLOCK_LOC "'+TARGET+'/')]
    assert len(removed) == 3 and all('LOCAL_CLOCK' in x for x in removed)
    cst = '\n'.join(line for line in cst.splitlines() if line not in removed)+'\n'
    (out/'g2.cst').write_text(cst, encoding='utf-8', newline='\n')
    sdc = (source/'g2.sdc').read_text(encoding='utf-8')
    (out/'g2.sdc').write_text(sdc+'\n# Diagnostic reports only; all original constraints retained.\n'+'\n'.join(REPORTS)+'\n', encoding='utf-8', newline='\n')
    plan = json.loads((source/'clock_resource_plan.json').read_text(encoding='utf-8'))
    plan['actions'] = [r for r in plan['actions'] if not r['net'].startswith(TARGET+'/')]
    plan['diagnostic_removed_constraints'] = removed
    (out/'clock_resource_plan.json').write_text(json.dumps(plan, indent=2), encoding='utf-8')
    settings = []
    for line in (source/'run.tcl').read_text(encoding='utf-8').splitlines():
        if line.strip() in ('run pnr', 'exit'):
            continue
        settings.append(line.replace(source.as_posix(), out.as_posix()).replace(str(source), str(out)))
    manifest = json.loads((source/'manifest.json').read_text(encoding='utf-8'))
    manifest.update(diagnostic_only=True, programmed=False, frozen_netlist=True,
        partition_diagnostic=dict(source=str(source), target=TARGET, type=target_type,
            replaced_by='constant-zero output primitives; no emulation of matcher functionality',
            removed_explicit_cells=sum(n.startswith(TARGET+'/') for n in before),
            removed_primitives=dict(Counter(c['kind'] for n,c in before.items() if n.startswith(TARGET+'/'))),
            replacement_output_bits=output_bits, unaffected_explicit_cells=len(outside(before)),
            identical_control_explicit_cells=len(control_before), outside_cells_and_pins_identical=True,
            original_sdc_prefix_unchanged=True, original_csr_identical=True,
            removed_local_constraints=removed, report_requests=list(REPORTS),
            interpretation='Screening only. Pruning and global placement changes confound precise attribution; '
                           'a pass does not prove matcher fault, and a fail does not exonerate it.'),
        max_runtime_minutes=120, max_no_progress_minutes=0,
        scope='partition ablation only; permanently reject G2/deployment acceptance')
    tool = Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/bin/gw_sh.exe')
    manifest['effective_options'] = validate(tool, out, settings, 50, 1,
        dict(replicate_resources=1, place_option=1, route_option=2, timing_driven=1,
             show_all_warn=1, gen_posp=1))
    (out/'manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8', newline='\n')
    (out/'run.tcl').write_text('\n'.join(settings+['run pnr','exit'])+'\n', encoding='utf-8', newline='\n')
    return manifest


def main():
    p = argparse.ArgumentParser()
    p.add_argument('source', type=Path)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--prepare-only', action='store_true')
    p.add_argument('--run-prepared', action='store_true')
    p.add_argument('--memory-wait-minutes', type=float, default=10)
    a = p.parse_args()
    assert not (a.prepare_only and a.run_prepared)
    assert 0 <= a.memory_wait_minutes <= 15
    out = a.out.resolve()
    if a.run_prepared:
        manifest = json.loads((out/'manifest.json').read_text(encoding='utf-8'))
        assert manifest['diagnostic_only'] and manifest['partition_diagnostic']['source'] == str(a.source.resolve())
        assert not (out/'impl').exists(), 'do not overwrite an implementation'
    else:
        manifest = prepare(a.source, out)
    print(json.dumps(manifest['partition_diagnostic']), flush=True)
    if a.prepare_only:
        return
    job = dict(build=str(out), source=str(a.source.resolve()), diagnostic_only=True,
               queued_at=datetime.now().astimezone().isoformat(), phase='waiting_for_memory',
               minimum_free_bytes=2*1024**3, minimum_commit_reserve_bytes=4*1024**3,
               memory_wait_minutes=a.memory_wait_minutes, programmed=False)
    def save_job():
        job['updated_at'] = datetime.now().astimezone().isoformat()
        (out/'job.json').write_text(json.dumps(job, indent=2), encoding='utf-8')
    from retry_frozen_synthesis import memory_status
    deadline = time.monotonic()+a.memory_wait_minutes*60
    while True:
        memory = memory_status()
        job.update(free_bytes=memory.avail_phys, commit_reserve_bytes=memory.avail_page)
        save_job()
        if memory.avail_phys >= job['minimum_free_bytes'] and memory.avail_page >= job['minimum_commit_reserve_bytes']:
            break
        if time.monotonic() >= deadline:
            job['phase'] = 'not_started_host_memory'
            save_job()
            return 1
        time.sleep(5)
    job.update(phase='pnr', started_at=datetime.now().astimezone().isoformat())
    save_job()
    r = run_tool(['D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/bin/gw_sh.exe', 'run.tcl'], out,
                 max_runtime_s=7200, max_no_progress_s=0, capture_system_telemetry=True)
    finalize_gate(out)
    job.update(phase='finished', ended_at=datetime.now().astimezone().isoformat(),
               tool_exit=r['tool_exit'], cancelled=r['cancelled'], deployment_accepted=False)
    save_job()
    from collect_routing_diagnostic import collect
    (out/'routing_diagnostic.json').write_text(json.dumps(collect(out), indent=2),
                                              encoding='utf-8', newline='\n')
    return int(bool(r['tool_exit'] or r['cancelled']))


if __name__ == '__main__':
    raise SystemExit(main())
