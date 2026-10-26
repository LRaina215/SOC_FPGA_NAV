"""Prepare/run a completed-PNR51 read-only system review, never EDA.

Default only prints the review configuration. --review refuses to parse the VO
until the original tool identity has exited and tool_runtime has an ended_at.
New ring data/address/prefetch equivalence belongs to the independent region
reviewer; this entry links its result instead of repeating that proof.
"""
from pathlib import Path
from datetime import datetime
from collections import Counter, defaultdict
from types import SimpleNamespace
import argparse, json, re, traceback
import psutil
from synthesis_connectivity import CELL, PORT
from audit_cdc_first_stages import sdc_fragment
from prepare_baseline_fusion_retime import metadata, same_bytes
from related_clock_physical_review_common import make_context
from review_lease_tree_pnr50_final import report as resource_report, CHANNELS
import review_pnr50_bsram_init as bsram
import review_pnr50_distributed_init as distributed

BASE = Path('F:/nav_integration_g2_20261023')
SOURCE = BASE / 'baseline_fusion_model25_observation_ring_syn39'
PHYSICAL = BASE / 'baseline_fusion_model25_observation_ring_pnr51'
CONTROL = BASE / 'baseline_fusion_model25_observation_ring_pnr_one_shot'
REFERENCE50 = BASE / 'baseline_fusion_model25_bounded_MAX_pnr50'
SOURCE_REVIEW = BASE / 'SYN39_ring_resources_memories_clocks_review2'
REGION_RESULT = BASE / 'pnr51_ring_final_actual_review1/final_review.json'
DEFAULT_OUT = BASE / 'pnr51_system_final_actual_review1'
GUARD_KINDS = {'g2_model25_guard', 'safety_guard'}

def load(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))

def save(path, value):
    assert not path.exists(), ('evidence must not be overwritten', str(path))
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf8')

def configuration():
    return dict(prepared=True, source=str(SOURCE), physical=str(PHYSICAL),
                output=str(DEFAULT_OUT), automatic_wait=False, starts_EDA=False,
                writes_frozen_build=False, automatic_programming=False,
                required_finished_runtime=str(PHYSICAL / 'tool_runtime.json'),
                new_ring_independent_result=str(REGION_RESULT),
                reuse=['review_pnr50_bsram_init.extract/check',
                       'review_pnr50_distributed_init.review',
                       'PNR50 flat VO/primitive/registered-copy comparator',
                       'collect_pnr_result.collect',
                       'check_setup_endpoint_coverage.scan_report',
                       'review_fusion32_CDC52_final.parse_exception_delays'])

def finished_identity():
    path = PHYSICAL / 'tool_runtime.json'
    runtime = load(path)
    assert 'ended_at' in runtime and 'tool_exit' in runtime, 'PNR51 still in progress; VO not opened'
    assert runtime['max_runtime_s'] == runtime['max_no_observable_progress_s'] == 0
    assert runtime['cancelled'] is False, 'Cancelled run must be investigated as cancelled, not natural PNR completion'
    pid = runtime['pid']
    created = datetime.fromisoformat(runtime['started_at']).timestamp()
    exists = psutil.pid_exists(pid)
    current = psutil.Process(pid).create_time() if exists else None
    assert not (exists and abs(current-created) < 2), ('original PNR51 tool identity is still alive', pid, created, current)
    state = load(CONTROL / 'job.json')
    assert Path(state['output']).resolve() == PHYSICAL.resolve()
    return runtime, dict(pid=pid, original_start_epoch=created, current_PID_exists=exists,
                         current_creation_epoch=current, original_tool_identity_exited=True)

def guard_hierarchies(modules):
    found = {}
    def visit(kind, prefix):
        for child, raw, body in CELL.findall(modules[kind]):
            name = (prefix + '/' + raw.lstrip('\\')).strip('/')
            if child in GUARD_KINDS:
                found[name] = child
            if child in modules:
                visit(child, name)
    visit('g2_top', '')
    assert set(found.values()) == GUARD_KINDS
    return found

def memory_review(out):
    folder = out / 'BSRAM_CPU'
    folder.mkdir()
    paths = [SOURCE/'impl/gwsynthesis/g2_top.vg', PHYSICAL/'g2_top.vg',
             PHYSICAL/'impl/pnr/g2_top.vo', PHYSICAL/'impl/pnr/g2_top.rpt.txt',
             PHYSICAL/'impl/pnr/cmd.do', PHYSICAL/'g2_gate.json', bsram.LIB,
             SOURCE/'firmware/nav_fw.bin', SOURCE/'ram_w.hex'] + [SOURCE/f'ram_b{i}.hex' for i in range(4)]
    before = [bsram.metadata(p) for p in paths]
    syn, physical, defaults = bsram.extract(SOURCE, PHYSICAL)
    bsram.save(folder/'actual_bsram_parameters_ports.json', dict(source=syn, physical=physical, library_defaults=defaults))
    result = bsram.check(SOURCE, PHYSICAL, syn, physical, defaults)
    result['legacy_reader_unrouted1659_literal_found'] = result.pop('routing_failure_recorded_in_report')
    actual_report = (PHYSICAL/'impl/pnr/g2_top.rpt.txt').read_text()
    unrouted = re.findall(r'total\s+(\d+)\s+unrouted nets', actual_report)
    result['actual_reported_unrouted_nets'] = int(unrouted[-1]) if unrouted else None
    # Reused reader's old key contains SYN37 in its name; source/path and this
    # explicit new binding identify actual SYN39. Do not claim this is SYN37.
    result['actual_frozen_input_is_completed_SYN39'] = same_bytes(SOURCE/'impl/gwsynthesis/g2_top.vg', PHYSICAL/'g2_top.vg')
    result['legacy_reader_key_note'] = 'frozen_physical_input_VG_equals_SYN37 is a legacy field name; its comparator uses the source argument recorded here (SYN39).'
    result['inputs_before'] = before
    result['inputs_unchanged_after_read_only_review'] = before == [bsram.metadata(p) for p in paths]
    result['passed'] &= result['inputs_unchanged_after_read_only_review']
    bsram.save(folder/'result.json', result)
    assert result['passed'], ('BSRAM/CPU audit failed', str(folder/'result.json'))
    dist = out/'distributed_INIT'
    dist.mkdir()
    dr = distributed.review(SimpleNamespace(source=SOURCE, physical=PHYSICAL,
        bsram_proof=folder/'result.json', out=dist))
    assert dr['passed'], ('distributed INIT audit failed', str(dist/'result.json'))
    return result, dr

def verify_extra_guard_replicas(ctx, names, extras):
    after, params, effective = ctx['physical'], ctx['physical_parameters'], ctx['effective_parameters']
    canonical, drivers = ctx['canonical'], ctx['physical_buffer_drivers']
    rows = []
    for n in sorted(extras):
        c = after[n]
        if c['kind'] in ('INV', 'BUF', 'BUFG'):
            rows.append(dict(instance=n, kind=c['kind'], nonstate_buffer=True))
            continue
        assert c['kind'].startswith('DFF'), ('unreviewed extra guard primitive', n, c)
        possible = []
        for original in names:
            a = after[original]
            if a['kind'] != c['kind'] or effective(a['kind'], params[original]) != effective(c['kind'], params[n]):
                continue
            if a['ports'].keys() != c['ports'].keys():
                continue
            if all([canonical(after, drivers, v) for v in a['ports'][p]] ==
                   [canonical(after, drivers, v) for v in c['ports'][p]]
                   for p in a['ports'] if p != 'Q'):
                possible.append(original)
        assert possible, ('extra guard FF has no same-INIT/same-transition original', n)
        rows.append(dict(instance=n, kind=c['kind'], same_kind_effective_INIT_and_all_nonQ_transition_inputs_to_actual_originals=sorted(possible)))
    return rows

def connections_clocks_guards(out, reference):
    from revalidate_lidar_cdc import definitions
    modules = definitions(SOURCE/'impl/gwsynthesis/g2_top.vg')
    guards = guard_hierarchies(modules)
    extra = {row[role] for row in reference['paths'] for role in ('source_register','first_register','second_register')}
    ctx = make_context(SOURCE/'impl/gwsynthesis/g2_top.vg', PHYSICAL/'impl/pnr/g2_top.vo',
                       extra=extra, extra_prefixes=[p+'/' for p in guards])
    before, after, equal = ctx['source'], ctx['physical'], ctx['equal_cell']
    directions, canonical = ctx['directions'], ctx['canonical']
    pd = ctx['physical_buffer_drivers']
    selected = []
    channel_rows = []
    for name, prefix, width in CHANNELS:
        names = {n for n,c in before.items() if n.startswith(prefix+'/') and c['kind'] not in ('GND','VCC')}
        actual = {n for n,c in after.items() if n.startswith(prefix+'/') and c['kind'] not in ('GND','VCC')}
        assert names <= actual, ('related channel lost cells', name, sorted(names-actual))
        checks = [equal(n) for n in sorted(names)]
        copies = verify_extra_guard_replicas(ctx, names, actual-names)
        selected += checks
        channel_rows.append(dict(name=name, logical_width=width, compared_primitives=len(names),
                                 original_primitives_all_pins_effective_INIT_equal=True,
                                 additional_checked_buffers_or_same_transition_replicas=copies))
    for n in sorted(extra):
        selected.append(equal(n))
    first_qs = {}
    for row in reference['paths']:
        src, first, second = [after[row[p]] for p in ('source_register','first_register','second_register')]
        assert src['ports']['Q'] == first['ports']['D'] and first['ports']['Q'] == second['ports']['D']
        assert len(first['ports']['Q']) == 1
        first_qs[first['ports']['Q'][0]] = second['name']
    # Same full-VO first-stage load count used in PNR50; no selected-scope-only
    # fanout assertion and no blanket asynchronous-clock exception.
    from review_lease_tree_pnr50_final import CELL as VO_CELL
    text = (PHYSICAL/'impl/pnr/g2_top.vo').read_text()
    loads = defaultdict(list)
    for kind,n,body in VO_CELL.findall(text):
        if kind == 'module':
            continue
        n = n.lstrip('\\')
        for pin, expr in PORT.findall(body):
            if directions[kind][pin.lstrip('\\')] == 'output':
                continue
            net = re.sub(r'\s+(?=\[)', '', expr.strip().lstrip('\\'))
            if net in first_qs:
                loads[net].append((n,pin))
    for q, second in first_qs.items():
        assert loads[q] == [(second,'D')], ('first-stage extra physical loads', q, loads[q])
    guard_rows = []
    for prefix, kind in guards.items():
        names = {n for n,c in before.items() if n.startswith(prefix+'/') and c['kind'] not in ('GND','VCC')}
        actual = {n for n,c in after.items() if n.startswith(prefix+'/') and c['kind'] not in ('GND','VCC')}
        assert names <= actual
        checks = [equal(n) for n in sorted(names)]
        copies = verify_extra_guard_replicas(ctx, names, actual-names)
        guard_rows.append(dict(instance=prefix, kind=kind, checked_primitive_count=len(names),
                               primitive_function_ports_effective_INIT_preserved=True,
                               additional_checked_buffers_or_registered_replicas=copies))
        selected += checks
    clock_rows, counts = [], Counter()
    root_map = {'clk50':'clk50', 'rgmii_rxc':'rgmii_rxc', 'u_model_pll/PLL_inst/CLKOUT0':'model25'}
    for key, net in ctx['clockpins'].items():
        root, phase = canonical(after, pd, net)
        assert root in root_map, ('unexpected physical clock root', key, net, root, phase)
        counts[root_map[root],phase] += 1
        clock_rows.append(dict(instance=key[0],pin=key[1],bit=key[2],net=net,root=root_map[root],inverted=bool(phase)))
    assert {r for r,p in counts} == {'clk50','rgmii_rxc','model25'}
    pll = equal('u_model_pll/PLL_inst')
    save(out/'actual_connections_clocks_guards.json', dict(passed=True,
        original26_paths=26, original26_endpoints=78, first_stage_Q_single_second_D_fanout=True,
        original26_and_channel_primitive_checks=selected, five_related_channels=channel_rows,
        guards=guard_rows, actual_clock_pin_count=len(clock_rows), actual_clock_pin_traces=clock_rows,
        actual_clock_root_counts=[dict(root=r,inverted=bool(p),pins=n)for(r,p),n in sorted(counts.items())],
        actual_PLL=after['u_model_pll/PLL_inst'], PLL_parameters=ctx['physical_parameters']['u_model_pll/PLL_inst'],
        active_PLL_and_disabled_unused_output_parameter_checks=pll,
        proved_registered_replica_aliases=list(ctx['proved_registered_replica_aliases'].values()),
        physical_reset_release_payload_delays_and_MTBF_accepted=False,
        new_ring_semantics_not_repeated_here=True))
    return dict(passed=True, original26_paths=26, original26_endpoints=78,
                actual_clock_pin_count=len(clock_rows), guard_instances=guards,
                three_actual_clock_roots=True, five_related_channels_checked=True)

def run(out):
    runtime, identity = finished_identity()
    assert not out.exists(), 'choose a fresh evidence directory'
    out.mkdir(parents=True)
    required = [SOURCE/'impl/gwsynthesis/g2_top.vg', PHYSICAL/'g2_top.vg',
                PHYSICAL/'impl/pnr/g2_top.vo', PHYSICAL/'impl/pnr/g2_top.rpt.txt',
                PHYSICAL/'impl/pnr/cmd.do', PHYSICAL/'impl/pnr/device.cfg',
                PHYSICAL/'g2.sdc', PHYSICAL/'g2.cst', PHYSICAL/'tool_runtime.json']
    bindings = [metadata(p) for p in required]
    proof = load(SOURCE_REVIEW/'result.json')
    assert proof['passed'] and proof['actual_complete_SYN39_exit'] == 0
    assert same_bytes(SOURCE/'impl/gwsynthesis/g2_top.vg', PHYSICAL/'g2_top.vg')
    assert same_bytes(SOURCE/'g2.cst', PHYSICAL/'g2.cst')
    assert same_bytes(PHYSICAL/'impl/pnr/device.cfg', REFERENCE50/'impl/pnr/device.cfg')
    expected = (REFERENCE50/'impl/pnr/cmd.do').read_text().replace(str(REFERENCE50), str(PHYSICAL)).replace(REFERENCE50.as_posix(), PHYSICAL.as_posix())
    assert expected == (PHYSICAL/'impl/pnr/cmd.do').read_text()
    reference = load(PHYSICAL/'cdc_reference_audit.json')
    assert reference['passed'] and len(reference['paths']) == 26
    normalize = lambda s: ' '.join(s.split())
    commands = [x.strip() for x in (PHYSICAL/'g2.sdc').read_text().splitlines() if x.strip() and not x.lstrip().startswith('#')]
    exceptions = [x for x in commands if x.startswith('set_')]
    assert list(map(normalize,exceptions)) == list(map(normalize,[x for x in sdc_fragment(reference).splitlines() if x.startswith('set_')]))
    assert len(exceptions) == 52 and sum(x.endswith(' -hold') for x in exceptions) == 26
    assert len([x for x in commands if x.startswith('create_clock ')]) == 2
    assert any(x == 'create_clock -name clk50 -period 20 [get_ports {clk50}]' for x in commands)
    assert any(x == 'create_clock -name rgmii_rxc -period 10 [get_ports {rgmii_rxc}]' for x in commands)
    assert any(x == 'create_generated_clock -name model25 -source [get_ports {clk50}] -divide_by 2 [get_pins {u_model_pll/PLL_inst/CLKOUT0}]' for x in commands)
    print('completed51 BSRAM/split/CPU and distributed INIT', flush=True)
    memory, distributed_result = memory_review(out)
    print('completed51 three clocks original26 related channels and guards', flush=True)
    connections = connections_clocks_guards(out, reference)
    print('completed51 actual resource/global and available TR evidence', flush=True)
    resources = {tag:resource_report(folder) for tag,folder in [('35',BASE/'baseline_fusion32_CDC52_pnr35'),('50',REFERENCE50),('51',PHYSICAL)]}
    from review_three_clock_TR import review_timing
    timing = review_timing(PHYSICAL, out/'actual_timing_review.json')
    independent_ring = dict(available=REGION_RESULT.exists(), result=str(REGION_RESULT), reviewed_here=False)
    if REGION_RESULT.exists():
        rr = load(REGION_RESULT)
        independent_ring.update(passed=rr.get('passed'), independent_root_binding_still_required=True,
                                metadata=metadata(REGION_RESULT))
    assert all(metadata(Path(row['path'])) == row for row in bindings)
    result = dict(review_completed=True, passed=True, recorded_at=datetime.now().astimezone().isoformat(),
        scope='Completed51 actual resources/init/clock/connectivity plus available timing evidence; no deployment acceptance.',
        source=str(SOURCE), physical=str(PHYSICAL), runtime=runtime, original_process_identity=identity,
        actual_resources=resources, source_VG_actual_cmd_device_CST_and52requests_bound=True,
        all_BSRAM_and_distributed_effective_INIT_CPU_checks_passed=memory['passed'] and distributed_result['passed'],
        BSRAM=memory['physical_BSRAM'], split_pairs=memory['split_pairs'], CPU=memory['CPU64'],
        connections_clocks_guards=connections, timing=timing, independent_ring=independent_ring,
        actual_ring_semantics_not_duplicated=True, new_EDA_started=False, frozen_build_modified=False,
        automatic_PNR=False, automatic_programming=False, deployment_accepted=False,
        metadata_bindings=bindings, physical_IO_CDC_reset_complete_signoff=False,
        global_assignment_changes_are_not_failed_net_identity_or_unique_cause=True)
    save(out/'final_review.json', result)
    return result

if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--review', action='store_true', help='Read completed51; refuses an in-progress original tool')
    ap.add_argument('--out', type=Path, default=DEFAULT_OUT)
    args = ap.parse_args()
    if not args.review:
        print(json.dumps(configuration(), ensure_ascii=False, indent=2))
    else:
        try:
            r = run(args.out)
            print(json.dumps(dict(review_completed=r['review_completed'], result=str(args.out/'final_review.json'),
                                  BSRAM=r['BSRAM'], deployment_accepted=False), ensure_ascii=False), flush=True)
        except Exception as error:
            if args.out.exists():
                save(args.out/'review_failed.json', dict(passed=False,error=str(error),traceback=traceback.format_exc(),recorded_at=datetime.now().astimezone().isoformat()))
            raise
