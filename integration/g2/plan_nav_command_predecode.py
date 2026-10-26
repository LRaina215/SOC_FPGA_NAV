"""Read-only PNR53/SYN43 command predecode plan; never emits RTL or runs EDA."""
from pathlib import Path
from datetime import datetime
from collections import Counter
import argparse
import json
import re
from analyze_pnr53_failed_endpoints import parse_path

BASE = Path('F:/nav_integration_g2_20261023')
BUILD = BASE / 'SLF128_storage_safeFW_syn43'
CLASSES = BASE / 'PNR53_all_failed_endpoints_classification1'
OLD = Path('F:/nav_integration_g2_20261022/directed_static_coverage_effective50')
REPO = Path(__file__).resolve().parents[2]


def read(p):
    return json.loads(p.read_text(encoding='utf8'))


def meta(p):
    s = p.stat()
    return dict(path=str(p), bytes=s.st_size, mtime_ns=s.st_mtime_ns)


def cell(text, name):
    m = re.search(r'\b(LUT[1-6])\s+' + re.escape(name) + r'\s*\((.*?)\);', text, re.S)
    assert m, name
    init = re.search(r'defparam\s+' + re.escape(name) + r'\.INIT\s*=\s*([^;]+);', text)
    assert init, name
    return dict(name=name, kind=m[1], ports=dict(re.findall(r'\.(\w+)\(([^)]*)\)', m[2])), INIT=init[1])


def plan(out):
    assert not out.exists()
    files = {
        'nav': BUILD / 'nav/rtl/top/nav_subsystem.v',
        'regfile': BUILD / 'nav/rtl/regs/nav_regfile.v',
        'map': BUILD / 'nav/rtl/regs/nav_regs_map.vh',
        'xt': BUILD / 'nav/rtl/lidar/xt_frame_rx.v',
        'match': BUILD / 'nav/rtl/match/slf_match.v',
        'arb': BUILD / 'nav/rtl/io/nav_bus_arb.v',
        'soc': BUILD / 'nav/rtl/soc/nav_soc.v',
        'elev': BUILD / 'nav/rtl/map/elev_map.v',
        'old_nav': OLD / 'nav/rtl/top/nav_subsystem.v',
        'old_regfile': OLD / 'nav/rtl/regs/nav_regfile.v',
        'old_proof': BASE / 'pose_predecode_frozen_source_review_20261025.json',
        'old_generator': REPO / 'integration/g2/prepare_pose_predecode.py',
        'PB43': BASE / 'SYN43_PB_owner_actual_binding1/result.json',
        'classes': CLASSES / 'all_failed_endpoint_classes.json',
    }
    texts = {k: p.read_text(encoding='utf8') for k, p in files.items() if p.suffix in ('.v', '.vh', '.py')}
    nav = texts['nav']
    catchall = "if (bus_we) rf_wr <= 1'b1; else rf_rd <= 1'b1;"
    assert nav.count(catchall) == texts['old_nav'].count(catchall) == 1
    assert 'pose_predecoded_q' not in nav
    for symbol, address in (('p_LIDAR_POSE_COMMIT', '2070'), ('p_MATCH_CMD', '700C'), ('p_MAP_CMD', '3004')):
        assert f"assign {symbol} = wr && (addr == 16'h{address});" in texts['regfile']
    rows = read(files['classes'])
    selected = [r for r in rows if r['from_node'] == 'u_nav/a_q_7_s0/Q']
    paths = {n: parse_path((CLASSES / f'worst_class_path{n}.txt').read_text(encoding='utf8')) for n in (24, 54, 65)}
    local = BASE / 'SLF128_storage_safeFW_syn43_memory_review1/local_modules'
    native = []
    for module, names in (('nav_subsystem', ('n12374_s3', 'n12374_s1')), ('slf_match', ('n5682_s14',)), ('xt_frame_rx', ('raw_frame_accept_s0',))):
        t = (local / f'{module}.v').read_text(encoding='utf8')
        native += [dict(module=module, **cell(t, n)) for n in names]
    # Stream only the actual regfile module, never build a second whole-G2 graph.
    buffer, active = [], False
    with (BUILD / 'impl/gwsynthesis/g2_top.vg').open(encoding='utf8') as f:
        for line in f:
            if line.startswith('module nav_regfile'): active = True
            if active: buffer.append(line)
            if active and line.startswith('endmodule'): break
    native.insert(0, dict(module='nav_regfile', **cell(''.join(buffer), 'n16388_s4')))
    prefix = []
    for row in paths[24]['actual_data_rows']:
        if row['type'] in ('tNET', 'tINS'): prefix.append(row)
        if row['node'].endswith('n12374_s1/F'): break
    prefix_delay = sum(r['delay_ns'] for r in prefix)
    excerpts = []
    terms = ('a_q <=', 'rf_wr <=', 'B_RF:', 'B_END:', 'mb_shadow_read', 'frame_can_start =',
             '!point_owned &&', 'frame_accept =', 'pose_pending <=', 'con_post_', 'nav_busy', 'm_req && m_ack',
             's_req   =', 'p_MATCH_CMD', 'p_LIDAR_POSE_COMMIT', 'p_MAP_CMD', '.frame_start(raw_frame_accept')
    for key in ('nav', 'regfile', 'xt', 'match', 'arb', 'soc'):
        excerpts.append(dict(path=str(files[key]), lines=[dict(line=i, text=s) for i, s in enumerate(texts[key].splitlines(), 1) if any(t in s for t in terms)]))
    old = read(files['old_proof'])
    result = dict(
        recorded_at=datetime.now().astimezone().isoformat(), scope='Read-only plan, not an implemented or timed candidate',
        RTL_generated=False, simulation_started=False, EDA_started=False, frozen_inputs_modified=False,
        actual_PNR53=dict(negative_unique_endpoints=len(rows), address7_launch_endpoints=len(selected),
                         categories=dict(Counter(r['category'] for r in selected)),
                         slack_range_ns=[min(r['slack_ns'] for r in selected), max(r['slack_ns'] for r in selected)],
                         representatives=paths),
        actual_command_prefix_cells=native,
        root_cause='Address700C MATCH command decode plus wd_q[0] feeds matcher frame_can_start; RAW accept then support/map acceptance and the reverse-ready chain reach XT RAM/elevation/extract CE. It is not solely POSE_COMMIT2070.',
        timing_limit=dict(path24_logic_levels=15, path24_data_delay_ns=29.880, path24_route_percent=79.017,
                          listed_first3_decode_gate_and_route_delay_ns=round(prefix_delay, 3),
                          unchanged_suffix_arithmetic_ns=round(29.880 - prefix_delay, 3),
                          meaning='Subtracting the listed prefix is illustrative only; remaining24.4ns means predecode alone does not prove50MHz closure. Placement/routing must be measured.'),
        old_reuse=dict(old_finite_proof=old['existing_proof'], catchall_write_branch_text_equal=True,
                       full_nav_bytes_equal=files['nav'].read_bytes() == files['old_nav'].read_bytes(),
                       full_regfile_bytes_equal=files['regfile'].read_bytes() == files['old_regfile'].read_bytes(),
                       reusable='Same-edge capture principle and pulse-reference harness, not old endpoint/whole-design acceptance',
                       not_reusable=['Old generator edits live repo ROOT and a now-absent coverage .frame_start(p_LIDAR_POSE_COMMIT & !match_only); do not run it on current source',
                                     'Current coverage and ownership begin at accepted RAW, and WAIT_POINTS/extraction/retain controls did not exist in the old proof',
                                     'Old541 transactions/83POSE pulses only proved old2070 pulse; it does not prove new MATCH/MAP predecode or physical timing']),
        proposed_structure=dict(
            phase='Plan only; generate a fresh candidate only after RTL-tool freeze/review authorization',
            base=str(BUILD), RTL_file_to_change='nav/rtl/top/nav_subsystem.v',
            minimum_primary='Three registered MATCH command bits evaluated at the existing RF-write acceptance edge, using aligned live bus_addr and bus_wdata[2:0]',
            cohesive_six_pulses=['MATCH_EVAL', 'MATCH_UPDATE', 'MATCH_CLEAR', 'LIDAR_POSE_COMMIT', 'MAP_SWEEP', 'MAP_CLEAR'],
            new_logical_pulse_bits=6, physical_FF_change=None, extra_RAM=0, extra_bus_or_point_cycles=0,
            capture='Default all pulses0 every clock. Set only inside the exact existing B_IDLE final-RF-write branch alongside rf_wr/a_q/wd_q. Never bypass special-window priority.',
            payload='Combine command bits from bus_wdata on that acceptance edge; keep original wd_q and every configuration register/operand snapshot timing. No later live-bus data dependency.',
            post_clock_equalities=['new_match_eval_q == rf_wr && (a_q==700C) && wd_q[0]',
                                  'new_match_update_q == rf_wr && (a_q==700C) && wd_q[1]',
                                  'new_match_clear_q == rf_wr && (a_q==700C) && wd_q[2]',
                                  'new_pose_q == rf_wr && (a_q==2070)',
                                  'new_map_sweep_q == rf_wr && (a_q==3004) && wd_q[0]',
                                  'new_map_clear_q == rf_wr && (a_q==3004) && wd_q[1]'],
            preserve_multi_bit_commands='Bits are independent pulses, not a one-hot encoding: EVAL/UPDATE/CLEAR and SWEEP/CLEAR combinations retain original consumer priority.',
            replace='Replace all matching consumers in this nav_subsystem, including mt_eval_st, XT pose_commit, map cmd_clear/sweep and support clear; keep raw_frame_accept-based support frame_start unchanged.',
            no_early_lookahead='Only registered pulses reach consumers. Do not feed current B_IDLE/bus_req combinationally into read_admit: that would suppress a RAW acceptance one edge earlier.',
            no_extra_CMD_latch_stage='Do not register already-decoded p_* after a_q: that would delay command/ACK-relative effects by one clock.'),
        edge_table=[
            dict(edge='E0 accepts RF transaction', old='a_q/wd_q/rf_wr capture; bs->B_RF; command becomes combinationally visible after E0', proposed='same captures plus pulse vector; no ACK and no earlier consumer event'),
            dict(edge='E1 consumes RF transaction', old='CSR/command consumers sample old rf_wr/address/data; RF ACK/rdata registered; bs->B_END', proposed='same values sampled, pulse vector clears with rf_wr; same ACK/rdata'),
            dict(edge='E2/end bubble', old='ACK clears; bs->B_IDLE; arbiter/CPU observe and retire ACK', proposed='identical'),
            dict(edge='stalled windows/B_MBX', old='held bus state; no rf_wr pulse', proposed='all predecoded command pulses remain0 regardless other-master signals')],
        required_future_proofs=[
            'All-state one-step equality of six new post-clock pulses to untouched original regfile pulse/word bits; Boolean/all16-bit-address proof and finite X/Z/reset tests separately',
            'Old/new nav_subsystem bus miter: ACK/rdata/IRQ/command effects identical every edge; address/data/mode atomic under both-master arbitration, waits and B_END',
            'RF writes0/FFFFFFFF/allcommand-bit combinations; all four byte-alignment aliases; reads never trigger; special windows retain priority and ACK timing',
            'Costmap4-byte read stalls, MATCH/TRAJ result latency, MBX_RX variable wait and held req across ACK/reset. Preserve original repeated-req behavior, not a new exactly-once claim',
            'CPU NAV request held stable plus simultaneous UART master; other master may change its payload while blocked. Preserve selector and original nonposted NAV capture',
            'Console posted byte versus UART console read/reset and concurrent NAV commands: keep original accepted byte, console ACK and21-bit pending ownership. Do not move posted logic',
            'TXshadow read at E0 versus TX_DATA write at B_RF; injection DATA/GO/large lengths/wrap/busy GO retain original capacity/order/ACK. No new mailbox protocol restriction',
            'RAW accept E0 with command bus acceptance E0, command E1 versus SOF/ex_busy/last pb_we, EVAL/UPDATE/CLEAR combinations, HOLD/disable/zero-point/reset and retained second pass',
            'POSE request early/when rd_wait and read_admit blocked, pose_pending retention and exact accepted RAW edge; do not revert coverage to pre-ownership POSE trigger',
            'MAP clear/sweep simultaneous with incoming valid/SOF and support epoch clear; compare stream backpressure and all RAM read/write enables/addresses/data',
            'Stop/IR/lease/tick and SAFE_CMD same-edge boundaries retain original priority and publication cycles; this optimization does not waive their independent existing failures',
            'Actual C mapping: predecoded FFs present, no a_q MATCH/MAP/POSE decode remains in the targeted CE cones, actual new FF D paths also meet timing, unchanged RAM geometry/INIT/clock/26CDC',
            'Full physical report: all setup/hold endpoints and stops checked. Retiming input decode is not proof that the remaining ready-chain routing closes'],
        fundamental_limits=['Current reverse-ready chain still traverses RAW/support/map/filter/transform/project/XT output-buffer/RAM; predecode does not register or shorten that whole chain',
                            'Registering ready/RAW accept/SOF or adding skid queues can change buffering/latency/ownership; it is a separate contract, not silently part of this minimal change',
                            'New predecode D logic moves to upstream bus/arbiter paths and can create a new setup limit; no physical saving/closure is assumed',
                            'Stop/IR and control-ring paths are separate failing classes; this proposal is no algorithm degradation and no global timing acceptance'],
        source_excerpts=excerpts, bindings=[meta(p) for p in files.values()] + [meta(Path(__file__).resolve())],
        actual_mapping_bindings=[meta(BUILD / 'impl/gwsynthesis/g2_top.vg')] + [meta(local / f'{n}.v') for n in ('nav_subsystem', 'slf_match', 'xt_frame_rx')],
        P95_policy='Latest root-reported user override: P95<=50mm is not this round advancement gate; actual error remains recorded, collision/localization failure/long drift/stop contracts remain required.',
        no_SHA=True)
    out.mkdir(parents=True)
    (out / 'plan.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf8')
    return result


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out', type=Path, required=True)
    a = p.parse_args()
    v = plan(a.out)
    print(json.dumps(dict(out=str(a.out), endpoints=v['actual_PNR53']['address7_launch_endpoints'],
                         RTL_generated=False, suffix_arithmetic_ns=v['timing_limit']['unchanged_suffix_arithmetic_ns']), ensure_ascii=False))
