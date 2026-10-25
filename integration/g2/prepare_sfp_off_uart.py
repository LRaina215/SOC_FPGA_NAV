"""Prepare a frozen PHY-off/UART-retained candidate, without invoking tools.

The origin is the complete non-DDR SYN11 used by Lane2 PNR15 (184 unrouted).
Only the top's physical SFP attachment is replaced by its idle core-clock stub.
Mailbox injection/readback, HOST_EXTENSION_ENABLE=1, full V2 control UART,
navigation, policy, CPU, memories and stop sources remain byte-identical.
Old generated CLOCK_LOC names are archived for actual post-SYN driver review.
"""
import argparse
import copy
from datetime import datetime
import json
from pathlib import Path
import re
import shutil

from audit_cdc_first_stages import sdc_fragment

SFP_SOURCES = {'sfp_host_mac.v', 'sfp_management.v',
               'ip/serdes.v', 'ip/giga_serial_ethernet.v'}
ASSET_SUFFIXES = {'.hex', '.mem', '.bin', '.mif'}


def load(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def save(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n',
                          encoding='utf-8', newline='\n')


def relocate(value, source, out):
    if isinstance(value, str):
        return value.replace(source.as_posix(), out.as_posix()).replace(str(source), str(out))
    if isinstance(value, list):
        return [relocate(x, source, out) for x in value]
    if isinstance(value, dict):
        return {k: relocate(v, source, out) for k, v in value.items()}
    return value


def transform_top(original):
    assert original.count('`define G2_REAL_SFP\n') == 1
    text = original.replace('`define G2_REAL_SFP\n', '', 1)
    pattern = re.compile(r'^`ifdef G2_REAL_SFP\n(.*?)^`else\n(.*?)^`endif\n', re.M | re.S)
    matched = list(pattern.finditer(text))
    assert len(matched) == 1 and 'sfp_management u_sfp(' in matched[0][1]
    old_stub = matched[0][2]
    assert old_stub.count('assign sfp1_tx_dis=0;') == 1
    tap_zero = '    assign tap_valid=0;assign tap_first=0;assign tap_last=0;assign tap_data=0;\n'
    assert old_stub.count(tap_zero) == 1
    stub = old_stub.replace('assign sfp1_tx_dis=0;', 'assign sfp1_tx_dis=1;')
    stub = stub.replace(tap_zero, '')
    text = text[:matched[0].start()]+(
        '// Physical SFP disabled; full UART telemetry and host mailboxes retained.\n'+stub)+text[matched[0].end():]
    wrapper = re.compile(r'^`ifdef G2_REAL_SFP\n(.*?)^`endif\n', re.M | re.S)
    blocks = list(wrapper.finditer(text))
    assert len(blocks) == 2
    assert '.telemetry_valid(tap_valid)' in blocks[0][1]
    assert '.HOST_EXTENSION_ENABLE(1)' in blocks[1][1]
    text = wrapper.sub(lambda match: match[1], text)
    core_tap = '        .telemetry_valid(tap_valid),.telemetry_first(tap_first),.telemetry_last(tap_last),.telemetry_data(tap_data),'
    assert text.count(core_tap) == 1
    text = text.replace(core_tap,
        '        .telemetry_valid(),.telemetry_first(),.telemetry_last(),.telemetry_data(),')
    for name, constant in (('valid', "1'b0"), ('first', "1'b0"),
                           ('last', "1'b0"), ('data', "8'd0")):
        binding = '.control_tap_'+name+'(tap_'+name+')'
        assert text.count(binding) == 1
        text = text.replace(binding, '.control_tap_'+name+'('+constant+')')
    for declaration in ('    wire tap_valid,tap_first,tap_last;\n',
                        '    wire [7:0] tap_data;\n'):
        assert text.count(declaration) == 1
        text = text.replace(declaration, '')
    assert not re.search(r'^`(?:ifdef|define) G2_REAL_SFP', text, re.M)
    assert 'sfp_management u_sfp(' not in text and 'assign sfp1_tx_dis=1;' in text
    assert text.count('.HOST_EXTENSION_ENABLE(1)') == 1
    assert text.count('.telemetry_valid()') == 1
    assert text.count(".control_tap_valid(1'b0)") == 1
    assert 'assign tap_valid=0;' not in text
    for binding in ('assign h_rx_clk=clk;', 'assign h_tx_ready=1;',
                    'assign h_core_rst=rst;', '.UART_TXD(control_txd)',
                    '.BAUD(921600)', '.h_rx_clk(h_rx_clk)', '.h_tx_clk(h_tx_clk)'):
        assert binding in text, ('required UART/idle-host binding missing', binding)
    return text


def prepare(source, reference, out):
    source, reference, out = [Path(p).resolve() for p in (source, reference, out)]
    assert not out.exists() and out != source and source not in out.parents and out not in source.parents
    assert source.name == 'sfp_lane2_private_snapshot_syn11_retry1'
    job, manifest = load(source/'job.json'), load(source/'manifest.json')
    assert job['phase'] == 'synthesis_complete' and job['tool_exit'] == 0
    assert manifest['real_sfp'] and manifest['full_control_telemetry']
    assert manifest['clock_core_mhz'] == 50 and not manifest['control_weights_changed']
    assert (source/'impl/gwsynthesis/g2_top.vg').read_bytes() == (reference/'g2_top.vg').read_bytes()
    hardware = [Path(p).relative_to(source).as_posix() for p in manifest['source_files']]
    assert len(hardware) == len(set(hardware)) == 68 and SFP_SOURCES < set(hardware)
    top = transform_top((source/'g2_top.v').read_text(encoding='utf-8'))
    assets = {p.relative_to(source).as_posix(): p for p in source.rglob('*')
        if p.is_file() and p.suffix.lower() in ASSET_SUFFIXES
        and 'impl' not in p.relative_to(source).parts}
    headers = {p.relative_to(source).as_posix(): p for p in source.rglob('*')
        if p.is_file() and p.suffix.lower() in {'.vh', '.h'}
        and 'impl' not in p.relative_to(source).parts}
    assert len(assets) == 104
    retained = [p for p in hardware if p not in SFP_SOURCES]
    assert len(retained) == 64
    out.mkdir(parents=True)
    files = {name: source/name for name in retained}
    files.update(assets); files.update(headers)
    for name, origin in files.items():
        target = out/name; target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(origin, target)
    shutil.copytree(source/'fw', out/'fw', dirs_exist_ok=True)
    (out/'g2_top.v').write_text(top, encoding='utf-8', newline='\n')
    changed = sorted(name for name in retained if (source/name).read_bytes() != (out/name).read_bytes())
    assert changed == ['g2_top.v']
    assert all(origin.read_bytes() == (out/name).read_bytes() for name, origin in assets.items())
    assert all(origin.read_bytes() == (out/name).read_bytes() for name, origin in headers.items())
    assert not any((out/name).exists() for name in SFP_SOURCES | {'ip/serdes.csr'})
    cst = (reference/'g2.cst').read_text(encoding='utf-8')
    requests = [line for line in cst.splitlines() if line.startswith('CLOCK_LOC ')]
    assert len(requests) == 14
    retained_cst = '\n'.join(line for line in cst.splitlines() if not line.startswith('CLOCK_LOC '))+'\n'
    (out/'g2.cst').write_text(retained_cst, encoding='utf-8', newline='\n')
    assert [x for x in cst.splitlines() if x.startswith(('IO_LOC ', 'IO_PORT '))] == [
        x for x in retained_cst.splitlines() if x.startswith(('IO_LOC ', 'IO_PORT '))]
    reference26 = load(reference/'cdc_reference_audit.json')
    fragment = sdc_fragment(reference26)
    sdc = ('create_clock -name clk50 -period 20 [get_ports {clk50}]\n'
           'create_clock -name rgmii_rxc -period 10 [get_ports {rgmii_rxc}]\n\n'+fragment+'\n\n'
           'report_route_congestion -max_grids 200\n'
           'report_high_fanout_nets -max_nets 100 -min_fanout 100\n')
    assert len(re.findall(r'^set_false_path ', sdc, re.M)) == 26
    assert len(re.findall(r'^set_max_delay ', sdc, re.M)) == 26
    assert not any(x in sdc for x in ('u_sfp/', 'u_neth/', 'h_rx_clk', 'h_tx_clk', 'sfp_rx_clk', 'sfp_tx_clk'))
    (out/'g2.sdc').write_text(sdc, encoding='utf-8', newline='\n')
    shutil.copyfile(reference/'cdc_reference_audit.json', out/'cdc_reference_audit.json')
    old_tcl = (source/'run.tcl').read_text(encoding='utf-8')
    old_lines = old_tcl.splitlines()
    removed_tcl = [line for line in old_lines if line.startswith('set_csr ') or
        (line.startswith('add_file -type verilog ') and any((source/name).as_posix() in line for name in SFP_SOURCES))]
    assert len(removed_tcl) == 5
    new_tcl = relocate('\n'.join(line for line in old_lines if line not in removed_tcl)+'\n', source, out)
    assert [x.strip() for x in new_tcl.splitlines() if x.strip().startswith('run ')] == ['run syn']
    assert 'set_csr ' not in new_tcl and not any(name in new_tcl for name in SFP_SOURCES)
    assert sum(line.startswith('add_file -type verilog ') for line in new_tcl.splitlines()) == 64
    (out/'run.tcl').write_text(new_tcl, encoding='utf-8', newline='\n')
    new_manifest = relocate(copy.deepcopy(manifest), source, out)
    historical_sfp = new_manifest.pop('sfp_lane2_input_checks', None)
    new_manifest.pop('effective_options', None)
    new_manifest.update(stage='syn', source_files=[str(out/name) for name in retained],
        frozen_source_build=str(source), real_sfp=False, full_control_telemetry=True,
        sfp_phy_link_guard=False, host_extension_enable=1, sfp_disabled_diagnostic=True,
        historical_enabled_sfp_input_proof=historical_sfp, programmed=False,
        automatic_pnr=False, deployment_accepted=False, source_preparation_only=True,
        SFP_source_removed=sorted(SFP_SOURCES), sfp_TX_DISABLE_constant=1,
        expected_true_clock_domains=['clk50_d', 'rgmii_rxc_d'],
        known_CDC_constraint_pairs=26, post_SYN_CDC_and_clock_driver_review_pending=True)
    save(out/'manifest.json', new_manifest)
    save(out/'pending_clock_resource_plan.json', dict(constraint_reference=str(reference),
        archived_original_requests=requests, applied_to_candidate=False,
        requires_new_actual_SYN_driver_and_clock_validation=True,
        reason='Removed PHY/125MHz domains may change automatic net names and global allocation.'))
    review = dict(passed=True, review_kind='strict_source_preparation_only',
        source=str(source), constraint_reference=str(reference), prepared=str(out),
        created_at=datetime.now().astimezone().isoformat(), modified_rtl_inputs=changed,
        excluded_hardware_inputs=sorted(SFP_SOURCES), retained_hardware_inputs=64,
        unchanged_retained_hardware_inputs=63, unchanged_assets=104,
        unchanged_headers=len(headers), exact_source_transform_rechecked=True,
        control4500_and_full_V2_UART_source_byte_identical=True,
        complete_navigation_CPU128KiB_policy_weights_capacity_and_stop_sources_byte_identical=True,
        HOST_EXTENSION_ENABLE=1, control_tap_SFP_copy_disconnected_only=True,
        NAV_control_tap_inputs_constant_zero=True,
        original_control_tap_output_ports_explicitly_unconnected=True,
        physical_SFP_MAC_PCS_SerDes_disabled=True, sfp_TX_DISABLE_constant=1,
        host_stub_clocks_equal_core_and_idle_RX=True, host_TX_ready_idle_stub=True,
        host_MBX_TX_shadow_RX_injection_console_and_UART_debug_retained=True,
        original_true_IO_constraints_preserved=True, removed_SFP_125MHz_clock_constraints=2,
        removed_host_hold_only_and_max_delay_pairs=51, lidar26_pairs_regenerated=True,
        original_generated_CLOCK_LOC_requests_archived=14, automatic_CLOCK_LOC_names_not_applied=True,
        source_bound_behavioral_UART_and_full_NV_response_proof_pending=True,
        actual_new_RAM_INIT_clocks_CDC_and_driver_precheck_pending=True,
        no_synthesis_or_PNR_started=True, ready_for_PNR=False,
        programmed=False, deployment_accepted=False)
    save(out/'prepared_source_review.json', review)
    return review


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--reference', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    result = prepare(args.source, args.reference, args.out)
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
