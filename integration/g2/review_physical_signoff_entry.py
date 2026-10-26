"""Read existing Gowin reports and installed manuals; never invoke EDA or edit SDC.

This is an evidence collector, not a deployment gate. Manual support, reported
rows, external-device assumptions, and unverified coverage remain separate.
"""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime
import json
from pathlib import Path
import re

WORK = Path('F:/nav_integration_g2_20261023')
IDE = Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE')
BOARD = Path('D:/Desktop/SOC FPGA/小梅哥ACG750型国产高云GW5AT-138K FPGA开发板资料')


def read(p):
    return Path(p).read_text(encoding='utf-8', errors='replace')


def section(text, title, following):
    start = text.rfind(title)
    if start < 0:
        return ''
    end = re.search(following, text[start + len(title):])
    return text[start:] if not end else text[start:start + len(title) + end.start()]


def report_rows(text, name, title, following):
    part = section(text, title, following)
    command = re.search(r'(?:<Report Command>|Report Command):([^\n]+)', part)
    rows = []
    for line in part.splitlines():
        if name == 'minimum_clock_pulse_width':
            m = re.match(r'^\s*(\d+)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+(High|Low) Pulse Width\s+(\S+)\s+(\S+)\s*$', line)
            if m:
                rows.append(dict(number=int(m[1]), slack_ns=float(m[2]),
                    actual_width_ns=float(m[3]), required_width_ns=float(m[4]),
                    type=m[5] + ' Pulse Width', clock=m[6], object=m[7]))
        else:
            x = line.split()
            if len(x) == 9 and x[0].isdigit():
                try:
                    rows.append(dict(number=int(x[0]), slack_ns=float(x[1]),
                        from_pin=x[2], to_pin=x[3], from_clock=x[4], to_clock=x[5],
                        relation_ns=float(x[6]), skew_ns=float(x[7]), data_delay_ns=float(x[8])))
                except ValueError:
                    pass
    return dict(available=bool(part), report_command=command[1].strip() if command else None,
        reported=len(rows), worst_reported_slack_ns=min((x['slack_ns'] for x in rows), default=None),
        negative_reported_rows=sum(x['slack_ns'] < 0 for x in rows),
        unique_reported_endpoints=len({x.get('to_pin', x.get('object')) for x in rows}),
        empty_is_not_pass=not bool(rows), rows=rows,
        complete_endpoint_coverage_proved=False)


def refs(manual_root):
    return {
        'timing': dict(path=str(manual_root / 'SUG940-2.0.5E_Gowin Design Timing Constraints User Guide.pdf'),
            pages=dict(single_sdc=8, io_overview=[21, 22], path_coverage=[39, 40, 41, 42, 43],
                recovery=47, removal=48, minimum_clock_pulse=[49, 50], constraint_state=54,
                collections=55, io_syntax=[65, 66, 67, 68], report_timing=[73, 74, 75, 76],
                reset_fanout=[76, 77], congestion=[77, 78], report_min_pulse_width=[78, 79],
                exceptions=[80, 81, 82], corners=82), pdf_page_offset=10),
        'tcl': dict(path=str(manual_root / 'SUG1220-2.1E_Gowin Software Tcl Commands User Guide.pdf'),
            printed_pages_run=[16, 17], pdf_pages_run=[23, 24]),
        'software': dict(path=str(manual_root / 'SUG100-4.4.7E_Gowin Software User Guide.pdf'),
            printed_page_timing_analysis_report=80, pdf_page=92),
    }


def io_inventory(rtl):
    # This inventory is explicitly bound to the frozen SYN23 source of PNR35.
    # Later designs must bind their own port/clock/CDC inventory independently.
    paths = {k: str(rtl / v) for k, v in {
        'top': 'g2_top.v', 'mac': 'nav/rtl/net/rgmii_mac100.v',
        'uart_bridge': 'nav/rtl/io/uart_bus_bridge.v', 'nav_cdc': 'nav/rtl/common/nav_cdc.v',
        'imu_uart': 'nav/rtl/io/imu_uart.v', 'uart_speed_rx': 'control/uart_speed_rx.v',
        'i2c': 'control/oled_driver.v', 'bmi088': 'control/bmi088.v',
        'encoder': 'control/motor520_speed.v', 'servo': 'control/servo_driver.v',
        'motor': 'control/motor520_closed_loop.v', 'ir': 'control/ir_remote_control.v',
    }.items()}
    markers = {
        'top': ['.BAUD(921600)', ".UART_RXD(1'b1)", '.imu_rx(imu_rxd)'],
        'mac': ['assign rx_clk = rgmii_rxc', 'posedge rgmii_rxc', 'negedge rgmii_rxc',
                'negedge clk', 'mdio_i', 'MDIV'],
        'uart_bridge': ['nav_sync2'], 'imu_uart': ['nav_sync2'],
        'uart_speed_rx': ['rx_meta<=rx;rx_sync<=rx_meta'],
        'i2c': ["assign IIC_SCL = scl_low ? 1'b0 : 1'bz", 'sda_sync_1<=sda_sync_0',
                'scl_sync_1<=scl_sync_0', 'read_data<={read_data[6:0],sda_sync_1}'],
        'encoder': ['sync_a<={sync_a[1:0],enc_a};sync_b<={sync_b[1:0],enc_b}'],
    }
    checks = {}
    locations = {}
    for key, needles in markers.items():
        lines = read(paths[key]).splitlines()
        for needle in needles:
            hits = [n for n, line in enumerate(lines, 1) if needle in line]
            checks[f'{key}:{needle}'] = bool(hits)
            locations[f'{key}:{needle}'] = hits
    phy = str(BOARD / '器件手册/Ethernet/RTL8211F-CG.pdf')
    board = str(BOARD / '硬件图纸/ACX750-GW5A138-CB-250928.pdf')
    groups = [
        dict(group='RGMII_RX_TX_and_MDIO',
            ports=['rgmii_rxc', 'rgmii_rx_ctl', 'rgmii_rxd[3:0]', 'rgmii_txc',
                   'rgmii_tx_ctl', 'rgmii_txd[3:0]', 'eth_mdc', 'eth_mdio', 'eth_rst_n'],
            confirmed_source_relationship=[
                'RXC is PHY-sourced; RX data/control sampled on both edges and selected using synchronized rx_neg.',
                'Frozen MAC is explicitly 100BASE-TX only; PHY datasheet gives 25MHz nominal RXC/TXC in that mode.',
                'TXC derives from core clk; at CLK_HZ=50MHz, phase register updates on negedge clk.',
                'MDIO is sampled by the internally divided MDC transaction state; not a free UART input.'
            ],
            available_official_docs=[dict(path=phy, printed_pages=[18, 60, 61, 62, 63],
                pdf_pages=[25, 67, 68, 69, 70]), dict(path=board, sheet='02_ETH', pdf_page=2)],
            required_before_delay_values=[
                'Actual PHY speed/negotiation, RX sample edge, fitted RXDLY/TXDLY straps and MDIO delay configuration readback.',
                'PCB data-vs-clock trace delays/skew and PVT/mode-correct PHY AC timing.',
                'Actual generated TXC relationship at output; coherent min/max and both-edge receiver budgets.',
                'MDIO input/output delay versus MDC, turnaround/tri-state and pull-up RC; PHY reset minimum and power-up sequence.'
            ],
            official_device_values_not_board_budgets=dict(
                mdc_period_min_ns=80, mdc_high_low_min_ns=32,
                mdio_setup_min_ns=10, mdio_hold_min_ns=10, mdio_valid_after_mdc_rise_ns=[0, 300],
                table58_100base_clock_period_ns=[36, 40, 44]),
            current_delay_values_verified=False,
            caution='PNR35 rgmii_rxc 100MHz is the configured target, not measured RXC frequency or an established 10ns external timing budget.'),
        dict(group='asynchronous_UART', ports=['uart_rxd', 'uart_txd', 'imu_rxd', 'control_txd'],
            confirmed_source_relationship=[
                'Top-level uart_bus_bridge is 921600 baud and nav_sync2 samples RX into core clk.',
                'PNR35 imu_rxd feeds the original navigation imu_uart; its name is historical, and the P11 A15 role must be checked against selected firmware.',
                "PNR35 control UART_RXD is tied to 1; uart_speed_rx exists in source but is not this external input path.",
                'control_txd is the separate control telemetry output. Do not infer the current deployment protocol from PNR35 alone.'
            ], available_official_docs=[dict(path=board, sheet='08_Uart&I2C', pdf_page=8,
                schematic_component='U19 CH9102F; VIO tied to VCC3P3',
                fitted_current_board_revision_verified=False)], required_docs=[
                'CH9102F manufacturer electrical/baud tolerance datasheet; actual fitted board must match the dated schematic.',
                'USB-TTL module chip/electrical/baud tolerance and actual RK port settings/cable path.',
                'Selected UART protocol and allowed baud error/minimum input pulse; actual first/second synchronizer placement and resolution time.'
            ], current_delay_values_verified=False,
            caution='No synchronous launch-clock phase exists for an asynchronous serial RX. Do not invent set_input_delay=0 or cut the second synchronizer stage.'),
        dict(group='sensors_and_user_inputs',
            ports=['BMI088_SCL', 'BMI088_SDA', 'sck', 'sda', 'M1_ENCA', 'M1_ENCB', 'M2_ENCA', 'M2_ENCB', 'IRIN', 'key_rst_n'],
            confirmed_source_relationship=[
                'BMI088 and OLED use separate open-drain I2C pins; shared iic RTL has two-stage SCL/SDA synchronizers.',
                'At frozen 50MHz and QUARTER_CYCLES=125, four nominal quarter phases give 100kHz before clock-stretch/line-blocked extension.',
                'Encoder A/B each enter three-register sampling chains; IR has a two-register sampling chain plus edge history.',
                'key_rst_n asynchronously asserts reset, then nav_rst_sync releases separately in the receiving clocks.'
            ], available_docs=[str(BOARD / '器件手册/IR/hs38b中文资料.pdf'),
                'D:/Desktop/SOC FPGA/nav/control/fpga/md/BMI088_OLED.md'],
            required_docs=[
                'Bosch BST-BMI088-DS001 device I2C AC timing plus exact sensor-module schematic and measured pull-up voltage/RC/cable capacitance.',
                'Exact OLED controller/module datasheet (not identified by current project note), I2C timing and pull-ups.',
                'Exact 520 encoder electrical and max-frequency/min-pulse specification, A/B edge separation and cable/conditioning.',
                'IR receiver output timing and board reset/key circuit; actual asynchronous reset pulse and deassertion phase inventory.'
            ], current_delay_values_verified=False,
            caution='Clock/FF MPW does not establish off-chip I2C pulse/edge timing, encoder sampling coverage, IR pulse coverage, or asynchronous reset assertion width.'),
        dict(group='actuators_and_indicators',
            ports=['SERVO1', 'SERVO2', 'SERVO3', 'SERVO4', 'M1_PWM', 'M1_DIR1', 'M1_DIR2', 'M2_PWM', 'M2_DIR1', 'M2_DIR2', 'led', 'led_cpu', 'led_trap'],
            confirmed_source_relationship=[
                'Servo source defines a 20ms frame with nominal 500..2500us pulse range and frame-boundary capture.',
                'Motor source defines 2500 core-cycle carrier and one whole carrier-period coast on sign reversal; reset/enable gate outputs.',
                'At frozen 50MHz, 2500-cycle carrier is nominal 20kHz. This is source behavior, not a measured actuator input timing result.'
            ], required_docs=[
                'Exact servo models: allowed voltage, min/max high/low pulses, frame frequency, mechanical limits and power sequencing.',
                'Exact H-bridge/driver model: PWM/DIR input timing, required direction deadtime, enable/fault behavior and electrical limits.',
                'Actual stop path, output-safe state, external enable/power cut and supported-vehicle stopping measurements.'
            ], current_delay_values_verified=False,
            caution='These pulse/protocol outputs do not gain a valid external synchronous deadline by assigning an arbitrary output clock. Internal STA, protocol pulse checks and physical safety acceptance are separate.'),
    ]
    return dict(reference_rtl=str(rtl), binding='PNR35 frozen SYN23 only; later candidate ports/CDC must be rebound',
        source_paths=paths, marker_checks=checks, marker_lines=locations,
        all_marker_checks_passed=all(checks.values()), groups=groups)


def collect(args):
    pnr = Path(args.pnr)
    t = read(pnr / 'impl/pnr/g2_top.tr')
    sdc = read(pnr / 'g2.sdc')
    commands = [x.strip() for x in sdc.splitlines() if x.strip() and not x.lstrip().startswith('#')]
    rows = {name: report_rows(t, name, title, following) for name, title, following in [
        ('recovery', '3.1.3 Recovery Paths Table', r'3\.1\.4 '),
        ('removal', '3.1.4 Removal Paths Table', r'3\.2 '),
        ('minimum_clock_pulse_width', '3.2 Minimum Pulse Width Table', r'3\.3 '),
    ]}
    constraints = section(t, '3.8 Timing Constraints Report', r'\Z')
    active = re.findall(r'^\s*(TC_\w+)\s+(Actived|Invalid)\s+(.+?)\s*$', constraints, re.M)
    refs_ = refs(Path(args.manual_root))
    for v in refs_.values():
        v['exists'] = Path(v['path']).is_file()
    r = dict(schema='gowin_physical_signoff_entry_review_v1', collected_local_time=datetime.now().isoformat(),
        read_only_inputs=True, eda_invocations=0, current_running_input_changes=0,
        deployment_signoff_passed=False, manuals=refs_,
        report_entry=dict(text=str(pnr / 'impl/pnr/g2_top.tr'),
            html=str(pnr / 'impl/pnr/g2_top.tr.html'),
            html_content=str(pnr / 'impl/pnr/g2_top_tr_content.html'),
            gui='Process View -> Timing Analysis Report',
            existing_pnr_flags='-tr -timing',
            standalone_routed_database_STA_command_verified=False,
            standalone_reason='SUG1220 documents run syn/pnr/all only; no verified STA-only load-and-report entry was found. No PNR was run to query reports.'),
        supported_reporting=dict(
            report_timing=dict(analysis_flags=['-setup', '-hold', '-recovery', '-removal'],
                path_controls=['-max_paths <positive_int>', '-max_common_paths <positive_int>'],
                objects_from_to=['get_clocks', 'get_ports', 'get_regs', 'get_pins'],
                objects_through=['get_nets', 'get_pins'], default_max_paths=25, default_max_common_paths=1),
            report_min_pulse_width=dict(syntax='report_min_pulse_width -nworst N [-detail] [get_regs {...}]',
                default_nworst=10, object_filter='get_regs',
                min_max_filters_apply_to='actual pulse width, not slack',
                verified_PNR35_subject='clock early/late paths to CLK/CLKA; not arbitrary asynchronous reset assertion pulses'),
            report_exceptions=dict(analyses=['-setup', '-hold', '-recovery', '-removal'],
                purpose='Audit exception-covered/excluded paths alongside normal analysis; do not add new exceptions.'),
            report_route_congestion=dict(syntax='report_route_congestion -max_grids N [-min_route_congestion x] [-max_route_congestion y] [-LOC R[a:b]C[c:d]]',
                bounds='congestion filters in [0,1]', purpose='Reported congestion grid, not failed-net identity',
                runtime_GW5AT_C_verified=False, failed_route_output_available_verified=False),
            report_reset_fanout=dict(syntax='report_high_fanout_nets -max_nets N -slr',
                slr='Clear, Set, Reset, Preset', runtime_GW5AT_C_verified=False)),
        output_file_and_coexistence=dict(
            independent_output_filename_flag_documented=False,
            report_timing_file_append_return_string_flags_documented=False,
            report_min_pulse_width_file_append_flags_documented=False,
            not_documented_does_not_prove_all_undocumented_flags_impossible=True,
            default_different_analysis_sections_coexist_in_PNR35=True,
            repeated_user_same_analysis_overwrite_or_append_verified=False,
            proposed_next_natural_run_policy='Keep exactly one global report_timing statement per analysis type and one MPW command in the same existing SDC; inspect actual Report Command/row and unique-endpoint coverage. Extract independent files after collection.',
            single_sdc_reason='Installed SUG940 p8: last-added SDC takes effect by default. Do not add a second report-only SDC.'),
        future_same_SDC_report_block_not_applied=[
            'report_timing -setup -max_paths 5000 -max_common_paths 1',
            'report_timing -hold -max_paths 5000 -max_common_paths 1',
            'report_timing -recovery -max_paths 5000 -max_common_paths 1',
            'report_timing -removal -max_paths 5000 -max_common_paths 1',
            'report_min_pulse_width -nworst 5000 -detail',
            'report_high_fanout_nets -max_nets 100 -slr',
            'report_route_congestion -max_grids 100'],
        PNR35=dict(pnr=str(pnr), sdc=str(pnr / 'g2.sdc'), sections=rows,
            active_constraint_types=dict(Counter(x[0] for x in active if x[1] == 'Actived')),
            invalid_constraint_rows=[x for x in active if x[1] == 'Invalid'],
            sdc_command_count=len(commands), sdc_IO_delay_count=sum(bool(re.match(r'set_(input|output)_delay\b', x)) for x in commands),
            active_IO_delay_count=sum('set_input_delay' in x[2] or 'set_output_delay' in x[2] for x in active),
            constraint_actived_means='Tool accepted/applied statement; does not prove correct external numbers or complete coverage.',
            clock_relationship_caution='Worst recovery crosses clk50 -> rgmii_rxc under a 10ns assumed relation. This does not sign off arbitrary asynchronous phase.',
            full_recovery_removal_coverage_proved=False, asynchronous_reset_assertion_pulse_width_proved=False,
            IO_signoff_proved=False),
        IO=io_inventory(Path(args.reference_rtl)),
        collector_semantic_gap=dict(path='D:/Desktop/SOC FPGA/nav/integration/g2/collect_pnr_result.py',
            field='reported_reset_pulse_checks_pass', production_code_modified=False,
            limitations=['Existing boolean checks section presence and negative printed row count; an empty section can satisfy those tests.',
                'Clock MPW under that flag is not proof of asynchronous reset pulse width.',
                'Nonnegative reported rows remain useful evidence but do not establish complete endpoint/corner coverage.']),
        required_unverified=['Actual latest candidate endpoint/clock inventory and all applied/excluded reset checks.',
            'Reset assertion min width at actual asynchronous control pins, release under real clock relation, cold start and stopped-clock behavior.',
            'All relevant PVT corners/grades, physical CDC stable payload, second-stage resolution/MTBF.',
            'External IO min/max budgets and both-edge/source-synchronous relationships, plus electrical/pulse protocol checks.',
            'Whether failed routing can emit the official congestion report and how user repeated same-type reports are stored.'])
    return r


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--pnr', default=str(WORK / 'baseline_fusion32_CDC52_pnr35'))
    p.add_argument('--reference-rtl', default=str(WORK / 'baseline_fusion_only_syn23'))
    p.add_argument('--manual-root', default=str(IDE / 'doc/EN'))
    p.add_argument('--out', default=str(WORK / 'physical_signoff_entry_review_20261026'))
    args = p.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    r = collect(args)
    (out / 'final_review.json').write_text(json.dumps(r, ensure_ascii=False, indent=2), encoding='utf-8')
    for name, part in r['PNR35']['sections'].items():
        (out / (name + '_reported_rows.json')).write_text(json.dumps(part, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(dict(result=str(out / 'final_review.json'),
        source_markers_passed=r['IO']['all_marker_checks_passed'],
        reported={k: dict(rows=v['reported'], worst_ns=v['worst_reported_slack_ns']) for k, v in r['PNR35']['sections'].items()},
        active_constraints=r['PNR35']['active_constraint_types'],
        IO_delays=r['PNR35']['active_IO_delay_count'], deployment_signoff_passed=False), ensure_ascii=False))


if __name__ == '__main__':
    main()
