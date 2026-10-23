"""Reproducible joint original4500 / navigation resource build; never programs.

Both variants retain the real policy ROM, BMI, motors, servos and nav feedback.
G2 is a resource gate, not complete G3 attitude integration or vehicle acceptance.
"""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'sim'))
from tests import ALL_RTL,SOC


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--nav',choices=['as','coverage'],required=True)
    ap.add_argument('--stage',choices=['syn','all'],default='syn')
    ap.add_argument('--packing',choices=['none','nav','control','both','ssram','hybrid','planes','directed'],default='none')
    ap.add_argument('--command-lease',action='store_true',help='isolated command-age/stop gate, not accepted vehicle behavior')
    ap.add_argument('--observation-pipeline',action='store_true',help='cycle-exact wheel numerator timing candidate; original source remains intact')
    ap.add_argument('--command-units-pipeline',action='store_true',help='exact units/sequence adapter pipeline before lease guard')
    ap.add_argument('--coverage-static-bounds',action='store_true',help='cycle-exact precomputation of frame-static coverage bounds')
    ap.add_argument('--fusion-norm-pipeline',action='store_true',help='cycle-exact original-square / sqrt-input register rebalance')
    ap.add_argument('--pose-predecode',action='store_true',help='cycle-exact POSE_COMMIT bus capture predecode')
    ap.add_argument('--action-map-exact-widths',action='store_true',help='exact private gain arithmetic widths, no quantization or latency change')
    ap.add_argument('--packed-weight9',action='store_true',help='lossless weight bitplanes in native9 modes; no weight/cycle change')
    ap.add_argument('--gyro-sum-exact-widths',action='store_true',help='exact live unsigned17 raw gyro sum, no gate/parameter/cycle change')
    ap.add_argument('--trig-block-rom',action='store_true',help='initialized navigation lookup tables in native BSRAM; values and read cycle unchanged')
    ap.add_argument('--snapshot-mux-fanout',action='store_true',help='cycle-exact snapshot update with bounded data-mux selector fanout; CE/reset unchanged')
    ap.add_argument('--history-data-fanout',action='store_true',help='bound the observation initialized data-mux selector to 128 loads; attribute only')
    ap.add_argument('--real-sfp',action='store_true',help='real Q0 SFP MAC/PCS/SerDes plus read-only full UART V2 telemetry')
    ap.add_argument('--prepare-only',action='store_true',help='freeze/preflight inputs only; run with bounded frozen synthesis supervisor')
    ap.add_argument('--wide-split',type=int,choices=[0,1],default=0,help='explicit PNR wide-RAM split; 1 duplicates every 32/36-bit block')
    a=ap.parse_args()
    if a.command_units_pipeline and not a.command_lease:ap.error('--command-units-pipeline requires --command-lease')
    if a.packed_weight9 and a.packing not in ('control','both','ssram','hybrid','directed'):ap.error('--packed-weight9 requires a control storage layout')
    if a.trig_block_rom and a.packing!='directed':ap.error('--trig-block-rom requires directed storage')
    out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
    nav=out/'nav';nav.mkdir()
    shutil.copytree(ROOT/'rtl',nav/'rtl')
    if a.pose_predecode:
        from prepare_pose_predecode import generate
        generate(nav/'rtl/top/nav_subsystem.v')
    if a.coverage_static_bounds:
        from prepare_coverage_timing import generate
        generate(nav/'rtl/map/support_coverage.v')
    if a.packing in ('nav','both'):
        shutil.copyfile(ROOT/'integration/g2/nav_ram_sdp_banked.v',nav/'rtl/common/nav_ram_sdp.v')
    elif a.packing=='ssram':
        shutil.copyfile(ROOT/'integration/g2/nav_ram_sdp_ssram.v',nav/'rtl/common/nav_ram_sdp.v')
    elif a.packing=='hybrid':
        shutil.copyfile(ROOT/'integration/g2/nav_ram_sdp_hybrid.v',nav/'rtl/common/nav_ram_sdp.v')
        shutil.copyfile(ROOT/'integration/g2/nav_ram_banked_hybrid.v',nav/'rtl/common/nav_ram_banked.v')
    elif a.packing=='planes':
        shutil.copyfile(ROOT/'integration/g2/nav_ram_sdp_planes.v',nav/'rtl/common/nav_ram_sdp.v')
    elif a.packing=='directed':
        for name in ('nav_ram_sdp','nav_ram_sp','nav_ram_banked'):
            shutil.copyfile(ROOT/f'integration/g2/{name}_directed.v',nav/f'rtl/common/{name}.v')
    if a.trig_block_rom:
        from prepare_trig_block_rom import generate
        generate(nav/'rtl/common/nav_ram_sdp.v')
    fw=out/'fw';shutil.copytree(ROOT/'fw',fw,ignore=shutil.ignore_patterns('build','__pycache__'))
    if a.nav=='coverage':
        for p in fw.rglob('*'):
            if p.suffix in ('.c','.h'):p.write_bytes(p.read_text(encoding='utf-8').encode('utf-8'))
        patch=ROOT/'sim/closed_loop/candidates/n2_support_coverage.patch'
        cmd=['git','apply','--unsafe-paths','--directory='+out.as_posix()]
        subprocess.run(cmd+['--check',str(patch)],cwd=ROOT,check=True)
        subprocess.run(cmd+[str(patch)],cwd=ROOT,check=True)
    with (out/'firmware.log').open('w',encoding='utf-8') as log:
        subprocess.run([sys.executable,'-X','utf8',str(fw/'bsp/build_rv.py'),'--output-dir',str(out/'firmware'),
                        '--copy-to',str(out),'--ram-kb','128'],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,check=True)
    control=ROOT/'control/fpga/src'
    shutil.copytree(control/'mem',out/'src/mem')
    shutil.copyfile(ROOT/'rtl/plan/sin_lut.hex',out/'sin_lut.hex')
    top=(ROOT/'integration/g2/g2_top.v').read_text(encoding='utf-8')
    top=top.replace('`include "soc_cfg.vh"','`define NAV_SOC_RAM_KB 128\n`define G2_COVERAGE '+str(int(a.nav=='coverage')))
    if a.real_sfp:top='`define G2_REAL_SFP\n'+top
    if a.command_lease:top='`define G2_COMMAND_LEASE\n'+top
    if a.command_units_pipeline:top='`define G2_UNITS_PIPELINE\n'+top
    (out/'g2_top.v').write_text(top,encoding='utf-8')
    shutil.copyfile(ROOT/'integration/g2/g2.cst',out/'g2.cst')
    shutil.copyfile(ROOT/'integration/g2/g2.sdc',out/'g2.sdc')
    from prepare_control import prepare as prepare_control
    prepare_control(out)
    if a.real_sfp:
        from prepare_telemetry_tap import prepare as telemetry_tap
        telemetry_tap(out)
    if a.observation_pipeline:
        import importlib.util
        spec=importlib.util.spec_from_file_location('observation_pipeline',ROOT/'control/tools/g2_observation_pipeline.py')
        generator=importlib.util.module_from_spec(spec);spec.loader.exec_module(generator)
        generator.generate(out/'control/rl_observation_v5.v')
    if a.fusion_norm_pipeline:
        import importlib.util
        spec=importlib.util.spec_from_file_location('fusion_pipeline',ROOT/'control/tools/g2_fusion_pipeline.py')
        generator=importlib.util.module_from_spec(spec);spec.loader.exec_module(generator)
        generator.generate(out/'control/imu_fusion.v')
    if a.action_map_exact_widths:
        import importlib.util
        spec=importlib.util.spec_from_file_location('action_widths',ROOT/'control/tools/g2_action_map_widths.py')
        generator=importlib.util.module_from_spec(spec);spec.loader.exec_module(generator)
        generator.generate(out/'control/policy_action_map.v')
    if a.gyro_sum_exact_widths:
        import importlib.util
        spec=importlib.util.spec_from_file_location('gyro_widths',ROOT/'control/tools/g2_gyro_sum_widths.py')
        generator=importlib.util.module_from_spec(spec);spec.loader.exec_module(generator)
        generator.generate(out/'control/imu_fusion.v')
    if a.snapshot_mux_fanout:
        import importlib.util
        spec=importlib.util.spec_from_file_location('snapshot_fanout',ROOT/'control/tools/g2_snapshot_fanout.py')
        generator=importlib.util.module_from_spec(spec);spec.loader.exec_module(generator)
        generator.generate(out/'control/rl_policy_tick_v5.v')
    if a.history_data_fanout:
        import importlib.util
        spec=importlib.util.spec_from_file_location('history_fanout',ROOT/'control/tools/g2_history_fanout.py')
        generator=importlib.util.module_from_spec(spec);spec.loader.exec_module(generator)
        generator.generate(out/'control/rl_observation_v5.v')
    shutil.copyfile(ROOT/'integration/g2/command_lease_guard.v',out/'command_lease_guard.v')
    shutil.copyfile(ROOT/'integration/g2/nav_command_units.v',out/'nav_command_units.v')
    files=[nav/x for x in ALL_RTL+SOC+['rtl/net/rgmii_mac100.v']]
    if nav/'rtl/net/host_control_tx.v' not in files:files.append(nav/'rtl/net/host_control_tx.v')
    files += [p for p in (out/'control').glob('*.v') if p.name!='main.v']
    files += [out/'control4500_core.v',out/'command_lease_guard.v',out/'nav_command_units.v',out/'g2_top.v']
    if a.packing in ('control','both','ssram','hybrid','directed'):
        from optimize_storage import prepare
        prepare(out,directed=a.packing=='directed',packed9=a.packed_weight9)
        files=[p for p in files if p!=out/'control/fc_v4_top.v']
        files += [out/'fc_v4_top.v',out/'fixed4500_weight_rom.v']
    if a.real_sfp:
        import importlib.util
        spec=importlib.util.spec_from_file_location('joint_sfp_ip',ROOT/'integration/sfp/generate_ip.py')
        generator=importlib.util.module_from_spec(spec);spec.loader.exec_module(generator)
        ip=generator.generate(out/'ip')
        for origin in ('integration/sfp/sfp_host_mac.v','integration/g2/sfp_management.v'):
            dest=out/Path(origin).name;shutil.copyfile(ROOT/origin,dest);files.append(dest)
        files += [ip/'serdes.v',ip/'giga_serial_ethernet.v']
        with (out/'g2.sdc').open('a',encoding='utf-8',newline='\n') as f:
            f.write('\ncreate_clock -name sfp_rx_clk -period 8.000 [get_pins {u_sfp/u_mac/u_serdes/gtr12_quad_inst0/LANE0_PCS_RX_O_FABRIC_CLK}]\n')
            f.write('create_clock -name sfp_tx_clk -period 8.000 [get_pins {u_sfp/u_mac/u_serdes/gtr12_quad_inst0/LANE0_PCS_TX_O_FABRIC_CLK}]\n')
    tcl=['set_device -device_version C GW5AT-LV138PG484AC1/I0']
    if a.real_sfp:tcl += [f'set_csr {{{(out/"ip/serdes.csr").as_posix()}}}']
    tcl += [f'add_file -type verilog {{{p.as_posix()}}}' for p in files]
    tcl += [f'add_file -type cst {{{(out/"g2.cst").as_posix()}}}',
            f'add_file -type sdc {{{(out/"g2.sdc").as_posix()}}}',
            f'set_option -include_path {{{(nav/"rtl/regs").as_posix()}}}',
            'set_option -top_module g2_top','set_option -output_base_name g2_top',
            'set_option -verilog_std v2001','set_option -global_freq 50','set_option -print_all_synthesis_warning 1',
            'set_option -looplimit 100000','set_option -convert_sdp32_36_to_sdp16_18 '+str(a.wide_split)]
    tcl += ['set_option -'+x+' 1' for x in ['use_sspi_as_gpio','use_mspi_as_gpio','use_ready_as_gpio','use_done_as_gpio','use_i2c_as_gpio','use_cpu_as_gpio']]
    tcl += ['set_option -place_option 1','set_option -route_option 0']
    tool=os.environ.get('GW_SH',r'D:\Gowin2\Gowin_V1.9.12.04_x64\IDE\bin\gw_sh.exe')
    from preflight import validate
    effective=validate(tool,out,tcl,50,a.wide_split)
    tcl += ['run '+a.stage,'exit']
    (out/'run.tcl').write_text('\n'.join(tcl)+'\n',encoding='utf-8')
    manifest=dict(nav_variant=a.nav,coverage_enabled=a.nav=='coverage',stage=a.stage,packing=a.packing,command_lease=a.command_lease,
        observation_pipeline=a.observation_pipeline,command_units_pipeline=a.command_units_pipeline,
        coverage_static_bounds=a.coverage_static_bounds,wide_split=a.wide_split,
        fusion_norm_pipeline=a.fusion_norm_pipeline,
        pose_predecode=a.pose_predecode,
        action_map_exact_widths=a.action_map_exact_widths,
        packed_weight9=a.packed_weight9,
        gyro_sum_exact_widths=a.gyro_sum_exact_widths,
        trig_block_rom=a.trig_block_rom,
        snapshot_mux_fanout=a.snapshot_mux_fanout,history_data_fanout=a.history_data_fanout,
        real_sfp=a.real_sfp,full_control_telemetry=a.real_sfp,
        sfp_phy_link_guard=a.real_sfp,
        complete_sfp_cdc_acceptance=False,
        device='GW5AT-LV138PG484AC1/I0',device_version='C',clock_core_mhz=50,synthesis_frequency_hint_mhz=50,
        control_commit='6443f5678b4de222c22cbf331b8396f7c8df6a41',control_source=str(control),control_adapter='control_nav_adapter.patch',control_weights_changed=False,
        source_files=[str(p) for p in files],programmed=False,
        effective_options=effective,scope='G2 resource gate, not complete G3 nav IMU adaptation or real vehicle acceptance')
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    print('joint build prepared: '+str(out),flush=True)
    if a.prepare_only:return 0
    with (out/'gw_sh.log').open('w',encoding='utf-8') as log:
        result=subprocess.run([tool,'run.tcl'],cwd=out,stdout=log,stderr=subprocess.STDOUT)
    text=(out/'gw_sh.log').read_text(encoding='utf-8',errors='replace')
    errors=[line for line in text.splitlines() if 'ERROR' in line]
    print('\n'.join(errors[:15]) if errors else text[-1000:])
    print('Gowin process return code: '+str(result.returncode))
    if a.stage=='all':
        from collect_pnr_result import finalize_gate
        gate=finalize_gate(out)
        return result.returncode or bool(errors) or not gate
    return result.returncode or bool(errors)


if __name__=='__main__':raise SystemExit(main())
