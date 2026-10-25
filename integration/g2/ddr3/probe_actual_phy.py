"""Run the unmodified current-C vendor simulation netlist and protected DQS.

The local Micron model is a functional/protocol surrogate, not a claim about
the fitted Longsys parts, board wiring or an unconditional service bound.
Licensing is passed to child processes only; the license is never copied.
"""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import subprocess

HERE = Path(__file__).resolve().parent


def run(args):
    out = args.out.resolve()
    assert not out.exists(), 'preserve previous runs; choose a new directory'
    phy = args.phy.resolve()
    vo = phy / 'ip/nav_ddr3_phy/nav_ddr3_phy.vo'
    ipc = vo.with_suffix('.ipc')
    text = ipc.read_text()
    assert 'SIM=false' in text, 'hardware IP must not use accelerated SIM mode'
    params = phy / 'ip/nav_ddr3_phy/temp/DDR3/gwmc_param.v'
    import re
    for key, value in dict(DQ_WIDTH=32, DRAM_WIDTH=16, ROW_WIDTH=15,
                           COL_WIDTH=10, nCK_PER_CLK=4, CL=6, CWL=5,
                           tCK=2500, tRFC=260000, tREFI=3900000).items():
        assert re.search(r'parameter\s+'+key+r'\s*=\s*'+str(value)+r'\s*;',
                         params.read_text()), key
    assert 'Device Version: C' in vo.with_name('nav_ddr3_phy_tmp.v').read_text()
    assert args.license_file.is_file()
    assert args.pll.is_file()
    model = args.model_dir.resolve()
    assert (model/'ddr3_model.sv').is_file()
    assert (model/'ddr3_model_parameters.vh').is_file()
    prim = args.simlib.resolve()
    assert (prim/'prim_sim.v').is_file()
    dqs = args.dqs_model.resolve() if args.dqs_model else prim/'DQS_sim_M.vp'
    assert dqs.is_file()
    import psutil
    active = [p.pid for p in psutil.process_iter(['name'])
              if (p.info['name'] or '').lower() in ('vsim.exe', 'vsimk.exe')]
    assert not active, 'preserve the existing single licensed runtime: '+str(active)
    out.mkdir(parents=True)
    wrapper = '\n'.join([
        '`include "'+(prim/'prim_sim.v').as_posix()+'"',
        '`timescale 1ns/1ps',
        '`include "'+dqs.as_posix()+'"',
        '`timescale 1ns/1ps',
        '`include "'+args.pll.resolve().as_posix()+'"', ''])
    (out/'vendor_lib.v').write_text(wrapper)
    tb = (HERE/'tb_actual_phy.sv').read_text()
    tb = tb.replace('parameter integer RESET_HOLD_NS=200;',
                    f'parameter integer RESET_HOLD_NS={args.reset_hold_ns};')
    (out/'tb_actual_phy.sv').write_text(tb)
    (out/'run.do').write_text('onerror {quit -code 2}\nonbreak {quit -code 3}\n'
                              'run -all\nquit -code 0\n')
    env = os.environ.copy()
    env['MGLS_LICENSE_FILE'] = str(args.license_file.resolve())
    env['LM_LICENSE_FILE'] = env['MGLS_LICENSE_FILE']
    commands = [
        ['vlib.exe', 'work'],
        ['vlog.exe', '-sv', *(['+define+OBSERVE_VENDOR_PUBLIC_CALIBRATION']
          if args.observe_public_calibration else []),
         '+incdir+'+str(model), 'vendor_lib.v', str(vo),
         str(model/'ddr3_model.sv'), 'tb_actual_phy.sv'],
        ['vsim.exe', '-c', '-t', '1ps', 'work.tb_actual_c', '-do', 'run.do']]
    result = dict(started_at=datetime.now().astimezone().isoformat(),
        original_vendor_models_unmodified=True, forced_calibration=False,
        vendor_model=str(vo), vendor_dqs_model=str(dqs), pll=str(args.pll.resolve()),
        public_calibration_net_observation=args.observe_public_calibration,
        reset_hold_ns=args.reset_hold_ns, license_used_child_environment_only=True,
        actual_board_training_verified=False, unconditional_service_bound=False,
        complete_UPDATE_5ms_verified=False, full_joint_build_started=False,
        programmed=False, steps=[])
    for index, command in enumerate(commands):
        with (out/f'step{index}.log').open('w', encoding='utf-8') as log:
            process = subprocess.Popen([str(args.simulator/command[0]), *command[1:]],
                cwd=out, env=env, stdout=log, stderr=subprocess.STDOUT,
                creationflags=subprocess.CREATE_NO_WINDOW)
            (out/'runtime.json').write_text(json.dumps(dict(pid=process.pid,
                command=command, started_at=datetime.now().astimezone().isoformat()), indent=2))
            print('started', command[0], process.pid, flush=True)
            rc = process.wait()
        result['steps'].append(dict(tool=command[0], exit_code=rc))
        (out/'result.json').write_text(json.dumps(result, indent=2))
        if rc:
            break
    log = (out/'step2.log').read_text(errors='replace') if (out/'step2.log').exists() else ''
    result.update(completed_at=datetime.now().astimezone().isoformat(),
        calibration_observed='CALIBRATED at' in log,
        functional_readback_passed=rc == 0 and 'ACTUAL C PHY PROBE PASSED' in log,
        model_errors=[line for line in log.splitlines() if re.search(r'\bERROR\b|\*\* Error|\*\* Fatal', line)],
        model_warnings=[line for line in log.splitlines() if re.search(r'\bWARNING\b|\*\* Warning', line)],
        unresolved_vendor_port_warning='Missing connection for port' in log)
    result['strict_protocol_probe_passed'] = result['functional_readback_passed'] and not result['model_errors'] and not any(
        'required before' in line for line in result['model_warnings']) and not result['unresolved_vendor_port_warning']
    (out/'result.json').write_text(json.dumps(result, indent=2))
    print(json.dumps({k: result[k] for k in ('calibration_observed',
          'functional_readback_passed', 'strict_protocol_probe_passed')}, indent=2))
    return result


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--phy', type=Path, required=True)
    p.add_argument('--pll', type=Path, required=True)
    p.add_argument('--model-dir', type=Path, required=True)
    p.add_argument('--license-file', type=Path, required=True)
    p.add_argument('--simulator', type=Path, default=Path('E:/modelsim_ae/win32aloem'))
    p.add_argument('--simlib', type=Path, default=Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a'))
    p.add_argument('--dqs-model', type=Path)
    p.add_argument('--observe-public-calibration', action='store_true')
    p.add_argument('--reset-hold-ns', type=int, default=200)
    run(p.parse_args())
