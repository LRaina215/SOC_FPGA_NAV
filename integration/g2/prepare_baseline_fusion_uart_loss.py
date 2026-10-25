"""Freeze the existing, tested UART-loss software onto the routed fusion source.

This prepares an independent source candidate.  It does not compile firmware,
synthesize, implement, program, open serial ports, or modify the RK application.
Only preprocessing and the existing fake-link RK handshake test are performed.
"""
import argparse
import copy
from datetime import datetime
import difflib
import json
from pathlib import Path
import re
import shutil
import struct
import subprocess
import sys

from prepare_baseline_fusion_retime import hardware_inputs, metadata, relocate, same_bytes, save

BASE = Path('F:/nav_integration_g2_20261023')
SOURCE = BASE/'baseline_fusion_only_syn23'
ROUTED = BASE/'baseline_fusion_only_pnr32'
DONOR = BASE/'uart_host_loss_stop_candidate2'
WEB = Path('D:/Desktop/SOC FPGA/QHXD_Web')
OUT = BASE/'baseline_fusion_uart_loss_prepared2'
GCC = Path('D:/tools/xpack-riscv-none-elf-gcc-15.2.0-1/bin/riscv-none-elf-gcc.exe')
EXISTING_FW = ['fw/app/nav_app.c', 'fw/drv/hal_fpga.c']
ADDED_FW = ['fw/nav/host_link_guard.c', 'fw/nav/host_link_guard.h']
RAM_FILES = ['ram_w.hex'] + [f'ram_b{i}.hex' for i in range(4)]


def load(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def inventory(root, suffixes=None):
    return {p.relative_to(root).as_posix(): p for p in root.rglob('*')
            if p.is_file() and '__pycache__' not in p.parts
            and (suffixes is None or p.suffix.lower() in suffixes)}


def read_elf(path):
    data = path.read_bytes()
    h = struct.unpack_from('<16sHHIIIIIHHHHHH', data, 0)
    assert h[0][:7] == b'\x7fELF\x01\x01\x01' and h[1:4] == (2, 243, 1)
    loads = [struct.unpack_from('<IIIIIIII', data, h[5]+i*h[9]) for i in range(h[10])]
    loads = [p for p in loads if p[0] == 1 and p[4]]
    assert len(loads) == 1 and loads[0][3] == 0
    p = loads[0]
    image = data[p[1]:p[1]+p[4]]
    sections = [struct.unpack_from('<IIIIIIIIII', data, h[6]+i*h[11]) for i in range(h[12])]
    symbols = {}
    for section in sections:
        if section[1] != 2:
            continue
        strings = sections[section[6]]
        table = data[strings[4]:strings[4]+strings[5]]
        for offset in range(section[4], section[4]+section[5], section[9]):
            name, value, size, info, other, index = struct.unpack_from('<IIIBBH', data, offset)
            name = table[name:].split(b'\0', 1)[0].decode('utf-8')
            symbols[name] = value
    required = {key: symbols[key] for key in ('RAM_SIZE', 'STACK_RESERVE', '_bss_start', '_bss_end', '_stack_top')}
    assert required == dict(RAM_SIZE=131072, STACK_RESERVE=6144,
                           _bss_start=52168, _bss_end=122136, _stack_top=131072)
    return image, dict(elf_class=32, machine='RISC-V', executable=True,
                      load_file_bytes=p[4], load_memory_bytes=p[5], linked_symbols=required,
                      BSS_bytes=required['_bss_end']-required['_bss_start'],
                      aligned_remaining_bytes=required['_stack_top']-required['STACK_RESERVE']-required['_bss_end'])


def decode_ram(directory):
    banks = []
    for bit in range(4):
        lines = (directory/f'ram_b{bit}.hex').read_text(encoding='ascii').splitlines()
        assert len(lines) == 32768 and all(re.fullmatch('[0-9a-fA-F]{2}', line) for line in lines)
        banks.append(bytes(int(line, 16) for line in lines))
    image = bytes(banks[bit][word] for word in range(32768) for bit in range(4))
    words = (directory/'ram_w.hex').read_text(encoding='ascii').splitlines()
    assert len(words) == 32768 and all(re.fullmatch('[0-9a-fA-F]{8}', word) for word in words)
    assert image == b''.join(int(word, 16).to_bytes(4, 'little') for word in words)
    return image


def check_preprocessed_sources(donor):
    """Bind current source/header tokens to the preserved actual compiler inputs.

    No machine-code generation or linking occurs.  Date/time are rebound only
    to the preserved build banner, rather than comparing a new current banner.
    """
    fw, built = donor/'fw', donor/'rv32'
    main = (built/'nav_fw.elf-main.i').read_text(encoding='utf-8')
    matches = re.findall(r'nav fw " "([A-Z][a-z]{2} [ 0-9][0-9] [0-9]{4})" " " "([0-9:]{8})"', main)
    assert len(matches) == 1
    date, time = matches[0]
    options = ['-E', '-flto', '-fstack-usage', '-march=rv32imc_zicsr', '-mabi=ilp32', '-Os', '-g',
               '-std=c99', '-Wall', '-Wno-unused-function', '-ffunction-sections', '-fdata-sections',
               '-fno-common', '-nostartfiles', '-specs=nano.specs', '-DNAV_BASE=0x40000000u',
               '-DGM_N=64', '-DASTAR_MAXN=64', '-DGM_DS=2', '-Wno-builtin-macro-redefined',
               f'-D__DATE__="{date}"', f'-D__TIME__="{time}"']
    options += [f'-I{fw/name}' for name in ('include', 'hal', 'nav', 'app', 'bsp')]
    sources = sorted((fw/'nav').glob('*.c')) + [fw/'app/nav_app.c', fw/'app/main.c',
                                                 fw/'drv/hal_fpga.c', fw/'bsp/console.c']
    assert len(sources) == 25
    normalize = lambda s: '\n'.join(line for line in s.splitlines() if not line.lstrip().startswith('#'))
    rows = []
    for source in sources:
        preserved = built/f'nav_fw.elf-{source.stem}.i'
        command = [str(GCC), *options, str(source)]
        result = subprocess.run(command, capture_output=True, text=True, encoding='utf-8',
                                creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
        assert result.returncode == 0, (source, result.stderr)
        assert normalize(result.stdout) == normalize(preserved.read_text(encoding='utf-8')), source
        rows.append(dict(source=metadata(source), preserved_preprocessed=metadata(preserved),
                         passed=True, exit_code=0, stderr=result.stderr))
    return dict(passed=True, C_translation_units=25, assembly_start_source_unchanged=True,
                current_source_tokens_equal_preserved_actual_compiler_inputs=True,
                ignored_only_preprocessor_line_directives=True,
                preserved_DATE=date, preserved_TIME=time, preprocessing_only=True,
                no_codegen_or_link_or_firmware_recompile=True, gcc=metadata(GCC), options=options, units=rows)


def prepare(out=OUT):
    source, routed, donor, out = [p.resolve() for p in (SOURCE, ROUTED, DONOR, Path(out))]
    assert not out.exists() and not (source/'impl/pnr').exists()
    job = load(source/'job.json')
    assert job['phase'] == 'synthesis_complete' and job['tool_exit'] == 0
    assert load(routed/'g2_gate.json')['routing_passed']
    assert same_bytes(source/'impl/gwsynthesis/g2_top.vg', routed/'g2_top.vg')
    manifest = load(source/'manifest.json')
    assert manifest['stage'] == 'syn' and manifest['fusion_norm_pipeline']
    assert not manifest.get('pose_predecode') and not manifest.get('action_map_exact_widths')
    assert not manifest['control_weights_changed'] and manifest['clock_core_mhz'] == 50
    assert manifest['wide_split'] == 1 and not manifest['programmed']
    donor_manifest = load(donor/'manifest.json')
    origin = Path(donor_manifest['frozen_origin']).resolve()
    old_fw, routed_fw, new_fw = [inventory(path/'fw') for path in (origin, source, donor)]
    assert len(old_fw) == 61 and set(old_fw) == set(routed_fw)
    assert all(same_bytes(old_fw[name], routed_fw[name]) for name in old_fw)
    changed_fw = sorted(name for name in old_fw if not same_bytes(old_fw[name], new_fw[name]))
    added_fw = sorted(set(new_fw)-set(old_fw))
    assert changed_fw == [name[3:] for name in EXISTING_FW]
    assert added_fw == [name[3:] for name in ADDED_FW]
    proof_names = ['manifest.json', 'prepared_origin_binding.json', 'result.json',
                   'rv32_validation_result.json', 'navigation_regression_result.json',
                   'rk_mission_handshake_result.json', 'frozen_origin_and_image_binding.json']
    proofs = {name: load(donor/name) for name in proof_names}
    assert proofs['manifest.json']['all_requested_light_contracts_passed']
    assert all(proofs[name]['passed'] for name in proof_names if name not in ('manifest.json', 'prepared_origin_binding.json'))
    assert proofs['rv32_validation_result.json']['image_bytes'] == 52164
    assert proofs['rk_mission_handshake_result.json']['checks'] == 31
    stack = load(donor/'rv32/stack_analysis.json')
    assert stack['complete'] and stack['static_bound_bytes'] == 2736 and stack['stack_reserved_bytes'] == 6144
    assert stack['irq_disabled'] and not stack['unresolved_indirect'] and not stack['nonstatic_frames']
    image = (donor/'rv32/nav_fw.bin').read_bytes()
    elf_image, elf_review = read_elf(donor/'rv32/nav_fw.elf')
    assert image == elf_image and len(image) == 52164
    ram = decode_ram(donor/'rv32')
    assert ram == image + bytes(131072-len(image))
    original_image = (source/'firmware/nav_fw.bin').read_bytes()
    assert len(original_image) == 51712
    assert decode_ram(source) == original_image + bytes(131072-len(original_image))
    assert decode_ram(source/'firmware') == decode_ram(source)
    core_relative = 'yuntuo/bridge/core.py'
    core_before = donor/'baseline/rk'/core_relative
    working_core = WEB/core_relative
    core_after = donor/'rk'/core_relative
    assert core_before.read_text(encoding='utf-8') == working_core.read_text(encoding='utf-8')
    assert core_before.read_bytes().replace(b'\r\n', b'\n') == working_core.read_bytes().replace(b'\r\n', b'\n')
    preprocess = check_preprocessed_sources(donor)
    hardware = hardware_inputs(source, manifest)
    assert len(hardware) == 63
    assets = {name:p for name,p in inventory(source, {'.hex','.mem','.bin','.mif'}).items() if 'impl/' not in name}
    headers = {name:p for name,p in inventory(source, {'.vh','.h'}).items() if 'impl/' not in name}
    assert len(assets) == 69 and len(headers) == 31
    out.mkdir(parents=True)
    for name, file in {**hardware, **assets, **headers}.items():
        target=out/name; target.parent.mkdir(parents=True, exist_ok=True); shutil.copyfile(file, target)
    shutil.copytree(donor/'fw', out/'fw', dirs_exist_ok=True,
                    ignore=shutil.ignore_patterns('__pycache__'))
    # All compiler artifacts are the preserved candidate image, not stale old objects.
    shutil.copytree(donor/'rv32', out/'firmware', dirs_exist_ok=True)
    for name in RAM_FILES:
        shutil.copyfile(donor/'rv32'/name, out/name)
    for name in ('g2.cst', 'g2.sdc', 'storage_layout.json'):
        shutil.copyfile(source/name, out/name)
    assert all(same_bytes(source/name, out/name) for name in hardware)
    modified_assets = sorted(name for name in assets if not same_bytes(source/name, out/name))
    expected = ['firmware/nav_fw.bin'] + RAM_FILES + ['firmware/'+name for name in RAM_FILES]
    assert modified_assets == sorted(expected) and len(modified_assets) == 11
    assert len(inventory(out, {'.vh','.h'})) == 32
    assert decode_ram(out) == decode_ram(out/'firmware') == ram
    assert same_bytes(donor/'rv32/nav_fw.elf', out/'firmware/nav_fw.elf')
    assert all(same_bytes(source/name,out/name) and same_bytes(source/name,routed/name) for name in ('g2.cst','g2.sdc'))
    assert (out/'g2.sdc').read_text(encoding='utf-8').splitlines() == [
        'create_clock -name clk50 -period 20 [get_ports {clk50}]',
        'create_clock -name rgmii_rxc -period 10 [get_ports {rgmii_rxc}]']
    (out/'rk'/core_relative).parent.mkdir(parents=True)
    shutil.copyfile(core_after, out/'rk'/core_relative)
    # Context is a snapshot for tests/review, not an instruction to deploy all files.
    context = {name:p for name,p in inventory(WEB/'yuntuo').items() if p.suffix == '.py'}
    for name, file in context.items():
        target=out/'rk_context/yuntuo'/name; target.parent.mkdir(parents=True,exist_ok=True); shutil.copyfile(file,target)
    context_config = ['backend/app/config/yuntuo/waypoints.json',
                      'backend/app/config/yuntuo/patrol_routes.json']
    for name in context_config:
        target=out/'rk_context'/name; target.parent.mkdir(parents=True,exist_ok=True); shutil.copyfile(WEB/name,target)
    test = (donor/'test_rk_mission_handshake.py').read_text(encoding='utf-8')
    old_import = "sys.path.insert(0,'D:/Desktop/SOC FPGA/QHXD_Web')"
    assert test.count(old_import) == 1
    rebound = test.replace(old_import, "sys.path.insert(0,str(Path(__file__).resolve().parent/'rk_context'))", 1)
    (out/'revalidate_rk_mission_handshake.py').write_text(rebound, encoding='utf-8', newline='\n')
    run_test = subprocess.run([sys.executable, '-X', 'utf8', str(out/'revalidate_rk_mission_handshake.py')],
                              capture_output=True, text=True, encoding='utf-8',
                              creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
    assert run_test.returncode == 0, (run_test.stdout,run_test.stderr)
    rk_result = load(out/'rk_mission_handshake_result.json')
    assert rk_result['passed'] and rk_result['checks'] == 31
    assert not rk_result['serial_opened'] and not rk_result['production_services_changed']
    assert all(same_bytes(file,out/'rk_context/yuntuo'/name) for name,file in context.items())
    run = (source/'run.tcl').read_text(encoding='utf-8')
    assert [line.strip() for line in run.splitlines() if line.strip().startswith('run ')] == ['run syn']
    (out/'run.tcl').write_text(relocate(run,source,out),encoding='utf-8',newline='\n')
    prepared = relocate(copy.deepcopy(manifest),source,out)
    prepared.pop('effective_options',None)
    prepared.update(source_files=[str(out/name) for name in hardware],frozen_source_build=str(source),
                    source_preparation_only=True,synthesis_complete=False,automatic_pnr=False,programmed=False,
                    deployment_accepted=False,candidate='fusion_only_with_frozen_UART_host_loss_software',
                    modified_rtl_inputs=[],modified_firmware_inputs=EXISTING_FW,added_firmware_inputs=ADDED_FW,
                    modified_hardware_assets=modified_assets,host_link_loss_guard=True,
                    host_guard_timeout_us=1000000,firmware_image_bytes=52164)
    save(out/'manifest.json',prepared)
    save(out/'preprocessed_source_binding.json',preprocess)
    patch = []
    for name in EXISTING_FW:
        patch.extend(difflib.unified_diff((source/name).read_text(encoding='utf-8').splitlines(keepends=True),
                     (out/name).read_text(encoding='utf-8').splitlines(keepends=True),fromfile='a/'+name,tofile='b/'+name))
    for name in ADDED_FW:
        patch.extend(difflib.unified_diff([], (out/name).read_text(encoding='utf-8').splitlines(keepends=True),
                                         fromfile='/dev/null',tofile='b/'+name))
    patch.extend(difflib.unified_diff(core_before.read_text(encoding='utf-8').splitlines(keepends=True),
                 core_after.read_text(encoding='utf-8').splitlines(keepends=True),fromfile='a/rk/'+core_relative,tofile='b/rk/'+core_relative))
    (out/'uart_host_loss_only.patch').write_text(''.join(patch),encoding='utf-8',newline='\n')
    result = dict(passed=True,recorded_at=datetime.now().astimezone().isoformat(),source=str(source),
                  routed_reference=str(routed),prepared=str(out),donor=str(donor),donor_origin=str(origin),
                  review_kind='source/image preparation and existing software contracts, not new joint implementation',
                  ready_for_integrated_synthesis=True,new_SYN_or_PNR_started=False,
                  source_origin_FW61_byte_identical=True,hardware_inputs=63,modified_rtl_inputs=[],
                  modified_existing_FW=EXISTING_FW,added_FW=ADDED_FW,unchanged_existing_FW=59,
                  original_headers=31,new_headers=32,hardware_assets=69,modified_hardware_assets=modified_assets,
                  unchanged_hardware_assets=58,CPU128KiB_capacity_unchanged=True,
                  new_CPU_INIT_not_yet_measured_in_actual_joint_netlist=True,
                  image=dict(original_bytes=51712,candidate_bytes=52164,growth_bytes=452,
                             candidate_ELF_BIN_load_equal=True,RAM_bank_and_word_views_equal=True,
                             zero_padding_bytes=131072-52164,**elf_review),
                  compiler_support_artifacts_replaced_separately_from11_hardware_assets=True,
                  preprocessing_proof=str(out/'preprocessed_source_binding.json'),
                  old_frozen_origin_timestamp_only_proof_not_misrepresented_as_candidate_difference=True,
                  stack_static_bound_bytes=2736,stack_reserved_bytes=6144,
                  original_4500_all_weights125Hz_and_stop_RTL_byte_identical=True,
                  original_two_clocks_CST_20ns10ns_SDC_byte_identical=True,
                  no_new_CDC52_exceptions=True,
                  RK=dict(minimum_production_change=['yuntuo/bridge/core.py'],
                          overlay=str(out/'rk'),context_snapshot=str(out/'rk_context'),
                          working_core_equals_donor_baseline_after_CRLF_normalization=True,
                          baseline_bytes=core_before.stat().st_size,working_bytes=working_core.stat().st_size,
                          byte_difference_is_only_CRLF_vs_LF=True,production_modified=False,
                          existing_31_check_source_has_no_historical_dependency_snapshot=True,
                          current_frozen_context_31_checks_passed=True,
                          current_frozen_context_test=str(out/'rk_mission_handshake_result.json'),
                          heartbeat_seconds=0.2,fresh_idle_feedback_seconds=0.5),
                  reused_software_proofs=[dict(path=str(donor/name),metadata=metadata(donor/name)) for name in proof_names],
                  stop_contract=dict(host_timeout_us=1000000,timeout_is_candidate_parameter_not_measured_deadline=True,
                       liveness='valid inbound NV application packet timestamp; outgoing STATUS or raw MMIO polls do not renew',
                       controlled_stop='clear task/goal/old publication, publish zero numeric NAV demand; retain balance policy',
                       status_mode_ESTOP_is_not_SAFE_CTRL_ESTOP_or_power_cut=True,
                       true_local_emergency_stop_path_hardware_unchanged=True),
                  remaining=['Actual complete joint synthesis: new CPU INIT, all RAM, original 26 CDC and clock/IO sources',
                             'Full setup/hold/recovery/removal/pulse/physical CDC and board IO acceptance',
                             'Real dual UART/NV/telemetry, heartbeats, deadline under planner load, explicit rearm',
                             'Controlled deceleration/balance/physical standstill and true emergency path separately',
                             'NV has no wire session or nonce; pre-arrival delayed USB replay remains unproved',
                             'Full software hard WCET, actual stack watermark, full RK service/vehicle acceptance'],
                  bindings=[metadata(p) for p in (source/'manifest.json',source/'firmware/nav_fw.bin',
                      donor/'rv32/nav_fw.elf',donor/'rv32/nav_fw.bin',out/'firmware/nav_fw.elf',out/'firmware/nav_fw.bin',
                      donor/'test_rk_mission_handshake.py',out/'revalidate_rk_mission_handshake.py',core_before,working_core,core_after)],
                  frozen_FW_bindings=[dict(relative=name,source=metadata(p),prepared=metadata(out/'fw'/name)) for name,p in new_fw.items()],
                  RK_context_bindings=[dict(relative=name,original=metadata(p),snapshot=metadata(out/'rk_context/yuntuo'/name)) for name,p in context.items()],
                  RK_configuration_bindings=[dict(relative=name,original=metadata(WEB/name),snapshot=metadata(out/'rk_context'/name)) for name in context_config],
                  programmed=False,serial_opened=False,physical_stop_proved=False,deployment_accepted=False)
    save(out/'prepared_source_review.json',result)
    return result


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--out',type=Path,default=OUT);args=parser.parse_args()
    import psutil
    me=psutil.Process();me.nice(psutil.IDLE_PRIORITY_CLASS);me.cpu_affinity([18])
    result=prepare(args.out)
    print(json.dumps({key:result[key] for key in ('passed','prepared','modified_rtl_inputs','modified_existing_FW',
                      'added_FW','modified_hardware_assets','ready_for_integrated_synthesis','deployment_accepted')},ensure_ascii=False))


if __name__=='__main__':main()
