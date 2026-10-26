"""Prepare a source-only UART deadline candidate from the actual CPU8 result.

The old one-second probe passed its finite contract but took1.09518s to publish
zero. This candidate uses250ms admission,50ms heartbeat/ACK polling and sends
one immediate zero publication on each new lock. No running PNR is changed.
"""
from datetime import datetime
from pathlib import Path
import argparse
import copy
import json
import os
import re
import shutil
import struct
import subprocess
import sys

import psutil
from prepare_baseline_fusion_retime import hardware_inputs, metadata, relocate, same_bytes, save
from prepare_baseline_fusion_uart_loss import decode_ram, inventory

BASE = Path('F:/nav_integration_g2_20261023')
SOURCE = BASE/'baseline_fusion_preview_FIFO_MAC_UART_prepared1'
PROBE = BASE/'preview_full_G2_UART_CPU_probe8_modelsim'
GCC = Path('D:/tools/xpack-riscv-none-elf-gcc-15.2.0-1/bin/riscv-none-elf-gcc.exe')


def read_image(path):
    data = Path(path).read_bytes()
    h = struct.unpack_from('<16sHHIIIIIHHHHHH', data)
    assert h[0][:7] == b'\x7fELF\x01\x01\x01' and h[1:4] == (2, 243, 1)
    loads = [struct.unpack_from('<IIIIIIII', data, h[5]+i*h[9]) for i in range(h[10])]
    loads = [p for p in loads if p[0] == 1 and p[4]]
    assert len(loads) == 1 and loads[0][3] == 0
    p = loads[0]; image = data[p[1]:p[1]+p[4]]
    sections = [struct.unpack_from('<IIIIIIIIII', data, h[6]+i*h[11]) for i in range(h[12])]
    symbols = {}
    for s in sections:
        if s[1] != 2: continue
        strings = sections[s[6]]; names = data[strings[4]:strings[4]+strings[5]]
        for off in range(s[4], s[4]+s[5], s[9]):
            name, value, size, info, other, index = struct.unpack_from('<IIIBBH', data, off)
            symbols[names[name:].split(b'\0', 1)[0].decode()] = value
    keys = ['RAM_SIZE', 'STACK_RESERVE', '_bss_start', '_bss_end', '_stack_top']
    v = {k:symbols[k] for k in keys}
    assert v['RAM_SIZE'] == v['_stack_top'] == 131072 and v['STACK_RESERVE'] == 6144
    assert len(image) <= v['_bss_start'] <= v['_bss_end'] <= 131072-6144
    assert v['_bss_start'] % 4 == v['_bss_end'] % 4 == 0
    assert v['_bss_start']-len(image) < 4
    return image, dict(linked_symbols=v, load_file_bytes=p[4], load_memory_bytes=p[5],
                       aligned_remaining_bytes=131072-6144-v['_bss_end'])


def prepare(out):
    assert not out.exists()
    previous = json.loads((PROBE/'result.json').read_text())
    assert previous['passed'] and previous['metrics']['timeout_elapsed_us'] == 1000350
    for row in previous['source_bindings']+[previous['firmware_binding'], previous['TB_binding']]:
        assert metadata(row['path']) == row
    shutil.copytree(SOURCE, out, ignore=shutil.ignore_patterns('__pycache__'))
    manifest = json.loads((SOURCE/'manifest.json').read_text())
    hardware = hardware_inputs(SOURCE, manifest); assert len(hardware) == 63
    header = 'fw/nav/host_link_guard.h'; app = 'fw/app/nav_app.c'
    old = (out/header).read_text(); assert old.count('HOST_LINK_TIMEOUT_US 1000000u') == 1
    (out/header).write_text(old.replace('HOST_LINK_TIMEOUT_US 1000000u', 'HOST_LINK_TIMEOUT_US 250000u'), newline='\n')
    old = (out/app).read_text()
    anchor = '    if(!host_link_guard_poll(&g_host_link_guard,now)) return;'
    assert old.count(anchor) == 1
    new = old.replace(anchor, '''    static bool zero_published_for_lock;
    if(!host_link_guard_poll(&g_host_link_guard,now)) {
        zero_published_for_lock=false;
        return;
    }''', 1)
    anchor = '    support_note_cmd(&nt->cmd);\n}'
    assert new.count(anchor) == 1
    new = new.replace(anchor, '''    support_note_cmd(&nt->cmd);
    /* Publish on the lock edge instead of waiting for the100ms planner tick.
       hal_send_cmd keeps its independent final guard and hardware STOP path. */
    if(!zero_published_for_lock) {
        hal_send_cmd(&nt->cmd);
        zero_published_for_lock=true;
    }
}''', 1)
    (out/app).write_text(new, newline='\n')
    # rk_context is the preserved baseline dependency snapshot; the explicit
    # selected overlay lives under rk/. Do not mistake that snapshot for the
    # selected candidate or broaden the production overlay to other modules.
    for relative in ['rk/yuntuo/bridge/core.py']:
        p = out/relative; text = p.read_text()
        assert text.count('now-self.last_keepalive >= 0.2') == 1
        text = text.replace('now-self.last_keepalive >= 0.2', 'now-self.last_keepalive >= 0.05', 1)
        anchor = 'self.link.recv(min(timeout, max(0.0, deadline-time.monotonic())))'
        assert text.count(anchor) == 1
        text = text.replace(anchor, 'self.link.recv(min(timeout, 0.05 if self.link.kind == "uart" else timeout, max(0.0, deadline-time.monotonic())))', 1)
        p.write_text(text, encoding='utf-8', newline='\n')
    assert same_bytes(SOURCE/'rk_context/yuntuo/bridge/core.py', out/'rk_context/yuntuo/bridge/core.py')
    assert all(same_bytes(SOURCE/n, out/n) for n in hardware)
    changed_fw = sorted(n for n in inventory(SOURCE/'fw') if not same_bytes(SOURCE/'fw'/n, out/'fw'/n))
    assert changed_fw == ['app/nav_app.c', 'nav/host_link_guard.h']
    # Compile only the genuinely changed firmware; preserve all build flags and
    # the prior banner so a new timestamp does not become a spurious ROM delta.
    old_main = (SOURCE/'firmware/nav_fw.elf-main.i').read_text()
    banner = re.findall(r'nav fw " "([A-Z][a-z]{2} [ 0-9][0-9] [0-9]{4})" " " "([0-9:]{8})"', old_main)
    assert len(banner) == 1
    fw = out/'fw'; built = out/'firmware'; elf = built/'nav_fw.elf'
    sources = sorted(str(p) for p in (fw/'nav').glob('*.c')) + [str(fw/n) for n in
        ['app/nav_app.c','app/main.c','drv/hal_fpga.c','bsp/console.c','bsp/start.S']]
    command = [str(GCC), '-fstack-usage', '-save-temps=obj', '-flto', '-march=rv32imc_zicsr',
        '-mabi=ilp32', '-Os', '-g', '-std=c99', '-Wall', '-Wno-unused-function',
        '-ffunction-sections', '-fdata-sections', '-fno-common', '-nostartfiles', '-specs=nano.specs',
        '-DNAV_BASE=0x40000000u', '-DGM_N=64', '-DASTAR_MAXN=64', '-DGM_DS=2',
        '-Wno-builtin-macro-redefined', f'-D__DATE__="{banner[0][0]}"', f'-D__TIME__="{banner[0][1]}"',
        '-Wl,--defsym=RAM_SIZE=131072', '-Wl,--defsym=STACK_RESERVE=6144', '-Wl,--gc-sections',
        f'-Wl,-Map={built/"nav_fw.map"}', '-T', str(fw/'bsp/link.ld')]
    command += [f'-I{fw/n}' for n in ['include','hal','nav','app','bsp']]+sources+['-o', str(elf), '-lgcc']
    process = psutil.Process(); process.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
    env = os.environ.copy(); env['PATH'] = str(GCC.parent)+';'+env.get('PATH', '')
    r = subprocess.run(command, cwd=out/'nav', env=env, capture_output=True, text=True,
                       encoding='utf-8', creationflags=subprocess.CREATE_NO_WINDOW)
    save(out/'firmware_compile.json', dict(command=command, exit_code=r.returncode, stdout=r.stdout, stderr=r.stderr))
    assert r.returncode == 0, r.stderr
    subprocess.run([str(GCC).replace('gcc.exe','objcopy.exe'), '-O','binary',str(elf),str(built/'nav_fw.bin')],
                   env=env, check=True, creationflags=subprocess.CREATE_NO_WINDOW)
    sys.path.insert(0, str(fw/'bsp'))
    from analyze_stack import analyze
    stack = analyze(elf, Path(str(GCC).replace('gcc.exe','objdump.exe')),
                    built/'nav_fw.elf.ltrans0.ltrans.su', built/'nav_fw.elf.ltrans0.ltrans.s',
                    '.ENABLE_IRQ(0)' in (out/'nav/rtl/soc/nav_soc.v').read_text())
    assert stack['complete'] and stack['static_bound_bytes'] <= 3072
    save(built/'stack_analysis.json', stack)
    image, layout = read_image(elf); assert image == (built/'nav_fw.bin').read_bytes()
    ram = image+bytes(131072-len(image))
    for directory in [out, built]:
        for lane in range(4):
            (directory/f'ram_b{lane}.hex').write_text(''.join(f'{ram[i+lane]:02x}\n' for i in range(0,131072,4)), encoding='ascii')
        (directory/'ram_w.hex').write_text(''.join(ram[i:i+4][::-1].hex()+'\n' for i in range(0,131072,4)), encoding='ascii')
        assert decode_ram(directory) == ram
    old_assets = inventory(SOURCE,{'.hex','.mem','.bin','.mif'})
    changed_assets = sorted(n for n in old_assets if not same_bytes(SOURCE/n,out/n))
    assert len(old_assets) == 69 and len(changed_assets) == 11
    assert all(n == 'firmware/nav_fw.bin' or re.fullmatch(r'(firmware/)?ram_(w|b[0-3])\.hex',n) for n in changed_assets)
    manifest = relocate(copy.deepcopy(manifest), SOURCE, out)
    manifest.update(host_guard_timeout_us=250000, immediate_zero_on_new_host_lock=True,
                    source_preparation_only=True, synthesis_complete=False, automatic_pnr=False,
                    firmware_image_bytes=len(image), programmed=False, deployment_accepted=False)
    save(out/'manifest.json', manifest)
    test = subprocess.run([sys.executable,'-X','utf8',str(out/'revalidate_rk_mission_handshake.py')],
                          cwd=out, capture_output=True, text=True, encoding='utf-8',
                          creationflags=subprocess.CREATE_NO_WINDOW)
    (out/'rk_handshake_recheck.log').write_text(test.stdout+'\n'+test.stderr, encoding='utf-8')
    assert test.returncode == 0, test.stderr
    rk = json.loads((out/'rk_mission_handshake_result.json').read_text()); assert rk['passed'] and rk['checks'] == 31
    proof = dict(passed=True, recorded_at=datetime.now().astimezone().isoformat(), source=str(SOURCE), prepared=str(out),
                 basis_actual_CPU8_timeout_us=1000350, basis_actual_CPU8_zero_publication_age_us=1095180,
                 host_timeout_us=250000, UART_heartbeat_seconds=0.05, UART_map_ACK_wait_slice_seconds=0.05,
                 existing_total_map_wait_deadline_and_UDP_path_preserved=True,
                 modified_existing_FW=changed_fw, hardware_RTL63_all_byte_identical=True,
                 hardware_assets69_only_CPU11_changed=True, CPU128KiB_capacity_preserved=True,
                 new_image_bytes=len(image), image_layout=layout, stack_analysis=stack,
                 RK31_rearm_and_feedback_contracts_passed=True,
                 immediate_zero_once_per_lock_not_each_poll=True,
                 new_actual_CPU_latency_not_yet_proven=True, physical_500ms_stop_not_proven=True,
                 no_SYN_PNR_serial_RK_production_or_board_actions=True,
                 bindings=[metadata(p)for p in [out/'manifest.json',elf,built/'nav_fw.bin',out/header,out/app,
                    out/'rk/yuntuo/bridge/core.py',out/'rk_context/yuntuo/bridge/core.py']]+[metadata(out/n)for n in hardware])
    save(out/'prepared_source_review.json', proof)
    return proof


if __name__ == '__main__':
    parser = argparse.ArgumentParser(); parser.add_argument('--out',type=Path,required=True)
    result = prepare(parser.parse_args().out)
    print(json.dumps(dict(passed=result['passed'], image_bytes=result['new_image_bytes'],
                          host_timeout_us=result['host_timeout_us'], actual_CPU_pending=True)))
