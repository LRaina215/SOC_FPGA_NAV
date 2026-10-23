"""One bounded SYN from frozen inputs, optionally after an owned PNR finishes.

Copies source inputs into a new directory before waiting. Never programs,
changes running builds, reruns physics or automatically launches another PNR.
"""
import argparse
import ctypes
from datetime import datetime
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time
from preflight import validate


class MemoryStatus(ctypes.Structure):
    _fields_=[('length',ctypes.c_uint32),('load',ctypes.c_uint32)]+[(n,ctypes.c_uint64)for n in
        ('total_phys','avail_phys','total_page','avail_page','total_virtual','avail_virtual','avail_extended')]


def memory_status():
    value=MemoryStatus();value.length=ctypes.sizeof(value)
    if not ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(value)):raise ctypes.WinError()
    return value


def main():
    ap=argparse.ArgumentParser();ap.add_argument('source',type=Path)
    ap.add_argument('--out',type=Path,required=True)
    launch=ap.add_mutually_exclusive_group(required=True)
    launch.add_argument('--wait-pid',type=int)
    launch.add_argument('--start-now',action='store_true',help='explicit serial launch after the prior tool has exited')
    ap.add_argument('--gyro-sum-exact-widths',action='store_true',help='derive only the already-validated17-bit gyro sum from frozen RTL')
    ap.add_argument('--trig-block-rom',action='store_true',help='derive initialized navigation lookup storage from the frozen directed RTL')
    ap.add_argument('--snapshot-mux-fanout',action='store_true',help='derive cycle-exact snapshot mux fanout from frozen scheduler RTL')
    ap.add_argument('--history-data-fanout',action='store_true',help='derive only the verified initialized data-mux fanout attribute')
    ap.add_argument('--sfp-phy-link-guard',action='store_true',help='require partner copper link in SGMII mode; explicit frozen one-line change')
    ap.add_argument('--lease-output-select',action='store_true',help='derive only the cycle-checked late lease stop selection')
    ap.add_argument('--slf-read-address-fanout',action='store_true',help='experimental attribute only; first full SYN retained275 RAM-input loads, so improvement must be measured')
    ap.add_argument('--bank-select-data-fanout',action='store_true',help='apply measured128 fanout limit at bank-read selector register; module576->128, full SYN/PNR still required')
    ap.add_argument('--slf-row-banks',action='store_true',help='partition only the verified160x98 row store into ten16-word distributed banks; same latency/capacity')
    ap.add_argument('--sfp-lane2-from-build',type=Path,help='adopt proved SFP1 Lane2 generated wrapper/CSR; preserve other frozen RTL and joint timing requirements')
    a=ap.parse_args();src=a.source.resolve();out=a.out.resolve()
    assert (a.wait_pid is None or a.wait_pid>0) and out!=src and src not in out.parents and out not in src.parents
    manifest=json.loads((src/'manifest.json').read_text(encoding='utf-8'))
    assert manifest['stage']=='syn' and not manifest['control_weights_changed'] and not manifest['programmed']
    assert not a.gyro_sum_exact_widths or not manifest.get('gyro_sum_exact_widths',False),'source already contains this candidate'
    assert not a.trig_block_rom or (manifest['packing']=='directed' and not manifest.get('trig_block_rom',False)),'requires directed source without this candidate'
    assert not a.snapshot_mux_fanout or not manifest.get('snapshot_mux_fanout',False),'source already contains snapshot candidate'
    assert not a.history_data_fanout or not manifest.get('history_data_fanout',False),'source already contains history candidate'
    assert not a.sfp_phy_link_guard or (manifest.get('real_sfp') and not manifest.get('sfp_phy_link_guard'))
    assert not a.lease_output_select or (manifest.get('command_lease') and not manifest.get('lease_output_select'))
    assert not a.slf_read_address_fanout or not manifest.get('slf_read_address_fanout')
    assert not a.bank_select_data_fanout or not manifest.get('bank_select_data_fanout')
    assert not a.slf_row_banks or not manifest.get('slf_row_banks')
    original=(src/'run.tcl').read_text(encoding='utf-8')
    assert [s.strip()for s in original.splitlines()if s.strip().startswith('run ')]==['run syn']
    # Preserve the cancelled directory, including partial implementation files.
    shutil.copytree(src,out,ignore=shutil.ignore_patterns('impl','gw_sh.log','preflight.log',
        'resource_cancellation.json','__pycache__','job.json','synthesis_summary.json',
        'memory_inventory.json','tool_runtime.json'))
    modified=[]
    if a.gyro_sum_exact_widths:
        import importlib.util
        generator_path=Path(__file__).resolve().parents[2]/'control/tools/g2_gyro_sum_widths.py'
        spec=importlib.util.spec_from_file_location('gyro_widths',generator_path)
        generator=importlib.util.module_from_spec(spec);spec.loader.exec_module(generator)
        generator.generate(out/'control/imu_fusion.v')
        manifest['gyro_sum_exact_widths']=True
    if a.trig_block_rom:
        from prepare_trig_block_rom import generate
        generate(out/'nav/rtl/common/nav_ram_sdp.v')
        manifest['trig_block_rom']=True
    if a.snapshot_mux_fanout:
        import importlib.util
        generator_path=Path(__file__).resolve().parents[2]/'control/tools/g2_snapshot_fanout.py'
        spec=importlib.util.spec_from_file_location('snapshot_fanout',generator_path)
        generator=importlib.util.module_from_spec(spec);spec.loader.exec_module(generator)
        generator.generate(out/'control/rl_policy_tick_v5.v')
        manifest['snapshot_mux_fanout']=True
    if a.history_data_fanout:
        import importlib.util
        generator_path=Path(__file__).resolve().parents[2]/'control/tools/g2_history_fanout.py'
        spec=importlib.util.spec_from_file_location('history_fanout',generator_path)
        generator=importlib.util.module_from_spec(spec);spec.loader.exec_module(generator)
        generator.generate(out/'control/rl_observation_v5.v')
        manifest['history_data_fanout']=True
    expected=[]
    if a.sfp_phy_link_guard:
        from patch_sfp_link_guard import generate
        generate(out/'sfp_management.v');manifest['sfp_phy_link_guard']=True
    if a.lease_output_select:
        from prepare_lease_output_select import generate
        generate(out/'command_lease_guard.v');manifest['lease_output_select']=True
    if a.slf_read_address_fanout:
        from prepare_slf_read_fanout import generate
        generate(out/'nav/rtl/match/slf_extract.v');manifest['slf_read_address_fanout']=True
    if a.bank_select_data_fanout:
        from prepare_bank_select_fanout import generate
        generate(out/'nav/rtl/common/nav_ram_sdp.v');manifest['bank_select_data_fanout']=True
    if a.slf_row_banks:
        from prepare_slf_row_banks import generate
        generate(out/'nav/rtl/common/nav_ram_sdp.v');manifest['slf_row_banks']=True
    if a.sfp_lane2_from_build:
        assert manifest.get('real_sfp') and not manifest.get('sfp_lane2_from_build')
        from prepare_sfp_lane2 import adopt, verify_frozen
        expected += adopt(out,src,a.sfp_lane2_from_build)
        manifest['sfp_lane2_from_build']=str(a.sfp_lane2_from_build.resolve())
        manifest['sfp_lane2_input_checks']=verify_frozen(out,src,a.sfp_lane2_from_build)
    if a.gyro_sum_exact_widths:expected.append('control/imu_fusion.v')
    if a.trig_block_rom:expected.append('nav/rtl/common/nav_ram_sdp.v')
    if a.snapshot_mux_fanout:expected.append('control/rl_policy_tick_v5.v')
    if a.history_data_fanout:expected.append('control/rl_observation_v5.v')
    if a.sfp_phy_link_guard:expected.append('sfp_management.v')
    if a.lease_output_select:expected.append('command_lease_guard.v')
    if a.slf_read_address_fanout:expected.append('nav/rtl/match/slf_extract.v')
    if a.bank_select_data_fanout:expected.append('nav/rtl/common/nav_ram_sdp.v')
    if a.slf_row_banks:expected.append('nav/rtl/common/nav_ram_sdp.v')
    for name in manifest['source_files']:
        previous=Path(name);relative=previous.relative_to(src)
        if previous.read_bytes()!=(out/relative).read_bytes():modified.append(relative.as_posix())
    assert sorted(modified)==sorted(expected),modified
    def rewrite(value):
        if isinstance(value,str):return value.replace(src.as_posix(),out.as_posix()).replace(str(src),str(out))
        if isinstance(value,list):return [rewrite(v)for v in value]
        if isinstance(value,dict):return {k:rewrite(v)for k,v in value.items()}
        return value
    manifest=rewrite(manifest);manifest['frozen_source_build']=str(src)
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    settings=[rewrite(line)for line in original.splitlines()if line.strip()not in ('run syn','exit')]
    state=dict(source=str(src),build=str(out),wait_pid=a.wait_pid,programmed=False,
        source_frozen_before_wait=True,automatic_pnr=False,
        modified_rtl_inputs=modified,
        scope='one bounded frozen-input SYN only; SYN completion is not G2/vehicle acceptance')
    def status(phase,**extra):
        state.update(phase=phase,updated_at=datetime.now().astimezone().isoformat(),**extra)
        (out/'job.json').write_text(json.dumps(state,indent=2),encoding='utf-8')
        print(json.dumps(dict(phase=phase,**extra)),flush=True)
    tool=Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/bin/gw_sh.exe')
    status('validating_frozen_settings')
    effective=validate(tool,out,settings,50,manifest['wide_split'])
    (out/'run.tcl').write_text('\n'.join(settings+['run syn','exit'])+'\n',encoding='utf-8')
    manifest['effective_options']=effective
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    if a.wait_pid is not None:
        kernel=ctypes.WinDLL('kernel32',use_last_error=True)
        kernel.OpenProcess.argtypes=[ctypes.c_uint32,ctypes.c_int,ctypes.c_uint32];kernel.OpenProcess.restype=ctypes.c_void_p
        kernel.WaitForSingleObject.argtypes=[ctypes.c_void_p,ctypes.c_uint32]
        kernel.CloseHandle.argtypes=[ctypes.c_void_p]
        handle=kernel.OpenProcess(0x100000,False,a.wait_pid)
        if not handle:
            status('wait_process_unavailable',windows_error=ctypes.get_last_error());return 1
        status('waiting_for_prior_pnr')
        try:
            while True:
                result=kernel.WaitForSingleObject(handle,2000)
                if result==0:break
                if result!=0x102:raise ctypes.WinError(ctypes.get_last_error())
        finally:kernel.CloseHandle(handle)
    status('waiting_for_memory',minimum_start_free_bytes=2*1024**3,minimum_start_available_commit_bytes=4*1024**3)
    for _ in range(24):
        memory=memory_status()
        if memory.avail_phys>=2*1024**3 and memory.avail_page>=4*1024**3:break
        time.sleep(5)
    else:
        status('not_started_host_memory',free_bytes=memory.avail_phys,available_commit_bytes=memory.avail_page);return 1
    status('synthesizing',max_runtime_minutes=40,
        memory_policy='cancel sustained commit reserve below512MiB; or physical below256MiB together with commit below1GiB')
    with (out/'gw_sh.log').open('w',encoding='utf-8') as log:
        process=subprocess.Popen([str(tool),'run.tcl'],cwd=out,stdout=log,stderr=subprocess.STDOUT,
            creationflags=subprocess.CREATE_NO_WINDOW)
        state.update(tool_pid=process.pid,synthesis_started_at=datetime.now().astimezone().isoformat())
        low=0;started=time.monotonic();last_heartbeat=0
        while process.poll()is None:
            memory=memory_status()
            now=time.monotonic()
            if now-last_heartbeat>=30:
                state.update(elapsed_s=now-started,free_bytes=memory.avail_phys,available_commit_bytes=memory.avail_page,
                    updated_at=datetime.now().astimezone().isoformat())
                (out/'job.json').write_text(json.dumps(state,indent=2),encoding='utf-8');last_heartbeat=now
            danger=memory.avail_page<512*1024**2 or (memory.avail_phys<256*1024**2 and memory.avail_page<1024**3)
            low=low+1 if danger else 0
            if low>=2:
                process.kill();process.wait()
                status('cancelled_host_memory',free_bytes=memory.avail_phys,available_commit_bytes=memory.avail_page,tool_exit=process.returncode);return 1
            if now-started>=40*60:
                process.kill();process.wait()
                status('cancelled_runtime_budget',tool_exit=process.returncode);return 1
            time.sleep(2)
    text=(out/'gw_sh.log').read_text(encoding='utf-8',errors='replace')
    errors=re.findall(r'^.*ERROR\s*\(.*$',text,re.M)
    if process.returncode or errors:
        status('synthesis_failed',tool_exit=process.returncode,errors=errors);return 1
    here=Path(__file__).resolve().parent
    for script,name in [('collect_evidence.py','synthesis_summary.json'),('audit_netlist_memories.py','memory_inventory.json')]:
        subprocess.run([sys.executable,'-X','utf8',str(here/script),str(out),'--out',str(out/name)],check=True)
    status('synthesis_complete',tool_exit=process.returncode,joint_pnr_measured=False)
    return 0


if __name__=='__main__':raise SystemExit(main())
