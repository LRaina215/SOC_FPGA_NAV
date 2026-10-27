"""Rebind the existing finite MAC/UDP/XT bench to actual frozen SYN49 RTL.

No FPGA access, no synthesis, no modification to production RTL or its files.
The existing stimulus is reused; the changed RX pipeline is the new subject.
"""
from datetime import datetime
import json
from pathlib import Path
import re
import shutil
import subprocess

from probe_mac_udp_xt_reset import NAMES, VIV
from retry_frozen_synthesis import memory_status

ROOT=Path(__file__).resolve().parents[2]
BASE=Path('F:/nav_integration_g2_20261023')
SOURCE=BASE/'K_timing_syn49'
OLD=BASE/'preview_MAC_UDP_XT_reset_probe2'
OUT=BASE/'PNR61_MAC_UDP_XT_pipeline_probe2'


def main():
    assert not OUT.exists(),'Read the existing probe; do not repeat this input'
    m=memory_status();assert m.avail_phys>=2*1024**3 and m.avail_page>=4*1024**3
    OUT.mkdir()
    manifest=[]
    for rel in NAMES:
        src=SOURCE/rel;dst=OUT/src.name
        shutil.copyfile(src,dst)
        assert dst.read_bytes()==src.read_bytes()
        old=OLD/src.name
        manifest.append(dict(source=str(src),copied=str(dst),bytes=src.stat().st_size,
            changed_from_previous_probe=src.read_bytes()!=old.read_bytes()))
    for name in ('wire.hex','packets.hex','groups.hex','pixels.hex','tb_mac_udp_xt_reset.sv'):
        shutil.copyfile(OLD/name,OUT/name)
    # SYN49's accepted SLF128 frame-owner interface has an explicit admission
    # input. The pre-owner historical TB left it Z and therefore read no pixels.
    # This isolated consumer admits every frame; production owner arbitration
    # remains a separate check. No internal state or hardware RTL is modified.
    tb=OUT/'tb_mac_udp_xt_reset.sv'
    stimulus=tb.read_text()
    needle=".cfg_wait_pose(1'b0),.pose_commit(1'b0),.cfg_retain(1'b0),"
    assert stimulus.count(needle)==1
    stimulus=stimulus.replace(needle,".cfg_wait_pose(1'b0),.read_admit(1'b1),.frame_accept(),.pose_commit(1'b0),.cfg_retain(1'b0),")
    tb.write_text(stimulus,encoding='utf8')
    # Verify the actual production's reset topology and exact instantiated path.
    top=(SOURCE/'g2_top.v').read_text();nav=(SOURCE/'nav/rtl/top/nav_subsystem.v').read_text()
    assert '.clk(clk), .rst(rst), .rx_rst(l_rx_rst),' in top
    assert 'nav_rst_sync u_lrr (.clk(l_rx_clk), .arst(rst), .rst(l_rx_rst));' in top
    assert 'net_port #(.RX_AW(11), .TX_AW(11), .PL_AW(11), .RAW(1)) u_netl (' in nav
    commands=[('xvlog',[str(OUT/Path(n).name) for n in NAMES]),
        ('xvlog',['-sv',str(OUT/'tb_mac_udp_xt_reset.sv')]),
        ('xelab',['-debug','off','-timescale','1ns/1ps','tb_mac_udp_xt_reset','-s','network']),
        ('xsim',['network','-runall'])]
    state=dict(started_at=datetime.now().astimezone().isoformat(),phase='running',source=str(SOURCE),bindings=manifest,
        finite_stimulus_copied_from=str(OLD),physical_memory_at_start=m.avail_phys,commit_at_start=m.avail_page,
        hardware_access=False,new_SYN_PNR=False,
        previous_harness_rejection=str(BASE/'PNR61_MAC_UDP_XT_pipeline_probe1/xsim.log'),
        sole_testbench_change='Connect new public read_admit input to an always-admitting consumer; preserve all other stimulus')
    (OUT/'job.json').write_text(json.dumps(state,indent=2),encoding='utf8')
    for i,(tool,args) in enumerate(commands):
        with (OUT/f'{i}_{tool}.log').open('w') as f:
            r=subprocess.run(['cmd','/c',str(VIV/(tool+'.bat')),*args],cwd=OUT,
                stdout=f,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
        assert r.returncode==0,(tool,r.returncode)
    text=(OUT/'xsim.log').read_text(errors='replace')
    line=re.search(r'MAC UDP XT RESET PASSED[^\r\n]*',text)
    assert line and 'Fatal:' not in text,'Tool exit is not the functional verdict'
    metrics={k:int(v) for k,v in re.findall(r'(\w+)=(\d+)',line[0])}
    assert metrics['frames']==4 and metrics['pixels']==38400 and metrics['resets']==2
    assert all((SOURCE/n).read_bytes()==(OUT/Path(n).name).read_bytes() for n in NAMES)
    state.update(phase='complete',passed=True,ended_at=datetime.now().astimezone().isoformat(),metrics=metrics,
        scope='Actual MAC/FIFO/RAW1 UDP/XT at50MHz core,25MHz synthetic PHY,falling-edge sampling; current extra RX register included',
        no_internal_force=True,no_CPU_or_IMU_or_actuator_in_this_test=True,
        external_PHY_IO_STA_or_real_link_accepted=False,complete_vehicle_accepted=False)
    (OUT/'result.json').write_text(json.dumps(state,indent=2),encoding='utf8')
    (OUT/'job.json').write_text(json.dumps(state,indent=2),encoding='utf8')
    print(json.dumps(dict(passed=True,metrics=metrics,out=str(OUT))))


if __name__=='__main__':main()
