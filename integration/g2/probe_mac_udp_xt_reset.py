"""Exercise the eventual source's real MAC/FIFO/net_port/XT chain without a board.

This adds the UDP/XT composition proof missing from the smaller reset tests.
It does not prove full G2, CPU scheduling, control inference or physical timing.
"""
import argparse
from datetime import datetime
import json
from pathlib import Path
import shutil
import subprocess
import sys
import psutil
from prepare_baseline_fusion_retime import metadata,save,same_bytes

ROOT=Path(__file__).resolve().parents[2]
SOURCE=Path('F:/nav_integration_g2_20261023/baseline_fusion_preview_FIFO_MAC_UART_prepared1')
VIV=Path('E:/2026.1/Vivado/bin')
NAMES=[f'nav/rtl/common/{s}.v'for s in('nav_async_fifo','nav_cdc','nav_ram_sdp','nav_ram_banked','nav_fifo_ob','nav_fifo')]
NAMES += [f'nav/rtl/net/{s}.v'for s in('rgmii_mac100','net_port','udp_rx','udp_tx','raw_cap')]
NAMES += ['nav/rtl/lidar/xt_frame_rx.v']

def probe(out):
    assert not out.exists();out.mkdir(parents=True)
    source_review=json.loads((SOURCE/'prepared_source_review.json').read_text())
    assert source_review['passed'] and not source_review['combined_functional_acceptance']
    for row in source_review['bindings']:assert metadata(Path(row['path']))==row
    top=(SOURCE/'g2_top.v').read_text()
    nav=(SOURCE/'nav/rtl/top/nav_subsystem.v').read_text()
    assert '.clk(clk), .rst(rst), .rx_rst(l_rx_rst),'in top
    assert 'nav_rst_sync u_lrr (.clk(l_rx_clk), .arst(rst), .rst(l_rx_rst));'in top
    assert '.l_rx_clk(l_rx_clk), .l_rx_rst(l_rx_rst), .l_rx_valid(l_rx_valid), .l_rx_data(l_rx_data),'in top
    for text in ('.rx_mac_clk(l_rx_clk), .rx_mac_rst(l_rx_rst), .rx_mac_valid(l_rx_valid), .rx_mac_data(l_rx_data),',
                 '.s_valid(lu_valid), .s_data(lu_data), .s_last(lu_last), .s_err(lu_err),',
                 'net_port #(.RX_AW(11), .TX_AW(11), .PL_AW(11), .RAW(1)) u_netl ('):
        assert text in nav
    for name in NAMES:shutil.copyfile(SOURCE/name,out/Path(name).name)
    tb=ROOT/'integration/g2/tb_mac_udp_xt_reset.sv';shutil.copyfile(tb,out/tb.name)
    sys.path.insert(0,str(ROOT/'model'));import xt_proto as xt;import netpkt
    wire=bytearray();packets=[];groups=[];pixels=[]
    for k in range(4):
        image=[[(u*113+v*409+k*97)%65000 for u in range(160)]for v in range(60)]
        for i in range(4):image[0][i]=[0,64000,64001,65535][i]
        frame=xt.build_frame(image,sn=k+1)
        payloads=xt.datagrams(xt.packetize(frame,sn=k+1,chunk=1400))
        groups.append((len(packets)<<16)|len(payloads))
        reference=xt.RxModel(hdr_le=True,pix_le=True,subhdr=20,amp_min=0,raw_inv=64000)
        for pl in payloads:
            reference.feed(pl,False)
            packet=netpkt.udp_frame(0x024e56000001,0x021337000002,'192.168.0.101','192.168.0.100',7687,7687,pl,fcs=True)
            assert len(packet)<=1518
            packets.append((len(wire)<<32)|(len(packet)<<16)|len(pl));wire.extend(packet)
        assert len(reference.frames)==1
        expected=reference.frames[0]
        assert expected['w']==160 and expected['h']==60
        for v in range(60):
            for u in range(160):pixels.append(((int(u==0 and v==0))<<33)|(int(u==159 and v==59)<<32)|(u<<24)|(v<<16)|expected['depth'][v][u])
    for name,values,width in [('wire',wire,2),('packets',packets,16),('groups',groups,8),('pixels',pixels,9)]:
        (out/(name+'.hex')).write_text(''.join(f'{v:0{width}x}\n'for v in values))
    runs=[]
    cmds=[('xvlog',[str(out/Path(n).name)for n in NAMES]),('xvlog',['-sv',str(out/tb.name)]),
          ('xelab',['-debug','off','-timescale','1ns/1ps','tb_mac_udp_xt_reset','-s','network']),('xsim',['network','-runall'])]
    for tool,args in cmds:
        log=out/(f'{len(runs)}_{tool}.log')
        with log.open('w')as f:
            r=subprocess.run(['cmd','/c',str(VIV/(tool+'.bat')),*args],cwd=out,stdout=f,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
        runs.append(dict(tool=tool,exit_code=r.returncode,log=str(log)))
        assert r.returncode==0,(tool,str(log))
    text=(out/'xsim.log').read_text(errors='replace')
    import re
    line=re.search(r'MAC UDP XT RESET PASSED[^\r\n]*',text)
    assert line and 'Fatal:'not in text
    counts={k:int(v)for k,v in re.findall(r'(\w+)=(\d+)',line[0])}
    assert counts['frames']==4 and counts['pixels']==38400 and counts['resets']==2
    assert all(same_bytes(SOURCE/n,out/Path(n).name)for n in NAMES)
    result=dict(passed=True,recorded_at=datetime.now().astimezone().isoformat(),source=str(SOURCE),probe=str(out),
        metrics=counts,full_size_frames=[160,60],actual_MAC_FIFO_RAW1_UDP_XT_sources=True,
        root_and_RX_common_reset_topology_bound_to_frozen_top=True,original_FIFO_depths_and_XT_capacity=True,
        stopped_RX_and_running_RX_reset_with_DV_high_tail_rejected=True,invalid_final_FCS_frame_rejected=True,
        independent_python_XT_expected_all_pixels=True,XT_backpressure_stability_checked=True,
        simulated_core_period_ns=20,PHY_RX_period_ns=40,RX_falling_sampling=True,PHY_data_phase_ns_after_rising=2,
        no_internal_force=True,no_original_source_or_active_PNR_changed=True,new_SYN_PNR=False,serial_opened=False,
        full_G2_CPU_RL_or_physical_acceptance=False,programmed=False,deployment_accepted=False,
        remaining=['Full G2/CPU/UART loss-stop and independent ESTOP composition','All async reset sequences and physical CDC/reset/IO/timing signoff'],
        runs=runs,source_bindings=[metadata(SOURCE/n)for n in NAMES]+[metadata(SOURCE/'g2_top.v'),metadata(SOURCE/'nav/rtl/top/nav_subsystem.v')],
        probe_bindings=[metadata(out/Path(n).name)for n in NAMES]+[metadata(out/tb.name)],reference_bindings=[metadata(ROOT/'model/xt_proto.py'),metadata(ROOT/'model/netpkt.py')])
    save(out/'result.json',result);return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    me=psutil.Process();me.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
    try:r=probe(a.out)
    except Exception as e:
        import traceback
        save(a.out/'failed.json',dict(error=str(e),traceback=traceback.format_exc()));raise
    print(json.dumps(dict(passed=r['passed'],metrics=r['metrics'],new_SYN_PNR=False)))
