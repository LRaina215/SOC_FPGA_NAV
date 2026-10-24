"""Original53-bit elevation arithmetic vs stalled DDR-backed arithmetic."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import re
import shutil
import subprocess

from derive_ddr_matcher import HERE,ROOT
from derive_ddr_elevation import derive,derive_traversability
FROZEN=Path('F:/nav_integration_g2_20261023/sfp_lane2_private_snapshot_syn11_retry1/nav')


def run(out,trace=False):
    assert not out.exists();out.mkdir()
    source=(FROZEN/'rtl/map/elev_map.v').read_text(encoding='utf-8-sig')
    assert source==(ROOT/'rtl/map/elev_map.v').read_text(encoding='utf-8-sig')
    (out/'elev_map.v').write_text(source,encoding='utf-8')
    (out/'elev_map_ddr.v').write_text(derive(source),encoding='utf-8')
    # Derived scan kernel is prepared, but this test does NOT exercise it.
    (out/'trav_v3_ce.v').write_text(derive_traversability((FROZEN/'rtl/map/trav_v3.v').read_text(encoding='utf-8-sig')),encoding='utf-8')
    shutil.copyfile(FROZEN/'rtl/common/nav_ram_sdp.v',out/'nav_ram_sdp.v')
    names=['nav_ddr_dual_step.v','nav_ddr_map_cache.v','nav_ddr_elevation_store.v','nav_ddr_native.v']
    for n in names+['tb_ddr_elevation.sv']:shutil.copyfile(HERE/n,out/n)
    if trace:
        p=out/'tb_ddr_elevation.sv';s=p.read_text()
        s=s.replace('    always@(posedge clk)begin cycles++;', '''    always@(posedge clk)if(!rst&&ce&&((dut.acc&&a==15)||(dut.fw1_v&&dut.fw1_a==15)||(dut.v2&&dut.a2==15)))begin
        $display("TRACE cycle=%0d accept=%b a=%h tag=%h z=%h v2=%b a2=%h tag2=%h old=%h new=%h m=%b%b%b fw=%b/%h/%h q2=%h",cycles,dut.acc,a,tag,z,dut.v2,dut.a2,dut.tag2,dut.old,dut.nw,dut.m1,dut.m2,dut.m3,dut.fw1_v,dut.fw1_a,dut.fw1_d,dut.q2);
        $display("DETAIL q=%h bridgeq=%h held_read=%b state=%d result=%h re=%b ra=%h we=%b wa=%h",dut.q,dut.u_memory.q,dut.u_memory.held_read,dut.u_memory.state,dut.u_memory.read_result,dut.ram_re,dut.ram_raddr,dut.ram_we,dut.ram_waddr);
    end
    always@(posedge clk)begin cycles++;''',1)
        p.write_text(s)
    model=(HERE/'tb_slf_row_update.sv').read_text().split('module tb_slf_row_update;')[0]
    for a,b in [('[0:8191]','[0:16383]'),('[12:0]held_addr','[13:0]held_addr'),
                ('addr[27:16]','addr[27:17]'),('addr[15:3]','addr[16:3]')]:
        assert model.count(a)==1,a
        model=model.replace(a,b)
    (out/'native_service_model.sv').write_text(model)
    commands=[('xvlog',names+['elev_map.v','elev_map_ddr.v','nav_ram_sdp.v']),
        ('xvlog',['-sv','native_service_model.sv','tb_ddr_elevation.sv']),
        ('xelab',['-debug','off','-timescale','1ns/1ps','tb_ddr_elevation','-s','snap']),
        ('xsim',['snap','-runall'])]
    for i,(tool,args) in enumerate(commands):
        with (out/f'{i}_{tool}_stdout.log').open('w') as log:
            subprocess.run(['cmd','/c',str(Path('E:/2026.1/Vivado/bin')/(tool+'.bat')),*args],
                cwd=out,stdout=log,stderr=subprocess.STDOUT,timeout=600,check=True,
                creationflags=subprocess.CREATE_NO_WINDOW)
    s=(out/'3_xsim_stdout.log').read_text(errors='replace')
    assert 'TEST PASSED' in s and not re.search(r'FATAL|Fatal|ERROR:',s),s[-3000:]
    def lines(prefix):
        return [{k:float(v) if '.' in v else int(v) for k,v in re.findall(r'(\w+)=([\d.]+)',l)}
                for l in s.splitlines() if l.startswith(prefix)]
    result=dict(tested_at=datetime.now().astimezone().isoformat(),passed=True,
        metrics={k:int(v) for k,v in re.findall(r'(\w+)=(\d+)',re.search(r'TEST PASSED[^\r\n]*',s)[0])},
        clear=lines('ELEV_CLEAR'),frames=lines('ELEV_FRAME'),
        scope='original elevation point arithmetic/CPU reads plus modeled DDR cache/native service',
        full_capacity=16384,word_bits=53,traversability_scan_verified=False,
        shared_structure_arbitration_verified=False,hardware_service_bound_verified=False,
        full_joint_implementation=False,programmed=False)
    (out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--trace',action='store_true')
    a=p.parse_args();run(a.out.resolve(),a.trace)
