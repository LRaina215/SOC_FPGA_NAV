"""Isolated cache/boot-ROM synthesis with bounded owned-process protection.

Requires the same 2/4 GiB startup reserve as G2. No full joint SYN or PNR.
"""
import argparse
import json
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent))
from measure_slf_row_banks import run_syn,inventory,TOOL
from retry_frozen_synthesis import memory_status
from preflight import validate


def measure(candidate,out):
    m=memory_status()
    assert m.avail_phys>=2*1024**3 and m.avail_page>=4*1024**3,'2/4 GiB startup gate'
    assert not out.exists()
    out.mkdir(parents=True)
    for f in ('nav_ddr_cache.v','nav_ddr_boot_rom.v'):shutil.copyfile(candidate/f,out/f)
    for f in candidate.glob('boot_b*.hex'):shutil.copyfile(f,out/f.name)
    assert (out/'nav_ddr_cache.v').read_bytes()==(HERE/'nav_ddr_cache.v').read_bytes()
    top='''module g2_top(input clk,cold_rst,req,input[14:0]word_addr,input[31:0]wdata,
input[3:0]wstrb,output ack,output[31:0]rdata,output initialized,
output line_req,line_we,output[24:0]line_addr,output[255:0]line_wdata,
output[31:0]line_mask,input line_ack,input[255:0]line_rdata);
nav_ddr_cache u_memory
(clk,cold_rst,req,word_addr,wdata,wstrb,ack,rdata,initialized,line_req,line_we,
line_addr,line_wdata,line_mask,line_ack,line_rdata);
endmodule
'''
    (out/'top.v').write_text(top)
    (out/'clock.sdc').write_text('create_clock -name clk -period 20 [get_ports {clk}]\n')
    settings=['set_device -device_version C GW5AT-LV138PG484AC1/I0']
    settings += [f'add_file -type verilog {{{(out/f).as_posix()}}}' for f in ('nav_ddr_boot_rom.v','nav_ddr_cache.v','top.v')]
    settings += [f'add_file -type sdc {{{(out/"clock.sdc").as_posix()}}}',
        'set_option -top_module g2_top','set_option -output_base_name g2_top',
        'set_option -global_freq 50','set_option -convert_sdp32_36_to_sdp16_18 0',
        'set_option -looplimit 100000']
    effective=validate(TOOL,out,settings,50,0)
    (out/'run.tcl').write_text('\n'.join(settings+['run syn','exit'])+'\n')
    runtime=run_syn(out,scope='DDR cache/boot ROM only; not full G2 or physical placement')
    syn=out/'impl/gwsynthesis'
    result=dict(candidate=str(candidate),build=str(out),runtime=runtime,
        startup_available_physical=m.avail_phys,startup_available_commit=m.avail_page,
        effective_options=effective,resources=ET.parse(syn/'g2_top_syn_rsc.xml').getroot().attrib,
        inventory=inventory(syn/'g2_top.vg'),programmed=False,physical_resources_verified=False)
    (out/'result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('candidate',type=Path);p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();print(json.dumps(measure(a.candidate.resolve(),a.out.resolve()),indent=2))
