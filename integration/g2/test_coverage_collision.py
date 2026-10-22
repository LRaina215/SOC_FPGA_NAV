"""Poison same-address RAM read/write to check coverage forwarding consumers.

Reuse the existing independent capture-age oracle, actual full-size coverage
RTL and directed memory. This does not promise primitive read-first behavior.
"""
import argparse,json,re,subprocess,sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--static-bounds',action='store_true');a=ap.parse_args()
    out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
    subprocess.run([sys.executable,'-X','utf8',str(ROOT/'sim/gen/gen_support_coverage.py'),str(out)],check=True)
    # Duplicate an already received point within its capture epoch. The oracle's
    # distinct-epoch count and age do not change, but the 3-cycle write pipeline
    # must now mask many actual simultaneous same-address reads and writes.
    operations=[];last=None;extra_points=0
    for line in (out/'coverage.ops').read_text().splitlines():
        if line.startswith(('F ','E ')) or line=='C':last=None
        if line.startswith('P '):last=line
        if line=='D' and last:
            operations.extend([last]*32);extra_points+=32;last=None
        operations.append(line)
    (out/'coverage.ops').write_text('\n'.join(operations)+'\n')
    memory=(ROOT/'integration/g2/nav_ram_sdp_directed.v').read_text().replace('module nav_ram_sdp #(','module nav_ram_impl #(',1)
    memory+='''
module nav_ram_sdp #(parameter DW=16,AW=10,DEPTH=(1<<AW),
 parameter INIT_FILE="",parameter [DW-1:0] INIT_VAL={DW{1'b0}})
 (input clk,we,input [AW-1:0] waddr,input [DW-1:0] wdata,
 input re,input [AW-1:0] raddr,output [DW-1:0] rdata);
 wire [DW-1:0] q;reg poisoned=0;integer collision_count=0;
 nav_ram_impl #(.DW(DW),.AW(AW),.DEPTH(DEPTH),.INIT_FILE(INIT_FILE),.INIT_VAL(INIT_VAL))
   inner(clk,we,waddr,wdata,re,raddr,q);
 always @(posedge clk)if(re)begin
   poisoned<=we && waddr==raddr;
   if(we && waddr==raddr)collision_count<=collision_count+1;
 end
 assign rdata=poisoned?{DW{1'bx}}:q;
endmodule
'''
    (out/'memory.v').write_text(memory,encoding='utf-8')
    tb=(ROOT/'rtl/map/tb/tb_support_coverage.sv').read_text()
    tb=tb.replace('$display("TEST PASSED");', '''
    if(dut.u_ram.collision_count<1000)$fatal(1,"too few poisoned collisions exercised");
    $display("POISONED COLLISIONS %0d",dut.u_ram.collision_count);
    $display("TEST PASSED");''')
    coverage_sources=[ROOT/'rtl/map/support_coverage.v']
    if a.static_bounds:
        from prepare_coverage_timing import generate
        old=(ROOT/'rtl/map/support_coverage.v').read_text()
        (out/'coverage_original.v').write_text(old.replace('module support_coverage (','module support_coverage_old (',1))
        generate(out/'coverage_candidate.v')
        new=(out/'coverage_candidate.v').read_text().replace('module support_coverage (','module support_coverage_new (',1)
        (out/'coverage_candidate.v').write_text(new)
        header=old.split(');',1)[0]+');\n'
        header=header.replace('output reg','output wire')
        inputs='clk rst clear age_epoch frame_start frame_cx frame_cy capture_epoch capture_valid s_valid s_upd s_addr cpu_req cpu_addr'.split()
        outputs='s_ready cpu_ack cpu_q busy'.split()
        common=','.join('.'+x+'('+x+')' for x in inputs)
        wrapper=header+'wire old_ready,old_ack,old_busy;wire [31:0] old_q;\n'
        wrapper+='support_coverage_old original('+common+',.s_ready(old_ready),.cpu_ack(old_ack),.cpu_q(old_q),.busy(old_busy));\n'
        wrapper+='support_coverage_new candidate('+common+','+','.join('.'+x+'('+x+')' for x in outputs)+');\n'
        wrapper+='''always @(negedge clk)begin
 if({s_ready,cpu_ack,cpu_q,busy}!=={old_ready,old_ack,old_q,old_busy})$fatal(1,"coverage external cycle mismatch");
 if({original.ram_we,original.ram_re}!=={candidate.ram_we,candidate.ram_re})$fatal(1,"RAM enables changed");
 if(original.ram_we && {original.ram_wa,original.ram_wd}!=={candidate.ram_wa,candidate.ram_wd})$fatal(1,"RAM write changed");
 if(original.ram_re && original.ram_ra!==candidate.ram_ra)$fatal(1,"RAM read changed");
end
endmodule
'''
        (out/'coverage_wrapper.v').write_text(wrapper)
        coverage_sources=[out/'coverage_original.v',out/'coverage_candidate.v',out/'coverage_wrapper.v']
        tb=tb.replace('dut.u_ram','dut.candidate.u_ram')
    (out/'tb.sv').write_text(tb,encoding='utf-8')
    viv=Path('E:/2026.1/Vivado/bin')
    def run(name,args):
        with (out/(name+'_stdout.log')).open('w') as log:
            subprocess.run(['cmd','/c',str(viv/(name+'.bat')),*args],cwd=out,stdout=log,stderr=subprocess.STDOUT,check=True)
    run('xvlog',[str(out/'memory.v'),*[str(x) for x in coverage_sources]])
    run('xvlog',['-sv',str(out/'tb.sv')])
    run('xelab',['-debug','off','-timescale','1ns/1ps','tb_support_coverage','-s','snap'])
    run('xsim',['snap','-runall']);s=(out/'xsim.log').read_text();assert 'TEST PASSED' in s and 'Fatal:' not in s
    counts=re.search(r'SUPPORT queries=(\d+) points=(\d+)',s)
    result=dict(pass_=True,queries=int(counts[1]),points=int(counts[2]),
        poisoned_collisions=int(re.search(r'POISONED COLLISIONS (\d+)',s)[1]),
        depth=16384,width=9,extra_same_epoch_duplicates=extra_points,
        static_bounds=a.static_bounds,external_and_ram_ports_cycle_exact=a.static_bounds,
        oracle='existing independent CompactCoverage capture-time vectors; same-epoch duplicates leave distinct-epoch count unchanged',
        scope='coverage consumer tolerates unknown colliding read data for these vectors; not all callers, gate timing or hardware collision guarantee')
    (out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))


if __name__=='__main__':main()
