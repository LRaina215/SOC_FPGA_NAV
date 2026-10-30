"""Prepare an E0 read-decode candidate from the completed video SYN2 inputs.

All data remains live at original E1; only the address selection is registered
on the same E0 as a_q. No EDA is started by this preparation/proof script.
"""
from pathlib import Path
from datetime import datetime
import argparse,json,re,shutil
from prove_soc_console_remap import z3

BASE=Path('F:/nav_integration_g2_20261023/K_video160_20261030_prepared4')
RF='nav/rtl/regs/nav_regfile.v';TOP='nav/rtl/top/nav_subsystem.v';BIND='nav/rtl/regs/nav_subsystem_rf.vh'

def generate(out):
    assert not out.exists();shutil.copytree(BASE,out)
    for f in out.rglob('*'):
        if f.is_file()and f.suffix in {'.v','.vh','.tcl'}:
            t=f.read_text(encoding='utf-8-sig')
            if BASE.as_posix()in t:f.write_text(t.replace(BASE.as_posix(),out.as_posix()),encoding='utf-8',newline='\n')
    original=(BASE/RF).read_text(encoding='utf-8')
    marker="    always @(*) begin\n        hit = 1'b1;\n        case (addr)"
    start=original.index(marker);end=original.index('\nendmodule',start)
    oldread=original[start:end]
    pairs=re.findall(r"16'h([0-9a-fA-F]{4}): rdata = ([^;]+);",oldread)
    assert len(pairs)>100 and len(pairs)==len({a for a,_ in pairs})
    assert all(int(a,16)%4==0 and re.fullmatch(r"(?:[ir]_\w+|32'h0)",e)for a,e in pairs)
    n=len(pairs)
    decode="rf_read_select_q <= 0;\n                case ({bus_addr[15:2],2'b00})\n"
    decode+=''.join("                16'h"+a+': rf_read_select_q['+str(i)+"] <= 1'b1;\n"for i,(a,e)in enumerate(pairs))
    decode+='                default: ;\n                endcase\n'
    mux='    always @* begin\n        hit=|read_select;\n        rdata=32\'d0;\n'
    mux+=''.join('        rdata=rdata | ({32{read_select['+str(i)+']}} & '+e+');\n'for i,(a,e)in enumerate(pairs))
    mux+='    end\n'
    updated=original.replace('module nav_regfile (','module nav_regfile #(parameter PREDECODED_READ=0) (',1)
    updated=updated.replace('    input  wire [15:0] addr,','    input  wire [15:0] addr,\n    input wire ['+str(n-1)+':0] read_select,',1)
    updated=updated.replace(oldread,'    generate if(PREDECODED_READ)begin:g_read_predecoded\n'+mux+'    end else begin:g_read_original\n'+oldread+'\n    end endgenerate',1)
    (out/RF).write_text(updated,encoding='utf-8',newline='\n')
    top=(out/TOP).read_text(encoding='utf-8')
    anchor='    reg  [15:0] a_q;';assert top.count(anchor)==1
    top=top.replace(anchor,anchor+'\n    // Private selection: overwritten at every accepted E0 before B_RF. No new reset load.\n    reg ['+str(n-1)+':0] rf_read_select_q;',1)
    anchor="                a_q <= {bus_addr[15:2], 2'b00}; wd_q <= bus_wdata;";assert top.count(anchor)==1
    top=top.replace(anchor,anchor+'\n                '+decode,1)
    (out/TOP).write_text(top,encoding='utf-8',newline='\n')
    bind=(out/BIND).read_text(encoding='utf-8');assert bind.count('nav_regfile u_rf (')==1
    bind=bind.replace('nav_regfile u_rf (','nav_regfile #(.PREDECODED_READ(1)) u_rf (\n        .read_select(rf_read_select_q),',1)
    (out/BIND).write_text(bind,encoding='utf-8',newline='\n')
    addr=z3.BitVec('address',16);values=[z3.Bool('data_bit_'+str(i))if e!="32'h0"else z3.BoolVal(False)for i,(a,e)in enumerate(pairs)]
    before=z3.BoolVal(False);after=z3.BoolVal(False);hit=z3.BoolVal(False)
    for (a,e),value in zip(reversed(pairs),reversed(values)):before=z3.If(addr==int(a,16),value,before)
    for (a,e),value in zip(pairs,values):
        selected=addr==int(a,16);after=z3.Or(after,z3.And(selected,value));hit=z3.Or(hit,selected)
    solver=z3.Solver();solver.set(timeout=20000);solver.add(before!=after);result=solver.check();assert result==z3.unsat,result
    probe=out/'isolated_read_decode_probe';probe.mkdir()
    # Exercise real old/new register files, not a substituted data model.
    (probe/'reference.v').write_text(original.replace('module nav_regfile (','module nav_regfile_ref (',1),encoding='utf-8')
    header=original.split(');',1)[0]
    ports=re.findall(r'^\s*(input|output)\s+(?:wire|reg)\s*(\[\d+:\d+\]\s*)?(\w+)',header,re.M)
    assert len(ports)>100
    declarations=[];connections=[[],[]];random=[];compares=[]
    for direction,width,name in ports:
        width=(width or '').strip()
        if direction=='input':
            if name not in ('clk','rst','wr','rd','addr','wdata'):
                declarations.append('reg '+width+' '+name+'=0;');random.append(name+'=$random;')
            connections[0].append('.'+name+'('+name+')');connections[1].append('.'+name+'('+name+')')
        else:
            for prefix in ('a_','b_'):declarations.append('wire '+width+' '+prefix+name+';')
            connections[0].append('.'+name+'(a_'+name+')');connections[1].append('.'+name+'(b_'+name+')')
            compares.append('if(a_'+name+'!==b_'+name+')$fatal(1,"'+name+' address=%h",addr);')
    tb='''`timescale 1ns/1ps
module tb_mmio_read_predecode;
 reg clk=0,rst=1,wr=0,rd=0,accept=0,write_request=0;
 reg[15:0]addr=0,bus_addr=0;reg[31:0]wdata=0,bus_wdata=0;
 reg [N-1:0] rf_read_select_q;integer checks=0;
 always #10 clk=~clk;
 always @(posedge clk)begin
  wr<=accept && write_request;rd<=accept && !write_request;
  if(!rst && accept)begin
   addr<={bus_addr[15:2],2'b00};wdata<=bus_wdata;
   DECODE
  end
 end
 DECLARATIONS
 nav_regfile_ref reference(CONNECT0);
 nav_regfile #(.PREDECODED_READ(1)) dut(.read_select(rf_read_select_q),CONNECT1);
 task check;
  begin COMPARES checks=checks+1;end
 endtask
 initial begin
  repeat(4)@(negedge clk);rst=0;
  for(integer k=0;k<16384;k=k+1)begin
   bus_addr=k*4;bus_wdata=$random;write_request=(k%7==0);accept=1;
   RANDOM
   @(negedge clk);check();
  end
  for(integer k=0;k<2048;k=k+1)begin
   bus_addr=$random;bus_wdata=$random;accept=(k%5!=0);write_request=(k%3==0);
   RANDOM
   @(negedge clk);check();
  end
  // Unknown address bits use the same exact-case default; no casex/force.
  accept=1;write_request=0;
  for(integer k=2;k<16;k=k+1)begin
   bus_addr=16'h5018;bus_addr[k]=1'bx;@(negedge clk);check();
   bus_addr=16'h5018;bus_addr[k]=1'bz;@(negedge clk);check();
  end
  accept=0;rst=1;repeat(3)@(negedge clk);rst=0;
  accept=1;bus_addr=16'h5028;@(negedge clk);check();
  $display("MMIO_READ_PREDECODE_PASS checks=%0d all_word_addresses=16384",checks);$finish;
 end
 initial begin #10000000;$fatal(1,"watchdog");end
endmodule
'''.replace('[N-1:0]','['+str(n-1)+':0]').replace('   DECODE\n','   '+decode+'\n').replace('DECLARATIONS','\n'.join(declarations)).replace('CONNECT0',','.join(connections[0])).replace('CONNECT1',','.join(connections[1])).replace('COMPARES','\n'.join(compares)).replace('RANDOM','\n'.join(random))
    (probe/'tb_mmio_read_predecode.sv').write_text(tb,encoding='utf-8')
    m=json.loads((out/'video_candidate_manifest.json').read_text(encoding='utf-8'))
    m.update(output=str(out),candidate='E0 registered read decode, original E1 data/ACK',read_select_bits=n,
        changed_from_video_prepared4=[TOP,RF,BIND],source_bit_miter=str(result),isolated_actual_RTL_simulation_pending=True,
        actual_resource_and_timing_pending=True,automatic_PNR=False)
    (out/'video_candidate_manifest.json').write_text(json.dumps(m,indent=2),encoding='utf-8')
    return dict(output=str(out),read_select_bits=n,all16bit_addresses_arbitrary_data_bit_miter=str(result),
        physical_time_or_resource_benefit_claimed=False,isolated_actual_RTL_probe=str(probe),EDA_started=False)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);a=p.parse_args();print(json.dumps(generate(a.out.resolve())))
