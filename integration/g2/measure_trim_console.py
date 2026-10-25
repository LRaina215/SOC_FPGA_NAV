"""Check only the SoC bus consequences of discarding offline console storage.

The unchanged PicoRV32 core is represented by a bus driver, so this does not
claim full firmware execution. RAM capacity, byte writes, navigation requests,
CPU peripheral write ACK, HOLD/download and debug protocol are exercised.
"""
import argparse
from datetime import datetime
import json
from pathlib import Path

from prepare_lf_line_compact import simulate, save
from trim_peripheral_rtl import trim_console_storage


STUB = r'''module picorv32 #(
parameter ENABLE_COUNTERS=1,ENABLE_COUNTERS64=0,ENABLE_REGS_16_31=1,ENABLE_REGS_DUALPORT=1,
BARREL_SHIFTER=1,TWO_CYCLE_COMPARE=0,TWO_CYCLE_ALU=0,COMPRESSED_ISA=1,CATCH_MISALIGN=1,CATCH_ILLINSN=1,
ENABLE_PCPI=0,ENABLE_MUL=0,ENABLE_FAST_MUL=1,ENABLE_DIV=1,ENABLE_IRQ=0,ENABLE_TRACE=0,PROGADDR_RESET=0,STACKADDR=0
)(input clk,resetn,output trap,output reg mem_valid=0,output reg mem_instr=0,
input mem_ready,output reg[31:0]mem_addr=0,output reg[31:0]mem_wdata=0,output reg[3:0]mem_wstrb=0,
input[31:0]mem_rdata,output mem_la_read,mem_la_write,output[31:0]mem_la_addr,mem_la_wdata,output[3:0]mem_la_wstrb,
output pcpi_valid,output[31:0]pcpi_insn,pcpi_rs1,pcpi_rs2,input pcpi_wr,input[31:0]pcpi_rd,input pcpi_wait,pcpi_ready,
input[31:0]irq,output[31:0]eoi,output trace_valid,output[35:0]trace_data);
assign trap=0;assign mem_la_read=0;assign mem_la_write=0;assign mem_la_addr=0;assign mem_la_wdata=0;
assign mem_la_wstrb=0;assign pcpi_valid=0;assign pcpi_insn=0;assign pcpi_rs1=0;assign pcpi_rs2=0;
assign eoi=0;assign trace_valid=0;assign trace_data=0;
endmodule
'''
TB = r'''`timescale 1ns/1ps
module tb;
reg clk=0,rst=1;always #10 clk=~clk;
reg dr=0,dw=0;reg[15:0]da=0;reg[31:0]dd=0;
wire dq0,dq1,mr0,mr1,mw0,mw1,tr0,tr1;wire[15:0]ma0,ma1;wire[31:0]md0,md1,q0,q1;wire[3:0]led0,led1;
nav_soc_old #(.RAM_KB(128)) a(.clk(clk),.rst(rst),.m_req(mr0),.m_we(mw0),.m_addr(ma0),.m_wdata(md0),
 .m_ack(1'b1),.m_rdata(32'h4500abcd),.d_req(dr),.d_we(dw),.d_addr(da),.d_wdata(dd),.d_ack(dq0),.d_rdata(q0),.led(led0),.trap(tr0));
nav_soc_trim #(.RAM_KB(128)) b(.clk(clk),.rst(rst),.m_req(mr1),.m_we(mw1),.m_addr(ma1),.m_wdata(md1),
 .m_ack(1'b1),.m_rdata(32'h4500abcd),.d_req(dr),.d_we(dw),.d_addr(da),.d_wdata(dd),.d_ack(dq1),.d_rdata(q1),.led(led1),.trap(tr1));
integer cycles=0,cpu_ops=0,debug_ops=0,discarded_logs=0;reg[31:0]reply0,reply1;
always@(negedge clk)begin
 cycles++;
 if({mr0,mw0,ma0,md0,led0,tr0,dq0,a.mem_ready,a.cpu_rst}!=={mr1,mw1,ma1,md1,led1,tr1,dq1,b.mem_ready,b.cpu_rst})
  $fatal(1,"main SoC protocol changed cycle=%0d",cycles);
 if(a.mem_ready && a.mem_rdata!==b.mem_rdata)$fatal(1,"CPU read response changed");
 if(dq0 && !(da[15:12]==4'he) && q0!==q1)$fatal(1,"non-console debug response changed");
end
task cpu(input[31:0]addr,input[31:0]data,input[3:0]mask,input[31:0]expected_reply);
 integer n;
 begin
 @(negedge clk);#1;
 a.u_cpu.mem_valid=1;a.u_cpu.mem_addr=addr;a.u_cpu.mem_wdata=data;a.u_cpu.mem_wstrb=mask;
 b.u_cpu.mem_valid=1;b.u_cpu.mem_addr=addr;b.u_cpu.mem_wdata=data;b.u_cpu.mem_wstrb=mask;
 n=0;
 do begin @(posedge clk);#2;n++;if(n>30)$fatal(1,"CPU write/read ACK stalled");end while(!a.mem_ready);
 if(mask==0 && (a.mem_rdata!==expected_reply || b.mem_rdata!==expected_reply))$fatal(1,"CPU result mismatch addr=%h got=%h expected=%h",addr,a.mem_rdata,expected_reply);
 cpu_ops++;@(negedge clk);#1;a.u_cpu.mem_valid=0;b.u_cpu.mem_valid=0;
 repeat(3)@(negedge clk);
 end
endtask
task debug(input bit write,input[15:0]addr,input[31:0]data);
 integer n;
 begin
 @(negedge clk);#1;dr=1;dw=write;da=addr;dd=data;n=0;
 do begin @(posedge clk);#2;n++;if(n>30)$fatal(1,"debug ACK stalled");end while(!dq0);
 if(!dq1)$fatal(1,"debug ACK cycle changed");reply0=q0;reply1=q1;
 debug_ops++;@(negedge clk);#1;dr=0;repeat(3)@(negedge clk);
 end
endtask
initial begin
 repeat(8)@(negedge clk);#1;rst=0;repeat(6)@(negedge clk);
 cpu(32'h00000000,32'h11223344,15,0);cpu(0,0,0,32'h11223344);
 cpu(32'h0001fffc,32'h89abcdef,15,0);cpu(32'h0001fffc,32'h01234567,5,0);
 cpu(32'h0001fffc,0,0,32'h8923cd67);
 for(integer i=0;i<32;i++)begin cpu(32'h80000000,32'h20+i,1,0);discarded_logs++;end
 cpu(32'h80000004,9,15,0);cpu(32'h80000008,32'hdead4500,15,0);
 cpu(32'h8000000c,0,0,32'd131072);
 cpu(32'h40007044,32'h00012345,15,0);cpu(32'h40000008,0,0,32'h4500abcd);
 debug(0,16'hf00c,0);if(reply0!=32 || reply1!=32)$fatal(1,"console count changed");
 debug(0,16'he000,0);if(reply0!==32'h23222120 || reply1!==0)$fatal(1,"offline log trade mismatch");
 debug(0,16'hf010,0);if(reply0!==32'hdead4500 || reply1!==reply0)$fatal(1,"status changed");
 debug(1,16'hf000,1);debug(1,16'hf004,32767);debug(1,16'hf008,32'ha55a4500);
 debug(1,16'hf004,32767);debug(0,16'hf008,0);
 if(reply0!==32'ha55a4500 || reply1!==reply0)$fatal(1,"HOLD download/RAM read failed");
 debug(1,16'hf000,0);cpu(32'h0001fffc,0,0,32'ha55a4500);
 $display("TEST PASSED TRIM_CONSOLE cycles=%0d cpu_ops=%0d debug_ops=%0d log_writes_discarded=%0d CPU128KiB=1 highest_word=1 byte_writes=1 nav_forwarding=1 hold_download_preserved=1",cycles,cpu_ops,debug_ops,discarded_logs);$finish;
end
initial begin #1000000;$fatal(1,"SoC trim proof timeout");end
endmodule
'''


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--source',type=Path,required=True);ap.add_argument('--out',type=Path,required=True)
    a=ap.parse_args();source=a.source.resolve();out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
    old=(source/'nav/rtl/soc/nav_soc.v').read_text(encoding='utf-8');new=trim_console_storage(old)
    (out/'nav_soc_trim.v').write_text(new,encoding='utf-8',newline='\n')
    (out/'old.v').write_text(old.replace('module nav_soc #(', 'module nav_soc_old #(',1),encoding='utf-8')
    (out/'new.v').write_text(new.replace('module nav_soc #(', 'module nav_soc_trim #(',1),encoding='utf-8')
    for name in ('nav_ram_sp.v','nav_ram_sdp.v'):(out/name).write_bytes((source/'nav/rtl/common'/name).read_bytes())
    (out/'cpu_bus_driver.v').write_text(STUB,encoding='utf-8');(out/'tb.sv').write_text(TB,encoding='utf-8')
    proof=simulate(out,[out/n for n in ('nav_ram_sp.v','nav_ram_sdp.v','cpu_bus_driver.v','old.v','new.v','tb.sv')],'tb')
    result=dict(proof,source=str(source),created_at=datetime.now().astimezone().isoformat(),
        passed=True,console_storage_contract_passed=True,CPU128KiB_hold_download_and_nav_protocol_passed=True,
        original_PicoRV32_source_unchanged=True,CPU_program_execution_verified=False,
        offline_console_readback_expected_zero=True,programmed=False,SYN_started=False,PNR_started=False)
    save(out/'result.json',result);print(json.dumps({k:result[k]for k in('passed','result')},ensure_ascii=False))


if __name__=='__main__':main()
