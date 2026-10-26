`timescale 1ns/1ps
module tb_stage2b_console_bus_miter;
 reg clk=0,run_clock=1,rst=1;
 always begin #10;if(run_clock)clk=~clk;else clk=0;end
 reg cv=0;reg[31:0]ca=0,cw=0;reg[3:0]cs=0;
 reg dr=0,dw=0;reg[15:0]da=0;reg[31:0]dd=0;
 wire r0,r1,cr0,cr1,ak0,ak1;wire[31:0]rq0,rq1,dq0,dq1;
 console_reference #(.RAM_KB(128)) ref0(.clk(clk),.rst(rst),.proof_mem_valid(cv),.proof_mem_addr(ca),.proof_mem_wdata(cw),.proof_mem_wstrb(cs),.proof_mem_ready(r0),.proof_mem_rdata(rq0),.proof_cpu_rst(cr0),
 .m_req(),.m_we(),.m_addr(),.m_wdata(),.m_ack(1'b0),.m_rdata(32'd0),.d_req(dr),.d_we(dw),.d_addr(da),.d_wdata(dd),.d_ack(ak0),.d_rdata(dq0),.led(),.trap());
 console_candidate #(.RAM_KB(128)) dut(.clk(clk),.rst(rst),.proof_mem_valid(cv),.proof_mem_addr(ca),.proof_mem_wdata(cw),.proof_mem_wstrb(cs),.proof_mem_ready(r1),.proof_mem_rdata(rq1),.proof_cpu_rst(cr1),
 .m_req(),.m_we(),.m_addr(),.m_wdata(),.m_ack(1'b0),.m_rdata(32'd0),.d_req(dr),.d_we(dw),.d_addr(da),.d_wdata(dd),.d_ack(ak1),.d_rdata(dq1),.led(),.trap());
 reg[7:0]gold[0:4095];reg[31:0]gwp=0,expected_console=0;
 integer checks=0,log_writes=0,debug_reads=0,posted=0,stops=0,resets=0;
 always@(posedge clk)begin
  if(ref0.con_we)begin gold[gwp[11:0]]<=cw[7:0];log_writes++;end
  if(rst)gwp<=0;else if(!ref0.cpu_rst && ref0.con_we)gwp<=gwp+1;
  if(!rst&&ref0.d_st==1&&ref0.d_con)begin
   expected_console<={gold[{ref0.con_ra,2'b00}+3],gold[{ref0.con_ra,2'b00}+2],gold[{ref0.con_ra,2'b00}+1],gold[{ref0.con_ra,2'b00}]};debug_reads++;
  end
  if(dut.con_post_valid)begin
   if(dut.con_read_needed||dut.con_we)$fatal(1,"queued commit overlaps demanded read or new CPU request");
   posted++;
  end
  if(dut.con_read_needed&&dut.con_commit)$fatal(1,"physical CE overlap independent of AD[13:4]");
 end
 always@(negedge clk)if($time>0&&!rst)begin
  checks++;
  if($isunknown({r0,r1,cr0,cr1,ak0,ak1}))$fatal(1,"unknown bus control");
  if({r0,cr0,ak0}!=={r1,cr1,ak1})$fatal(1,"bus ACK/reset cycle mismatch");
  if(r0&&($isunknown({rq0,rq1})||rq0!==rq1))$fatal(1,"CPU response mismatch");
  if(ref0.con_wp!==dut.con_wp||ref0.con_wp!==gwp)$fatal(1,"console pointer mismatch");
  if(ak0)begin
   if($isunknown({dq0,dq1})||dq0!==dq1)$fatal(1,"debug data mismatch");
   if(da[15:12]==4'he&&dq0!==expected_console)$fatal(1,"independent pre-write console scoreboard mismatch");
  end
 end
 task cpu_byte(input[7:0]x);
  begin
   @(negedge clk);#1;cv=1;ca=32'h80000000;cw={24'd0,x};cs=1;
   do @(posedge clk);while(!r0);
   @(negedge clk);#1;cv=0;cs=0;
  end
 endtask
 task debug(input bit wr,input[15:0]a,input[31:0]d);
  begin
   @(negedge clk);#1;dr=1;dw=wr;da=a;dd=d;
   do @(posedge clk);while(!ak0);
   @(negedge clk);#1;dr=0;repeat(2)@(negedge clk);
  end
 endtask
 task conflict(input bit adjacent,input bit reset_edge,input bit pause_edge);
  reg[15:0]addr;
  begin
   addr=16'he000|(({6'd0,gwp[11:2]}^(adjacent?16'd1:16'd0))<<2);
   @(negedge clk);#1;dr=1;dw=0;da=addr;
   @(posedge clk);@(negedge clk);#1;cv=1;ca=32'h80000000;cw=32'hca;cs=1;
   @(posedge clk);#1;if(!dut.con_post_valid)$fatal(1,"collision stimulus did not create a posted byte");
   @(negedge clk);#1;
   if(pause_edge)begin run_clock=0;stops++;#200;if(!dut.con_post_valid)$fatal(1,"stopped clock lost pending byte");end
   if(reset_edge)begin rst=1;cv=0;cs=0;dr=0;resets++;end
   if(pause_edge)begin
    // A key pulse while stopped cannot cancel accepted data.
    if(!reset_edge)begin rst=1;#40;rst=0;end
    run_clock=1;
   end
   @(posedge clk);@(negedge clk);#1;cv=0;cs=0;
   if(reset_edge)begin repeat(2)@(negedge clk);rst=0;end
   else begin
    do @(posedge clk);while(!ak0);
    @(negedge clk);#1;dr=0;
   end
   repeat(3)@(negedge clk);
   if(dut.con_post_valid)$fatal(1,"pending byte not drained after next edge/reset/resume");
  end
 endtask
 initial begin
  for(integer i=0;i<4096;i++)gold[i]=0;
  repeat(5)@(negedge clk);rst=0;
  for(integer i=0;i<4100;i++)cpu_byte(i[7:0]^8'h5a);
  for(integer lane=0;lane<4;lane++)begin
   conflict(0,0,0);conflict(1,0,0);conflict(0,0,1);conflict(0,1,0);conflict(1,1,1);
  end
  debug(1,16'hf000,1);debug(0,16'hf00c,0);debug(0,16'he000,0);debug(1,16'hf000,0);
  for(integer i=0;i<1024;i++)debug(0,16'he000|(i<<2),0);
  repeat(5)@(negedge clk);
  if(posted<20||stops<8||resets<8||log_writes<4120||debug_reads<1024)$fatal(1,"incomplete bus cases");
  $display("TEST PASSED CONSOLE BUS checks=%0d writes=%0d reads=%0d posted=%0d stopped=%0d resets=%0d",checks,log_writes,debug_reads,posted,stops,resets);$finish;
 end
 initial begin #3000000;$fatal(1,"finite console bus stimulus watchdog");end
endmodule
