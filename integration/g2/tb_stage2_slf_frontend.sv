`timescale 1ns/1ps
module tb_stage2_slf_frontend;
 reg clk=0,rst=1;always #10 clk=~clk;
 reg [31:0] sm_addr=0,dbg_addr=0,wd=0;reg pw=0,dcmd=0,mcmd=0;
 reg busy=0,ack=0,edone=0;reg[17:0]q=0;
 wire req,done,ram_we;wire[13:0]ram_addr;wire[17:0]word;
 wire[31:0]error,profile;
 slf_frontend_cut dut(clk,rst,sm_addr,dbg_addr,wd,pw,dcmd,mcmd,busy,ack,edone,q,req,done,ram_we,ram_addr,word,error,profile);
 integer checks=0,issued=0;reg[17:0]last_invalid;
 always@(posedge clk)if(req)issued++;
 always@(negedge clk)if(!rst)begin
  checks++;
  if($isunknown({req,done,ram_we,error,profile}))$fatal(1,"unknown public frontend control");
  if(profile!==32'h534c0107)$fatal(1,"production geometry identity");
  if(ram_we && |sm_addr[31:14])$fatal(1,"invalid prior aliased to RAM");
 end
 task clock_edge;begin @(posedge clk);#1;end endtask
 task command(input[31:0]addr,input bit is_valid);
  begin @(negedge clk);#1;dbg_addr=addr;wd=1;dcmd=1;clock_edge();
   if(done!==!is_valid || req!==0)$fatal(1,"latest command ownership/invalid completion");
   @(negedge clk);#1;dcmd=0;wd=0;
  end
 endtask
 initial begin
  repeat(4)clock_edge();@(negedge clk);rst=0;
  // High bits, including bit31, must be rejected before 14bit narrowing.
  for(integer i=0;i<32;i++)begin
   @(negedge clk);sm_addr=(32'd1<<i);pw=1;#1;
   if(ram_we !== (i<14))$fatal(1,"prior range gate bit%0d",i);
   if(error[0] !== (i>=14))$fatal(1,"prior error bit%0d",i);
  end
  @(negedge clk);pw=0;
  // A valid command waits through existing work. Address CSR changes cannot
  // redirect the snapshot; a later invalid command supersedes its completion.
  busy=1;command(32'h123,1);
  repeat(4)clock_edge();if(req)$fatal(1,"issued while matcher busy");
  command(32'h4000,0);last_invalid=word;
  if(word!==0)$fatal(1,"invalid debug returned old word");
  // Exercise every old ACK alignment and keep stale ACK high for multiple clocks.
  for(integer offset=0;offset<8;offset++)begin
   ack=0;busy=1;command(32'h100+offset,1);
   repeat(offset)clock_edge();command(32'h80000000,0);
   q=18'h35555;ack=1;repeat(4)clock_edge();busy=0;repeat(3)clock_edge();
   if(!done||word!==0||req)$fatal(1,"old ACK overrode invalid zero");
  end
  // A new valid command cannot consume that lingering old ACK.
  busy=0;ack=1;command(32'h3fff,1);repeat(3)clock_edge();
  if(req||done)$fatal(1,"new command consumed stale ACK");
  ack=0;clock_edge();if(!req||ram_addr!==14'h3fff)$fatal(1,"latched address launch");
  dbg_addr=32'h4000; // source CSR changes after acceptance
  clock_edge();q=18'h12345;ack=1;clock_edge();
  if(req||!done||word!==18'h12345)$fatal(1,"valid query publication");
  ack=0;clock_edge();if(word!==18'h12345)$fatal(1,"publication hold");
  if(issued==0)$fatal(1,"no valid read observed");
  $display("TEST PASSED SLF FRONTEND checks=%0d issued_cycles=%0d highbits=18 staleACKalignments=8",checks,issued);$finish;
 end
 initial begin #100000;$fatal(1,"finite frontend stimulus watchdog");end
endmodule
