`timescale 1ns/1ps
// Directed composition check only. No simultaneous RAM read/write is exercised.
module stage3_ram_case #(parameter integer ID=0,DW=1,AW=14,BLOCK=1,
 parameter INIT_FILE="",parameter [DW-1:0] IV=0)(output reg done=0);
 localparam integer DEPTH=1<<AW;
 reg clk=0,we=0,re=0;reg[AW-1:0]wa=0,ra=0;reg[DW-1:0]wd=0;
 wire[DW-1:0]q;reg[DW-1:0]expected[0:DEPTH-1],held;
 integer checks=0,a;
 always #10 if(!done)clk=~clk;
 nav_ram_sdp #(.DW(DW),.AW(AW),.STAGE2B_BLOCK(BLOCK),.INIT_FILE(INIT_FILE),.INIT_VAL(IV)) dut
  (.clk(clk),.we(we),.waddr(wa),.wdata(wd),.re(re),.raddr(ra),.rdata(q));
 function integer addr(input integer n);
  case(n)
   0:addr=0;1:addr=1;2:addr=DEPTH-1;3:addr=DEPTH/2-1;
   4:addr=DEPTH/2;5:addr=511%DEPTH;6:addr=512%DEPTH;
   7:addr=1023%DEPTH;8:addr=1024%DEPTH;9:addr=2047%DEPTH;
   10:addr=2048%DEPTH;default:addr=(n*769+31)%DEPTH;
  endcase
 endfunction
 task read_check(input integer address);
  begin
   @(negedge clk);we=0;re=1;ra=address;
   @(posedge clk);#1;checks++;
   if(q!==expected[address])$fatal(1,"RAM case%0d addr%0d q%h expected%h",ID,address,q,expected[address]);
   held=q;@(negedge clk);re=0;ra=~ra;
   repeat(2)begin @(posedge clk);#1;checks++;if(q!==held)$fatal(1,"RAM hold case%0d",ID);end
  end
 endtask
 initial begin
  for(integer i=0;i<DEPTH;i++)expected[i]=IV;
  if(INIT_FILE!="")$readmemh(INIT_FILE,expected);
  #1;if(q!==0)$fatal(1,"RAM initial q case%0d",ID);
  for(integer n=0;n<16;n++)read_check(addr(n));
  // Write and read phases are separated by an actual idle edge.
  for(integer n=0;n<16;n++)begin
   a=addr(n);@(negedge clk);re=0;we=1;wa=a;
   wd=64'hd6ac953b2710e84f^(64'h1937469bd0*n)^a;expected[a]=wd;
   @(posedge clk);#1;
  end
  @(negedge clk);we=0;@(posedge clk);#1;
  for(integer n=0;n<16;n++)read_check(addr(n));
  $display("STAGE3_RAM_BRANCH_PASS id=%0d DW=%0d AW=%0d block=%0d checks=%0d",ID,DW,AW,BLOCK,checks);
  done=1;
 end
endmodule

module stage3_storage_branches(output wire done);
 wire[8:0]ok;
 stage3_ram_case #(.ID(0),.DW(36),.AW(12),.BLOCK(0),.IV(36'h812345678))p0(ok[0]);
 stage3_ram_case #(.ID(1),.DW(36),.AW(12),.BLOCK(1),.IV(36'h123456789))p1(ok[1]);
 stage3_ram_case #(.ID(2),.DW(1),.AW(14),.BLOCK(1),.IV(1))touch_bitmap(ok[2]);
 stage3_ram_case #(.ID(3),.DW(14),.AW(10),.BLOCK(1),.IV(14'h31a5))slist(ok[3]);
 stage3_ram_case #(.ID(4),.DW(16),.AW(10),.BLOCK(1),.INIT_FILE("sin_lut.hex"))trig(ok[4]);
 stage3_ram_case #(.ID(5),.DW(8),.AW(10),.BLOCK(1),.IV(8'hb6))console(ok[5]);
 stage3_ram_case #(.ID(6),.DW(32),.AW(9),.BLOCK(1),.IV(32'h89abcdef))txshadow(ok[6]);
 stage3_ram_case #(.ID(7),.DW(18),.AW(14),.BLOCK(0),.IV(18'h1935a))banked_structmap(ok[7]);
 stage3_ram_case #(.ID(8),.DW(32),.AW(9),.BLOCK(0),.IV(32'hdeadbeef))inj_distributed(ok[8]);
 assign done=&ok;
endmodule
