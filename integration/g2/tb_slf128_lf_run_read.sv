`timescale 1ns/1ps
module tb_slf128_lf_run_read;
  reg clk=0;always #10 clk=~clk;
  reg rst=1,start=0;
  reg signed[31:0]cx=0,cy=0;
  wire[1:0]busy,done,re,we;
  wire[13:0]ra[0:1],wa[0:1];wire[17:0]wd[0:1];
  reg[17:0]q[0:1];reg[17:0]mem0[0:16383],mem1[0:16383];
  integer checks=0,writes=0,completed=0,resets=0,frame_writes=0;
  slf_lf_old #(.LOG2N(7))a(clk,rst,start,cx,cy,busy[0],done[0],re[0],ra[0],q[0],we[0],wa[0],wd[0]);
  slf_lf_new #(.LOG2N(7))b(clk,rst,start,cx,cy,busy[1],done[1],re[1],ra[1],q[1],we[1],wa[1],wd[1]);
  always@(posedge clk)begin
    if(re[0])q[0]<=mem0[ra[0]];
    if(re[1])q[1]<=mem1[ra[1]];
    if(we[0])mem0[wa[0]]<=wd[0];
    if(we[1])mem1[wa[1]]<=wd[1];
  end
  always@(negedge clk)begin
    #1;
    checks++;
    if($isunknown({busy,done,re,we}))$fatal(1,"unknown public control at check%0d",checks);
    if(busy[0]!==busy[1]||done[0]!==done[1]||re[0]!==re[1]||we[0]!==we[1])$fatal(1,"public event mismatch check%0d",checks);
    if(re[0]&&($isunknown({ra[0],ra[1]})||ra[0]!==ra[1]))$fatal(1,"read address mismatch check%0d",checks);
    if(we[0])begin
      if($isunknown({wa[0],wa[1],wd[0],wd[1]})||wa[0]!==wa[1]||wd[0]!==wd[1])$fatal(1,"valid write mismatch/X check%0d old=%h new=%h wa=%0d/%0d",checks,wd[0],wd[1],wa[0],wa[1]);
      writes++;frame_writes++;
    end
  end
  task reset_now;
    begin @(negedge clk);#2;rst=1;start=0;resets++;
      repeat(3)@(negedge clk);#2;rst=0;repeat(3)@(negedge clk);
    end
  endtask
  task fill(input integer mode);
    reg[31:0]v;reg[17:0]d;
    begin
      v=32'h185a77a1^mode;
      for(integer i=0;i<16384;i++)begin
        v=v^(v<<13);v=v^(v>>17);v=v^(v<<5);
        if(mode==0)d=0;
        else if(mode==1)d={2'b00,2'b11,v[5:0],8'h80};
        else if(mode==2)d=v[17:0];
        else if(mode==3)d={2'b11,2'b11,v[5:0],8'h80};
        else d={i[13:12],i[4:3],i[8:3],i[7:0]};
        mem0[i]=d;mem1[i]=d;
      end
    end
  endtask
  task launch(input integer x,y);
    begin @(negedge clk);#2;cx=x;cy=y;start=1;frame_writes=0;
      @(negedge clk);#2;start=0;
    end
  endtask
  task finish_frame;
    begin wait(done[0]);@(negedge clk);#2;
      if(frame_writes!=16384)$fatal(1,"incomplete LF output%0d",frame_writes);
      for(integer i=0;i<16384;i++)if(mem0[i]!==mem1[i])$fatal(1,"map mismatch slot%0d",i);
      completed++;$display("LF_MITER complete%0d writes%0d",completed,frame_writes);repeat(4)@(negedge clk);
    end
  endtask
  initial begin
    repeat(4)@(negedge clk);#2;rst=0;repeat(4)@(negedge clk);
    for(integer k=0;k<5;k++)begin fill(k);launch((k-2)*129,257-k*128);finish_frame();end
    // Interrupt first column, row boundary, middle scan, final scan and flush.
    // Array contents remain untouched across reset; restart must mask all old
    // private prefetch/line data until their normal writes establish validity.
    for(integer p=0;p<5;p++)begin
      fill(2);launch(-129,127);
      case(p)
        0:repeat(1)@(negedge clk);
        1:repeat(130)@(negedge clk);
        2:repeat(8401)@(negedge clk);
        3:repeat(16899)@(negedge clk);
        4:repeat(16903)@(negedge clk);
      endcase
      reset_now();launch(128+p,-129-p);finish_frame();
    end
    $display("TEST PASSED LF128_RUN_READ checks=%0d valid_writes=%0d full_maps=%0d resets=%0d natural_private_X=1",checks,writes,completed,resets);$finish;
  end
  initial begin #10000000;$fatal(1,"finite LF128 miter watchdog");end
endmodule
