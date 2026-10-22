// Compare SAFE transaction model with RTL. Tick period is compressed to one
// strobe; time_us advances 1000us per strobe. Three settle clocks preserve the
// target/clamp pipeline; the model intentionally omits these sub-us latencies.
`timescale 1ns/1ps
module tb_safe_reference;
    reg clk=0,rst=1;always #5 clk=~clk;
    reg [31:0] time_us=0;reg tick=0,enable=1,estop=0,submit=0;
    reg signed [15:0] vx=0,vy=0,wz=0;
    wire signed [15:0] ox,oy,ow;wire valid,stop,wd;
    reg [31:0] vectors[0:11999];integer n,fd,errs=0;
    safety_guard dut(.clk(clk),.rst(rst),.time_us(time_us),.tick_us(tick),
      .cfg_enable(enable),.cfg_holonomic(1'b0),.cfg_estop(estop),
      .cfg_vx_max(16'd500),.cfg_vy_max(16'd0),.cfg_wz_max(16'd1200),
      .cfg_dv(16'd2),.cfg_dw(16'd3),.cfg_tick_us(32'd1),.cfg_wd_us(32'd200000),
      .cfg_ext_mask(4'd0),.ext_stop(4'd0),.cmd_we(submit),.cmd_vx(vx),.cmd_vy(vy),
      .cmd_wz(wz),.cmd_mode(8'd0),.out_vx(ox),.out_vy(oy),.out_wz(ow),
      .out_mode(),.out_stop(stop),.out_wdog(wd),.out_valid(valid),.out_seq(),.ext_active());
    initial begin
      fd=$fopen("counts.txt","r");void'($fscanf(fd,"%d",n));$fclose(fd);
      $readmemh("vectors.hex",vectors);
      repeat(5) @(negedge clk);rst=0;
      for(integer i=0;i<n;i=i+1) begin
        @(negedge clk);time_us=vectors[i*10];submit=vectors[i*10+1];
        vx=vectors[i*10+2];vy=vectors[i*10+3];wz=vectors[i*10+4];
        enable=vectors[i*10+5];estop=vectors[i*10+6];
        @(negedge clk);submit=0;
        repeat(3) @(negedge clk);tick=1;
        @(negedge clk);tick=0;
        if(!valid || ox!==vectors[i*10+7][15:0] || ow!==vectors[i*10+8][15:0] ||
           oy!==16'd0 || wd!==vectors[i*10+9][0] || stop!==estop) begin
          errs=errs+1;
          if(errs<8)$display("SAFE vector %0d expected %0d/%0d got %0d/%0d wd%0d",i,
            $signed(vectors[i*10+7]),$signed(vectors[i*10+8]),ox,ow,wd);
        end
      end
      if(errs==0)$display("TEST PASSED: %0d SAFE vectors",n);
      else $display("TEST FAILED errs=%0d",errs);
      $finish;
    end
endmodule
