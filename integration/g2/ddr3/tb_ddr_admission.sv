`timescale 1ns/1ps
module tb_ddr_admission;
    reg clk=0,ui=0,run_ui=1,cold=1,rst=1,lock=0,cal=0,maps=0,store_fault=0;
    always #10 clk=~clk;
    always #5 if(run_ui)ui=~ui;
    wire admitted,stop_request,fault;
    nav_ddr_admission guard(clk,ui,cold,lock,cal,maps,store_fault,admitted,stop_request,fault);
    reg cfg_estop=0,cmd_we=0;
    wire signed[15:0]vx,vy,wz;wire valid,stopped;
    integer checks=0;reg[31:0]time_us=0;
    always@(posedge clk)time_us<=time_us+1;
    // Exact original safety_guard. DDR stop joins its unmaskable ESTOP
    // input; the existing external stop mask cannot suppress memory failure.
    safety_guard safety(.clk(clk),.rst(rst),.time_us(time_us),.tick_us(1'b1),
        .cfg_enable(1'b1),.cfg_holonomic(1'b0),.cfg_estop(cfg_estop|stop_request),
        .cfg_vx_max(16'd1000),.cfg_vy_max(16'd1000),.cfg_wz_max(16'd1000),
        .cfg_dv(16'd1000),.cfg_dw(16'd1000),.cfg_tick_us(32'd4),.cfg_wd_us(32'd1000000),
        .cfg_ext_mask(4'd0),.ext_stop(4'd0),.cmd_we(cmd_we),.cmd_vx(16'sd300),
        .cmd_vy(16'sd0),.cmd_wz(16'sd100),.cmd_mode(8'd1),
        .out_vx(vx),.out_vy(vy),.out_wz(wz),.out_mode(),.out_stop(stopped),
        .out_wdog(),.out_valid(valid),.out_seq(),.ext_active());
    task automatic command;
        @(negedge clk);cmd_we=1;@(negedge clk);cmd_we=0;
    endtask
    task automatic expect_stop;
        repeat(10)@(negedge clk);
        if(vx!==0||vy!==0||wz!==0||!stopped||admitted)
            $fatal(1,"unmaskable DDR stop failed");
        checks++;
    endtask
    task automatic reboot;
        run_ui=1;cold=1;rst=1;lock=0;cal=0;maps=0;store_fault=0;
        repeat(12)@(negedge clk);cold=0;rst=0;
        command();expect_stop();
        lock=1;cal=1;repeat(80)@(negedge clk);expect_stop();
        maps=1;repeat(20)@(negedge clk);
        if(!admitted||fault||vx!==300||wz!==100)$fatal(1,"healthy memory blocked");
        checks++;
    endtask
    initial begin
        reboot();
        // CPU restart does not touch the guard's cold-only reset. Original
        // safety reset is exercised independently while a fault is latched.
        run_ui=0;repeat(525)@(negedge clk);expect_stop();
        if(!fault)$fatal(1,"UI-stop fault not latched");
        run_ui=1;rst=1;repeat(8)@(negedge clk);rst=0;command();
        repeat(80)@(negedge clk);expect_stop();
        if(!fault)$fatal(1,"warm safety reset cleared memory fault");
        reboot();lock=0;repeat(8)@(negedge clk);expect_stop();
        lock=1;repeat(80)@(negedge clk);expect_stop();
        reboot();store_fault=1;@(negedge clk);store_fault=0;
        repeat(15)@(negedge clk);expect_stop();
        cfg_estop=1;repeat(8)@(negedge clk);cfg_estop=0;expect_stop();
        reboot();cal=0;repeat(8)@(negedge clk);expect_stop();
        cal=1;repeat(80)@(negedge clk);expect_stop();
        $display("TEST PASSED checks=%0d original_safety_zero_verified=1 fault_survives_warm_reset=1",checks);$finish;
    end
    initial begin #1000000;$fatal(1,"bounded functional test budget");end
endmodule
