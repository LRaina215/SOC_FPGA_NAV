`timescale 1ns/1ps
module tb_ddr_health;
    reg clk=0,ui=0,run_ui=1,cold=1,lock=0,cal=0;
    always #10 clk=~clk;
    always #5 if(run_ui)ui=~ui;
    wire ready,fault;
    nav_ddr_health dut(clk,ui,cold,lock,cal,ready,fault);
    integer checks=0;
    task automatic reboot;
        begin
            run_ui=1;cold=1;lock=0;cal=0;
            repeat(12)@(negedge clk);cold=0;
            repeat(600)@(negedge clk);
            if(fault||ready)$fatal(1,"untrained device falsely admitted/faulted");
            lock=1;repeat(20)@(negedge clk);cal=1;
            repeat(40)@(negedge clk);
            if(fault||!ready)$fatal(1,"healthy clocks not admitted");checks++;
        end
    endtask
    task automatic check_latched;
        begin
            repeat(600)@(negedge clk);
            if(!fault||ready)$fatal(1,"fault recovered without cold reset");checks++;
        end
    endtask
    initial begin
        reboot();
        repeat(4000)@(negedge clk);
        if(!ready||fault)$fatal(1,"healthy heartbeat false timeout");checks++;
        // Stop UI but deliberately retain the PLL/calibration status high.
        run_ui=0;
        repeat(520)@(negedge clk);
        if(!fault||ready)$fatal(1,"stopped UI clock missed");checks++;
        run_ui=1;check_latched();
        reboot();cal=0;repeat(5)@(negedge clk);
        if(!fault||ready)$fatal(1,"calibration loss missed");checks++;
        cal=1;check_latched();
        reboot();run_ui=0;lock=0;repeat(5)@(negedge clk);
        if(!fault||ready)$fatal(1,"PLL loss with stopped UI missed");checks++;
        lock=1;run_ui=1;check_latched();
        reboot();
        $display("TEST PASSED checks=%0d ui_stop_detect_core_cycles_max=520",checks);$finish;
    end
    initial begin #1000000;$fatal(1,"bounded test budget");end
endmodule
