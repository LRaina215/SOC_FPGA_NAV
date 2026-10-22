`timescale 1ns/1ps
module tb_fixed_forward;
    reg clk=0,reset_n=0;
    always #10 clk=~clk;
    main #(.FIXED_FORWARD_ENABLE(1),.COMMAND_FORWARD_MPS(-0.05)) dut(.clk(clk),.reset_n(reset_n),.UART_RXD(1'b1),.IRIN(1'b1),
        .M1_ENCA(1'b0),.M1_ENCB(1'b0),.M2_ENCA(1'b0),.M2_ENCB(1'b0));
    initial begin
        #2;
        force dut.ir_emergency_stop=1'b0;
        #1;
        if($signed(dut.command_forward_q12)!==-205) $fatal(1,"Not -0.05m/s at reset");
        reset_n=1;
        force dut.ir_forward_q12=24'sd1229;
        force dut.ir_yaw_q12=24'sd4096;
        #2;
        if($signed(dut.command_forward_q12)!==-205) $fatal(1,"Forward key changed fixed velocity");
        if($signed(dut.command_yaw_q12)!==4096) $fatal(1,"Left turn unavailable");
        force dut.ir_forward_q12=-24'sd1229;
        force dut.ir_yaw_q12=-24'sd4096;
        #2;
        if($signed(dut.command_forward_q12)!==-205) $fatal(1,"Reverse key changed fixed velocity");
        if($signed(dut.command_yaw_q12)!==-4096) $fatal(1,"Right turn unavailable");
        force dut.ir_forward_q12=24'sd0;
        force dut.ir_yaw_q12=24'sd0;
        #2;
        if($signed(dut.command_forward_q12)!==-205) $fatal(1,"Release cleared fixed velocity");
        $display("FIXED_FORWARD_PASS reset/forward/reverse/release=-205 Q12; yaw preserved");
        $finish;
    end
endmodule
