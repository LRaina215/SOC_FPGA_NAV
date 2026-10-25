// FPGA configuration starts the power epoch. Ordinary board reset restarts
// the DDR protocol and NAV together, while a known fault survives that reset.
// PicoRV32 debug HOLD is deliberately not connected here.
module nav_ddr_board_reset #(
    parameter integer STARTUP_CYCLES=10000
)(
    input wire clk,board_reset,memory_fault,
    output wire power_reset,protocol_reset,
    output reg sticky_fault=0
);
    localparam W=$clog2(STARTUP_CYCLES+1);
    reg[W-1:0]power_count=0,protocol_count=0;
    reg power_ready=0,protocol_ready=0;
    always@(posedge clk)begin
        if(!power_ready)begin
            sticky_fault<=0;
            if(power_count==STARTUP_CYCLES-1)power_ready<=1;
            else power_count<=power_count+1'b1;
        end else if(memory_fault)sticky_fault<=1;
        if(board_reset)begin protocol_count<=0;protocol_ready<=0;end
        else if(!protocol_ready)begin
            if(protocol_count==STARTUP_CYCLES-1)protocol_ready<=1;
            else protocol_count<=protocol_count+1'b1;
        end
    end
    assign power_reset=!power_ready;
    assign protocol_reset=board_reset||!protocol_ready;
endmodule
