// Health is measured by the original independent core clock. A CPU warm
// reset is intentionally not a reset of the maps or of this sticky latch.
// cold_maps_ready must mean BOTH full maps have completed initialization;
// this module does not synthesize that fact from PLL/calibration status.
module nav_ddr_admission(
    input wire core_clk,ui_clk,cold_rst,pll_locked,calib_done,
    input wire cold_maps_ready,store_fault,
    output wire admitted,stop_request,output reg fault
);
    wire health_ready,health_fault,cr;
    nav_ddr_reset u_reset(core_clk,cold_rst,cr);
    nav_ddr_health u_health(core_clk,ui_clk,cold_rst,pll_locked,calib_done,
        health_ready,health_fault);
    always@(posedge core_clk or posedge cr)
        if(cr)fault<=0;
        else if(store_fault||health_fault)fault<=1;
    assign admitted=cold_maps_ready&&health_ready&&!fault&&!store_fault;
    assign stop_request=!admitted;
endmodule
