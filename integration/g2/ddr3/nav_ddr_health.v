// Observe DDR health on the independent ORIGINAL core clock. No CPU/key warm
// reset input: an admitted memory failure stays latched until cold reset.
// A running PLL flag alone cannot prove that the UI clock is still toggling.
module nav_ddr_health #(
    parameter integer HEARTBEAT_DIV=16,
    parameter integer MAX_SILENT_CORE_CYCLES=512
)(
    input wire core_clk,ui_clk,cold_rst,pll_locked,calib_done,
    output wire ready,output reg fault
);
    localparam HB_W=$clog2(HEARTBEAT_DIV), AGE_W=$clog2(MAX_SILENT_CORE_CYCLES+1);
    wire cr,ur;
    nav_ddr_reset u_cr(core_clk,cold_rst,cr);
    nav_ddr_reset u_ur(ui_clk,cold_rst,ur);
    reg[HB_W-1:0]hb_count;reg hb;
    (* syn_keep=1 *)reg hb1,hb2,lock1,lock2,cal1,cal2;
    reg seen,last_hb;reg[AGE_W-1:0]age;
    always@(posedge ui_clk or posedge ur)
        if(ur)begin hb_count<=0;hb<=0;end
        else if(hb_count==HEARTBEAT_DIV-1)begin hb_count<=0;hb<=~hb;end
        else hb_count<=hb_count+1'b1;
    always@(posedge core_clk or posedge cr)begin
        if(cr)begin hb1<=0;hb2<=0;lock1<=0;lock2<=0;cal1<=0;cal2<=0;
            seen<=0;last_hb<=0;age<=0;fault<=0;end
        else begin
            hb1<=hb;hb2<=hb1;
            lock1<=pll_locked;lock2<=lock1;cal1<=calib_done;cal2<=cal1;
            last_hb<=hb2;
            if(hb2!=last_hb)begin
                age<=0;
                if(lock2&&cal2)seen<=1;
            end else if(age<MAX_SILENT_CORE_CYCLES)age<=age+1'b1;
            if(seen&&(!lock2||!cal2||age>=MAX_SILENT_CORE_CYCLES))fault<=1;
        end
    end
    assign ready=seen&&lock2&&cal2&&!fault;
endmodule
