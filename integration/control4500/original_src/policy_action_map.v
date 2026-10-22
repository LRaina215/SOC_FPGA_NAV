// model_4500: action order LH,LK,LWheel,RH,RK,RWheel; signed Q12.
// IK ROM already contains calibrated signs. Do NOT apply servo signs again.
module policy_action_map #(
    parameter integer LH_ZERO_US=1500,LK_ZERO_US=1500,RH_ZERO_US=1500,RK_ZERO_US=1500
)(
    input wire clk,rst_n,run,action_valid,
    input wire [143:0] actions,
    input wire signed [11:0] ff_lh,ff_lk,ff_rh,ff_rk,
    output reg signed [31:0] left_mmps,right_mmps,
    output reg [15:0] lh_us,lk_us,rh_us,rk_us,
    output reg [143:0] last_actions,
    output reg have_action
);
    function signed [23:0] clip;
        input signed [23:0] a;
        begin clip=a>4096 ? 24'sd4096 : a < -4096 ? -24'sd4096 : a; end
    endfunction
    function signed [31:0] scale;
        input signed [23:0] a;
        input integer gain;
        reg signed [63:0] product;
        begin
            product=$signed(clip(a))*gain;
            scale=product<0 ? -((-product+2048)>>>12) : ((product+2048)>>>12);
        end
    endfunction
    function [15:0] pulse;
        input integer zero_us,delta;
        input signed [11:0] ff;
        integer v;
        begin v=zero_us+$signed(ff)+delta; pulse=v<500 ? 500 : v>2500 ? 2500 : v; end
    endfunction
    integer i;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            left_mmps<=0;right_mmps<=0;last_actions<=0;have_action<=0;
            lh_us<=LH_ZERO_US;lk_us<=LK_ZERO_US;rh_us<=RH_ZERO_US;rk_us<=RK_ZERO_US;
        end else if(!run) begin
            left_mmps<=0;right_mmps<=0;last_actions<=0;have_action<=0;
            // Hold the last physical leg pose on loss of control.
        end else if(action_valid) begin
            left_mmps<=scale(actions[48+:24],-600);
            right_mmps<=scale(actions[120+:24],600);
            lh_us<=pulse(LH_ZERO_US,scale(actions[0+:24],19),ff_lh);
            lk_us<=pulse(LK_ZERO_US,scale(actions[24+:24],-19),ff_lk);
            rh_us<=pulse(RH_ZERO_US,scale(actions[72+:24],19),ff_rh);
            rk_us<=pulse(RK_ZERO_US,scale(actions[96+:24],19),ff_rk);
            for(i=0;i<6;i=i+1) last_actions[i*24+:24]<=clip(actions[i*24+:24]);
            have_action<=1;
        end
    end
endmodule
