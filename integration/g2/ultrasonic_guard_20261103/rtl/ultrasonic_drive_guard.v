// Minimal manual-only distance control. No added timeout, fault latch or rearm.
// Normal remote forward remains1229Q12. Navigation and the existing stop bypass it.
module ultrasonic_drive_guard #(
    parameter integer CLOCK_HZ=50000000,
    parameter [7:0] MODE_KEY=8'h16
)(
    input wire clk,reset_n,
    input wire [7:0] remote_key,
    input wire remote_key_event,remote_key_active,manual_owner,hard_stop,
    input wire signed [23:0] requested_forward_q12,requested_yaw_q12,
    input wire sample_strobe,sample_valid,
    input wire [15:0] distance_mm,
    input wire feedback_valid,
    input wire signed [15:0] feedback_forward_mmps,
    output reg enabled,
    output wire signed [23:0] forward_q12,yaw_q12
);
    function integer width_for;
        input integer value;integer n;
        begin width_for=0;for(n=value-1;n>0;n=n>>1)width_for=width_for+1;
              if(width_for<1)width_for=1;end
    endfunction
    localparam integer MS_CYCLES=CLOCK_HZ/1000;
    reg [width_for(MS_CYCLES)-1:0] ms_count;
    wire ms_tick=ms_count==MS_CYCLES-1;
    reg mode_key_held,near_obstacle;
    reg [10:0] speed_q12,range_cap_q12;
    wire active=enabled && manual_owner && requested_forward_q12>0;
    wire parking_active=active && near_obstacle;
    wire [10:0] desired_q12=requested_forward_q12<$signed({13'd0,range_cap_q12}) ?
        requested_forward_q12[10:0] : range_cap_q12;
    function [18:0] distance_product;
        input [8:0] d;
        begin distance_product=({10'd0,d}<<10)+({10'd0,d}<<7)+({10'd0,d}<<6)+
            ({10'd0,d}<<3)+({10'd0,d}<<2)+{10'd0,d};end
    endfunction
    reg cap_busy;
    reg [4:0] cap_bit;
    reg [18:0] cap_dividend;
    reg [10:0] cap_quotient;
    reg [8:0] cap_remainder;
    wire [9:0] cap_trial={cap_remainder,cap_dividend[18]};
    wire cap_take=cap_trial>=350;
    wire [10:0] cap_next_q={cap_quotient[9:0],cap_take};

    //Parking PI:Kp0.5,Ki0.5/s,100Hz encoder feedback; same equation as MuJoCo.
    reg signed [14:0] parking_sum;
    reg signed [22:0] parking_error;
    reg signed [8:0] parking_q12;
    reg parking_negative;
    reg [1:0] parking_phase;
    reg [4:0] parking_bit;
    reg [23:0] parking_dividend;
    reg [7:0] parking_quotient;
    reg [15:0] parking_remainder;
    wire signed [16:0] sum_trial={{2{parking_sum[14]}},parking_sum}+{feedback_forward_mmps[15],feedback_forward_mmps};
    wire signed [16:0] sum_limited=sum_trial>10000 ? 17'sd10000 : sum_trial < -10000 ? -17'sd10000 : sum_trial;
    wire signed [22:0] feedback_extended={{7{feedback_forward_mmps[15]}},feedback_forward_mmps};
    wire signed [22:0] feedback100=(feedback_extended<<<6)+(feedback_extended<<<5)+(feedback_extended<<<2);
    wire [22:0] parking_magnitude=parking_error<0 ? -parking_error : parking_error;
    wire [16:0] parking_trial={parking_remainder,parking_dividend[23]};
    wire parking_take=parking_trial>=50000;
    wire [7:0] parking_next_q={parking_quotient[6:0],parking_take};
    wire signed [23:0] limited_forward={13'd0,speed_q12};
    wire signed [23:0] parking_forward={{15{parking_q12[8]}},parking_q12};
    assign forward_q12=hard_stop ? 24'sd0 : !active ? requested_forward_q12 :
        near_obstacle ? parking_forward : requested_forward_q12<limited_forward ? requested_forward_q12 : limited_forward;
    assign yaw_q12=hard_stop ? 24'sd0 : parking_active ? 24'sd0 : requested_yaw_q12;
    always @(posedge clk or negedge reset_n)begin
        if(!reset_n)begin enabled<=0;mode_key_held<=0;end
        else begin
            if(!remote_key_active || remote_key!=MODE_KEY)mode_key_held<=0;
            if(remote_key_event && remote_key==MODE_KEY && !mode_key_held)begin enabled<=!enabled;mode_key_held<=1;end
        end
    end
    //Datapath registers use power-up values only; enabled is reset, so the guard is bypassed after reset.
    initial begin
        ms_count=0;near_obstacle=0;speed_q12=1229;range_cap_q12=1229;
        cap_busy=0;cap_bit=0;cap_dividend=0;cap_quotient=0;cap_remainder=0;
        parking_q12=0;parking_sum=0;parking_error=0;parking_negative=0;
        parking_phase=0;parking_bit=0;parking_dividend=0;parking_quotient=0;parking_remainder=0;
    end
    always @(posedge clk)begin
        begin
            ms_count<=ms_tick ? 0 : ms_count+1'b1;
            if(cap_busy)begin
                cap_dividend<={cap_dividend[17:0],1'b0};cap_remainder<=cap_take ? cap_trial-350 : cap_trial;
                cap_quotient<=cap_next_q;
                if(cap_bit==0)begin range_cap_q12<=cap_next_q;cap_busy<=0;end
                else cap_bit<=cap_bit-1'b1;
            end
            //Only completed valid distances update the target. No added stale/fault policy.
            if(sample_strobe && sample_valid)begin
                near_obstacle<=distance_mm<=250;
                if(distance_mm<=250)begin range_cap_q12<=0;cap_busy<=0;end
                else if(distance_mm>=600)begin range_cap_q12<=1229;cap_busy<=0;end
                else begin cap_dividend<=distance_product(distance_mm-250);
                    cap_remainder<=0;cap_quotient<=0;cap_bit<=18;cap_busy<=1;end
            end
            if(!active) speed_q12<=1229;
            else if(near_obstacle) speed_q12<=0;
            else if(ms_tick)begin
                if(speed_q12<desired_q12)speed_q12<=desired_q12-speed_q12<=2 ? desired_q12 : speed_q12+2;
                else if(speed_q12>desired_q12)speed_q12<=speed_q12-desired_q12<=4 ? desired_q12 : speed_q12-4;
            end
        end
    end
    always @(posedge clk)begin
        if(!parking_active || hard_stop)begin parking_q12<=0;parking_sum<=0;parking_error<=0;
            parking_negative<=0;parking_phase<=0;parking_bit<=0;parking_dividend<=0;parking_quotient<=0;parking_remainder<=0;end
        else case(parking_phase)
            0:if(feedback_valid)begin parking_sum<=sum_limited[14:0];
                parking_error<=-(feedback100+{{6{sum_limited[16]}},sum_limited});parking_phase<=1;end
            1:begin
                parking_negative<=parking_error<0;
                if(parking_magnitude>=9986)begin parking_q12<=parking_error<0 ? -9'sd205 : 9'sd205;parking_phase<=0;end
                else begin parking_dividend<=({10'd0,parking_magnitude[13:0]}<<10)+24'd25000;
                    parking_remainder<=0;parking_quotient<=0;parking_bit<=23;parking_phase<=2;end
            end
            2:begin
                parking_dividend<={parking_dividend[22:0],1'b0};parking_remainder<=parking_take ? parking_trial-50000 : parking_trial;
                parking_quotient<=parking_next_q;
                if(parking_bit==0)begin parking_q12<=parking_negative ? -$signed({1'b0,parking_next_q}) : $signed({1'b0,parking_next_q});parking_phase<=0;end
                else parking_bit<=parking_bit-1'b1;
            end
        endcase
    end
endmodule
