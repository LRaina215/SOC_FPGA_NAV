`ifndef RL_OBSERVATION_V5_V_INCLUDED
`define RL_OBSERVATION_V5_V_INCLUDED
`timescale 1ns/1ps
// model_4500: 28 current words and five frames, oldest first.
// IMU inputs are already body-frame physical rad/s and unit gravity, signed Q12.
// Encoder deltas are measured over SPEED_GATE_CYCLES, not the 8ms policy period.
// last_executed_actions must describe applied actions; shadow operation passes zero.
module rl_observation_v5 #(
    parameter integer ENCODER_CPR = 1320,
    parameter integer SPEED_GATE_CYCLES = 500000,
    parameter integer LEFT_ENCODER_SIGN = 1,
    parameter integer RIGHT_ENCODER_SIGN = 1
)(
    input wire clk, rst_n, capture,
    input wire [71:0] angular_velocity_q12, gravity_q12,
    input wire signed [15:0] left_delta, right_delta,
    input wire signed [23:0] linear_command_q12, yaw_command_q12, height_command_q12,
    input wire [143:0] last_executed_actions,
    output reg valid,
    output reg initialized,
    output reg [671:0] current_flat,
    output reg [3359:0] history_flat
);
    localparam signed [63:0] WHEEL_NUM = 64'sd193019452637;
    localparam signed [63:0] WHEEL_DEN = 64'sd1 * ENCODER_CPR * SPEED_GATE_CYCLES;
    reg signed [23:0] words [0:27];
    reg signed [63:0] numerators [0:27];
    reg [63:0] denominators [0:27];
    localparam IDLE=0, PREPARE=1, DIVIDE=2, SAVE=3, COMMIT=4;
    reg [2:0] state;
    reg [4:0] lane;
    reg [6:0] count;
    reg negative;
    reg [63:0] dividend,quotient,remainder,denominator;
    wire [64:0] trial={remainder,dividend[63]};
    wire subtract=trial>={1'b0,denominator};
    wire [64:0] next_remainder=subtract ? trial-{1'b0,denominator} : trial;
    wire [63:0] next_quotient={quotient[62:0],subtract};
    integer i,j;
    function signed [23:0] sat24;
        input signed [63:0] value;
        begin
            if(value>8388607) sat24=24'sh7fffff;
            else if(value < -8388608) sat24=24'sh800000;
            else sat24=value[23:0];
        end
    endfunction
    function signed [23:0] clip_current;
        input signed [23:0] value;
        begin
            if (value > 409600) clip_current = 24'sd409600;
            else if (value < -409600) clip_current = -24'sd409600;
            else clip_current = value;
        end
    endfunction
    // One shared 64-step divider, rather than ten parallel combinational dividers.
    // Inputs freeze on capture. Even all 28 lanes divided take less than 40us @50MHz.
    // valid reports completed current/history; capture while busy is ignored.
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            valid<=0;initialized<=0;current_flat<=0;history_flat<=0;
            state<=IDLE;lane<=0;count<=0;negative<=0;
            dividend<=0;quotient<=0;remainder<=0;denominator<=1;
        end else begin
            valid<=0;
            case(state)
                IDLE: if(capture) begin
                    for(i=0;i<28;i=i+1) begin numerators[i]<=0;denominators[i]<=1;end
                    for(i=0;i<3;i=i+1) begin
                        numerators[i]<=$signed(angular_velocity_q12[i*24+:24])*64'sd13;
                        denominators[i]<=10;
                        numerators[3+i]<=$signed(gravity_q12[i*24+:24])*64'sd3;
                    end
                    numerators[6]<=$signed(linear_command_q12)*64'sd2;
                    numerators[7]<=$signed(yaw_command_q12)*64'sd13;denominators[7]<=10;
                    numerators[8]<=$signed(height_command_q12)*64'sd5;
                    for(i=0;i<6;i=i+1) begin
                        if(i!=2 && i!=5) begin
                            numerators[10+i]<=$signed(last_executed_actions[i*24+:24])*64'sd3;
                            denominators[10+i]<=100;
                        end
                        numerators[22+i]<=$signed(last_executed_actions[i*24+:24]);
                    end
                    numerators[18]<=$signed(left_delta)*WHEEL_NUM*LEFT_ENCODER_SIGN;
                    numerators[21]<=$signed(right_delta)*WHEEL_NUM*RIGHT_ENCODER_SIGN;
                    denominators[18]<=WHEEL_DEN;denominators[21]<=WHEEL_DEN;
                    lane<=0;state<=PREPARE;
                end
                PREPARE: begin
                    if(denominators[lane]==1) begin
                        words[lane]<=sat24(numerators[lane]);
                        if(lane==27) state<=COMMIT;
                        else lane<=lane+1'b1;
                    end else begin
                        negative<=numerators[lane][63];
                        dividend<=(numerators[lane][63] ? -numerators[lane] : numerators[lane])
                                  +(denominators[lane]>>1);
                        denominator<=denominators[lane];
                        quotient<=0;remainder<=0;count<=64;state<=DIVIDE;
                    end
                end
                DIVIDE: begin
                    remainder<=next_remainder[63:0];quotient<=next_quotient;
                    dividend<={dividend[62:0],1'b0};count<=count-1'b1;
                    if(count==1) state<=SAVE;
                end
                SAVE: begin
                    words[lane]<=sat24(negative ? -$signed(quotient) : $signed(quotient));
                    if(lane==27) state<=COMMIT;
                    else begin lane<=lane+1'b1;state<=PREPARE;end
                end
                COMMIT: begin
                    valid<=1;initialized<=1;state<=IDLE;
                    for(j=0;j<28;j=j+1) current_flat[j*24+:24]<=clip_current(words[j]);
                    if(!initialized) begin
                        for(j=0;j<140;j=j+1) history_flat[j*24+:24]<=words[j%28];
                    end else begin
                        history_flat[2687:0]<=history_flat[3359:672];
                        for(j=0;j<28;j=j+1) history_flat[(112+j)*24+:24]<=words[j];
                    end
                end
                default: state<=IDLE;
            endcase
        end
    end
endmodule

`endif
