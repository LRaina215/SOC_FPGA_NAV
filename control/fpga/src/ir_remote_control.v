// Adapted from competition/src/ir_sensor.v NEC falling-edge decoder.
// Removed 74HC595 display scan; added reset and policy-command outputs.
// Reference keys: 2=18 forward,8=52 backward,4=08 left,6=5A right.
module ir_remote_control #(
    parameter integer CLOCK_FREQ_HZ=50_000_000,
    parameter integer KEY_HOLD_MS=180,
    parameter integer HEIGHT_INITIAL_Q12=535,
    parameter integer HEIGHT_STEP_MS=10
)(
    input wire clk,reset_n,ir_in,
    output reg signed [23:0] forward_q12,yaw_q12,
    output reg signed [23:0] height_q12,
    output reg emergency_stop,
    output reg [7:0] key_code,
    output wire key_active,
    output reg key_event,repeat_event
);
    // Fixed limits: no remote key increases these values. Wheel action mapper
    // separately limits physical wheel targets to +/-600mm/s as before.
    localparam signed [23:0] FORWARD_Q12=24'sd1229; // nearest Q12 to0.3m/s
    localparam signed [23:0] TURN_Q12=24'sd4096;    // 1rad/s; +left/-right
    localparam integer LEADER_MIN=CLOCK_FREQ_HZ/1000*12;
    localparam integer LEADER_MAX=CLOCK_FREQ_HZ/1000*15;
    localparam integer REPEAT_MIN=CLOCK_FREQ_HZ/1000*10;
    localparam integer REPEAT_MAX=LEADER_MIN-1;
    localparam integer BIT0_MIN=CLOCK_FREQ_HZ/10000*8;
    localparam integer BIT0_MAX=CLOCK_FREQ_HZ/10000*14;
    localparam integer BIT1_MIN=CLOCK_FREQ_HZ/10000*18;
    localparam integer BIT1_MAX=CLOCK_FREQ_HZ/10000*27;
    localparam integer KEY_HOLD_COUNT=CLOCK_FREQ_HZ/1000*KEY_HOLD_MS;
    localparam integer HEIGHT_STEP_COUNT=CLOCK_FREQ_HZ/1000*HEIGHT_STEP_MS;
    localparam signed [23:0] HEIGHT_MIN_Q12=24'sd348,HEIGHT_MAX_Q12=24'sd778;
    reg [31:0] height_count;
    reg ir_sync_0,ir_sync_1,ir_last;
    wire falling=ir_last && !ir_sync_1;
    reg [31:0] edge_count,frame_bits,hold_count;
    reg [5:0] bit_count;
    reg receiving;
    wire bit_zero=edge_count>=BIT0_MIN && edge_count<=BIT0_MAX;
    wire bit_one=edge_count>=BIT1_MIN && edge_count<=BIT1_MAX;
    wire [31:0] completed_frame={bit_one,frame_bits[30:0]};
    assign key_active=hold_count!=0;
    always @* begin
        forward_q12=0;yaw_q12=0;
        if(key_active && !emergency_stop) begin
            case(key_code)
                8'h18:forward_q12=FORWARD_Q12;
                8'h52:forward_q12=-FORWARD_Q12;
                8'h08:yaw_q12=TURN_Q12;
                8'h5a:yaw_q12=-TURN_Q12;
                default:begin forward_q12=0;yaw_q12=0;end
            endcase
        end
    end
    // +/- changes height by one Q12 unit per10ms (~24.4mm/s).
    // NEC repeats keep the key live; release timeout also stops height motion.
    always @(posedge clk) begin
        if(!reset_n) begin
            height_q12<=HEIGHT_INITIAL_Q12<348 ? HEIGHT_MIN_Q12 :
                        HEIGHT_INITIAL_Q12>778 ? HEIGHT_MAX_Q12 : HEIGHT_INITIAL_Q12;
            height_count<=0;
        end else if(emergency_stop || !key_active ||
                    (key_code!=8'h15 && key_code!=8'h07)) height_count<=0;
        else if(height_count==HEIGHT_STEP_COUNT-1) begin
            height_count<=0;
            if(key_code==8'h15 && height_q12<HEIGHT_MAX_Q12) height_q12<=height_q12+24'sd1;
            else if(key_code==8'h07 && height_q12>HEIGHT_MIN_Q12) height_q12<=height_q12-24'sd1;
        end else height_count<=height_count+1'b1;
    end
    always @(posedge clk) begin
        if(!reset_n) begin
            ir_sync_0<=1;ir_sync_1<=1;ir_last<=1;
            edge_count<=32'hffffffff;frame_bits<=0;bit_count<=0;receiving<=0;
            hold_count<=0;key_code<=0;key_event<=0;repeat_event<=0;emergency_stop<=0;
        end else begin
            ir_sync_0<=ir_in;ir_sync_1<=ir_sync_0;ir_last<=ir_sync_1;
            key_event<=0;repeat_event<=0;
            if(edge_count!=32'hffffffff) edge_count<=edge_count+1'b1;
            if(edge_count>LEADER_MAX) receiving<=0;
            if(hold_count!=0) hold_count<=hold_count-1'b1;
            if(falling) begin
                edge_count<=0;
                if(edge_count>=LEADER_MIN && edge_count<=LEADER_MAX) begin
                    receiving<=1;bit_count<=0;frame_bits<=0;
                end else if(edge_count>=REPEAT_MIN && edge_count<=REPEAT_MAX) begin
                    receiving<=0;
                    // A repeat can only extend a still-live, fully checked key.
                    if(hold_count!=0) begin hold_count<=KEY_HOLD_COUNT;repeat_event<=1;end
                end else if(receiving) begin
                    if(bit_zero || bit_one) begin
                        frame_bits[bit_count]<=bit_one;
                        if(bit_count==31) begin
                            receiving<=0;
                            // Same standard/extended-NEC address policy as reference.
                            if((completed_frame[23:16]^completed_frame[31:24])==8'hff) begin
                                key_code<=completed_frame[23:16];hold_count<=KEY_HOLD_COUNT;key_event<=1;
                                // 5 is latched until hardware reset, never toggled by repeats.
                                if(completed_frame[23:16]==8'h1c) emergency_stop<=1;
                            end
                        end else bit_count<=bit_count+1'b1;
                    end else begin receiving<=0;bit_count<=0;end
                end
            end
        end
    end
endmodule
