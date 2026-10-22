// Four calibrated servo PWM channels:1 left hip,2 left knee,3 right hip,4 right knee.
// pulse_*_us are logical pulse-width commands in microseconds. Default direction map:+,-,+,+.
// Updates are latched together at20ms frame boundaries; outputs are clock-registered.
module servo_driver #(
    parameter integer CLK_HZ=50_000_000,
    parameter integer FRAME_US=20_000,
    parameter [15:0] MIN_US=16'd500,MAX_US=16'd2500,
    parameter [15:0] CENTER_LH_US=16'd1500,CENTER_LK_US=16'd1500,
    parameter [15:0] CENTER_RH_US=16'd1500,CENTER_RK_US=16'd1500,
    parameter integer LH_SIGN=1,LK_SIGN=-1,RH_SIGN=1,RK_SIGN=1,
    // Set0 when the upstream controller already applied calibrated signs/IK to physical pulses.
    parameter APPLY_CALIBRATED_DIRECTION=1'b1
)(
    input wire clk,reset_n,enable,
    input wire [15:0] pulse_lh_us,pulse_lk_us,pulse_rh_us,pulse_rk_us,
    output reg SERVO1,SERVO2,SERVO3,SERVO4,
    output reg frame_tick,
    output reg [15:0] active_lh_us,active_lk_us,active_rh_us,active_rk_us
);
    localparam integer CLOCKS_PER_US=CLK_HZ/1_000_000;
    localparam integer DIV_BITS=(CLOCKS_PER_US<=1) ? 1 : $clog2(CLOCKS_PER_US);
    localparam integer FRAME_BITS=(FRAME_US<=1) ? 1 : $clog2(FRAME_US);
    reg [DIV_BITS-1:0] divider;
    reg [FRAME_BITS-1:0] frame_counter;
    reg running;
    function [15:0] calibrated_pulse;
        input [15:0] command_us,center_us;
        input integer direction;
        reg signed [31:0] physical_us;
        begin
            if(APPLY_CALIBRATED_DIRECTION && direction<0)
                physical_us=32'sd2*$signed({1'b0,center_us})-$signed({1'b0,command_us});
            else physical_us=$signed({1'b0,command_us});
            if(physical_us<$signed({1'b0,MIN_US})) calibrated_pulse=MIN_US;
            else if(physical_us>$signed({1'b0,MAX_US})) calibrated_pulse=MAX_US;
            else calibrated_pulse=physical_us[15:0];
        end
    endfunction
    wire [15:0] command_lh=calibrated_pulse(pulse_lh_us,CENTER_LH_US,LH_SIGN);
    wire [15:0] command_lk=calibrated_pulse(pulse_lk_us,CENTER_LK_US,LK_SIGN);
    wire [15:0] command_rh=calibrated_pulse(pulse_rh_us,CENTER_RH_US,RH_SIGN);
    wire [15:0] command_rk=calibrated_pulse(pulse_rk_us,CENTER_RK_US,RK_SIGN);
    wire [FRAME_BITS:0] next_us={1'b0,frame_counter}+1'b1;
    always @(posedge clk) begin
        frame_tick<=0;
        if(!reset_n || !enable) begin
            divider<=0;frame_counter<=0;running<=0;
            SERVO1<=0;SERVO2<=0;SERVO3<=0;SERVO4<=0;
            active_lh_us<=calibrated_pulse(CENTER_LH_US,CENTER_LH_US,LH_SIGN);
            active_lk_us<=calibrated_pulse(CENTER_LK_US,CENTER_LK_US,LK_SIGN);
            active_rh_us<=calibrated_pulse(CENTER_RH_US,CENTER_RH_US,RH_SIGN);
            active_rk_us<=calibrated_pulse(CENTER_RK_US,CENTER_RK_US,RK_SIGN);
        end else if(!running) begin
            running<=1;divider<=0;frame_counter<=0;frame_tick<=1;
            active_lh_us<=command_lh;active_lk_us<=command_lk;
            active_rh_us<=command_rh;active_rk_us<=command_rk;
            SERVO1<=1;SERVO2<=1;SERVO3<=1;SERVO4<=1;
        end else if(divider==CLOCKS_PER_US-1) begin
            divider<=0;
            if(frame_counter==FRAME_US-1) begin
                frame_counter<=0;frame_tick<=1;
                active_lh_us<=command_lh;active_lk_us<=command_lk;
                active_rh_us<=command_rh;active_rk_us<=command_rk;
                SERVO1<=1;SERVO2<=1;SERVO3<=1;SERVO4<=1;
            end else begin
                frame_counter<=next_us[FRAME_BITS-1:0];
                SERVO1<=(next_us<active_lh_us);SERVO2<=(next_us<active_lk_us);
                SERVO3<=(next_us<active_rh_us);SERVO4<=(next_us<active_rk_us);
            end
        end else divider<=divider+1'b1;
    end
endmodule
