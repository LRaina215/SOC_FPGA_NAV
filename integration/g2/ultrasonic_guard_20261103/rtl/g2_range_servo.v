// Candidate based on K_terrain19_follow1/video/g2_range_servo.v.
// Existing 65ms/12us/30ms HC-SR04 timing and 20ms SG90 PWM retained.
module g2_range_servo #(
    parameter integer CLOCKS_PER_US=50,
    parameter integer SERVO_HOLD_FRAMES=50,
    parameter integer FORWARD_SETTLE_FRAMES=15
)(
    input wire clk,reset_n,echo,hold_forward,
    output reg trig,servo_pwm,
    output reg [15:0] distance_mm,
    output reg distance_valid,distance_timeout,
    output reg [1:0] servo_position,
    output reg sample_strobe,
    output wire forward_ready
);
    reg [5:0] us_div;
    wire us_tick=us_div==CLOCKS_PER_US-1;
    (* syn_preserve=1 *) reg echo_meta,echo_sync;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin us_div<=0;echo_meta<=0;echo_sync<=0;end
        else begin
            us_div<=us_tick ? 0 : us_div+1'b1;
            echo_meta<=echo;echo_sync<=echo_meta;
        end
    end
    reg [14:0] servo_us;
    reg [5:0] hold_frames;
    reg [15:0] settle_frames;
    reg center_frame_started;
    wire [11:0] pulse_us=servo_position==0 ? 12'd500 :
                               servo_position==1 ? 12'd1500 : 12'd2500;
    assign forward_ready=hold_forward && servo_position==1 && settle_frames>=FORWARD_SETTLE_FRAMES;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            servo_us<=0;hold_frames<=0;servo_position<=0;servo_pwm<=0;
            settle_frames<=0;center_frame_started<=0;
        end else begin
          if(!hold_forward) begin settle_frames<=0;center_frame_started<=0;end
          if(us_tick) begin
            servo_pwm<=servo_us<pulse_us;
            if(servo_us==19999) begin
                servo_us<=0;
                if(hold_forward) begin
                    servo_position<=1;hold_frames<=0;
                    if(servo_position!=1 || !center_frame_started) begin
                        settle_frames<=0;center_frame_started<=1;
                    end
                    else if(settle_frames<FORWARD_SETTLE_FRAMES) settle_frames<=settle_frames+1'b1;
                end else if(hold_frames==SERVO_HOLD_FRAMES-1) begin
                    hold_frames<=0;servo_position<=servo_position==2 ? 0 : servo_position+1'b1;
                end else hold_frames<=hold_frames+1'b1;
            end else servo_us<=servo_us+1'b1;
          end
        end
    end
    reg hold_previous;
    reg [15:0] period_us;
    reg [14:0] wait_us;
    reg [12:0] measured_mm;
    reg [5:0] remainder;
    wire [6:0] next_remainder={1'b0,remainder}+7'd10;
    localparam IDLE=2'd0,WAIT_HIGH=2'd1,MEASURE=2'd2;
    reg [1:0] state;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            period_us<=0;wait_us<=0;measured_mm<=0;remainder<=0;hold_previous<=0;
            state<=IDLE;trig<=0;distance_mm<=16'hffff;
            distance_valid<=0;distance_timeout<=0;sample_strobe<=0;
        end else begin
            sample_strobe<=0;
            hold_previous<=hold_forward;
            if(hold_previous!=hold_forward || (hold_forward && !forward_ready)) begin
                // Abort scans/in-flight echoes when entering or leaving the forward mode.
                state<=IDLE;trig<=0;distance_valid<=0;distance_timeout<=0;
                distance_mm<=16'hffff;period_us<=0;wait_us<=0;measured_mm<=0;remainder<=0;
            end else if(us_tick) begin
                period_us<=period_us==64999 ? 0 : period_us+1'b1;
                trig<=period_us<12;
                if(period_us==12) begin
                    state<=echo_sync ? IDLE : WAIT_HIGH;
                    wait_us<=0;measured_mm<=0;remainder<=0;
                    if(echo_sync) begin
                        distance_valid<=0;distance_timeout<=1;distance_mm<=16'hffff;sample_strobe<=1;
                    end
                end else case(state)
                    WAIT_HIGH: begin
                        if(echo_sync) begin state<=MEASURE;wait_us<=0;end
                        else if(wait_us==29999) begin
                            state<=IDLE;distance_valid<=0;distance_timeout<=1;
                            distance_mm<=16'hffff;sample_strobe<=1;
                        end else wait_us<=wait_us+1'b1;
                    end
                    MEASURE: begin
                        if(!echo_sync) begin
                            state<=IDLE;distance_timeout<=0;sample_strobe<=1;
                            distance_valid<=measured_mm>=20 && measured_mm<=4000;
                            distance_mm<=measured_mm>=20 && measured_mm<=4000 ?
                                {3'b0,measured_mm} : 16'hffff;
                        end else if(wait_us==29999) begin
                            state<=IDLE;distance_valid<=0;distance_timeout<=1;
                            distance_mm<=16'hffff;sample_strobe<=1;
                        end else begin
                            wait_us<=wait_us+1'b1;
                            if(next_remainder>=58) begin
                                remainder<=next_remainder-7'd58;measured_mm<=measured_mm+1'b1;
                            end else remainder<=next_remainder[5:0];
                        end
                    end
                    default:state<=IDLE;
                endcase
            end
        end
    end
endmodule
