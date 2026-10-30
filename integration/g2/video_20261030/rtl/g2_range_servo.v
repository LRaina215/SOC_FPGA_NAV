// Source: Chenyudog/fpga@715378a / fpga/src/range_servo.v
// Integration copy: module identifiers only were namespaced; original notices retained.
// 50MHz shared 1us enable. No multiplied clocks, DSPs, RAM or divider.
// HC-SR04: 65ms trigger interval, 12us trigger, 30ms echo timeout.
// distance_mm=floor(echo_us*10/58); accuracy is sensor-limited, not 1mm.
module g2_range_servo #(
    parameter integer CLOCKS_PER_US=50,
    parameter integer SERVO_HOLD_FRAMES=50
)(
    input wire clk, reset_n, echo,
    output reg trig, servo_pwm,
    output reg [15:0] distance_mm,
    output reg distance_valid, distance_timeout,
    output reg [1:0] servo_position
);
    reg [5:0] us_div;
    wire us_tick=(us_div==CLOCKS_PER_US-1);
    (* syn_preserve=1 *) reg echo_meta, echo_sync;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin us_div<=0;echo_meta<=0;echo_sync<=0;end
        else begin
            us_div<=us_tick ? 0 : us_div+1'b1;
            echo_meta<=echo;echo_sync<=echo_meta;
        end
    end
    reg [14:0] servo_us;
    reg [5:0] hold_frames;
    wire [11:0] pulse_us=servo_position==0 ? 12'd500 :
                              servo_position==1 ? 12'd1500 : 12'd2500;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            servo_us<=0;hold_frames<=0;servo_position<=0;servo_pwm<=0;
        end else if(us_tick) begin
            servo_pwm<=(servo_us<pulse_us);
            if(servo_us==19999) begin
                servo_us<=0;
                if(hold_frames==SERVO_HOLD_FRAMES-1) begin
                    hold_frames<=0;
                    servo_position<=servo_position==2 ? 0 : servo_position+1'b1;
                end else hold_frames<=hold_frames+1'b1;
            end else servo_us<=servo_us+1'b1;
        end
    end
    reg [15:0] period_us;
    reg [14:0] wait_us;
    reg [12:0] measured_mm;
    reg [5:0] remainder;
    wire [6:0] next_remainder={1'b0,remainder}+7'd10;
    localparam IDLE=2'd0, WAIT_HIGH=2'd1, MEASURE=2'd2;
    reg [1:0] state;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            period_us<=0;wait_us<=0;measured_mm<=0;remainder<=0;
            state<=IDLE;trig<=0;distance_mm<=16'hffff;
            distance_valid<=0;distance_timeout<=0;
        end else if(us_tick) begin
            period_us<=period_us==64999 ? 0 : period_us+1'b1;
            trig<=(period_us<12);
            if(period_us==12) begin
                // Do not accept stale high Echo from a prior transaction.
                state<=echo_sync ? IDLE : WAIT_HIGH;
                wait_us<=0;measured_mm<=0;remainder<=0;
                if(echo_sync) begin
                    distance_valid<=0;distance_timeout<=1;distance_mm<=16'hffff;
                end
            end else case(state)
                WAIT_HIGH: begin
                    if(echo_sync) begin state<=MEASURE;wait_us<=0;end
                    else if(wait_us==29999) begin
                        state<=IDLE;distance_valid<=0;distance_timeout<=1;distance_mm<=16'hffff;
                    end else wait_us<=wait_us+1'b1;
                end
                MEASURE: begin
                    if(!echo_sync) begin
                        state<=IDLE;distance_timeout<=0;
                        distance_valid<=(measured_mm>=20 && measured_mm<=4000);
                        distance_mm<=(measured_mm>=20 && measured_mm<=4000) ?
                                         {3'b0,measured_mm} : 16'hffff;
                    end else if(wait_us==29999) begin
                        state<=IDLE;distance_valid<=0;distance_timeout<=1;distance_mm<=16'hffff;
                    end else begin
                        wait_us<=wait_us+1'b1;
                        if(next_remainder>=58) begin
                            remainder<=next_remainder-7'd58;measured_mm<=measured_mm+1'b1;
                        end else remainder<=next_remainder[5:0];
                    end
                end
                default: state<=IDLE;
            endcase
        end
    end
endmodule

