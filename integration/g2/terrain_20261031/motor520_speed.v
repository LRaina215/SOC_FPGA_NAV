// 520 AB quadrature speed measurement. Parameters/pins follow the competition project.
// 65mm wheel; 11 pulses/phase/motor turn; 30:1 gearbox; quadrature CPR=1320.
// Speed output is signed 0.01m/s (cm/s), with symmetric nearest rounding.
module motor520_speed #(
    parameter integer CLK_HZ=50_000_000,
    parameter integer SPEED_GATE_CYCLES=500_000,
    parameter integer ENCODER_CPR=1320,
    parameter integer WHEEL_DIAMETER_UM=65_000,
    parameter integer ENCODER_SIGN=1,
    // FILTER_CYCLES>0: an A/B level change is accepted only after it has been stable for this many clocks
    // (PWM switching noise on the encoder wires produced whole fake quadrature cycles).
    parameter integer FILTER_CYCLES=0,
    // MAX_DELTA>0: a gate count above this magnitude is physically impossible; keep the previous sample.
    parameter integer MAX_DELTA=0
)(
    input wire clk,reset_n,enc_a,enc_b,
    output reg signed [31:0] position,speed_delta,
    output reg signed [15:0] speed_cms,
    output reg sample_valid,
    output reg [31:0] invalid_transitions,
    output reg [15:0] rejected_samples
);
    // Reduce the constant conversion ratio at elaboration; no hardware division here.
    function [63:0] gcd64;
        input [63:0] a,b;
        reg [63:0] x,y,t;
        begin
            x=a;y=b;
            while(y!=0) begin t=x%y;x=y;y=t;end
            gcd64=x;
        end
    endfunction
    localparam [63:0] CIRCUMFERENCE_UM=(64'd1*WHEEL_DIAMETER_UM*3141593+500000)/1000000;
    localparam [63:0] NUMERATOR=64'd1*CIRCUMFERENCE_UM*CLK_HZ*100;
    localparam [63:0] DENOMINATOR=64'd1*ENCODER_CPR*SPEED_GATE_CYCLES*1000000;
    localparam [63:0] COMMON=gcd64(NUMERATOR,DENOMINATOR);
    localparam [63:0] GAIN_NUM=NUMERATOR/COMMON;
    localparam [63:0] GAIN_DEN=DENOMINATOR/COMMON;
    localparam NUM_BITS=32+$clog2(GAIN_NUM+1)+1;
    localparam REM_BITS=($clog2(GAIN_DEN+1)<1) ? 1 : $clog2(GAIN_DEN+1);
    localparam [NUM_BITS-1:0] NUM_CONST=GAIN_NUM;
    localparam [NUM_BITS-1:0] ROUND_CONST=GAIN_DEN/2;
    localparam [REM_BITS-1:0] DEN_CONST=GAIN_DEN;
    reg [2:0] sync_a,sync_b,warmup;
    localparam FB=(FILTER_CYCLES>1) ? $clog2(FILTER_CYCLES) : 1;
    reg [FB-1:0] stable_a,stable_b;
    reg filt_a,filt_b,prev_a,prev_b;
    always @(posedge clk) begin
        if(!reset_n) begin
            stable_a<=0;stable_b<=0;filt_a<=0;filt_b<=0;prev_a<=0;prev_b<=0;
        end else begin
            prev_a<=filt_a;prev_b<=filt_b;
            // Start from the settled input level so the filter does not fabricate an edge after warm-up.
            if(warmup<5) begin filt_a<=sync_a[1];filt_b<=sync_b[1];prev_a<=sync_a[1];prev_b<=sync_b[1];stable_a<=0;stable_b<=0;end
            else begin
            if(sync_a[1]==filt_a) stable_a<=0;
            else if(stable_a>=FILTER_CYCLES-1) begin filt_a<=sync_a[1];stable_a<=0;end
            else stable_a<=stable_a+1'b1;
            if(sync_b[1]==filt_b) stable_b<=0;
            else if(stable_b>=FILTER_CYCLES-1) begin filt_b<=sync_b[1];stable_b<=0;end
            else stable_b<=stable_b+1'b1;
            end
        end
    end
    wire [3:0] transition=(FILTER_CYCLES>0) ? {prev_a,prev_b,filt_a,filt_b} :
                                              {sync_a[2],sync_b[2],sync_a[1],sync_b[1]};
    wire signed [31:0] gate_abs_limit=MAX_DELTA;
    reg signed [31:0] step;
    always @* begin
        case(transition)
            4'b0001,4'b0111,4'b1110,4'b1000:step=32'sd1;
            4'b0010,4'b1011,4'b1101,4'b0100:step=-32'sd1;
            default:step=32'sd0;
        endcase
    end
    reg signed [31:0] accumulator;
    wire signed [31:0] gate_total=accumulator+step;
    wire implausible=(MAX_DELTA>0) && (gate_total>gate_abs_limit || gate_total< -gate_abs_limit);
    reg [31:0] gate_counter;
    reg convert_start;
    always @(posedge clk) begin
        convert_start<=0;
        if(!reset_n) begin
            sync_a<=0;sync_b<=0;warmup<=0;position<=0;speed_delta<=0;
            accumulator<=0;gate_counter<=0;invalid_transitions<=0;rejected_samples<=0;
        end else begin
            sync_a<={sync_a[1:0],enc_a};sync_b<={sync_b[1:0],enc_b};
            // Let the initial AB state settle; do not fabricate an edge at power-up.
            if(warmup<5) begin warmup<=warmup+1'b1;gate_counter<=0;end
            else begin
                position<=position+step;
                if(transition[3:2]!=transition[1:0] && step==0)
                    invalid_transitions<=invalid_transitions+1'b1;
                if(gate_counter>=SPEED_GATE_CYCLES-1) begin
                    if(implausible) begin
                        if(rejected_samples!=16'hFFFF) rejected_samples<=rejected_samples+1'b1;
                    end else speed_delta<=gate_total;
                    accumulator<=0;gate_counter<=0;convert_start<=1;
                end else begin accumulator<=gate_total;gate_counter<=gate_counter+1'b1;end
            end
        end
    end
    // Exact rational conversion: v(cm/s)=delta*1547/1000 for the default parameters.
    // A sequential divider avoids a long combinational division path.
    localparam IDLE=0,MULTIPLY=1,DIVIDE=2,COMMIT=3;
    reg [1:0] state;
    reg negative;
    reg [31:0] magnitude;
    reg [NUM_BITS-1:0] dividend,quotient;
    reg [REM_BITS-1:0] remainder;
    reg [7:0] div_count;
    wire [REM_BITS:0] trial={remainder,dividend[NUM_BITS-1]};
    wire take=(trial>={1'b0,DEN_CONST});
    wire [REM_BITS:0] reduced=take ? (trial-{1'b0,DEN_CONST}) : trial;
    always @(posedge clk) begin
        sample_valid<=0;
        if(!reset_n) begin
            state<=IDLE;negative<=0;magnitude<=0;dividend<=0;quotient<=0;
            remainder<=0;div_count<=0;speed_cms<=0;
        end else case(state)
            IDLE:if(convert_start) begin
                magnitude<=speed_delta[31] ? (~speed_delta+32'd1) : speed_delta;
                negative<=speed_delta[31] ^ (ENCODER_SIGN<0);state<=MULTIPLY;
            end
            MULTIPLY:begin
                dividend<={ {(NUM_BITS-32){1'b0}},magnitude}*NUM_CONST+ROUND_CONST;quotient<=0;remainder<=0;
                div_count<=0;state<=DIVIDE;
            end
            DIVIDE:begin
                dividend<={dividend[NUM_BITS-2:0],1'b0};
                quotient<={quotient[NUM_BITS-2:0],take};remainder<=reduced[REM_BITS-1:0];
                if(div_count==NUM_BITS-1) state<=COMMIT;else div_count<=div_count+1'b1;
            end
            COMMIT:begin
                if(quotient>32767) speed_cms<=negative ? -16'sd32767 : 16'sd32767;
                else speed_cms<=negative ? -$signed(quotient[15:0]) : $signed(quotient[15:0]);
                sample_valid<=1;state<=IDLE;
            end
        endcase
    end
endmodule
