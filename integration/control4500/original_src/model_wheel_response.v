// Model target conditioning only; this is not a motor-plant model.
// At 2ms/sample: five stored samples =10ms delay, alpha2849/65536
// approximates 1-exp(-2/45). State retains12 fractional bits of mm/s.
// Three arithmetic stages keep the multiplier out of the state feedback path.
module model_wheel_response #(
    parameter integer UPDATE_CYCLES=100000,DELAY_STEPS=5,ALPHA_Q16=2849
)(
    input wire clk,reset_n,enable,
    input wire signed [31:0] left_target_mmps,right_target_mmps,
    output wire signed [31:0] left_filtered_mmps,right_filtered_mmps
);
    localparam CW=$clog2(UPDATE_CYCLES+1);
    localparam signed [16:0] COEFFICIENT=ALPHA_Q16;
    localparam IDLE=0,MULTIPLY=1,ACCUMULATE=2,PUBLISH=3;
    reg [CW-1:0] counter;
    reg [1:0] state;
    reg signed [15:0] delay_left[0:DELAY_STEPS-1];
    reg signed [15:0] delay_right[0:DELAY_STEPS-1];
    reg signed [23:0] filter_left,filter_right;
    reg signed [24:0] delta_left,delta_right;
    reg signed [41:0] product_left,product_right;
    reg signed [31:0] output_left,output_right;
    integer i;
    function signed [15:0] bound;
        input signed [31:0] v;
        begin bound=v>600 ? 16'sd600 : v < -600 ? -16'sd600 : v[15:0];end
    endfunction
    function signed [24:0] as_q12;
        input signed [15:0] v;
        begin as_q12=$signed(v)*25'sd4096;end
    endfunction
    function signed [23:0] round_product;
        input signed [41:0] v;
        begin round_product=v<0 ? -((-v+42'sd32768)>>>16) : ((v+42'sd32768)>>>16);end
    endfunction
    function signed [31:0] round_state;
        input signed [23:0] v;
        begin round_state=v<0 ? -((-v+24'sd2048)>>>12) : ((v+24'sd2048)>>>12);end
    endfunction
    // Cut off immediately on loss of enable; queued commands are cleared at
    // the next clock, independently of delayed/filtered target decay.
    assign left_filtered_mmps=enable ? output_left : 32'sd0;
    assign right_filtered_mmps=enable ? output_right : 32'sd0;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            counter<=0;state<=IDLE;filter_left<=0;filter_right<=0;
            delta_left<=0;delta_right<=0;product_left<=0;product_right<=0;
            output_left<=0;output_right<=0;
            for(i=0;i<DELAY_STEPS;i=i+1)begin delay_left[i]<=0;delay_right[i]<=0;end
        end else if(!enable) begin
            counter<=0;state<=IDLE;filter_left<=0;filter_right<=0;
            delta_left<=0;delta_right<=0;product_left<=0;product_right<=0;
            output_left<=0;output_right<=0;
            for(i=0;i<DELAY_STEPS;i=i+1)begin delay_left[i]<=0;delay_right[i]<=0;end
        end else begin
            if(counter==UPDATE_CYCLES-1) begin
                counter<=0;
                delay_left[0]<=bound(left_target_mmps);delay_right[0]<=bound(right_target_mmps);
                for(i=1;i<DELAY_STEPS;i=i+1) begin
                    delay_left[i]<=delay_left[i-1];delay_right[i]<=delay_right[i-1];
                end
                delta_left<=as_q12(delay_left[DELAY_STEPS-1])-$signed(filter_left);
                delta_right<=as_q12(delay_right[DELAY_STEPS-1])-$signed(filter_right);
                state<=MULTIPLY;
            end else counter<=counter+1'b1;
            case(state)
                MULTIPLY:begin
                    product_left<=$signed(delta_left)*COEFFICIENT;
                    product_right<=$signed(delta_right)*COEFFICIENT;
                    state<=ACCUMULATE;
                end
                ACCUMULATE:begin
                    filter_left<=filter_left+round_product(product_left);
                    filter_right<=filter_right+round_product(product_right);
                    state<=PUBLISH;
                end
                PUBLISH:begin output_left<=round_state(filter_left);output_right<=round_state(filter_right);state<=IDLE;end
            endcase
        end
    end
endmodule
