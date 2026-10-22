`timescale 1ns/1ps

module fc_v4_top #(
    parameter WEIGHTS_FILE = "/home/cyd/fpga/pt_to_fpga/full_center_v4_deploy_20261005/assets/weights.mem",
    parameter BIASES_FILE = "/home/cyd/fpga/pt_to_fpga/full_center_v4_deploy_20261005/assets/bias.mem",
    parameter ELU_FILE = "/home/cyd/fpga/pt_to_fpga/full_center_v4_deploy_20261005/assets/elu.mem"
) (
    input wire clk,
    input wire rst_n,
    input wire wr_en,
    input wire wr_history,
    input wire [7:0] wr_addr,
    input wire signed [23:0] wr_data,
    input wire start,
    input wire [31:0] frame_id,
    output reg busy,
    output reg done,
    output reg [4:0] error_flags,
    output reg saturated,
    output reg [31:0] result_frame_id,
    output reg [31:0] cycles,
    output reg signed [23:0] action0,
    output reg signed [23:0] action1,
    output reg signed [23:0] action2,
    output reg signed [23:0] action3,
    output reg signed [23:0] action4,
    output reg signed [23:0] action5,
    output reg dbg_valid,
    output reg [2:0] dbg_layer,
    output reg [7:0] dbg_index,
    output reg signed [23:0] dbg_data
);
    localparam IDLE=0, BIAS_REQ=1, BIAS_CAPTURE=2, MAC_REQ=3,
               MAC_ACC=4, NORMALIZE=5, ACT_WAIT=6, EMIT=7, COMMIT=8;
    reg [3:0] state;
    reg signed [15:0] weight_mem [0:39935];
    reg signed [31:0] bias_mem [0:424];
    reg signed [23:0] elu_mem [0:3072];
    // Busy gates every write, making this memory the immutable frame snapshot.
    reg signed [23:0] input_mem [0:161];
    reg [161:0] input_valid;
    reg signed [23:0] bank_a [0:127];
    reg signed [23:0] bank_b [0:127];
    reg signed [23:0] latent [0:2];
    reg signed [23:0] pending_action [0:5];
    reg [31:0] pending_frame_id;
    reg [2:0] layer;
    reg [7:0] neuron, input_index, n_inputs, n_outputs;
    reg [15:0] weight_base;
    reg [8:0] bias_base;
    wire [15:0] weight_address = weight_base + neuron * n_inputs + input_index;
    wire [8:0] bias_address = bias_base + neuron;
    wire [7:0] write_address = wr_history ? wr_addr + 8'd27 : wr_addr;
    reg signed [15:0] weight_q;
    reg signed [31:0] bias_q;
    reg signed [23:0] input_q, elu_q;
    wire signed [39:0] product = input_q * weight_q;
    reg signed [47:0] accumulator;
    wire signed [23:0] normalized;
    wire normalize_saturated;
    reg signed [23:0] normalized_q;
    wire linear_layer = (layer == 2 || layer == 6);
    wire signed [23:0] activated = (linear_layer || normalized_q >= 0) ? normalized_q : elu_q;
    reg [24:0] elu_magnitude;
    reg [11:0] elu_address;

    fc_v4_round_sat normalizer(.acc(accumulator), .value(normalized), .saturated(normalize_saturated));

    function [23:0] clip_action;
        input signed [23:0] value;
        begin
            if (value > 24'sd4096) clip_action = 24'sd4096;
            else if (value < -24'sd4096) clip_action = -24'sd4096;
            else clip_action = value;
        end
    endfunction

    initial begin
        $readmemh(WEIGHTS_FILE, weight_mem);
        $readmemh(BIASES_FILE, bias_mem);
        $readmemh(ELU_FILE, elu_mem);
    end

    always @* begin
        elu_magnitude = 25'd0;
        elu_address = 12'd0;
        if (normalized_q < 0) begin
            elu_magnitude = -$signed({normalized_q[23], normalized_q}) + 25'd8;
            if ((elu_magnitude >> 4) > 25'd3072) elu_address = 12'd3072;
            else elu_address = elu_magnitude[15:4];
        end
    end

    // Clocked, enabled reads map naturally onto FPGA ROM/RAM resources.
    always @(posedge clk) begin
        if (state == BIAS_REQ) bias_q <= bias_mem[bias_address];
        if (state == MAC_REQ) begin
            weight_q <= weight_mem[weight_address];
            if (layer == 0) input_q <= input_mem[8'd27 + input_index];
            else if (layer == 3) begin
                if (input_index < 27) input_q <= input_mem[input_index];
                else input_q <= latent[input_index - 8'd27];
            end else if (layer[0]) input_q <= bank_a[input_index];
            else input_q <= bank_b[input_index];
        end
        if (state == ACT_WAIT) elu_q <= elu_mem[elu_address];
        if (wr_en && !busy && ((!wr_history && wr_addr < 27) || (wr_history && wr_addr < 135)))
            input_mem[write_address] <= wr_data;
        if (state == EMIT) begin
            if (layer[0]) bank_b[neuron] <= activated;
            else bank_a[neuron] <= activated;
            if (layer == 2) latent[neuron] <= activated;
            if (layer == 6) pending_action[neuron] <= clip_action(activated);
        end
    end

    // Each accepted start consumes readiness; reload all162 for the next frame.
    // Flags: busy-write, busy-or-same-edge-start, bad-address, not-ready, sat24.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            busy <= 0; done <= 0; error_flags <= 0; saturated <= 0;
            input_valid <= 0; result_frame_id <= 0; pending_frame_id <= 0; cycles <= 0;
            action0 <= 0; action1 <= 0; action2 <= 0;
            action3 <= 0; action4 <= 0; action5 <= 0;
            dbg_valid <= 0; dbg_layer <= 0; dbg_index <= 0; dbg_data <= 0;
            layer <= 0; neuron <= 0; input_index <= 0;
            n_inputs <= 135; n_outputs <= 128; weight_base <= 0; bias_base <= 0;
            accumulator <= 0; normalized_q <= 0;
        end else begin
            done <= 0;
            dbg_valid <= 0;
            if (busy) cycles <= cycles + 1'b1;
            if (wr_en) begin
                if (busy) error_flags[0] <= 1;
                else if ((!wr_history && wr_addr >= 27) || (wr_history && wr_addr >= 135))
                    error_flags[2] <= 1;
                else input_valid[write_address] <= 1;
            end
            if (start && busy) error_flags[1] <= 1;
            case (state)
                IDLE: if (start) begin
                    // A write and start on the same edge is rejected. The
                    // producer must complete a coherent snapshot first.
                    if (wr_en) error_flags[1] <= 1;
                    else if (!(&input_valid)) error_flags[3] <= 1;
                    else begin
                        busy <= 1; cycles <= 0; saturated <= 0;
                        input_valid <= 0;
                        pending_frame_id <= frame_id;
                        layer <= 0; neuron <= 0; input_index <= 0;
                        n_inputs <= 135; n_outputs <= 128; weight_base <= 0; bias_base <= 0;
                        state <= BIAS_REQ;
                    end
                end
                BIAS_REQ: state <= BIAS_CAPTURE;
                BIAS_CAPTURE: begin
                    accumulator <= {{16{bias_q[31]}}, bias_q};
                    input_index <= 0;
                    state <= MAC_REQ;
                end
                MAC_REQ: state <= MAC_ACC;
                MAC_ACC: begin
                    accumulator <= accumulator + {{8{product[39]}}, product};
                    if (input_index == n_inputs - 1'b1) state <= NORMALIZE;
                    else begin input_index <= input_index + 1'b1; state <= MAC_REQ; end
                end
                NORMALIZE: begin
                    normalized_q <= normalized;
                    if (normalize_saturated) begin saturated <= 1; error_flags[4] <= 1; end
                    state <= ACT_WAIT;
                end
                ACT_WAIT: state <= EMIT;
                EMIT: begin
                    dbg_valid <= 1; dbg_layer <= layer; dbg_index <= neuron; dbg_data <= activated;
                    if (neuron != n_outputs - 1'b1) begin
                        neuron <= neuron + 1'b1; state <= BIAS_REQ;
                    end else if (layer == 6) state <= COMMIT;
                    else begin
                        layer <= layer + 1'b1; neuron <= 0; state <= BIAS_REQ;
                        case (layer)
                            0: begin n_inputs<=128; n_outputs<=64; weight_base<=17280; bias_base<=128; end
                            1: begin n_inputs<=64; n_outputs<=3; weight_base<=25472; bias_base<=192; end
                            2: begin n_inputs<=30; n_outputs<=128; weight_base<=25664; bias_base<=195; end
                            3: begin n_inputs<=128; n_outputs<=64; weight_base<=29504; bias_base<=323; end
                            4: begin n_inputs<=64; n_outputs<=32; weight_base<=37696; bias_base<=387; end
                            5: begin n_inputs<=32; n_outputs<=6; weight_base<=39744; bias_base<=419; end
                            default: state <= IDLE;
                        endcase
                    end
                end
                COMMIT: begin
                    action0 <= pending_action[0]; action1 <= pending_action[1]; action2 <= pending_action[2];
                    action3 <= pending_action[3]; action4 <= pending_action[4]; action5 <= pending_action[5];
                    result_frame_id <= pending_frame_id; busy <= 0; done <= 1; state <= IDLE;
                end
                default: begin state <= IDLE; busy <= 0; end
            endcase
        end
    end
endmodule
