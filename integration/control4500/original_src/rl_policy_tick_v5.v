`ifndef RL_POLICY_TICK_V5_V_INCLUDED
`define RL_POLICY_TICK_V5_V_INCLUDED
`timescale 1ns/1ps
// 50MHz / 400000 = 125Hz. Snapshot -> load 168 words -> run -> publish.
// This controller has no actuator pins. A snapshot provider responds to capture
// with frame_valid/current_flat/history_flat and holds them until the next capture.
// Frames include already updated history. No UART request is needed.
module rl_policy_tick_v5 #(
    parameter integer WEIGHT_BITS = 16,
    parameter integer PERIOD_CLKS = 400000,
    parameter RETRY_ON_FAULT = 1'b0,
    parameter WEIGHTS_FILE = "src/mem/weights.mem",
    parameter BIASES_FILE = "src/mem/bias.mem",
    parameter ELU_FILE = "src/mem/elu.mem"
)(
    input wire clk, rst_n, enable, sensor_ok,
    input wire frame_valid,
    input wire [671:0] current_flat,
    input wire [3359:0] history_flat,
    output reg capture,
    output reg action_valid,
    output reg [143:0] actions,
    output reg [31:0] completed_frames,
    output reg [31:0] inference_cycles,
    output reg [2:0] fault,
    output reg [2:0] diag_last_fault,
    output reg [6:0] diag_core_detail,
    output reg [31:0] diag_fault_count, diag_retry_count,
    output reg [31:0] diag_expected_id, diag_result_id
);
    localparam IDLE=0, SNAPSHOT=1, LOAD=2, START=3, WAIT_CORE=4;
    reg [2:0] state;
    reg [31:0] tick_count, accepted_id;
    reg [7:0] load_index;
    // Freeze the complete frame, then consume its low word each LOAD cycle.
    // Fixed adjacent-word shifts avoid a 168-to-1, 24-bit dynamic read mux.
    reg [4031:0] snapshot;
    wire run_ok = enable && sensor_ok && fault==0;
    wire core_rst_n = rst_n && run_ok;
    wire core_done, core_busy, core_sat;
    wire [4:0] core_errors;
    wire [31:0] result_id, cycles;
    wire signed [23:0] a0,a1,a2,a3,a4,a5;
    wire [7:0] core_addr = load_index>=28 ? load_index-8'd28 : load_index;
    fc_v4_top #(.WEIGHT_BITS(WEIGHT_BITS),.WEIGHTS_FILE(WEIGHTS_FILE),.BIASES_FILE(BIASES_FILE),.ELU_FILE(ELU_FILE)) core(
        .clk(clk),.rst_n(core_rst_n),.wr_en(state==LOAD && run_ok),
        .wr_history(load_index>=28),.wr_addr(core_addr),
        .wr_data(snapshot[23:0]),.start(state==START && run_ok),
        .frame_id(accepted_id),.busy(core_busy),.done(core_done),
        .error_flags(core_errors),.saturated(core_sat),.result_frame_id(result_id),.cycles(cycles),
        .action0(a0),.action1(a1),.action2(a2),.action3(a3),.action4(a4),.action5(a5),
        .dbg_valid(),.dbg_layer(),.dbg_index(),.dbg_data());
    // Retain evidence through automatic core reset/retry and sensor recalibration.
    // Detail bits: [4:0]=core error_flags, [5]=saturated, [6]=frame ID mismatch.
    wire deadline_missed = tick_count==PERIOD_CLKS-1 && state!=IDLE;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            diag_last_fault<=0;diag_core_detail<=0;
            diag_fault_count<=0;diag_retry_count<=0;
            diag_expected_id<=0;diag_result_id<=0;
        end else if(enable) begin
            if(!sensor_ok) begin
                if(!fault[0]) begin
                    diag_last_fault<=3'b001;
                    diag_fault_count<=diag_fault_count+1'b1;
                end
            end else if(fault!=0) begin
                if(RETRY_ON_FAULT) diag_retry_count<=diag_retry_count+1'b1;
            end else if(deadline_missed) begin
                diag_last_fault<=3'b010;
                diag_fault_count<=diag_fault_count+1'b1;
            end else if(state==WAIT_CORE && core_done &&
                        (core_errors!=0 || core_sat || result_id!=accepted_id)) begin
                diag_last_fault<=3'b100;
                diag_core_detail<={(result_id!=accepted_id),core_sat,core_errors};
                diag_expected_id<=accepted_id;diag_result_id<=result_id;
                diag_fault_count<=diag_fault_count+1'b1;
            end
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state<=IDLE; tick_count<=0; accepted_id<=0; load_index<=0; snapshot<=0;
            capture<=0; action_valid<=0; actions<=0; completed_frames<=0;
            inference_cycles<=0; fault<=0;
        end else begin
            capture<=0; action_valid<=0;
            if (!enable) begin
                state<=IDLE; tick_count<=0; accepted_id<=0; load_index<=0;
                actions<=0; completed_frames<=0; inference_cycles<=0; fault<=0;
            end else if (!sensor_ok || fault!=0) begin
                state<=IDLE; tick_count<=0; actions<=0;
                if (!sensor_ok) fault[0]<=1;
                else if(RETRY_ON_FAULT) fault<=0; // Core was reset while fault!=0; retry after recovery.
            end else begin
                if (tick_count==PERIOD_CLKS-1) begin
                    tick_count<=0;
                    if (state!=IDLE) begin
                        fault[1]<=1; actions<=0; state<=IDLE; // Missed deadline.
                    end else begin
                        accepted_id<=accepted_id+1;
                        capture<=1; state<=SNAPSHOT;
                    end
                end else tick_count<=tick_count+1;
                // Deadline failure takes precedence over a same-edge completion.
                if (!(tick_count==PERIOD_CLKS-1 && state!=IDLE)) begin
                    case (state)
                        SNAPSHOT: if (frame_valid) begin
                            snapshot<={history_flat,current_flat}; load_index<=0; state<=LOAD;
                        end
                        LOAD: begin
                            snapshot<={24'd0,snapshot[4031:24]};
                            if (load_index==167) state<=START;
                            else load_index<=load_index+1'b1;
                        end
                        START: state<=WAIT_CORE;
                        WAIT_CORE: if (core_done) begin
                            if (core_errors!=0 || core_sat || result_id!=accepted_id) begin
                                fault[2]<=1; actions<=0;
                            end else begin
                                actions<={a5,a4,a3,a2,a1,a0}; action_valid<=1;
                                completed_frames<=completed_frames+1;
                                inference_cycles<=cycles;
                            end
                            state<=IDLE;
                        end
                    endcase
                end
            end
        end
    end
endmodule

`endif
