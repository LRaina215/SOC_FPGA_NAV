`timescale 1ns/1ps
// 125 Hz shadow-loop scheduler. It owns one network-core input producer and
// never drives an external actuator. Results are atomically copied to shadow
// action registers after a successful deadline-bounded inference.
module shadow_scheduler #(
    parameter integer CLK_HZ=50000000,
    parameter integer CONTROL_HZ=125,
    parameter integer CORE_DEADLINE_CYCLES=400000
)(
    input wire clk,
    input wire rst_n,
    input wire snapshot_usable,
    input wire start_grant,
    input wire [31:0] snapshot_sequence,
    input wire [31:0] snapshot_age_ticks,
    input wire signed [27*24-1:0] current_flat,
    input wire signed [135*24-1:0] history_flat,
    input wire core_busy,
    input wire core_done,
    input wire [4:0] core_errors,
    input wire core_saturated,
    input wire [31:0] core_result_frame_id,
    input wire [31:0] core_cycles,
    input wire signed [23:0] core_action0,
    input wire signed [23:0] core_action1,
    input wire signed [23:0] core_action2,
    input wire signed [23:0] core_action3,
    input wire signed [23:0] core_action4,
    input wire signed [23:0] core_action5,
    output reg core_owned,
    output reg core_wr,
    output reg core_wr_history,
    output reg [7:0] core_wr_addr,
    output reg signed [23:0] core_wr_data,
    output reg core_start,
    output reg [31:0] core_frame_id,
    output reg signed [23:0] shadow_action0,
    output reg signed [23:0] shadow_action1,
    output reg signed [23:0] shadow_action2,
    output reg signed [23:0] shadow_action3,
    output reg signed [23:0] shadow_action4,
    output reg signed [23:0] shadow_action5,
    output reg shadow_valid,
    output reg deadline_fault,
    output reg snapshot_fault,
    output reg [31:0] inference_count,
    output reg [31:0] deadline_fault_count
);
    localparam integer TICK_CYCLES=CLK_HZ/CONTROL_HZ;
    localparam S_WAIT=0, S_LOAD=1, S_START=2, S_RUN=3;
    reg [2:0] state;
    reg [31:0] tick_count, deadline_count;
    reg [7:0] load_index;
    reg [31:0] active_frame_id;
    reg [31:0] last_snapshot_sequence;
    reg [31:0] active_snapshot_age_ticks;
    reg timed_out;
    reg signed [27*24-1:0] current_latched;
    reg signed [135*24-1:0] history_latched;

    always @* begin
        core_owned = (state != S_WAIT);
        core_wr = (state == S_LOAD);
        core_wr_history = core_wr && (load_index >= 8'd27);
        core_wr_addr = (load_index >= 8'd27) ? load_index - 8'd27 : load_index;
        core_wr_data = 0;
        if (load_index < 8'd27)
            core_wr_data = current_latched[load_index*24 +: 24];
        else if (load_index < 8'd162)
            core_wr_data = history_latched[(load_index-8'd27)*24 +: 24];
        core_start = (state == S_START);
        core_frame_id = active_frame_id;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state<=S_WAIT;
            tick_count<=0;
            deadline_count<=0;
            load_index<=0;
            active_frame_id<=0;
            last_snapshot_sequence<=0;
            active_snapshot_age_ticks<=0;
            timed_out<=0;
            current_latched<=0;
            history_latched<=0;
            shadow_action0<=0; shadow_action1<=0; shadow_action2<=0;
            shadow_action3<=0; shadow_action4<=0; shadow_action5<=0;
            shadow_valid<=0;
            deadline_fault<=0;
            snapshot_fault<=0;
            inference_count<=0;
            deadline_fault_count<=0;
        end else begin
            if (tick_count == TICK_CYCLES-1) begin
                tick_count <= 0;
                if (state == S_WAIT) begin
                    shadow_valid <= 0;
                    if (!snapshot_usable) begin
                        snapshot_fault <= 1;
                    end else if (snapshot_sequence == last_snapshot_sequence) begin
                        // A retained sample is not silently presented as a new
                        // control frame. The retained age remains diagnostic.
                        snapshot_fault <= 1;
                    end else if (start_grant && !core_busy) begin
                        current_latched <= current_flat;
                        history_latched <= history_flat;
                        active_frame_id <= snapshot_sequence;
                        active_snapshot_age_ticks <= snapshot_age_ticks;
                        last_snapshot_sequence <= snapshot_sequence;
                        load_index <= 0;
                        state <= S_LOAD;
                        snapshot_fault <= 0;
                    end
                end
            end else begin
                tick_count <= tick_count + 1'b1;
            end

            if (state == S_LOAD) begin
                if (load_index == 8'd161) state <= S_START;
                else load_index <= load_index + 1'b1;
            end else if (state == S_START) begin
                state <= S_RUN;
                deadline_count <= 0;
                timed_out <= 0;
            end else if (state == S_RUN) begin
                if (core_done) begin
                    if (!timed_out && core_errors == 0 && !core_saturated &&
                        core_result_frame_id == active_frame_id &&
                        core_cycles <= CORE_DEADLINE_CYCLES) begin
                        shadow_action0<=core_action0;
                        shadow_action1<=core_action1;
                        shadow_action2<=core_action2;
                        shadow_action3<=core_action3;
                        shadow_action4<=core_action4;
                        shadow_action5<=core_action5;
                        shadow_valid<=1;
                        inference_count<=inference_count+1'b1;
                        deadline_fault<=0;
                    end else begin
                        shadow_valid<=0;
                        deadline_fault<=1;
                        deadline_fault_count<=deadline_fault_count+1'b1;
                    end
                    state<=S_WAIT;
                end else if (deadline_count >= CORE_DEADLINE_CYCLES) begin
                    // Do not release ownership before the core's eventual done
                    // pulse; releasing early could let another producer corrupt
                    // the in-flight frame.
                    timed_out<=1;
                    deadline_fault<=1;
                end else begin
                    deadline_count<=deadline_count+1'b1;
                end
            end
        end
    end
endmodule
