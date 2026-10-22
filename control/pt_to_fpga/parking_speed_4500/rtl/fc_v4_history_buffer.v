`timescale 1ns/1ps

// Five observations, oldest first. First commit repeats the observation five
// times; subsequent commits discard the oldest 28 words and append the newest.
module fc_v4_history_buffer (
    input wire clk,
    input wire rst_n,
    input wire obs_wr_en,
    input wire [4:0] obs_wr_addr,
    input wire signed [23:0] obs_wr_data,
    input wire commit,
    input wire [7:0] rd_addr,
    output reg signed [23:0] rd_data,
    output wire rd_valid,
    output reg busy,
    output reg done,
    output reg initialized,
    output reg [3:0] error_flags
);
    reg signed [23:0] observation [0:27];
    reg signed [23:0] history [0:139];
    reg [27:0] obs_valid;
    reg [7:0] index;
    reg [4:0] obs_index;
    reg read_phase;
    reg signed [23:0] move_q;
    reg rd_valid_q;
    // Valid accompanies the registered word, never a live address.
    assign rd_valid = rd_valid_q && !busy;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) rd_valid_q <= 0;
        else rd_valid_q <= initialized && !busy && !commit && rd_addr < 140;
    end

    always @(posedge clk) begin
        if (rd_addr < 140) rd_data <= history[rd_addr];
        else rd_data <= 0;
        if (obs_wr_en && !busy && obs_wr_addr < 28) observation[obs_wr_addr] <= obs_wr_data;
        if (busy && !read_phase) begin
            if (!initialized || index >= 112) move_q <= observation[obs_index];
            else move_q <= history[index + 8'd28];
        end
        if (busy && read_phase) history[index] <= move_q;
    end

    // Flags: busy-write, busy-commit, invalid-address, incomplete-observation.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy <= 0; done <= 0; initialized <= 0; error_flags <= 0;
            obs_valid <= 0; index <= 0; obs_index <= 0; read_phase <= 0;
        end else begin
            done <= 0;
            if (obs_wr_en) begin
                if (busy) error_flags[0] <= 1;
                else if (obs_wr_addr >= 28) error_flags[2] <= 1;
                else obs_valid[obs_wr_addr] <= 1;
            end
            if (commit) begin
                if (busy || obs_wr_en) error_flags[1] <= 1;
                else if (!(&obs_valid)) error_flags[3] <= 1;
                else begin
                    busy <= 1; obs_valid <= 0;
                    index <= 0; obs_index <= 0; read_phase <= 0;
                end
            end
            if (busy) begin
                read_phase <= !read_phase;
                if (read_phase) begin
                    if (index == 139) begin busy <= 0; done <= 1; initialized <= 1; end
                    else begin
                        index <= index + 1'b1;
                        if (!initialized || index >= 112) begin
                            if (obs_index == 27) obs_index <= 0;
                            else obs_index <= obs_index + 1'b1;
                        end
                    end
                end
            end
        end
    end
endmodule
