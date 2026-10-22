`timescale 1ns/1ps
// The checkpoint digest is an identifier, not a source/bitstream security binding.
module board_rl_bridge #(
    parameter integer CLK_HZ=50000000,
    parameter integer BAUD=115200,
    parameter integer CLKS_PER_BIT=CLK_HZ/BAUD,
    parameter integer RX_TIMEOUT_CLKS=CLK_HZ/50,
    parameter WEIGHTS_FILE="/home/cyd/fpga/rl_ws2/vehicle_integration/assets/weights.mem",
    parameter BIASES_FILE="/home/cyd/fpga/rl_ws2/vehicle_integration/assets/bias.mem",
    parameter ELU_FILE="/home/cyd/fpga/rl_ws2/vehicle_integration/assets/elu.mem"
)(input wire clk, input wire rst_n, input wire UART_RXD, output wire UART_TXD, input wire [511:0] sensor_snapshot,
    input wire shadow_snapshot_usable,
    input wire [31:0] shadow_snapshot_sequence,
    input wire [31:0] shadow_snapshot_age_ticks,
    input wire signed [27*24-1:0] shadow_current_flat,
    input wire signed [135*24-1:0] shadow_history_flat);
    localparam [255:0] DIGEST=256'h29b12e353b84380fa57ed7c3f8928fb1d51e9df99c0fa01bd3f4aec2de5a8c03;
    function [15:0] crc_byte;
        input [15:0] crc; input [7:0] value;
        reg [15:0] c; integer b;
        begin
            c=crc ^ {value,8'd0};
            for(b=0;b<8;b=b+1) c=c[15] ? (c<<1)^16'h1021 : (c<<1);
            crc_byte=c;
        end
    endfunction
    wire [7:0] rx_data;
    wire rx_valid, rx_error, tx_ready, tx_busy;
    reg [7:0] tx_data;
    wire tx_valid;
    board_uart_rx #(.CLKS_PER_BIT(CLKS_PER_BIT)) rx(
        .clk(clk),.rst_n(rst_n),.rxd(UART_RXD),.data(rx_data),.valid(rx_valid),.framing_error(rx_error));
    board_uart_tx #(.CLKS_PER_BIT(CLKS_PER_BIT)) tx(
        .clk(clk),.rst_n(rst_n),.data(tx_data),.valid(tx_valid),.ready(tx_ready),.txd(UART_TXD),.busy(tx_busy));

    localparam SEEK_A5=0, SEEK_5A=1, HEADER=2, PAYLOAD=3, CRC_LO=4, CRC_HI=5;
    reg [2:0] rx_state;
    reg [3:0] header_index;
    reg [7:0] version, packet_type, crc_low;
    reg [15:0] payload_len, payload_index, rx_crc;
    reg [31:0] packet_id, timeout_count;
    reg packet_blocked;
    // This staging RAM is never written by packets that began while reserved.
    reg [7:0] request_mem [0:485];
    localparam J_IDLE=0, J_LOAD=1, J_START=2, J_RUN=3;
    reg [1:0] job;
    reg [7:0] load_index;
    reg [31:0] accepted_id, last_id;
    reg last_id_valid;
    wire core_busy, core_done, core_sat, dbg_valid;
    wire [4:0] core_errors;
    wire [31:0] core_cycles, core_result_id;
    wire signed [23:0] a0,a1,a2,a3,a4,a5,dbg_data;
    wire [2:0] dbg_layer;
    wire [7:0] dbg_index;
    wire uart_core_wr=(job==J_LOAD);
    wire [7:0] uart_core_addr=(load_index>=8'd27) ? load_index-8'd27 : load_index;
    wire signed [23:0] uart_core_data={request_mem[load_index*3+2],request_mem[load_index*3+1],request_mem[load_index*3]};
    wire shadow_core_owned;
    wire shadow_core_wr, shadow_core_wr_history, shadow_core_start;
    wire [7:0] shadow_core_addr;
    wire signed [23:0] shadow_core_data;
    wire [31:0] shadow_core_frame_id;
    wire signed [23:0] shadow_a0,shadow_a1,shadow_a2,shadow_a3,shadow_a4,shadow_a5;
    wire shadow_valid, shadow_deadline_fault, shadow_snapshot_fault;
    wire [31:0] shadow_inference_count, shadow_deadline_fault_count;
    wire shadow_start_grant=(job==J_IDLE) && !normal_pending && !response_active &&
        !error_pending && !tx_busy && (rx_state==SEEK_A5) && !rx_valid && !rx_error &&
        !shadow_core_owned;
    wire core_wr=shadow_core_owned ? shadow_core_wr : uart_core_wr;
    wire core_wr_history=shadow_core_owned ? shadow_core_wr_history : (load_index>=8'd27);
    wire [7:0] core_addr=shadow_core_owned ? shadow_core_addr : uart_core_addr;
    wire signed [23:0] core_data=shadow_core_owned ? shadow_core_data : uart_core_data;
    wire core_start=shadow_core_owned ? shadow_core_start : (job==J_START);
    wire [31:0] core_frame_id=shadow_core_owned ? shadow_core_frame_id : accepted_id;
    shadow_scheduler scheduler(
        .clk(clk),.rst_n(rst_n),.snapshot_usable(shadow_snapshot_usable),
        .start_grant(shadow_start_grant),.snapshot_sequence(shadow_snapshot_sequence),
        .snapshot_age_ticks(shadow_snapshot_age_ticks),.current_flat(shadow_current_flat),
        .history_flat(shadow_history_flat),.core_busy(core_busy),.core_done(core_done),
        .core_errors(core_errors),.core_saturated(core_sat),.core_result_frame_id(core_result_id),
        .core_cycles(core_cycles),.core_action0(a0),.core_action1(a1),.core_action2(a2),
        .core_action3(a3),.core_action4(a4),.core_action5(a5),.core_owned(shadow_core_owned),
        .core_wr(shadow_core_wr),.core_wr_history(shadow_core_wr_history),.core_wr_addr(shadow_core_addr),
        .core_wr_data(shadow_core_data),.core_start(shadow_core_start),.core_frame_id(shadow_core_frame_id),
        .shadow_action0(shadow_a0),.shadow_action1(shadow_a1),.shadow_action2(shadow_a2),
        .shadow_action3(shadow_a3),.shadow_action4(shadow_a4),.shadow_action5(shadow_a5),
        .shadow_valid(shadow_valid),.deadline_fault(shadow_deadline_fault),
        .snapshot_fault(shadow_snapshot_fault),.inference_count(shadow_inference_count),
        .deadline_fault_count(shadow_deadline_fault_count));
    fc_v4_top #(.WEIGHTS_FILE(WEIGHTS_FILE),.BIASES_FILE(BIASES_FILE),.ELU_FILE(ELU_FILE)) core(
        .clk(clk),.rst_n(rst_n),.wr_en(core_wr),.wr_history(core_wr_history),
        .wr_addr(core_addr),.wr_data(core_data),.start(core_start),.frame_id(core_frame_id),
        .busy(core_busy),.done(core_done),.error_flags(core_errors),.saturated(core_sat),
        .result_frame_id(core_result_id),.cycles(core_cycles),
        .action0(a0),.action1(a1),.action2(a2),.action3(a3),.action4(a4),.action5(a5),
        .dbg_valid(dbg_valid),.dbg_layer(dbg_layer),.dbg_index(dbg_index),.dbg_data(dbg_data));
    reg [23:0] layers [0:424];
    reg [23:0] latent [0:2];
    reg [143:0] actions;
    reg [31:0] saved_cycles;
    reg [7:0] saved_errors, saved_sat;
    reg [8:0] debug_count;
    reg debug_bad;
    reg [2:0] expected_layer;
    reg [7:0] expected_index;
    always @* begin
        if(debug_count<128) begin expected_layer=0; expected_index=debug_count[7:0]; end
        else if(debug_count<192) begin expected_layer=1; expected_index=debug_count[7:0]-8'd128; end
        else if(debug_count<195) begin expected_layer=2; expected_index=debug_count[7:0]-8'd192; end
        else if(debug_count<323) begin expected_layer=3; expected_index=debug_count[7:0]-8'd195; end
        else if(debug_count<387) begin expected_layer=4; expected_index=debug_count[7:0]-8'd67; end
        else if(debug_count<419) begin expected_layer=5; expected_index=debug_count[7:0]-8'd131; end
        else begin expected_layer=6; expected_index=debug_count[7:0]-8'd163; end
    end

    reg [511:0] saved_sensors;
    reg normal_pending;
    reg [7:0] normal_type;
    reg [31:0] normal_id;
    // One deferred error, first-error-wins. Further errors while occupied are
    // counted/dropped in dropped_errors, never inserted into an active reply.
    reg error_pending;
    reg [7:0] queued_error;
    reg [31:0] queued_id, dropped_errors;
    // Blocking temporaries arbitrate all error producers in this clocked block.
    // Core completion is evaluated first and therefore wins a same-edge clash.
    reg error_claimed;
    reg [31:0] drop_next;
    reg response_active, response_drain;
    reg [7:0] response_type, response_error;
    reg [31:0] response_id;
    reg [15:0] response_len, tx_index, tx_crc;
    wire reserved=(job!=J_IDLE)||normal_pending||response_active||error_pending||tx_busy;
    assign tx_valid=response_active && !response_drain;
    reg [10:0] p;
    reg [8:0] tx_word_index;
    reg [1:0] tx_byte_index;
    reg [23:0] payload_word;
    always @* begin
        tx_data=0; p=0; payload_word=0;
        case(tx_index)
            0: tx_data=8'ha5;
            1: tx_data=8'h5a;
            2: tx_data=1;
            3: tx_data=response_type;
            4: tx_data=response_len[7:0];
            5: tx_data=response_len[15:8];
            6: tx_data=response_id[7:0];
            7: tx_data=response_id[15:8];
            8: tx_data=response_id[23:16];
            9: tx_data=response_id[31:24];
            default: begin
                p=tx_index[10:0]-11'd10;
                if(p==response_len) tx_data=tx_crc[7:0];
                else if(p==response_len+1) tx_data=tx_crc[15:8];
                else if(response_type==8'hff) tx_data=response_error;
                else if(response_type==8'h83) tx_data=saved_sensors[p*8 +: 8];
                else if(p<32) tx_data=DIGEST[255-p*8 -: 8];
                else if(p<36) tx_data=saved_cycles[(p-32)*8 +: 8];
                else if(p==36) tx_data=saved_errors;
                else if(p==37) tx_data=saved_sat;
                else begin
                    if(p<56) payload_word=actions[tx_word_index*24 +: 24];
                    else if(p<65) payload_word=latent[tx_word_index];
                    else if(tx_word_index<425) payload_word=layers[tx_word_index];
                    tx_data=payload_word[tx_byte_index*8 +: 8];
                end
            end
        endcase
    end
    task enqueue_error;
        input [7:0] code; input [31:0] id;
        begin
            if(!error_claimed) begin
                error_claimed=1;
                error_pending<=1; queued_error<=code; queued_id<=id;
            end else begin
                drop_next=drop_next+1'b1;
                dropped_errors<=drop_next;
            end
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            rx_state<=SEEK_A5; header_index<=0; version<=0; packet_type<=0;
            payload_len<=0; payload_index<=0; rx_crc<=16'hffff; crc_low<=0;
            packet_id<=0; timeout_count<=0; packet_blocked<=0;
            job<=J_IDLE; load_index<=0; accepted_id<=0; last_id<=0; last_id_valid<=0;
            debug_count<=0; debug_bad<=0;
            actions<=0; saved_cycles<=0; saved_errors<=0; saved_sat<=0;
            saved_sensors<=0; normal_pending<=0; normal_type<=0; normal_id<=0;
            error_pending<=0; queued_error<=0; queued_id<=0; dropped_errors<=0;
            response_active<=0; response_drain<=0; response_type<=0; response_error<=0;
            response_id<=0; response_len<=0; tx_index<=0; tx_crc<=16'hffff;
            tx_word_index<=0; tx_byte_index<=0;
        end else begin
            error_claimed=error_pending;
            drop_next=dropped_errors;
            // Serializer owns frozen results until the UART drains its stop bit.
            if(response_active) begin
                if(response_drain) begin
                    if(tx_ready) begin response_active<=0; response_drain<=0; end
                end else if(tx_ready) begin
                    // Advance only on an accepted UART byte. Reset at the
                    // action/latent/layer boundaries; no combinational /3.
                    if(tx_index==47 || tx_index==65 || tx_index==74) begin
                        tx_word_index<=0; tx_byte_index<=0;
                    end else if(tx_index>=48 && tx_index<1350) begin
                        if(tx_byte_index==2) begin
                            tx_byte_index<=0; tx_word_index<=tx_word_index+1'b1;
                        end else tx_byte_index<=tx_byte_index+1'b1;
                    end
                    if(tx_index>=2 && tx_index<response_len+10) tx_crc<=crc_byte(tx_crc,tx_data);
                    if(tx_index==response_len+11) response_drain<=1;
                    else tx_index<=tx_index+1'b1;
                end
            end else if(!tx_busy && job==J_IDLE) begin
                if(normal_pending) begin
                    normal_pending<=0; response_active<=1; response_type<=normal_type;
                    response_id<=normal_id; response_len<=(normal_type==8'h81)?1340:((normal_type==8'h83)?64:32);
                    response_error<=0; tx_index<=0; tx_crc<=16'hffff;
                end else if(error_pending) begin
                    error_pending<=0; response_active<=1; response_type<=8'hff;
                    response_id<=queued_id; response_len<=1; response_error<=queued_error;
                    tx_index<=0; tx_crc<=16'hffff;
                end
            end
            case(job)
                J_LOAD: if(load_index==161) job<=J_START; else load_index<=load_index+1'b1;
                J_START: job<=J_RUN;
                J_RUN: begin
                    if(dbg_valid) begin
                        if(debug_count<425) layers[debug_count]<=dbg_data;
                        if(dbg_layer==2 && dbg_index<3) latent[dbg_index]<=dbg_data;
                        if(debug_count>=425 || dbg_layer!=expected_layer || dbg_index!=expected_index) debug_bad<=1;
                        debug_count<=debug_count+1'b1;
                    end
                    if(core_done) begin
                        job<=J_IDLE;
                        if(core_errors[3:0]!=0 || debug_bad || debug_count!=425 || core_result_id!=accepted_id)
                            enqueue_error(8,accepted_id);
                        else begin
                            actions<={a5,a4,a3,a2,a1,a0}; saved_cycles<=core_cycles;
                            saved_errors<={3'd0,core_errors}; saved_sat<={7'd0,core_sat};
                            normal_pending<=1; normal_type<=8'h81; normal_id<=accepted_id;
                        end
                    end
                end
                default: begin end
            endcase
            // Keep collecting/checksumming even busy packets, without staging
            // writes. A frame that STARTED while reserved can never be accepted.
            if(rx_error) begin
                enqueue_error(5,packet_id); rx_state<=SEEK_A5; timeout_count<=0;
            end else if(rx_valid) begin
                timeout_count<=0;
                case(rx_state)
                    SEEK_A5: if(rx_data==8'ha5) begin
                        rx_state<=SEEK_5A; packet_blocked<=reserved; packet_id<=0;
                    end
                    SEEK_5A: if(rx_data==8'h5a) begin
                        rx_state<=HEADER; header_index<=0; rx_crc<=16'hffff;
                        payload_len<=0; packet_id<=0;
                    end else if(rx_data==8'ha5) packet_blocked<=reserved;
                    else rx_state<=SEEK_A5;
                    HEADER: begin
                        rx_crc<=crc_byte(rx_crc,rx_data);
                        case(header_index)
                            0: version<=rx_data;
                            1: packet_type<=rx_data;
                            2: payload_len[7:0]<=rx_data;
                            3: payload_len[15:8]<=rx_data;
                            4: packet_id[7:0]<=rx_data;
                            5: packet_id[15:8]<=rx_data;
                            6: packet_id[23:16]<=rx_data;
                            7: begin
                                packet_id[31:24]<=rx_data; payload_index<=0;
                                // Reject impossible headers immediately. Never
                                // let an untrusted length pin payload collection.
                                if(version!=1) begin
                                    enqueue_error(1,{rx_data,packet_id[23:0]}); rx_state<=SEEK_A5;
                                end else if(packet_type!=1 && packet_type!=2 && packet_type!=3) begin
                                    enqueue_error(2,{rx_data,packet_id[23:0]}); rx_state<=SEEK_A5;
                                end else if((packet_type==1 && payload_len!=486)||((packet_type==2 || packet_type==3) && payload_len!=0)) begin
                                    enqueue_error(3,{rx_data,packet_id[23:0]}); rx_state<=SEEK_A5;
                                end else rx_state<=(payload_len==0)?CRC_LO:PAYLOAD;
                            end
                        endcase
                        header_index<=header_index+1'b1;
                    end
                    PAYLOAD: begin
                        rx_crc<=crc_byte(rx_crc,rx_data);
                        if(!packet_blocked && !reserved && payload_index<486) request_mem[payload_index]<=rx_data;
                        if(payload_index==payload_len-1) rx_state<=CRC_LO;
                        else payload_index<=payload_index+1'b1;
                    end
                    CRC_LO: begin crc_low<=rx_data; rx_state<=CRC_HI; end
                    CRC_HI: begin
                        rx_state<=SEEK_A5;
                        if(version!=1) enqueue_error(1,packet_id);
                        else if(packet_type!=1 && packet_type!=2 && packet_type!=3) enqueue_error(2,packet_id);
                        else if((packet_type==1 && payload_len!=486)||((packet_type==2 || packet_type==3) && payload_len!=0)) enqueue_error(3,packet_id);
                        else if({rx_data,crc_low}!=rx_crc) enqueue_error(4,packet_id);
                        else if(packet_blocked || reserved) enqueue_error(6,packet_id);
                        else if(packet_type==1 && last_id_valid && packet_id==last_id) enqueue_error(7,packet_id);
                        else if(packet_type==3) begin saved_sensors<=sensor_snapshot; normal_pending<=1; normal_type<=8'h83; normal_id<=packet_id; end
                        else if(packet_type==2) begin normal_pending<=1; normal_type<=8'h82; normal_id<=packet_id; end
                        else begin
                            accepted_id<=packet_id; last_id<=packet_id; last_id_valid<=1;
                            job<=J_LOAD; load_index<=0; debug_count<=0; debug_bad<=0;
                        end
                    end
                    default: rx_state<=SEEK_A5;
                endcase
            end else if(rx_state!=SEEK_A5) begin
                if(timeout_count>=RX_TIMEOUT_CLKS-1) begin
                    enqueue_error(5,packet_id); rx_state<=SEEK_A5; timeout_count<=0;
                end else timeout_count<=timeout_count+1'b1;
            end
        end
    end
endmodule
