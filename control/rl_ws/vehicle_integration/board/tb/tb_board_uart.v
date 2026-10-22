`timescale 1ns/1ps
module tb_board_uart;
    localparam D=50000000/115200;
    reg clk=0; always #10 clk=~clk;
    reg rst_n=0, rxd=1, valid=0;
    reg [7:0] data=0;
    wire ready, txd, busy, rx_valid, rx_error;
    wire [7:0] rx_data;
    board_uart_tx tx(.clk(clk),.rst_n(rst_n),.data(data),.valid(valid),.ready(ready),.txd(txd),.busy(busy));
    board_uart_rx rx(.clk(clk),.rst_n(rst_n),.rxd(rxd),.data(rx_data),.valid(rx_valid),.framing_error(rx_error));
    integer rx_count=0, error_count=0, clock_count=0, last_edge, edges=0;
    reg [7:0] seen[0:7];
    reg monitor=0;
    always @(posedge clk) begin
        clock_count=clock_count+1;
        #1;
        if(rx_valid) begin seen[rx_count]=rx_data; rx_count=rx_count+1; end
        if(rx_error) error_count=error_count+1;
        if(rst_n && ready!==!busy) $fatal(1,"TX ready/busy handshake mismatch");
    end
    always @(txd) if(monitor) begin
        if(edges>0 && clock_count-last_edge!=D) $fatal(1,"default baud bit divider got=%0d expected=%0d",clock_count-last_edge,D);
        edges=edges+1; last_edge=clock_count;
    end
    task send;
        input [7:0] value; input bad_stop;
        integer i;
        begin
            @(negedge clk); rxd=0; repeat(D) @(negedge clk);
            for(i=0;i<8;i=i+1) begin rxd=value[i]; repeat(D) @(negedge clk); end
            rxd=!bad_stop; repeat(D) @(negedge clk); rxd=1;
        end
    endtask
    integer begin_cycle;
    initial begin
        if(D!=434) $fatal(1,"unexpected nominal divisor");
        repeat(5) @(negedge clk); rst_n=1; repeat(5) @(negedge clk);
        // 0x55 alternates every bit, including start and stop.
        monitor=1; data=8'h55; valid=1;
        @(posedge clk); #2; begin_cycle=clock_count;
        if(!busy || ready || txd!==0) $fatal(1,"TX first handshake");
        @(negedge clk); valid=0;
        // Attempt a second byte during busy must not be accepted.
        repeat(100) @(negedge clk); data=8'hff; valid=1;
        repeat(20) @(negedge clk); valid=0;
        wait(ready); #2;
        if(clock_count-begin_cycle!=10*D || edges!=10 || txd!==1) $fatal(1,"TX stop duration/bit count mismatch elapsed=%0d edges=%0d",clock_count-begin_cycle,edges);
        monitor=0;
        @(negedge clk); data=8'ha6; valid=1; @(negedge clk); valid=0;
        wait(ready);
        // A short low glitch cannot pass the half-bit start qualification.
        @(negedge clk); rxd=0; repeat(20) @(negedge clk); rxd=1;
        repeat(D) @(negedge clk);
        if(rx_count || error_count) $fatal(1,"false start accepted at default baud");
        send(8'h00,0); send(8'hff,0); send(8'ha5,0); send(8'h5a,0);
        repeat(D) @(negedge clk);
        if(rx_count!=4 || seen[0]!==8'h00 || seen[1]!==8'hff || seen[2]!==8'ha5 || seen[3]!==8'h5a) $fatal(1,"default baud back-to-back RX failed");
        send(8'h33,1); repeat(2*D) @(negedge clk);
        if(error_count!=1 || rx_count!=4) $fatal(1,"framing error not isolated");
        send(8'h96,0); repeat(D) @(negedge clk);
        if(rx_count!=5 || seen[4]!==8'h96) $fatal(1,"framing recovery failed");
        @(negedge clk); rst_n=0; repeat(2) @(negedge clk);
        if(txd!==1 || busy || !ready || rx_valid || rx_error) $fatal(1,"UART reset not idle");
        $display("PASS BOARD_UART clk50MHz baud115200 divisor434 actual_baud=115207 bit_clocks434 stop_clocks434 handshake/false-start/framing/backtoback/reset");
        $finish;
    end
    initial begin #3000000; $fatal(1,"UART test timeout"); end
endmodule
