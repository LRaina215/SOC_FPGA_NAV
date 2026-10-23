`timescale 1ns/1ps
// Tests top-level reset/config/UART wiring with a behavioral MAC stub below.
// No vendor/analog behavior is claimed by this test.
module tb_sfp_control;
    reg clk=0, reset_n=0, uart_rxd=1, los=0;
    always #10 clk=~clk;
    wire uart_txd, tx_dis;
    wire [9:0] actuators;
    sfp_bringup_top dut (
        .clk(clk), .reset_n(reset_n), .uart_rxd(uart_rxd), .uart_txd(uart_txd),
        .sfp1_los(los), .sfp1_tx_dis(tx_dis),
        .M1_DIR1(actuators[0]), .M1_DIR2(actuators[1]), .M1_PWM(actuators[2]),
        .M2_DIR1(actuators[3]), .M2_DIR2(actuators[4]), .M2_PWM(actuators[5]),
        .SERVO1(actuators[6]), .SERVO2(actuators[7]), .SERVO3(actuators[8]), .SERVO4(actuators[9])
    );
    localparam integer BIT_NS=1080; // UART DIV=50MHz/921600=54
    byte responses[$];
    initial forever begin
        byte b;
        @(negedge uart_txd);
        #(BIT_NS+BIT_NS/2);
        for(integer i=0;i<8;i=i+1) begin b[i]=uart_txd; #(BIT_NS); end
        if (uart_txd!==1'b1) $fatal(1,"UART stop bit");
        responses.push_back(b);
    end
    task send_byte(input byte b);
        uart_rxd=0; #(BIT_NS);
        for(integer i=0;i<8;i=i+1) begin uart_rxd=b[i]; #(BIT_NS); end
        uart_rxd=1; #(BIT_NS);
    endtask
    task get_byte(output byte b);
        wait(responses.size()!=0); b=responses.pop_front();
    endtask
    task read_reg(input bit[15:0] addr, output bit[31:0] value);
        byte b;
        send_byte("R"); send_byte(addr[15:8]); send_byte(addr[7:0]);
        get_byte(b); if(b!="D") $fatal(1,"read reply header");
        for(integer i=3;i>=0;i=i-1) begin get_byte(b); value[8*i+:8]=b; end
    endtask
    task write_reg(input bit[15:0] addr,input bit[31:0] value);
        byte b;
        send_byte("W"); send_byte(addr[15:8]); send_byte(addr[7:0]);
        for(integer i=3;i>=0;i=i-1) send_byte(value[8*i+:8]);
        get_byte(b); if(b!="K") $fatal(1,"write reply header");
    endtask
    always @(posedge clk) if (actuators!==10'b0) $fatal(1,"Actuator output is not low");
    initial begin
        bit [31:0] v;
        repeat(10) @(negedge clk); reset_n=1;
        #3000000;
        read_reg(0,v); if(v!==32'h53465031) $fatal(1,"signature");
        read_reg(4,v); if(v!==2) $fatal(1,"default BASE-X/AN");
        read_reg(16,v); if(v!==23) $fatal(1,"default stable link flags: %x",v);
        read_reg(64,v); if(v!==32'hc0a80264) $fatal(1,"IP address");
        read_reg(68,v); if(v!==9000) $fatal(1,"UDP port");
        write_reg(4,7); // SGMII/AN, honour LOS
        read_reg(4,v); if(v!==7) $fatal(1,"SGMII config readback");
        if(!tx_dis) $fatal(1,"Config write did not reset IP");
        read_reg(16,v); if(!v[5] || v[4]) $fatal(1,"reset/link flags");
        #1500000;
        read_reg(16,v); if(v!==23) $fatal(1,"link did not recover");
        los=1; #10000;
        read_reg(16,v); if(v[4] || !v[3]) $fatal(1,"LOS did not reset network");
        read_reg(20,v); if(v!==2) $fatal(1,"link losses: %d",v);
        los=0; #10000;
        read_reg(16,v); if(v!==23) $fatal(1,"LOS recovery");
        write_reg(4,0); // Forced BASE-X
        #1500000;
        read_reg(4,v); if(v!==0) $fatal(1,"forced BASE-X config");
        read_reg(16,v); if(v!==23) $fatal(1,"forced BASE-X link");
        $display("TEST PASSED: UART signature/config, reset-on-change, link/LOS recovery, constant actuator outputs");
        $finish;
    end
    initial begin #15000000; $fatal(1,"UART/control timeout"); end
endmodule

module sfp_host_mac (
    input wire clk, input wire rst,
    input wire cfg_sgmii, input wire cfg_an, input wire cfg_use_los,
    input wire sfp_los, output wire sfp_tx_disable,
    output reg rx_clk=0, output wire rx_valid, output wire [7:0] rx_data,
    output wire rx_last, output wire rx_error,
    output reg tx_clk=0, input wire tx_valid, input wire [7:0] tx_data,
    input wire tx_last, output wire tx_ready,
    output wire [10:0] status, output wire [15:0] partner,
    output wire qpll_ok, output wire rx_lock, output wire aligned
);
    always #4 rx_clk=~rx_clk;
    initial begin #1; forever #4 tx_clk=~tx_clk; end
    assign sfp_tx_disable=rst;
    assign qpll_ok=!rst;
    assign rx_lock=!rst && !(cfg_use_los && sfp_los);
    assign aligned=rx_lock;
    assign status=rx_lock ? (cfg_an ? 11'd7 : 11'd3) : 11'd0;
    assign partner=cfg_sgmii ? 16'hd801 : 16'h0020;
    assign {rx_valid,rx_data,rx_last,rx_error}=11'd0;
    assign tx_ready=!rst;
endmodule
