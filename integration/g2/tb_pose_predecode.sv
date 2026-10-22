`timescale 1ns/1ps
module tb_pose_predecode;
    reg clk=0,rst=1,req=0,we=0;
    reg [15:0] addr=0;
    reg [31:0] data=0;
    wire ack;
    integer comparisons=0,pulses=0,transactions=0,resets=0;
    always #10 clk=~clk;
    nav_subsystem #(.CLK_HZ(50000000)) dut (
        .clk(clk),.rst(rst),.bus_req(req),.bus_we(we),.bus_addr(addr),.bus_wdata(data),.bus_ack(ack),.bus_rdata(),.irq(),
        .l_rx_clk(clk),.l_rx_rst(rst),.l_rx_valid(1'b0),.l_rx_data(8'd0),.l_rx_last(1'b0),.l_rx_error(1'b0),
        .l_tx_clk(clk),.l_tx_rst(rst),.l_tx_valid(),.l_tx_data(),.l_tx_last(),.l_tx_ready(1'b1),.l_phy_stat(32'd0),.l_rx_neg(),
        .h_rx_clk(clk),.h_rx_rst(rst),.h_rx_valid(1'b0),.h_rx_data(8'd0),.h_rx_last(1'b0),.h_rx_error(1'b0),
        .h_tx_clk(clk),.h_tx_rst(rst),.h_tx_valid(),.h_tx_data(),.h_tx_last(),.h_tx_ready(1'b1),
        .imu_rx(1'b1),.imu_tx(),.ch_vx(),.ch_vy(),.ch_wz(),.ch_mode(),.ch_stop(),.ch_wdog(),.ch_valid(),.ch_seq(),
        .odo_valid(1'b0),.odo_vx(16'd0),.odo_vy(16'd0),.odo_wz(16'd0),.ext_stop(4'd0));
    // The unmodified real nav_regfile remains the reference. This compares
    // pulses in the actual bus FSM, including alignment/window/reset behavior.
    always @(negedge clk) begin
        if($time>20) begin
            if(dut.pose_predecoded_q !== dut.p_LIDAR_POSE_COMMIT)
                $fatal(1,"pose mismatch rst=%b bs=%d addr=%h new=%b old=%b",rst,dut.bs,addr,dut.pose_predecoded_q,dut.p_LIDAR_POSE_COMMIT);
            comparisons=comparisons+1;
            if(dut.pose_predecoded_q)pulses=pulses+1;
        end
    end
    task automatic transact(input bit write_,input [15:0] address_,input [31:0] word_,input integer hold_);
        integer n;
        @(negedge clk);#1;req=1;we=write_;addr=address_;data=word_;
        n=0;
        do begin @(negedge clk);n=n+1;if(n>200)$fatal(1,"bus timeout %h",addr);end while(!ack);
        #1;req=0;
        repeat(hold_+2)@(negedge clk);
        transactions=transactions+1;
    endtask
    reg [31:0] random_state=32'h3ef20a81;
    function automatic [31:0] next_random(input [31:0] x);
        reg[31:0] y;
        begin y=x^(x<<13);y=y^(y>>17);next_random=y^(y<<5);end
    endfunction
    initial begin
        repeat(5)@(negedge clk);#1;rst=0;
        for(integer i=0;i<4;i=i+1)begin
            transact(1,16'h2070+i,0,0); // Old pulse does not depend on data.
            transact(0,16'h2070+i,32'hffffffff,0);
        end
        for(integer i=0;i<16;i=i+1)begin
            transact(1,16'h2070^(16'h1<<i),32'hffffffff,0);
        end
        // Special windows use a different bus branch; none may emit a pose.
        transact(1,16'h8000,1,0);transact(1,16'h7000,1,0);
        transact(1,16'h6000,1,0);transact(1,16'h6800,1,0);
        for(integer i=0;i<512;i=i+1)begin
            random_state=next_random(random_state);
            transact(1,(i%7==0)?(16'h2070|{14'd0,random_state[1:0]}):random_state[15:0],random_state,i%3);
        end
        // Reset just after accepting a pose write. Both pulses clear together.
        @(negedge clk);#1;req=1;we=1;addr=16'h2071;
        @(negedge clk);#1;rst=1;req=0;
        repeat(3)@(negedge clk);#1;rst=0;resets=resets+1;
        transact(1,16'h2072,32'hcafef00d,0);
        // A request held during reset is accepted only once reset is released.
        @(negedge clk);#1;rst=1;req=1;we=1;addr=16'h2073;
        repeat(3)@(negedge clk);#1;rst=0;
        do @(negedge clk);while(!ack);
        #1;req=0;resets=resets+1;
        repeat(5)@(negedge clk);
        if(pulses<75||comparisons<2000)$fatal(1,"insufficient stimulus");
        $display("TEST PASSED comparisons=%0d pulses=%0d transactions=%0d resets=%0d",comparisons,pulses,transactions,resets);
        $finish;
    end
    initial begin #1000000;$fatal(1,"timeout");end
endmodule
