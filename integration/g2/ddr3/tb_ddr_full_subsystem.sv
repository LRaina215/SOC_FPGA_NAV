`timescale 1ns/1ps
module tb_ddr_full_subsystem;
    reg clk=0,ui=0,run_ui=1,lr=0,hr=0;
    always #10 clk=~clk;
    always #5 if(run_ui)ui=~ui;
    always #5 lr=~lr;
    always #4 hr=~hr;
    reg rst=1,cold=1,pll=0,cal=0,req=0,we=0;
    reg[15:0]a=0;reg[31:0]wd=0;
    wire ack;wire[31:0]q;wire fault,initialized;
    wire signed[15:0]vx,vy,wz;wire stop,valid;
    wire[2:0]cmd;wire en,cr,wen,wend,wr,rv,re;
    wire[27:0]addr;wire[255:0]data,rdata;wire[31:0]mask;
    reg d_req=0,d_we=0;reg[15:0]d_addr=0;reg[31:0]d_wdata=0;
    wire d_ack;wire[31:0]d_reply;
    // The original CPU/128KiB memory is unchanged. Its program's NAV master
    // waits at ACK=0; scripted MMIO below owns the independent NAV fixture.
    nav_soc #(.RAM_KB(128),.INIT_B0("ram_b0.hex"),.INIT_B1("ram_b1.hex"),
        .INIT_B2("ram_b2.hex"),.INIT_B3("ram_b3.hex")) cpu(
        .clk(clk),.rst(rst),.m_req(),.m_we(),.m_addr(),.m_wdata(),.m_ack(1'b0),.m_rdata(32'd0),
        .d_req(d_req),.d_we(d_we),.d_addr(d_addr),.d_wdata(d_wdata),.d_ack(d_ack),.d_rdata(d_reply),.led(),.trap());
    integer refresh_phase=389,ready_period=16,read_delay=20;
    row_service_model dram(ui,cold,cal,cmd,en,addr,cr,data,mask,wen,wend,wr,rdata,rv,re,
        refresh_phase,ready_period,read_delay);
    initial for(integer i=0;i<16384;i++)dram.mem[i]='1;
    nav_subsystem_ddr_full #(.CLK_HZ(50000000),.SIN_INIT("sin_lut.hex"),
        .SUPPORT_COVERAGE_ENABLE(1),.HOST_EXTENSION_ENABLE(1)) dut(
        .clk(clk),.rst(rst),.bus_req(req),.bus_we(we),.bus_addr(a),.bus_wdata(wd),.bus_ack(ack),.bus_rdata(q),.irq(),
        .l_rx_clk(lr),.l_rx_rst(rst),.l_rx_valid(1'b0),.l_rx_data(8'd0),.l_rx_last(1'b0),.l_rx_error(1'b0),
        .l_tx_clk(lr),.l_tx_rst(rst),.l_tx_valid(),.l_tx_data(),.l_tx_last(),.l_tx_ready(1'b1),.l_phy_stat(32'd0),.l_rx_neg(),
        .h_rx_clk(hr),.h_rx_rst(rst),.h_rx_valid(1'b0),.h_rx_data(8'd0),.h_rx_last(1'b0),.h_rx_error(1'b0),
        .h_tx_clk(hr),.h_tx_rst(rst),.h_tx_valid(),.h_tx_data(),.h_tx_last(),.h_tx_ready(1'b1),.h_core_rst(rst),
        .control_tap_valid(1'b0),.control_tap_first(1'b0),.control_tap_last(1'b0),.control_tap_data(8'd0),
        .control_tx_accepted(),.control_tx_dropped(),.control_tx_sent(),.imu_rx(1'b1),.imu_tx(),
        .ch_vx(vx),.ch_vy(vy),.ch_wz(wz),.ch_mode(),.ch_stop(stop),.ch_wdog(),.ch_valid(valid),.ch_seq(),
        .odo_valid(1'b0),.odo_vx(16'd0),.odo_vy(16'd0),.odo_wz(16'd0),.ext_stop(4'd0),
        .ddr_ui_clk(ui),.ddr_calib_done(cal),.ddr_cold_rst(cold),.ddr_pll_locked(pll),
        .ddr_memory_fault(fault),.ddr_initialized(initialized),.ddr_cmd(cmd),.ddr_cmd_en(en),.ddr_addr(addr),.ddr_cmd_ready(cr),
        .ddr_wr_data(data),.ddr_wr_mask(mask),.ddr_wr_en(wen),.ddr_wr_end(wend),.ddr_wr_ready(wr),
        .ddr_rd_data(rdata),.ddr_rd_valid(rv),.ddr_rd_end(re));
    integer cycles=0,checks=0,bus_ops=0,max_polls=0,longest_bus=0;
    reg checking_mode=0;
    always@(posedge clk)begin
        cycles++;
        if(checking_mode&&dut.mt_busy&&dut.r_MATCH_CTRL!==32'd1)
            $fatal(1,"mode changed during active matcher command");
    end
    task automatic access(input bit w,input[15:0]address,input[31:0]d,output[31:0]reply);
        integer waited;
        begin
            @(negedge clk);req=1;we=w;a=address;wd=d;waited=0;
            do begin @(posedge clk);#1;waited++;
                if(waited>1000000)$fatal(1,"MMIO transaction timeout addr=%h",address);end while(!ack);
            reply=q;bus_ops++;if(waited>longest_bus)longest_bus=waited;
            @(negedge clk);req=0;
            repeat(3)@(negedge clk);
        end
    endtask
    task automatic write_reg(input[15:0]address,input[31:0]d);
        reg[31:0]unused;access(1,address,d,unused);
    endtask
    task automatic debug_access(input bit w,input[15:0]address,input[31:0]d,output[31:0]reply);
        integer waited;
        begin
            @(negedge clk);d_req=1;d_we=w;d_addr=address;d_wdata=d;waited=0;
            do begin @(posedge clk);#1;waited++;
                if(waited>1000)$fatal(1,"original CPU debug timeout");end while(!d_ack);
            reply=d_reply;@(negedge clk);d_req=0;repeat(3)@(negedge clk);
        end
    endtask
    task automatic read_marker;
        reg[31:0]x;integer polls;
        begin
            write_reg(16'h702c,16'h2000);write_reg(16'h7030,1);x=0;polls=0;
            while(!x[31]&&polls<64)begin access(0,16'h7034,0,x);polls++;end
            if(!x[31]||x[17:0]!==18'h23456)$fatal(1,"map changed across CPU HOLD restart");
        end
    endtask
    initial begin
        reg[31:0]x,boot_word;integer polls,t0,cold_started,mode_started;
        repeat(20)@(negedge clk);cold=0;rst=0;pll=1;cold_started=cycles;
        repeat(200)@(negedge clk);
        if(initialized||fault)$fatal(1,"untrained memory admitted");checks++;
        cal=1;wait(initialized);#1;
        if(!dut.sm_initialized||!dut.em_initialized||fault)$fatal(1,"incomplete cold maps admitted");checks++;
        $display("SUBSYSTEM_COLD cycles=%0d us_at_50MHz=%0f",cycles-cold_started,(cycles-cold_started)/50.0);
        debug_access(1,16'hf000,1,x);debug_access(0,16'hf014,0,x);
        if(x[31:16]!==16'd128)$fatal(1,"original CPU capacity changed");checks++;
        debug_access(1,16'hf004,32767,x);debug_access(1,16'hf008,32'h5a81e77d,x);
        debug_access(1,16'hf004,32767,x);debug_access(0,16'hf008,0,x);
        if(x!==32'h5a81e77d)$fatal(1,"original highest CPU RAM word unavailable");checks++;
        debug_access(1,16'hf004,8,x);debug_access(0,16'hf008,0,boot_word);
        write_reg(16'h7040,16'h2000);write_reg(16'h7044,18'h23456);wait(!dut.mt_busy);read_marker();
        debug_access(1,16'hf000,0,x);repeat(3)@(negedge clk);debug_access(1,16'hf000,1,x);
        debug_access(1,16'hf004,8,x);debug_access(0,16'hf008,0,x);
        if(x!==boot_word||!initialized||fault)$fatal(1,"CPU warm restart reset memory admission");
        read_marker();checks++;$display("CPU_HOLD_PROOF capacity_KiB=128 highest_word=32767 DDR_marker_retained=1");
        // Real regfile + bus + elevation handshake, all64 polls retained.
        // The first reads evict cold-clear dirty entries and use both BL8s.
        for(integer i=0;i<64;i++)begin
            access(0,16'h3028,0,x);
            if(x[0])$fatal(1,"map busy before idle read probe");
            write_reg(16'h3048,(i*4)%16384);write_reg(16'h304c,1);polls=0;x=0;
            while(polls<64&&!x[31])begin access(0,16'h3054,0,x);polls++;end
            if(!x[31]||x[20:0]!==0)$fatal(1,"elevation64-poll failure i=%0d polls=%0d word=%h",i,polls,x);
            access(0,16'h3050,0,x);if(x!==0)$fatal(1,"cold53-bit elevation mismatch");
            if(polls>max_polls)max_polls=polls;checks++;
        end
        // Command/mode writes are ordered, not allowed to alter a live
        // operation after a firmware polling timeout has expired.
        write_reg(16'h700c,4);wait(dut.mt_busy);checking_mode=1;mode_started=cycles;
        write_reg(16'h7000,3);checking_mode=0;
        if(dut.mt_busy||dut.r_MATCH_CTRL!==3||cycles-mode_started<100000)
            $fatal(1,"prior mode write was lost or not serialized");checks++;
        $display("SUBSYSTEM_MODE wait_cycles=%0d",cycles-mode_started);
        write_reg(16'h5010,100);write_reg(16'h5014,1000000);
        write_reg(16'h500c,32'h03e803e8);write_reg(16'h5000,1);
        write_reg(16'h5018,300);write_reg(16'h501c,100);
        repeat(11000)@(negedge clk);
        if(vx!==300||wz!==100||stop)$fatal(1,"healthy subsystem velocity endpoint failed");checks++;
        // Lose UI during an actual elevation miss but retain lock/cal high.
        write_reg(16'h3048,5000);write_reg(16'h304c,1);
        for(integer i=0;i<10000&&dut.u_match.u_store.u_native.ui_state==0;i++)@(negedge clk);
        if(dut.u_match.u_store.u_native.ui_state==0)$fatal(1,"fault fixture did not cause a native miss");
        run_ui=0;t0=cycles;
        repeat(525)@(negedge clk);
        if(!fault||initialized)$fatal(1,"full NAV missed stopped UI");checks++;
        repeat(5500)@(negedge clk);
        if(vx!==0||vy!==0||wz!==0||!stop)$fatal(1,"full NAV stop endpoint not zero");checks++;
        access(0,16'h3054,0,x);if(x[31])$fatal(1,"faulted elevation read falsely completed");checks++;
        run_ui=1;repeat(1000)@(negedge clk);
        if(!fault||initialized||!stop)$fatal(1,"full NAV recovered without cold reset");checks++;
        $display("TEST PASSED checks=%0d MMIO_ops=%0d elevation_reads=64 max_polls=%0d longest_bus_cycles=%0d",checks,bus_ops,max_polls,longest_bus);
        $finish;
    end
    initial begin #100000000;$fatal(1,"bounded functional test budget");end
endmodule
