// nav_subsystem 端到端测试：测试台扮演"软核 + 雷达 + RK3588 + 底盘 + IMU"。
//   1. 雷达口：ARP -> 应答；3 帧 XT 点云（真实报文格式）
//   2. 软核：WAIT_POSE 握手写位姿 -> 等代价图 -> 经总线窗口读出整张代价图，与定点模型逐格比对
//   3. 软核：写候选、启动轨迹评估、读结果，与模型逐字比对
//   4. 主机口：RK3588 目标报文 -> 邮箱读出比对；软核回发状态报文 -> 检查主机口发出 UDP 帧
//   5. 安全限幅：写速度指令 -> 底盘输出按加速度限制爬升 -> 停发指令后看门狗归零
//   6. IMU 串口：发 3 字节 -> 总线读出，首字节带 FIRST 标记和时间戳
`timescale 1ns/1ps
module tb_nav_top;
`include "nav_regs_map.vh"
    logic clk = 0, mclk = 0, rst = 1;
    always #5 clk = ~clk;
    always #4 mclk = ~mclk;

    logic [8:0]  ethl [0:131071];
    logic [8:0]  ethh [0:2047];
    logic [31:0] pose [0:255];
    logic [7:0]  cexp [0:16384*4-1];
    logic [31:0] trj  [0:1023];
    logic [31:0] gexp [0:63];
    logic [31:0] elx [0:47];
    logic [31:0] slx [0:0];            // 最后一帧 SLF 结构点数          // 高程读期望：{槽地址, 低 32 位, 高 20 位} × 16
    int n_ethl, n_frm, n_ethh, n_gexp;
    int c_valid, c_thr0, c_thr1, c_w, c_infl, c_zband;

    // 总线
    logic bus_req = 0, bus_we = 0, bus_ack, irq;
    logic [15:0] bus_addr;
    logic [31:0] bus_wdata, bus_rdata;
    // MAC
    logic lrv = 0, lrl = 0, hrv = 0, hrl = 0;
    logic [7:0] lrd = 0, hrd = 0;
    logic ltv, ltl, htv, htl;
    logic [7:0] ltd, htd;
    logic imu_rx = 1, imu_tx;
    logic signed [15:0] ch_vx, ch_vy, ch_wz;
    logic [7:0] ch_mode;
    logic ch_stop, ch_wdog, ch_valid;

    nav_subsystem #(.LUT_INIT("lut_default.hex"), .SIN_INIT("sin_lut.hex")) dut (
        .clk(clk), .rst(rst),
        .bus_req(bus_req), .bus_we(bus_we), .bus_addr(bus_addr), .bus_wdata(bus_wdata),
        .bus_ack(bus_ack), .bus_rdata(bus_rdata), .irq(irq),
        .l_rx_clk(mclk), .l_rx_rst(rst), .l_rx_valid(lrv), .l_rx_data(lrd), .l_rx_last(lrl), .l_rx_error(1'b0),
        .l_tx_clk(mclk), .l_tx_rst(rst), .l_tx_valid(ltv), .l_tx_data(ltd), .l_tx_last(ltl), .l_tx_ready(1'b1),
        .h_rx_clk(mclk), .h_rx_rst(rst), .h_rx_valid(hrv), .h_rx_data(hrd), .h_rx_last(hrl), .h_rx_error(1'b0),
        .h_tx_clk(mclk), .h_tx_rst(rst), .h_tx_valid(htv), .h_tx_data(htd), .h_tx_last(htl), .h_tx_ready(1'b1),
        .imu_rx(imu_rx), .imu_tx(imu_tx),
        .ch_vx(ch_vx), .ch_vy(ch_vy), .ch_wz(ch_wz), .ch_mode(ch_mode), .ch_stop(ch_stop), .ch_wdog(ch_wdog),
        .ch_valid(ch_valid), .ch_seq(), .odo_valid(1'b0), .odo_vx(16'd0), .odo_vy(16'd0), .odo_wz(16'd0),
        .ext_stop(4'd0));

    // 调试：导出内部像素流与滤波输出
    int pf_dump, px_dump;
    initial begin pf_dump = $fopen("pf_dump.hex", "w"); px_dump = $fopen("px_dump.hex", "w"); end
    always @(posedge clk) begin
        if (dut.pf_valid && dut.pf_ready)
            $fdisplay(pf_dump, "%010x", {dut.pf_upd, dut.pf_eof, dut.pf_addr, dut.pf_tag, dut.pf_z});
        if (dut.x_valid && dut.x_ready)
            $fdisplay(px_dump, "%09x", {dut.x_sof, dut.x_eof, dut.x_u, dut.x_v, dut.x_depth});
    end

    int errs = 0;
    task automatic wr(input [15:0] a, input [31:0] d);
        @(posedge clk); bus_req <= 1; bus_we <= 1; bus_addr <= a; bus_wdata <= d;
        do @(posedge clk); while (!bus_ack);
        bus_req <= 0;
    endtask
    task automatic rd(input [15:0] a, output [31:0] d);
        @(posedge clk); bus_req <= 1; bus_we <= 0; bus_addr <= a;
        do @(posedge clk); while (!bus_ack);
        d = bus_rdata;
        bus_req <= 0;
    endtask
    task automatic chk(string what, input [31:0] got, input [31:0] exp);
        if (got !== exp) begin errs++; $display("%s: 期望 %h 实得 %h", what, exp, got); end
    endtask

    // 雷达口发送进程
    int li = 0;
    bit lidar_go = 0;
    int gapc = 0;
    always @(posedge mclk) begin
        lrv <= 0; lrl <= 0;
        if (lidar_go && li < n_ethl) begin
            // 百兆线速：125 MHz MAC 时钟下每 10 拍 1 字节，帧间隔 12 字节
            if (gapc > 0) gapc <= gapc - 1;
            else begin
                {lrl, lrd} <= ethl[li]; lrv <= 1; li <= li + 1;
                gapc <= ethl[li][8] ? 120 : 9;
            end
        end
    end

    // 雷达口发出的帧（应为 ARP 应答）
    int l_tx_frames = 0;
    logic [7:0] ltbuf [0:127];
    int ltn = 0;
    always @(posedge mclk) if (ltv) begin
        ltbuf[ltn] = ltd; ltn++;
        if (ltl) begin
            l_tx_frames++;
            if (ltbuf[12] != 8'h08 || ltbuf[13] != 8'h06 || ltbuf[21] != 8'h02) begin
                errs++; $display("雷达口发出的不是 ARP 应答");
            end
            ltn = 0;
        end
    end
    // 主机口发出的帧
    logic [7:0] htbuf [0:2047];
    int htn = 0, h_tx_frames = 0, h_udp_len = 0;
    always @(posedge mclk) if (htv) begin
        htbuf[htn] = htd; htn++;
        if (htl) begin
            h_tx_frames++;
            if (htbuf[12] == 8'h08 && htbuf[13] == 8'h00) h_udp_len = {htbuf[38], htbuf[39]} - 8;
            htn = 0;
        end
    end

    task automatic uart_byte(input [7:0] b);
        imu_rx = 0; #(8680);
        for (int i = 0; i < 8; i++) begin imu_rx = b[i]; #(8680); end
        imu_rx = 1; #(8680);
    endtask

    initial begin
        int fd, p, K, T;
        logic [31:0] d, seq0;
        fd = $fopen("cfg.txt", "r");
        void'($fscanf(fd, "%d %d %d %d %d %d %d %d %d %d", n_ethl, n_frm, n_ethh, n_gexp,
                      c_valid, c_thr0, c_thr1, c_w, c_infl, c_zband));
        $fclose(fd);
        $readmemh("eth_l.hex", ethl);
        $readmemh("eth_h.hex", ethh);
        $readmemh("pose.hex", pose);
        $readmemh("elev_exp.hex", elx);
        $readmemh("slf_exp.hex", slx);
        $readmemh("cost_exp.hex", cexp);
        $readmemh("traj.hex", trj);
        $readmemh("goal_exp.hex", gexp);
        repeat (20) @(posedge clk);
        rst = 0;
        repeat (20000) @(posedge clk);     // 等地图上电清除

        // ---------- 配置 ----------
        rd(RA_SYS_ID, d); chk("SYS.ID", d, 32'h4E415630);
        wr(RA_SYS_SCRATCH, 32'h12345678); rd(RA_SYS_SCRATCH, d); chk("SCRATCH", d, 32'h12345678);
        wr(RA_LIDAR_CTRL, 32'h9);            // ENABLE + WAIT_POSE，大端，仅深度
        wr(RA_LIDAR_ZBAND, c_zband);
        wr(RA_MAP_VALID_CFG, c_valid);
        wr(RA_MAP_THR0, c_thr0);
        wr(RA_MAP_THR1, c_thr1);
        wr(RA_MAP_WEIGHT, c_w);
        wr(RA_MAP_INFL, c_infl);
        wr(RA_SYS_IRQ_ENABLE, 32'h2);        // MAP_READY

        // ---------- 雷达帧 ----------
        lidar_go = 1;
        for (int k = 0; k < n_frm; k++) begin
            rd(RA_MAP_STAT, seq0);
            begin
                int tries = 0;
                do begin
                    rd(RA_LIDAR_POSE_STAT, d); tries++;
                    if (tries % 20000 == 0) begin
                        logic [31:0] a, b, c;
                        rd(RA_LIDAR_STAT_FRAMES, a); rd(RA_LIDAR_STAT_BAD, b); rd(RA_NETL_STAT_RX_UDP, c);
                        $display("[%0t] 等待帧%0d：lidar frames=%0d bad=%0d udp=%0d 注入 %0d/%0d", $time, k, a, b, c, li, n_ethl);
                    end
                end while (!d[0]);
            end
            $display("[%0t] 帧%0d 等待位姿", $time, k);
            for (int j = 0; j < 9; j++) wr(RA_LIDAR_R00 + 4 * j, pose[k*14 + j]);
            wr(RA_LIDAR_TX, pose[k*14 + 9]); wr(RA_LIDAR_TY, pose[k*14 + 10]); wr(RA_LIDAR_TZ, pose[k*14 + 11]);
            wr(RA_MAP_CENTER_X, pose[k*14 + 12]); wr(RA_MAP_CENTER_Y, pose[k*14 + 13]);
            wr(RA_LIDAR_POSE_COMMIT, 1);
            wait (irq);
            wr(RA_SYS_IRQ_STATUS, 32'h2);
            rd(RA_MAP_STAT, d);
            chk("MAP.SEQ", d[31:16], seq0[31:16] + 1);
            begin
                int bad = 0;
                for (int w = 0; w < 4096; w++) begin
                    rd(RW_COSTMAP_BASE + 4 * w, d);
                    for (int b = 0; b < 4; b++)
                        if (d[8*b +: 8] !== cexp[k*16384 + 4*w + b]) begin
                            bad++;
                            if (bad < 5) $display("帧%0d 槽%0d 期望 %0d 实得 %0d", k, 4*w+b, cexp[k*16384 + 4*w + b], d[8*b +: 8]);
                        end
                end
                if (bad) begin errs += bad; $display("帧%0d 代价图不一致 %0d", k, bad); end
                else $display("帧%0d 代价图经总线读出一致", k);
            end
        end
        // ---------- SLF：最后一帧的结构点数、一次打分 + 并图走通 ----------
        begin
            int tries = 0;
            do begin rd(RA_MATCH_STAT, d); tries++; end while (!d[1] && tries < 1000);
            chk("MATCH.PTS_READY", d[1], 1);
            chk("MATCH.NPTS", d[28:16], slx[0][12:0]);
            wr(RW_MATCH_CAND_BASE, 32'h0000_0000); wr(RW_MATCH_CAND_BASE + 4, 32'h0);
            wr(RW_MATCH_CAND_BASE + 8, 32'h0019_0000); wr(RW_MATCH_CAND_BASE + 12, 32'h80);
            wr(RA_MATCH_NCAND, 2);
            wr(RA_MATCH_CMD, 1);
            tries = 0;
            do begin rd(RA_MATCH_STAT, d); tries++; end while (!d[2] && tries < 100000);
            chk("MATCH.EVAL_DONE", d[2], 1);
            rd(RW_MATCH_RES_BASE + 4, d); chk("MATCH 结果（空结构图：无参与点）", d, 0);
            wr(RA_MATCH_CMD, 2);
            tries = 0;
            do begin rd(RA_MATCH_STAT, d); tries++; end while (d[0] && tries < 100000);
            chk("MATCH 并图后空闲", d[1:0], 0);
        end
        // ---------- 高程图读口 ----------
        for (int i = 0; i < 16; i++) begin
            int tries = 0;
            wr(RA_MAP_ELEV_ADDR, elx[3*i]);
            wr(RA_MAP_ELEV_CMD, 1);
            do begin rd(RA_MAP_ELEV_Q1, d); tries++; end while (!d[31] && tries < 100);
            chk($sformatf("ELEV 槽%0d 高位", elx[3*i]), d[19:0], elx[3*i+2][19:0]);
            rd(RA_MAP_ELEV_Q0, d);
            chk($sformatf("ELEV 槽%0d 低位", elx[3*i]), d, elx[3*i+1]);
        end
        rd(RA_LIDAR_STAT_FRAMES, d); chk("LIDAR.STAT_FRAMES", d, n_frm);
        rd(RA_LIDAR_STAT_BAD, d);    chk("LIDAR.STAT_BAD", d, 0);
        rd(RA_NETL_STAT_ARP, d);     chk("NETL.STAT_ARP", d, 1);
        if (l_tx_frames != 1) begin errs++; $display("雷达口 ARP 应答帧数 %0d", l_tx_frames); end

        // ---------- 轨迹评估 ----------
        wr(RA_TRAJ_X0, trj[0]); wr(RA_TRAJ_Y0, trj[1]); wr(RA_TRAJ_TH0, trj[2]);
        T = trj[3]; K = trj[4];
        wr(RA_TRAJ_CFG, (T << 8) | K);
        p = 5;
        for (int i = 0; i < 2 * K; i++) wr(RW_TRAJ_CAND_BASE + 4 * i, trj[p + i]);
        p += 2 * K;
        wr(RA_TRAJ_CMD, 1);
        do rd(RA_TRAJ_STAT, d); while (!d[1]);
        for (int i = 0; i < 4 * K; i++) begin
            rd(RW_TRAJ_RES_BASE + 4 * i, d);
            if (d !== trj[p + i]) begin
                errs++; if (errs < 20) $display("轨迹结果 %0d.%0d 期望 %h 实得 %h", i / 4, i % 4, trj[p + i], d);
            end
        end
        $display("轨迹评估 %0d 条完成", K);

        // ---------- 主机口 ----------
        for (int i = 0; i < n_ethh; i++) begin
            @(posedge mclk); {hrl, hrd} <= ethh[i]; hrv <= 1;
        end
        @(posedge mclk); hrv <= 0; hrl <= 0;
        repeat (500) @(posedge clk);
        rd(RA_MBX_RX_STAT, d);
        chk("MBX.AVAIL", d[31], 1);
        chk("MBX.LEN", d[15:0], gexp[0]);
        for (int w = 1; w < n_gexp; w++) begin
            rd(RA_MBX_RX_DATA, d); chk($sformatf("MBX 字%0d", w), d, gexp[w]);
        end
        rd(RA_MBX_RX_SRC_PORT, d); chk("MBX.SRC_PORT", d, 9001);
        wr(RA_MBX_RX_POP, 1);
        rd(RA_MBX_RX_STAT, d); chk("MBX 弹出后", d[31], 0);
        // 回发 12 字节状态
        wr(RA_MBX_TX_DATA, 32'h01535456); wr(RA_MBX_TX_DATA, 32'h11223344); wr(RA_MBX_TX_DATA, 32'h55667788);
        wr(RA_MBX_TX_LEN, 12); wr(RA_MBX_TX_GO, 1);
        repeat (3000) @(posedge clk);
        if (h_udp_len != 12) begin errs++; $display("主机口未发出 12 字节 UDP（%0d 帧，长度 %0d）", h_tx_frames, h_udp_len); end
        rd(RA_NETH_PEER_PORT, d); chk("NETH.PEER_PORT（学习）", d, 9001);

        // ---------- 安全限幅 ----------
        wr(RA_SAFE_TICK_US, 10); wr(RA_SAFE_WD_US, 400); wr(RA_SAFE_LIM_A, (32'd10 << 16) | 32'd30);
        wr(RA_SAFE_CTRL, 1);
        wr(RA_SAFE_CMD_V, 32'h0064_0258);   // vx=600, vy=100（非全向：应为 0）
        wr(RA_SAFE_CMD_W, 32'h0000_00C8);   // wz=200，提交
        repeat (30000) @(posedge clk);       // 300 us：vx 已到 600（10/10us）、wz 已到 200
        if (ch_vx != 600 || ch_vy != 0 || ch_wz != 200) begin
            errs++; $display("限幅输出 vx=%0d vy=%0d wz=%0d", ch_vx, ch_vy, ch_wz);
        end
        repeat (150000) @(posedge clk);      // 1.5 ms 无新指令 -> 看门狗减速到 0
        if (!ch_wdog || ch_vx != 0 || ch_wz != 0) begin
            errs++; $display("看门狗未生效 wdog=%0d vx=%0d wz=%0d", ch_wdog, ch_vx, ch_wz);
        end

        // ---------- IMU 串口 ----------
        uart_byte(8'h55); uart_byte(8'hA1); uart_byte(8'h7E);
        repeat (2000) @(posedge clk);
        rd(RA_IMU_RX_COUNT, d); chk("IMU.RX_COUNT", d, 3);
        rd(RA_IMU_RX_DATA, d); chk("IMU 字节0", d, 32'h8000_0155);
        rd(RA_IMU_RX_DATA, d); chk("IMU 字节1", d, 32'h8000_00A1);
        rd(RA_IMU_RX_DATA, d); chk("IMU 字节2", d, 32'h8000_007E);
        rd(RA_IMU_RX_TS, d);
        if (d == 0) begin errs++; $display("IMU 时间戳缺失"); end

        if (errs == 0) $display("TEST PASSED");
        else $display("TEST FAILED errs=%0d", errs);
        $finish;
    end
    initial begin #400ms; $display("TEST FAILED 超时"); $finish; end
endmodule
