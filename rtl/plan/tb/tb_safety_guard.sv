// safety_guard 测试：随机指令、急停、外部停止、看门狗超时，输出与测试台内参考模型逐拍比对。
`timescale 1ns/1ps
module tb_safety_guard;
    logic clk = 0, rst = 1;
    always #5 clk = ~clk;

    logic [31:0] time_us;
    logic        tick_us;
    nav_timebase #(.CLK_HZ(100000000)) u_tb (.clk(clk), .rst(rst), .time_us(time_us), .tick_us(tick_us));

    logic cfg_enable = 1, cfg_holo = 0, cfg_estop = 0;
    logic [3:0] ext = 0, ext_mask = 4'b0101;
    logic cmd_we = 0;
    logic signed [15:0] cvx = 0, cvy = 0, cwz = 0;
    logic [7:0] cmode = 0;
    logic signed [15:0] ovx, ovy, owz;
    logic [7:0] omode;
    logic ostop, owdog, ovalid;
    localparam int VXM = 1000, VYM = 300, WZM = 1500, DV = 20, DW = 30, TICK = 50, WD = 2000;

    safety_guard dut (
        .clk(clk), .rst(rst), .time_us(time_us), .tick_us(tick_us),
        .cfg_enable(cfg_enable), .cfg_holonomic(cfg_holo), .cfg_estop(cfg_estop),
        .cfg_vx_max(VXM[15:0]), .cfg_vy_max(VYM[15:0]), .cfg_wz_max(WZM[15:0]), .cfg_dv(DV[15:0]), .cfg_dw(DW[15:0]),
        .cfg_tick_us(TICK), .cfg_wd_us(WD), .cfg_ext_mask(ext_mask), .ext_stop(ext),
        .cmd_we(cmd_we), .cmd_vx(cvx), .cmd_vy(cvy), .cmd_wz(cwz), .cmd_mode(cmode),
        .out_vx(ovx), .out_vy(ovy), .out_wz(owz), .out_mode(omode), .out_stop(ostop), .out_wdog(owdog),
        .out_valid(ovalid), .out_seq(), .ext_active());

    // 参考模型
    int rvx = 0, rvy = 0, rwz = 0, tvx = 0, tvy = 0, twz = 0;
    int last_cmd = 0; bit have = 0;
    int errs = 0, ticks = 0, n_stop = 0, n_wd = 0;

    function int clampm(int v, int m); return v > m ? m : (v < -m ? -m : v); endfunction
    function int ramp(int cur, int tgt, int d);
        if (tgt - cur > d) return cur + d;
        if (tgt - cur < -d) return cur - d;
        return tgt;
    endfunction

    always @(posedge clk) begin
        if (!rst && cmd_we) begin tvx = cvx; tvy = cvy; twz = cwz; last_cmd = time_us; have = 1; end
        if (!rst && ovalid) begin
            bit es, wd;
            ticks++;
            es = cfg_estop || ((ext & ext_mask) != 0);
            wd = !have || (time_us - 1 - last_cmd > WD);    // 输出比判定晚一拍，time_us 可能已跨一微秒，见下方容差
            if (es) begin rvx = 0; rvy = 0; rwz = 0; n_stop++; end
            else begin
                if (owdog) n_wd++;
                rvx = ramp(rvx, owdog ? 0 : clampm(tvx, VXM), DV);
                rvy = ramp(rvy, (owdog || !cfg_holo) ? 0 : clampm(tvy, VYM), DV);
                rwz = ramp(rwz, owdog ? 0 : clampm(twz, WZM), DW);
            end
            if (ovx !== rvx[15:0] || ovy !== rvy[15:0] || owz !== rwz[15:0] || ostop !== es) begin
                errs++;
                if (errs < 10) $display("t=%0d 期望 %0d %0d %0d stop%0d 实得 %0d %0d %0d stop%0d wd%0d",
                                        time_us, rvx, rvy, rwz, es, ovx, ovy, owz, ostop, owdog);
            end
        end
    end

    initial begin
        repeat (5) @(posedge clk);
        rst = 0;
        for (int i = 0; i < 400; i++) begin
            repeat ($urandom_range(500, 6000)) @(posedge clk);
            case ($urandom % 10)
                0: cfg_estop = ~cfg_estop;
                1: ext = $urandom;
                2: cfg_holo = ~cfg_holo;
                3: repeat (300000) @(posedge clk);          // 3 ms 无指令 -> 看门狗
                default: begin
                    cvx = $urandom_range(0, 3000) - 1500; cvy = $urandom_range(0, 1000) - 500;
                    cwz = $urandom_range(0, 4000) - 2000; cmode = $urandom;
                    @(posedge clk) cmd_we = 1;
                    @(posedge clk) cmd_we = 0;
                end
            endcase
        end
        $display("节拍 %0d 急停 %0d 看门狗 %0d", ticks, n_stop, n_wd);
        if (errs == 0 && ticks > 1000 && n_stop > 0 && n_wd > 0) $display("TEST PASSED");
        else $display("TEST FAILED errs=%0d", errs);
        $finish;
    end
endmodule
