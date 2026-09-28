// 速度指令安全限幅：每 TICK_US 输出一次给底盘（RL 运动控制）。
//   目标值 = 软核指令经速度上限裁剪；非全向底盘 vy 强制为 0。
//   看门狗：超过 WD_US 未收到新指令 -> 目标值为 0（按加速度限制减速）。
//   急停：寄存器 ESTOP 或使能的外部输入 -> 立即输出 0。
//   输出变化量每拍不超过 DV / DW（加速度限制）。
// 注意：输出 0 速度表示"原地保持"，由底盘 RL 策略维持平衡，不是断电。
module safety_guard (
    input  wire        clk,
    input  wire        rst,
    input  wire [31:0] time_us,
    input  wire        tick_us,

    input  wire        cfg_enable,
    input  wire        cfg_holonomic,
    input  wire        cfg_estop,
    input  wire [15:0] cfg_vx_max,
    input  wire [15:0] cfg_vy_max,
    input  wire [15:0] cfg_wz_max,
    input  wire [15:0] cfg_dv,
    input  wire [15:0] cfg_dw,
    input  wire [31:0] cfg_tick_us,
    input  wire [31:0] cfg_wd_us,
    input  wire [3:0]  cfg_ext_mask,
    input  wire [3:0]  ext_stop,

    input  wire        cmd_we,          // 单拍：提交一条指令
    input  wire signed [15:0] cmd_vx,
    input  wire signed [15:0] cmd_vy,
    input  wire signed [15:0] cmd_wz,
    input  wire [7:0]  cmd_mode,

    output reg  signed [15:0] out_vx,
    output reg  signed [15:0] out_vy,
    output reg  signed [15:0] out_wz,
    output reg  [7:0]  out_mode,
    output reg         out_stop,        // 急停中
    output reg         out_wdog,        // 看门狗触发中
    output reg         out_valid,       // 单拍：新输出
    output reg  [15:0] out_seq,
    output wire [3:0]  ext_active
);
    reg signed [15:0] tvx, tvy, twz;
    reg [7:0]  tmode;
    reg [31:0] last_cmd_us, tick_cnt;
    reg        have_cmd;
    reg        tick_last;     // 下一个 tick_us 即为输出节拍
    always @(posedge clk) tick_last <= (tick_cnt + 1'b1 >= cfg_tick_us);

    assign ext_active = ext_stop & cfg_ext_mask;
    wire estop = cfg_estop | (|ext_active);
    wire wdog  = !have_cmd || ((time_us - last_cmd_us) > cfg_wd_us);

    function signed [15:0] clampm(input signed [15:0] v, input [15:0] m);
        reg signed [16:0] mm;
        begin
            mm = {1'b0, m};
            if ($signed({v[15], v}) > mm) clampm = m;
            else if ($signed({v[15], v}) < -mm) clampm = -m;
            else clampm = v;
        end
    endfunction
    // 从 cur 向 tgt 靠近，步长不超过 d（两个加法并行，再比较选择，缩短关键路径）
    function signed [15:0] ramp(input signed [15:0] cur, input signed [15:0] tgt, input [15:0] d);
        reg signed [17:0] up, dn, t;
        begin
            up = $signed({{2{cur[15]}}, cur}) + $signed({2'b0, d});
            dn = $signed({{2{cur[15]}}, cur}) - $signed({2'b0, d});
            t  = $signed({{2{tgt[15]}}, tgt});
            if (t > up) ramp = up[15:0];
            else if (t < dn) ramp = dn[15:0];
            else ramp = tgt;
        end
    endfunction

    // 第 1 级（每拍）：限幅后的目标值寄存；第 2 级（节拍时）：按加速度限制靠近
    reg signed [15:0] gvx, gvy, gwz;
    reg               g_zero;
    always @(posedge clk) begin
        gvx <= wdog ? 16'sd0 : clampm(tvx, cfg_vx_max);
        gvy <= (wdog || !cfg_holonomic) ? 16'sd0 : clampm(tvy, cfg_vy_max);
        gwz <= wdog ? 16'sd0 : clampm(twz, cfg_wz_max);
        g_zero <= !cfg_enable || estop;
    end

    always @(posedge clk) begin
        out_valid <= 1'b0;
        if (rst) begin
            tvx <= 0; tvy <= 0; twz <= 0; tmode <= 0; have_cmd <= 1'b0; last_cmd_us <= 0;
            out_vx <= 0; out_vy <= 0; out_wz <= 0; out_mode <= 0; out_stop <= 1'b0; out_wdog <= 1'b1;
            out_seq <= 0; tick_cnt <= 0;
        end else begin
            if (cmd_we) begin
                tvx <= cmd_vx; tvy <= cmd_vy; twz <= cmd_wz; tmode <= cmd_mode;
                last_cmd_us <= time_us; have_cmd <= 1'b1;
            end
            if (tick_us) tick_cnt <= tick_cnt + 1'b1;
            if (tick_us && tick_last) begin
                tick_cnt  <= 0;
                out_valid <= 1'b1;
                out_seq   <= out_seq + 1'b1;
                out_stop  <= estop;
                out_wdog  <= wdog;
                out_mode  <= tmode;
                if (g_zero || !cfg_enable || estop) begin
                    out_vx <= 0; out_vy <= 0; out_wz <= 0;
                end else begin
                    out_vx <= ramp(out_vx, gvx, cfg_dv);
                    out_vy <= ramp(out_vy, gvy, cfg_dv);
                    out_wz <= ramp(out_wz, gwz, cfg_dw);
                end
            end
        end
    end
endmodule
