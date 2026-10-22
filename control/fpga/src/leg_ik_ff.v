`ifndef LEG_IK_FF_V_INCLUDED
`define LEG_IK_FF_V_INCLUDED
// 腿 IK 前馈查表（2026-10-14 接入）：height_q12 -> 四路舵机脉宽偏移[µs]。
//
// 为什么需要它：训练侧关节参考是 IK(高度指令) + 动作×0.03rad（car_robot.py:265-268），
// 而板上原先只执行了"动作×0.03rad"那一半，把 IK 前馈整个省掉了，靠假设
// "ZERO_US=1500 就等于 IK(130.5mm)"。实测这个假设差约 2.9°（32µs），
// 比整个残差行程（±19µs）还大 —— 闭环参考是错的。
//
// 偏移以【机械标称位】为基准：LEG_*_ZERO_US 标定的是"机械装配姿态"对应的脉宽
// （= CAD 默认位 = 训练侧 default_joint_angles），而训练要的是 IK(高度指令)。
// 两者之差就是本模块输出的 ff。用法：
//
//     pulse = LEG_*_ZERO_US + ff_* + SIGN × action × 增益
//
// 表由 tools/gen_leg_ik_ff.py 生成（数学照抄 car_robot.py:287-317 _leg_reference()），
// 已含各舵机方向 SIGN，所以 ff 直接相加、不要再乘 SIGN。
//
// 索引设计：直接用 height_q12 当下标（348~778，即 84.96~189.94mm，431 档），不做插值。
// 因为 IK 在接近满伸腿处是奇异的（190mm 附近膝角变化约 2.7°/mm），任何"粗表+插值"
// 都会引入 1~2° 误差，正好把要修的 2.9° 抵消掉。直接查表则误差只剩命令本身的
// 0.244mm 量化。超出范围的命令按端点夹取（训练侧也是 clamp 到 0.085~0.19m）。
module leg_ik_ff #(
    parameter LH_FILE = "src/mem/leg_ik_lh.mem",
    parameter LK_FILE = "src/mem/leg_ik_lk.mem",
    parameter RH_FILE = "src/mem/leg_ik_rh.mem",
    parameter RK_FILE = "src/mem/leg_ik_rk.mem",
    parameter integer IDX_BASE = 348,   // 85mm 的 q12 值
    parameter integer IDX_TOP  = 778    // 190mm 的 q12 值
)(
    input  wire        clk,
    input  wire [23:0] height_q12,      // 与喂给 rl_observation_v5 的必须是同一个数
    output reg  signed [11:0] ff_lh,
    output reg  signed [11:0] ff_lk,
    output reg  signed [11:0] ff_rh,
    output reg  signed [11:0] ff_rk,
    output wire [8:0]  idx_out          // 观测/调试用：本拍用的表项下标
);
    localparam integer DEPTH = IDX_TOP - IDX_BASE + 1;   // 431

    reg signed [11:0] rom_lh [0:DEPTH-1];
    reg signed [11:0] rom_lk [0:DEPTH-1];
    reg signed [11:0] rom_rh [0:DEPTH-1];
    reg signed [11:0] rom_rk [0:DEPTH-1];

    initial begin
        $readmemh(LH_FILE, rom_lh);
        $readmemh(LK_FILE, rom_lk);
        $readmemh(RH_FILE, rom_rh);
        $readmemh(RK_FILE, rom_rk);
    end

    // 端点夹取（对应训练侧 clamp(h, min_reference_height, 0.19)），再减去基址。
    // 先减后截位：clamped 最大 778（10 位），直接截 9 位会回绕。
    wire [23:0] clamped = (height_q12 < IDX_BASE) ? IDX_BASE :
                          (height_q12 > IDX_TOP)  ? IDX_TOP  : height_q12;
    wire [23:0] delta   = clamped - IDX_BASE;            // 0 .. DEPTH-1
    wire [8:0]  idx     = delta[8:0];
    assign idx_out = idx;

    // 寄存读出（BSRAM 推断要求），一拍延迟。相对 8ms 的推理周期可忽略。
    always @(posedge clk) begin
        ff_lh <= rom_lh[idx];
        ff_lk <= rom_lk[idx];
        ff_rh <= rom_rh[idx];
        ff_rk <= rom_rk[idx];
    end
endmodule
`endif
