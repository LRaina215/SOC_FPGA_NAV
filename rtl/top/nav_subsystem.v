// 导航子系统顶层（交付给整车顶层的唯一模块）。
//
//   雷达网口 --UDP--> xt_frame_rx -> pc_project -> pc_transform -> pc_filter -> map_subsys（代价图）
//   主机网口 <-UDP-> host_mbx（目标点/目标坐标/状态报文，固件解析）
//   traj_eval（候选轨迹评估）、safety_guard（限幅/看门狗/急停）-> 底盘速度指令
//   imu_uart（IMU 原始字节 + 时间戳）
//   软核经 bus_* 访问寄存器（config/regmap.yaml），irq 为电平中断。
//
// 总线：bus_req 保持到 bus_ack（单拍）；ack 后一拍内可撤销 req。按字访问，addr 为字节地址。
module nav_subsystem #(
    parameter integer CLK_HZ = 100000000,  // 核心时钟频率（须为 1 MHz 整数倍；串口分频等按此换算）
    parameter SIN_INIT = "sin_lut.hex"
) (
    input  wire        clk,             // 核心时钟 100 MHz
    input  wire        rst,

    // 软核总线
    input  wire        bus_req,
    input  wire        bus_we,
    input  wire [15:0] bus_addr,
    input  wire [31:0] bus_wdata,
    output reg         bus_ack,
    output reg  [31:0] bus_rdata,
    output wire        irq,

    // 雷达网口 MAC 客户端（RJ45 / TSE_MAC）
    input  wire        l_rx_clk, input wire l_rx_rst,
    input  wire        l_rx_valid, input wire [7:0] l_rx_data, input wire l_rx_last, input wire l_rx_error,
    input  wire        l_tx_clk, input wire l_tx_rst,
    output wire        l_tx_valid, output wire [7:0] l_tx_data, output wire l_tx_last, input wire l_tx_ready,
    input  wire [31:0] l_phy_stat,      // 板级 MAC / PHY 状态（rgmii_mac100），寄存器 NETL.PHY_STAT
    output wire        l_rx_neg,        // 板级 RGMII 接收采样沿（NETL.CTRL.RX_NEG），clk 域
    // 主机网口 MAC 客户端（SFP / 1G Serial Ethernet）
    input  wire        h_rx_clk, input wire h_rx_rst,
    input  wire        h_rx_valid, input wire [7:0] h_rx_data, input wire h_rx_last, input wire h_rx_error,
    input  wire        h_tx_clk, input wire h_tx_rst,
    output wire        h_tx_valid, output wire [7:0] h_tx_data, output wire h_tx_last, input wire h_tx_ready,

    // IMU 串口
    input  wire        imu_rx,
    output wire        imu_tx,

    // 底盘（RL 运动控制）接口
    output wire signed [15:0] ch_vx,    // mm/s
    output wire signed [15:0] ch_vy,    // mm/s（非全向底盘恒为 0）
    output wire signed [15:0] ch_wz,    // mrad/s
    output wire [7:0]  ch_mode,
    output wire        ch_stop,         // 急停
    output wire        ch_wdog,         // 指令超时
    output wire        ch_valid,        // 单拍：每 TICK_US 一次
    output wire [15:0] ch_seq,
    input  wire        odo_valid,       // 单拍：底盘回传
    input  wire signed [15:0] odo_vx,
    input  wire signed [15:0] odo_vy,
    input  wire signed [15:0] odo_wz,
    input  wire [3:0]  ext_stop         // 外部急停源（ToF 近距、急停按钮等）
);
`include "nav_regs_map.vh"

    // ================================================================ 时基
    wire [31:0] time_us;
    wire        tick_us;
    nav_timebase #(.CLK_HZ(CLK_HZ)) u_time (.clk(clk), .rst(rst), .time_us(time_us), .tick_us(tick_us));

    // ================================================================ 寄存器文件连线
    reg         rf_wr, rf_rd;
    reg  [15:0] a_q;
    reg  [31:0] wd_q;
    wire [31:0] rf_rdata;
    wire        rf_hit;

`include "nav_subsystem_rf.vh"

    // ================================================================ 雷达网口
    wire        lu_valid, lu_last, lu_err;
    wire [7:0]  lu_data;
    wire [47:0] netl_peer_mac;
    wire [31:0] netl_peer_ip;
    wire [15:0] netl_peer_port;
    net_port #(.RX_AW(11), .TX_AW(11), .PL_AW(11)) u_netl (
        .clk(clk), .rst(rst), .time_us(time_us),
        .rx_mac_clk(l_rx_clk), .rx_mac_rst(l_rx_rst), .rx_mac_valid(l_rx_valid), .rx_mac_data(l_rx_data),
        .rx_mac_last(l_rx_last), .rx_mac_error(l_rx_error),
        .tx_mac_clk(l_tx_clk), .tx_mac_rst(l_tx_rst), .tx_mac_valid(l_tx_valid), .tx_mac_data(l_tx_data),
        .tx_mac_last(l_tx_last), .tx_mac_ready(l_tx_ready),
        .cfg_mac({r_NETL_MAC_HI[15:0], r_NETL_MAC_LO}), .cfg_ip(r_NETL_IP), .cfg_port(r_NETL_UDP_PORT[15:0]),
        .cfg_enable(r_NETL_CTRL[0]), .cfg_any_ip(r_NETL_CTRL[1]), .cfg_arp_en(r_NETL_CTRL[2]), .cfg_learn(r_NETL_CTRL[3]),
        .cfg_mcast_ip(r_NETL_MCAST_IP),
        .peer_we(p_NETL_PEER_PORT), .peer_mac_w({r_NETL_PEER_MAC_HI[15:0], r_NETL_PEER_MAC_LO}),
        .peer_ip_w(r_NETL_PEER_IP), .peer_port_w(wd_q[15:0]),
        .peer_mac(netl_peer_mac), .peer_ip(netl_peer_ip), .peer_port(netl_peer_port),
        .u_valid(lu_valid), .u_data(lu_data), .u_last(lu_last), .u_err(lu_err),
        .u_src_mac(), .u_src_ip(), .u_src_port(), .u_len(), .u_ts(),
        .tx_go(1'b0), .tx_len(16'd0), .tx_busy(), .pl_re(), .pl_addr(), .pl_q(8'd0),
        .st_rx_frames(i_NETL_STAT_RX_FRAMES), .st_rx_udp(i_NETL_STAT_RX_UDP), .st_rx_err(i_NETL_STAT_RX_ERR),
        .st_arp(i_NETL_STAT_ARP), .st_tx_frames(i_NETL_STAT_TX_FRAMES), .st_rx_drop()
    );
    assign i_NETL_PEER_IP     = netl_peer_ip;
    assign i_NETL_PEER_MAC_LO = netl_peer_mac[31:0];
    assign i_NETL_PEER_MAC_HI = {16'd0, netl_peer_mac[47:32]};
    assign i_NETL_PEER_PORT   = {16'd0, netl_peer_port};
    assign i_NETL_PHY_STAT    = l_phy_stat;
    assign l_rx_neg           = r_NETL_CTRL[4];

    // ================================================================ 主机网口 + 邮箱
    wire        hu_valid, hu_last, hu_err;
    wire [7:0]  hu_data;
    wire [31:0] hu_src_ip, hu_ts;
    wire [15:0] hu_src_port, hu_len;
    wire [47:0] neth_peer_mac;
    wire [31:0] neth_peer_ip;
    wire [15:0] neth_peer_port;
    wire        h_tx_go, h_tx_busy, h_pl_re;
    wire [15:0] h_tx_len;
    wire [10:0] h_pl_addr;
    wire [7:0]  h_pl_q;
    net_port #(.RX_AW(12), .TX_AW(11), .PL_AW(11)) u_neth (
        .clk(clk), .rst(rst), .time_us(time_us),
        .rx_mac_clk(h_rx_clk), .rx_mac_rst(h_rx_rst), .rx_mac_valid(h_rx_valid), .rx_mac_data(h_rx_data),
        .rx_mac_last(h_rx_last), .rx_mac_error(h_rx_error),
        .tx_mac_clk(h_tx_clk), .tx_mac_rst(h_tx_rst), .tx_mac_valid(h_tx_valid), .tx_mac_data(h_tx_data),
        .tx_mac_last(h_tx_last), .tx_mac_ready(h_tx_ready),
        .cfg_mac({r_NETH_MAC_HI[15:0], r_NETH_MAC_LO}), .cfg_ip(r_NETH_IP), .cfg_port(r_NETH_UDP_PORT[15:0]),
        .cfg_enable(r_NETH_CTRL[0]), .cfg_any_ip(r_NETH_CTRL[1]), .cfg_arp_en(r_NETH_CTRL[2]), .cfg_learn(r_NETH_CTRL[3]),
        .cfg_mcast_ip(r_NETH_MCAST_IP),
        .peer_we(p_NETH_PEER_PORT), .peer_mac_w({r_NETH_PEER_MAC_HI[15:0], r_NETH_PEER_MAC_LO}),
        .peer_ip_w(r_NETH_PEER_IP), .peer_port_w(wd_q[15:0]),
        .peer_mac(neth_peer_mac), .peer_ip(neth_peer_ip), .peer_port(neth_peer_port),
        .u_valid(hu_valid), .u_data(hu_data), .u_last(hu_last), .u_err(hu_err),
        .u_src_mac(), .u_src_ip(hu_src_ip), .u_src_port(hu_src_port), .u_len(hu_len), .u_ts(hu_ts),
        .tx_go(h_tx_go), .tx_len(h_tx_len), .tx_busy(h_tx_busy), .pl_re(h_pl_re), .pl_addr(h_pl_addr), .pl_q(h_pl_q),
        .st_rx_frames(i_NETH_STAT_RX_FRAMES), .st_rx_udp(i_NETH_STAT_RX_UDP), .st_rx_err(i_NETH_STAT_RX_ERR),
        .st_arp(i_NETH_STAT_ARP), .st_tx_frames(i_NETH_STAT_TX_FRAMES), .st_rx_drop()
    );
    assign i_NETH_PEER_IP     = neth_peer_ip;
    assign i_NETH_PEER_MAC_LO = neth_peer_mac[31:0];
    assign i_NETH_PEER_MAC_HI = {16'd0, neth_peer_mac[47:32]};
    assign i_NETH_PEER_PORT   = {16'd0, neth_peer_port};

    wire        mb_avail, mb_rd_ack, mb_irq;
    wire [15:0] mb_len, mb_src_port;
    wire [7:0]  mb_pkts;
    wire [31:0] mb_ts, mb_src_ip, mb_word, mb_drop;
    wire [9:0]  mb_twords;
    reg         mb_rd;
    host_mbx #(.RX_AW(12), .DQ_AW(4), .TX_WAW(9)) u_mbx (
        .clk(clk), .rst(rst),
        .e_valid(hu_valid), .e_data(hu_data), .e_last(hu_last), .e_err(hu_err), .e_src_ip(hu_src_ip),
        .e_src_port(hu_src_port), .e_len(hu_len), .e_ts(hu_ts),
        .inj_we(p_MBX_INJ_DATA), .inj_wdata(wd_q), .inj_go(p_MBX_INJ_GO), .inj_len(r_MBX_INJ_LEN[15:0]), .now_us(time_us),
        .txb_re(1'b1), .txb_addr(r_MBX_TXB_ADDR[8:0]), .txb_q(i_MBX_TXB_Q), .tx_seq(i_MBX_TX_SEQ),
        .rx_avail(mb_avail), .rx_len(mb_len), .rx_pkts(mb_pkts), .rx_ts(mb_ts), .rx_src_ip(mb_src_ip),
        .rx_src_port(mb_src_port), .rx_rd(mb_rd), .rx_rd_ack(mb_rd_ack), .rx_word(mb_word),
        .rx_pop(p_MBX_RX_POP), .rx_irq(mb_irq), .st_drop(mb_drop),
        .tx_we(p_MBX_TX_DATA), .tx_wdata(wd_q), .tx_go_in(p_MBX_TX_GO), .tx_len_in(r_MBX_TX_LEN[15:0]),
        .tx_words(mb_twords),
        .tx_go(h_tx_go), .tx_len(h_tx_len), .tx_busy(h_tx_busy), .pl_re(h_pl_re), .pl_addr(h_pl_addr), .pl_q(h_pl_q)
    );
    assign i_MBX_RX_STAT     = {mb_avail, 7'd0, mb_pkts, mb_len};
    assign i_MBX_RX_DATA     = mb_word;
    assign i_MBX_RX_TS       = mb_ts;
    assign i_MBX_RX_SRC_IP   = mb_src_ip;
    assign i_MBX_RX_SRC_PORT = {16'd0, mb_src_port};
    assign i_MBX_TX_STAT     = {16'd0, 2'd0, mb_twords, 3'd0, h_tx_busy};

    // ================================================================ 雷达点云链路
    wire        x_valid, x_ready, x_sof, x_eof, frm_start, frm_ready, pose_wait;
    wire [7:0]  x_u, x_v;
    wire [15:0] x_depth, frm_sn, frm_w, frm_h;
    wire [7:0]  frm_ds, frm_pv, frm_unit, frm_roix, frm_roiy;
    wire [1:0]  frm_bin;
    wire        frm_iok;
    wire [31:0] frm_ts, xs_frames, xs_bad, xs_drop, xs_skip;
    wire [287:0] inf_lens;
    xt_frame_rx #(.CLK_HZ(CLK_HZ)) u_xt (
        .clk(clk), .rst(rst), .time_us(time_us),
        .cfg_enable(r_LIDAR_CTRL[0]), .cfg_hdr_le(r_LIDAR_CTRL[1]), .cfg_pix_le(r_LIDAR_CTRL[2]),
        .cfg_subhdr(r_LIDAR_SUBHDR[7:0]), .cfg_amp_min(r_LIDAR_VALID_CFG[15:0]),
        .cfg_raw_inv(r_LIDAR_VALID_CFG[31:16]), .cfg_wait_pose(r_LIDAR_CTRL[3]), .pose_commit(p_LIDAR_POSE_COMMIT),
        .s_valid(lu_valid), .s_data(lu_data), .s_last(lu_last), .s_err(lu_err),
        .m_valid(x_valid), .m_ready(x_ready), .m_u(x_u), .m_v(x_v), .m_depth(x_depth), .m_sof(x_sof), .m_eof(x_eof),
        .frm_sn(frm_sn), .frm_w(frm_w), .frm_h(frm_h), .frm_ts(frm_ts), .frm_devstate(frm_ds), .frm_proto(frm_pv),
        .frm_iok(frm_iok), .frm_unit(frm_unit), .frm_bin(frm_bin), .frm_roix(frm_roix), .frm_roiy(frm_roiy),
        .pose_waiting(pose_wait), .frame_start(frm_start), .frame_ready(frm_ready),
        .inf_lens(inf_lens), .inf_cfg(i_LIDAR_INFO_CFG), .inf_size(i_LIDAR_INFO_SIZE), .inf_temp(i_LIDAR_INFO_TEMP),
        .inf_seq(i_LIDAR_INFO_SEQ),
        .stat_frames(xs_frames), .stat_bad(xs_bad), .stat_drop(xs_drop), .stat_skip(xs_skip)
    );
    assign i_LIDAR_STAT_FRAMES = xs_frames;
    assign i_LIDAR_STAT_BAD    = xs_bad;
    assign i_LIDAR_STAT_DROP   = xs_drop;
    assign i_LIDAR_STAT_SKIP   = xs_skip;
    assign i_LIDAR_LAST_FRAME  = {frm_pv, frm_ds, frm_sn};
    assign i_LIDAR_LAST_WH     = {frm_h, frm_w};
    assign i_LIDAR_LAST_TS     = frm_ts;
    assign i_LIDAR_POSE_STAT   = {31'd0, pose_wait};
    assign i_LIDAR_LENS0 = inf_lens[31:0];    assign i_LIDAR_LENS1 = inf_lens[63:32];
    assign i_LIDAR_LENS2 = inf_lens[95:64];   assign i_LIDAR_LENS3 = inf_lens[127:96];
    assign i_LIDAR_LENS4 = inf_lens[159:128]; assign i_LIDAR_LENS5 = inf_lens[191:160];
    assign i_LIDAR_LENS6 = inf_lens[223:192]; assign i_LIDAR_LENS7 = inf_lens[255:224];
    assign i_LIDAR_LENS8 = inf_lens[287:256];

    // 帧尾 FrameInfo 给出的深度单位、ROI、像素合并（AUTO_INFO=1 且本帧带有效 FrameInfo 时）优先于寄存器。
    // 打一拍（切断到方向表地址 / 片选的长路径）：frm_* 在帧进入等位姿状态时更新，至少 2 拍后才出第一个像素
    wire        use_inf = r_LIDAR_CTRL[4] && frm_iok;
    reg  [15:0] pp_dmul;
    reg  [7:0]  pp_roix, pp_roiy;
    reg  [1:0]  pp_bin;
    always @(posedge clk) begin
        pp_dmul <= (use_inf && frm_unit != 8'd0) ? {frm_unit, 8'd0} : r_LIDAR_DEPTH_CFG[15:0];
        pp_roix <= use_inf ? frm_roix : r_LIDAR_ROI_OFS[7:0];
        pp_roiy <= use_inf ? frm_roiy : r_LIDAR_ROI_OFS[15:8];
        pp_bin  <= use_inf ? frm_bin : 2'b00;
    end

    // 查找表写口：LUT_ADDR 设起点，写 LUT_XY 暂存，写 LUT_Z 提交并自增
    reg [13:0] lut_ptr;
    reg [23:0] lut_xy;
    always @(posedge clk) begin
        if (rst) lut_ptr <= 0;
        else if (rf_wr && a_q == RA_LIDAR_LUT_ADDR) lut_ptr <= wd_q[13:0];
        else if (p_LIDAR_LUT_Z) lut_ptr <= lut_ptr + 1'b1;
        if (p_LIDAR_LUT_XY) lut_xy <= wd_q[23:0];
    end

    wire        pp_valid, pp_ready, pp_pv, pp_sof, pp_eof;
    wire signed [15:0] pp_x, pp_y, pp_z;
    pc_project u_pp (
        .clk(clk), .rst(rst), .cfg_depth_mul(pp_dmul), .cfg_min_mm(r_LIDAR_RANGE[15:0]),
        .cfg_max_mm(r_LIDAR_RANGE[31:16]), .cfg_roi_x0(pp_roix), .cfg_roi_y0(pp_roiy), .cfg_bin(pp_bin),
        .lut_we(p_LIDAR_LUT_Z), .lut_waddr(lut_ptr), .lut_wdata({wd_q[11:0], lut_xy}),
        .s_valid(x_valid), .s_ready(x_ready), .s_u(x_u), .s_v(x_v), .s_depth(x_depth), .s_sof(x_sof), .s_eof(x_eof),
        .m_valid(pp_valid), .m_ready(pp_ready), .m_x(pp_x), .m_y(pp_y), .m_z(pp_z), .m_pv(pp_pv),
        .m_sof(pp_sof), .m_eof(pp_eof)
    );
    // 每帧有效点计数
    reg [31:0] pt_cnt, pt_last;
    always @(posedge clk) begin
        if (rst) begin pt_cnt <= 0; pt_last <= 0; end
        else if (pp_valid && pp_ready) begin
            if (pp_eof) begin pt_last <= (pp_sof ? 32'd0 : pt_cnt) + pp_pv; pt_cnt <= 0; end
            else pt_cnt <= (pp_sof ? 32'd0 : pt_cnt) + pp_pv;
        end
    end
    assign i_LIDAR_STAT_POINTS = pt_last;

    wire        pt_valid, pt_ready, pt_pv, pt_sof, pt_eof;
    wire signed [31:0] pt_x, pt_y, pt_z;
    pc_transform u_pt (
        .clk(clk), .rst(rst),
        .cfg_r({r_LIDAR_R22[15:0], r_LIDAR_R21[15:0], r_LIDAR_R20[15:0], r_LIDAR_R12[15:0], r_LIDAR_R11[15:0],
                r_LIDAR_R10[15:0], r_LIDAR_R02[15:0], r_LIDAR_R01[15:0], r_LIDAR_R00[15:0]}),
        .cfg_t({r_LIDAR_TZ, r_LIDAR_TY, r_LIDAR_TX}),
        .s_valid(pp_valid), .s_ready(pp_ready), .s_x(pp_x), .s_y(pp_y), .s_z(pp_z), .s_pv(pp_pv),
        .s_sof(pp_sof), .s_eof(pp_eof),
        .m_valid(pt_valid), .m_ready(pt_ready), .m_x(pt_x), .m_y(pt_y), .m_z(pt_z), .m_pv(pt_pv),
        .m_sof(pt_sof), .m_eof(pt_eof)
    );

    wire        pf_valid, pf_ready, pf_upd, pf_sof, pf_eof;
    wire [13:0] pf_addr;
    wire [7:0]  pf_tag;
    wire signed [15:0] pf_z;
    wire signed [31:0] pf_cx, pf_cy;
    pc_filter u_pf (
        .clk(clk), .rst(rst), .cfg_inv_res(r_MAP_INV_RES[15:0]), .cfg_cx(r_MAP_CENTER_X), .cfg_cy(r_MAP_CENTER_Y),
        .cfg_zmin(r_LIDAR_ZBAND[15:0]), .cfg_zmax(r_LIDAR_ZBAND[31:16]),
        .s_valid(pt_valid), .s_ready(pt_ready), .s_x(pt_x), .s_y(pt_y), .s_z(pt_z), .s_pv(pt_pv),
        .s_sof(pt_sof), .s_eof(pt_eof),
        .m_valid(pf_valid), .m_ready(pf_ready), .m_upd(pf_upd), .m_addr(pf_addr), .m_tag(pf_tag), .m_z(pf_z),
        .m_sof(pf_sof), .m_eof(pf_eof), .m_cx(pf_cx), .m_cy(pf_cy),
        .stat_upd(i_MAP_STAT_UPD), .stat_drop(i_MAP_STAT_DROP)
    );

    // ================================================================ 地图
    wire        cm_a_re;
    wire [13:0] cm_a_addr;
    wire [7:0]  cm_a_q, cm_b_q;
    reg         cm_b_req;
    reg  [13:0] cm_b_addr;
    wire        cm_b_ack, map_busy, map_front, map_ready;
    wire [15:0] map_seq;
    wire signed [31:0] map_lcx, map_lcy;
    wire [7:0]  map_fc;
    reg         el_req, el_done;
    reg  [51:0] el_word;
    wire        el_ack;
    wire [51:0] el_q;
    map_subsys u_map (
        .clk(clk), .rst(rst), .cfg_enable(r_MAP_CTRL[0]), .cfg_auto_sweep(r_MAP_CTRL[2]),
        .cmd_sweep(p_MAP_CMD & wd_q[0]), .cmd_clear(p_MAP_CMD & wd_q[1]),
        .cfg_cx(r_MAP_CENTER_X), .cfg_cy(r_MAP_CENTER_Y),
        .cfg_age_max(r_MAP_VALID_CFG[7:0]), .cfg_cnt_min(r_MAP_VALID_CFG[11:8]), .cfg_keep_r(r_MAP_VALID_CFG[19:16]),
        .cfg_step_max(r_MAP_THR0[15:0]), .cfg_rough_max(r_MAP_THR0[31:16]),
        .cfg_slope_rise(r_MAP_THR1[15:0]), .cfg_jump(r_MAP_THR1[23:16]),
        .cfg_drop_k(r_MAP_DROP[4:0]), .cfg_drop_slope(r_MAP_DROP[15:8]),
        .cfg_k_slope(r_MAP_WEIGHT[7:0]), .cfg_k_step(r_MAP_WEIGHT[15:8]), .cfg_k_rough(r_MAP_WEIGHT[23:16]),
        .cfg_fill_min(r_MAP_FILL[3:0]), .cfg_fill_tol(r_MAP_FILL[15:8]), .cfg_fill_cost(r_MAP_FILL[23:16]),
        .cfg_infl_r(r_MAP_INFL[3:0]), .cfg_infl_disk(r_MAP_INFL[4]),
        .s_valid(pf_valid), .s_ready(pf_ready), .s_upd(pf_upd), .s_addr(pf_addr), .s_tag(pf_tag), .s_z(pf_z),
        .s_eof(pf_eof), .s_cx(pf_cx), .s_cy(pf_cy),
        .a_re(cm_a_re), .a_addr(cm_a_addr), .a_q(cm_a_q),
        .b_req(cm_b_req), .b_addr(cm_b_addr), .b_ack(cm_b_ack), .b_q(cm_b_q),
        .e_req(el_req), .e_addr(r_MAP_ELEV_ADDR[13:0]), .e_ack(el_ack), .e_q(el_q),
        .busy(map_busy), .front(map_front), .seq(map_seq), .lat_cx(map_lcx), .lat_cy(map_lcy),
        .frame_cnt(map_fc), .map_ready(map_ready)
    );
    assign i_MAP_STAT      = {map_seq, 14'd0, map_front, map_busy};
    // 高程图读：ELEV_CMD.READ 发起，完成后锁存到 ELEV_Q0/Q1 并置 DONE
    always @(posedge clk) begin
        if (rst) begin el_req <= 1'b0; el_done <= 1'b0; el_word <= 52'd0; end
        else if (p_MAP_ELEV_CMD & wd_q[0]) begin el_req <= 1'b1; el_done <= 1'b0; end
        else if (el_ack) begin el_req <= 1'b0; el_done <= 1'b1; el_word <= el_q; end
    end
    assign i_MAP_ELEV_Q0 = el_word[31:0];
    assign i_MAP_ELEV_Q1 = {el_done, 11'd0, el_word[51:32]};
    assign i_MAP_LAT_CX    = map_lcx;
    assign i_MAP_LAT_CY    = map_lcy;
    assign i_MAP_FRAME_CNT = {24'd0, map_fc};

    // ================================================================ SLF 匹配定位
    wire        mt_busy, mt_ready, mt_eval_done, mt_dbg_ack;
    wire [12:0] mt_npts;
    wire [17:0] mt_dbg_q;
    wire [31:0] mt_res_q;
    reg         mt_res_re, mt_cand_we, mt_eval_st, mt_dbg_req, mt_dbg_done;
    reg  [8:0]  mt_res_addr, mt_cand_addr;
    reg  [17:0] mt_dbg_word;
    wire signed [31:0] mt_zref = $signed(r_LIDAR_TZ) - $signed(r_MATCH_MOUNT_Z);
    // 高度带无效（zmin > zmax，姿态突变跳帧）的帧不提取结构点
    wire        mt_frame_en = r_MATCH_CTRL[0] && ($signed(r_LIDAR_ZBAND[15:0]) <= $signed(r_LIDAR_ZBAND[31:16]));
    slf_match #(.SIN_INIT(SIN_INIT)) u_match (
        .clk(clk), .rst(rst),
        .s_fire(pt_valid & pt_ready), .s_x(pt_x), .s_y(pt_y), .s_z(pt_z), .s_pv(pt_pv), .s_sof(pt_sof), .s_eof(pt_eof),
        .frame_en(mt_frame_en),
        .cfg_zlo(mt_zref + {{16{r_MATCH_ZREL[15]}}, r_MATCH_ZREL[15:0]}),
        .cfg_zhi(mt_zref + {{16{r_MATCH_ZREL[31]}}, r_MATCH_ZREL[31:16]}),
        .cfg_tx(r_LIDAR_TX), .cfg_ty(r_LIDAR_TY), .cfg_inv_res(r_MAP_INV_RES[15:0]),
        .cmd_eval(p_MATCH_CMD & wd_q[0]), .cmd_update(p_MATCH_CMD & wd_q[1]), .cmd_clear(p_MATCH_CMD & wd_q[2]),
        .cfg_cx_mm(r_MATCH_CX_MM), .cfg_cy_mm(r_MATCH_CY_MM), .cfg_ncand(r_MATCH_NCAND[8:0]),
        .upd_dx(r_MATCH_UPD_DX), .upd_dy(r_MATCH_UPD_DY), .upd_dyaw(r_MATCH_UPD_DYAW[15:0]),
        .lf_cx(r_MAP_CENTER_X), .lf_cy(r_MAP_CENTER_Y),
        .cfg_prior(r_MATCH_CTRL[1]), .cfg_nogate(r_MATCH_CTRL[2]), .cfg_lfo_x(r_MATCH_LFO_X), .cfg_lfo_y(r_MATCH_LFO_Y),
        .pw_we(p_MATCH_SM_WDATA), .pw_addr(r_MATCH_SM_ADDR[15:0]), .pw_data(wd_q[17:0]),
        .cand_we(mt_cand_we), .cand_waddr(mt_cand_addr), .cand_wdata(wd_q),
        .res_re(mt_res_re), .res_raddr(mt_res_addr), .res_q(mt_res_q),
        .dbg_req(mt_dbg_req), .dbg_addr(r_MATCH_DBG_ADDR[15:0]), .dbg_ack(mt_dbg_ack), .dbg_q(mt_dbg_q),
        .busy(mt_busy), .pts_ready(mt_ready), .n_pts(mt_npts), .eval_done(mt_eval_done)
    );
    always @(posedge clk) begin
        if (rst) begin mt_eval_st <= 1'b0; mt_dbg_req <= 1'b0; mt_dbg_done <= 1'b0; end
        else begin
            if (p_MATCH_CMD & wd_q[0]) mt_eval_st <= 1'b0;
            else if (mt_eval_done) mt_eval_st <= 1'b1;
            if (p_MATCH_DBG_CMD & wd_q[0]) begin mt_dbg_req <= 1'b1; mt_dbg_done <= 1'b0; end
            else if (mt_dbg_ack) begin mt_dbg_req <= 1'b0; mt_dbg_done <= 1'b1; mt_dbg_word <= mt_dbg_q; end
        end
    end
    assign i_MATCH_STAT  = {3'd0, mt_npts, 13'd0, mt_eval_st, mt_ready, mt_busy};
    assign i_MATCH_DBG_Q = {mt_dbg_done, 13'd0, mt_dbg_word};

    // ================================================================ 轨迹评估
    wire        tr_busy, tr_done, tr_stale;
    reg         tr_res_re, tr_cand_we;
    reg  [7:0]  tr_res_addr;
    reg  [6:0]  tr_cand_addr;
    wire [31:0] tr_res_q;
    reg         tr_done_s;
    traj_eval #(.SIN_INIT(SIN_INIT)) u_tr (
        .clk(clk), .rst(rst), .start(p_TRAJ_CMD & wd_q[0]),
        .cfg_k(r_TRAJ_CFG[7:0]), .cfg_t(r_TRAJ_CFG[15:8]), .cfg_x0(r_TRAJ_X0), .cfg_y0(r_TRAJ_Y0),
        .cfg_th0(r_TRAJ_TH0[15:0]), .cfg_cmax(r_TRAJ_CMAX[7:0]),
        .map_cx(map_lcx), .map_cy(map_lcy), .map_seq(map_seq),
        .busy(tr_busy), .done(tr_done), .stale(tr_stale),
        .cand_we(tr_cand_we), .cand_addr(tr_cand_addr), .cand_wdata(wd_q),
        .res_re(tr_res_re), .res_addr(tr_res_addr), .res_rdata(tr_res_q),
        .cm_re(cm_a_re), .cm_addr(cm_a_addr), .cm_q(cm_a_q)
    );
    always @(posedge clk) begin
        if (rst || (p_TRAJ_CMD & wd_q[0])) tr_done_s <= 1'b0;
        else if (tr_done) tr_done_s <= 1'b1;
    end
    assign i_TRAJ_STAT = {29'd0, tr_stale, tr_done_s, tr_busy};

    // ================================================================ 安全限幅与底盘
    wire [3:0] ext_act;
    safety_guard u_sg (
        .clk(clk), .rst(rst), .time_us(time_us), .tick_us(tick_us),
        .cfg_enable(r_SAFE_CTRL[0]), .cfg_holonomic(r_SAFE_CTRL[1]), .cfg_estop(r_SAFE_CTRL[2]),
        .cfg_vx_max(r_SAFE_LIM_V[15:0]), .cfg_vy_max(r_SAFE_LIM_V[31:16]), .cfg_wz_max(r_SAFE_LIM_W[15:0]),
        .cfg_dv(r_SAFE_LIM_A[15:0]), .cfg_dw(r_SAFE_LIM_A[31:16]), .cfg_tick_us(r_SAFE_TICK_US),
        .cfg_wd_us(r_SAFE_WD_US), .cfg_ext_mask(r_SAFE_EXT_MASK[3:0]), .ext_stop(ext_stop),
        .cmd_we(rf_wr && a_q == RA_SAFE_CMD_W),
        .cmd_vx(r_SAFE_CMD_V[15:0]), .cmd_vy(r_SAFE_CMD_V[31:16]),
        .cmd_wz(wd_q[15:0]), .cmd_mode(wd_q[23:16]),
        .out_vx(ch_vx), .out_vy(ch_vy), .out_wz(ch_wz), .out_mode(ch_mode), .out_stop(ch_stop), .out_wdog(ch_wdog),
        .out_valid(ch_valid), .out_seq(ch_seq), .ext_active(ext_act)
    );
    assign i_SAFE_OUT_V = {ch_vy, ch_vx};
    assign i_SAFE_OUT_W = {8'd0, ch_mode, ch_wz};
    assign i_SAFE_STAT  = {ch_seq, 8'd0, ext_act, 2'd0, ch_stop, ch_wdog};

    reg [31:0] odo_v, odo_w, odo_ts, odo_cnt;
    always @(posedge clk) begin
        if (rst) begin odo_v <= 0; odo_w <= 0; odo_ts <= 0; odo_cnt <= 0; end
        else if (odo_valid) begin
            odo_v <= {odo_vy, odo_vx}; odo_w <= {16'd0, odo_wz}; odo_ts <= time_us; odo_cnt <= odo_cnt + 1'b1;
        end
    end
    assign i_SAFE_ODO_V = odo_v;
    assign i_SAFE_ODO_W = odo_w;
    assign i_SAFE_ODO_TS = odo_ts;
    assign i_SAFE_ODO_CNT = odo_cnt;

    // ================================================================ IMU 串口
    wire        iu_valid, iu_first, iu_tsv, iu_irq, iu_busy, iu_full;
    wire [7:0]  iu_data;
    wire [9:0]  iu_cnt;
    wire [31:0] iu_ts, iu_ovf;
    imu_uart #(.FIFO_AW(8), .TS_AW(5)) u_imu (
        .clk(clk), .rst(rst), .time_us(time_us), .tick_us(tick_us),
        .cfg_div(r_IMU_BAUD_DIV[15:0]), .cfg_idle_us(r_IMU_IDLE_US[15:0]),
        .uart_rx(imu_rx), .uart_tx(imu_tx),
        .rx_valid(iu_valid), .rx_data(iu_data), .rx_first(iu_first), .rx_pop(p_IMU_RX_DATA & iu_valid),
        .rx_count(iu_cnt), .ts_valid(iu_tsv), .ts_data(iu_ts), .ts_pop(p_IMU_RX_TS & iu_tsv),
        .st_ovf(iu_ovf), .irq(iu_irq),
        .tx_we(p_IMU_TX_DATA), .tx_data(wd_q[7:0]), .tx_busy(iu_busy), .tx_full(iu_full)
    );
    assign i_IMU_RX_DATA  = {iu_valid, 22'd0, iu_first, iu_data};
    assign i_IMU_RX_COUNT = {22'd0, iu_cnt};
    assign i_IMU_RX_TS    = iu_tsv ? iu_ts : 32'd0;
    assign i_IMU_TX_STAT  = {30'd0, iu_full, iu_busy};
    assign i_IMU_STAT_OVF = iu_ovf;

    // ================================================================ 系统与中断
    assign i_SYS_ID      = RR_SYS_ID;
    assign i_SYS_VERSION = RR_SYS_VERSION;
    assign i_SYS_TIME_US = time_us;
    reg [31:0] drop_d;
    reg        wdog_d;
    always @(posedge clk) begin drop_d <= xs_drop; wdog_d <= ch_wdog; end
    assign s_SYS_IRQ_STATUS = {25'd0, (xs_drop != drop_d), (ch_wdog & ~wdog_d), iu_irq, mb_irq, tr_done,
                               map_ready, frm_start | (frm_ready & r_LIDAR_CTRL[3])};
    assign irq = |(r_SYS_IRQ_STATUS & r_SYS_IRQ_ENABLE);

    // ================================================================ 总线状态机
    localparam [3:0] B_IDLE = 4'd0, B_RF = 4'd1, B_CM = 4'd2, B_TRR = 4'd3, B_MBX = 4'd4, B_END = 4'd5, B_TRR2 = 4'd6,
                     B_MRR = 4'd7, B_MRR2 = 4'd8;
    reg [3:0] bs;
    reg [1:0] cm_i;
    wire in_cm   = (bus_addr >= RW_COSTMAP_BASE) && (bus_addr < RW_COSTMAP_BASE + RW_COSTMAP_SIZE);
    wire in_res  = (bus_addr >= RW_TRAJ_RES_BASE) && (bus_addr < RW_TRAJ_RES_BASE + RW_TRAJ_RES_SIZE);
    wire in_cand = (bus_addr >= RW_TRAJ_CAND_BASE) && (bus_addr < RW_TRAJ_CAND_BASE + RW_TRAJ_CAND_SIZE);
    // 窗口内偏移（窗口基址不一定按窗口大小对齐）
    wire [15:0] off_cm   = bus_addr - RW_COSTMAP_BASE;
    wire [15:0] off_res  = bus_addr - RW_TRAJ_RES_BASE;
    wire [15:0] off_cand = bus_addr - RW_TRAJ_CAND_BASE;
    wire in_mres  = (bus_addr >= RW_MATCH_RES_BASE) && (bus_addr < RW_MATCH_RES_BASE + RW_MATCH_RES_SIZE);
    wire in_mcand = (bus_addr >= RW_MATCH_CAND_BASE) && (bus_addr < RW_MATCH_CAND_BASE + RW_MATCH_CAND_SIZE);
    wire [15:0] off_mres  = bus_addr - RW_MATCH_RES_BASE;
    wire [15:0] off_mcand = bus_addr - RW_MATCH_CAND_BASE;

    always @(posedge clk) begin
        bus_ack <= 1'b0; rf_wr <= 1'b0; rf_rd <= 1'b0; tr_res_re <= 1'b0; tr_cand_we <= 1'b0; mb_rd <= 1'b0;
        mt_res_re <= 1'b0; mt_cand_we <= 1'b0;
        if (rst) begin
            bs <= B_IDLE; cm_b_req <= 1'b0;
        end else begin
            case (bs)
            B_IDLE: if (bus_req) begin
                a_q <= {bus_addr[15:2], 2'b00}; wd_q <= bus_wdata;
                if (in_cm) begin
                    if (bus_we) begin bus_ack <= 1'b1; bus_rdata <= 32'd0; bs <= B_END; end
                    else begin
                        cm_i <= 0; cm_b_req <= 1'b1;
                        cm_b_addr <= {off_cm[13:2], 2'b00};
                        bs <= B_CM;
                    end
                end else if (in_res) begin
                    if (bus_we) begin bus_ack <= 1'b1; bus_rdata <= 32'd0; bs <= B_END; end
                    else begin tr_res_re <= 1'b1; tr_res_addr <= off_res[9:2]; bs <= B_TRR; end
                end else if (in_cand) begin
                    if (bus_we) begin tr_cand_we <= 1'b1; tr_cand_addr <= off_cand[8:2]; end
                    bus_ack <= 1'b1; bus_rdata <= 32'd0; bs <= B_END;
                end else if (in_mres) begin
                    if (bus_we) begin bus_ack <= 1'b1; bus_rdata <= 32'd0; bs <= B_END; end
                    else begin mt_res_re <= 1'b1; mt_res_addr <= off_mres[10:2]; bs <= B_MRR; end
                end else if (in_mcand) begin
                    if (bus_we) begin mt_cand_we <= 1'b1; mt_cand_addr <= off_mcand[10:2]; end
                    bus_ack <= 1'b1; bus_rdata <= 32'd0; bs <= B_END;
                end else if (!bus_we && {bus_addr[15:2], 2'b00} == RA_MBX_RX_DATA) begin
                    mb_rd <= 1'b1; bs <= B_MBX;
                end else begin
                    if (bus_we) rf_wr <= 1'b1; else rf_rd <= 1'b1;
                    bs <= B_RF;
                end
            end
            B_RF: begin
                bus_rdata <= rf_rdata; bus_ack <= 1'b1; bs <= B_END;
            end
            B_CM: if (cm_b_ack) begin
                bus_rdata[8*cm_i +: 8] <= cm_b_q;
                if (cm_i == 2'd3) begin cm_b_req <= 1'b0; bus_ack <= 1'b1; bs <= B_END; end
                else begin cm_i <= cm_i + 1'b1; cm_b_addr <= cm_b_addr + 1'b1; end
            end
            B_TRR: bs <= B_TRR2;
            B_TRR2: begin bus_rdata <= tr_res_q; bus_ack <= 1'b1; bs <= B_END; end
            B_MRR: bs <= B_MRR2;
            B_MRR2: begin bus_rdata <= mt_res_q; bus_ack <= 1'b1; bs <= B_END; end
            B_MBX: if (mb_rd_ack) begin bus_rdata <= mb_word; bus_ack <= 1'b1; bs <= B_END; end
            B_END: bs <= B_IDLE;       // 给主机撤销 req 的一拍
            default: bs <= B_IDLE;
            endcase
        end
    end
endmodule
