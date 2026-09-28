// 自动生成，勿手改。来源: config/regmap.yaml，生成器: tools/gen_config.py
    wire [31:0] i_SYS_ID;
    wire [31:0] i_SYS_VERSION;
    wire [31:0] i_SYS_TIME_US;
    wire [31:0] r_SYS_SCRATCH;
    wire [31:0] r_SYS_IRQ_STATUS;
    wire [31:0] s_SYS_IRQ_STATUS;
    wire [31:0] r_SYS_IRQ_ENABLE;
    wire [31:0] r_NETL_MAC_LO;
    wire [31:0] r_NETL_MAC_HI;
    wire [31:0] r_NETL_IP;
    wire [31:0] r_NETL_UDP_PORT;
    wire [31:0] r_NETL_CTRL;
    wire [31:0] i_NETL_STAT_RX_FRAMES;
    wire [31:0] i_NETL_STAT_RX_UDP;
    wire [31:0] i_NETL_STAT_RX_ERR;
    wire [31:0] i_NETL_STAT_ARP;
    wire [31:0] i_NETL_STAT_TX_FRAMES;
    wire [31:0] r_NETL_PEER_IP;
    wire [31:0] i_NETL_PEER_IP;
    wire        p_NETL_PEER_IP;
    wire [31:0] r_NETL_PEER_MAC_LO;
    wire [31:0] i_NETL_PEER_MAC_LO;
    wire        p_NETL_PEER_MAC_LO;
    wire [31:0] r_NETL_PEER_MAC_HI;
    wire [31:0] i_NETL_PEER_MAC_HI;
    wire        p_NETL_PEER_MAC_HI;
    wire [31:0] r_NETL_PEER_PORT;
    wire [31:0] i_NETL_PEER_PORT;
    wire        p_NETL_PEER_PORT;
    wire [31:0] r_NETH_MAC_LO;
    wire [31:0] r_NETH_MAC_HI;
    wire [31:0] r_NETH_IP;
    wire [31:0] r_NETH_UDP_PORT;
    wire [31:0] r_NETH_CTRL;
    wire [31:0] i_NETH_STAT_RX_FRAMES;
    wire [31:0] i_NETH_STAT_RX_UDP;
    wire [31:0] i_NETH_STAT_RX_ERR;
    wire [31:0] i_NETH_STAT_ARP;
    wire [31:0] i_NETH_STAT_TX_FRAMES;
    wire [31:0] r_NETH_PEER_IP;
    wire [31:0] i_NETH_PEER_IP;
    wire        p_NETH_PEER_IP;
    wire [31:0] r_NETH_PEER_MAC_LO;
    wire [31:0] i_NETH_PEER_MAC_LO;
    wire        p_NETH_PEER_MAC_LO;
    wire [31:0] r_NETH_PEER_MAC_HI;
    wire [31:0] i_NETH_PEER_MAC_HI;
    wire        p_NETH_PEER_MAC_HI;
    wire [31:0] r_NETH_PEER_PORT;
    wire [31:0] i_NETH_PEER_PORT;
    wire        p_NETH_PEER_PORT;
    wire [31:0] i_MBX_RX_STAT;
    wire [31:0] i_MBX_RX_DATA;
    wire        p_MBX_RX_DATA;
    wire        p_MBX_RX_POP;
    wire [31:0] i_MBX_RX_TS;
    wire [31:0] i_MBX_RX_SRC_IP;
    wire [31:0] i_MBX_RX_SRC_PORT;
    wire        p_MBX_TX_DATA;
    wire [31:0] r_MBX_TX_LEN;
    wire        p_MBX_TX_GO;
    wire [31:0] i_MBX_TX_STAT;
    wire [31:0] r_LIDAR_CTRL;
    wire [31:0] r_LIDAR_UDP_SKIP;
    wire [31:0] r_LIDAR_DEPTH_CFG;
    wire [31:0] r_LIDAR_RANGE;
    wire [31:0] r_LIDAR_VALID_CFG;
    wire [31:0] r_LIDAR_ROI_OFS;
    wire [31:0] i_LIDAR_STAT_FRAMES;
    wire [31:0] i_LIDAR_STAT_BAD;
    wire [31:0] i_LIDAR_LAST_FRAME;
    wire [31:0] i_LIDAR_LAST_WH;
    wire [31:0] i_LIDAR_LAST_TS;
    wire [31:0] i_LIDAR_STAT_POINTS;
    wire [31:0] i_LIDAR_STAT_DROP;
    wire [31:0] r_LIDAR_R00;
    wire [31:0] r_LIDAR_R01;
    wire [31:0] r_LIDAR_R02;
    wire [31:0] r_LIDAR_R10;
    wire [31:0] r_LIDAR_R11;
    wire [31:0] r_LIDAR_R12;
    wire [31:0] r_LIDAR_R20;
    wire [31:0] r_LIDAR_R21;
    wire [31:0] r_LIDAR_R22;
    wire [31:0] r_LIDAR_TX;
    wire [31:0] r_LIDAR_TY;
    wire [31:0] r_LIDAR_TZ;
    wire        p_LIDAR_POSE_COMMIT;
    wire [31:0] i_LIDAR_POSE_STAT;
    wire [31:0] r_LIDAR_ZBAND;
    wire [31:0] r_LIDAR_LUT_ADDR;
    wire        p_LIDAR_LUT_XY;
    wire        p_LIDAR_LUT_Z;
    wire [31:0] r_MAP_CTRL;
    wire        p_MAP_CMD;
    wire [31:0] r_MAP_INV_RES;
    wire [31:0] r_MAP_CENTER_X;
    wire [31:0] r_MAP_CENTER_Y;
    wire [31:0] r_MAP_VALID_CFG;
    wire [31:0] r_MAP_THR0;
    wire [31:0] r_MAP_THR1;
    wire [31:0] r_MAP_WEIGHT;
    wire [31:0] r_MAP_INFL;
    wire [31:0] i_MAP_STAT;
    wire [31:0] i_MAP_LAT_CX;
    wire [31:0] i_MAP_LAT_CY;
    wire [31:0] i_MAP_STAT_UPD;
    wire [31:0] i_MAP_STAT_DROP;
    wire [31:0] i_MAP_FRAME_CNT;
    wire [31:0] r_MAP_DROP;
    wire [31:0] r_MAP_FILL;
    wire        p_TRAJ_CMD;
    wire [31:0] i_TRAJ_STAT;
    wire [31:0] r_TRAJ_CFG;
    wire [31:0] r_TRAJ_X0;
    wire [31:0] r_TRAJ_Y0;
    wire [31:0] r_TRAJ_TH0;
    wire [31:0] r_TRAJ_CMAX;
    wire [31:0] r_SAFE_CTRL;
    wire [31:0] r_SAFE_LIM_V;
    wire [31:0] r_SAFE_LIM_W;
    wire [31:0] r_SAFE_LIM_A;
    wire [31:0] r_SAFE_TICK_US;
    wire [31:0] r_SAFE_WD_US;
    wire [31:0] r_SAFE_CMD_V;
    wire [31:0] r_SAFE_CMD_W;
    wire [31:0] i_SAFE_OUT_V;
    wire [31:0] i_SAFE_OUT_W;
    wire [31:0] i_SAFE_STAT;
    wire [31:0] r_SAFE_EXT_MASK;
    wire [31:0] i_SAFE_ODO_V;
    wire [31:0] i_SAFE_ODO_W;
    wire [31:0] i_SAFE_ODO_TS;
    wire [31:0] i_SAFE_ODO_CNT;
    wire [31:0] r_IMU_BAUD_DIV;
    wire [31:0] i_IMU_RX_DATA;
    wire        p_IMU_RX_DATA;
    wire [31:0] i_IMU_RX_COUNT;
    wire [31:0] i_IMU_RX_TS;
    wire        p_IMU_RX_TS;
    wire        p_IMU_TX_DATA;
    wire [31:0] i_IMU_TX_STAT;
    wire [31:0] r_IMU_IDLE_US;
    wire [31:0] i_IMU_STAT_OVF;
    nav_regfile u_rf (
        .clk(clk), .rst(rst), .wr(rf_wr), .rd(rf_rd), .addr(a_q), .wdata(wd_q),
        .rdata(rf_rdata), .hit(rf_hit),
        .i_SYS_ID(i_SYS_ID),
        .i_SYS_VERSION(i_SYS_VERSION),
        .i_SYS_TIME_US(i_SYS_TIME_US),
        .r_SYS_SCRATCH(r_SYS_SCRATCH),
        .r_SYS_IRQ_STATUS(r_SYS_IRQ_STATUS),
        .s_SYS_IRQ_STATUS(s_SYS_IRQ_STATUS),
        .r_SYS_IRQ_ENABLE(r_SYS_IRQ_ENABLE),
        .r_NETL_MAC_LO(r_NETL_MAC_LO),
        .r_NETL_MAC_HI(r_NETL_MAC_HI),
        .r_NETL_IP(r_NETL_IP),
        .r_NETL_UDP_PORT(r_NETL_UDP_PORT),
        .r_NETL_CTRL(r_NETL_CTRL),
        .i_NETL_STAT_RX_FRAMES(i_NETL_STAT_RX_FRAMES),
        .i_NETL_STAT_RX_UDP(i_NETL_STAT_RX_UDP),
        .i_NETL_STAT_RX_ERR(i_NETL_STAT_RX_ERR),
        .i_NETL_STAT_ARP(i_NETL_STAT_ARP),
        .i_NETL_STAT_TX_FRAMES(i_NETL_STAT_TX_FRAMES),
        .r_NETL_PEER_IP(r_NETL_PEER_IP),
        .i_NETL_PEER_IP(i_NETL_PEER_IP),
        .p_NETL_PEER_IP(p_NETL_PEER_IP),
        .r_NETL_PEER_MAC_LO(r_NETL_PEER_MAC_LO),
        .i_NETL_PEER_MAC_LO(i_NETL_PEER_MAC_LO),
        .p_NETL_PEER_MAC_LO(p_NETL_PEER_MAC_LO),
        .r_NETL_PEER_MAC_HI(r_NETL_PEER_MAC_HI),
        .i_NETL_PEER_MAC_HI(i_NETL_PEER_MAC_HI),
        .p_NETL_PEER_MAC_HI(p_NETL_PEER_MAC_HI),
        .r_NETL_PEER_PORT(r_NETL_PEER_PORT),
        .i_NETL_PEER_PORT(i_NETL_PEER_PORT),
        .p_NETL_PEER_PORT(p_NETL_PEER_PORT),
        .r_NETH_MAC_LO(r_NETH_MAC_LO),
        .r_NETH_MAC_HI(r_NETH_MAC_HI),
        .r_NETH_IP(r_NETH_IP),
        .r_NETH_UDP_PORT(r_NETH_UDP_PORT),
        .r_NETH_CTRL(r_NETH_CTRL),
        .i_NETH_STAT_RX_FRAMES(i_NETH_STAT_RX_FRAMES),
        .i_NETH_STAT_RX_UDP(i_NETH_STAT_RX_UDP),
        .i_NETH_STAT_RX_ERR(i_NETH_STAT_RX_ERR),
        .i_NETH_STAT_ARP(i_NETH_STAT_ARP),
        .i_NETH_STAT_TX_FRAMES(i_NETH_STAT_TX_FRAMES),
        .r_NETH_PEER_IP(r_NETH_PEER_IP),
        .i_NETH_PEER_IP(i_NETH_PEER_IP),
        .p_NETH_PEER_IP(p_NETH_PEER_IP),
        .r_NETH_PEER_MAC_LO(r_NETH_PEER_MAC_LO),
        .i_NETH_PEER_MAC_LO(i_NETH_PEER_MAC_LO),
        .p_NETH_PEER_MAC_LO(p_NETH_PEER_MAC_LO),
        .r_NETH_PEER_MAC_HI(r_NETH_PEER_MAC_HI),
        .i_NETH_PEER_MAC_HI(i_NETH_PEER_MAC_HI),
        .p_NETH_PEER_MAC_HI(p_NETH_PEER_MAC_HI),
        .r_NETH_PEER_PORT(r_NETH_PEER_PORT),
        .i_NETH_PEER_PORT(i_NETH_PEER_PORT),
        .p_NETH_PEER_PORT(p_NETH_PEER_PORT),
        .i_MBX_RX_STAT(i_MBX_RX_STAT),
        .i_MBX_RX_DATA(i_MBX_RX_DATA),
        .p_MBX_RX_DATA(p_MBX_RX_DATA),
        .p_MBX_RX_POP(p_MBX_RX_POP),
        .i_MBX_RX_TS(i_MBX_RX_TS),
        .i_MBX_RX_SRC_IP(i_MBX_RX_SRC_IP),
        .i_MBX_RX_SRC_PORT(i_MBX_RX_SRC_PORT),
        .p_MBX_TX_DATA(p_MBX_TX_DATA),
        .r_MBX_TX_LEN(r_MBX_TX_LEN),
        .p_MBX_TX_GO(p_MBX_TX_GO),
        .i_MBX_TX_STAT(i_MBX_TX_STAT),
        .r_LIDAR_CTRL(r_LIDAR_CTRL),
        .r_LIDAR_UDP_SKIP(r_LIDAR_UDP_SKIP),
        .r_LIDAR_DEPTH_CFG(r_LIDAR_DEPTH_CFG),
        .r_LIDAR_RANGE(r_LIDAR_RANGE),
        .r_LIDAR_VALID_CFG(r_LIDAR_VALID_CFG),
        .r_LIDAR_ROI_OFS(r_LIDAR_ROI_OFS),
        .i_LIDAR_STAT_FRAMES(i_LIDAR_STAT_FRAMES),
        .i_LIDAR_STAT_BAD(i_LIDAR_STAT_BAD),
        .i_LIDAR_LAST_FRAME(i_LIDAR_LAST_FRAME),
        .i_LIDAR_LAST_WH(i_LIDAR_LAST_WH),
        .i_LIDAR_LAST_TS(i_LIDAR_LAST_TS),
        .i_LIDAR_STAT_POINTS(i_LIDAR_STAT_POINTS),
        .i_LIDAR_STAT_DROP(i_LIDAR_STAT_DROP),
        .r_LIDAR_R00(r_LIDAR_R00),
        .r_LIDAR_R01(r_LIDAR_R01),
        .r_LIDAR_R02(r_LIDAR_R02),
        .r_LIDAR_R10(r_LIDAR_R10),
        .r_LIDAR_R11(r_LIDAR_R11),
        .r_LIDAR_R12(r_LIDAR_R12),
        .r_LIDAR_R20(r_LIDAR_R20),
        .r_LIDAR_R21(r_LIDAR_R21),
        .r_LIDAR_R22(r_LIDAR_R22),
        .r_LIDAR_TX(r_LIDAR_TX),
        .r_LIDAR_TY(r_LIDAR_TY),
        .r_LIDAR_TZ(r_LIDAR_TZ),
        .p_LIDAR_POSE_COMMIT(p_LIDAR_POSE_COMMIT),
        .i_LIDAR_POSE_STAT(i_LIDAR_POSE_STAT),
        .r_LIDAR_ZBAND(r_LIDAR_ZBAND),
        .r_LIDAR_LUT_ADDR(r_LIDAR_LUT_ADDR),
        .p_LIDAR_LUT_XY(p_LIDAR_LUT_XY),
        .p_LIDAR_LUT_Z(p_LIDAR_LUT_Z),
        .r_MAP_CTRL(r_MAP_CTRL),
        .p_MAP_CMD(p_MAP_CMD),
        .r_MAP_INV_RES(r_MAP_INV_RES),
        .r_MAP_CENTER_X(r_MAP_CENTER_X),
        .r_MAP_CENTER_Y(r_MAP_CENTER_Y),
        .r_MAP_VALID_CFG(r_MAP_VALID_CFG),
        .r_MAP_THR0(r_MAP_THR0),
        .r_MAP_THR1(r_MAP_THR1),
        .r_MAP_WEIGHT(r_MAP_WEIGHT),
        .r_MAP_INFL(r_MAP_INFL),
        .i_MAP_STAT(i_MAP_STAT),
        .i_MAP_LAT_CX(i_MAP_LAT_CX),
        .i_MAP_LAT_CY(i_MAP_LAT_CY),
        .i_MAP_STAT_UPD(i_MAP_STAT_UPD),
        .i_MAP_STAT_DROP(i_MAP_STAT_DROP),
        .i_MAP_FRAME_CNT(i_MAP_FRAME_CNT),
        .r_MAP_DROP(r_MAP_DROP),
        .r_MAP_FILL(r_MAP_FILL),
        .p_TRAJ_CMD(p_TRAJ_CMD),
        .i_TRAJ_STAT(i_TRAJ_STAT),
        .r_TRAJ_CFG(r_TRAJ_CFG),
        .r_TRAJ_X0(r_TRAJ_X0),
        .r_TRAJ_Y0(r_TRAJ_Y0),
        .r_TRAJ_TH0(r_TRAJ_TH0),
        .r_TRAJ_CMAX(r_TRAJ_CMAX),
        .r_SAFE_CTRL(r_SAFE_CTRL),
        .r_SAFE_LIM_V(r_SAFE_LIM_V),
        .r_SAFE_LIM_W(r_SAFE_LIM_W),
        .r_SAFE_LIM_A(r_SAFE_LIM_A),
        .r_SAFE_TICK_US(r_SAFE_TICK_US),
        .r_SAFE_WD_US(r_SAFE_WD_US),
        .r_SAFE_CMD_V(r_SAFE_CMD_V),
        .r_SAFE_CMD_W(r_SAFE_CMD_W),
        .i_SAFE_OUT_V(i_SAFE_OUT_V),
        .i_SAFE_OUT_W(i_SAFE_OUT_W),
        .i_SAFE_STAT(i_SAFE_STAT),
        .r_SAFE_EXT_MASK(r_SAFE_EXT_MASK),
        .i_SAFE_ODO_V(i_SAFE_ODO_V),
        .i_SAFE_ODO_W(i_SAFE_ODO_W),
        .i_SAFE_ODO_TS(i_SAFE_ODO_TS),
        .i_SAFE_ODO_CNT(i_SAFE_ODO_CNT),
        .r_IMU_BAUD_DIV(r_IMU_BAUD_DIV),
        .i_IMU_RX_DATA(i_IMU_RX_DATA),
        .p_IMU_RX_DATA(p_IMU_RX_DATA),
        .i_IMU_RX_COUNT(i_IMU_RX_COUNT),
        .i_IMU_RX_TS(i_IMU_RX_TS),
        .p_IMU_RX_TS(p_IMU_RX_TS),
        .p_IMU_TX_DATA(p_IMU_TX_DATA),
        .i_IMU_TX_STAT(i_IMU_TX_STAT),
        .r_IMU_IDLE_US(r_IMU_IDLE_US),
        .i_IMU_STAT_OVF(i_IMU_STAT_OVF)
    );
