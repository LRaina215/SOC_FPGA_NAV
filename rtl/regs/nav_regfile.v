// 自动生成，勿手改。来源: config/regmap.yaml，生成器: tools/gen_config.py
module nav_regfile (
    input  wire        clk,
    input  wire        rst,
    input  wire        wr,          // 单拍写
    input  wire        rd,          // 单拍读（仅用于 rop 弹出）
    input  wire [15:0] addr,
    input  wire [31:0] wdata,
    output reg  [31:0] rdata,
    output reg         hit,
    input  wire [31:0] i_SYS_ID,
    input  wire [31:0] i_SYS_VERSION,
    input  wire [31:0] i_SYS_TIME_US,
    output reg  [31:0] r_SYS_SCRATCH,
    output reg  [31:0] r_SYS_IRQ_STATUS,
    input  wire [31:0] s_SYS_IRQ_STATUS,
    output reg  [31:0] r_SYS_IRQ_ENABLE,
    output reg  [31:0] r_NETL_MAC_LO,
    output reg  [31:0] r_NETL_MAC_HI,
    output reg  [31:0] r_NETL_IP,
    output reg  [31:0] r_NETL_UDP_PORT,
    output reg  [31:0] r_NETL_CTRL,
    input  wire [31:0] i_NETL_STAT_RX_FRAMES,
    input  wire [31:0] i_NETL_STAT_RX_UDP,
    input  wire [31:0] i_NETL_STAT_RX_ERR,
    input  wire [31:0] i_NETL_STAT_ARP,
    input  wire [31:0] i_NETL_STAT_TX_FRAMES,
    output reg  [31:0] r_NETL_PEER_IP,
    output wire        p_NETL_PEER_IP,
    input  wire [31:0] i_NETL_PEER_IP,
    output reg  [31:0] r_NETL_PEER_MAC_LO,
    output wire        p_NETL_PEER_MAC_LO,
    input  wire [31:0] i_NETL_PEER_MAC_LO,
    output reg  [31:0] r_NETL_PEER_MAC_HI,
    output wire        p_NETL_PEER_MAC_HI,
    input  wire [31:0] i_NETL_PEER_MAC_HI,
    output reg  [31:0] r_NETL_PEER_PORT,
    output wire        p_NETL_PEER_PORT,
    input  wire [31:0] i_NETL_PEER_PORT,
    output reg  [31:0] r_NETL_MCAST_IP,
    input  wire [31:0] i_NETL_PHY_STAT,
    output wire        p_NETL_RAW_TX,
    input  wire [31:0] i_NETL_RAW_STAT,
    output wire        p_NETL_RAW_POP,
    output reg  [31:0] r_NETL_LIDAR_IP,
    output reg  [31:0] r_NETH_MAC_LO,
    output reg  [31:0] r_NETH_MAC_HI,
    output reg  [31:0] r_NETH_IP,
    output reg  [31:0] r_NETH_UDP_PORT,
    output reg  [31:0] r_NETH_CTRL,
    input  wire [31:0] i_NETH_STAT_RX_FRAMES,
    input  wire [31:0] i_NETH_STAT_RX_UDP,
    input  wire [31:0] i_NETH_STAT_RX_ERR,
    input  wire [31:0] i_NETH_STAT_ARP,
    input  wire [31:0] i_NETH_STAT_TX_FRAMES,
    output reg  [31:0] r_NETH_PEER_IP,
    output wire        p_NETH_PEER_IP,
    input  wire [31:0] i_NETH_PEER_IP,
    output reg  [31:0] r_NETH_PEER_MAC_LO,
    output wire        p_NETH_PEER_MAC_LO,
    input  wire [31:0] i_NETH_PEER_MAC_LO,
    output reg  [31:0] r_NETH_PEER_MAC_HI,
    output wire        p_NETH_PEER_MAC_HI,
    input  wire [31:0] i_NETH_PEER_MAC_HI,
    output reg  [31:0] r_NETH_PEER_PORT,
    output wire        p_NETH_PEER_PORT,
    input  wire [31:0] i_NETH_PEER_PORT,
    output reg  [31:0] r_NETH_MCAST_IP,
    input  wire [31:0] i_MBX_RX_STAT,
    input  wire [31:0] i_MBX_RX_DATA,
    output wire        p_MBX_RX_DATA,
    output wire        p_MBX_RX_POP,
    input  wire [31:0] i_MBX_RX_TS,
    input  wire [31:0] i_MBX_RX_SRC_IP,
    input  wire [31:0] i_MBX_RX_SRC_PORT,
    output wire        p_MBX_TX_DATA,
    output reg  [31:0] r_MBX_TX_LEN,
    output wire        p_MBX_TX_GO,
    input  wire [31:0] i_MBX_TX_STAT,
    output wire        p_MBX_INJ_DATA,
    output reg  [31:0] r_MBX_INJ_LEN,
    output wire        p_MBX_INJ_GO,
    output reg  [31:0] r_MBX_TXB_ADDR,
    input  wire [31:0] i_MBX_TXB_Q,
    input  wire [31:0] i_MBX_TX_SEQ,
    output reg  [31:0] r_LIDAR_CTRL,
    output reg  [31:0] r_LIDAR_SUBHDR,
    output reg  [31:0] r_LIDAR_DEPTH_CFG,
    output reg  [31:0] r_LIDAR_RANGE,
    output reg  [31:0] r_LIDAR_VALID_CFG,
    output reg  [31:0] r_LIDAR_ROI_OFS,
    input  wire [31:0] i_LIDAR_STAT_FRAMES,
    input  wire [31:0] i_LIDAR_STAT_BAD,
    input  wire [31:0] i_LIDAR_LAST_FRAME,
    input  wire [31:0] i_LIDAR_LAST_WH,
    input  wire [31:0] i_LIDAR_LAST_TS,
    input  wire [31:0] i_LIDAR_STAT_POINTS,
    input  wire [31:0] i_LIDAR_STAT_DROP,
    input  wire [31:0] i_LIDAR_STAT_SKIP,
    output reg  [31:0] r_LIDAR_R00,
    output reg  [31:0] r_LIDAR_R01,
    output reg  [31:0] r_LIDAR_R02,
    output reg  [31:0] r_LIDAR_R10,
    output reg  [31:0] r_LIDAR_R11,
    output reg  [31:0] r_LIDAR_R12,
    output reg  [31:0] r_LIDAR_R20,
    output reg  [31:0] r_LIDAR_R21,
    output reg  [31:0] r_LIDAR_R22,
    output reg  [31:0] r_LIDAR_TX,
    output reg  [31:0] r_LIDAR_TY,
    output reg  [31:0] r_LIDAR_TZ,
    output wire        p_LIDAR_POSE_COMMIT,
    input  wire [31:0] i_LIDAR_POSE_STAT,
    output reg  [31:0] r_LIDAR_ZBAND,
    output reg  [31:0] r_LIDAR_LUT_ADDR,
    output wire        p_LIDAR_LUT_XY,
    output wire        p_LIDAR_LUT_Z,
    input  wire [31:0] i_LIDAR_LENS0,
    input  wire [31:0] i_LIDAR_LENS1,
    input  wire [31:0] i_LIDAR_LENS2,
    input  wire [31:0] i_LIDAR_LENS3,
    input  wire [31:0] i_LIDAR_LENS4,
    input  wire [31:0] i_LIDAR_LENS5,
    input  wire [31:0] i_LIDAR_LENS6,
    input  wire [31:0] i_LIDAR_LENS7,
    input  wire [31:0] i_LIDAR_LENS8,
    input  wire [31:0] i_LIDAR_INFO_CFG,
    input  wire [31:0] i_LIDAR_INFO_SIZE,
    input  wire [31:0] i_LIDAR_INFO_TEMP,
    input  wire [31:0] i_LIDAR_INFO_SEQ,
    output reg  [31:0] r_MAP_CTRL,
    output wire        p_MAP_CMD,
    output reg  [31:0] r_MAP_INV_RES,
    output reg  [31:0] r_MAP_CENTER_X,
    output reg  [31:0] r_MAP_CENTER_Y,
    output reg  [31:0] r_MAP_VALID_CFG,
    output reg  [31:0] r_MAP_THR0,
    output reg  [31:0] r_MAP_THR1,
    output reg  [31:0] r_MAP_WEIGHT,
    output reg  [31:0] r_MAP_INFL,
    input  wire [31:0] i_MAP_STAT,
    input  wire [31:0] i_MAP_LAT_CX,
    input  wire [31:0] i_MAP_LAT_CY,
    input  wire [31:0] i_MAP_STAT_UPD,
    input  wire [31:0] i_MAP_STAT_DROP,
    input  wire [31:0] i_MAP_FRAME_CNT,
    output reg  [31:0] r_MAP_DROP,
    output reg  [31:0] r_MAP_FILL,
    output reg  [31:0] r_MAP_ELEV_ADDR,
    output wire        p_MAP_ELEV_CMD,
    input  wire [31:0] i_MAP_ELEV_Q0,
    input  wire [31:0] i_MAP_ELEV_Q1,
    output wire        p_TRAJ_CMD,
    input  wire [31:0] i_TRAJ_STAT,
    output reg  [31:0] r_TRAJ_CFG,
    output reg  [31:0] r_TRAJ_X0,
    output reg  [31:0] r_TRAJ_Y0,
    output reg  [31:0] r_TRAJ_TH0,
    output reg  [31:0] r_TRAJ_CMAX,
    output reg  [31:0] r_SAFE_CTRL,
    output reg  [31:0] r_SAFE_LIM_V,
    output reg  [31:0] r_SAFE_LIM_W,
    output reg  [31:0] r_SAFE_LIM_A,
    output reg  [31:0] r_SAFE_TICK_US,
    output reg  [31:0] r_SAFE_WD_US,
    output reg  [31:0] r_SAFE_CMD_V,
    output reg  [31:0] r_SAFE_CMD_W,
    input  wire [31:0] i_SAFE_OUT_V,
    input  wire [31:0] i_SAFE_OUT_W,
    input  wire [31:0] i_SAFE_STAT,
    output reg  [31:0] r_SAFE_EXT_MASK,
    input  wire [31:0] i_SAFE_ODO_V,
    input  wire [31:0] i_SAFE_ODO_W,
    input  wire [31:0] i_SAFE_ODO_TS,
    input  wire [31:0] i_SAFE_ODO_CNT,
    output reg  [31:0] r_IMU_BAUD_DIV,
    input  wire [31:0] i_IMU_RX_DATA,
    output wire        p_IMU_RX_DATA,
    input  wire [31:0] i_IMU_RX_COUNT,
    input  wire [31:0] i_IMU_RX_TS,
    output wire        p_IMU_RX_TS,
    output wire        p_IMU_TX_DATA,
    input  wire [31:0] i_IMU_TX_STAT,
    output reg  [31:0] r_IMU_IDLE_US,
    input  wire [31:0] i_IMU_STAT_OVF,
    output reg  [31:0] r_MATCH_CTRL,
    output reg  [31:0] r_MATCH_ZREL,
    output reg  [31:0] r_MATCH_MOUNT_Z,
    output wire        p_MATCH_CMD,
    input  wire [31:0] i_MATCH_STAT,
    output reg  [31:0] r_MATCH_CX_MM,
    output reg  [31:0] r_MATCH_CY_MM,
    output reg  [31:0] r_MATCH_NCAND,
    output reg  [31:0] r_MATCH_UPD_DX,
    output reg  [31:0] r_MATCH_UPD_DY,
    output reg  [31:0] r_MATCH_UPD_DYAW,
    output reg  [31:0] r_MATCH_DBG_ADDR,
    output wire        p_MATCH_DBG_CMD,
    input  wire [31:0] i_MATCH_DBG_Q,
    output reg  [31:0] r_MATCH_LFO_X,
    output reg  [31:0] r_MATCH_LFO_Y,
    output reg  [31:0] r_MATCH_SM_ADDR,
    output wire        p_MATCH_SM_WDATA
);
    always @(posedge clk) begin
        if (rst) r_SYS_IRQ_STATUS <= 32'h0;
        else r_SYS_IRQ_STATUS <= (r_SYS_IRQ_STATUS & ~((wr && addr == 16'h0010) ? wdata : 32'h0)) | s_SYS_IRQ_STATUS;
    end
    assign p_NETL_PEER_IP = wr && (addr == 16'h1028);
    assign p_NETL_PEER_MAC_LO = wr && (addr == 16'h102C);
    assign p_NETL_PEER_MAC_HI = wr && (addr == 16'h1030);
    assign p_NETL_PEER_PORT = wr && (addr == 16'h1034);
    assign p_NETL_RAW_TX = wr && (addr == 16'h1040);
    assign p_NETL_RAW_POP = wr && (addr == 16'h1048);
    assign p_NETH_PEER_IP = wr && (addr == 16'h1128);
    assign p_NETH_PEER_MAC_LO = wr && (addr == 16'h112C);
    assign p_NETH_PEER_MAC_HI = wr && (addr == 16'h1130);
    assign p_NETH_PEER_PORT = wr && (addr == 16'h1134);
    assign p_MBX_RX_DATA = rd && (addr == 16'h1204);
    assign p_MBX_RX_POP = wr && (addr == 16'h1208);
    assign p_MBX_TX_DATA = wr && (addr == 16'h1220);
    assign p_MBX_TX_GO = wr && (addr == 16'h1228);
    assign p_MBX_INJ_DATA = wr && (addr == 16'h1230);
    assign p_MBX_INJ_GO = wr && (addr == 16'h1238);
    assign p_LIDAR_POSE_COMMIT = wr && (addr == 16'h2070);
    assign p_LIDAR_LUT_XY = wr && (addr == 16'h2084);
    assign p_LIDAR_LUT_Z = wr && (addr == 16'h2088);
    assign p_MAP_CMD = wr && (addr == 16'h3004);
    assign p_MAP_ELEV_CMD = wr && (addr == 16'h304C);
    assign p_TRAJ_CMD = wr && (addr == 16'h4000);
    assign p_IMU_RX_DATA = rd && (addr == 16'h6004);
    assign p_IMU_RX_TS = rd && (addr == 16'h600C);
    assign p_IMU_TX_DATA = wr && (addr == 16'h6010);
    assign p_MATCH_CMD = wr && (addr == 16'h700C);
    assign p_MATCH_DBG_CMD = wr && (addr == 16'h7030);
    assign p_MATCH_SM_WDATA = wr && (addr == 16'h7044);
    always @(posedge clk) begin
        if (rst) begin
            r_SYS_SCRATCH <= 32'h00000000;
            r_SYS_IRQ_ENABLE <= 32'h00000000;
            r_NETL_MAC_LO <= 32'h56000001;
            r_NETL_MAC_HI <= 32'h0000024E;
            r_NETL_IP <= 32'hC0A80064;
            r_NETL_UDP_PORT <= 32'h00001E07;
            r_NETL_CTRL <= 32'h0000007D;
            r_NETL_PEER_IP <= 32'h00000000;
            r_NETL_PEER_MAC_LO <= 32'h00000000;
            r_NETL_PEER_MAC_HI <= 32'h00000000;
            r_NETL_PEER_PORT <= 32'h00000000;
            r_NETL_MCAST_IP <= 32'hEFFFFF4C;
            r_NETL_LIDAR_IP <= 32'hC0A80065;
            r_NETH_MAC_LO <= 32'h56000002;
            r_NETH_MAC_HI <= 32'h0000024E;
            r_NETH_IP <= 32'hC0A80264;
            r_NETH_UDP_PORT <= 32'h00002328;
            r_NETH_CTRL <= 32'h0000000D;
            r_NETH_PEER_IP <= 32'h00000000;
            r_NETH_PEER_MAC_LO <= 32'h00000000;
            r_NETH_PEER_MAC_HI <= 32'h00000000;
            r_NETH_PEER_PORT <= 32'h00002329;
            r_NETH_MCAST_IP <= 32'h00000000;
            r_MBX_TX_LEN <= 32'h00000000;
            r_MBX_INJ_LEN <= 32'h00000000;
            r_MBX_TXB_ADDR <= 32'h00000000;
            r_LIDAR_CTRL <= 32'h00000017;
            r_LIDAR_SUBHDR <= 32'h00000014;
            r_LIDAR_DEPTH_CFG <= 32'h00000100;
            r_LIDAR_RANGE <= 32'h4E200064;
            r_LIDAR_VALID_CFG <= 32'hFA000000;
            r_LIDAR_ROI_OFS <= 32'h00000000;
            r_LIDAR_R00 <= 32'h00004000;
            r_LIDAR_R01 <= 32'h00000000;
            r_LIDAR_R02 <= 32'h00000000;
            r_LIDAR_R10 <= 32'h00000000;
            r_LIDAR_R11 <= 32'h00004000;
            r_LIDAR_R12 <= 32'h00000000;
            r_LIDAR_R20 <= 32'h00000000;
            r_LIDAR_R21 <= 32'h00000000;
            r_LIDAR_R22 <= 32'h00004000;
            r_LIDAR_TX <= 32'h00000000;
            r_LIDAR_TY <= 32'h00000000;
            r_LIDAR_TZ <= 32'h00000000;
            r_LIDAR_ZBAND <= 32'h07D0FC18;
            r_LIDAR_LUT_ADDR <= 32'h00000000;
            r_MAP_CTRL <= 32'h00000005;
            r_MAP_INV_RES <= 32'h0000051F;
            r_MAP_CENTER_X <= 32'h00000000;
            r_MAP_CENTER_Y <= 32'h00000000;
            r_MAP_VALID_CFG <= 32'h000C013C;
            r_MAP_THR0 <= 32'h0032001E;
            r_MAP_THR1 <= 32'h00190039;
            r_MAP_WEIGHT <= 32'h002030A0;
            r_MAP_INFL <= 32'h00000014;
            r_MAP_DROP <= 32'h00000410;
            r_MAP_FILL <= 32'h00281E05;
            r_MAP_ELEV_ADDR <= 32'h00000000;
            r_TRAJ_CFG <= 32'h00002010;
            r_TRAJ_X0 <= 32'h00000000;
            r_TRAJ_Y0 <= 32'h00000000;
            r_TRAJ_TH0 <= 32'h00000000;
            r_TRAJ_CMAX <= 32'h000000FD;
            r_SAFE_CTRL <= 32'h00000000;
            r_SAFE_LIM_V <= 32'h012C03E8;
            r_SAFE_LIM_W <= 32'h000005DC;
            r_SAFE_LIM_A <= 32'h00030002;
            r_SAFE_TICK_US <= 32'h000003E8;
            r_SAFE_WD_US <= 32'h00030D40;
            r_SAFE_CMD_V <= 32'h00000000;
            r_SAFE_CMD_W <= 32'h00000000;
            r_SAFE_EXT_MASK <= 32'h00000000;
            r_IMU_BAUD_DIV <= 32'h00000364;
            r_IMU_IDLE_US <= 32'h000001F4;
            r_MATCH_CTRL <= 32'h00000001;
            r_MATCH_ZREL <= 32'h05DC003C;
            r_MATCH_MOUNT_Z <= 32'h0000009B;
            r_MATCH_CX_MM <= 32'h00000000;
            r_MATCH_CY_MM <= 32'h00000000;
            r_MATCH_NCAND <= 32'h000000F5;
            r_MATCH_UPD_DX <= 32'h00000000;
            r_MATCH_UPD_DY <= 32'h00000000;
            r_MATCH_UPD_DYAW <= 32'h00000000;
            r_MATCH_DBG_ADDR <= 32'h00000000;
            r_MATCH_LFO_X <= 32'h00000000;
            r_MATCH_LFO_Y <= 32'h00000000;
            r_MATCH_SM_ADDR <= 32'h00000000;
        end else if (wr) begin
            case (addr)
            16'h000C: r_SYS_SCRATCH <= wdata;
            16'h0014: r_SYS_IRQ_ENABLE <= wdata;
            16'h1000: r_NETL_MAC_LO <= wdata;
            16'h1004: r_NETL_MAC_HI <= wdata;
            16'h1008: r_NETL_IP <= wdata;
            16'h100C: r_NETL_UDP_PORT <= wdata;
            16'h1010: r_NETL_CTRL <= wdata;
            16'h1028: r_NETL_PEER_IP <= wdata;
            16'h102C: r_NETL_PEER_MAC_LO <= wdata;
            16'h1030: r_NETL_PEER_MAC_HI <= wdata;
            16'h1034: r_NETL_PEER_PORT <= wdata;
            16'h1038: r_NETL_MCAST_IP <= wdata;
            16'h104C: r_NETL_LIDAR_IP <= wdata;
            16'h1100: r_NETH_MAC_LO <= wdata;
            16'h1104: r_NETH_MAC_HI <= wdata;
            16'h1108: r_NETH_IP <= wdata;
            16'h110C: r_NETH_UDP_PORT <= wdata;
            16'h1110: r_NETH_CTRL <= wdata;
            16'h1128: r_NETH_PEER_IP <= wdata;
            16'h112C: r_NETH_PEER_MAC_LO <= wdata;
            16'h1130: r_NETH_PEER_MAC_HI <= wdata;
            16'h1134: r_NETH_PEER_PORT <= wdata;
            16'h1138: r_NETH_MCAST_IP <= wdata;
            16'h1224: r_MBX_TX_LEN <= wdata;
            16'h1234: r_MBX_INJ_LEN <= wdata;
            16'h123C: r_MBX_TXB_ADDR <= wdata;
            16'h2000: r_LIDAR_CTRL <= wdata;
            16'h2004: r_LIDAR_SUBHDR <= wdata;
            16'h2008: r_LIDAR_DEPTH_CFG <= wdata;
            16'h200C: r_LIDAR_RANGE <= wdata;
            16'h2010: r_LIDAR_VALID_CFG <= wdata;
            16'h2014: r_LIDAR_ROI_OFS <= wdata;
            16'h2040: r_LIDAR_R00 <= wdata;
            16'h2044: r_LIDAR_R01 <= wdata;
            16'h2048: r_LIDAR_R02 <= wdata;
            16'h204C: r_LIDAR_R10 <= wdata;
            16'h2050: r_LIDAR_R11 <= wdata;
            16'h2054: r_LIDAR_R12 <= wdata;
            16'h2058: r_LIDAR_R20 <= wdata;
            16'h205C: r_LIDAR_R21 <= wdata;
            16'h2060: r_LIDAR_R22 <= wdata;
            16'h2064: r_LIDAR_TX <= wdata;
            16'h2068: r_LIDAR_TY <= wdata;
            16'h206C: r_LIDAR_TZ <= wdata;
            16'h2078: r_LIDAR_ZBAND <= wdata;
            16'h2080: r_LIDAR_LUT_ADDR <= wdata;
            16'h3000: r_MAP_CTRL <= wdata;
            16'h3008: r_MAP_INV_RES <= wdata;
            16'h300C: r_MAP_CENTER_X <= wdata;
            16'h3010: r_MAP_CENTER_Y <= wdata;
            16'h3014: r_MAP_VALID_CFG <= wdata;
            16'h3018: r_MAP_THR0 <= wdata;
            16'h301C: r_MAP_THR1 <= wdata;
            16'h3020: r_MAP_WEIGHT <= wdata;
            16'h3024: r_MAP_INFL <= wdata;
            16'h3040: r_MAP_DROP <= wdata;
            16'h3044: r_MAP_FILL <= wdata;
            16'h3048: r_MAP_ELEV_ADDR <= wdata;
            16'h4008: r_TRAJ_CFG <= wdata;
            16'h400C: r_TRAJ_X0 <= wdata;
            16'h4010: r_TRAJ_Y0 <= wdata;
            16'h4014: r_TRAJ_TH0 <= wdata;
            16'h4018: r_TRAJ_CMAX <= wdata;
            16'h5000: r_SAFE_CTRL <= wdata;
            16'h5004: r_SAFE_LIM_V <= wdata;
            16'h5008: r_SAFE_LIM_W <= wdata;
            16'h500C: r_SAFE_LIM_A <= wdata;
            16'h5010: r_SAFE_TICK_US <= wdata;
            16'h5014: r_SAFE_WD_US <= wdata;
            16'h5018: r_SAFE_CMD_V <= wdata;
            16'h501C: r_SAFE_CMD_W <= wdata;
            16'h502C: r_SAFE_EXT_MASK <= wdata;
            16'h6000: r_IMU_BAUD_DIV <= wdata;
            16'h6018: r_IMU_IDLE_US <= wdata;
            16'h7000: r_MATCH_CTRL <= wdata;
            16'h7004: r_MATCH_ZREL <= wdata;
            16'h7008: r_MATCH_MOUNT_Z <= wdata;
            16'h7014: r_MATCH_CX_MM <= wdata;
            16'h7018: r_MATCH_CY_MM <= wdata;
            16'h701C: r_MATCH_NCAND <= wdata;
            16'h7020: r_MATCH_UPD_DX <= wdata;
            16'h7024: r_MATCH_UPD_DY <= wdata;
            16'h7028: r_MATCH_UPD_DYAW <= wdata;
            16'h702C: r_MATCH_DBG_ADDR <= wdata;
            16'h7038: r_MATCH_LFO_X <= wdata;
            16'h703C: r_MATCH_LFO_Y <= wdata;
            16'h7040: r_MATCH_SM_ADDR <= wdata;
            default: ;
            endcase
        end
    end
    always @(*) begin
        hit = 1'b1;
        case (addr)
        16'h0000: rdata = i_SYS_ID;
        16'h0004: rdata = i_SYS_VERSION;
        16'h0008: rdata = i_SYS_TIME_US;
        16'h000C: rdata = r_SYS_SCRATCH;
        16'h0010: rdata = r_SYS_IRQ_STATUS;
        16'h0014: rdata = r_SYS_IRQ_ENABLE;
        16'h1000: rdata = r_NETL_MAC_LO;
        16'h1004: rdata = r_NETL_MAC_HI;
        16'h1008: rdata = r_NETL_IP;
        16'h100C: rdata = r_NETL_UDP_PORT;
        16'h1010: rdata = r_NETL_CTRL;
        16'h1014: rdata = i_NETL_STAT_RX_FRAMES;
        16'h1018: rdata = i_NETL_STAT_RX_UDP;
        16'h101C: rdata = i_NETL_STAT_RX_ERR;
        16'h1020: rdata = i_NETL_STAT_ARP;
        16'h1024: rdata = i_NETL_STAT_TX_FRAMES;
        16'h1028: rdata = i_NETL_PEER_IP;
        16'h102C: rdata = i_NETL_PEER_MAC_LO;
        16'h1030: rdata = i_NETL_PEER_MAC_HI;
        16'h1034: rdata = i_NETL_PEER_PORT;
        16'h1038: rdata = r_NETL_MCAST_IP;
        16'h103C: rdata = i_NETL_PHY_STAT;
        16'h1040: rdata = 32'h0;
        16'h1044: rdata = i_NETL_RAW_STAT;
        16'h1048: rdata = 32'h0;
        16'h104C: rdata = r_NETL_LIDAR_IP;
        16'h1100: rdata = r_NETH_MAC_LO;
        16'h1104: rdata = r_NETH_MAC_HI;
        16'h1108: rdata = r_NETH_IP;
        16'h110C: rdata = r_NETH_UDP_PORT;
        16'h1110: rdata = r_NETH_CTRL;
        16'h1114: rdata = i_NETH_STAT_RX_FRAMES;
        16'h1118: rdata = i_NETH_STAT_RX_UDP;
        16'h111C: rdata = i_NETH_STAT_RX_ERR;
        16'h1120: rdata = i_NETH_STAT_ARP;
        16'h1124: rdata = i_NETH_STAT_TX_FRAMES;
        16'h1128: rdata = i_NETH_PEER_IP;
        16'h112C: rdata = i_NETH_PEER_MAC_LO;
        16'h1130: rdata = i_NETH_PEER_MAC_HI;
        16'h1134: rdata = i_NETH_PEER_PORT;
        16'h1138: rdata = r_NETH_MCAST_IP;
        16'h1200: rdata = i_MBX_RX_STAT;
        16'h1204: rdata = i_MBX_RX_DATA;
        16'h1208: rdata = 32'h0;
        16'h120C: rdata = i_MBX_RX_TS;
        16'h1210: rdata = i_MBX_RX_SRC_IP;
        16'h1214: rdata = i_MBX_RX_SRC_PORT;
        16'h1220: rdata = 32'h0;
        16'h1224: rdata = r_MBX_TX_LEN;
        16'h1228: rdata = 32'h0;
        16'h122C: rdata = i_MBX_TX_STAT;
        16'h1230: rdata = 32'h0;
        16'h1234: rdata = r_MBX_INJ_LEN;
        16'h1238: rdata = 32'h0;
        16'h123C: rdata = r_MBX_TXB_ADDR;
        16'h1240: rdata = i_MBX_TXB_Q;
        16'h1244: rdata = i_MBX_TX_SEQ;
        16'h2000: rdata = r_LIDAR_CTRL;
        16'h2004: rdata = r_LIDAR_SUBHDR;
        16'h2008: rdata = r_LIDAR_DEPTH_CFG;
        16'h200C: rdata = r_LIDAR_RANGE;
        16'h2010: rdata = r_LIDAR_VALID_CFG;
        16'h2014: rdata = r_LIDAR_ROI_OFS;
        16'h2018: rdata = i_LIDAR_STAT_FRAMES;
        16'h201C: rdata = i_LIDAR_STAT_BAD;
        16'h2020: rdata = i_LIDAR_LAST_FRAME;
        16'h2024: rdata = i_LIDAR_LAST_WH;
        16'h2028: rdata = i_LIDAR_LAST_TS;
        16'h202C: rdata = i_LIDAR_STAT_POINTS;
        16'h2030: rdata = i_LIDAR_STAT_DROP;
        16'h2034: rdata = i_LIDAR_STAT_SKIP;
        16'h2040: rdata = r_LIDAR_R00;
        16'h2044: rdata = r_LIDAR_R01;
        16'h2048: rdata = r_LIDAR_R02;
        16'h204C: rdata = r_LIDAR_R10;
        16'h2050: rdata = r_LIDAR_R11;
        16'h2054: rdata = r_LIDAR_R12;
        16'h2058: rdata = r_LIDAR_R20;
        16'h205C: rdata = r_LIDAR_R21;
        16'h2060: rdata = r_LIDAR_R22;
        16'h2064: rdata = r_LIDAR_TX;
        16'h2068: rdata = r_LIDAR_TY;
        16'h206C: rdata = r_LIDAR_TZ;
        16'h2070: rdata = 32'h0;
        16'h2074: rdata = i_LIDAR_POSE_STAT;
        16'h2078: rdata = r_LIDAR_ZBAND;
        16'h2080: rdata = r_LIDAR_LUT_ADDR;
        16'h2084: rdata = 32'h0;
        16'h2088: rdata = 32'h0;
        16'h2090: rdata = i_LIDAR_LENS0;
        16'h2094: rdata = i_LIDAR_LENS1;
        16'h2098: rdata = i_LIDAR_LENS2;
        16'h209C: rdata = i_LIDAR_LENS3;
        16'h20A0: rdata = i_LIDAR_LENS4;
        16'h20A4: rdata = i_LIDAR_LENS5;
        16'h20A8: rdata = i_LIDAR_LENS6;
        16'h20AC: rdata = i_LIDAR_LENS7;
        16'h20B0: rdata = i_LIDAR_LENS8;
        16'h20B4: rdata = i_LIDAR_INFO_CFG;
        16'h20B8: rdata = i_LIDAR_INFO_SIZE;
        16'h20BC: rdata = i_LIDAR_INFO_TEMP;
        16'h20C0: rdata = i_LIDAR_INFO_SEQ;
        16'h3000: rdata = r_MAP_CTRL;
        16'h3004: rdata = 32'h0;
        16'h3008: rdata = r_MAP_INV_RES;
        16'h300C: rdata = r_MAP_CENTER_X;
        16'h3010: rdata = r_MAP_CENTER_Y;
        16'h3014: rdata = r_MAP_VALID_CFG;
        16'h3018: rdata = r_MAP_THR0;
        16'h301C: rdata = r_MAP_THR1;
        16'h3020: rdata = r_MAP_WEIGHT;
        16'h3024: rdata = r_MAP_INFL;
        16'h3028: rdata = i_MAP_STAT;
        16'h302C: rdata = i_MAP_LAT_CX;
        16'h3030: rdata = i_MAP_LAT_CY;
        16'h3034: rdata = i_MAP_STAT_UPD;
        16'h3038: rdata = i_MAP_STAT_DROP;
        16'h303C: rdata = i_MAP_FRAME_CNT;
        16'h3040: rdata = r_MAP_DROP;
        16'h3044: rdata = r_MAP_FILL;
        16'h3048: rdata = r_MAP_ELEV_ADDR;
        16'h304C: rdata = 32'h0;
        16'h3050: rdata = i_MAP_ELEV_Q0;
        16'h3054: rdata = i_MAP_ELEV_Q1;
        16'h4000: rdata = 32'h0;
        16'h4004: rdata = i_TRAJ_STAT;
        16'h4008: rdata = r_TRAJ_CFG;
        16'h400C: rdata = r_TRAJ_X0;
        16'h4010: rdata = r_TRAJ_Y0;
        16'h4014: rdata = r_TRAJ_TH0;
        16'h4018: rdata = r_TRAJ_CMAX;
        16'h5000: rdata = r_SAFE_CTRL;
        16'h5004: rdata = r_SAFE_LIM_V;
        16'h5008: rdata = r_SAFE_LIM_W;
        16'h500C: rdata = r_SAFE_LIM_A;
        16'h5010: rdata = r_SAFE_TICK_US;
        16'h5014: rdata = r_SAFE_WD_US;
        16'h5018: rdata = r_SAFE_CMD_V;
        16'h501C: rdata = r_SAFE_CMD_W;
        16'h5020: rdata = i_SAFE_OUT_V;
        16'h5024: rdata = i_SAFE_OUT_W;
        16'h5028: rdata = i_SAFE_STAT;
        16'h502C: rdata = r_SAFE_EXT_MASK;
        16'h5030: rdata = i_SAFE_ODO_V;
        16'h5034: rdata = i_SAFE_ODO_W;
        16'h5038: rdata = i_SAFE_ODO_TS;
        16'h503C: rdata = i_SAFE_ODO_CNT;
        16'h6000: rdata = r_IMU_BAUD_DIV;
        16'h6004: rdata = i_IMU_RX_DATA;
        16'h6008: rdata = i_IMU_RX_COUNT;
        16'h600C: rdata = i_IMU_RX_TS;
        16'h6010: rdata = 32'h0;
        16'h6014: rdata = i_IMU_TX_STAT;
        16'h6018: rdata = r_IMU_IDLE_US;
        16'h601C: rdata = i_IMU_STAT_OVF;
        16'h7000: rdata = r_MATCH_CTRL;
        16'h7004: rdata = r_MATCH_ZREL;
        16'h7008: rdata = r_MATCH_MOUNT_Z;
        16'h700C: rdata = 32'h0;
        16'h7010: rdata = i_MATCH_STAT;
        16'h7014: rdata = r_MATCH_CX_MM;
        16'h7018: rdata = r_MATCH_CY_MM;
        16'h701C: rdata = r_MATCH_NCAND;
        16'h7020: rdata = r_MATCH_UPD_DX;
        16'h7024: rdata = r_MATCH_UPD_DY;
        16'h7028: rdata = r_MATCH_UPD_DYAW;
        16'h702C: rdata = r_MATCH_DBG_ADDR;
        16'h7030: rdata = 32'h0;
        16'h7034: rdata = i_MATCH_DBG_Q;
        16'h7038: rdata = r_MATCH_LFO_X;
        16'h703C: rdata = r_MATCH_LFO_Y;
        16'h7040: rdata = r_MATCH_SM_ADDR;
        16'h7044: rdata = 32'h0;
        default: begin rdata = 32'h0; hit = 1'b0; end
        endcase
    end
endmodule
