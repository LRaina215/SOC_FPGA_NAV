// 自动生成，勿手改。来源: config/nav_params.yaml，生成器: tools/gen_config.py
`ifndef NAV_PARAMS_VH
`define NAV_PARAMS_VH
localparam integer NP_CLK_HZ = 100000000;  // core
localparam integer NP_TIME_US_BITS = 32;  // core
localparam integer NP_LIDAR_W = 160;  // lidar
localparam integer NP_LIDAR_H = 60;  // lidar
localparam integer NP_UV_BITS = 8;  // lidar
localparam integer NP_DEPTH_BITS = 16;  // lidar
localparam integer NP_LUT_FRAC = 13;  // lidar
localparam integer NP_PT_BITS = 16;  // lidar
localparam integer NP_ROT_FRAC = 14;  // pose
localparam integer NP_MAP_LOG2N = 7;  // map
localparam integer NP_MAP_RES_MM = 50;  // map
localparam integer NP_H_BITS = 16;  // map
localparam integer NP_CNT_BITS = 4;  // map
localparam integer NP_AGE_BITS = 8;  // map
localparam integer NP_TAG_BITS = 4;  // map
localparam integer NP_COST_FREE_MAX = 200;  // map
localparam integer NP_COST_LETHAL = 254;  // map
localparam integer NP_COST_UNKNOWN = 255;  // map
localparam integer NP_INFL_R_MAX = 8;  // map
localparam integer NP_TRAJ_K_MAX = 64;  // traj
localparam integer NP_TRAJ_T_MAX = 64;  // traj
localparam integer NP_TRAJ_FRAC = 8;  // traj
localparam integer NP_SIN_LUT_LOG2 = 10;  // traj
localparam integer NP_SIN_FRAC = 14;  // traj
localparam integer NP_RX_FIFO_BYTES = 4096;  // net
localparam integer NP_TX_BUF_BYTES = 2048;  // net
localparam integer NP_LIDAR_FIFO_BYTES = 8192;  // net
localparam integer NP_UART_FIFO_DEPTH = 256;  // io
localparam integer NP_MAP_N = 128;  // 派生
localparam integer NP_MAP_CELLS = 16384;  // 派生
localparam integer NP_LIDAR_PIX = 9600;  // 派生
`endif
