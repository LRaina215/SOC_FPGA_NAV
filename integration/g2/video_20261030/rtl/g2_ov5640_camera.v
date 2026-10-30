// Source: Chenyudog/fpga@715378a / fpga/src/ov5640_camera.v
// Integration copy: module identifiers only were namespaced; original notices retained.
// OV5640 DVP RGB565,16-bit SCCB register addresses. Low-resource160x120 capture.
// RGB register sequence derived from board vendor ov5640_init_table_rgb.v;
// source attribution: Wuhan Xinluheng, corecourse.cn,2019-05-01.
// The18-pin module has no XCLK input: onboard oscillator required.
module g2_ov5640_camera #(
    parameter CLOCK_FREQ_HZ=50_000_000,
    parameter SCCB_FREQ_HZ=100_000,
    parameter POWER_WAIT_CYCLES=CLOCK_FREQ_HZ/1000*30,
    parameter RESET_WAIT_CYCLES=CLOCK_FREQ_HZ/1000*5,
    parameter HARD_RESET_CYCLES=CLOCK_FREQ_HZ/1000*5,
    parameter SOURCE_WIDTH=640,SOURCE_HEIGHT=480,DECIMATE=4
)(
    input wire clk,reset_n,
    input wire [7:0] CAMERA_D,
    input wire CAMERA_PCLK,CAMERA_HREF,CAMERA_VSYNC,
    output wire CAMERA_SCLK,inout wire CAMERA_SDA,
    output wire CAMERA_RESET_N,
    output reg init_done=0,
    output reg init_error=0,
    output reg pixel_valid=0,
    output reg [16:0] pixel_addr=0,
    output reg [15:0] pixel_data=0,
    output reg write_bank=0,
    output reg frame_req=0,
    input wire frame_ack
);
    localparam IMAGE_PIXELS=(SOURCE_WIDTH/DECIMATE)*(SOURCE_HEIGHT/DECIMATE);
    localparam SCCB_DIV=CLOCK_FREQ_HZ/SCCB_FREQ_HZ/4;
    localparam REG_COUNT=252;
    localparam [15:0] IMAGE_WIDTH=SOURCE_WIDTH,IMAGE_HEIGHT=SOURCE_HEIGHT;
    localparam [7:0] IMAGE_FLIP_DAT=8'h40;
    localparam [3:0] IMAGE_MIRROR_DAT=4'h7;
    reg [20:0] hard_reset_count;
    always @(posedge clk or negedge reset_n)
        if(!reset_n) hard_reset_count<=0;
        else if(hard_reset_count<HARD_RESET_CYCLES) hard_reset_count<=hard_reset_count+1'b1;
    assign CAMERA_RESET_N=reset_n && (hard_reset_count>=HARD_RESET_CYCLES);
    reg scl_low=0,sda_low=0;
    assign CAMERA_SCLK=scl_low ? 1'b0 : 1'bz;
    assign CAMERA_SDA=sda_low ? 1'b0 : 1'bz;
    function [23:0] ov5640_reg;
        input [7:0] index;
        begin
            case(index)
                0: ov5640_reg=24'h3103_11;
                1: ov5640_reg=24'h3008_82;
                2: ov5640_reg=24'h3008_42;
                3: ov5640_reg=24'h3103_03;
                4: ov5640_reg=24'h3017_ff;
                5: ov5640_reg=24'h3018_ff;
                6: ov5640_reg=24'h3034_1a;
                7: ov5640_reg=24'h3037_13;
                8: ov5640_reg=24'h3108_01;
                9: ov5640_reg=24'h3630_36;
                10: ov5640_reg=24'h3631_0e;
                11: ov5640_reg=24'h3632_e2;
                12: ov5640_reg=24'h3633_12;
                13: ov5640_reg=24'h3621_e0;
                14: ov5640_reg=24'h3704_a0;
                15: ov5640_reg=24'h3703_5a;
                16: ov5640_reg=24'h3715_78;
                17: ov5640_reg=24'h3717_01;
                18: ov5640_reg=24'h370b_60;
                19: ov5640_reg=24'h3705_1a;
                20: ov5640_reg=24'h3905_02;
                21: ov5640_reg=24'h3906_10;
                22: ov5640_reg=24'h3901_0a;
                23: ov5640_reg=24'h3731_12;
                24: ov5640_reg=24'h3600_08;
                25: ov5640_reg=24'h3601_33;
                26: ov5640_reg=24'h302d_60;
                27: ov5640_reg=24'h3620_52;
                28: ov5640_reg=24'h371b_20;
                29: ov5640_reg=24'h471c_50;
                30: ov5640_reg=24'h3a13_43;
                31: ov5640_reg=24'h3a18_00;
                32: ov5640_reg=24'h3a19_f8;
                33: ov5640_reg=24'h3635_13;
                34: ov5640_reg=24'h3636_03;
                35: ov5640_reg=24'h3634_40;
                36: ov5640_reg=24'h3622_01;
                37: ov5640_reg=24'h3c01_34;
                38: ov5640_reg=24'h3c04_28;
                39: ov5640_reg=24'h3c05_98;
                40: ov5640_reg=24'h3c06_00;
                41: ov5640_reg=24'h3c07_08;
                42: ov5640_reg=24'h3c08_00;
                43: ov5640_reg=24'h3c09_1c;
                44: ov5640_reg=24'h3c0a_9c;
                45: ov5640_reg=24'h3c0b_40;
                46: ov5640_reg=24'h3810_00;
                47: ov5640_reg=24'h3811_10;
                48: ov5640_reg=24'h3812_00;
                49: ov5640_reg=24'h3708_64;
                50: ov5640_reg=24'h4001_02;
                51: ov5640_reg=24'h4005_1a;
                52: ov5640_reg=24'h3000_00;
                53: ov5640_reg=24'h3004_ff;
                54: ov5640_reg=24'h300e_58;
                55: ov5640_reg=24'h302e_00;
                56: ov5640_reg=24'h4300_61;
                57: ov5640_reg=24'h501f_01;
                58: ov5640_reg=24'h440e_00;
                59: ov5640_reg=24'h5000_a7;
                60: ov5640_reg=24'h3a0f_30;
                61: ov5640_reg=24'h3a10_28;
                62: ov5640_reg=24'h3a1b_30;
                63: ov5640_reg=24'h3a1e_26;
                64: ov5640_reg=24'h3a11_60;
                65: ov5640_reg=24'h3a1f_14;
                66: ov5640_reg=24'h5800_23;
                67: ov5640_reg=24'h5801_14;
                68: ov5640_reg=24'h5802_0f;
                69: ov5640_reg=24'h5803_0f;
                70: ov5640_reg=24'h5804_12;
                71: ov5640_reg=24'h5805_26;
                72: ov5640_reg=24'h5806_0c;
                73: ov5640_reg=24'h5807_08;
                74: ov5640_reg=24'h5808_05;
                75: ov5640_reg=24'h5809_05;
                76: ov5640_reg=24'h580a_08;
                77: ov5640_reg=24'h580b_0d;
                78: ov5640_reg=24'h580c_08;
                79: ov5640_reg=24'h580d_03;
                80: ov5640_reg=24'h580e_00;
                81: ov5640_reg=24'h580f_00;
                82: ov5640_reg=24'h5810_03;
                83: ov5640_reg=24'h5811_09;
                84: ov5640_reg=24'h5812_07;
                85: ov5640_reg=24'h5813_03;
                86: ov5640_reg=24'h5814_00;
                87: ov5640_reg=24'h5815_01;
                88: ov5640_reg=24'h5816_03;
                89: ov5640_reg=24'h5817_08;
                90: ov5640_reg=24'h5818_0d;
                91: ov5640_reg=24'h5819_08;
                92: ov5640_reg=24'h581a_05;
                93: ov5640_reg=24'h581b_06;
                94: ov5640_reg=24'h581c_08;
                95: ov5640_reg=24'h581d_0e;
                96: ov5640_reg=24'h581e_29;
                97: ov5640_reg=24'h581f_17;
                98: ov5640_reg=24'h5820_11;
                99: ov5640_reg=24'h5821_11;
                100: ov5640_reg=24'h5822_15;
                101: ov5640_reg=24'h5823_28;
                102: ov5640_reg=24'h5824_46;
                103: ov5640_reg=24'h5825_26;
                104: ov5640_reg=24'h5826_08;
                105: ov5640_reg=24'h5827_26;
                106: ov5640_reg=24'h5828_64;
                107: ov5640_reg=24'h5829_26;
                108: ov5640_reg=24'h582a_24;
                109: ov5640_reg=24'h582b_22;
                110: ov5640_reg=24'h582c_24;
                111: ov5640_reg=24'h582d_24;
                112: ov5640_reg=24'h582e_06;
                113: ov5640_reg=24'h582f_22;
                114: ov5640_reg=24'h5830_40;
                115: ov5640_reg=24'h5831_42;
                116: ov5640_reg=24'h5832_24;
                117: ov5640_reg=24'h5833_26;
                118: ov5640_reg=24'h5834_24;
                119: ov5640_reg=24'h5835_22;
                120: ov5640_reg=24'h5836_22;
                121: ov5640_reg=24'h5837_26;
                122: ov5640_reg=24'h5838_44;
                123: ov5640_reg=24'h5839_24;
                124: ov5640_reg=24'h583a_26;
                125: ov5640_reg=24'h583b_28;
                126: ov5640_reg=24'h583c_42;
                127: ov5640_reg=24'h583d_ce;
                128: ov5640_reg=24'h5180_ff;
                129: ov5640_reg=24'h5181_f2;
                130: ov5640_reg=24'h5182_00;
                131: ov5640_reg=24'h5183_14;
                132: ov5640_reg=24'h5184_25;
                133: ov5640_reg=24'h5185_24;
                134: ov5640_reg=24'h5186_09;
                135: ov5640_reg=24'h5187_09;
                136: ov5640_reg=24'h5188_09;
                137: ov5640_reg=24'h5189_75;
                138: ov5640_reg=24'h518a_54;
                139: ov5640_reg=24'h518b_e0;
                140: ov5640_reg=24'h518c_b2;
                141: ov5640_reg=24'h518d_42;
                142: ov5640_reg=24'h518e_3d;
                143: ov5640_reg=24'h518f_56;
                144: ov5640_reg=24'h5190_46;
                145: ov5640_reg=24'h5191_f8;
                146: ov5640_reg=24'h5192_04;
                147: ov5640_reg=24'h5193_70;
                148: ov5640_reg=24'h5194_f0;
                149: ov5640_reg=24'h5195_f0;
                150: ov5640_reg=24'h5196_03;
                151: ov5640_reg=24'h5197_01;
                152: ov5640_reg=24'h5198_04;
                153: ov5640_reg=24'h5199_12;
                154: ov5640_reg=24'h519a_04;
                155: ov5640_reg=24'h519b_00;
                156: ov5640_reg=24'h519c_06;
                157: ov5640_reg=24'h519d_82;
                158: ov5640_reg=24'h519e_38;
                159: ov5640_reg=24'h5480_01;
                160: ov5640_reg=24'h5481_08;
                161: ov5640_reg=24'h5482_14;
                162: ov5640_reg=24'h5483_28;
                163: ov5640_reg=24'h5484_51;
                164: ov5640_reg=24'h5485_65;
                165: ov5640_reg=24'h5486_71;
                166: ov5640_reg=24'h5487_7d;
                167: ov5640_reg=24'h5488_87;
                168: ov5640_reg=24'h5489_91;
                169: ov5640_reg=24'h548a_9a;
                170: ov5640_reg=24'h548b_aa;
                171: ov5640_reg=24'h548c_b8;
                172: ov5640_reg=24'h548d_cd;
                173: ov5640_reg=24'h548e_dd;
                174: ov5640_reg=24'h548f_ea;
                175: ov5640_reg=24'h5490_1d;
                176: ov5640_reg=24'h5381_1e;
                177: ov5640_reg=24'h5382_5b;
                178: ov5640_reg=24'h5383_08;
                179: ov5640_reg=24'h5384_0a;
                180: ov5640_reg=24'h5385_7e;
                181: ov5640_reg=24'h5386_88;
                182: ov5640_reg=24'h5387_7c;
                183: ov5640_reg=24'h5388_6c;
                184: ov5640_reg=24'h5389_10;
                185: ov5640_reg=24'h538a_01;
                186: ov5640_reg=24'h538b_98;
                187: ov5640_reg=24'h5580_06;
                188: ov5640_reg=24'h5583_40;
                189: ov5640_reg=24'h5584_10;
                190: ov5640_reg=24'h5589_10;
                191: ov5640_reg=24'h558a_00;
                192: ov5640_reg=24'h558b_f8;
                193: ov5640_reg=24'h501d_40;
                194: ov5640_reg=24'h5300_08;
                195: ov5640_reg=24'h5301_30;
                196: ov5640_reg=24'h5302_10;
                197: ov5640_reg=24'h5303_00;
                198: ov5640_reg=24'h5304_08;
                199: ov5640_reg=24'h5305_30;
                200: ov5640_reg=24'h5306_08;
                201: ov5640_reg=24'h5307_16;
                202: ov5640_reg=24'h5309_08;
                203: ov5640_reg=24'h530a_30;
                204: ov5640_reg=24'h530b_04;
                205: ov5640_reg=24'h530c_06;
                206: ov5640_reg=24'h5025_00;
                207: ov5640_reg=24'h3008_02;
                208: ov5640_reg=24'h3035_41;
                209: ov5640_reg=24'h3036_69;
                210: ov5640_reg=24'h3c07_07;
                211: ov5640_reg={16'h3820, IMAGE_FLIP_DAT};
                212: ov5640_reg={20'h38210, IMAGE_MIRROR_DAT};
                213: ov5640_reg=24'h3814_31;
                214: ov5640_reg=24'h3815_31;
                215: ov5640_reg=24'h3800_00;
                216: ov5640_reg=24'h3801_00;
                217: ov5640_reg=24'h3802_00;
                218: ov5640_reg=24'h3803_04;
                219: ov5640_reg=24'h3804_0a;
                220: ov5640_reg=24'h3805_3f;
                221: ov5640_reg=24'h3806_07;
                222: ov5640_reg=24'h3807_9b;
                223: ov5640_reg={16'h3808, IMAGE_WIDTH[15:8]};
                224: ov5640_reg={16'h3809, IMAGE_WIDTH[ 7:0]};
                225: ov5640_reg={16'h380a, IMAGE_HEIGHT[15:8]};
                226: ov5640_reg={16'h380b, IMAGE_HEIGHT[ 7:0]};
                227: ov5640_reg=24'h380c_07;
                228: ov5640_reg=24'h380d_68;
                229: ov5640_reg=24'h380e_03;
                230: ov5640_reg=24'h380f_d8;
                231: ov5640_reg=24'h3813_04;
                232: ov5640_reg=24'h3618_00;
                233: ov5640_reg=24'h3612_29;
                234: ov5640_reg=24'h3709_52;
                235: ov5640_reg=24'h370c_03;
                236: ov5640_reg=24'h3a02_03;
                237: ov5640_reg=24'h3a03_d8;
                238: ov5640_reg=24'h3a14_03;
                239: ov5640_reg=24'h3a15_d8;
                240: ov5640_reg=24'h4004_02;
                241: ov5640_reg=24'h3002_1c;
                242: ov5640_reg=24'h3006_c3;
                243: ov5640_reg=24'h4713_03;
                244: ov5640_reg=24'h4407_04;
                245: ov5640_reg=24'h460b_37;
                246: ov5640_reg=24'h460c_20;
                247: ov5640_reg=24'h4837_16;
                248: ov5640_reg=24'h3824_04;
                249: ov5640_reg=24'h5001_83;
                250: ov5640_reg=24'h3503_00;
                251: ov5640_reg=24'h4740_20;
                default: ov5640_reg=24'h000000;
            endcase
        end
    endfunction
    reg [2:0] verify_index;
    function [15:0] verify_address;
        input [2:0] i;
        begin case(i)
            0:verify_address=16'h4300;1:verify_address=16'h501f;
            2:verify_address=16'h3808;3:verify_address=16'h3809;
            4:verify_address=16'h380a;5:verify_address=16'h380b;
            default:verify_address=16'h3008;
        endcase end
    endfunction
    function [7:0] verify_value;
        input [2:0] i;
        begin case(i)
            0:verify_value=8'h61;1:verify_value=8'h01;
            2:verify_value=IMAGE_WIDTH[15:8];3:verify_value=IMAGE_WIDTH[7:0];
            4:verify_value=IMAGE_HEIGHT[15:8];5:verify_value=IMAGE_HEIGHT[7:0];
            default:verify_value=8'h02;
        endcase end
    endfunction
    localparam WAIT_POWER = 5'd0, START_IDLE = 5'd1, START_LOW = 5'd2,
               START_CLOCK_LOW = 5'd3, BIT_SETUP = 5'd4, BIT_HIGH = 5'd5,
               BIT_HOLD = 5'd6, BIT_LOW = 5'd7, ACK_SETUP = 5'd8,
               ACK_HIGH = 5'd9, ACK_SAMPLE = 5'd10, ACK_LOW = 5'd11,
               STOP_LOW = 5'd12, STOP_HIGH = 5'd13, STOP_RELEASE = 5'd14,
               REG_WAIT = 5'd15, DONE = 5'd16,
               READ_SETUP = 5'd17, READ_HIGH = 5'd18,
               READ_SAMPLE = 5'd19, READ_LOW = 5'd20,
               NA_SETUP = 5'd21, NA_HIGH = 5'd22, NA_HOLD = 5'd23,
               NA_LOW = 5'd24;
    localparam ID_HIGH_ADDR = 3'd0, ID_HIGH_READ = 3'd1,
               ID_LOW_ADDR = 3'd2, ID_LOW_READ = 3'd3,
               WRITE_REG = 3'd4, VERIFY_ADDR = 3'd5, VERIFY_READ = 3'd6;
    reg [4:0] state = WAIT_POWER;
    reg [2:0] operation = ID_HIGH_ADDR;
    reg [31:0] wait_cnt = 32'd0;
    reg [15:0] sccb_div_cnt = 16'd0;
    reg [7:0] reg_index = 8'd0;
    reg [1:0] byte_index = 2'd0;
    reg [2:0] bit_index = 3'd7;
    reg [7:0] send_byte = 8'h78;
    reg [1:0] retry_cnt = 2'd0;
    reg [7:0] read_byte = 8'd0;
    reg [7:0] id_high = 8'd0;
    reg sda_sync_0 = 1'b1;
    reg sda_sync_1 = 1'b1;
    wire [23:0] reg_value = ov5640_reg(reg_index);
    wire read_operation = operation == ID_HIGH_READ ||
                          operation == ID_LOW_READ || operation == VERIFY_READ;
    wire [15:0] read_reg_addr = operation == ID_HIGH_ADDR ? 16'h300a :
                              operation == ID_LOW_ADDR ? 16'h300b : verify_address(verify_index);
    reg transaction_nack;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state <= WAIT_POWER;
            operation <= ID_HIGH_ADDR;
            wait_cnt <= 32'd0;
            sccb_div_cnt <= 16'd0;
            reg_index <= 8'd0;
            verify_index <= 0;transaction_nack<=0;
            byte_index <= 2'd0;
            bit_index <= 3'd7;
            send_byte <= 8'h78;
            retry_cnt <= 2'd0;
            read_byte <= 8'd0;
            id_high <= 8'd0;
            scl_low <= 1'b0;
            sda_low <= 1'b0;
            sda_sync_0 <= 1'b1;
            sda_sync_1 <= 1'b1;
            init_done <= 1'b0;
            init_error <= 1'b0;
        end else begin
            sda_sync_0 <= CAMERA_SDA;
            sda_sync_1 <= sda_sync_0;
            if (state == WAIT_POWER || state == REG_WAIT) begin
                sccb_div_cnt <= 16'd0;
                wait_cnt <= wait_cnt + 32'd1;
                if ((state == WAIT_POWER && wait_cnt >= POWER_WAIT_CYCLES - 1) ||
                    (state == REG_WAIT && wait_cnt >= RESET_WAIT_CYCLES - 1)) begin
                    wait_cnt <= 32'd0;
                    state <= START_IDLE;
                end
            end else if (sccb_div_cnt == SCCB_DIV - 1) begin
                sccb_div_cnt <= 16'd0;
                case (state)
                    START_IDLE: begin
                        scl_low <= 1'b0;
                        sda_low <= 1'b0;
                        byte_index <= 2'd0;
                        bit_index <= 3'd7;
                        send_byte <= read_operation ? 8'h79 : 8'h78;
                        transaction_nack<=0;
                        read_byte <= 8'd0;
                        state <= START_LOW;
                    end
                    START_LOW: begin
                        sda_low <= 1'b1;
                        state <= START_CLOCK_LOW;
                    end
                    START_CLOCK_LOW: begin
                        scl_low <= 1'b1;
                        state <= BIT_SETUP;
                    end
                    BIT_SETUP: begin
                        sda_low <= ~send_byte[bit_index];
                        state <= BIT_HIGH;
                    end
                    BIT_HIGH: begin
                        scl_low <= 1'b0;
                        state <= BIT_HOLD;
                    end
                    BIT_HOLD: state <= BIT_LOW;
                    BIT_LOW: begin
                        scl_low <= 1'b1;
                        if (bit_index == 3'd0)
                            state <= ACK_SETUP;
                        else begin
                            bit_index <= bit_index - 3'd1;
                            state <= BIT_SETUP;
                        end
                    end
                    ACK_SETUP: begin
                        sda_low <= 1'b0;
                        state <= ACK_HIGH;
                    end
                    ACK_HIGH: begin
                        scl_low <= 1'b0;
                        state <= ACK_SAMPLE;
                    end
                    ACK_SAMPLE: begin
                        if(sda_sync_1 && !(operation==WRITE_REG && reg_value==24'h300882 && byte_index==3))
                            transaction_nack<=1;
                        state<=ACK_LOW;
                    end
                    ACK_LOW: begin
                        scl_low <= 1'b1;
                        if(transaction_nack) state<=STOP_LOW;
                        else if (read_operation) begin
                            bit_index <= 3'd7;
                            state <= READ_SETUP;
                        end else if ((operation == WRITE_REG && byte_index == 2'd3) ||
                                     (operation != WRITE_REG && byte_index == 2'd2))
                            state <= STOP_LOW;
                        else begin
                            byte_index <= byte_index + 2'd1;
                            if (operation == WRITE_REG)
                                send_byte <= byte_index == 2'd0 ? reg_value[23:16] : byte_index == 2'd1 ? reg_value[15:8] : reg_value[7:0];
                            else
                                send_byte <= byte_index==0 ? read_reg_addr[15:8] : read_reg_addr[7:0];
                            bit_index <= 3'd7;
                            state <= BIT_SETUP;
                        end
                    end
                    READ_SETUP: begin
                        sda_low <= 1'b0; // 释放SDA，让摄像头驱动数据。
                        state <= READ_HIGH;
                    end
                    READ_HIGH: begin
                        scl_low <= 1'b0;
                        state <= READ_SAMPLE;
                    end
                    READ_SAMPLE: begin
                        read_byte <= {read_byte[6:0], sda_sync_1};
                        state <= READ_LOW;
                    end
                    READ_LOW: begin
                        scl_low <= 1'b1;
                        if (bit_index == 3'd0)
                            state <= NA_SETUP;
                        else begin
                            bit_index <= bit_index - 3'd1;
                            state <= READ_SETUP;
                        end
                    end
                    NA_SETUP: begin
                        sda_low <= 1'b0;
                        state <= NA_HIGH;
                    end
                    NA_HIGH: begin
                        scl_low <= 1'b0;
                        state <= NA_HOLD;
                    end
                    NA_HOLD: state <= NA_LOW;
                    NA_LOW: begin
                        scl_low <= 1'b1;
                        state <= STOP_LOW;
                    end
                    STOP_LOW: begin
                        sda_low <= 1'b1;
                        state <= STOP_HIGH;
                    end
                    STOP_HIGH: begin
                        scl_low <= 1'b0;
                        state <= STOP_RELEASE;
                    end
                    STOP_RELEASE: begin
                        sda_low <= 1'b0;
                        if(transaction_nack) begin
                            if(retry_cnt==2) begin init_error<=1;state<=DONE;end
                            else begin retry_cnt<=retry_cnt+1'b1;operation<=ID_HIGH_ADDR;reg_index<=0;verify_index<=0;wait_cnt<=0;state<=REG_WAIT;end
                        end else case (operation)
                            ID_HIGH_ADDR: begin
                                operation <= ID_HIGH_READ;
                                state <= START_IDLE;
                            end
                            ID_HIGH_READ: begin
                                id_high <= read_byte;
                                operation <= ID_LOW_ADDR;
                                state <= START_IDLE;
                            end
                            ID_LOW_ADDR: begin
                                operation <= ID_LOW_READ;
                                state <= START_IDLE;
                            end
                            ID_LOW_READ: begin
                                if (id_high == 8'h56 && read_byte == 8'h40) begin
                                    operation <= WRITE_REG;
                                    state <= START_IDLE;
                                end else if (retry_cnt == 2'd2) begin
                                    init_error <= 1'b1;
                                    state <= DONE;
                                end else begin
                                    retry_cnt <= retry_cnt + 2'd1;
                                    operation <= ID_HIGH_ADDR;
                                    wait_cnt <= 32'd0;
                                    state <= REG_WAIT;
                                end
                            end
                            WRITE_REG: begin
                                if (reg_index == REG_COUNT - 1) begin
                                    operation <= VERIFY_ADDR;
                                    state <= START_IDLE;
                                end else begin
                                    reg_index <= reg_index + 1'b1;
                                    if (reg_index == 8'd1) begin
                                        wait_cnt <= 32'd0;
                                        state <= REG_WAIT;
                                    end else
                                        state <= START_IDLE;
                                end
                            end
                            VERIFY_ADDR: begin
                                operation <= VERIFY_READ;
                                state <= START_IDLE;
                            end
                            VERIFY_READ: begin
                                if(read_byte!=verify_value(verify_index)) begin init_error<=1;state<=DONE;end
                                else if(verify_index==6) begin init_done<=1;state<=DONE;end
                                else begin verify_index<=verify_index+1'b1;operation<=VERIFY_ADDR;state<=START_IDLE;end
                            end
                            default: state <= DONE;
                        endcase
                    end
                    DONE: begin
                        scl_low <= 1'b0;
                        sda_low <= 1'b0;
                    end
                    default: state <= WAIT_POWER;
                endcase
            end else
                sccb_div_cnt <= sccb_div_cnt + 16'd1;
        end
    end

    // 只有状态信号做两级同步；DVP数据、HREF和VSYNC在PCLK域直接采样。
    reg ready_sync_0 = 1'b0;
    reg ready_sync_1 = 1'b0;
    reg ack_sync_0 = 1'b0;
    reg ack_sync_1 = 1'b0;
    reg [1:0] pclk_reset = 2'b00;
    always @(posedge CAMERA_PCLK or negedge reset_n) begin
        if (!reset_n)
            pclk_reset <= 2'b00;
        else
            pclk_reset <= {pclk_reset[0], 1'b1};
    end

    wire camera_reset_n=pclk_reset[1];

    reg vsync_last = 1'b0;
    reg href_last = 1'b0;
    reg byte_phase = 1'b0;
    reg [7:0] high_byte = 8'd0;
    reg [10:0] x_cnt = 11'd0;
    reg [9:0] y_cnt = 10'd0;
    reg [16:0] stored_pixels = 17'd0;
    reg capturing = 1'b0;
    reg waiting_ack = 1'b0;
    reg bad_frame = 1'b0;
    wire frame_start = vsync_last && !CAMERA_VSYNC;
    wire frame_end = !vsync_last && CAMERA_VSYNC;

    always @(posedge CAMERA_PCLK or negedge camera_reset_n) begin
        if (!camera_reset_n) begin
            ready_sync_0 <= 1'b0;
            ready_sync_1 <= 1'b0;
            ack_sync_0 <= 1'b0;
            ack_sync_1 <= 1'b0;
        end else begin
            ready_sync_0 <= init_done;
            ready_sync_1 <= ready_sync_0;
            ack_sync_0 <= frame_ack;
            ack_sync_1 <= ack_sync_0;
        end
    end

    // 两个字节组成一个RGB565像素；只发布行长、行数均正确的完整帧。
    // 缓存尚未被LCD接收时丢弃新帧，避免读写同一块缓存造成撕裂。
    always @(posedge CAMERA_PCLK or negedge camera_reset_n) begin
        if (!camera_reset_n) begin
            vsync_last <= 1'b0;
            href_last <= 1'b0;
            byte_phase <= 1'b0;
            high_byte <= 8'd0;
            x_cnt <= 11'd0;
            y_cnt <= 10'd0;
            stored_pixels <= 17'd0;
            capturing <= 1'b0;
            waiting_ack <= 1'b0;
            bad_frame <= 1'b0;
            pixel_valid <= 1'b0;
            pixel_addr <= 17'd0;
            pixel_data <= 16'd0;
            write_bank <= 1'b0;
            frame_req <= 1'b0;
        end else begin
            vsync_last <= CAMERA_VSYNC;
            href_last <= CAMERA_HREF;
            pixel_valid <= 1'b0;
            if (!pclk_reset[1] || !ready_sync_1) begin
                capturing <= 1'b0;
                byte_phase <= 1'b0;
            end else begin
                if (waiting_ack && ack_sync_1 == frame_req) begin
                    waiting_ack <= 1'b0;
                    write_bank <= ~write_bank;
                end

                if (frame_end) begin
                    capturing <= 1'b0;
                    byte_phase <= 1'b0;
                    if (capturing && !bad_frame && !href_last &&
                        y_cnt == SOURCE_HEIGHT && stored_pixels == IMAGE_PIXELS) begin
                        frame_req <= ~frame_req;
                        waiting_ack <= 1'b1;
                    end
                end else if (frame_start) begin
                    capturing <= !waiting_ack;
                    byte_phase <= 1'b0;
                    x_cnt <= 11'd0;
                    y_cnt <= 10'd0;
                    stored_pixels <= 17'd0;
                    bad_frame <= 1'b0;
                end else if (capturing && !CAMERA_VSYNC) begin
                    if (CAMERA_HREF) begin
                        if (!byte_phase) begin
                            high_byte <= CAMERA_D;
                            byte_phase <= 1'b1;
                        end else begin
                            byte_phase <= 1'b0;
                            if (x_cnt < SOURCE_WIDTH && y_cnt < SOURCE_HEIGHT) begin
                                if ((x_cnt % DECIMATE)==0 && (y_cnt % DECIMATE)==0) begin
                                    pixel_valid <= 1'b1;
                                    pixel_addr <= stored_pixels;
                                    pixel_data <= {high_byte, CAMERA_D};
                                    stored_pixels <= stored_pixels + 17'd1;
                                end
                                x_cnt <= x_cnt + 11'd1;
                            end else
                                bad_frame <= 1'b1;
                        end
                    end else if (href_last) begin
                        if (byte_phase || x_cnt != SOURCE_WIDTH)
                            bad_frame <= 1'b1;
                        byte_phase <= 1'b0;
                        x_cnt <= 11'd0;
                        if (y_cnt < SOURCE_HEIGHT)
                            y_cnt <= y_cnt + 10'd1;
                        else
                            bad_frame <= 1'b1;
                    end
                end
            end
        end
    end
endmodule

