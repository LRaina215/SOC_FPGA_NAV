# ACG750 GW5AT-138K FPGA 开发笔记

本文档根据本机 `/home/cyd/fpga` 下已有资料整理，用于快速确认小梅哥 ACG750 高云 FPGA 开发板的核心器件、板载资源、常用引脚和当前电脑开发环境状态。

## 资料来源

本地资料目录：

`/home/cyd/fpga/A所有原创板卡模块资料/小梅哥ACG750型国产高云GW5AT-138K FPGA开发板资料`

主要参考文件：

- `硬件图纸/ACX750-GW5A138-CORE-25092801.pdf`
- `硬件图纸/ACX750-GW5A138-CB-250928.pdf`
- `硬件图纸/ACX750-GW5A138开发板管脚信息表.xlsx`
- `例程源码/`
- `器件手册/`

## 核心器件与板卡资源

核心 FPGA 器件为 `GW5AT-138K-PG484A`。核心板原理图中也标注了这个型号。

板载主要资源如下：

- 50MHz 有源晶振输入：`FPGA_CLK`
- 9 个用户 LED：`LED0` 到 `LED8`
- 5 个按键：`S0` 到 `S4`
- 8 位拨码开关：`SW0` 到 `SW7`
- UART、I2C、4 线 SPI、数码管、蜂鸣器、红外接收、风扇 PWM
- JTAG 接口
- QSPI Flash，原理图中器件为 `MX25L12835F`
- DDR3 存储器接口，32 位数据线，4 组 DQS
- 千兆以太网 RGMII，底板原理图中 PHY 为 RTL8211F 系列
- HDMI TX、RGB TFT、电容触摸屏、CMOS 摄像头接口
- GPIO0 40 针通用扩展接口
- PCIe、SFP1、SFP2 与 MGT Lane

## 常用引脚

| 功能 | 信号 | FPGA 管脚 |
| --- | --- | --- |
| 50MHz 时钟 | FPGA_CLK | W19 |
| LED | LED0 | U22 |
| LED | LED1 | V22 |
| LED | LED2 | W21 |
| LED | LED3 | W22 |
| LED | LED4 | Y21 |
| LED | LED5 | Y22 |
| LED | LED6 | N13 |
| LED | LED7 | N17 |
| LED | LED8 | P17 |
| 按键 | S0 | D21 |
| 按键 | S1 | E21 |
| 按键 | S2 | G22 |
| 按键 | S3 | G21 |
| 按键 | S4 | B22 |
| 拨码 | SW0 | B21 |
| 拨码 | SW1 | C22 |
| 拨码 | SW2 | E22 |
| 拨码 | SW3 | D22 |
| 拨码 | SW4 | F15 |
| 拨码 | SW5 | F21 |
| 拨码 | SW6 | D20 |
| 拨码 | SW7 | C20 |
| UART | UART_RXD | L21 |
| UART | UART_TXD | M21 |
| 数码管 | SEG7_DIO | U20 |
| 数码管 | SEG7_RCLK | V20 |
| 数码管 | SEG7_SCLK | T21 |
| I2C | IIC_SCL | T18 |
| I2C | IIC_SDA | P16 |
| 蜂鸣器 | BEEP | V17 |
| 红外接收 | IRIN | A21 |
| 风扇控制 | FAN_PWM | N14 |

## 配置、下载与存储相关引脚

| 功能 | 信号 | FPGA 管脚 |
| --- | --- | --- |
| QSPI Flash | QSPI_CS | T19 |
| QSPI Flash | QSPI_DQ0 | P22 |
| QSPI Flash | QSPI_DQ1 | R22 |
| QSPI Flash | QSPI_DQ3 | R21 |
| QSPI Flash | QSPI_SCK | L12 |
| JTAG | TCK | V12 |
| JTAG | TDI | R13 |
| JTAG | TDO | U13 |
| JTAG | TMS | T13 |

备注：管脚表中未看到 `QSPI_DQ2` 单独列在“开发板管脚名”页，但核心板原理图里 QSPI Flash 器件连接包含 `QSPI_DQ2`。

## 外设接口摘要

### 4 线 SPI

| 信号 | FPGA 管脚 |
| --- | --- |
| SPI_CS | AA21 |
| SPI_MISO | U17 |
| SPI_MOSI | U18 |
| SPI_SCLK | AA20 |

### 千兆以太网 RGMII

| 信号 | FPGA 管脚 |
| --- | --- |
| ENET1_GTX_CLK | AB21 |
| ENET1_INT_N | R16 |
| ENET1_MDC | R14 |
| ENET1_MDIO | U21 |
| ENET1_RESET | P14 |
| ENET1_RX_CLK | Y18 |
| ENET1_RX_DATA0 | P20 |
| ENET1_RX_DATA1 | N15 |
| ENET1_RX_DATA2 | AA18 |
| ENET1_RX_DATA3 | AB18 |
| ENET1_RX_DV | T20 |
| ENET1_TX_DATA0 | AB20 |
| ENET1_TX_DATA1 | Y19 |
| ENET1_TX_DATA2 | AB22 |
| ENET1_TX_DATA3 | W20 |
| ENET1_TX_EN | AA19 |

### HDMI TX

| 信号 | FPGA 管脚 |
| --- | --- |
| HDMI_CLK_N | H19 |
| HDMI_CLK_P | J19 |
| HDMI_D0_N | K22 |
| HDMI_D0_P | K21 |
| HDMI_D1_N | J21 |
| HDMI_D1_P | J20 |
| HDMI_D2_N | H22 |
| HDMI_D2_P | J22 |
| HDMI_HPD | N22 |
| HDMI_OE | M22 |
| HDMI_SCL | L20 |
| HDMI_SDA | L19 |

### RGB TFT 与触摸屏

RGB TFT 使用 `LCD_R0..R4`、`LCD_G0..G5`、`LCD_B0..B4`、`LCD_PCLK`、`LCD_HSYNC`、`LCD_VSYNC`、`LCD_DE`、`LCD_BL`。触摸屏使用 `TP_INT`、`TP_RST`、`TP_SCL`、`TP_SDA`。

关键管脚：

| 信号 | FPGA 管脚 |
| --- | --- |
| LCD_PCLK | J15 |
| LCD_HSYNC | H14 |
| LCD_VSYNC | J14 |
| LCD_DE | K14 |
| LCD_BL | K13 |
| TP_INT | M13 |
| TP_RST | L15 |
| TP_SCL | L13 |
| TP_SDA | L14 |

### CMOS 摄像头

CMOS 摄像头接口使用 8 位数据总线和同步/时钟/I2C 类控制信号：

| 信号 | FPGA 管脚 |
| --- | --- |
| CAMERA_D0 | M20 |
| CAMERA_D1 | N20 |
| CAMERA_D2 | K19 |
| CAMERA_D3 | K16 |
| CAMERA_D4 | L16 |
| CAMERA_D5 | L18 |
| CAMERA_D6 | M18 |
| CAMERA_D7 | M16 |
| CAMERA_HREF | N18 |
| CAMERA_PCLK | K18 |
| CAMERA_SCLK | J16 |
| CAMERA_SDA | M17 |
| CAMERA_VSYNC | N19 |
| CAMERA_XCLK | M15 |

### PCIe 与 SFP

PCIe 使用 MGT Lane：

| 信号 | MGT 引脚名 |
| --- | --- |
| PCIE_CLK_N | MGT_CLK0_N |
| PCIE_CLK_P | MGT_CLK0_P |
| PCIE_RX_N | MGT_RX2_N |
| PCIE_RX_P | MGT_RX2_P |
| PCIE_TX_N | MGT_TX2_N |
| PCIE_TX_P | MGT_TX2_P |
| PCIE_RST | V19 |

SFP1/SFP2 使用 MGT Lane，另有 LOS/DIS 控制信号：

| 接口 | 信号 | 管脚 |
| --- | --- | --- |
| SFP1 | SFP1_TX_DIS | R17 |
| SFP1 | SFP1_TX_LOS | P15 |
| SFP1 | RX/TX | MGT_RX0 / MGT_TX0 |
| SFP2 | SFP2_TX_DIS | V18 |
| SFP2 | SFP2_TX_LOS | W17 |
| SFP2 | RX/TX | MGT_RX1 / MGT_TX1 |
| SFP 时钟 | DIFCLK125M_N | MGT_CLK1_N |
| SFP 时钟 | DIFCLK125M_P | MGT_CLK1_P |

管脚表里第二个 125MHz 差分时钟信号名也写成了 `DIFCLK125M_N`，按差分对命名习惯这里记录为 `DIFCLK125M_P`。

## GPIO0 40 针扩展接口

| 信号 | FPGA 管脚 |
| --- | --- |
| GPIO0_0 | C18 |
| GPIO0_1 | C19 |
| GPIO0_2 | E19 |
| GPIO0_3 | D19 |
| GPIO0_4 | F19 |
| GPIO0_5 | F20 |
| GPIO0_6 | F18 |
| GPIO0_7 | E18 |
| GPIO0_8 | E16 |
| GPIO0_9 | D16 |
| GPIO0_10 | F16 |
| GPIO0_11 | E17 |
| GPIO0_12 | D14 |
| GPIO0_13 | D15 |
| GPIO0_14 | F13 |
| GPIO0_15 | F14 |
| GPIO0_16 | B20 |
| GPIO0_17 | A20 |
| GPIO0_18 | B17 |
| GPIO0_19 | B18 |
| GPIO0_20 | D17 |
| GPIO0_21 | C17 |
| GPIO0_22 | B15 |
| GPIO0_23 | B16 |
| GPIO0_24 | A18 |
| GPIO0_25 | A19 |
| GPIO0_26 | C14 |
| GPIO0_27 | C15 |
| GPIO0_28 | E13 |
| GPIO0_29 | E14 |
| GPIO0_30 | C13 |
| GPIO0_31 | B13 |
| GPIO0_32 | A13 |
| GPIO0_33 | A14 |
| GPIO0_34 | A15 |
| GPIO0_35 | A16 |

GPIO0 接口还包含 `VCC5P0`、`VCC3P3` 和 `GND` 电源/地脚。实际外接设备前应核对底板连接器原理图和电平要求。

## DDR3 引脚摘要

DDR3 接口包括：

- 地址：`DDR3_A0..A14`
- Bank 地址：`DDR3_BA0..BA2`
- 控制：`DDR3_CAS`、`DDR3_RAS`、`DDR3_WE`、`DDR3_CS`、`DDR3_CKE0`、`DDR3_ODT`、`DDR3_RESET`
- 时钟：`DDR3_CK_P`、`DDR3_CK_N`
- 数据：`DDR3_DQ0..DQ31`
- 掩码：`DDR3_DM0..DM3`
- 数据选通：`DDR3_DQS0..3_P/N`

关键控制与时钟管脚：

| 信号 | FPGA 管脚 |
| --- | --- |
| DDR3_CK_P | V4 |
| DDR3_CK_N | W4 |
| DDR3_CKE0 | AA5 |
| DDR3_CS | U5 |
| DDR3_CAS | W6 |
| DDR3_RAS | U3 |
| DDR3_WE | V3 |
| DDR3_RESET | T1 |
| DDR3_ODT | T5 |
