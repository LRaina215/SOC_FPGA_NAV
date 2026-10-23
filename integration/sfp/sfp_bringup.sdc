create_clock -name core_clk -period 20.000 [get_ports {clk}]
create_clock -name rx_mac_clk -period 8.000 [get_pins {u_mac/u_serdes/gtr12_quad_inst0/LANE0_PCS_RX_O_FABRIC_CLK}]
create_clock -name tx_mac_clk -period 8.000 [get_pins {u_mac/u_serdes/gtr12_quad_inst0/LANE0_PCS_TX_O_FABRIC_CLK}]
# Vendor reference uses these independent clock-domain exceptions.
# Data crosses through nav_async_fifo; diagnostic levels through nav_sync2.
# Configuration is held constant while operating; changing it resets the IP.
# This standalone STA is not a substitute for CDC/Gray-bus physical signoff.
set_false_path -from [get_clocks {core_clk}] -to [get_clocks {rx_mac_clk tx_mac_clk}]
set_false_path -from [get_clocks {rx_mac_clk}] -to [get_clocks {core_clk tx_mac_clk}]
set_false_path -from [get_clocks {tx_mac_clk}] -to [get_clocks {core_clk rx_mac_clk}]
# External reset and UART/LOS are asynchronous inputs with synchronizers.
set_false_path -from [get_ports {reset_n uart_rxd sfp1_los}]
