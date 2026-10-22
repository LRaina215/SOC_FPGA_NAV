# 50 MHz onboard oscillator, W19. All internal synchronous paths remain timed.
create_clock -name clk -period 20.000 [get_ports {clk}]
# USB UART is asynchronous to clk. Exempt only pad -> first synchronizer D.
# sync1 -> sync2 and every downstream synchronous path remain analyzed.
set_false_path -from [get_ports {UART_RXD}] -to [get_pins {bridge/rx/sync1_s0/D}]
# UART_TXD has no external synchronous capture clock. 0 ns is an explicitly
# synthetic one-clock-cycle output budget, NOT a CH9102 setup/hold specification.
set_output_delay -clock clk -max 0.000 [get_ports {UART_TXD}]
set_output_delay -clock clk -min 0.000 [get_ports {UART_TXD}]
# Actuator outputs are constant zero, so no clocked data path exists to constrain.
report_timing -setup -max_paths 25 -max_common_paths 1
report_timing -hold -max_paths 25 -max_common_paths 1
report_exceptions -setup -from [get_ports {UART_RXD}] -to [get_pins {bridge/rx/sync1_s0/D}] -max_paths 5
report_exceptions -hold -from [get_ports {UART_RXD}] -to [get_pins {bridge/rx/sync1_s0/D}] -max_paths 5

set_false_path -from [get_ports {SPI_MISO}] -to [get_pins {imu/u_spi/miso_sync_0_s0/D}]
set_false_path -from [get_ports {MT6701_A}] -to [get_pins {encoder/do_sync_0_s0/D}]
set_output_delay -clock clk -max 0.000 [get_ports {SPI_SCLK}]
set_output_delay -clock clk -min 0.000 [get_ports {SPI_SCLK}]
set_output_delay -clock clk -max 0.000 [get_ports {SPI_MOSI}]
set_output_delay -clock clk -min 0.000 [get_ports {SPI_MOSI}]
set_output_delay -clock clk -max 0.000 [get_ports {SPI_CSB1}]
set_output_delay -clock clk -min 0.000 [get_ports {SPI_CSB1}]
set_output_delay -clock clk -max 0.000 [get_ports {SPI_CSB2}]
set_output_delay -clock clk -min 0.000 [get_ports {SPI_CSB2}]
set_output_delay -clock clk -max 0.000 [get_ports {MT6701_B}]
set_output_delay -clock clk -min 0.000 [get_ports {MT6701_B}]
set_output_delay -clock clk -max 0.000 [get_ports {MT6701_I}]
set_output_delay -clock clk -min 0.000 [get_ports {MT6701_I}]

set_false_path -from [get_ports {MT6701_A_R}] -to [get_pins {encoder_right/do_sync_0_s0/D}]
set_output_delay -clock clk -max 0.000 [get_ports {MT6701_B_R}]
set_output_delay -clock clk -min 0.000 [get_ports {MT6701_B_R}]
set_output_delay -clock clk -max 0.000 [get_ports {MT6701_I_R}]
set_output_delay -clock clk -min 0.000 [get_ports {MT6701_I_R}]
