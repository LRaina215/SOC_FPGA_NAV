set_device -device_version C GW5AT-LV138PG484AC1/I0
add_file -type verilog {F:/nav_integration_g2_20261023/K_motor_test/motor_test_top.v}
add_file -type verilog {F:/nav_integration_g2_20261023/K_timing_syn49/control/motor520_speed.v}
add_file -type verilog {F:/nav_integration_g2_20261023/K_timing_syn49/control/motor520_closed_loop.v}
add_file -type verilog {F:/nav_integration_g2_20261023/K_timing_syn49/control/board_uart_tx.v}
add_file -type cst {F:/nav_integration_g2_20261023/K_motor_test/motor_test.cst}
set_option -top_module motor_test_top
set_option -output_base_name motor_test
set_option -use_sspi_as_gpio 1
set_option -use_mspi_as_gpio 1
set_option -use_ready_as_gpio 1
set_option -use_done_as_gpio 1
set_option -use_i2c_as_gpio 1
set_option -use_cpu_as_gpio 1
run all
