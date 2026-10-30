set_device -device_version C GW5AT-LV138PG484AC1/I0
add_file -type verilog {F:/nav_integration_g2_20261023/K_terrain1_prepared/nav/rtl/common/nav_ram_sdp.v}
add_file -type verilog {F:/nav_integration_g2_20261023/K_terrain1_prepared/nav/rtl/common/nav_ram_banked.v}
add_file -type verilog {F:/nav_integration_g2_20261023/K_terrain1_prepared/nav/rtl/map/elev_map.v}
add_file -type verilog {F:/nav_integration_g2_20261023/K_em_timing/em_top.v}
add_file -type sdc {F:/nav_integration_g2_20261023/K_em_timing/em.sdc}
set_option -top_module em_top
set_option -verilog_std v2001
set_option -global_freq 50
set_option -looplimit 100000
set_option -convert_sdp32_36_to_sdp16_18 1
set_option -place_option 2
set_option -route_option 0
set_option -use_sspi_as_gpio 1
set_option -use_mspi_as_gpio 1
set_option -use_ready_as_gpio 1
set_option -use_done_as_gpio 1
set_option -use_i2c_as_gpio 1
set_option -use_cpu_as_gpio 1
run all
exit
