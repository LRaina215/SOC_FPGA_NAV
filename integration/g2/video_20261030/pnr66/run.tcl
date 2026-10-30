set_device -device_version C GW5AT-LV138PG484AC1/I0
add_file -type netlist {F:/nav_integration_g2_20261023/K_pnr66_lcd480p2r0/g2_top.vg}
add_file -type cst {F:/nav_integration_g2_20261023/K_pnr66_lcd480p2r0/g2.cst}
add_file -type sdc {F:/nav_integration_g2_20261023/K_pnr66_lcd480p2r0/g2.sdc}
set_option -top_module g2_top
set_option -output_base_name g2_top
set_option -global_freq 50.0
set_option -replicate_resources 1
set_option -convert_sdp32_36_to_sdp16_18 1
set_option -gen_verilog_sim_netlist 1
set_option -place_option 2
set_option -route_option 0
set_option -use_sspi_as_gpio 1
set_option -use_mspi_as_gpio 1
set_option -use_ready_as_gpio 1
set_option -use_done_as_gpio 1
set_option -use_i2c_as_gpio 1
set_option -use_cpu_as_gpio 1
run pnr

exit
