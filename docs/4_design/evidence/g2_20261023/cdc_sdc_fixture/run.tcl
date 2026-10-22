set_device -device_version C GW5AT-LV138PG484AC1/I0
add_file -type netlist {F:/nav_integration_g2_20261023/cdc_sdc_tool_check_v2/fixture.vg}
add_file -type sdc {F:/nav_integration_g2_20261023/cdc_sdc_tool_check_v2/fixture.sdc}
add_file -type cst {F:/nav_integration_g2_20261023/cdc_sdc_tool_check_v2/fixture.cst}
set_option -top_module cdc_fixture
set_option -output_base_name cdc_fixture
set_option -global_freq 50
set_option -gen_verilog_sim_netlist 1
run pnr
