# Independent implementation only. Never open or run the live project.
set script_dir [file dirname [file normalize [info script]]]
set board_dir [file dirname $script_dir]
set deploy_dir [file dirname $board_dir]
if {![info exists ::env(BOARD_RUN_DIR)]} {error "Use board/build_board.sh to create an audited run"}
set run_dir [file normalize $::env(BOARD_RUN_DIR)]
if {![string match "$script_dir/runs/*" $run_dir]} {error "Run must be isolated under board/gowin/runs"}
cd $run_dir
create_project -name fc_v4_board -dir $run_dir -pn GW5AT-LV138PG484AC1/I0 -device_version C
# Explicit protected core dependency list; no unrelated original RTL glob.
add_file [file join $deploy_dir rtl fc_v4_round_sat.v]
add_file [file join $deploy_dir rtl fc_v4_top.v]
foreach source [lsort [glob -directory [file join $board_dir rtl] *.v]] {add_file $source}
add_file [file join $script_dir board.cst]
add_file [file join $script_dir board.sdc]
set_option -top_module fc_v4_board_top
set_option -output_base_name fc_v4_board
set_option -verilog_std v2001
set_option -global_freq 50.000
set_option -include_path "$deploy_dir;[file join $deploy_dir rtl];[file join $board_dir rtl]"
set_option -dsp_style dsp
set_option -rom_style block_rom
set_option -timing_driven 1
set_option -show_all_warn 0
set_option -cst_warn_to_error 1
set_option -gen_io_cst 0
set_option -gen_verilog_sim_netlist 0
set_option -show_init_in_vo 0
# Default = input tri-state with weak pull-up; never open-drain outputs on unused pads.
set_option -unused_pin default
set_option -use_jtag_as_gpio 0
set_option -use_sspi_as_gpio 0
set_option -bit_incl_bsram_init 1
# Leave BSRAM init print mode at its documented default; this release's
# explicit setter erroneously requires unrelated Quad multi-boot options.
set_option -bit_compress 0
set_option -bg_programming off
saveto -all_options [file join $run_dir effective_options.tcl]
run all
