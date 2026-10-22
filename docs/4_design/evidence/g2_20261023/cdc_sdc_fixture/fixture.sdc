create_clock -name clk50 -period 20 [get_ports {clk50}]
create_clock -name rgmii_rxc -period 10 [get_ports {rgmii_rxc}]
set_false_path -from [get_pins {source_s1/Q}] -to [get_pins {first_s1/D}] -hold
set_max_delay -from [get_pins {source_s1/Q}] -to [get_pins {first_s1/D}] 18.000
