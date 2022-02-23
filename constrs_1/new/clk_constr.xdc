create_clock -period 10.000 -name sys_clk_1 -waveform {0.000 5.000} -add [get_ports sys_clk_i_1]
create_clock -period 5.000 -name sys_clk_2 -waveform {0.000 2.500} -add [get_ports sys_clk_i_2]






set_property PACKAGE_PIN AR25 [get_ports {data_input[15]}]
set_property PACKAGE_PIN AN25 [get_ports {data_input[14]}]
set_property PACKAGE_PIN AN26 [get_ports {data_input[13]}]
set_property PACKAGE_PIN AM28 [get_ports {data_input[12]}]
set_property PACKAGE_PIN AM29 [get_ports {data_input[11]}]
set_property PACKAGE_PIN AK27 [get_ports {data_input[10]}]
set_property PACKAGE_PIN AL27 [get_ports {data_input[9]}]
set_property PACKAGE_PIN AM26 [get_ports {data_input[8]}]
set_property PACKAGE_PIN AM27 [get_ports {data_input[7]}]
set_property PACKAGE_PIN AK24 [get_ports {data_input[6]}]
set_property PACKAGE_PIN AK25 [get_ports {data_input[5]}]
set_property PACKAGE_PIN AL25 [get_ports {data_input[4]}]
set_property PACKAGE_PIN AL26 [get_ports {data_input[3]}]
set_property PACKAGE_PIN AJ25 [get_ports {data_input[2]}]
set_property PACKAGE_PIN AJ26 [get_ports {data_input[1]}]
set_property PACKAGE_PIN AP26 [get_ports {data_input[0]}]
set_property PACKAGE_PIN AV26 [get_ports {data_print[15]}]
set_property PACKAGE_PIN AW27 [get_ports {data_print[14]}]
set_property PACKAGE_PIN AW28 [get_ports {data_print[13]}]
set_property PACKAGE_PIN AU28 [get_ports {data_print[12]}]
set_property PACKAGE_PIN AV28 [get_ports {data_print[11]}]
set_property PACKAGE_PIN AU26 [get_ports {data_print[10]}]
set_property PACKAGE_PIN AU27 [get_ports {data_print[9]}]
set_property PACKAGE_PIN AR27 [get_ports {data_print[8]}]
set_property PACKAGE_PIN AT27 [get_ports {data_print[7]}]
set_property PACKAGE_PIN AP27 [get_ports {data_print[6]}]
set_property PACKAGE_PIN AR28 [get_ports {data_print[5]}]
set_property PACKAGE_PIN AN28 [get_ports {data_print[4]}]
set_property PACKAGE_PIN AP28 [get_ports {data_print[3]}]
set_property PACKAGE_PIN AT25 [get_ports {data_print[2]}]
set_property PACKAGE_PIN AT26 [get_ports {data_print[1]}]
set_property PACKAGE_PIN AP25 [get_ports {data_print[0]}]
set_property PACKAGE_PIN AU32 [get_ports sys_clk_i_2]
set_property PACKAGE_PIN AW32 [get_ports sys_clk_i_1]
set_property PACKAGE_PIN F31 [get_ports finish_flag]
set_property PACKAGE_PIN F30 [get_ports init_calib_complete]
set_property PACKAGE_PIN C24 [get_ports int]
set_property PACKAGE_PIN F27 [get_ports load_data_ddr]
set_property PACKAGE_PIN F26 [get_ports load_ins_ddr]
set_property PACKAGE_PIN E29 [get_ports load_int_ins_ddr]
set_property PACKAGE_PIN D25 [get_ports sys_rst]
set_property PACKAGE_PIN F29 [get_ports wr_burst_data_req]
set_property PACKAGE_PIN F24 [get_ports data_print_rdy]
set_property PACKAGE_PIN B24 [get_ports {ins_input[29]}]
set_property PACKAGE_PIN E23 [get_ports {ins_input[28]}]
set_property PACKAGE_PIN E24 [get_ports {ins_input[27]}]
set_property PACKAGE_PIN F22 [get_ports {ins_input[26]}]
set_property PACKAGE_PIN E22 [get_ports {ins_input[25]}]
set_property PACKAGE_PIN F25 [get_ports {ins_input[24]}]
set_property PACKAGE_PIN E25 [get_ports {ins_input[23]}]
set_property PACKAGE_PIN D22 [get_ports {ins_input[22]}]
set_property PACKAGE_PIN D23 [get_ports {ins_input[21]}]
set_property PACKAGE_PIN D26 [get_ports {ins_input[20]}]
set_property PACKAGE_PIN C25 [get_ports {ins_input[19]}]
set_property PACKAGE_PIN C26 [get_ports {ins_input[18]}]
set_property PACKAGE_PIN D27 [get_ports {ins_input[17]}]
set_property PACKAGE_PIN D28 [get_ports {ins_input[16]}]
set_property PACKAGE_PIN C28 [get_ports {ins_input[15]}]
set_property PACKAGE_PIN C29 [get_ports {ins_input[14]}]
set_property PACKAGE_PIN B28 [get_ports {ins_input[13]}]
set_property PACKAGE_PIN B29 [get_ports {ins_input[12]}]
set_property PACKAGE_PIN A31 [get_ports {ins_input[11]}]
set_property PACKAGE_PIN A32 [get_ports {ins_input[10]}]
set_property PACKAGE_PIN A29 [get_ports {ins_input[9]}]
set_property PACKAGE_PIN A30 [get_ports {ins_input[8]}]
set_property PACKAGE_PIN C31 [get_ports {ins_input[7]}]
set_property PACKAGE_PIN B31 [get_ports {ins_input[6]}]
set_property PACKAGE_PIN E30 [get_ports {ins_input[5]}]
set_property PACKAGE_PIN D31 [get_ports {ins_input[4]}]
set_property PACKAGE_PIN D30 [get_ports {ins_input[3]}]
set_property PACKAGE_PIN C30 [get_ports {ins_input[2]}]
set_property PACKAGE_PIN E27 [get_ports {ins_input[1]}]
set_property PACKAGE_PIN E28 [get_ports {ins_input[0]}]
set_property IOSTANDARD LVCMOS18 [get_ports {data_input[15]}]
set_property IOSTANDARD LVCMOS18 [get_ports {data_input[14]}]
set_property IOSTANDARD LVCMOS18 [get_ports {data_input[13]}]
set_property IOSTANDARD LVCMOS18 [get_ports {data_input[12]}]
set_property IOSTANDARD LVCMOS18 [get_ports {data_input[11]}]
set_property IOSTANDARD LVCMOS18 [get_ports {data_input[10]}]
set_property IOSTANDARD LVCMOS18 [get_ports {data_input[9]}]
set_property IOSTANDARD LVCMOS18 [get_ports {data_input[8]}]
set_property IOSTANDARD LVCMOS18 [get_ports {data_input[7]}]
set_property IOSTANDARD LVCMOS18 [get_ports {data_input[6]}]
set_property IOSTANDARD LVCMOS18 [get_ports {data_input[5]}]
set_property IOSTANDARD LVCMOS18 [get_ports {data_input[4]}]
set_property IOSTANDARD LVCMOS18 [get_ports {data_input[3]}]
set_property IOSTANDARD LVCMOS18 [get_ports {data_input[2]}]
set_property IOSTANDARD LVCMOS18 [get_ports {data_input[1]}]
set_property IOSTANDARD LVCMOS18 [get_ports {data_input[0]}]
set_property IOSTANDARD LVCMOS18 [get_ports sys_clk_i_1]
set_property IOSTANDARD LVCMOS18 [get_ports sys_clk_i_2]
set_property IOSTANDARD LVCMOS18 [get_ports {data_print[15]}]
set_property IOSTANDARD LVCMOS18 [get_ports {data_print[14]}]
set_property IOSTANDARD LVCMOS18 [get_ports {data_print[13]}]
set_property IOSTANDARD LVCMOS18 [get_ports {data_print[12]}]
set_property IOSTANDARD LVCMOS18 [get_ports {data_print[11]}]
set_property IOSTANDARD LVCMOS18 [get_ports {data_print[10]}]
set_property IOSTANDARD LVCMOS18 [get_ports {data_print[9]}]
set_property IOSTANDARD LVCMOS18 [get_ports {data_print[8]}]
set_property IOSTANDARD LVCMOS18 [get_ports {data_print[7]}]
set_property IOSTANDARD LVCMOS18 [get_ports {data_print[6]}]
set_property IOSTANDARD LVCMOS18 [get_ports {data_print[5]}]
set_property IOSTANDARD LVCMOS18 [get_ports {data_print[4]}]
set_property IOSTANDARD LVCMOS18 [get_ports {data_print[3]}]
set_property IOSTANDARD LVCMOS18 [get_ports {data_print[2]}]
set_property IOSTANDARD LVCMOS18 [get_ports {data_print[1]}]
set_property IOSTANDARD LVCMOS18 [get_ports {data_print[0]}]
set_property IOSTANDARD LVCMOS18 [get_ports {ins_input[29]}]
set_property IOSTANDARD LVCMOS18 [get_ports {ins_input[28]}]
set_property IOSTANDARD LVCMOS18 [get_ports {ins_input[27]}]
set_property IOSTANDARD LVCMOS18 [get_ports {ins_input[26]}]
set_property IOSTANDARD LVCMOS18 [get_ports {ins_input[25]}]
set_property IOSTANDARD LVCMOS18 [get_ports {ins_input[24]}]
set_property IOSTANDARD LVCMOS18 [get_ports {ins_input[23]}]
set_property IOSTANDARD LVCMOS18 [get_ports {ins_input[22]}]
set_property IOSTANDARD LVCMOS18 [get_ports {ins_input[21]}]
set_property IOSTANDARD LVCMOS18 [get_ports {ins_input[20]}]
set_property IOSTANDARD LVCMOS18 [get_ports {ins_input[19]}]
set_property IOSTANDARD LVCMOS18 [get_ports {ins_input[18]}]
set_property IOSTANDARD LVCMOS18 [get_ports {ins_input[17]}]
set_property IOSTANDARD LVCMOS18 [get_ports {ins_input[16]}]
set_property IOSTANDARD LVCMOS18 [get_ports {ins_input[15]}]
set_property IOSTANDARD LVCMOS18 [get_ports {ins_input[14]}]
set_property IOSTANDARD LVCMOS18 [get_ports {ins_input[13]}]
set_property IOSTANDARD LVCMOS18 [get_ports {ins_input[12]}]
set_property IOSTANDARD LVCMOS18 [get_ports {ins_input[11]}]
set_property IOSTANDARD LVCMOS18 [get_ports {ins_input[10]}]
set_property IOSTANDARD LVCMOS18 [get_ports {ins_input[9]}]
set_property IOSTANDARD LVCMOS18 [get_ports {ins_input[8]}]
set_property IOSTANDARD LVCMOS18 [get_ports {ins_input[7]}]
set_property IOSTANDARD LVCMOS18 [get_ports {ins_input[6]}]
set_property IOSTANDARD LVCMOS18 [get_ports {ins_input[5]}]
set_property IOSTANDARD LVCMOS18 [get_ports {ins_input[4]}]
set_property IOSTANDARD LVCMOS18 [get_ports {ins_input[3]}]
set_property IOSTANDARD LVCMOS18 [get_ports {ins_input[2]}]
set_property IOSTANDARD LVCMOS18 [get_ports {ins_input[1]}]
set_property IOSTANDARD LVCMOS18 [get_ports {ins_input[0]}]
set_property IOSTANDARD LVCMOS18 [get_ports data_print_rdy]
set_property IOSTANDARD LVCMOS18 [get_ports finish_flag]
set_property IOSTANDARD LVCMOS18 [get_ports init_calib_complete]
set_property IOSTANDARD LVCMOS18 [get_ports int]
set_property IOSTANDARD LVCMOS18 [get_ports load_ins_ddr]
set_property IOSTANDARD LVCMOS18 [get_ports load_data_ddr]
set_property IOSTANDARD LVCMOS18 [get_ports load_int_ins_ddr]
set_property IOSTANDARD LVCMOS18 [get_ports sys_rst]
set_property IOSTANDARD LVCMOS18 [get_ports wr_burst_data_req]
