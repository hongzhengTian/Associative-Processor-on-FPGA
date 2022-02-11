create_clock -period 5.000 -name sys_clk_1 -waveform {0.000 2.500} -add [get_ports sys_clk_i_1]
create_clock -period 5.000 -name sys_clk_2 -waveform {0.000 2.500} -add [get_ports sys_clk_i_2]

