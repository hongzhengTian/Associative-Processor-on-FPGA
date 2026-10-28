# Recreate the Vivado project from the sources in this repository.
#
#   vivado -mode batch -source scripts/create_project.tcl
#
# The project is created in ./vivado (ignored by git), and the IP
# configurations in ip/ are copied there before Vivado generates their output
# products, so ip/ keeps holding only the .xci / .prj files.
#
# The original project was built with Vivado 2018.3. Newer versions open the
# IP locked; it is upgraded below, which may change its version numbers.
# The target is a Virtex-7 (xc7vx690t), which the free Vivado editions do not
# support.

set root      [file normalize [file join [file dirname [info script]] ..]]
set proj_name AP_5v
set proj_dir  [file join $root vivado]
set part      xc7vx690tffg1761-2

create_project $proj_name $proj_dir -part $part -force
set_property target_language Verilog [current_project]
set_property simulator_language Mixed [current_project]

# RTL
add_files -norecurse -fileset sources_1 [glob [file join $root rtl *.v]]

# IP: MIG DDR3 controller and the two clock-crossing FIFOs
set ip_dir [file join $proj_dir ip]
foreach ip {mig_7series_0 fifo_generator_0 fifo_generator_1} {
    file mkdir [file join $ip_dir $ip]
    foreach f [glob [file join $root ip $ip *]] {
        file copy -force $f [file join $ip_dir $ip]
    }
    add_files -norecurse -fileset sources_1 [file join $ip_dir $ip $ip.xci]
}
set locked [get_ips -quiet -filter {IS_LOCKED == 1}]
if {[llength $locked] > 0} {
    upgrade_ip $locked
}
generate_target all [get_ips]

set_property top AP_top [get_filesets sources_1]

# Constraints
add_files -norecurse -fileset constrs_1 [file join $root constraints AP_top.xdc]

# Simulation: testbench, Micron DDR3 model from the MIG example design,
# program/data files (Vivado copies them into the xsim run directory)
set mig_sim [file join $ip_dir mig_7series_0 mig_7series_0 example_design sim]
add_files -norecurse -fileset sim_1 [list \
    [file join $root sim tb AP_top_tb.v] \
    [file join $mig_sim ddr3_model.sv] \
    [file join $mig_sim ddr3_model_parameters.vh] \
    [file join $mig_sim wiredly.v] \
]
add_files -norecurse -fileset sim_1 [glob [file join $root sim programs *.txt]]
add_files -norecurse -fileset sim_1 [file join $root sim tb AP_top_tb_behav.wcfg]
set_property top AP_top_tb [get_filesets sim_1]
set_property top_lib xil_defaultlib [get_filesets sim_1]
set_property xsim.view [file join $root sim tb AP_top_tb_behav.wcfg] [get_filesets sim_1]
set_property -name {xsim.simulate.runtime} -value {all} -objects [get_filesets sim_1]

update_compile_order -fileset sources_1
update_compile_order -fileset sim_1

puts "Project created: [file join $proj_dir $proj_name.xpr]"
