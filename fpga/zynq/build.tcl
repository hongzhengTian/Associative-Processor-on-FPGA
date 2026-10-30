# Builds the AP SoC for a Zynq-7000 board: a block design with the processing
# system (PS7) and ap_zynq, implementation, bitstream, and the hardware
# platform (.xsa) for Vitis.
#
#   vivado -mode batch -source fpga/zynq/build.tcl -tclargs preset=<ps7.tcl> [key=value ...]
#
# Options:
#   preset=<file>  PS7 configuration of the board (DDR, clocks, MIO/UART):
#                  a preset saved from a working project of the board, or
#                  the ps_config.tcl of an ALINX example (docs/zynq.md).
#                  Required: it is the only board-specific input.
#   part=<part>    FPGA part                        default xc7z020clg400-2
#   rows=<n>       AP array rows                    default 512
#   cols=<n>       AP array columns                 default 64
#   fclk=<MHz>     clock of the AP SoC (FCLK_CLK0)  default 50
#   jobs=<n>       parallel jobs                    default 4
#   dir=<path>     project directory                default ./vivado_zynq
#
# Block design:
#   PS7 M_AXI_GP0 -> AXI Interconnect -> ap_0/s_axi_ctrl   at 0x43C0_0000 (128 KiB)
#   ap_0/m_axi_mem -> AXI Interconnect -> PS7 S_AXI_HP0     (DDR)
#   FCLK_CLK0 clocks everything; a Processor System Reset drives aresetn.
# No PL pins are used, so no constraints file is needed.

# Globals carry an ap_ prefix: while runs are in progress, the Vivado GUI runs
# its own scripts at global level, and they overwrite common names like dir.
set ap_root [file normalize [file join [file dirname [info script]] .. ..]]

array set ap_opt {
    preset ""
    part   xc7z020clg400-2
    rows   512
    cols   64
    fclk   50
    jobs   4
    dir    vivado_zynq
}
foreach arg $argv {
    if {![regexp {^([a-z]+)=(.*)$} $arg -> key value] || ![info exists ap_opt($key)]} {
        error "unknown argument '$arg' (see the header of [info script])"
    }
    set ap_opt($key) $value
}
if {$ap_opt(preset) eq ""} {
    error "preset=<file> is required: the PS7 configuration of the board (see docs/zynq.md)"
}
set ap_preset [file normalize $ap_opt(preset)]
set ap_dir [file normalize $ap_opt(dir)]

# Newest installed version of a Xilinx IP.
proc latest_ip {name} {
    set defs [lsort -dictionary [get_ipdefs -all "xilinx.com:ip:${name}:*"]]
    if {[llength $defs] == 0} { error "IP $name is not installed" }
    return [lindex $defs end]
}

# ------------------------------------------------------------------ project
create_project ap_zynq $ap_dir -part $ap_opt(part) -force
# Work in the project directory, so that files the tools drop into the
# current directory (such as the PS7 summary) stay with the build output.
set ap_cwd [pwd]
cd $ap_dir
set_property target_language Verilog [current_project]

set rtl {
    rtl/ap/ap_pkg.sv
    rtl/ap/ap_plane_mem.sv
    rtl/ap/ap_row_alu.sv
    rtl/ap/ap_core.sv
    rtl/ap/ap_dma.sv
    rtl/ap/ap_unit.sv
    rtl/soc/ap_pcpi.sv
    rtl/soc/soc_ram.sv
    rtl/soc/ap_soc.sv
    rtl/zynq/ap_host_bridge.sv
    rtl/zynq/ap_zynq.v
    third_party/picorv32/picorv32.v
}
foreach f $rtl { add_files -norecurse [file join $ap_root $f] }
set_property file_type SystemVerilog [get_files -filter {NAME =~ "*.sv"}]
update_compile_order -fileset sources_1

# ------------------------------------------------------------- block design
create_bd_design system

set ps7 [create_bd_cell -type ip -vlnv [latest_ip processing_system7] ps7]
source $ap_preset
if {[llength [info procs apply_preset]]} {
    # Saved with Presets > Save Configuration in the PS7 dialog.
    set_property -dict [apply_preset $ps7] $ps7
} elseif {[llength [info procs set_ps_config]]} {
    # ps_config.tcl of an ALINX example project (auto_create_project).
    set_ps_config $ps7
} else {
    error "$ap_preset defines neither apply_preset nor set_ps_config (see docs/zynq.md)"
}
set_property -dict [list \
    CONFIG.PCW_USE_M_AXI_GP0            {1} \
    CONFIG.PCW_USE_S_AXI_HP0            {1} \
    CONFIG.PCW_S_AXI_HP0_DATA_WIDTH     {32} \
    CONFIG.PCW_EN_CLK0_PORT             {1} \
    CONFIG.PCW_EN_RST0_PORT             {1} \
    CONFIG.PCW_FPGA0_PERIPHERAL_FREQMHZ $ap_opt(fclk) \
] $ps7
apply_bd_automation -rule xilinx.com:bd_rule:processing_system7 \
    -config {make_external "FIXED_IO, DDR" apply_board_preset "0" Master "Disable" Slave "Disable"} $ps7

set ap [create_bd_cell -type module -reference ap_zynq ap_0]
set_property -dict [list CONFIG.ROWS $ap_opt(rows) CONFIG.COLS $ap_opt(cols)] $ap

# control: PS master -> AP SoC registers and program RAM
apply_bd_automation -rule xilinx.com:bd_rule:axi4 -config { \
    Clk_master {Auto} Clk_slave {Auto} Clk_xbar {Auto} \
    Master {/ps7/M_AXI_GP0} Slave {/ap_0/s_axi_ctrl} \
    ddr_seg {Auto} intc_ip {New AXI Interconnect} master_apm {0} \
} [get_bd_intf_pins ap_0/s_axi_ctrl]

# data: AP SoC master -> DDR through HP0 (the interconnect splits the SoC's
# AXI4 bursts of up to 32 beats for the AXI3 port)
apply_bd_automation -rule xilinx.com:bd_rule:axi4 -config { \
    Clk_master {Auto} Clk_slave {Auto} Clk_xbar {Auto} \
    Master {/ap_0/m_axi_mem} Slave {/ps7/S_AXI_HP0} \
    ddr_seg {Auto} intc_ip {New AXI Interconnect} master_apm {0} \
} [get_bd_intf_pins ps7/S_AXI_HP0]

assign_bd_address
set ctrl_seg [get_bd_addr_segs -of_objects [get_bd_addr_spaces ps7/Data] -filter {NAME =~ "*ap_0*"}]
if {[llength $ctrl_seg] != 1} { error "address segment of ap_0/s_axi_ctrl not found" }
set_property offset 0x43C00000 $ctrl_seg
set_property range  128K       $ctrl_seg

validate_bd_design
save_bd_design

set bd_file [get_files system.bd]
generate_target all $bd_file
set wrapper [make_wrapper -files $bd_file -top]
add_files -norecurse $wrapper
set_property top system_wrapper [current_fileset]
update_compile_order -fileset sources_1

# -------------------------------------------------------- implementation
launch_runs synth_1 -jobs $ap_opt(jobs)
wait_on_run synth_1
if {[get_property PROGRESS [get_runs synth_1]] ne "100%"} { error "synthesis failed" }

launch_runs impl_1 -to_step write_bitstream -jobs $ap_opt(jobs)
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] ne "100%"} { error "implementation failed" }

open_run impl_1
report_utilization    -file [file join $ap_dir utilization.txt]
report_timing_summary -file [file join $ap_dir timing.txt]
set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]

write_hw_platform -fixed -include_bit -force -file [file join $ap_dir ap_zynq.xsa]

puts ""
puts "AP SoC for Zynq: $ap_opt(rows) x $ap_opt(cols) array, FCLK_CLK0 $ap_opt(fclk) MHz, part $ap_opt(part)"
puts "  hardware platform: [file join $ap_dir ap_zynq.xsa]"
puts "  reports:           [file join $ap_dir utilization.txt], [file join $ap_dir timing.txt]"
puts "  worst setup slack: $wns ns"
if {$wns < 0} {
    puts "  WARNING: timing is not met; rebuild with a lower fclk=<MHz>"
}
cd $ap_cwd
