##################################################################################################
## 
##  Xilinx, Inc. 2010            www.xilinx.com 
##  Tue May 10 11:17:46 2022

##  Generated by MIG Version 4.2
##  
##################################################################################################
##  File name :       example_top.sd
##  Details :     Constraints file
##                    FPGA Family:       VIRTEX7
##                    FPGA Part:         XC7VX485TFFG1761_PKG
##                    Speedgrade:        -2
##                    Design Entry:      VERILOG
##                    Frequency:         800 MHz
##                    Time Period:       1250 ps
##################################################################################################

##################################################################################################
## Controller 0
## Memory Device: DDR3_SDRAM->Components->MT41J128M8XX-125
## Data Width: 16
## Time Period: 1250
## Data Mask: 1
##################################################################################################

set_property IO_BUFFER_TYPE NONE [get_ports {ddr3_ck_n[*]} ]
set_property IO_BUFFER_TYPE NONE [get_ports {ddr3_ck_p[*]} ]
          
#create_clock -period 5 [get_ports sys_clk_i]
          
############## NET - IOSTANDARD ##################


