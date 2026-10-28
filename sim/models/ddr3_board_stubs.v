`timescale 1ps/1ps
// Empty stand-ins for the board-level models that AP_top_tb instantiates
// (Micron ddr3_model and the MIG example design's WireDelay). With the
// behavioral mig_7series_0 the DDR3 pins carry nothing, so these do nothing.
module ddr3_model (
    input           rst_n,
    input           ck,
    input           ck_n,
    input           cke,
    input           cs_n,
    input           ras_n,
    input           cas_n,
    input           we_n,
    inout           dm_tdqs,
    input  [2:0]    ba,
    input  [13:0]   addr,
    inout  [7:0]    dq,
    inout           dqs,
    inout           dqs_n,
    output          tdqs_n,
    input           odt
);
endmodule

module WireDelay #(
    parameter Delay_g    = 0,
    parameter Delay_rd   = 0,
    parameter ERR_INSERT = "OFF"
) (
    inout           A,
    inout           B,
    input           reset,
    input           phy_init_done
);
endmodule
