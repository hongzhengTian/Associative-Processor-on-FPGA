// Bit-plane storage of the AP array.
//
// The array holds ROWS words of COLS bits. It is stored transposed: entry c
// ("plane" c) holds bit c of every row, so one access reads or writes the same
// bit position of all rows at once, which is what bit-serial word-parallel
// processing needs.
//
// Two asynchronous read ports and one write port with a per-row write mask.
// The mask is how an operation touches only the tagged rows. A per-bit write
// mask rules out block RAM; on Xilinx devices each row maps to distributed
// (LUT) RAM, duplicated for the second read port.
module ap_plane_mem #(
    parameter int ROWS = 256,
    parameter int COLS = 128,
    localparam int AW = (COLS > 1) ? $clog2(COLS) : 1
) (
    input  logic            clk,

    input  logic [AW-1:0]   ra_addr,
    output logic [ROWS-1:0] ra_data,

    input  logic [AW-1:0]   rb_addr,
    output logic [ROWS-1:0] rb_data,

    input  logic [AW-1:0]   w_addr,
    input  logic [ROWS-1:0] w_data,
    input  logic [ROWS-1:0] w_mask
);

    // One COLS x 1 memory per row: the write mask becomes that row's write
    // enable, and both read ports share the plane address.
    for (genvar r = 0; r < ROWS; r++) begin : g_row
        logic bits [COLS];

        // Configuration-time contents (FPGA RAMs start at zero as well).
        initial begin
            for (int c = 0; c < COLS; c++) bits[c] = 1'b0;
        end

        always_ff @(posedge clk) begin
            if (w_mask[r]) bits[w_addr] <= w_data[r];
        end

        assign ra_data[r] = bits[ra_addr];
        assign rb_data[r] = bits[rb_addr];
    end

endmodule
