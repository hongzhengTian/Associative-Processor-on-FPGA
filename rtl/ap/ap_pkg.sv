// Shared definitions of the associative processing (AP) unit.
//
// AP instructions use the RISC-V custom-1 major opcode (0101011) in R-type
// format. funct3 selects an instruction group and funct7 the operation inside
// the group. The full instruction set is described in docs/isa.md; the values
// below must stay in sync with sw/include/ap.h.
package ap_pkg;

    localparam logic [6:0] OPCODE_CUSTOM1 = 7'b0101011;

    localparam logic [31:0] AP_VERSION = 32'h0002_0000;  // 2.0.0

    // ---------------------------------------------------------------- groups
    localparam logic [2:0] GRP_VV   = 3'd0;  // d = op(a, b), field b
    localparam logic [2:0] GRP_VS   = 3'd1;  // d = op(a, s), scalar s = rs2
    localparam logic [2:0] GRP_CMP  = 3'd2;  // compare, result into the tag
    localparam logic [2:0] GRP_TAG  = 3'd3;  // tag operations and reductions
    localparam logic [2:0] GRP_ELEM = 3'd4;  // single element access
    localparam logic [2:0] GRP_XBUF = 3'd5;  // transpose buffer
    localparam logic [2:0] GRP_DMA  = 3'd6;  // DMA to/from external memory
    localparam logic [2:0] GRP_SYS  = 3'd7;  // configuration queries

    // ----------------------------------------- GRP_VV / GRP_VS (funct7 value)
    localparam logic [6:0] ALU_ADD  = 7'd0;   // d = a + b
    localparam logic [6:0] ALU_SUB  = 7'd1;   // d = a - b
    localparam logic [6:0] ALU_AND  = 7'd2;   // d = a & b
    localparam logic [6:0] ALU_OR   = 7'd3;   // d = a | b
    localparam logic [6:0] ALU_XOR  = 7'd4;   // d = a ^ b
    localparam logic [6:0] ALU_MOV  = 7'd5;   // d = a
    localparam logic [6:0] ALU_NOT  = 7'd6;   // d = ~a
    localparam logic [6:0] ALU_NEG  = 7'd7;   // d = -a
    localparam logic [6:0] ALU_ABS  = 7'd8;   // d = |a|  (a signed)
    localparam logic [6:0] ALU_SET  = 7'd9;   // d = b
    localparam logic [6:0] ALU_RSUB = 7'd10;  // d = b - a
    localparam int unsigned ALU_LAST = 10;

    // ------------------------------------------------- GRP_CMP (funct7 fields)
    // funct7 = {mode[1:0], scalar, cond[3:0]}
    localparam logic [3:0] CMP_EQ  = 4'd0;
    localparam logic [3:0] CMP_NE  = 4'd1;
    localparam logic [3:0] CMP_LTU = 4'd2;
    localparam logic [3:0] CMP_LT  = 4'd3;
    localparam logic [3:0] CMP_GEU = 4'd4;
    localparam logic [3:0] CMP_GE  = 4'd5;
    localparam logic [3:0] CMP_GTU = 4'd6;
    localparam logic [3:0] CMP_GT  = 4'd7;
    localparam logic [3:0] CMP_LEU = 4'd8;
    localparam logic [3:0] CMP_LE  = 4'd9;
    localparam int unsigned CMP_LAST = 9;

    localparam logic [1:0] TMODE_SET = 2'd0;  // tag = cond
    localparam logic [1:0] TMODE_AND = 2'd1;  // tag = tag & cond
    localparam logic [1:0] TMODE_OR  = 2'd2;  // tag = tag | cond

    // ------------------------------------------------ GRP_TAG (funct7 value)
    localparam logic [6:0] TAG_ALL   = 7'd0;   // tag = 1 for every row
    localparam logic [6:0] TAG_NONE  = 7'd1;   // tag = 0
    localparam logic [6:0] TAG_NOT   = 7'd2;   // tag = ~tag
    localparam logic [6:0] TAG_RANGE = 7'd3;   // tag = (rs1 <= row < rs2)
    localparam logic [6:0] TAG_SAVE  = 7'd4;   // bit field = tag
    localparam logic [6:0] TAG_LOAD  = 7'd5;   // tag = bit field
    localparam logic [6:0] TAG_COUNT = 7'd6;   // rd = number of tagged rows
    localparam logic [6:0] TAG_FIRST = 7'd7;   // rd = first tagged row or ~0
    localparam logic [6:0] TAG_MAXU  = 7'd8;   // keep rows holding the maximum
    localparam logic [6:0] TAG_MINU  = 7'd9;
    localparam logic [6:0] TAG_MAX   = 7'd10;
    localparam logic [6:0] TAG_MIN   = 7'd11;

    // ----------------------------------------------- GRP_ELEM (funct7 value)
    localparam logic [6:0] ELEM_RD = 7'd0;     // rd = field of one row
    localparam logic [6:0] ELEM_WR = 7'd1;     // field of one row = rs2

    // ----------------------------------------------- GRP_XBUF (funct7 value)
    localparam logic [6:0] XBUF_WR  = 7'd0;    // xbuf[rs1] = rs2
    localparam logic [6:0] XBUF_RD  = 7'd1;    // rd = xbuf[rs1]
    localparam logic [6:0] XBUF_PUT = 7'd2;    // 32 rows of a field = xbuf
    localparam logic [6:0] XBUF_GET = 7'd3;    // xbuf = 32 rows of a field

    // ------------------------------------------------ GRP_DMA (funct7 value)
    localparam logic [6:0] DMA_CFG   = 7'd0;   // address = rs1, rows = rs2
    localparam logic [6:0] DMA_LOAD  = 7'd1;   // memory -> field
    localparam logic [6:0] DMA_STORE = 7'd2;   // field -> memory

    // ------------------------------------------------ GRP_SYS (funct7 value)
    localparam logic [6:0] SYS_INFO = 7'd0;    // rd = parameter selected by rs1

    localparam logic [31:0] INFO_ROWS    = 32'd0;
    localparam logic [31:0] INFO_COLS    = 32'd1;
    localparam logic [31:0] INFO_VERSION = 32'd2;

    // Field descriptor: {len - 1 [15:11], offset [10:0]}.
    localparam int unsigned FIELD_LEN_BITS = 5;
    localparam int unsigned FIELD_OFF_BITS = 11;
    localparam int unsigned MAX_FIELD_LEN  = 32;
    localparam int unsigned MAX_COLS       = 2048;
    localparam int unsigned MAX_ROWS       = 65536;

endpackage
