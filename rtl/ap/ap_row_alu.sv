// Bit-serial datapath, replicated for every row of the array.
//
// Each call processes one bit position of the operand fields in all rows at
// once: a and b hold that bit of every row, the per-row carry/flag registers
// link consecutive bit positions. Purely combinational; the registers live in
// ap_core.
//
// Arithmetic (one full adder per row, x + y + carry):
//   ADD  x = a,      y = b,  carry_in = 0
//   SUB  x = a,      y = ~b, carry_in = 1        a + ~b + 1
//   RSUB x = ~a,     y = b,  carry_in = 1        b + ~a + 1
//   NEG  x = ~a,     y = 0,  carry_in = 1
//   ABS  x = a ^ s,  y = 0,  carry_in = s        s = sign of a, per row
//
// Compare (bit positions are visited LSB first, so a higher bit overrides the
// verdict of the lower ones):
//   EQ/NE    flag = flag & (a == b)              flag starts at 1
//   LT/GE    flag = a < b at this bit, else flag  flag starts at 0
//   GT/LE    flag = a > b at this bit, else flag
// At the sign bit of a signed compare the sense of "less than" flips.
module ap_row_alu
#(
    parameter int ROWS = 256
) (
    input  logic [6:0]      alu_op,
    input  logic            is_cmp,
    input  logic [3:0]      cmp_cond,
    input  logic            first,        // first (least significant) bit
    input  logic            sign_bit,     // sign bit of a signed compare

    input  logic [ROWS-1:0] a,
    input  logic [ROWS-1:0] b,
    input  logic [ROWS-1:0] carry_q,
    input  logic [ROWS-1:0] flag_q,

    output logic [ROWS-1:0] result,
    output logic [ROWS-1:0] carry_d,
    output logic [ROWS-1:0] flag_d,
    output logic [ROWS-1:0] cond          // compare outcome so far
);

    logic [ROWS-1:0] x, y, cin0, cin, sum;
    logic [ROWS-1:0] eq_acc, lt_acc, same;

    // ------------------------------------------------------------ arithmetic
    always_comb begin
        x    = a;
        y    = b;
        cin0 = '0;
        case (alu_op)
            ap_pkg::ALU_ADD:  begin x = a;          y = b;  cin0 = '0;     end
            ap_pkg::ALU_SUB:  begin x = a;          y = ~b; cin0 = '1;     end
            ap_pkg::ALU_RSUB: begin x = ~a;         y = b;  cin0 = '1;     end
            ap_pkg::ALU_NEG:  begin x = ~a;         y = '0; cin0 = '1;     end
            ap_pkg::ALU_ABS:  begin x = a ^ flag_q; y = '0; cin0 = flag_q; end
            default: ;
        endcase
        cin     = first ? cin0 : carry_q;
        sum     = x ^ y ^ cin;
        carry_d = (x & y) | (x & cin) | (y & cin);

        case (alu_op)
            ap_pkg::ALU_AND: result = a & b;
            ap_pkg::ALU_OR:  result = a | b;
            ap_pkg::ALU_XOR: result = a ^ b;
            ap_pkg::ALU_MOV: result = a;
            ap_pkg::ALU_NOT: result = ~a;
            ap_pkg::ALU_SET: result = b;
            default: result = sum;
        endcase
    end

    // --------------------------------------------------------------- compare
    always_comb begin
        same   = ~(a ^ b);
        eq_acc = first ? '1 : flag_q;
        lt_acc = first ? '0 : flag_q;

        case (cmp_cond)
            ap_pkg::CMP_EQ, ap_pkg::CMP_NE:
                flag_d = eq_acc & same;
            ap_pkg::CMP_LTU, ap_pkg::CMP_LT, ap_pkg::CMP_GEU, ap_pkg::CMP_GE:
                flag_d = sign_bit ? ((a & ~b) | (same & lt_acc))
                                  : ((~a & b) | (same & lt_acc));
            ap_pkg::CMP_GTU, ap_pkg::CMP_GT, ap_pkg::CMP_LEU, ap_pkg::CMP_LE:
                flag_d = sign_bit ? ((~a & b) | (same & lt_acc))
                                  : ((a & ~b) | (same & lt_acc));
            default:
                flag_d = flag_q;
        endcase

        case (cmp_cond)
            ap_pkg::CMP_NE, ap_pkg::CMP_GEU, ap_pkg::CMP_GE,
            ap_pkg::CMP_LEU, ap_pkg::CMP_LE: cond = ~flag_d;
            default:                         cond = flag_d;
        endcase

        // Outside a compare the flag register is left alone (ABS keeps the
        // sign there).
        if (!is_cmp) flag_d = flag_q;
    end

endmodule
