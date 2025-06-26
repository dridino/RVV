module vec_alu #(
	parameter [9:0] VLEN = 10'd 128,
    parameter [2:0] LANE_WIDTH = 3'b100, // 2^LANE_WIDTH bits per lane (8,16,32,64)
    // LANE_WIDTH * 2^nb_lanes must be less than or equal to VLEN
    parameter [2:0] LANE_I = 3'b000
) (
    input               clk, resetn,
    input      [1:0]    nb_lanes, // 2^nb_lanes lanes used for arith / logic operations
    input      [5:0]    opcode,
    input               run,
    input      [VLEN-1:0] vs1_in,
    input      [VLEN-1:0] vs2_in,
    input      [2:0]    vsew,
    input      [2:0]    op_type, // 001 : vv | 010 : VX | 100 : VI

    input      [9:0]    index,
    input      [3:0]    in_reg_offset,

    output [63:0] vd
    // output [9:0] reg_index,
    // output reg       done
);
    localparam [7:0] SHIFTED_LANE_WIDTH = 1 << LANE_WIDTH;
    localparam [7:0] SHIFTED_LANE_WIDTH_M1 = SHIFTED_LANE_WIDTH - 1;
    localparam [8:0] ADD_SHIFTED_LANE_WIDTH = SHIFTED_LANE_WIDTH + 1;
	localparam [2:0] VV = 3'b001;
	localparam [2:0] VX = 3'b010;
	localparam [2:0] VI = 3'b100;

    // reg [9:0]  byte_i;
    reg [64:0] temp_vreg; // 64 + 1 for carry out
    // reg [3:0]  in_reg_offset;

    // set to 0 when computing a new element of the vector
    wire cout = (in_reg_offset == (vsew + 3 <= LANE_WIDTH ? 0 : (1 << (vsew+3-LANE_WIDTH)) - 1)) ? 0 : temp_vreg[ADD_SHIFTED_LANE_WIDTH - 1];

    reg cout_q;

    /* integer index;

    assign reg_index = index; */

    assign vd = temp_vreg[0 +: 64];

    always @* begin
        if (!resetn) begin
            temp_vreg = {65{1'b0}};
            // index = 0;
        end else if (run) begin
            // index = ((LANE_I + byte_i) << (vsew + 3)) + (in_reg_offset << LANE_WIDTH);
            temp_vreg = 0;
            case (opcode)
                // vand
                6'b001001: begin
                    temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vs1_in[op_type == VV ? index : (in_reg_offset << LANE_WIDTH) +: SHIFTED_LANE_WIDTH] & vs2_in[index +: SHIFTED_LANE_WIDTH];
                end
                // vor
                6'b001010: begin
                    temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vs1_in[op_type == VV ? index : (in_reg_offset << LANE_WIDTH) +: SHIFTED_LANE_WIDTH] | vs2_in[index +: SHIFTED_LANE_WIDTH];
                end
                // vxor
                6'b001011: begin
                    temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vs1_in[op_type == VV ? index : (in_reg_offset << LANE_WIDTH) +: SHIFTED_LANE_WIDTH] ^ vs2_in[index +: SHIFTED_LANE_WIDTH];
                end
                // vadd
                6'b000000: begin
                    temp_vreg[0 +: ADD_SHIFTED_LANE_WIDTH] = vs1_in[op_type == VV ? index : (in_reg_offset << LANE_WIDTH) +: SHIFTED_LANE_WIDTH] + vs2_in[index +: SHIFTED_LANE_WIDTH] + cout_q;
                end
                default: begin
                    temp_vreg = 0;
                end
            endcase
        end else begin
            temp_vreg = 0;
            // index = 0;
        end
    end

    always @(posedge clk) begin
        cout_q <= cout;
        /* if (!resetn) begin
            byte_i <= 0;
            in_reg_offset <= 0;
            done <= 0;
        end else if (run) begin
            if (!done) begin
                // $display("lane%d byte_i : %d, reg_off : %d, index : %d", LANE_I, byte_i, in_reg_offset, index);
                if (vsew+3 <= LANE_WIDTH) begin
                    done <= byte_i + (1 << (nb_lanes+1)) >= (VLEN >> (vsew + 3));
                end else begin
                    done <= byte_i + (1 << (nb_lanes)) >= (VLEN >> (vsew + 3)) && in_reg_offset == ((1 << (vsew+3-LANE_WIDTH)) - 2);
                end

                if (vsew + 3 < LANE_WIDTH || in_reg_offset == (vsew + 3 <= LANE_WIDTH ? 0 : (1 << (vsew+3-LANE_WIDTH)) - 1)) begin
                    in_reg_offset <= 0;
                    byte_i <= byte_i + (1<<nb_lanes);
                end else
                    in_reg_offset <= in_reg_offset + 1;
            end else
                done <= 0;
        end else begin
            byte_i <= 0;
            in_reg_offset <= 0;
            done <= 0;
        end */
    end
endmodule

/*
4.26. Printing statistics.

=== $paramod\vec_alu\VLEN=10'0010000000\LANE_WIDTH=3'100\LANE_I=3'000 ===

   Number of wires:               1546
   Number of wire bits:           1949
   Number of public wires:          15
   Number of public wire bits:     418
   Number of memories:               0
   Number of memory bits:            0
   Number of processes:              0
   Number of cells:               1550
     $_ANDNOT_                     211
     $_AND_                         19
     $_AOI3_                        29
     $_DFF_P_                        1
     $_MUX_                        469
     $_NAND_                        31
     $_NOR_                         18
     $_NOT_                         62
     $_OAI3_                       198
     $_OAI4_                        15
     $_ORNOT_                      197
     $_OR_                         232
     $_XNOR_                        16
     $_XOR_                         52

=== $paramod\vec_alu\VLEN=10'0010000000\LANE_WIDTH=3'100\LANE_I=3'001 ===

   Number of wires:               1531
   Number of wire bits:           1886
   Number of public wires:          15
   Number of public wire bits:     370
   Number of memories:               0
   Number of memory bits:            0
   Number of processes:              0
   Number of cells:               1535
     $_ANDNOT_                     196
     $_AND_                         18
     $_AOI3_                        31
     $_DFF_P_                        1
     $_MUX_                        469
     $_NAND_                        46
     $_NOR_                         15
     $_NOT_                         34
     $_OAI3_                       184
     $_OAI4_                        15
     $_ORNOT_                      211
     $_OR_                         248
     $_XNOR_                        17
     $_XOR_                         50

=== $paramod\vec_alu\VLEN=10'0010000000\LANE_WIDTH=3'100\LANE_I=3'010 ===

   Number of wires:               1531
   Number of wire bits:           1886
   Number of public wires:          15
   Number of public wire bits:     370
   Number of memories:               0
   Number of memory bits:            0
   Number of processes:              0
   Number of cells:               1535
     $_ANDNOT_                     196
     $_AND_                         18
     $_AOI3_                        31
     $_DFF_P_                        1
     $_MUX_                        469
     $_NAND_                        46
     $_NOR_                         15
     $_NOT_                         34
     $_OAI3_                       184
     $_OAI4_                        15
     $_ORNOT_                      211
     $_OR_                         248
     $_XNOR_                        17
     $_XOR_                         50

=== $paramod\vec_alu\VLEN=10'0010000000\LANE_WIDTH=3'100\LANE_I=3'011 ===

   Number of wires:               1546
   Number of wire bits:           1949
   Number of public wires:          15
   Number of public wire bits:     418
   Number of memories:               0
   Number of memory bits:            0
   Number of processes:              0
   Number of cells:               1550
     $_ANDNOT_                     211
     $_AND_                         19
     $_AOI3_                        29
     $_DFF_P_                        1
     $_MUX_                        469
     $_NAND_                        31
     $_NOR_                         18
     $_NOT_                         62
     $_OAI3_                       198
     $_OAI4_                        15
     $_ORNOT_                      197
     $_OR_                         232
     $_XNOR_                        16
     $_XOR_                         52

=== vec_alu_wrapper ===

   Number of wires:                910
   Number of wire bits:           1522
   Number of public wires:          34
   Number of public wire bits:     634
   Number of memories:               0
   Number of memory bits:            0
   Number of processes:              0
   Number of cells:                945
     $_ANDNOT_                     173
     $_AND_                         31
     $_AOI3_                        63
     $_DFF_P_                       15
     $_MUX_                        129
     $_NAND_                        25
     $_NOR_                         53
     $_NOT_                         67
     $_OAI3_                        40
     $_ORNOT_                       61
     $_OR_                         105
     $_XNOR_                        33
     $_XOR_                        146
     $paramod\vec_alu\VLEN=10'0010000000\LANE_WIDTH=3'100\LANE_I=3'000      1
     $paramod\vec_alu\VLEN=10'0010000000\LANE_WIDTH=3'100\LANE_I=3'001      1
     $paramod\vec_alu\VLEN=10'0010000000\LANE_WIDTH=3'100\LANE_I=3'010      1
     $paramod\vec_alu\VLEN=10'0010000000\LANE_WIDTH=3'100\LANE_I=3'011      1

=== design hierarchy ===

   vec_alu_wrapper                   1
     $paramod\vec_alu\VLEN=10'0010000000\LANE_WIDTH=3'100\LANE_I=3'000      1
     $paramod\vec_alu\VLEN=10'0010000000\LANE_WIDTH=3'100\LANE_I=3'001      1
     $paramod\vec_alu\VLEN=10'0010000000\LANE_WIDTH=3'100\LANE_I=3'010      1
     $paramod\vec_alu\VLEN=10'0010000000\LANE_WIDTH=3'100\LANE_I=3'011      1

   Number of wires:               7064
   Number of wire bits:           9192
   Number of public wires:          94
   Number of public wire bits:    2210
   Number of memories:               0
   Number of memory bits:            0
   Number of processes:              0
   Number of cells:               7111
     $_ANDNOT_                     987
     $_AND_                        105
     $_AOI3_                       183
     $_DFF_P_                       19
     $_MUX_                       2005
     $_NAND_                       179
     $_NOR_                        119
     $_NOT_                        259
     $_OAI3_                       804
     $_OAI4_                        60
     $_ORNOT_                      877
     $_OR_                        1065
     $_XNOR_                        99
     $_XOR_                        350
*/