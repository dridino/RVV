`define min(a,b) (a < b ? a : b)

module rvv_alu #(
	parameter [9:0] VLEN = 10'd 128,
    parameter [2:0] LANE_WIDTH = 3'b011, // 2^LANE_WIDTH bits per lane (8,16,32,64)
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

    input      [9:0]    byte_i,
    input      [3:0]    in_reg_offset,

    output [63:0] vd,
    output [9:0]  index,
    output        instr_valid
);
    localparam [7:0] SHIFTED_LANE_WIDTH = 1 << LANE_WIDTH;
    localparam [8:0] ADD_SHIFTED_LANE_WIDTH = SHIFTED_LANE_WIDTH + 1;
	localparam [2:0] VV = 3'b001;
	localparam [2:0] VX = 3'b010;
	localparam [2:0] VI = 3'b100;

    assign instr_valid =
        (opcode == 6'b001001) | (opcode == 6'b001010) | (opcode == 6'b001011) |
        (opcode == 6'b000000) | (opcode == 6'b000010) | (opcode == 6'b000011) |
        (opcode == 6'b000100) | (opcode == 6'b000101) | (opcode == 6'b000110) |
        (opcode == 6'b000111) | (opcode == 6'b100101);

    assign vd = temp_vreg[0 +: 64];
    reg [64:0] temp_vreg; // 64 + 1 for carry out
    
    assign index = index_val;
    wire [9:0] base_index = ((LANE_I + byte_i) << (vsew + 3));
    wire [9:0] index_val =
        (opcode[5:2] == 4'b0001) ? base_index + (((1 << (vsew+3-LANE_WIDTH)) - 1) << LANE_WIDTH) - (in_reg_offset << LANE_WIDTH) : // cmp op : reversed index
        base_index + (in_reg_offset << LANE_WIDTH); // classic op index
    
    // set to 0 when computing a new element of the vector
    // set to 1 if it's a sub operation
    wire cout = (in_reg_offset == (vsew + 3 <= LANE_WIDTH ? 0 : (1 << (vsew+3-LANE_WIDTH)) - 1)) ? 0 : temp_vreg[ADD_SHIFTED_LANE_WIDTH - 1];
    reg cout_q;

    // carry for comparisons with vsew > LANE_WIDTH
    // 001 : not evaluated
    // 010 : vs1 <= vs2
    // 100 : vs2 <  vs1
    wire [2:0] cmp_c =
        (in_reg_offset == 0) ? 3'b001 : // reset when new element
        (!cmp_c[0]) ? cmp_c : // keep its state for the whole element
        (opcode[5:2] == 4'b0001 && !opcode[0] && !cmp_c[1] && ltu) ? 3'b100 :  // unsigned && vs2 <  vs1
        (opcode[5:2] == 4'b0001 && !opcode[0] && !cmp_c[2] && !ltu) ? 3'b010 : // unsigned && vs2 >= vs1
        (opcode[5:2] == 6'b0001 && opcode[0] && !cmp_c[1] && lt) ? 3'b100 :    //  signed  && vs2 <  vs1
        (opcode[5:2] == 6'b0001 && opcode[0] && !cmp_c[2] && !lt) ? 3'b010 :   //  signed  && vs2 >= vs1
        3'b001;

    wire [63:0] signed_vs1_sub =
        (vsew == 3'b000) ? $signed(~(vs1_in[op_type == VV ? base_index : 0 +:  8]) + 1) :
        (vsew == 3'b001) ? $signed(~(vs1_in[op_type == VV ? base_index : 0 +: 16]) + 1) :
        (vsew == 3'b010) ? $signed(~(vs1_in[op_type == VV ? base_index : 0 +: 32]) + 1) :
        (vsew == 3'b011) ? $signed(~(vs1_in[op_type == VV ? base_index : 0 +: 64]) + 1) : 0;
    
    wire [63:0] signed_vs2_rsub =
        (vsew == 3'b000) ? $signed(~(vs2_in[base_index +:  8]) + 1) :
        (vsew == 3'b001) ? $signed(~(vs2_in[base_index +: 16]) + 1) :
        (vsew == 3'b010) ? $signed(~(vs2_in[base_index +: 32]) + 1) :
        (vsew == 3'b011) ? $signed(~(vs2_in[base_index +: 64]) + 1) : 0;

    wire [SHIFTED_LANE_WIDTH-1:0] vs1 =
        opcode == 6'b000010 ? signed_vs1_sub[(in_reg_offset << LANE_WIDTH) +: SHIFTED_LANE_WIDTH]
        : vs1_in[op_type == VV ? index_val : (in_reg_offset << LANE_WIDTH) +: SHIFTED_LANE_WIDTH];
    wire [SHIFTED_LANE_WIDTH-1:0] vs2 =
        opcode == 6'b000011 ? signed_vs2_rsub[(in_reg_offset << LANE_WIDTH) +: SHIFTED_LANE_WIDTH]
        : vs2_in[index_val +: SHIFTED_LANE_WIDTH];

    // Reverse the iteration order for comparison operations
    wire [SHIFTED_LANE_WIDTH-1:0] vs1_cmp =
        vs1_in[(op_type == VV ? base_index : 0) + (((1 << (vsew+3-LANE_WIDTH)) - 1) << LANE_WIDTH) - (in_reg_offset << LANE_WIDTH) +: SHIFTED_LANE_WIDTH];
        
    wire [SHIFTED_LANE_WIDTH-1:0] vs2_cmp =
        vs2_in[base_index + (((1 << (vsew+3-LANE_WIDTH)) - 1) << LANE_WIDTH) - (in_reg_offset << LANE_WIDTH) +: SHIFTED_LANE_WIDTH];

    wire ltu = $unsigned(vs2_cmp) < $unsigned(vs1_cmp);
    wire lt = $signed(vs2_cmp) < $signed(vs1_cmp);

    // shifts
    // only the log2(sew) of vs1 are used to control the shift amount, with max sew of 64, it is 6 bits maximum
    wire [5:0] shift_amount_base = vs1_in[op_type == VV ? base_index : 0 +: 6];
    wire [5:0] shift_amount =
        (vsew == 3'b000) ? {3'b000, shift_amount_base[2:0]} :
        (vsew == 3'b001) ? {2'b00,  shift_amount_base[3:0]} :
        (vsew == 3'b010) ? {1'b0,   shift_amount_base[4:0]} :
        (vsew == 3'b011) ?          shift_amount_base[5:0]  : 0;

    wire shift_reg = in_reg_offset == 0 || (shift_rem >= `min((1 << (vsew+3)), SHIFTED_LANE_WIDTH));
    reg shift_reg_q;
    wire [5:0] shift_rem = (in_reg_offset == 0 ? shift_amount : (shift_rem >= `min((1 << (vsew+3)), SHIFTED_LANE_WIDTH) ? (shift_rem - `min((1 << (vsew+3)), SHIFTED_LANE_WIDTH)) : shift_rem));
                    
    reg [9:0] shift_index;
    
    
    always @* begin
        if (!resetn) begin
            temp_vreg = {65{1'b0}};
            shift_index = 0;
        end else if (run) begin
            temp_vreg = 0;
            case (opcode)
                // vand
                6'b001001: temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vs2 & vs1;
                // vor
                6'b001010: temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vs2 | vs1;
                // vxor
                6'b001011: temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vs2 ^ vs1;
                // vadd
                6'b000000: temp_vreg[0 +: ADD_SHIFTED_LANE_WIDTH] = vs2 + vs1 + cout_q;
                // vsub
                6'b000010: temp_vreg[0 +: ADD_SHIFTED_LANE_WIDTH] = vs2 + vs1 + cout_q;
                // vrsub
                6'b000011: temp_vreg[0 +: ADD_SHIFTED_LANE_WIDTH] = vs2 + vs1 + cout_q;
                // vminu
                6'b000100: begin
                    if ((ltu && !cmp_c[1]) || cmp_c[2]) begin
                        temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vs2_cmp;
                    end else if ((!ltu && !cmp_c[2]) || cmp_c[1]) begin
                        temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vs1_cmp;
                    end
                end
                // vmin
                6'b000101: begin
                    if ((lt && !cmp_c[1]) || cmp_c[2]) begin
                        temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vs2_cmp;
                    end else if ((!lt && !cmp_c[2]) || cmp_c[1]) begin
                        temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vs1_cmp;
                    end
                end
                // vmaxu
                6'b000110: begin
                    if ((ltu && !cmp_c[1]) || cmp_c[2]) begin
                        temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vs1_cmp;
                    end else if ((!ltu && !cmp_c[2]) || cmp_c[1]) begin
                        temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vs2_cmp;
                    end
                end
                // vmax
                6'b000111: begin
                    if ((lt && !cmp_c[1]) || cmp_c[2]) begin
                        temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vs1_cmp;
                    end else if ((!lt && !cmp_c[2]) || cmp_c[1]) begin
                        temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vs2_cmp;
                    end
                end
                // vsll
                6'b100101: begin
                    if (shift_rem >= SHIFTED_LANE_WIDTH) begin
                        // all zeros
                        temp_vreg[0 +: SHIFTED_LANE_WIDTH] = 0;
                    end else if (shift_reg_q) begin
                        // part zeros, part vs2
                        temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vs2_in[base_index +: SHIFTED_LANE_WIDTH] << shift_rem;
                        shift_index = base_index + shift_rem;
                    end else begin
                        // only vs2
                        temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vs2_in[shift_index +: SHIFTED_LANE_WIDTH];
                        shift_index = shift_index + `min((1 << (vsew+3)), SHIFTED_LANE_WIDTH);
                    end
                end
                // default
                default: temp_vreg = 0;
            endcase
        end else begin
            temp_vreg = 0;
        end
    end

    always @(posedge clk) begin
        cout_q <= cout;
        shift_reg_q <= shift_reg;
    end
endmodule
