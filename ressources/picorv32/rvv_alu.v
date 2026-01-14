`define min(a,b) ((a) < (b) ? (a) : (b))
`define max(a,b) ((a) > (b) ? (a) : (b))

module rvv_alu #(
	parameter [16:0] VLEN = 17'd 128,
    parameter [2:0] LANE_WIDTH = 3'b011, // 2^LANE_WIDTH bits per lane (8,16,32,64)
    // LANE_WIDTH * 2^nb_lanes must be less than or equal to VLEN
    parameter integer LANE_I = 0
) (
    input               clk, resetn,
    // TODO : taille port
    input      [4:0]    nb_lanes, // 2^nb_lanes lanes used for arith / logic operations
    input      [5:0]    opcode,
    input               instr_mask, // 1 if opcode is for mask insn (OPMV*) or int insn (OPIV*)
    input               run,
    input      [4:0]    vs1_index, // vs1 index, used for instruction decoding
    input      [VLEN-1:0] vs1_in,
    input      [VLEN-1:0] vs2_in,
    input      [2:0]    vsew,
    input      [2:0]    op_type, // 001 : vv | 010 : VX | 100 : VI

    input      [16:0]    byte_i,
    input      [3:0]    in_reg_offset,

    output [63:0] vd,
    output mask_cout,
    output [16:0]  out_index,
    output        instr_valid,
    output        instr_signed
);
    localparam integer VLEN_SIZE = $clog2(VLEN);
    localparam [SHIFTED_LANE_WIDTH-1:0] MASK_VLEN_SIZE = VLEN-1;
    localparam [6:0] SHIFTED_LANE_WIDTH = 1 << LANE_WIDTH;
    localparam integer SHIFTED_LANE_WIDTH_M8 = SHIFTED_LANE_WIDTH - 8;
    localparam integer SHIFTED_LANE_WIDTH_M16 = `max(0, $signed(SHIFTED_LANE_WIDTH - 16));
    localparam integer SHIFTED_LANE_WIDTH_M32 = `max(0, $signed(SHIFTED_LANE_WIDTH - 32));
    localparam integer SHIFTED_LANE_WIDTH_M64 = `max(0, $signed(SHIFTED_LANE_WIDTH - 64));
    localparam [SHIFTED_LANE_WIDTH-1:0] MASK8 = {{SHIFTED_LANE_WIDTH_M8{1'b0}}, 8'hff};
    localparam [SHIFTED_LANE_WIDTH-1:0] MASK16 = {{SHIFTED_LANE_WIDTH_M16{1'b0}}, 16'hffff};
    localparam [SHIFTED_LANE_WIDTH-1:0] MASK32 = {{SHIFTED_LANE_WIDTH_M32{1'b0}}, 32'hffff_ffff};
    localparam [SHIFTED_LANE_WIDTH-1:0] MASK64 = {{SHIFTED_LANE_WIDTH_M64{1'b0}}, 64'hffff_ffff_ffff_ffff};
    localparam [6:0] ADD_SHIFTED_LANE_WIDTH = SHIFTED_LANE_WIDTH + 1;
	localparam [2:0] VV = 3'b001;
	localparam [2:0] VX = 3'b010;
	localparam [2:0] VI = 3'b100;

    assign instr_valid = instr_mask ? mask_instr_valid : arith_instr_valid;
    assign instr_signed = !(!instr_mask && ((opcode == 6'b000100) | (opcode == 6'b000110) | (opcode == 6'b100101) | (opcode == 6'b101000) | (opcode == 6'b101001) | (opcode == 6'b011010) | (opcode == 6'b011100)));

    wire arith_instr_vmset = (opcode == 6'b011000) | (opcode == 6'b011001) | (opcode == 6'b011010) | (opcode == 6'b011011) | (opcode == 6'b011100) | (opcode == 6'b011101);

    wire arith_instr_valid =
        (opcode == 6'b001001) | (opcode == 6'b001010) | (opcode == 6'b001011) |
        (opcode == 6'b000000) | (opcode == 6'b000010) | (opcode == 6'b000011) |
        (opcode == 6'b000100) | (opcode == 6'b000101) | (opcode == 6'b000110) |
        (opcode == 6'b000111) | (opcode == 6'b100101) | (opcode == 6'b101000) |
        (opcode == 6'b101001) | arith_instr_vmset;

    wire mask_instr_valid =
        (opcode == 6'b011001) | (opcode == 6'b011101) | (opcode == 6'b011000) |
        (opcode == 6'b011011) | (opcode == 6'b011010) | (opcode == 6'b011110) |
        (opcode == 6'b011100) | (opcode == 6'b011111) | (opcode == 6'b010000 && vs1_index == 5'b10000) |
        (opcode == 6'b010000 && vs1_index == 5'b10001) | (opcode == 6'b010100 && vs1_index == 5'b00001) |
        (opcode == 6'b010100 && vs1_index == 5'b00011) | (opcode == 6'b010100 && vs1_index == 5'b00010) |
        (opcode == 6'b010100 && vs1_index == 5'b10000) | (opcode == 6'b010100 && vs1_index == 5'b10001);

    assign vd = temp_vreg[0 +: 64];
    assign mask_cout = (vs1_index == 5'b00010) ? |temp_vreg[0 +: SHIFTED_LANE_WIDTH] :
                       temp_vreg[1 << (`min(vsew+3, LANE_WIDTH)) - 1] & !(vs2[`min(vsew+3, LANE_WIDTH) - 1]);
    reg [64:0] temp_vreg; // 64 + 1 for carry out
    
    wire [16:0] base_index = ((LANE_I + byte_i) << (vsew + 3));
    assign out_index = (arith_instr_vmset) ? (LANE_I + byte_i) : index;
    wire [16:0] index =
	(opcode == 6'b010100 && vs1_index[4:1] == 4'b1000) ? base_index[0 +: VLEN_SIZE] :
	(opcode[5:2] == 4'b0001) || (opcode[5:1] == 5'b10100) /* || (arith_instr_vmset && (opcode == 6'b011010 || opcode == 6'b011011 || opcode == 6'b011100)) */ ? base_index + (((1 << `max($signed(vsew+3-LANE_WIDTH), $signed(0))) - 1) << LANE_WIDTH) - (in_reg_offset << LANE_WIDTH) : // min / max / right shift : reversed index
	// (opcode[5:2] == 4'b0001) || (opcode[5:1] == 5'b10100) ? base_index + (((1 << (vsew+3-LANE_WIDTH)) - 1) << LANE_WIDTH) - (in_reg_offset << LANE_WIDTH) : // min / max / right shift : reversed index
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
        ((opcode[5:2] == 4'b0001 && !opcode[0]) && !cmp_c[1] && ltu) ? 3'b100 :  // unsigned && vs2 <  vs1
        ((opcode[5:2] == 4'b0001 && !opcode[0]) && !cmp_c[2] && !ltu) ? 3'b010 : // unsigned && vs2 >= vs1
        ((opcode[5:2] == 4'b0001 && opcode[0]) && !cmp_c[1] && lt) ? 3'b100 :    //  signed  && vs2 <  vs1
        ((opcode[5:2] == 4'b0001 && opcode[0]) && !cmp_c[2] && !lt) ? 3'b010 :   //  signed  && vs2 >= vs1
        3'b001;
    
    reg [2:0] cmp_c_q;

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
        : vs1_in[op_type == VV ? index[VLEN_SIZE-1:0] : (in_reg_offset << LANE_WIDTH) +: SHIFTED_LANE_WIDTH];
    wire [SHIFTED_LANE_WIDTH-1:0] vs2 =
        opcode == 6'b000011 ? signed_vs2_rsub[(in_reg_offset << LANE_WIDTH) +: SHIFTED_LANE_WIDTH]
        : vs2_in[index[VLEN_SIZE-1:0] +: SHIFTED_LANE_WIDTH];

    wire [SHIFTED_LANE_WIDTH-1:0] vs1_clean =
        (vsew == 3'b000) ? {{SHIFTED_LANE_WIDTH_M8{(opcode == 6'b101001) && vs1[7]}}, vs1[0 +: 8]} :
        (vsew == 3'b001) ? {{SHIFTED_LANE_WIDTH_M16{(opcode == 6'b101001) && vs1[15]}}, vs1[0 +: 16]} :
        (vsew == 3'b010) ? {{SHIFTED_LANE_WIDTH_M32{(opcode == 6'b101001) && vs1[31]}}, vs1[0 +: 32]} :
        vs1[0 +: 64];
    
    wire [SHIFTED_LANE_WIDTH-1:0] vs2_clean =
        (vsew == 3'b000) ? {{SHIFTED_LANE_WIDTH_M8{(opcode == 6'b101001) && vs2[7]}}, vs2[0 +: 8]} :
        (vsew == 3'b001) ? {{SHIFTED_LANE_WIDTH_M16{(opcode == 6'b101001) && vs2[15]}}, vs2[0 +: 16]} :
        (vsew == 3'b010) ? {{SHIFTED_LANE_WIDTH_M32{(opcode == 6'b101001) && vs2[31]}}, vs2[0 +: 32]} :
        vs2[0 +: 64];

    // Reverse the iteration order for comparison operations
    wire [SHIFTED_LANE_WIDTH-1:0] vs1_cmp_tmp =
        vs1_in[((op_type == VV ? base_index : 0) + (((1 << `max($signed(vsew+3-LANE_WIDTH), $signed(0))) - 1) << LANE_WIDTH) - (in_reg_offset << LANE_WIDTH)) & MASK_VLEN_SIZE +: SHIFTED_LANE_WIDTH];

    wire [SHIFTED_LANE_WIDTH-1:0] vs1_cmp =
        (vsew == 3'b000) ? {{SHIFTED_LANE_WIDTH_M8{vs1_cmp_tmp[7]}}, vs1_cmp_tmp[0 +: 8]} :
        (vsew == 3'b001) ? {{SHIFTED_LANE_WIDTH_M16{vs1_cmp_tmp[15]}}, vs1_cmp_tmp[0 +: 16]} :
        (vsew == 3'b010) ? {{SHIFTED_LANE_WIDTH_M32{vs1_cmp_tmp[31]}}, vs1_cmp_tmp[0 +: 32]} :
        vs1_cmp_tmp[0 +: 64];

    wire [SHIFTED_LANE_WIDTH-1:0] vs2_cmp_tmp =
        vs2_in[(base_index + (((1 << `max($signed(vsew+3-LANE_WIDTH), $signed(0))) - 1) << LANE_WIDTH) - (in_reg_offset << LANE_WIDTH)) & MASK_VLEN_SIZE +: SHIFTED_LANE_WIDTH];

    wire [SHIFTED_LANE_WIDTH-1:0] vs2_cmp =
        (vsew == 3'b000) ? {{SHIFTED_LANE_WIDTH_M8{vs2_cmp_tmp[7]}}, vs2_cmp_tmp[0 +: 8]} :
        (vsew == 3'b001) ? {{SHIFTED_LANE_WIDTH_M16{vs2_cmp_tmp[15]}}, vs2_cmp_tmp[0 +: 16]} :
        (vsew == 3'b010) ? {{SHIFTED_LANE_WIDTH_M32{vs2_cmp_tmp[31]}}, vs2_cmp_tmp[0 +: 32]} :
        vs2_cmp_tmp[0 +: 64];
        
    wire eq = vs2_cmp == vs1_cmp;
    wire ltu = $unsigned(vs2_cmp) < $unsigned(vs1_cmp);
    wire lt = $signed(vs2_cmp) < $signed(vs1_cmp);

    wire [31:0] tmp = (base_index + (((1 << `max($signed(vsew+3-LANE_WIDTH), $signed(0))) - 1) << LANE_WIDTH) + SHIFTED_LANE_WIDTH - 1);
    wire vs2_neg = vs2_in[(base_index + (((1 << `max($signed(vsew+3-LANE_WIDTH), $signed(0))) - 1) << LANE_WIDTH) + SHIFTED_LANE_WIDTH - 1) & MASK_VLEN_SIZE];

    // shifts
    // only the log2(sew) of vs1 are used to control the shift amount, with max sew of 64, it is 6 bits maximum
    wire [5:0] shift_amount_base = vs1_in[op_type == VV ? base_index : 0 +: 6];
    wire [5:0] shift_amount =
        (vsew == 3'b000) ? {3'b000, shift_amount_base[2:0]} :
        (vsew == 3'b001) ? {2'b00,  shift_amount_base[3:0]} :
        (vsew == 3'b010) ? {1'b0,   shift_amount_base[4:0]} :
        (vsew == 3'b011) ?          shift_amount_base[5:0]  : 0;

    wire shift_reg = in_reg_offset == 0 ||
        ((opcode == 6'b100101) && shift_rem >= `min((1 << (vsew+3)), SHIFTED_LANE_WIDTH)) || // one way for left shifts
        ((opcode[5:1] == 5'b10100) && shift_rem <= `min((1 << (vsew+3)), SHIFTED_LANE_WIDTH)); // the other way for right shifts
    reg shift_reg_q;
    reg [5:0] shift_rem_base;
    wire [5:0] shift_rem = (in_reg_offset == 0) ? shift_amount : shift_rem_base;
    reg [5:0] shift_rem_q;
    reg [16:0] shift_index_base;
    // wire [16:0] shift_index = (in_reg_offset == 0) ? base_index/*  + (1 << `max(0, (vsew+3) - SHIFTED_LANE_WIDTH)) */ : shift_index_base;
    wire [16:0] shift_index = (in_reg_offset == 0) ? index/*  + (1 << `max(0, (vsew+3) - SHIFTED_LANE_WIDTH)) */ : shift_index_base;
    wire [SHIFTED_LANE_WIDTH-1:0] vs2_shift_tmp = vs2_in[shift_index +: SHIFTED_LANE_WIDTH];
    wire [SHIFTED_LANE_WIDTH-1:0] vs2_shift =
        (vsew == 3'b000) ? {{SHIFTED_LANE_WIDTH_M8{(opcode == 6'b101001) && vs2_shift_tmp[7]}}, vs2_shift_tmp[0 +: 8]} :
        (vsew == 3'b001) ? {{SHIFTED_LANE_WIDTH_M16{(opcode == 6'b101001) && vs2_shift_tmp[15]}}, vs2_shift_tmp[0 +: 16]} :
        (vsew == 3'b010) ? {{SHIFTED_LANE_WIDTH_M32{(opcode == 6'b101001) && vs2_shift_tmp[31]}}, vs2_shift_tmp[0 +: 32]} :
        vs2_shift_tmp[0 +: 64];

    reg mask_acc;

    reg vmset_q;
    wire vmset_acc = in_reg_offset == 0 ? opcode == 6'b011000 : vmset_q;

    always @* begin
        if (!resetn) begin
            temp_vreg = {65{1'b0}};
        end else if (run) begin
            temp_vreg = 0;
            if (instr_mask)
                case (opcode)
                    // vmand
                    6'b011001: temp_vreg[0 +: SHIFTED_LANE_WIDTH] = (vs2 & vs1);
                    // vmnand
                    6'b011101: temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vs2 ~& vs1;
                    // vmandn
                    6'b011000: temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vs2 & ~vs1;
                    // vmxor
                    6'b011011: temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vs2 ^ vs1;
                    // vmor
                    6'b011010: temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vs2 | vs1;
                    // vmnor
                    6'b011110: temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vs2 ~| vs1;
                    // vmorn
                    6'b011100: temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vs2 | ~vs1;
                    // vmxnor
                    6'b011111: temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vs2 ~^ vs1;
                    // vcpop | vfirst
                    6'b010000:
                        if (vs1_index == 5'b10000) // vcpop
                            case (`min(vsew+3, LANE_WIDTH))
                                // 8b
                                3'b011 : temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vcpop_lut(vs2);
                                // 16b
                                3'b100 : temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vcpop_lut(vs2[15:8]) + vcpop_lut(vs2[7:0]);
                                // 32b
                                3'b101 : temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vcpop_lut(vs2[31:24]) + vcpop_lut(vs2[23:16]) + vcpop_lut(vs2[15:8]) + vcpop_lut(vs2[7:0]);
                                // 64b
                                3'b110 : temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vcpop_lut(vs2[64:56]) + vcpop_lut(vs2[55:48]) + vcpop_lut(vs2[47:40]) + vcpop_lut(vs2[39:32]) + vcpop_lut(vs2[31:24]) + vcpop_lut(vs2[23:16]) + vcpop_lut(vs2[15:8]) + vcpop_lut(vs2[7:0]);
                            endcase
                        else if (vs1_index == 5'b10001) // vfirst
                            case (`min(vsew+3, LANE_WIDTH))
                                // 8b
                                3'b011 : temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vfirst8(vs2) + index;
                                // 16b
                                3'b100 : temp_vreg[0 +: SHIFTED_LANE_WIDTH] = (&(vfirst8(vs2[7:0])) ? vfirst8(vs2[15:8]) + 8 : vfirst8(vs2[7:0])) + index;
                                // 32b
                                3'b101 : temp_vreg[0 +: SHIFTED_LANE_WIDTH] = (&(vfirst8(vs2[7:0])) ?
                                                                                &(vfirst8(vs2[15:8])) ?
                                                                                    &(vfirst8(vs2[23:16])) ?
                                                                                        vfirst8(vs2[31:24]) + 24 :
                                                                                    vfirst8(vs2[23:16]) + 16 :
                                                                                vfirst8(vs2[15:8]) + 8 :
                                                                              vfirst8(vs2[7:0])) + index;
                                // 64b
                                3'b110 : temp_vreg[0 +: SHIFTED_LANE_WIDTH] = (&(vfirst8(vs2[7:0])) ?
                                                                                &(vfirst8(vs2[15:8])) ?
                                                                                    &(vfirst8(vs2[23:16])) ?
                                                                                        &(vfirst8(vs2[31:24])) ?
                                                                                            &(vfirst8(vs2[39:32])) ?
                                                                                                &(vfirst8(vs2[47:40])) ?
                                                                                                    &(vfirst8(vs2[55:48])) ?
                                                                                                        vfirst8(vs2[63:56]) + 56 :
                                                                                                    vfirst8(vs2[55:48]) + 48 :
                                                                                                vfirst8(vs2[47:40]) + 40 :
                                                                                            vfirst8(vs2[39:32]) + 32 :
                                                                                        vfirst8(vs2[31:24]) + 24 :
                                                                                    vfirst8(vs2[23:16]) + 16 :
                                                                                vfirst8(vs2[15:8]) + 8 :
                                                                              vfirst8(vs2[7:0])) + index;
                            endcase
                    6'b010100:
                        if (vs1_index == 5'b00001) // vmsbf
                            case (`min(vsew+3, LANE_WIDTH))
                                // 8b
                                3'b011 : temp_vreg[0 +: SHIFTED_LANE_WIDTH] = mask_acc ? 8'hFF >> (8 - (&vfirst8(vs2) ? 8 : vfirst8(vs2))) : 0;
                                // 16b
                                3'b100 : temp_vreg[0 +: SHIFTED_LANE_WIDTH] = mask_acc ? (&(vfirst8(vs2[7:0])) ? 16'hFFFF >> (8 - (&vfirst8(vs2[15:8]) ? 8 : vfirst8(vs2[15:8]))) : 16'hFFFF >> (16 - vfirst8(vs2[7:0]))) : 0;
                                // 32b
                                3'b101 : temp_vreg[0 +: SHIFTED_LANE_WIDTH] = mask_acc ? (&(vfirst8(vs2[7:0])) ?
                                                                                &(vfirst8(vs2[15:8])) ?
                                                                                    &(vfirst8(vs2[23:16])) ?
                                                                                        32'hFFFF_FFFF >> (8 - (&vfirst8(vs2[31:24]) ? 8 : vfirst8(vs2[31:24]))) :
                                                                                    32'hFFFF_FFFF >> (16 - vfirst8(vs2[23:16])) :
                                                                                32'hFFFF_FFFF >> (24 - vfirst8(vs2[15:8])) :
                                                                              32'hFFFF_FFFF >> (32 - vfirst8(vs2[7:0]))) : 0;
                                // 64b
                                3'b110 : temp_vreg[0 +: SHIFTED_LANE_WIDTH] = mask_acc ? (&(vfirst8(vs2[7:0])) ?
                                                                                &(vfirst8(vs2[15:8])) ?
                                                                                    &(vfirst8(vs2[23:16])) ?
                                                                                        &(vfirst8(vs2[31:24])) ?
                                                                                            &(vfirst8(vs2[39:32])) ?
                                                                                                &(vfirst8(vs2[47:40])) ?
                                                                                                    &(vfirst8(vs2[55:48])) ?
                                                                                                        64'hFFFF_FFFF_FFFF_FFFF >> (8 - (&vfirst8(vs2[63:56]) ? 8 : vfirst8(vs2[63:56]))) :
                                                                                                    64'hFFFF_FFFF_FFFF_FFFF >> (16 - vfirst8(vs2[55:48])) :
                                                                                                64'hFFFF_FFFF_FFFF_FFFF >> (24 - vfirst8(vs2[47:40])) :
                                                                                            64'hFFFF_FFFF_FFFF_FFFF >> (32 - vfirst8(vs2[39:32])) :
                                                                                        64'hFFFF_FFFF_FFFF_FFFF >> (40 - vfirst8(vs2[31:24])) :
                                                                                    64'hFFFF_FFFF_FFFF_FFFF >> (48 - vfirst8(vs2[23:16])) :
                                                                                64'hFFFF_FFFF_FFFF_FFFF >> (56 - vfirst8(vs2[15:8])) :
                                                                              64'hFFFF_FFFF_FFFF_FFFF >> (64 - vfirst8(vs2[7:0]))) : 0;
                            endcase
                        else if (vs1_index == 5'b00011) // vmsif
                            case (`min(vsew+3, LANE_WIDTH))
                                // 8b
                                3'b011 : temp_vreg[0 +: SHIFTED_LANE_WIDTH] = mask_acc ? 8'hFF >> (8 - (&vfirst8(vs2) ? 8 : vfirst8(vs2)+1)) : 0;
                                // 16b
                                3'b100 : temp_vreg[0 +: SHIFTED_LANE_WIDTH] = mask_acc ? (&(vfirst8(vs2[7:0])) ? 16'hFFFF >> (8 - (&vfirst8(vs2[15:8]) ? 8 : vfirst8(vs2[15:8])+1)) : 16'hFFFF >> (16 - vfirst8(vs2[7:0])+1)) : 0;
                                // 32b
                                3'b101 : temp_vreg[0 +: SHIFTED_LANE_WIDTH] = mask_acc ? (&(vfirst8(vs2[7:0])) ?
                                                                                &(vfirst8(vs2[15:8])) ?
                                                                                    &(vfirst8(vs2[23:16])) ?
                                                                                        32'hFFFF_FFFF >> (8 - (&vfirst8(vs2[31:24]) ? 8 : vfirst8(vs2[31:24])+1)) :
                                                                                    32'hFFFF_FFFF >> (16 - vfirst8(vs2[23:16])+1) :
                                                                                32'hFFFF_FFFF >> (24 - vfirst8(vs2[15:8])+1) :
                                                                              32'hFFFF_FFFF >> (32 - vfirst8(vs2[7:0])+1)) : 0;
                                // 64b
                                3'b110 : temp_vreg[0 +: SHIFTED_LANE_WIDTH] = mask_acc ? (&(vfirst8(vs2[7:0])) ?
                                                                                &(vfirst8(vs2[15:8])) ?
                                                                                    &(vfirst8(vs2[23:16])) ?
                                                                                        &(vfirst8(vs2[31:24])) ?
                                                                                            &(vfirst8(vs2[39:32])) ?
                                                                                                &(vfirst8(vs2[47:40])) ?
                                                                                                    &(vfirst8(vs2[55:48])) ?
                                                                                                        64'hFFFF_FFFF_FFFF_FFFF >> (8 - (&vfirst8(vs2[63:56]) ? 8 : vfirst8(vs2[63:56])+1)) :
                                                                                                    64'hFFFF_FFFF_FFFF_FFFF >> (16 - vfirst8(vs2[55:48])+1) :
                                                                                                64'hFFFF_FFFF_FFFF_FFFF >> (24 - vfirst8(vs2[47:40])+1) :
                                                                                            64'hFFFF_FFFF_FFFF_FFFF >> (32 - vfirst8(vs2[39:32])+1) :
                                                                                        64'hFFFF_FFFF_FFFF_FFFF >> (40 - vfirst8(vs2[31:24])+1) :
                                                                                    64'hFFFF_FFFF_FFFF_FFFF >> (48 - vfirst8(vs2[23:16])+1) :
                                                                                64'hFFFF_FFFF_FFFF_FFFF >> (56 - vfirst8(vs2[15:8])+1) :
                                                                              64'hFFFF_FFFF_FFFF_FFFF >> (64 - vfirst8(vs2[7:0])+1)) : 0;
                            endcase
                        else if (vs1_index == 5'b00010) // vmsof
                            case (`min(vsew+3, LANE_WIDTH))
                                // 8b
                                3'b011 : temp_vreg[0 +: SHIFTED_LANE_WIDTH] = mask_acc ? (&vfirst8(vs2) ? 0 : 1 << vfirst8(vs2)) : 0;
                                // 16b
                                3'b100 : temp_vreg[0 +: SHIFTED_LANE_WIDTH] = mask_acc ? (&(vfirst8(vs2[7:0])) ? (&vfirst8(vs2[15:8]) ? 0 : 1 << vfirst8(vs2[15:8])) : (1 << vfirst8(vs2[7:0]))) : 0;
                                // 32b
                                3'b101 : temp_vreg[0 +: SHIFTED_LANE_WIDTH] = mask_acc ? (&(vfirst8(vs2[7:0])) ?
                                                                                &(vfirst8(vs2[15:8])) ?
                                                                                    &(vfirst8(vs2[23:16])) ?
                                                                                        (&vfirst8(vs2[31:24]) ? 0 : 1 << vfirst8(vs2[31:24])) :
                                                                                    (1 << vfirst8(vs2[23:16])) :
                                                                                (1 << vfirst8(vs2[15:8])) :
                                                                              (1 << vfirst8(vs2[7:0]))) : 0;
                                // 64b
                                3'b110 : temp_vreg[0 +: SHIFTED_LANE_WIDTH] = mask_acc ? (&(vfirst8(vs2[7:0])) ?
                                                                                &(vfirst8(vs2[15:8])) ?
                                                                                    &(vfirst8(vs2[23:16])) ?
                                                                                        &(vfirst8(vs2[31:24])) ?
                                                                                            &(vfirst8(vs2[39:32])) ?
                                                                                                &(vfirst8(vs2[47:40])) ?
                                                                                                    &(vfirst8(vs2[55:48])) ?
                                                                                                        (&vfirst8(vs2[63:56]) ? 0 : 1 << vfirst8(vs2[63:56])) :
                                                                                                    (1 << vfirst8(vs2[55:48])) :
                                                                                                (1 << vfirst8(vs2[47:40])) :
                                                                                            (1 << vfirst8(vs2[39:32])) :
                                                                                        (1 << vfirst8(vs2[31:24])) :
                                                                                    (1 << vfirst8(vs2[23:16])) :
                                                                                (1 << vfirst8(vs2[15:8])) :
                                                                              (1 << vfirst8(vs2[7:0]))) : 0;
                            endcase
                        else if (vs1_index[4:1] == 4'b1000) // viota | vid
                        begin
			    // $display("vs2_in[%d] = %d", (LANE_I + byte_i), vs2_in[LANE_I+byte_i]);
                            case (`min(vsew+3, LANE_WIDTH))
                                // 8b
                                3'b011 : temp_vreg[0 +: SHIFTED_LANE_WIDTH] = in_reg_offset == 0 ? vs2_in[(LANE_I + byte_i)] : 0;
                                // 16b
                                3'b100 : temp_vreg[0 +: SHIFTED_LANE_WIDTH] = in_reg_offset == 0 ? vs2_in[(LANE_I + byte_i)] : 0;
                                // 32b
                                3'b101 : temp_vreg[0 +: SHIFTED_LANE_WIDTH] = in_reg_offset == 0 ? vs2_in[(LANE_I + byte_i)] : 0;
                                // 64b
                                3'b110 : temp_vreg[0 +: SHIFTED_LANE_WIDTH] = in_reg_offset == 0 ? vs2_in[(LANE_I + byte_i)] : 0;
                            endcase
                        end
                    default: temp_vreg = 0;
                endcase
            else
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
                        /* if ((ltu && !cmp_c[1]) || cmp_c[2]) begin
                            temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vs2_cmp;
                        end else if ((!ltu && !cmp_c[2]) || cmp_c[1]) begin
                            temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vs1_cmp;
                        end */
                        temp_vreg[0 +: SHIFTED_LANE_WIDTH] = (cmp_c_q[2] | (ltu & cmp_c_q[0])) ? vs2_cmp : vs1_cmp;
                    end
                    // vmin
                    6'b000101: begin
                        temp_vreg[0 +: SHIFTED_LANE_WIDTH] = (cmp_c_q[2] | ((in_reg_offset == 0 ? lt : ltu) & cmp_c_q[0])) ? vs2_cmp : vs1_cmp;
                        /* if ((lt && !cmp_c[1]) || cmp_c[2]) begin
                            temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vs2_cmp;
                        end else if ((!lt && !cmp_c[2]) || cmp_c[1]) begin
                            temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vs1_cmp;
                        end */
                    end
                    // vmaxu
                    6'b000110: begin
                        temp_vreg[0 +: SHIFTED_LANE_WIDTH] = (cmp_c_q[2] | (ltu & cmp_c_q[0])) ? vs1_cmp : vs2_cmp;
                        /* if ((ltu && !cmp_c[1]) || cmp_c[2]) begin
                            temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vs1_cmp;
                        end else if ((!ltu && !cmp_c[2]) || cmp_c[1]) begin
                            temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vs2_cmp;
                        end */
                    end
                    // vmax
                    6'b000111: begin
                        temp_vreg[0 +: SHIFTED_LANE_WIDTH] = (cmp_c_q[2] | ((in_reg_offset == 0 ? lt : ltu) & cmp_c_q[0])) ? vs1_cmp : vs2_cmp;
                        /* if ((lt && !cmp_c[1]) || cmp_c[2]) begin
                            temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vs1_cmp;
                        end else if ((!lt && !cmp_c[2]) || cmp_c[1]) begin
                            temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vs2_cmp;
                        end */
                    end
                    // vsll
                    6'b100101: begin
                        if (shift_rem > SHIFTED_LANE_WIDTH) // all zeros
                            temp_vreg[0 +: SHIFTED_LANE_WIDTH] = 0;
                        else if (in_reg_offset == 0 || (shift_rem < SHIFTED_LANE_WIDTH && shift_rem_q >= SHIFTED_LANE_WIDTH)) // part zeros, part vs2
                            temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vs2_in[base_index +: SHIFTED_LANE_WIDTH] << shift_rem;
                        else // only vs2
                            temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vs2_in[shift_index - shift_rem +: SHIFTED_LANE_WIDTH];
                    end
                    // vsrl
                    6'b101000: begin
                        /* if (shift_rem > SHIFTED_LANE_WIDTH) // only zeros
                            temp_vreg[0 +: SHIFTED_LANE_WIDTH] = 0;
                        else if (shift_reg && shift_rem != 0) // part zeros, part vs2
                            if (in_reg_offset == 0 || (shift_rem < SHIFTED_LANE_WIDTH && shift_rem_q >= SHIFTED_LANE_WIDTH))
                                temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vs2_in[shift_index +: SHIFTED_LANE_WIDTH] >> shift_rem;
                            else
                                temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vs2_in[shift_index + shift_rem +: SHIFTED_LANE_WIDTH];
                        else // only vs2
                            temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vs2_in[shift_index +: SHIFTED_LANE_WIDTH]; */
                        if (shift_rem >= SHIFTED_LANE_WIDTH) // only zeros
                            temp_vreg[0 +: SHIFTED_LANE_WIDTH] = 0;
                        else if (shift_reg && shift_rem != 0) // part zeros, part vs2
                            if (in_reg_offset == 0 || (shift_rem < SHIFTED_LANE_WIDTH && shift_rem_q >= SHIFTED_LANE_WIDTH))
                                temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vs2_shift >> shift_rem;
                            else
                                temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vs2_in[shift_index + shift_rem +: SHIFTED_LANE_WIDTH];
                        else // only vs2
                            temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vs2_in[shift_index +: SHIFTED_LANE_WIDTH];
                    end
                    // vsra
                    6'b101001: begin
                        /* if (shift_rem > SHIFTED_LANE_WIDTH) // only sign bit
                            temp_vreg[0 +: SHIFTED_LANE_WIDTH] = {SHIFTED_LANE_WIDTH{vs2_in[base_index + (1 << (vsew+3)) - 1]}};
                        else if (shift_reg && shift_rem != 0) // part zeros, part vs2
                            if (in_reg_offset == 0 || (shift_rem < SHIFTED_LANE_WIDTH && shift_rem_q >= SHIFTED_LANE_WIDTH))
                                temp_vreg[0 +: SHIFTED_LANE_WIDTH] = $signed(vs2_in[shift_index +: SHIFTED_LANE_WIDTH]) >>> shift_rem;
                            else
                                temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vs2_in[shift_index + shift_rem +: SHIFTED_LANE_WIDTH];
                        else // only vs2
                            temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vs2_in[shift_index +: SHIFTED_LANE_WIDTH]; */
                        if (shift_rem >= SHIFTED_LANE_WIDTH) // only sign bit
                            temp_vreg[0 +: SHIFTED_LANE_WIDTH] = {SHIFTED_LANE_WIDTH{vs2_in[base_index + (1 << (vsew+3)) - 1]}};
                        else if (shift_reg && shift_rem != 0) // part zeros, part vs2
                            if (in_reg_offset == 0 || (shift_rem < SHIFTED_LANE_WIDTH && shift_rem_q >= SHIFTED_LANE_WIDTH))
                                temp_vreg[0 +: SHIFTED_LANE_WIDTH] = $signed(vs2_shift) >>> shift_rem;
                            else
                                temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vs2_in[shift_index + shift_rem +: SHIFTED_LANE_WIDTH];
                        else // only vs2
                            temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vs2_in[shift_index +: SHIFTED_LANE_WIDTH];
                    end
                    // vmseq
                    6'b011000: temp_vreg[0] = vmset_acc & vs1_clean == vs2_clean;
                    // vmsne
                    6'b011001: temp_vreg[0] = vmset_acc | vs1_clean != vs2_clean;
                    // vmsltu
                    6'b011010: temp_vreg[0] = cmp_c_q[2] | (ltu & cmp_c_q[0]);
                    // vmslt
                    6'b011011: temp_vreg[0] = cmp_c_q[2] | ((in_reg_offset == 0 ? lt : ltu) & cmp_c_q[0]);
                    // vmsleu
                    6'b011100: temp_vreg[0] = cmp_c_q[2] | ((ltu | eq) & cmp_c_q[0]);
                    // vmsle
                    6'b011101: temp_vreg[0] = cmp_c_q[2] | (((in_reg_offset == 0 ? lt : ltu) | eq) & cmp_c_q[0]);
                    // default
                    default: temp_vreg = 0;
                endcase
        end else begin
            temp_vreg = 0;
        end
    end

    always @(posedge clk) begin
        if (!resetn) begin
            cout_q <= 0;
            shift_reg_q <= 0;
            shift_index_base <= 0;
            shift_rem_base <= 0;
            shift_rem_q <= 0;
            mask_acc <= 1;
            vmset_q <= opcode == 6'b011000;
            cmp_c_q <= 3'b001;
        end else begin
            if (in_reg_offset == (vsew + 3 <= LANE_WIDTH ? 0 : (1 << (vsew+3-LANE_WIDTH)) - 1))
                cmp_c_q <= 3'b001;
            else if (run)
                case (cmp_c_q)
                    3'b001: cmp_c_q <= eq ? 3'b001 : (((instr_signed && in_reg_offset == 0 ? lt : ltu) & !(vs2_neg & in_reg_offset != 0)) ? 3'b100 : 3'b010);
                    default: cmp_c_q <= cmp_c_q;
                endcase
            
            mask_acc <= run ? mask_cout : 1;
            vmset_q <= temp_vreg[0];
            cout_q <= cout;
            shift_reg_q <= shift_reg;
            shift_rem_base <= (shift_rem >= `min((1 << (vsew+3)), SHIFTED_LANE_WIDTH) ? (shift_rem - `min((1 << (vsew+3)), SHIFTED_LANE_WIDTH)) : shift_rem);
            shift_rem_q <= shift_rem;

            // vsll
            if (opcode == 6'b100101 && !(shift_rem >= SHIFTED_LANE_WIDTH)) begin
                if (shift_reg)
                    shift_index_base <= base_index + shift_rem;
                else
                    shift_index_base <= shift_index_base + `min((1 << (vsew+3)), SHIFTED_LANE_WIDTH);
            end else if (opcode[5:1] == 5'b10100) begin // right shifts
                if (in_reg_offset == 0 && shift_rem < SHIFTED_LANE_WIDTH)
                    shift_index_base <= base_index + (1 << (vsew+3)) - (SHIFTED_LANE_WIDTH << 1);
                else if (in_reg_offset == 0 && shift_rem >= SHIFTED_LANE_WIDTH)
                    shift_index_base <= base_index + (1 << (vsew+3)) - SHIFTED_LANE_WIDTH;
                else if (shift_rem < SHIFTED_LANE_WIDTH)
                    shift_index_base <= shift_index_base - `min((1 << (vsew+3)), SHIFTED_LANE_WIDTH);
            end
        end
    end

    function automatic [7:0] vcpop_lut;
        input [7:0] addr;
        begin
            case (addr)
                8'h0: vcpop_lut = 8'h0;
                8'h1: vcpop_lut = 8'h1;
                8'h2: vcpop_lut = 8'h1;
                8'h3: vcpop_lut = 8'h2;
                8'h4: vcpop_lut = 8'h1;
                8'h5: vcpop_lut = 8'h2;
                8'h6: vcpop_lut = 8'h2;
                8'h7: vcpop_lut = 8'h3;
                8'h8: vcpop_lut = 8'h1;
                8'h9: vcpop_lut = 8'h2;
                8'ha: vcpop_lut = 8'h2;
                8'hb: vcpop_lut = 8'h3;
                8'hc: vcpop_lut = 8'h2;
                8'hd: vcpop_lut = 8'h3;
                8'he: vcpop_lut = 8'h3;
                8'hf: vcpop_lut = 8'h4;
                8'h10: vcpop_lut = 8'h1;
                8'h11: vcpop_lut = 8'h2;
                8'h12: vcpop_lut = 8'h2;
                8'h13: vcpop_lut = 8'h3;
                8'h14: vcpop_lut = 8'h2;
                8'h15: vcpop_lut = 8'h3;
                8'h16: vcpop_lut = 8'h3;
                8'h17: vcpop_lut = 8'h4;
                8'h18: vcpop_lut = 8'h2;
                8'h19: vcpop_lut = 8'h3;
                8'h1a: vcpop_lut = 8'h3;
                8'h1b: vcpop_lut = 8'h4;
                8'h1c: vcpop_lut = 8'h3;
                8'h1d: vcpop_lut = 8'h4;
                8'h1e: vcpop_lut = 8'h4;
                8'h1f: vcpop_lut = 8'h5;
                8'h20: vcpop_lut = 8'h1;
                8'h21: vcpop_lut = 8'h2;
                8'h22: vcpop_lut = 8'h2;
                8'h23: vcpop_lut = 8'h3;
                8'h24: vcpop_lut = 8'h2;
                8'h25: vcpop_lut = 8'h3;
                8'h26: vcpop_lut = 8'h3;
                8'h27: vcpop_lut = 8'h4;
                8'h28: vcpop_lut = 8'h2;
                8'h29: vcpop_lut = 8'h3;
                8'h2a: vcpop_lut = 8'h3;
                8'h2b: vcpop_lut = 8'h4;
                8'h2c: vcpop_lut = 8'h3;
                8'h2d: vcpop_lut = 8'h4;
                8'h2e: vcpop_lut = 8'h4;
                8'h2f: vcpop_lut = 8'h5;
                8'h30: vcpop_lut = 8'h2;
                8'h31: vcpop_lut = 8'h3;
                8'h32: vcpop_lut = 8'h3;
                8'h33: vcpop_lut = 8'h4;
                8'h34: vcpop_lut = 8'h3;
                8'h35: vcpop_lut = 8'h4;
                8'h36: vcpop_lut = 8'h4;
                8'h37: vcpop_lut = 8'h5;
                8'h38: vcpop_lut = 8'h3;
                8'h39: vcpop_lut = 8'h4;
                8'h3a: vcpop_lut = 8'h4;
                8'h3b: vcpop_lut = 8'h5;
                8'h3c: vcpop_lut = 8'h4;
                8'h3d: vcpop_lut = 8'h5;
                8'h3e: vcpop_lut = 8'h5;
                8'h3f: vcpop_lut = 8'h6;
                8'h40: vcpop_lut = 8'h1;
                8'h41: vcpop_lut = 8'h2;
                8'h42: vcpop_lut = 8'h2;
                8'h43: vcpop_lut = 8'h3;
                8'h44: vcpop_lut = 8'h2;
                8'h45: vcpop_lut = 8'h3;
                8'h46: vcpop_lut = 8'h3;
                8'h47: vcpop_lut = 8'h4;
                8'h48: vcpop_lut = 8'h2;
                8'h49: vcpop_lut = 8'h3;
                8'h4a: vcpop_lut = 8'h3;
                8'h4b: vcpop_lut = 8'h4;
                8'h4c: vcpop_lut = 8'h3;
                8'h4d: vcpop_lut = 8'h4;
                8'h4e: vcpop_lut = 8'h4;
                8'h4f: vcpop_lut = 8'h5;
                8'h50: vcpop_lut = 8'h2;
                8'h51: vcpop_lut = 8'h3;
                8'h52: vcpop_lut = 8'h3;
                8'h53: vcpop_lut = 8'h4;
                8'h54: vcpop_lut = 8'h3;
                8'h55: vcpop_lut = 8'h4;
                8'h56: vcpop_lut = 8'h4;
                8'h57: vcpop_lut = 8'h5;
                8'h58: vcpop_lut = 8'h3;
                8'h59: vcpop_lut = 8'h4;
                8'h5a: vcpop_lut = 8'h4;
                8'h5b: vcpop_lut = 8'h5;
                8'h5c: vcpop_lut = 8'h4;
                8'h5d: vcpop_lut = 8'h5;
                8'h5e: vcpop_lut = 8'h5;
                8'h5f: vcpop_lut = 8'h6;
                8'h60: vcpop_lut = 8'h2;
                8'h61: vcpop_lut = 8'h3;
                8'h62: vcpop_lut = 8'h3;
                8'h63: vcpop_lut = 8'h4;
                8'h64: vcpop_lut = 8'h3;
                8'h65: vcpop_lut = 8'h4;
                8'h66: vcpop_lut = 8'h4;
                8'h67: vcpop_lut = 8'h5;
                8'h68: vcpop_lut = 8'h3;
                8'h69: vcpop_lut = 8'h4;
                8'h6a: vcpop_lut = 8'h4;
                8'h6b: vcpop_lut = 8'h5;
                8'h6c: vcpop_lut = 8'h4;
                8'h6d: vcpop_lut = 8'h5;
                8'h6e: vcpop_lut = 8'h5;
                8'h6f: vcpop_lut = 8'h6;
                8'h70: vcpop_lut = 8'h3;
                8'h71: vcpop_lut = 8'h4;
                8'h72: vcpop_lut = 8'h4;
                8'h73: vcpop_lut = 8'h5;
                8'h74: vcpop_lut = 8'h4;
                8'h75: vcpop_lut = 8'h5;
                8'h76: vcpop_lut = 8'h5;
                8'h77: vcpop_lut = 8'h6;
                8'h78: vcpop_lut = 8'h4;
                8'h79: vcpop_lut = 8'h5;
                8'h7a: vcpop_lut = 8'h5;
                8'h7b: vcpop_lut = 8'h6;
                8'h7c: vcpop_lut = 8'h5;
                8'h7d: vcpop_lut = 8'h6;
                8'h7e: vcpop_lut = 8'h6;
                8'h7f: vcpop_lut = 8'h7;
                8'h80: vcpop_lut = 8'h1;
                8'h81: vcpop_lut = 8'h2;
                8'h82: vcpop_lut = 8'h2;
                8'h83: vcpop_lut = 8'h3;
                8'h84: vcpop_lut = 8'h2;
                8'h85: vcpop_lut = 8'h3;
                8'h86: vcpop_lut = 8'h3;
                8'h87: vcpop_lut = 8'h4;
                8'h88: vcpop_lut = 8'h2;
                8'h89: vcpop_lut = 8'h3;
                8'h8a: vcpop_lut = 8'h3;
                8'h8b: vcpop_lut = 8'h4;
                8'h8c: vcpop_lut = 8'h3;
                8'h8d: vcpop_lut = 8'h4;
                8'h8e: vcpop_lut = 8'h4;
                8'h8f: vcpop_lut = 8'h5;
                8'h90: vcpop_lut = 8'h2;
                8'h91: vcpop_lut = 8'h3;
                8'h92: vcpop_lut = 8'h3;
                8'h93: vcpop_lut = 8'h4;
                8'h94: vcpop_lut = 8'h3;
                8'h95: vcpop_lut = 8'h4;
                8'h96: vcpop_lut = 8'h4;
                8'h97: vcpop_lut = 8'h5;
                8'h98: vcpop_lut = 8'h3;
                8'h99: vcpop_lut = 8'h4;
                8'h9a: vcpop_lut = 8'h4;
                8'h9b: vcpop_lut = 8'h5;
                8'h9c: vcpop_lut = 8'h4;
                8'h9d: vcpop_lut = 8'h5;
                8'h9e: vcpop_lut = 8'h5;
                8'h9f: vcpop_lut = 8'h6;
                8'ha0: vcpop_lut = 8'h2;
                8'ha1: vcpop_lut = 8'h3;
                8'ha2: vcpop_lut = 8'h3;
                8'ha3: vcpop_lut = 8'h4;
                8'ha4: vcpop_lut = 8'h3;
                8'ha5: vcpop_lut = 8'h4;
                8'ha6: vcpop_lut = 8'h4;
                8'ha7: vcpop_lut = 8'h5;
                8'ha8: vcpop_lut = 8'h3;
                8'ha9: vcpop_lut = 8'h4;
                8'haa: vcpop_lut = 8'h4;
                8'hab: vcpop_lut = 8'h5;
                8'hac: vcpop_lut = 8'h4;
                8'had: vcpop_lut = 8'h5;
                8'hae: vcpop_lut = 8'h5;
                8'haf: vcpop_lut = 8'h6;
                8'hb0: vcpop_lut = 8'h3;
                8'hb1: vcpop_lut = 8'h4;
                8'hb2: vcpop_lut = 8'h4;
                8'hb3: vcpop_lut = 8'h5;
                8'hb4: vcpop_lut = 8'h4;
                8'hb5: vcpop_lut = 8'h5;
                8'hb6: vcpop_lut = 8'h5;
                8'hb7: vcpop_lut = 8'h6;
                8'hb8: vcpop_lut = 8'h4;
                8'hb9: vcpop_lut = 8'h5;
                8'hba: vcpop_lut = 8'h5;
                8'hbb: vcpop_lut = 8'h6;
                8'hbc: vcpop_lut = 8'h5;
                8'hbd: vcpop_lut = 8'h6;
                8'hbe: vcpop_lut = 8'h6;
                8'hbf: vcpop_lut = 8'h7;
                8'hc0: vcpop_lut = 8'h2;
                8'hc1: vcpop_lut = 8'h3;
                8'hc2: vcpop_lut = 8'h3;
                8'hc3: vcpop_lut = 8'h4;
                8'hc4: vcpop_lut = 8'h3;
                8'hc5: vcpop_lut = 8'h4;
                8'hc6: vcpop_lut = 8'h4;
                8'hc7: vcpop_lut = 8'h5;
                8'hc8: vcpop_lut = 8'h3;
                8'hc9: vcpop_lut = 8'h4;
                8'hca: vcpop_lut = 8'h4;
                8'hcb: vcpop_lut = 8'h5;
                8'hcc: vcpop_lut = 8'h4;
                8'hcd: vcpop_lut = 8'h5;
                8'hce: vcpop_lut = 8'h5;
                8'hcf: vcpop_lut = 8'h6;
                8'hd0: vcpop_lut = 8'h3;
                8'hd1: vcpop_lut = 8'h4;
                8'hd2: vcpop_lut = 8'h4;
                8'hd3: vcpop_lut = 8'h5;
                8'hd4: vcpop_lut = 8'h4;
                8'hd5: vcpop_lut = 8'h5;
                8'hd6: vcpop_lut = 8'h5;
                8'hd7: vcpop_lut = 8'h6;
                8'hd8: vcpop_lut = 8'h4;
                8'hd9: vcpop_lut = 8'h5;
                8'hda: vcpop_lut = 8'h5;
                8'hdb: vcpop_lut = 8'h6;
                8'hdc: vcpop_lut = 8'h5;
                8'hdd: vcpop_lut = 8'h6;
                8'hde: vcpop_lut = 8'h6;
                8'hdf: vcpop_lut = 8'h7;
                8'he0: vcpop_lut = 8'h3;
                8'he1: vcpop_lut = 8'h4;
                8'he2: vcpop_lut = 8'h4;
                8'he3: vcpop_lut = 8'h5;
                8'he4: vcpop_lut = 8'h4;
                8'he5: vcpop_lut = 8'h5;
                8'he6: vcpop_lut = 8'h5;
                8'he7: vcpop_lut = 8'h6;
                8'he8: vcpop_lut = 8'h4;
                8'he9: vcpop_lut = 8'h5;
                8'hea: vcpop_lut = 8'h5;
                8'heb: vcpop_lut = 8'h6;
                8'hec: vcpop_lut = 8'h5;
                8'hed: vcpop_lut = 8'h6;
                8'hee: vcpop_lut = 8'h6;
                8'hef: vcpop_lut = 8'h7;
                8'hf0: vcpop_lut = 8'h4;
                8'hf1: vcpop_lut = 8'h5;
                8'hf2: vcpop_lut = 8'h5;
                8'hf3: vcpop_lut = 8'h6;
                8'hf4: vcpop_lut = 8'h5;
                8'hf5: vcpop_lut = 8'h6;
                8'hf6: vcpop_lut = 8'h6;
                8'hf7: vcpop_lut = 8'h7;
                8'hf8: vcpop_lut = 8'h5;
                8'hf9: vcpop_lut = 8'h6;
                8'hfa: vcpop_lut = 8'h6;
                8'hfb: vcpop_lut = 8'h7;
                8'hfc: vcpop_lut = 8'h6;
                8'hfd: vcpop_lut = 8'h7;
                8'hfe: vcpop_lut = 8'h7;
                8'hff: vcpop_lut = 8'h8;
            endcase
        end
    endfunction

    function automatic [7:0] vfirst8;
        input [7:0] in;
        begin
            if (in[0])
                vfirst8 = 0;
            else if (in[1])
                vfirst8 = 1;
            else if (in[2])
                vfirst8 = 2;
            else if (in[3])
                vfirst8 = 3;
            else if (in[4])
                vfirst8 = 4;
            else if (in[5])
                vfirst8 = 5;
            else if (in[6])
                vfirst8 = 6;
            else if (in[7])
                vfirst8 = 7;
            else
                vfirst8 = -1;
        end
    endfunction
endmodule
