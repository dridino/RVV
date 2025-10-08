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

    input      [9:0]    index,
    input      [3:0]    in_reg_offset,

    output [63:0] vd
);
    localparam [7:0] SHIFTED_LANE_WIDTH = 1 << LANE_WIDTH;
    localparam [8:0] ADD_SHIFTED_LANE_WIDTH = SHIFTED_LANE_WIDTH + 1;
	localparam [2:0] VV = 3'b001;
	localparam [2:0] VX = 3'b010;
	localparam [2:0] VI = 3'b100;

    function [SHIFTED_LANE_WIDTH:0] trunc_after_add(input [ADD_SHIFTED_LANE_WIDTH-1:0] i);
        trunc_after_add = i[0+:SHIFTED_LANE_WIDTH];
    endfunction

    reg [64:0] temp_vreg; // 64 + 1 for carry out
    
    // set to 0 when computing a new element of the vector
    // set to 1 if it's a sub operation
    wire cout = (in_reg_offset == (vsew + 3 <= LANE_WIDTH ? 0 : (1 << (vsew+3-LANE_WIDTH)) - 1)) ? 0 : temp_vreg[ADD_SHIFTED_LANE_WIDTH - 1];

    wire [9:0] base_index = index - (in_reg_offset << LANE_WIDTH);

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
        : vs1_in[op_type == VV ? index : (in_reg_offset << LANE_WIDTH) +: SHIFTED_LANE_WIDTH];
    wire [SHIFTED_LANE_WIDTH-1:0] vs2 =
        opcode == 6'b000011 ? signed_vs2_rsub[(in_reg_offset << LANE_WIDTH) +: SHIFTED_LANE_WIDTH]
        : vs2_in[index +: SHIFTED_LANE_WIDTH];
    
    reg cout_q;

    assign vd = temp_vreg[0 +: 64];

    always @* begin
        if (!resetn) begin
            temp_vreg = {65{1'b0}};
        end else if (run) begin
            temp_vreg = 0;
            case (opcode)
                // vand
                6'b001001: begin
                    temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vs2_in[index +: SHIFTED_LANE_WIDTH] & vs1_in[op_type == VV ? index : (in_reg_offset << LANE_WIDTH) +: SHIFTED_LANE_WIDTH];
                end
                // vor
                6'b001010: begin
                    temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vs2_in[index +: SHIFTED_LANE_WIDTH] | vs1_in[op_type == VV ? index : (in_reg_offset << LANE_WIDTH) +: SHIFTED_LANE_WIDTH];
                end
                // vxor
                6'b001011: begin
                    temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vs2_in[index +: SHIFTED_LANE_WIDTH] ^ vs1_in[op_type == VV ? index : (in_reg_offset << LANE_WIDTH) +: SHIFTED_LANE_WIDTH];
                end
                // vadd
                6'b000000: begin
                    temp_vreg[0 +: ADD_SHIFTED_LANE_WIDTH] = vs2_in[index +: SHIFTED_LANE_WIDTH] + vs1_in[op_type == VV ? index : (in_reg_offset << LANE_WIDTH) +: SHIFTED_LANE_WIDTH] + cout_q;
                end
                // vsub
                6'b000010: begin
                    temp_vreg[0 +: ADD_SHIFTED_LANE_WIDTH] = vs2 + vs1 + cout_q;
                end
                // vrsub
                6'b000011: begin
                    temp_vreg[0 +: ADD_SHIFTED_LANE_WIDTH] = vs2 + vs1 + cout_q;
                end
                default: begin
                    temp_vreg = 0;
                end
            endcase
        end else begin
            temp_vreg = 0;
        end
    end

    always @(posedge clk) begin
        cout_q <= cout;
    end
endmodule
