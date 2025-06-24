module vec_alu #(
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

    output reg [VLEN-1:0] vd,
    output reg [9:0] reg_index,
    output reg          done
);
    localparam [7:0] SHIFTED_LANE_WIDTH = 1 << LANE_WIDTH;
    localparam [8:0] ADD_SHIFTED_LANE_WIDTH = SHIFTED_LANE_WIDTH + 1;
	localparam [2:0] VV = 3'b001;
	localparam [2:0] VX = 3'b010;
	localparam [2:0] VI = 3'b100;

    reg [9:0]  byte_i;
    reg [64:0] temp_vreg; // 64 + 1 for carry out
    reg [3:0]  in_reg_offset;

    reg cout;

    integer index;

    reg [63:0] vs1;
    reg [63:0] vs2;

    always @(posedge clk) begin
        if (!resetn) begin
            byte_i <= 0;
            temp_vreg <= 0;
            in_reg_offset <= 0;
            cout <= 0;
            vs1 <= 0;
            vs2 <= 0;
            done <= 0;
            reg_index <= 0;
        end else if (run) begin
            if (!done) begin
                index = ((LANE_I + byte_i) << (vsew + 3)) + (in_reg_offset << LANE_WIDTH);
                
                case (vsew)
                    // 8 bits
                    3'b000: begin
                        vs1 = {{56{1'b0}}, vs1_in[op_type == VV ? index : (in_reg_offset << LANE_WIDTH) +: 8]};
                        vs2 = {{56{1'b0}}, vs2_in[index +: 8]};
                    end
                    // 16 bits
                    3'b001: begin
                        vs1 = {{48{1'b0}}, vs1_in[op_type == VV ? index : (in_reg_offset << LANE_WIDTH) +: 16]};
                        vs2 = {{48{1'b0}}, vs2_in[index +: 16]};
                    end
                    // 32 bits
                    3'b010: begin
                        vs1 = {{32{1'b0}}, vs1_in[op_type == VV ? index : (in_reg_offset << LANE_WIDTH) +: 32]};
                        vs2 = {{32{1'b0}}, vs2_in[index +: 32]};
                    end
                    // 64 bits
                    3'b011: begin
                        vs1 = vs1_in[op_type == VV ? index : (in_reg_offset << LANE_WIDTH) +: 64];
                        vs2 = vs2_in[index +: 64];
                    end
                endcase

                case (opcode)
                    // vand
                    6'b001001: temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vs1[0 +: SHIFTED_LANE_WIDTH] & vs2[0 +: SHIFTED_LANE_WIDTH];
                    // vor
                    6'b001010: temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vs1[0 +: SHIFTED_LANE_WIDTH] | vs2[0 +: SHIFTED_LANE_WIDTH];
                    // vxor
                    6'b001011: temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vs1[0 +: SHIFTED_LANE_WIDTH] ^ vs2[0 +: SHIFTED_LANE_WIDTH];
                    // vadd
                    6'b000000: begin
                        temp_vreg[0 +: ADD_SHIFTED_LANE_WIDTH] = vs1[0 +: SHIFTED_LANE_WIDTH] + vs2[0 +: SHIFTED_LANE_WIDTH] + cout;
                        cout = temp_vreg[ADD_SHIFTED_LANE_WIDTH - 1];
                        temp_vreg[ADD_SHIFTED_LANE_WIDTH - 1] = 1'b0;
                    end
                endcase

                case (vsew)
                    // 8 bits
                    3'b000: 
                        if (LANE_WIDTH >= vsew+3)
                            vd[index +: 8] = temp_vreg[0 +: 8];
                    // 16 bits
                    3'b001: 
                        if (LANE_WIDTH >= vsew+3)
                            vd[index +: 16] = temp_vreg[0 +: 16];
                    // 32 bits
                    3'b010: 
                        if (LANE_WIDTH >= vsew+3)
                            vd[index +: 32] = temp_vreg[0 +: 32];
                    // 64 bits
                    3'b011: 
                        if (LANE_WIDTH >= vsew+3)
                            vd[index +: 64] = temp_vreg[0 +: 64];
                endcase

                if (LANE_WIDTH < vsew+3)
                    vd[index +: SHIFTED_LANE_WIDTH] = temp_vreg[0 +: SHIFTED_LANE_WIDTH];

                // $display("lane%d byte_i : %d, reg_off : %d, index : %d", LANE_I, byte_i, in_reg_offset, index);
                done <= byte_i + (1 << nb_lanes) == (VLEN >> (vsew+3)) && in_reg_offset == (vsew + 3 <= LANE_WIDTH ? 0 : (1 << (vsew+3-LANE_WIDTH)) - 1);

                if (vsew + 3 < LANE_WIDTH || in_reg_offset == (vsew + 3 <= LANE_WIDTH ? 0 : (1 << (vsew+3-LANE_WIDTH)) - 1)) begin
                    in_reg_offset = 0;
                    byte_i = byte_i + (1<<nb_lanes);
                end else
                    in_reg_offset = in_reg_offset + 1;

                if (in_reg_offset == 0)
                    cout <= 0;
                
                reg_index <= index;
            end
        end else begin
            byte_i <= 0;
            in_reg_offset <= 0;
            done <= 0;
            reg_index <= 0;
            
            // FOR TESTS ONLY
            vd <= 0;
        end
    end
endmodule