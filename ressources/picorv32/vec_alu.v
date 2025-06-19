module vec_alu #(
	parameter [9:0] VLEN = 10'd 128,
    parameter [1:0] NB_LANES = 2'b01, // 2^NB_LANES lanes used for arith / logic operations
    parameter [2:0] LANE_WIDTH = 3'b011, // 2^LANE_WIDTH bits per lane (8,16,32,64)
    // LANE_WIDTH * 2^NB_LANES must be less than or equal to VLEN
    parameter [2:0] LANE_I = 3'b000
) (
    input               clk, resetn,
    input      [5:0]    opcode,
    input               run,
    input      [VLEN-1:0] vs1,
    input      [VLEN-1:0] vs2,
    input      [2:0]    vsew,

    output reg [VLEN-1:0] vd,
    output reg [9:0] reg_index,
    output reg          done
);
    localparam [7:0] SHIFTED_LANE_WIDTH = 1 << LANE_WIDTH;

    reg [9:0]  byte_i;
    reg [63:0] temp_vreg;
    reg [3:0]  in_reg_offset;

    integer index;

    always @(posedge clk) begin
        if (!resetn) begin
            byte_i <= 0;
            temp_vreg <= 0;
            in_reg_offset <= 0;
            done <= 0;
            reg_index <= 0;
        end else if (run) begin
            if (!done) begin
                index = ((LANE_I + byte_i) << (vsew + 3)) + (in_reg_offset << LANE_WIDTH);
                case (opcode)
                    6'b001001: begin // vand
                        case (vsew)
                            // 8 bits
                            3'b000: begin
                                temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vs1[index +: SHIFTED_LANE_WIDTH] & vs2[index +: SHIFTED_LANE_WIDTH];
                                if (LANE_WIDTH >= vsew+3)
                                    vd[index +: 8] = temp_vreg[0 +: 8];
                            end
                            // 16 bits
                            3'b001: begin
                                temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vs1[index +: SHIFTED_LANE_WIDTH] & vs2[index +: SHIFTED_LANE_WIDTH];
                                if (LANE_WIDTH >= vsew+3)
                                    vd[index +: 16] = temp_vreg[0 +: 16];
                            end
                            // 32 bits
                            3'b010: begin
                                temp_vreg[0 +: SHIFTED_LANE_WIDTH] = (vs1[index +: SHIFTED_LANE_WIDTH] & vs2[index +: SHIFTED_LANE_WIDTH]);
                                if (LANE_WIDTH >= vsew+3)
                                    vd[index +: 32] = temp_vreg[0 +: 32];
                            end
                            // 64 bits
                            3'b011: begin
                                temp_vreg[0 +: SHIFTED_LANE_WIDTH] = vs1[index +: SHIFTED_LANE_WIDTH] & vs2[index +: SHIFTED_LANE_WIDTH];
                                if (LANE_WIDTH >= vsew+3)
                                    vd[index +: 64] = temp_vreg[0 +: 64];
                            end
                        endcase
                        if (LANE_WIDTH < vsew+3)
                            vd[index +: SHIFTED_LANE_WIDTH] = temp_vreg[0 +: SHIFTED_LANE_WIDTH];

                    end
                endcase
                // $display("lane%d byte_i : %d, reg_off : %d, index : %d", LANE_I, byte_i, in_reg_offset, index);
                done <= byte_i + (NB_LANES) == (VLEN >> (vsew+3)) - 1 && in_reg_offset == (vsew + 3 <= LANE_WIDTH ? 0 : (1 << (vsew+3-LANE_WIDTH)) - 1);

                if (vsew + 3 < LANE_WIDTH || in_reg_offset == (vsew + 3 <= LANE_WIDTH ? 0 : (1 << (vsew+3-LANE_WIDTH)) - 1)) begin
                    in_reg_offset = 0;
                    byte_i = byte_i + (1<<NB_LANES);
                end else
                    in_reg_offset = in_reg_offset + 1;
                
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