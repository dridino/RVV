module vec_alu #(
	parameter [9:0] VLEN = 10'd 128,
    parameter [1:0] NB_LANES = 2'b01, // 2^NB_LANES lanes used for arith / logic operations
    parameter [2:0] LANE_WIDTH = 3'b101, // 2^LANE_WIDTH bits per lane (8,16,32,64)
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
    output reg          done
);

    reg [9:0] byte_i;
    reg [VLEN-1:0] temp_vreg;
    reg [2:0] in_reg_offset;
    integer index;

    always @(posedge clk) begin
        if (!resetn) begin
            byte_i <= 0;
            temp_vreg <= 0;
            in_reg_offset <= 0;
            done <= 0;
        end else if (run) begin
            if (!done) begin
                index = ((LANE_I + byte_i) << (vsew + 3)) + (in_reg_offset << LANE_WIDTH);
                // $display("lane byte_i : %d, reg_off : %d, index : %d", byte_i, in_reg_offset, index);
                case (opcode)
                    6'b001001: begin // vand
                        case (vsew)
                            // 8 bits
                            3'b000: begin
                                temp_vreg[0 +: 32] = ({{24{1'b0}}, vs1[index +: 8]} & {{24{1'b0}}, vs2[index +: 8]});
                                vd[index +: 8] = temp_vreg[0 +: 8];
                            end
                            // 16 bits
                            3'b001: begin
                                temp_vreg[0 +: 32] = ({{16{1'b0}}, vs1[index +: 16]} & {{16{1'b0}},vs2[index +: 16]});
                                vd[index +: 16] = temp_vreg[0 +: 16];
                            end
                            // 32 bits
                            3'b010: vd[index +: 32] = (vs1[index +: 32] & vs2[index +: 32]);
                            // 64 bits TODO
                            3'b011: vd[index +: 32] = vs1[index +: 32] & vs2[index +: 32];
                        endcase
                    end
                endcase

                // $display("lane op : %b", (1 << (LANE_WIDTH - (vsew+1))));
                // $display("lane op : %b, %b, %b", in_reg_offset, ((vsew+3) - LANE_WIDTH), $signed(in_reg_offset) >= $signed(((vsew+3) - LANE_WIDTH)));
                
                done <= byte_i == (1 << (LANE_WIDTH - (vsew+1))) - 1 && ($signed(in_reg_offset) >= $signed((vsew+3) - LANE_WIDTH));

                if (vsew + 3 < LANE_WIDTH || in_reg_offset == ((vsew+3) - LANE_WIDTH)) begin
                    in_reg_offset = 0;
                    byte_i = byte_i + (1<<NB_LANES);
                end else
                    in_reg_offset = in_reg_offset + 1;
            end
        end else begin
            byte_i <= 0;
            done <= 0;
        end
    end
endmodule