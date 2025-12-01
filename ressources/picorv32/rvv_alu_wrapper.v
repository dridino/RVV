`define min(a,b) (a < b ? a : b)

module rvv_alu_wrapper #(
    parameter [16:0] VLEN = 17'd 128,
    parameter [2:0] LANE_WIDTH = 3'b011,
    parameter integer NB_LANES = 1
) (
    input                               clk, resetn,
    input       [5:0]                   opcode,
    input                               instr_mask,
    input                               run,
    input       [VLEN-1:0]              vs1,
    input       [VLEN-1:0]              vs2,
    input       [2:0]                   vsew,
    input       [2:0]                   op_type,
    input       [16:0]                  vl,
    input       [16:0]                  arith_remaining,

    output      [(64<<NB_LANES) - 1:0]  vd,
    output      [(17<<NB_LANES) - 1:0]  regi,
    output      [(1<<NB_LANES) - 1:0]   res,
    output                              done_out,
    output                              instr_valid
);
    localparam SHIFTED_LANE_WIDTH = 1 << LANE_WIDTH;
	localparam [2:0] VV = 3'b001;
	localparam [2:0] VX = 3'b010;
	localparam [2:0] VI = 3'b100;

    reg [16:0] byte_i;
    reg [3:0] in_reg_offset;

    reg done;
    assign done_out = done;

    assign regi = {index7, index6, index5, index4, index3, index2, index1, index0};

    wire [16:0] index0, index1, index2, index3, index4, index5, index6, index7;

    // TODO : boucle for

    wire run0 = run;
    wire run1 = run && arith_remaining > 1 && NB_LANES >= 1;
    wire run2 = run && arith_remaining > 2 && NB_LANES >= 2;
    wire run3 = run && arith_remaining > 3 && NB_LANES >= 2;
    wire run4 = run && arith_remaining > 4 && NB_LANES >= 3;
    wire run5 = run && arith_remaining > 5 && NB_LANES >= 3;
    wire run6 = run && arith_remaining > 6 && NB_LANES >= 3;
    wire run7 = run && arith_remaining > 7 && NB_LANES >= 3;

    assign vd = {vd7,vd6,vd5,vd4,vd3,vd2,vd1,vd0};

    wire [63:0] vd0,vd1,vd2,vd3,vd4,vd5,vd6,vd7;

    assign res = {run7, run6, run5, run4, run3, run2, run1, run0};

    wire instr_valid0, instr_valid1, instr_valid2, instr_valid3, instr_valid4, instr_valid5, instr_valid6, instr_valid7;
    assign instr_valid = instr_valid0;

    // TODO : for
    wire [16:0] tmp_nb_lanes = `min(vl, 1 << NB_LANES);
    wire [1:0] nb_lanes = tmp_nb_lanes[3] ? 2'b11 :
                          tmp_nb_lanes[2] ? 2'b10 :
                          tmp_nb_lanes[1] ? 2'b01 :
                          2'b00;

    always @(posedge clk) begin
        if (!resetn) begin
            byte_i <= 0;
            done <= 0;
        end else if (run) begin
            if (!done) begin
                if (vsew+3 <= LANE_WIDTH) begin
                    done <= byte_i + (1 << (nb_lanes)) >= (instr_mask ? (VLEN >> (vsew + 3)) : `min(vl, (VLEN >> (vsew + 3))));
                end else begin
                    done <= byte_i + (1 << (nb_lanes)) >= (instr_mask ? (VLEN >> (vsew + 3)) : `min(vl, (VLEN >> (vsew + 3)))) && in_reg_offset == ((1 << (vsew+3-LANE_WIDTH)) - 2);
                end

                if (vsew + 3 < LANE_WIDTH || in_reg_offset == (vsew + 3 <= LANE_WIDTH ? 0 : (1 << (vsew+3-LANE_WIDTH)) - 1)) begin
                    in_reg_offset <= 0;
                    byte_i <= byte_i + (1<<nb_lanes);
                end else
                    in_reg_offset <= in_reg_offset + 1;
            end else begin
                done <= 0;
            end
        end else begin
            byte_i <= 0;
            in_reg_offset <= 0;
            done <= 0;
        end
    end

    rvv_alu #(
		.VLEN (VLEN),
		.LANE_WIDTH (LANE_WIDTH),
		.LANE_I (3'b000)
	) valu0 (
		.clk(clk),
		.resetn(resetn),
		.nb_lanes(nb_lanes),
		.opcode(opcode),
        .instr_mask(instr_mask),
		.run(run0),
		.vs1_in(vs1),
		.vs2_in(vs2),
		.vsew(vsew),
		.op_type(op_type),
        .byte_i(byte_i),
        .in_reg_offset(in_reg_offset),
		.vd(vd0),
        .index(index0),
        .instr_valid(instr_valid0)
	);
	
    generate if (NB_LANES >= 1)
        rvv_alu #(
            .VLEN (VLEN),
            .LANE_WIDTH (LANE_WIDTH),
            .LANE_I (3'b001)
        ) valu1 (
            .clk(clk),
            .resetn(resetn),
            .nb_lanes(nb_lanes),
            .opcode(opcode),
            .instr_mask(instr_mask),
            .run(run1),
            .vs1_in(vs1),
            .vs2_in(vs2),
            .vsew(vsew),
            .op_type(op_type),
            .byte_i(byte_i),
            .in_reg_offset(in_reg_offset),
            .vd(vd1),
            .index(index1),
            .instr_valid(instr_valid1)
        );
    endgenerate
	
    generate if (NB_LANES >= 2) begin
        rvv_alu #(
            .VLEN (VLEN),
            .LANE_WIDTH (LANE_WIDTH),
            .LANE_I (3'b010)
        ) valu2 (
            .clk(clk),
            .resetn(resetn),
            .nb_lanes(nb_lanes),
            .opcode(opcode),
            .instr_mask(instr_mask),
            .run(run2),
            .vs1_in(vs1),
            .vs2_in(vs2),
            .vsew(vsew),
            .op_type(op_type),
            .byte_i(byte_i),
            .in_reg_offset(in_reg_offset),
            .vd(vd2),
            .index(index2),
            .instr_valid(instr_valid2)
        );
        
       rvv_alu #(
            .VLEN (VLEN),
            .LANE_WIDTH (LANE_WIDTH),
            .LANE_I (3'b011)
        ) valu3 (
            .clk(clk),
            .resetn(resetn),
            .nb_lanes(nb_lanes),
            .opcode(opcode),
            .instr_mask(instr_mask),
            .run(run3),
            .vs1_in(vs1),
            .vs2_in(vs2),
            .vsew(vsew),
            .op_type(op_type),
            .byte_i(byte_i),
            .in_reg_offset(in_reg_offset),
            .vd(vd3),
            .index(index3),
            .instr_valid(instr_valid3)
        );
		end
    endgenerate
	
    generate if (NB_LANES >= 3) begin
        rvv_alu #(
            .VLEN (VLEN),
            .LANE_WIDTH (LANE_WIDTH),
            .LANE_I (3'b100)
        ) valu4 (
            .clk(clk),
            .resetn(resetn),
            .nb_lanes(nb_lanes),
            .opcode(opcode),
            .instr_mask(instr_mask),
            .run(run4),
            .vs1_in(vs1),
            .vs2_in(vs2),
            .vsew(vsew),
            .op_type(op_type),
            .byte_i(byte_i),
            .in_reg_offset(in_reg_offset),
            .vd(vd4),
            .index(index4),
            .instr_valid(instr_valid4)
        );
        
        rvv_alu #(
            .VLEN (VLEN),
            .LANE_WIDTH (LANE_WIDTH),
            .LANE_I (3'b101)
        ) valu5 (
            .clk(clk),
            .resetn(resetn),
            .nb_lanes(nb_lanes),
            .opcode(opcode),
            .instr_mask(instr_mask),
            .run(run5),
            .vs1_in(vs1),
            .vs2_in(vs2),
            .vsew(vsew),
            .op_type(op_type),
            .byte_i(byte_i),
            .in_reg_offset(in_reg_offset),
            .vd(vd5),
            .index(index5),
            .instr_valid(instr_valid5)
        );
        
        rvv_alu #(
            .VLEN (VLEN),
            .LANE_WIDTH (LANE_WIDTH),
            .LANE_I (3'b110)
        ) valu6 (
            .clk(clk),
            .resetn(resetn),
            .nb_lanes(nb_lanes),
            .opcode(opcode),
            .instr_mask(instr_mask),
            .run(run6),
            .vs1_in(vs1),
            .vs2_in(vs2),
            .vsew(vsew),
            .op_type(op_type),
            .byte_i(byte_i),
            .in_reg_offset(in_reg_offset),
            .vd(vd6),
            .index(index6),
            .instr_valid(instr_valid6)
        );
        
        rvv_alu #(
            .VLEN (VLEN),
            .LANE_WIDTH (LANE_WIDTH),
            .LANE_I (3'b111)
        ) valu7 (
            .clk(clk),
            .resetn(resetn),
            .nb_lanes(nb_lanes),
            .opcode(opcode),
            .instr_mask(instr_mask),
            .run(run7),
            .vs1_in(vs1),
            .vs2_in(vs2),
            .vsew(vsew),
            .op_type(op_type),
            .byte_i(byte_i),
            .in_reg_offset(in_reg_offset),
            .vd(vd7),
            .index(index7),
            .instr_valid(instr_valid7)
        );
		end
    endgenerate
endmodule