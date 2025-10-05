`define min(a,b) (a < b ? a : b)

module rvv_alu_wrapper #(
    parameter [9:0] VLEN = 10'd 128,
    parameter [2:0] LANE_WIDTH = 3'b011,
    parameter integer NB_LANES = 1
) (
    input                               clk, resetn,
    input       [5:0]                   opcode,
    input                               run,
    input       [VLEN-1:0]              vs1,
    input       [VLEN-1:0]              vs2,
    input       [2:0]                   vsew,
    input       [2:0]                   op_type,

    output      [(64<<NB_LANES) - 1:0]        vd,
    output      [(10<<NB_LANES) - 1:0]         regi,
    output      [(1<<NB_LANES) - 1:0]         res,
    output                  done_out
);
	localparam [2:0] VV = 3'b001;
	localparam [2:0] VX = 3'b010;
	localparam [2:0] VI = 3'b100;

    localparam integer BASE1 = NB_LANES >= 1 ? 64 : 0;
    localparam integer BASE2 = NB_LANES >= 2 ? 128 : 0;
    localparam integer BASE3 = NB_LANES >= 2 ? 192 : 0;
    localparam integer BASE4 = NB_LANES >= 3 ? 256 : 0;
    localparam integer BASE5 = NB_LANES >= 3 ? 320 : 0;
    localparam integer BASE6 = NB_LANES >= 3 ? 384 : 0;
    localparam integer BASE7 = NB_LANES >= 3 ? 448 : 0;

    reg [9:0] byte_i;
    reg [3:0] in_reg_offset;

    reg done;
    assign done_out = done;

    assign regi = {index7, index6, index5, index4, index3, index2, index1, index0};

    wire [9:0] index0 = ((0 + byte_i) << (vsew + 3)) + (in_reg_offset << LANE_WIDTH);
    wire [9:0] index1 = ((1 + byte_i) << (vsew + 3)) + (in_reg_offset << LANE_WIDTH);
    wire [9:0] index2 = ((2 + byte_i) << (vsew + 3)) + (in_reg_offset << LANE_WIDTH);
    wire [9:0] index3 = ((3 + byte_i) << (vsew + 3)) + (in_reg_offset << LANE_WIDTH);
    wire [9:0] index4 = ((4 + byte_i) << (vsew + 3)) + (in_reg_offset << LANE_WIDTH);
    wire [9:0] index5 = ((5 + byte_i) << (vsew + 3)) + (in_reg_offset << LANE_WIDTH);
    wire [9:0] index6 = ((6 + byte_i) << (vsew + 3)) + (in_reg_offset << LANE_WIDTH);
    wire [9:0] index7 = ((7 + byte_i) << (vsew + 3)) + (in_reg_offset << LANE_WIDTH);

    wire run0 = run;
    wire run1 = run && VLEN >> (vsew+3) > 1 && NB_LANES >= 1;
    wire run2 = run && VLEN >> (vsew+3) > 2 && NB_LANES >= 2;
    wire run3 = run && VLEN >> (vsew+3) > 3 && NB_LANES >= 2;
    wire run4 = run && VLEN >> (vsew+3) > 4 && NB_LANES >= 3;
    wire run5 = run && VLEN >> (vsew+3) > 5 && NB_LANES >= 3;
    wire run6 = run && VLEN >> (vsew+3) > 6 && NB_LANES >= 3;
    wire run7 = run && VLEN >> (vsew+3) > 7 && NB_LANES >= 3;

    assign vd = {vd7,vd6,vd5,vd4,vd3,vd2,vd1,vd0};

    wire [63:0] vd0,vd1,vd2,vd3,vd4,vd5,vd6,vd7;

    assign res = {run7, run6, run5, run4, run3, run2, run1, run0};

    wire [3:0] tmp_nb_lanes = `min(VLEN>>(vsew+3), 1 << NB_LANES);
    wire [1:0] nb_lanes = tmp_nb_lanes[3] ? 2'b11 :
                          tmp_nb_lanes[2] ? 2'b10 :
                          tmp_nb_lanes[1] ? 2'b01 :
                          2'b00;

    always @(posedge clk) begin
        if (!resetn) begin
            byte_i <= 0;
            done <= 0;
        end else if (run0 | run1 | run2 | run3 | run4 | run5 | run6 | run7) begin
            if (!done) begin
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
		.run(run0),
		.vs1_in(vs1),
		.vs2_in(vs2),
		.vsew(vsew),
		.op_type(op_type),
        .index(index0),
        .in_reg_offset(in_reg_offset),
		.vd(vd0)
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
            .run(run1),
            .vs1_in(vs1),
            .vs2_in(vs2),
            .vsew(vsew),
            .op_type(op_type),
            .index(index1),
            .in_reg_offset(in_reg_offset),
            .vd(vd1)
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
            .run(run2),
            .vs1_in(vs1),
            .vs2_in(vs2),
            .vsew(vsew),
            .op_type(op_type),
            .index(index2),
            .in_reg_offset(in_reg_offset),
            .vd(vd2)
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
            .run(run3),
            .vs1_in(vs1),
            .vs2_in(vs2),
            .vsew(vsew),
            .op_type(op_type),
            .index(index3),
            .in_reg_offset(in_reg_offset),
            .vd(vd3)
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
            .run(run4),
            .vs1_in(vs1),
            .vs2_in(vs2),
            .vsew(vsew),
            .op_type(op_type),
            .index(index4),
            .in_reg_offset(in_reg_offset),
            .vd(vd4)
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
            .run(run5),
            .vs1_in(vs1),
            .vs2_in(vs2),
            .vsew(vsew),
            .op_type(op_type),
            .index(index5),
            .in_reg_offset(in_reg_offset),
            .vd(vd5)
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
            .run(run6),
            .vs1_in(vs1),
            .vs2_in(vs2),
            .vsew(vsew),
            .op_type(op_type),
            .index(index6),
            .in_reg_offset(in_reg_offset),
            .vd(vd6)
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
            .run(run7),
            .vs1_in(vs1),
            .vs2_in(vs2),
            .vsew(vsew),
            .op_type(op_type),
            .index(index7),
            .in_reg_offset(in_reg_offset),
            .vd(vd7)
        );
		end
    endgenerate
endmodule