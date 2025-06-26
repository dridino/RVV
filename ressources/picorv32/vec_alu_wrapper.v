module vec_alu_wrapper #(
    parameter [9:0] VLEN = 10'd 128,
    parameter [2:0] LANE_WIDTH = 3'b100
) (
    input                   clk, resetn,
    input       [1:0]       nb_lanes,
    input       [5:0]       opcode,
    input                   run0,run1,run2,run3,
    input       [VLEN-1:0]  vs1,
    input       [VLEN-1:0]  vs2,
    input       [2:0]       vsew,
    input       [2:0]       op_type,

    output      [63:0]      vd0,vd1,vd2,vd3,
    output      [9:0]       regi0,regi1,regi2,regi3,
    output                  done0,done1,done2,done3
);
	localparam [2:0] VV = 3'b001;
	localparam [2:0] VX = 3'b010;
	localparam [2:0] VI = 3'b100;

    reg dd0,dd1,dd2,dd3;
    reg [9:0] byte_i;
    reg [3:0] in_reg_offset;

    assign done0 = dd0;
    assign done1 = dd1;
    assign done2 = dd2;
    assign done3 = dd3;

    assign regi0 = index0;
    assign regi1 = index1;
    assign regi2 = index2;
    assign regi3 = index3;

    wire [9:0] index0 = ((0 + byte_i) << (vsew + 3)) + (in_reg_offset << LANE_WIDTH);
    wire [9:0] index1 = ((1 + byte_i) << (vsew + 3)) + (in_reg_offset << LANE_WIDTH);
    wire [9:0] index2 = ((2 + byte_i) << (vsew + 3)) + (in_reg_offset << LANE_WIDTH);
    wire [9:0] index3 = ((3 + byte_i) << (vsew + 3)) + (in_reg_offset << LANE_WIDTH);

    always @(posedge clk) begin
        if (!resetn) begin
            byte_i <= 0;
            reg_index <= 0;
            dd0 <= 0;
            dd1 <= 0;
            dd2 <= 0;
            dd3 <= 0;
        end else if (run0 | run1 | run2 | run3) begin
            if (!(dd0 || dd1 || dd2 || dd3)) begin
                if (vsew+3 <= LANE_WIDTH) begin
                    dd0 <= byte_i + (1 << (nb_lanes+1)) >= (VLEN >> (vsew + 3));
                    dd1 <= byte_i + (1 << (nb_lanes+1)) >= (VLEN >> (vsew + 3));
                    dd2 <= byte_i + (1 << (nb_lanes+1)) >= (VLEN >> (vsew + 3));
                    dd3 <= byte_i + (1 << (nb_lanes+1)) >= (VLEN >> (vsew + 3));
                end else begin
                    dd0 <= byte_i + (1 << (nb_lanes)) >= (VLEN >> (vsew + 3)) && in_reg_offset == ((1 << (vsew+3-LANE_WIDTH)) - 2);
                    dd1 <= byte_i + (1 << (nb_lanes)) >= (VLEN >> (vsew + 3)) && in_reg_offset == ((1 << (vsew+3-LANE_WIDTH)) - 2);
                    dd2 <= byte_i + (1 << (nb_lanes)) >= (VLEN >> (vsew + 3)) && in_reg_offset == ((1 << (vsew+3-LANE_WIDTH)) - 2);
                    dd3 <= byte_i + (1 << (nb_lanes)) >= (VLEN >> (vsew + 3)) && in_reg_offset == ((1 << (vsew+3-LANE_WIDTH)) - 2);
                end

                if (vsew + 3 < LANE_WIDTH || in_reg_offset == (vsew + 3 <= LANE_WIDTH ? 0 : (1 << (vsew+3-LANE_WIDTH)) - 1)) begin
                    in_reg_offset <= 0;
                    byte_i <= byte_i + (1<<nb_lanes);
                end else
                    in_reg_offset <= in_reg_offset + 1;
            end else begin
                dd0 <= 0;
                dd1 <= 0;
                dd2 <= 0;
                dd3 <= 0;
            end
        end else begin
            byte_i <= 0;
            in_reg_offset <= 0;
            dd0 <= 0;
            dd1 <= 0;
            dd2 <= 0;
            dd3 <= 0;
        end
    end

    vec_alu #(
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
	
	vec_alu #(
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
	
	vec_alu #(
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
	
	vec_alu #(
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

endmodule