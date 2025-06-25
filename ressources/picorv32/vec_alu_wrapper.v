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

    wire done0_q, done1_q, done2_q, done3_q;

    assign done0 = done0_q;
    assign done1 = done1_q;
    assign done2 = done2_q;
    assign done3 = done3_q;

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
		.vd(vd0),
		.reg_index(regi0),
		.done(done0_q)
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
		.vd(vd1),
		.reg_index(regi1),
		.done(done1_q)
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
		.vd(vd2),
		.reg_index(regi2),
		.done(done2_q)
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
		.vd(vd3),
		.reg_index(regi3),
		.done(done3_q)
	);

endmodule