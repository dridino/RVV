`timescale 1 ns / 1 ps

`ifndef VERILATOR

`define assert(signal, value) \
        if (signal !== value) begin \
            $display("\n[ERROR] ASSERTION FAILED in %m: signal != value\n"); \
            $finish; \
        end

`define min(a,b) (a < b ? a : b)


module testbench ();
	localparam [2:0] LANE_WIDTH = 3'b101;
	localparam [7:0] SHIFTED_LANE_WIDTH = 1 << LANE_WIDTH;
	localparam [1:0] NB_LANES = 2'b10;
	localparam [9:0] VLEN = 10'd128;
	localparam [4:0] IMM = 5'b01111;
	localparam [31:0] RS1 = 32'hFFFFFFFF;
	
	localparam [2:0] OP_TYPE = 3'b001;
	localparam [2:0] VV = 3'b001;
	localparam [2:0] VX = 3'b010;
	localparam [2:0] VI = 3'b100;

	reg clk = 0;
	reg resetn = 0;
	wire trap;

	wire trace_valid;
	wire [35:0] trace_data;
	integer trace_file;

	reg [5:0] opcode;
	reg	run0,run1,run2,run3;
	reg [127:0] vs1;
	reg [127:0] vs2;
	reg [2:0] vsew;

	reg [127:0] vd;
	wire [127:0] vd0, vd1, vd2, vd3;
	wire done0, done1, done2, done3;
	wire [9:0] regi0,regi1,regi2,regi3;

	reg [1:0] nb_lanes;


	task repeat_loop;
		input integer n;
		input print;
		begin
			vd <= 0;
			run0 <= 1;
			if (VLEN >> (vsew+3) > 1 && NB_LANES >= 1) run1 <= 1; else run1 <= 0;
			if (VLEN >> (vsew+3) > 2 && NB_LANES >= 2) run2 <= 1; else run2 <= 0;
			if (VLEN >> (vsew+3) > 3 && NB_LANES >= 2) run3 <= 1; else run3 <= 0;
			repeat (n-1) begin
				run0 <= 1;
				if (VLEN >> (vsew+3) > 1 && NB_LANES >= 1) run1 <= 1; else run1 <= 0;
				if (VLEN >> (vsew+3) > 2 && NB_LANES >= 2) run2 <= 1; else run2 <= 0;
				if (VLEN >> (vsew+3) > 3 && NB_LANES >= 2) run3 <= 1; else run3 <= 0;

				clk <= 1;
				#5;
				clk <= 0;
				#5;
				
				`assert(done0, 1'b0)
				`assert(done1, 1'b0)
				`assert(done2, 1'b0)
				`assert(done3, 1'b0)

				if (vsew + 3 > SHIFTED_LANE_WIDTH) begin // impossible for 8b
					case (vsew)
						3'b001: begin
							vd[regi0 +: 16] = vd0[regi0 +: 16];
							if (nb_lanes >= 1) vd[regi1 +: 16] = vd1[regi1 +: 16];
							if (nb_lanes >= 2) vd[regi2 +: 16] = vd1[regi2 +: 16];
							if (nb_lanes >= 2) vd[regi3 +: 16] = vd1[regi3 +: 16];
						end
						3'b010: begin
							vd[regi0 +: 32] = vd0[regi0 +: 32];
							if (nb_lanes >= 1) vd[regi1 +: 32] = vd1[regi1 +: 32];
							if (nb_lanes >= 2) vd[regi2 +: 32] = vd1[regi2 +: 32];
							if (nb_lanes >= 2) vd[regi3 +: 32] = vd1[regi3 +: 32];
						end
						3'b011: begin
							vd[regi0 +: 64] = vd0[regi0 +: 64];
							if (nb_lanes >= 1) vd[regi1 +: 64] = vd1[regi1 +: 64];
							if (nb_lanes >= 2) vd[regi2 +: 64] = vd1[regi2 +: 64];
							if (nb_lanes >= 2) vd[regi3 +: 64] = vd1[regi3 +: 64];
						end
					endcase
				end else begin
					vd[regi0 +: SHIFTED_LANE_WIDTH] = vd0[regi0 +: SHIFTED_LANE_WIDTH];
					if (nb_lanes >= 1) vd[regi1 +: SHIFTED_LANE_WIDTH] = vd1[regi1 +: SHIFTED_LANE_WIDTH];
					if (nb_lanes >= 2) vd[regi2 +: SHIFTED_LANE_WIDTH] = vd2[regi2 +: SHIFTED_LANE_WIDTH];
					if (nb_lanes >= 2) vd[regi3 +: SHIFTED_LANE_WIDTH] = vd3[regi3 +: SHIFTED_LANE_WIDTH];
				end
				
				if (print) $display("index1 : %d", regi0);
				if (print) $display("index2 : %d", regi1);
				if (print) $display("index3 : %d", regi2);
				if (print) $display("index4 : %d", regi3);
				if (print) $display("vd : %h", vd);
			end
			clk <= 1;
			#5;
			clk <= 0;
			#5;
			if (vsew + 3 > SHIFTED_LANE_WIDTH) begin // impossible for 8b
				case (vsew)
					3'b001: begin
						vd[regi0 +: 16] = vd0[regi0 +: 16];
						if (nb_lanes >= 1) vd[regi1 +: 16] = vd1[regi1 +: 16];
						if (nb_lanes >= 2) vd[regi2 +: 16] = vd2[regi2 +: 16];
						if (nb_lanes >= 2) vd[regi3 +: 16] = vd3[regi3 +: 16];
					end
					3'b010: begin
						vd[regi0 +: 32] = vd0[regi0 +: 32];
						if (nb_lanes >= 1) vd[regi1 +: 32] = vd1[regi1 +: 32];
						if (nb_lanes >= 2) vd[regi2 +: 32] = vd2[regi2 +: 32];
						if (nb_lanes >= 2) vd[regi3 +: 32] = vd3[regi3 +: 32];
					end
					3'b011: begin
						vd[regi0 +: 64] = vd0[regi0 +: 64];
						if (nb_lanes >= 1) vd[regi1 +: 64] = vd1[regi1 +: 64];
						if (nb_lanes >= 2) vd[regi2 +: 64] = vd2[regi2 +: 64];
						if (nb_lanes >= 2) vd[regi3 +: 64] = vd3[regi3 +: 64];
					end
				endcase
			end else begin
				vd[regi0 +: SHIFTED_LANE_WIDTH] = vd0[regi0 +: SHIFTED_LANE_WIDTH];
				if (nb_lanes >= 1) vd[regi1 +: SHIFTED_LANE_WIDTH] = vd1[regi1 +: SHIFTED_LANE_WIDTH];
				if (nb_lanes >= 2) vd[regi2 +: SHIFTED_LANE_WIDTH] = vd2[regi2 +: SHIFTED_LANE_WIDTH];
				if (nb_lanes >= 2) vd[regi3 +: SHIFTED_LANE_WIDTH] = vd3[regi3 +: SHIFTED_LANE_WIDTH];
			end
			if (print) $display("index1 : %d", regi0);
			if (print) $display("index2 : %d", regi1);
			if (print) $display("index3 : %d", regi2);
			if (print) $display("index4 : %d", regi3);
			if (print) $display("vd : %h", vd);
		end
	endtask

	initial begin
		$display("+---------------------------------+\n| Test with %2d LANE(S) of %2d bits |\n+---------------------------------+", 1 << NB_LANES, 1 << LANE_WIDTH);

		repeat (200) begin
			clk <= ~clk;
			#5;
		end
		resetn <= 1;
		opcode <= 6'b001011;
		case (OP_TYPE)
			VV: vs1 <= 128'habcdabcdbeefbeef1234567887654321;
			VX: vs1 <= {{96{1'b0}}, RS1};
			VI: vs1 <= {{123{1'b0}}, IMM};
		endcase
		vs2 <= 128'h8765432112345678beefbeefabcdabcd;
		
		// -------------------------- 8 BITS --------------------------
		vsew <= 3'b000;
		run0 <= 0; run1 <= 0; run2 <= 0; run3 <= 0;
		clk <= 1;
		#5;
		case (`min(VLEN>>(vsew+3), 1 << NB_LANES))
			0,
			1: nb_lanes <= 0;
			2,
			3: nb_lanes <= 1;
			4,
			5,
			6,
			7: nb_lanes <= 2;
			8: nb_lanes <= 3;
		endcase
		clk <= 0;
		#5;
		
		repeat_loop((VLEN >> `min(3, LANE_WIDTH))>>nb_lanes, 1);
		`assert(done0, 1'b1)
		if (nb_lanes >= 1) `assert(done1, 1'b1)
		if (nb_lanes >= 2) `assert(done2, 1'b1)
		if (nb_lanes >= 2) `assert(done3, 1'b1)
		// `assert(vd, 128'h83450301122416681224166883450301)
		$display("[OK] 8b");

		// -------------------------- 16 BITS --------------------------
		vsew <= 3'b001;
		run0 <= 0; run1 <= 0; run2 <= 0; run3 <= 0;
		clk <= 1;
		#5;
		case (`min(VLEN>>(vsew+3), 1 << NB_LANES))
			0,
			1: nb_lanes <= 0;
			2,
			3: nb_lanes <= 1;
			4,
			5,
			6,
			7: nb_lanes <= 2;
			8: nb_lanes <= 3;
		endcase
		clk <= 0;
		#5;
		
		repeat_loop((VLEN >> `min(4, LANE_WIDTH))>>nb_lanes, 1);
		`assert(done0, 1'b1)
		if (nb_lanes >= 1) `assert(done1, 1'b1)
		if (nb_lanes >= 2) `assert(done2, 1'b1)
		if (nb_lanes >= 2) `assert(done3, 1'b1)
		// `assert(vd, 128'h83450301122416681224166883450301)
		$display("[OK] 16b");

		// -------------------------- 32 BITS --------------------------
		vsew <= 3'b010;
		run0 <= 0; run1 <= 0; run2 <= 0; run3 <= 0;
		clk <= 1;
		#5;
		case (`min(VLEN>>(vsew+3), 1 << NB_LANES))
			0,
			1: nb_lanes <= 0;
			2,
			3: nb_lanes <= 1;
			4,
			5,
			6,
			7: nb_lanes <= 2;
			8: nb_lanes <= 3;
		endcase
		clk <= 0;
		#5;
		
		repeat_loop((VLEN >> `min(5, LANE_WIDTH))>>nb_lanes, 1);
		`assert(done0, 1'b1)
		if (nb_lanes >= 1) `assert(done1, 1'b1)
		if (nb_lanes >= 2) `assert(done2, 1'b1)
		if (nb_lanes >= 2) `assert(done3, 1'b1)
		// `assert(vd, 128'h83450301122416681224166883450301)
		$display("[OK] 32b");
		
		// -------------------------- 64 BITS --------------------------
		vsew <= 3'b011;
		run0 <= 0; run1 <= 0; run2 <= 0; run3 <= 0;
		clk <= 1;
		#5;
		case (`min(VLEN>>(vsew+3), 1 << NB_LANES))
			0,
			1: nb_lanes <= 0;
			2,
			3: nb_lanes <= 1;
			4,
			5,
			6,
			7: nb_lanes <= 2;
			8: nb_lanes <= 3;
		endcase
		clk <= 0;
		#5;

		repeat_loop((VLEN >> `min(6, LANE_WIDTH))>>nb_lanes, 1);
		`assert(done0, 1'b1)
		if (nb_lanes >= 1) `assert(done1, 1'b1)
		if (nb_lanes >= 2) `assert(done2, 1'b1)
		if (nb_lanes >= 2) `assert(done3, 1'b1)
		// `assert(vd, 128'h83450301122416681224166883450301)
		$display("[OK] 64b");



		
	end

	initial begin
		if ($test$plusargs("vcd")) begin
			$dumpfile("testbench_vec_alu.vcd");
			$dumpvars(0, testbench);
		end
		repeat (1000000) @(posedge clk);
		$display("TIMEOUT");
		$finish;
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
		.vs1(vs1),
		.vs2(vs2),
		.vsew(vsew),
		.op_type(OP_TYPE),
		.vd(vd0),
		.reg_index(regi0),
		.done(done0)
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
		.vs1(vs1),
		.vs2(vs2),
		.vsew(vsew),
		.op_type(OP_TYPE),
		.vd(vd1),
		.reg_index(regi1),
		.done(done1)
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
		.vs1(vs1),
		.vs2(vs2),
		.vsew(vsew),
		.op_type(OP_TYPE),
		.vd(vd2),
		.reg_index(regi2),
		.done(done2)
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
		.vs1(vs1),
		.vs2(vs2),
		.vsew(vsew),
		.op_type(OP_TYPE),
		.vd(vd3),
		.reg_index(regi3),
		.done(done3)
	);
endmodule
`endif
