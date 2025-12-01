`timescale 1 ns / 1 ps

`ifndef VERILATOR

`define assert(signal, value) \
        if (signal !== value) begin \
            $display("\n[ERROR] ASSERTION FAILED in %m: signal != value\n"); \
            $finish; \
        end

`define min(a,b) (a < b ? a : b)


module testbench ();
	localparam [2:0] LANE_WIDTH = 3'b011;
	localparam [7:0] SHIFTED_LANE_WIDTH = 1 << LANE_WIDTH;
	localparam integer NB_LANES = 2'b01;
	localparam [16:0] VLEN = 17'd128;
	localparam [4:0] IMM = 5'b00001;
	localparam [31:0] RS1 = 32'h00000001;
	
	localparam [2:0] OP_TYPE = 3'b001;
	localparam [2:0] VV = 3'b001;
	localparam [2:0] VX = 3'b010;
	localparam [2:0] VI = 3'b100;

	localparam [5:0] VAND = 6'b001001;
	localparam [5:0] VOR = 6'b001010;
	localparam [5:0] VXOR = 6'b001011;
	localparam [5:0] VADD = 6'b000000;

	reg clk = 0;
	reg resetn = 0;
	wire trap;

	wire trace_valid;
	wire [35:0] trace_data;
	integer trace_file;

	reg [5:0] opcode;
	reg run;
	reg [127:0] vs1;
	reg [127:0] vs2;
	reg [2:0] vsew;

	reg [127:0] vd_res;
	wire [255:0] vd;
	wire t1,t2;
	wire done;
	wire [39:0] regi;
	wire [3:0] res;

	reg [1:0] nb_lanes;


	task repeat_loop;
		input integer n;
		input print;
		begin
			vd_res <= 0;
			run <= 1;
			clk <= 1;
			#5;
			repeat (n-1) begin				
				`assert(done, 1'b0)
				$display("vd_alu : %h", vd);
				
				if (vsew + 3 <= LANE_WIDTH) begin // lane larger than vsew
					case (vsew)
						3'b000: begin
							if (res[0]) vd_res[regi[0 +:10] +: 8] = vd[0   +: 8];
							if (res[1]) vd_res[regi[10+:10] +: 8] = vd[64  +: 8];
							if (res[2]) vd_res[regi[20+:10] +: 8] = vd[128 +: 8];
							if (res[3]) vd_res[regi[30+:10] +: 8] = vd[196 +: 8];
						end
						3'b001: begin
							if (res[0]) vd_res[regi[0 +:10] +: 16] = vd[0   +: 16];
							if (res[1]) vd_res[regi[10+:10] +: 16] = vd[64  +: 16];
							if (res[2]) vd_res[regi[20+:10] +: 16] = vd[128 +: 16];
							if (res[3]) vd_res[regi[30+:10] +: 16] = vd[196 +: 16];
						end
						3'b010: begin
							if (res[0]) vd_res[regi[0 +:10] +: 32] = vd[0   +: 32];
							if (res[1]) vd_res[regi[10+:10] +: 32] = vd[64  +: 32];
							if (res[2]) vd_res[regi[20+:10] +: 32] = vd[128 +: 32];
							if (res[3]) vd_res[regi[30+:10] +: 32] = vd[196 +: 32];
						end
						3'b011: begin
							if (res[0]) vd_res[regi[0 +:10] +: 64] = vd[0   +: 64];
							if (res[1]) vd_res[regi[10+:10] +: 64] = vd[64  +: 64];
							if (res[2]) vd_res[regi[20+:10] +: 64] = vd[128 +: 64];
							if (res[3]) vd_res[regi[30+:10] +: 64] = vd[196 +: 64];
						end
					endcase
				end else begin
					if (res[0]) vd_res[regi[0 +:10] +: SHIFTED_LANE_WIDTH] = vd[0   +: SHIFTED_LANE_WIDTH];
					if (res[1]) vd_res[regi[10+:10] +: SHIFTED_LANE_WIDTH] = vd[64  +: SHIFTED_LANE_WIDTH];
					if (res[2]) vd_res[regi[20+:10] +: SHIFTED_LANE_WIDTH] = vd[128 +: SHIFTED_LANE_WIDTH];
					if (res[3]) vd_res[regi[30+:10] +: SHIFTED_LANE_WIDTH] = vd[196 +: SHIFTED_LANE_WIDTH];
				end
				
				if (res[0] && print) $display("index1 : %d", regi[0 +:10]);
				if (res[1] && print) $display("index2 : %d", regi[10+:10]);
				if (res[2] && print) $display("index3 : %d", regi[20+:10]);
				if (res[3] && print) $display("index4 : %d", regi[30+:10]);
				if (print) $display("vd : %h", vd_res);
				
				clk <= 0;
				#5;
				clk <= 1;
				#5;
			end
			clk <= 1;
			#5;
			clk <= 0;
			#5;
			if (vsew + 3 <= LANE_WIDTH) begin // lane larger than vsew
				case (vsew)
					3'b000: begin
						if (res[0]) vd_res[regi[0 +:10] +: 8] = vd[0   +: 8];
						if (res[1]) vd_res[regi[10+:10] +: 8] = vd[64  +: 8];
						if (res[2]) vd_res[regi[20+:10] +: 8] = vd[128 +: 8];
						if (res[3]) vd_res[regi[30+:10] +: 8] = vd[196 +: 8];
					end
					3'b001: begin
						if (res[0]) vd_res[regi[0 +:10] +: 16] = vd[0   +: 16];
						if (res[1]) vd_res[regi[10+:10] +: 16] = vd[64  +: 16];
						if (res[2]) vd_res[regi[20+:10] +: 16] = vd[128 +: 16];
						if (res[3]) vd_res[regi[30+:10] +: 16] = vd[196 +: 16];
					end
					3'b010: begin
						if (res[0]) vd_res[regi[0 +:10] +: 32] = vd[0   +: 32];
						if (res[1]) vd_res[regi[10+:10] +: 32] = vd[64  +: 32];
						if (res[2]) vd_res[regi[20+:10] +: 32] = vd[128 +: 32];
						if (res[3]) vd_res[regi[30+:10] +: 32] = vd[196 +: 32];
					end
					3'b011: begin
						if (res[0]) vd_res[regi[0 +:10] +: 64] = vd[0   +: 64];
						if (res[1]) vd_res[regi[10+:10] +: 64] = vd[64  +: 64];
						if (res[2]) vd_res[regi[20+:10] +: 64] = vd[128 +: 64];
						if (res[3]) vd_res[regi[30+:10] +: 64] = vd[196 +: 64];
					end
				endcase
			end else begin
				if (res[0]) vd_res[regi[0 +:10] +: SHIFTED_LANE_WIDTH] = vd[0   +: SHIFTED_LANE_WIDTH];
				if (res[1]) vd_res[regi[10+:10] +: SHIFTED_LANE_WIDTH] = vd[64  +: SHIFTED_LANE_WIDTH];
				if (res[2]) vd_res[regi[20+:10] +: SHIFTED_LANE_WIDTH] = vd[128 +: SHIFTED_LANE_WIDTH];
				if (res[3]) vd_res[regi[30+:10] +: SHIFTED_LANE_WIDTH] = vd[196 +: SHIFTED_LANE_WIDTH];
			end
			if (res[0] && print) $display("index1 : %d", regi[0 +:10]);
			if (res[1] && print) $display("index2 : %d", regi[10+:10]);
			if (res[2] && print) $display("index3 : %d", regi[20+:10]);
			if (res[3] && print) $display("index4 : %d", regi[30+:10]);
			if (print) $display("vd : %h", vd_res);
		end
	endtask

	initial begin
		$display("+---------------------------------+\n| Test with %2d LANE(S) of %2d bits |\n+---------------------------------+", 1 << NB_LANES, 1 << LANE_WIDTH);

		resetn <= 0;
		run <= 0;

		repeat (200) begin
			clk <= ~clk;
			#5;
		end
		resetn <= 1;
		opcode <= VADD;
		case (OP_TYPE)
			VV: vs1 <= 128'habcdabcdbeefbeef1234567887654321;
			VX: vs1 <= {{96{RS1[31]}}, RS1};
			VI: vs1 <= {{123{IMM[4]}}, IMM};
		endcase
		vs2 <= 128'h8765432112345678beefbeefabcdabcd;
		
		// -------------------------- 8 BITS --------------------------
		vsew <= 3'b000;
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
		
		repeat_loop(((VLEN >> `min(3, LANE_WIDTH))>>nb_lanes), 1);
		`assert(done, 1'b1)
		// `assert(vd, 128'h83450301122416681224166883450301)
		// add.vv
		// 3232eeeed0231467d02314673232eeee
		$display("[OK] 8b");

		// -------------------------- 16 BITS --------------------------
		vsew <= 3'b001;
		run <= 0;
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
		`assert(done, 1'b1)
		// `assert(vd, 128'h83450301122416681224166883450301)
		// add.vv
		// 3332eeeed1231567d12315673332eeee
		$display("[OK] 16b");

		// -------------------------- 32 BITS --------------------------
		vsew <= 3'b010;
		run <= 0;
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
		`assert(done, 1'b1)
		// `assert(vd, 128'h83450301122416681224166883450301)
		// add.vv
		// 3332eeeed1241567d12415673332eeee
		$display("[OK] 32b");
		
		// -------------------------- 64 BITS --------------------------
		vsew <= 3'b011;
		run <= 0;
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
		`assert(done, 1'b1)
		// `assert(vd, 128'h83450301122416681224166883450301)
		// add.vv
		// 3332eeeed1241567d12415683332eeee
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

	rvv_alu_wrapper #(
		.VLEN(VLEN),
		.LANE_WIDTH(LANE_WIDTH),
		.NB_LANES(NB_LANES)
	) valu_wrapper (
		.clk(clk),
		.resetn(resetn),
		.opcode(opcode),
		.run(run),
		.vs1(vs1),
		.vs2(vs2),
		.vsew(vsew),
		.op_type(OP_TYPE),
		.vd(vd),
		.regi(regi),
		.res(res),
		.done_out(done)
	);
endmodule
`endif
