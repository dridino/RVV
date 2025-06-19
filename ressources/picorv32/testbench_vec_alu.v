`timescale 1 ns / 1 ps

`ifndef VERILATOR

`define assert(signal, value) \
        if (signal !== value) begin \
            $display("\n[ERROR] ASSERTION FAILED in %m: signal != value\n"); \
            $finish; \
        end

`define min(a,b) (a < b ? a : b)

module testbench ();
	localparam [2:0] LANE_WIDTH = 3'b100;
	localparam [7:0] SHIFTED_LANE_WIDTH = 1 << LANE_WIDTH;
	localparam [1:0] NB_LANES = 2'b01;
	localparam [9:0] VLEN = 10'd128;
	reg clk = 0;
	reg resetn = 0;
	wire trap;

	wire trace_valid;
	wire [35:0] trace_data;
	integer trace_file;

	reg [5:0] opcode;
	reg	run;
	reg [127:0] vs1;
	reg [127:0] vs2;
	reg [2:0] vsew;

	reg [127:0] vd;
	wire [127:0] vd0, vd1;
	wire done0, done1;
	wire [9:0] regi0,regi1;

	integer index;


	task repeat_loop;
		input integer n;
		input print;
		begin
			vd <= 0;
			index = `min(vsew+3, LANE_WIDTH);
			run <= 1;
			repeat (n-1) begin
				run <= 1;
				clk <= 1;
				#5;
				clk <= 0;
				#5;
				
				`assert(done0, 1'b0)
				`assert(done1, 1'b0)

				if (vsew + 3 > SHIFTED_LANE_WIDTH) begin // impossible for 8b
					case (vsew)
						3'b001: begin
							vd[regi0 +: 16] = vd0[regi0 +: 16];
							if (LANE_WIDTH >= 1) vd[regi1 +: 16] = vd1[regi1 +: 16];
						end
						3'b010: begin
							vd[regi0 +: 32] = vd0[regi0 +: 32];
							if (LANE_WIDTH >= 1) vd[regi1 +: 32] = vd1[regi1 +: 32];
						end
						3'b011: begin
							vd[regi0 +: 64] = vd0[regi0 +: 64];
							if (LANE_WIDTH >= 1) vd[regi1 +: 64] = vd1[regi1 +: 64];
						end
					endcase
				end else begin
					vd[regi0 +: SHIFTED_LANE_WIDTH] = vd0[regi0 +: SHIFTED_LANE_WIDTH];
					if (LANE_WIDTH >= 1) vd[regi1 +: SHIFTED_LANE_WIDTH] = vd1[regi1 +: SHIFTED_LANE_WIDTH];
				end
				
				if (print) $display("index1 : %d", regi0);
				if (print) $display("index2 : %d", regi1);
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
						if (LANE_WIDTH >= 1) vd[regi1 +: 16] = vd1[regi1 +: 16];
					end
					3'b010: begin
						vd[regi0 +: 32] = vd0[regi0 +: 32];
						if (LANE_WIDTH >= 1) vd[regi1 +: 32] = vd1[regi1 +: 32];
					end
					3'b011: begin
						vd[regi0 +: 64] = vd0[regi0 +: 64];
						if (LANE_WIDTH >= 1) vd[regi1 +: 64] = vd1[regi1 +: 64];
					end
				endcase
			end else begin
				vd[regi0 +: SHIFTED_LANE_WIDTH] = vd0[regi0 +: SHIFTED_LANE_WIDTH];
				if (LANE_WIDTH >= 1) vd[regi1 +: SHIFTED_LANE_WIDTH] = vd1[regi1 +: SHIFTED_LANE_WIDTH];
			end
			if (print) $display("index1 : %d", regi0);
			if (print) $display("index2 : %d", regi1);
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
		opcode <= 6'b001001;
		vs1 <= 128'habcdabcdbeefbeef1234567887654321;
		vs2 <= 128'h8765432112345678beefbeefabcdabcd;
		
		// -------------------------- 8 BITS --------------------------
		vsew <= 3'b000;
		run <= 0;
		clk <= 1;
		#5;
		clk <= 0;
		#5;
		
		repeat_loop((VLEN >> `min(3, LANE_WIDTH))>>NB_LANES, 0);
		`assert(done0, 1'b1)
		`assert(done1, 1'b1)
		`assert(vd, 128'h83450301122416681224166883450301)
		$display("[OK] 8b");

		// -------------------------- 16 BITS --------------------------
		vsew <= 3'b001;
		run <= 0;
		clk <= 1;
		#5;
		clk <= 0;
		#5;
		
		repeat_loop((VLEN >> `min(4, LANE_WIDTH))>>NB_LANES, 0);
		`assert(done0, 1'b1)
		`assert(done1, 1'b1)
		`assert(vd, 128'h83450301122416681224166883450301)
		$display("[OK] 16b");

		// -------------------------- 32 BITS --------------------------
		vsew <= 3'b010;
		run <= 0;
		clk <= 1;
		#5;
		clk <= 0;
		#5;
		
		repeat_loop((VLEN >> `min(5, LANE_WIDTH))>>NB_LANES, 0);
		`assert(done0, 1'b1)
		`assert(done1, 1'b1)
		`assert(vd, 128'h83450301122416681224166883450301)
		$display("[OK] 32b");
		
		// -------------------------- 64 BITS --------------------------
		vsew <= 3'b011;
		run <= 0;
		clk <= 1;
		#5;
		clk <= 0;
		#5;
		
		repeat_loop((VLEN >> `min(6, LANE_WIDTH))>>NB_LANES, 0);
		`assert(done0, 1'b1)
		`assert(done1, 1'b1)
		`assert(vd, 128'h83450301122416681224166883450301)
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
		.NB_LANES (NB_LANES),
		.LANE_I (3'b000)
	) valu0 (
		.clk(clk),
		.resetn(resetn),
		.opcode(opcode),
		.run(run),
		.vs1(vs1),
		.vs2(vs2),
		.vsew(vsew),
		.vd(vd0),
		.reg_index(regi0),
		.done(done0)
	);
	
	vec_alu #(
		.VLEN (VLEN),
		.LANE_WIDTH (LANE_WIDTH),
		.NB_LANES (NB_LANES),
		.LANE_I (3'b001)
	) valu1 (
		.clk(clk),
		.resetn(resetn),
		.opcode(opcode),
		.run(run),
		.vs1(vs1),
		.vs2(vs2),
		.vsew(vsew),
		.vd(vd1),
		.reg_index(regi1),
		.done(done1)
	);
endmodule
`endif
