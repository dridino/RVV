`timescale 1 ns / 1 ps

`ifndef VERILATOR

`define assert(signal, value) \
        if (signal !== value) begin \
            $display("\n[ERROR] ASSERTION FAILED in %m: signal != value\n"); \
            $finish; \
        end

module testbench ();
	reg clk = 0;
	reg resetn = 0;
	wire trap;

	task repeat_loop;
		input integer n;
		input print;
		begin
			repeat (n-1) begin
				run <= 1;
				clk <= 1;
				#5;
				clk <= 0;
				#5;
				if (print) $display("vd : %h", vd);
				`assert(done, 1'b0)
			end
			clk <= 1;
			#5;
			clk <= 0;
			#5;
			if (print) $display("vd : %h", vd);
			$display("[OK]");
		end

	endtask

	initial begin
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

		repeat_loop(16, 0);
		`assert(done, 1'b1)
		`assert(vd, 128'h83450301122416681224166883450301)

		// -------------------------- 16 BITS --------------------------
		vsew <= 3'b001;
		run <= 0;
		clk <= 1;
		#5;
		clk <= 0;
		#5;
		
		repeat_loop(8, 0);
		`assert(done, 1'b1)
		`assert(vd, 128'h83450301122416681224166883450301)

		// -------------------------- 32 BITS --------------------------
		vsew <= 3'b010;
		run <= 0;
		clk <= 1;
		#5;
		clk <= 0;
		#5;
		
		repeat_loop(4, 0);
		`assert(done, 1'b1)
		`assert(vd, 128'h83450301122416681224166883450301)
		
		// -------------------------- 64 BITS --------------------------
		vsew <= 3'b011;
		run <= 0;
		clk <= 1;
		#5;
		clk <= 0;
		#5;
		
		repeat_loop(4, 0);
		`assert(done, 1'b1)
		`assert(vd, 128'h83450301122416681224166883450301)



		
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

	wire trace_valid;
	wire [35:0] trace_data;
	integer trace_file;

	reg [5:0] opcode;
	reg	run;
	reg [127:0] vs1;
	reg [127:0] vs2;
	reg [2:0] vsew;

	wire [127:0] vd;
	wire done;

	vec_alu #(
		.NB_LANES (2'b00),
		.LANE_I (3'b000)
	) valu0 (
		.clk(clk),
		.resetn(resetn),
		.opcode(opcode),
		.run(run),
		.vs1(vs1),
		.vs2(vs2),
		.vsew(vsew),
		.vd(vd),
		.done(done)
	);
endmodule
`endif
