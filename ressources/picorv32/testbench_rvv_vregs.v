`timescale 1 ns / 1 ps

`ifndef VERILATOR

`define assert(signal, value) \
        if (signal !== value) begin \
            $display("\n[ERROR] ASSERTION FAILED in %m: signal != value\n"); \
            $finish; \
        end

`define min(a,b) (a < b ? a : b)

module rvv_vregs #(
	parameter [16:0] VLEN = 17'd 128,
	parameter REGS_INIT_ZERO = 1'b1
) (
	input clk,
	input [(VLEN>>3)-1:0] wstrb,
	input [4:0] waddr,
	input [4:0] raddr1,
	input [4:0] raddr2,
	input [VLEN-1:0] wdata,
	output [VLEN-1:0] rdata1,
	output [VLEN-1:0] rdata2
);
	localparam integer VLENB = VLEN >> 3;
	reg [VLEN-1:0] vregs [31:0];

	integer init_reg_i;
	initial begin
		if (REGS_INIT_ZERO) begin
			for (init_reg_i = 0; init_reg_i < 32; init_reg_i = init_reg_i+1)
				vregs[init_reg_i] = 0;
		end
	end

	integer i;
	always @(posedge clk)
		for (i=0; i<VLENB; i+=1)
			if (wstrb[i]) vregs[waddr][i<<3 +: 8] <= wdata[i<<3 +: 8];
	
	assign rdata1 = vregs[raddr1];
	assign rdata2 = vregs[raddr2];
endmodule

module testbench ();
	localparam [16:0] VLEN = 17'd32;
	localparam integer VLENB = VLEN >> 3;

	reg clk = 0;
	reg resetn = 0;

	
	reg [4:0] waddr, raddr1, raddr2;
	reg [VLENB-1:0] wstrb;
	reg [VLEN-1:0] wdata;
	wire [VLEN-1:0] rdata1, rdata2;

	rvv_vregs #(
		.VLEN(VLEN),
		.REGS_INIT_ZERO(1)
	) vregs (
		.clk(clk),
		.waddr(waddr),
		.wstrb(wstrb),
		.raddr1(raddr1),
		.raddr2(raddr2),
		.wdata(wdata),
		.rdata1(rdata1),
		.rdata2(rdata2)
	);

	initial begin
		resetn <= 0;
		raddr1 <= 0; raddr2 <= 0; waddr <= 0; wstrb <= 0; wdata <= 0;

		repeat (200) begin
			clk <= ~clk;
			#5;
		end

		resetn <= 1;
		clk <= 1;
		raddr1 <= 5'h02;
		raddr2 <= 5'h03;
		#5;
		clk <= 0;
		#5;

		clk <= 1;
		raddr1 <= 5'h04;
		raddr2 <= 5'h05;
		#5;
		clk <= 0;
		#5;

		clk <= 1;
		waddr <= 5'h04;
		wstrb <= 4'hf;
		wdata <= 32'h01234567;
		#5;
		clk <= 0;
		#5

		clk <= 1;
		#5;
		clk <= 0;
		#5;

		$display("end of test");
	end

	initial begin
		if ($test$plusargs("vcd")) begin
			$dumpfile("testbench_vec_vregs.vcd");
			$dumpvars(0, testbench);
		end
		repeat (1000000) @(posedge clk);
		$display("TIMEOUT");
		$finish;
	end
endmodule
`endif
