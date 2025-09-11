module top (
	input clk,
	output reg [9:0] leds
	/*,
	output trap,
	output trace_valid,
	output [35:0] trace_data */
);
	localparam AXI_TEST = 0;
	localparam VERBOSE = 0;

	wire tests_passed = mem_axi_awaddr == 32'h20000000 && mem_axi_wdata == 12345678;
	wire trap, trace_valid;
	wire [35:0] trace_data;
	
	reg [31:0] irq = 0;

	reg [15:0] count_cycle = 0;
	always @(posedge clk) count_cycle <= resetn ? count_cycle + 1 : 0;

	always @* begin
		irq = 0;
		irq[4] = &count_cycle[12:0];
		irq[5] = &count_cycle[15:0];
	end
	
	integer reset_cpt = 0;
	reg resetn = 0;
	always @(posedge clk) begin
		if (reset_cpt == 500_000_000) // 10 secs
			resetn <= 1;
		else begin
			reset_cpt <= reset_cpt + 1;
			resetn <= 0;
		end
	end

	wire        mem_axi_awvalid;
	wire        mem_axi_awready;
	wire [31:0] mem_axi_awaddr;
	wire [ 2:0] mem_axi_awprot;

	wire        mem_axi_wvalid;
	wire        mem_axi_wready;
	wire [31:0] mem_axi_wdata;
	wire [ 3:0] mem_axi_wstrb;

	wire        mem_axi_bvalid;
	wire        mem_axi_bready;

	wire        mem_axi_arvalid;
	wire        mem_axi_arready;
	wire [31:0] mem_axi_araddr;
	wire [ 2:0] mem_axi_arprot;

	wire        mem_axi_rvalid;
	wire        mem_axi_rready;
	wire [31:0] mem_axi_rdata;
	
	// RAM interface
	wire [31:0] ram_addr;
	wire [31:0] ram_data;
	wire 			ram_wren;
	wire [31:0] ram_q = {ram3_q, ram2_q, ram1_q, ram0_q};
	wire [3:0]  ram_wstrb;
	
	// RAM blocks interface
	wire [7:0] ram0_q, ram1_q, ram2_q, ram3_q;
	wire [14:0] shifted_ram_addr = {2'b00, ram_addr[14:2]};
	wire ram0_wren = ram_wren && ram_wstrb[0];
	wire ram1_wren = ram_wren && ram_wstrb[1];
	wire ram2_wren = ram_wren && ram_wstrb[2];
	wire ram3_wren = ram_wren && ram_wstrb[3];
	
	reg err;
	
	
	axi_to_ram axi2ram (
		 .clk(clk),
		 .resetn(resetn),
		 
		 .s_axi_awvalid(mem_axi_awvalid),
		 .s_axi_awready(mem_axi_awready),
		 .s_axi_awaddr(mem_axi_awaddr),
		 .s_axi_awprot(mem_axi_awprot),
		 
		 .s_axi_wvalid(mem_axi_wvalid),
		 .s_axi_wready(mem_axi_wready),
		 .s_axi_wdata(mem_axi_wdata),
		 .s_axi_wstrb(mem_axi_wstrb),
		 
		 .s_axi_bvalid(mem_axi_bvalid),
		 .s_axi_bready(mem_axi_bready),
		 
		 .s_axi_arvalid(mem_axi_arvalid),
		 .s_axi_arready(mem_axi_arready),
		 .s_axi_araddr(mem_axi_araddr),
		 .s_axi_arprot(mem_axi_arprot),
		 
		 .s_axi_rvalid(mem_axi_rvalid),
		 .s_axi_rready(mem_axi_rready),
		 .s_axi_rdata(mem_axi_rdata),
		 
		 .ram_address(ram_addr),
		 .ram_data(ram_data),
		 .ram_wren(ram_wren),
		 .ram_wstrb(ram_wstrb),
		 .ram_q(ram_q)
	);
	
	// first byte
	ram0_ip ram0 (
		.clock(clk),
		.address(shifted_ram_addr),
		.data(ram_data[7:0]),
		.wren(ram0_wren),
		.q(ram0_q)
	);
	
	// second byte
	ram1_ip ram1 (
		.clock(clk),
		.address(shifted_ram_addr),
		.data(ram_data[15:8]),
		.wren(ram1_wren),
		.q(ram1_q)
	);
	
	// third byte
	ram2_ip ram2 (
		.clock(clk),
		.address(shifted_ram_addr),
		.data(ram_data[23:16]),
		.wren(ram2_wren),
		.q(ram2_q)
	);
	
	// fourth byte
	ram3_ip ram3 (
		.clock(clk),
		.address(shifted_ram_addr),
		.data(ram_data[31:24]),
		.wren(ram3_wren),
		.q(ram3_q)
	);

	
	

	/* axi4_memory #(
		.AXI_TEST (AXI_TEST),
		.VERBOSE  (VERBOSE)
	) mem (
		.clk             (clk             ),
		.mem_axi_awvalid (mem_axi_awvalid ),
		.mem_axi_awready (mem_axi_awready ),
		.mem_axi_awaddr  (mem_axi_awaddr  ),
		.mem_axi_awprot  (mem_axi_awprot  ),

		.mem_axi_wvalid  (mem_axi_wvalid  ),
		.mem_axi_wready  (mem_axi_wready  ),
		.mem_axi_wdata   (mem_axi_wdata   ),
		.mem_axi_wstrb   (mem_axi_wstrb   ),

		.mem_axi_bvalid  (mem_axi_bvalid  ),
		.mem_axi_bready  (mem_axi_bready  ),

		.mem_axi_arvalid (mem_axi_arvalid ),
		.mem_axi_arready (mem_axi_arready ),
		.mem_axi_araddr  (mem_axi_araddr  ),
		.mem_axi_arprot  (mem_axi_arprot  ),

		.mem_axi_rvalid  (mem_axi_rvalid  ),
		.mem_axi_rready  (mem_axi_rready  ),
		.mem_axi_rdata   (mem_axi_rdata   ),

		.tests_passed    (tests_passed    )
	); */

/* `ifdef RISCV_FORMAL
	wire        rvfi_valid;
	wire [63:0] rvfi_order;
	wire [31:0] rvfi_insn;
	wire        rvfi_trap;
	wire        rvfi_halt;
	wire        rvfi_intr;
	wire [4:0]  rvfi_rs1_addr;
	wire [4:0]  rvfi_rs2_addr;
	wire [31:0] rvfi_rs1_rdata;
	wire [31:0] rvfi_rs2_rdata;
	wire [4:0]  rvfi_rd_addr;
	wire [31:0] rvfi_rd_wdata;
	wire [31:0] rvfi_pc_rdata;
	wire [31:0] rvfi_pc_wdata;
	wire [31:0] rvfi_mem_addr;
	wire [3:0]  rvfi_mem_rmask;
	wire [3:0]  rvfi_mem_wmask;
	wire [31:0] rvfi_mem_rdata;
	wire [31:0] rvfi_mem_wdata;
`endif */

	picorv32_axi #(
`ifndef SYNTH_TEST
`ifdef SP_TEST
		.ENABLE_REGS_DUALPORT(0),
`endif
`ifdef COMPRESSED_ISA
		.COMPRESSED_ISA(1),
`endif
		.ENABLE_MUL(0),
		.ENABLE_DIV(0),
		.ENABLE_IRQ(0),
		.REGS_INIT_ZERO(1),
		.ENABLE_TRACE(1),
		// RVV
		.ENABLE_RVV(1),
		.NB_LANES(0),
		.LANE_WIDTH(3'b011),
		.VLEN(128)
`endif
	) uut (
		.clk            (clk            ),
		.resetn         (resetn         ),
		.trap           (trap           ),
		.mem_axi_awvalid(mem_axi_awvalid),
		.mem_axi_awready(mem_axi_awready),
		.mem_axi_awaddr (mem_axi_awaddr ),
		.mem_axi_awprot (mem_axi_awprot ),
		.mem_axi_wvalid (mem_axi_wvalid ),
		.mem_axi_wready (mem_axi_wready ),
		.mem_axi_wdata  (mem_axi_wdata  ),
		.mem_axi_wstrb  (mem_axi_wstrb  ),
		.mem_axi_bvalid (mem_axi_bvalid ),
		.mem_axi_bready (mem_axi_bready ),
		.mem_axi_arvalid(mem_axi_arvalid),
		.mem_axi_arready(mem_axi_arready),
		.mem_axi_araddr (mem_axi_araddr ),
		.mem_axi_arprot (mem_axi_arprot ),
		.mem_axi_rvalid (mem_axi_rvalid ),
		.mem_axi_rready (mem_axi_rready ),
		.mem_axi_rdata  (mem_axi_rdata  ),
		.irq            (irq            ), /*
`ifdef RISCV_FORMAL
		.rvfi_valid     (rvfi_valid     ),
		.rvfi_order     (rvfi_order     ),
		.rvfi_insn      (rvfi_insn      ),
		.rvfi_trap      (rvfi_trap      ),
		.rvfi_halt      (rvfi_halt      ),
		.rvfi_intr      (rvfi_intr      ),
		.rvfi_rs1_addr  (rvfi_rs1_addr  ),
		.rvfi_rs2_addr  (rvfi_rs2_addr  ),
		.rvfi_rs1_rdata (rvfi_rs1_rdata ),
		.rvfi_rs2_rdata (rvfi_rs2_rdata ),
		.rvfi_rd_addr   (rvfi_rd_addr   ),
		.rvfi_rd_wdata  (rvfi_rd_wdata  ),
		.rvfi_pc_rdata  (rvfi_pc_rdata  ),
		.rvfi_pc_wdata  (rvfi_pc_wdata  ),
		.rvfi_mem_addr  (rvfi_mem_addr  ),
		.rvfi_mem_rmask (rvfi_mem_rmask ),
		.rvfi_mem_wmask (rvfi_mem_wmask ),
		.rvfi_mem_rdata (rvfi_mem_rdata ),
		.rvfi_mem_wdata (rvfi_mem_wdata ),
`endif */
		.trace_valid    (trace_valid    ),
		.trace_data     (trace_data     )
	);

/* `ifdef RISCV_FORMAL
	picorv32_rvfimon rvfi_monitor (
		.clock          (clk           ),
		.reset          (!resetn       ),
		.rvfi_valid     (rvfi_valid    ),
		.rvfi_order     (rvfi_order    ),
		.rvfi_insn      (rvfi_insn     ),
		.rvfi_trap      (rvfi_trap     ),
		.rvfi_halt      (rvfi_halt     ),
		.rvfi_intr      (rvfi_intr     ),
		.rvfi_rs1_addr  (rvfi_rs1_addr ),
		.rvfi_rs2_addr  (rvfi_rs2_addr ),
		.rvfi_rs1_rdata (rvfi_rs1_rdata),
		.rvfi_rs2_rdata (rvfi_rs2_rdata),
		.rvfi_rd_addr   (rvfi_rd_addr  ),
		.rvfi_rd_wdata  (rvfi_rd_wdata ),
		.rvfi_pc_rdata  (rvfi_pc_rdata ),
		.rvfi_pc_wdata  (rvfi_pc_wdata ),
		.rvfi_mem_addr  (rvfi_mem_addr ),
		.rvfi_mem_rmask (rvfi_mem_rmask),
		.rvfi_mem_wmask (rvfi_mem_wmask),
		.rvfi_mem_rdata (rvfi_mem_rdata),
		.rvfi_mem_wdata (rvfi_mem_wdata)
	);
`endif */

	/* reg [1023:0] firmware_file;
	initial begin
		if (!$value$plusargs("firmware=%s", firmware_file))
			firmware_file = "firmware/firmware.hex";
		$readmemh(firmware_file, mem.memory);
	end */

	// always @(posedge clk)
	// 	$display("%h | %h", mem_axi_araddr, mem_axi_rdata);

	// integer cycle_counter;
	always @(posedge clk) begin
		leds <= 0;
		if (!resetn) err <= 0;
		// cycle_counter <= resetn ? cycle_counter + 1 : 0;
		if (resetn && (ram_q == 32'hE0E0E0E0 || err)) begin
			leds <= 10'b1111111111;
			err <= 1;
		end else if (!err) leds <= resetn ? 10'b0011001100 : 10'b0000000001;
	end
endmodule


module axi_to_ram (
    input         clk,
    input         resetn,

    // AXI4-Lite Interface
    input         s_axi_awvalid,
    output reg    s_axi_awready,
    input  [31:0] s_axi_awaddr,
    input  [2:0]  s_axi_awprot,

    input         s_axi_wvalid,
    output reg    s_axi_wready,
    input  [31:0] s_axi_wdata,
    input  [3:0]  s_axi_wstrb,

    output  reg   s_axi_bvalid,
    input         s_axi_bready,

    input         s_axi_arvalid,
    output reg    s_axi_arready,
    input  [31:0] s_axi_araddr,
    input  [2:0]  s_axi_arprot,

    output  reg   s_axi_rvalid,
    input         s_axi_rready,
    output [31:0] s_axi_rdata,

    // RAM interface
    output [31:0] ram_address,
    output        ram_wren,
    output [31:0] ram_data,
	 output [3:0]  ram_wstrb,
    input  [31:0] ram_q
);
    // Internal state
    reg         aw_ready, w_ready, b_valid;
    reg         ar_ready, r_valid, r_valid_q;

    /* assign s_axi_awready = !aw_ready && s_axi_awvalid;
    assign s_axi_wready  = !w_ready && s_axi_wvalid;
    assign s_axi_bvalid  = b_valid;
    assign s_axi_arready = !ar_ready && s_axi_arvalid;
    assign s_axi_rvalid  = r_valid; */
    assign s_axi_rdata   = ram_q;

    assign ram_address = (s_axi_awvalid ? s_axi_awaddr : s_axi_araddr);
    assign ram_data    = s_axi_wdata;
    assign ram_wren    = s_axi_awvalid && s_axi_wvalid;
	assign ram_wstrb   = s_axi_wstrb;
	 
	always @(negedge clk) begin
		if (!resetn) begin
			s_axi_awready <= 0;
			s_axi_wready <= 0;
			s_axi_bvalid <= 0;
			s_axi_arready <= 0;
			s_axi_rvalid <= 0;
		end else begin
			s_axi_awready <= !aw_ready && s_axi_awvalid;
			s_axi_wready  <= !w_ready && s_axi_wvalid;
			s_axi_bvalid  <= b_valid;
			s_axi_arready <= !ar_ready && s_axi_arvalid;
			s_axi_rvalid  <= r_valid;
		end
	end

	always @(posedge clk) begin
		if (!resetn) begin
			s_axi_awready <= 0;
			s_axi_wready <= 0;
			s_axi_bvalid <= 0;
			s_axi_arready <= 0;
			s_axi_rvalid <= 0;
		end else begin
			s_axi_awready <= !aw_ready && s_axi_awvalid;
			s_axi_wready  <= !w_ready && s_axi_wvalid;
			s_axi_bvalid  <= b_valid;
			s_axi_arready <= !ar_ready && s_axi_arvalid;
			s_axi_rvalid  <= r_valid;
		end
	end

    always @(negedge clk) begin
        if (!resetn) begin
            aw_ready <= 0;
            w_ready  <= 0;
            b_valid  <= 0;
            ar_ready <= 0;
            r_valid  <= 0;
            // rdata    <= 0;
        end else begin
            // WRITE transaction
            if (s_axi_awvalid && s_axi_wvalid && !b_valid) begin
                aw_ready <= 1;
                w_ready  <= 1;
                b_valid  <= 1;  // Write complete immediately
            end else if (s_axi_bready && b_valid) begin
                b_valid  <= 0;
                aw_ready <= 0;
                w_ready  <= 0;
            end

            // READ transaction
            if (s_axi_arvalid && !r_valid) begin
                ar_ready <= 1;
                r_valid  <= 1;
            end else if (s_axi_rready && r_valid) begin
                r_valid  <= 0;
                ar_ready <= 0;
            end
        end
    end

	always @(posedge clk) begin
        if (!resetn) begin
            aw_ready <= 0;
            w_ready  <= 0;
            b_valid  <= 0;
            ar_ready <= 0;
            r_valid  <= 0;
            // rdata    <= 0;
        end else begin
            // WRITE transaction
            if (s_axi_awvalid && s_axi_wvalid && !b_valid) begin
                aw_ready <= 1;
                w_ready  <= 1;
                b_valid  <= 1;  // Write complete immediately
            end else if (s_axi_bready && b_valid) begin
                b_valid  <= 0;
                aw_ready <= 0;
                w_ready  <= 0;
            end

            // READ transaction
            if (s_axi_arvalid && !r_valid) begin
                ar_ready <= 1;
                r_valid  <= 1;
            end else if (s_axi_rready && r_valid) begin
                r_valid  <= 0;
                ar_ready <= 0;
            end
        end
    end
endmodule

































module axi4_memory #(
	parameter AXI_TEST = 0,
	parameter VERBOSE = 0
) (
	/* verilator lint_off MULTIDRIVEN */

	input             clk,
	input             mem_axi_awvalid,
	output reg        mem_axi_awready,
	input      [31:0] mem_axi_awaddr,
	input      [ 2:0] mem_axi_awprot,

	input             mem_axi_wvalid,
	output reg        mem_axi_wready,
	input      [31:0] mem_axi_wdata,
	input      [ 3:0] mem_axi_wstrb,

	output reg        mem_axi_bvalid,
	input             mem_axi_bready,

	input             mem_axi_arvalid,
	output reg        mem_axi_arready,
	input      [31:0] mem_axi_araddr,
	input      [ 2:0] mem_axi_arprot,

	output reg        mem_axi_rvalid,
	input             mem_axi_rready,
	output reg [31:0] mem_axi_rdata,

	output reg        tests_passed
);
	reg [31:0]   memory [0:128*1024/4-1] /* verilator public */;
	reg verbose;
	initial verbose = $test$plusargs("verbose") || VERBOSE;

	reg axi_test;
	initial axi_test = $test$plusargs("axi_test") || AXI_TEST;

	initial begin
		mem_axi_awready = 0;
		mem_axi_wready = 0;
		mem_axi_bvalid = 0;
		mem_axi_arready = 0;
		mem_axi_rvalid = 0;
		tests_passed = 0;
	end

	reg [63:0] xorshift64_state = 64'd88172645463325252;

	task xorshift64_next;
		begin
			// see page 4 of Marsaglia, George (July 2003). "Xorshift RNGs". Journal of Statistical Software 8 (14).
			xorshift64_state = xorshift64_state ^ (xorshift64_state << 13);
			xorshift64_state = xorshift64_state ^ (xorshift64_state >>  7);
			xorshift64_state = xorshift64_state ^ (xorshift64_state << 17);
		end
	endtask

	reg [2:0] fast_axi_transaction = ~0;
	reg [4:0] async_axi_transaction = ~0;
	reg [4:0] delay_axi_transaction = 0;

	always @(posedge clk) begin
		if (axi_test) begin
				xorshift64_next;
				{fast_axi_transaction, async_axi_transaction, delay_axi_transaction} <= xorshift64_state;
		end
	end

	reg latched_raddr_en = 0;
	reg latched_waddr_en = 0;
	reg latched_wdata_en = 0;

	reg fast_raddr = 0;
	reg fast_waddr = 0;
	reg fast_wdata = 0;

	reg [31:0] latched_raddr;
	reg [31:0] latched_waddr;
	reg [31:0] latched_wdata;
	reg [ 3:0] latched_wstrb;
	reg        latched_rinsn;

	task handle_axi_arvalid; begin
		mem_axi_arready <= 1;
		latched_raddr = mem_axi_araddr;
		latched_rinsn = mem_axi_arprot[2];
		latched_raddr_en = 1;
		fast_raddr <= 1;
	end endtask

	task handle_axi_awvalid; begin
		mem_axi_awready <= 1;
		latched_waddr = mem_axi_awaddr;
		latched_waddr_en = 1;
		fast_waddr <= 1;
	end endtask

	task handle_axi_wvalid; begin
		mem_axi_wready <= 1;
		latched_wdata = mem_axi_wdata;
		latched_wstrb = mem_axi_wstrb;
		latched_wdata_en = 1;
		fast_wdata <= 1;
	end endtask

	task handle_axi_rvalid; begin
		// if (verbose)
		// 	$display("RD: ADDR=%08x DATA=%08x%s", latched_raddr, memory[latched_raddr >> 2], latched_rinsn ? " INSN" : "");
		if (latched_raddr < 128*1024) begin
			mem_axi_rdata <= memory[latched_raddr >> 2];
			mem_axi_rvalid <= 1;
			latched_raddr_en = 0;
		end else begin
			// $display("OUT-OF-BOUNDS MEMORY READ FROM %08x", latched_raddr);
			// $finish;
		end
	end endtask

	task handle_axi_bvalid; begin
		if (verbose)
			// $display("WR: ADDR=%08x DATA=%08x STRB=%04b", latched_waddr, latched_wdata, latched_wstrb);
		if (latched_waddr < 128*1024) begin
			if (latched_wstrb[0]) memory[latched_waddr >> 2][ 7: 0] <= latched_wdata[ 7: 0];
			if (latched_wstrb[1]) memory[latched_waddr >> 2][15: 8] <= latched_wdata[15: 8];
			if (latched_wstrb[2]) memory[latched_waddr >> 2][23:16] <= latched_wdata[23:16];
			if (latched_wstrb[3]) memory[latched_waddr >> 2][31:24] <= latched_wdata[31:24];
		end else
		if (latched_waddr == 32'h1000_0000) begin
			if (verbose) begin
				// if (32 <= latched_wdata && latched_wdata < 128)
				// 	$display("OUT: '%c'", latched_wdata[7:0]);
				// else
				// 	$display("OUT: %3d", latched_wdata);
			end else begin
				// $write("%c", latched_wdata[7:0]);
// `ifndef VERILATOR
				// $fflush();
// `endif
			end
		end else
		if (latched_waddr == 32'h2000_0000) begin
			if (latched_wdata == 123456789)
				tests_passed = 1;
		end else begin
			// $display("OUT-OF-BOUNDS MEMORY WRITE TO %08x", latched_waddr);
			// $finish;
		end
		mem_axi_bvalid <= 1;
		latched_waddr_en = 0;
		latched_wdata_en = 0;
	end endtask

	always @(negedge clk) begin
		if (mem_axi_arvalid && !(latched_raddr_en || fast_raddr) && async_axi_transaction[0]) handle_axi_arvalid;
		if (mem_axi_awvalid && !(latched_waddr_en || fast_waddr) && async_axi_transaction[1]) handle_axi_awvalid;
		if (mem_axi_wvalid  && !(latched_wdata_en || fast_wdata) && async_axi_transaction[2]) handle_axi_wvalid;
		if (!mem_axi_rvalid && latched_raddr_en && async_axi_transaction[3]) handle_axi_rvalid;
		if (!mem_axi_bvalid && latched_waddr_en && latched_wdata_en && async_axi_transaction[4]) handle_axi_bvalid;
	end

	always @(posedge clk) begin
		mem_axi_arready <= 0;
		mem_axi_awready <= 0;
		mem_axi_wready <= 0;

		fast_raddr <= 0;
		fast_waddr <= 0;
		fast_wdata <= 0;

		if (mem_axi_rvalid && mem_axi_rready) begin
			mem_axi_rvalid <= 0;
		end

		if (mem_axi_bvalid && mem_axi_bready) begin
			mem_axi_bvalid <= 0;
		end

		if (mem_axi_arvalid && mem_axi_arready && !fast_raddr) begin
			latched_raddr = mem_axi_araddr;
			latched_rinsn = mem_axi_arprot[2];
			latched_raddr_en = 1;
		end

		if (mem_axi_awvalid && mem_axi_awready && !fast_waddr) begin
			latched_waddr = mem_axi_awaddr;
			latched_waddr_en = 1;
		end

		if (mem_axi_wvalid && mem_axi_wready && !fast_wdata) begin
			latched_wdata = mem_axi_wdata;
			latched_wstrb = mem_axi_wstrb;
			latched_wdata_en = 1;
		end

		if (mem_axi_arvalid && !(latched_raddr_en || fast_raddr) && !delay_axi_transaction[0]) handle_axi_arvalid;
		if (mem_axi_awvalid && !(latched_waddr_en || fast_waddr) && !delay_axi_transaction[1]) handle_axi_awvalid;
		if (mem_axi_wvalid  && !(latched_wdata_en || fast_wdata) && !delay_axi_transaction[2]) handle_axi_wvalid;

		if (!mem_axi_rvalid && latched_raddr_en && !delay_axi_transaction[3]) handle_axi_rvalid;
		if (!mem_axi_bvalid && latched_waddr_en && latched_wdata_en && !delay_axi_transaction[4]) handle_axi_bvalid;
	end
endmodule
