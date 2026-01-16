`ifdef DEBUG_RVV
  `define debug_rvv(debug_command) debug_command
`else
  `define debug_rvv(debug_command)
`endif

`define min(a,b) (a < b ? a : b)
`define max(a,b) (a > b ? a : b)

/***************************************************************
 * picorv32_pcpi_rvv
 ***************************************************************/

module picorv32_pcpi_rvv #(
	parameter [0:0] REGS_INIT_ZERO = 0,
	parameter [16:0] VLEN = 17'd 128,
	// arithmetic op
	parameter integer NB_LANES = 2'b01, // 2^NB_LANES lanes used for arith op
	parameter [2:0] LANE_WIDTH = 3'b011 // 2^LANE_WIDTH bits per lane, possible values : 8,16,32,64,128
	// This must verify LANE_WIDTH * 2^NB_LANES <= VLEN
) (
	input clk, resetn,

	input             pcpi_valid,
	input      [31:0] pcpi_insn,
	input      [31:0] pcpi_rs1, 			// first scalar
	input      [31:0] pcpi_rs2, 			// second scalar
	output reg        pcpi_wr, 				// 1 if scalar result
	output reg [31:0] pcpi_rd, 				// scalar result
	output reg        pcpi_wait,
	output reg        pcpi_ready,

	input			  pcpi_trap_in,			// receive external trap
	output reg		  pcpi_trap_out,		// send internal trap
	
	output wire		  pcpi_is_rvv_insn, 	// 1 if pcpi_insn is an rvv instruction
	
	input	   [31:0] pcpi_mem_rdata, 		// value from load
	input			  pcpi_mem_ifetch, 		// 1 if fetching next instruction after vmem instr
	input	          pcpi_mem_done, 		// 1 if new value available
	output reg		  pcpi_mem_op, 			// 1 if a memory operation is being executed
	output reg		  pcpi_mem_ftrans, 		// 1 if this is the first transfer
	output wire		  pcpi_mem_load, 		// 1 if vector memory load
	output wire		  pcpi_mem_store, 		// 1 if vector memory store
	output reg		  pcpi_mem_wen, 		// memory access
	output reg [31:0] pcpi_mem_addr, 		// memory access addr
	output reg [31:0] pcpi_mem_wdata, 		// data to store
	output reg [3:0]  pcpi_mem_strb, 		// i=1 if byte i should be writtent to memory
	output			  pcpi_mem_init,
	input			  pcpi_mem_trans_done
);
	localparam integer VLENB = VLEN>>3;
	localparam [16:0] VLEN8 = VLEN >> 3;
	localparam [16:0] VLEN8_LOG = $clog2(VLEN8);
	localparam [16:0] VLEN16 = VLEN >> 4;
	localparam [16:0] VLEN16_LOG = $clog2(VLEN16);
	localparam [16:0] VLEN32 = VLEN >> 5;
	localparam [16:0] VLEN32_LOG = $clog2(VLEN32);
	localparam [16:0] VLEN64 = VLEN >> 6;
	localparam [16:0] VLEN64_LOG = $clog2(VLEN64);
	localparam SHIFTED_NB_LANES = 1 << NB_LANES;
	localparam [7:0] SHIFTED_LANE_WIDTH = 1 << LANE_WIDTH;
	localparam integer VLEN_ARITH_IMM = VLEN-5;
	localparam integer VLEN_ARITH_RS  = VLEN-32;
	localparam integer SHIFTED_LANE_WIDTHB = SHIFTED_LANE_WIDTH >> 3;

	assign pcpi_is_rvv_insn = |{instr_cfg, instr_mem, instr_arith, instr_mask};
	assign pcpi_mem_init = vl > 0 && instr_mem && !pcpi_mem_op && byte_index == 0 && reg_index == 0 && mem_sending && !pcpi_ready && !pcpi_mem_trans_done;

	wire instr_run = pcpi_valid && !pcpi_ready; // 1 if should execute instruction

	reg [31:0] vxsat, vxrm, vcsr, vl, vlmax, vtype;
	reg [31:0] vstart; // reg must be as short as possible

	wire	   vill  = vtype[31];
	wire 	   vma   = vtype[7];
	wire 	   vta   = vtype[6];
	// wire [2:0] vsew  = instr_mask_unmaskable ? LANE_WIDTH-3 : vtype[5:3];
	wire [2:0] vsew  = instr_mask && !instr_viota && !(instr_vmv_sx || instr_vmv_xs) ? LANE_WIDTH-3 : vtype[5:3];
	wire [2:0] vlmul = vtype[2:0];

	wire [4:0] vs1 = pcpi_insn[19:15];
	wire [4:0] vs2 = pcpi_insn[24:20];
	wire [4:0] vd = pcpi_insn[11:7];
	wire vm = pcpi_insn[25];

	wire alu_vv = arith_vv | mask_vv;
	wire alu_vx = arith_vx | mask_vx;
	wire alu_vi = arith_vi;
	wire arith_instr_signed;

	wire [VLEN-1:0] alu_vs1 = alu_vv ? vregs_rdata1 :
							alu_vi ? {arith_instr_signed ? {VLEN_ARITH_IMM{pcpi_insn[19]}} : {VLEN_ARITH_IMM{1'b0}},pcpi_insn[19:15]} :
							alu_vx ? {arith_instr_signed ? {VLEN_ARITH_RS{pcpi_rs1[31]}} : {VLEN_ARITH_RS{1'b0}} ,pcpi_rs1} :
							0;

	reg	should_trap;
	reg pcpi_trap_in_q;
	
	// VSETVL
	reg instr_vsetvli, instr_vsetivli, instr_vsetvl;
	wire instr_cfg = |{instr_vsetvli, instr_vsetivli, instr_vsetvl};
	reg [31:0] avl;

	reg [16:0]  byte_index; // index in the vector register, same size as VLEN parameter
	reg [4:0]  reg_index, reg_index_q; // index of the vector register

	// LOAD-STORE
	reg instr_vload, instr_vstore;
	assign pcpi_mem_load = instr_vload;
	assign pcpi_mem_store = instr_vstore;
	reg instr_mem_unit;
	reg instr_mem_strided;
	reg instr_mem_indexed;
	reg instr_mem_whole_reg;
	wire instr_mem = |{instr_vload, instr_vstore};
	reg [31:0] mem_transfer_n; // number of 32 bits transfers
	reg 	   mem_sending; // 1 if addr needs to be sent, 0 if waiting for data from main core
	reg		   last_op; // condition to determine wether the transfer is over
	reg	[2:0]  mem_sew;
	reg [11:0] mem_stride_mask; // mask for strided transfers
	reg [1:0]  mem_stride_i; // index of transfer for 1 element in strided transfers, 0->2
	reg [31:0] mem_stride_amount; // mem offset value between two accesses
	reg [31:0] mem_offset_q;
	reg	[3:0]  tmp_mem_strb;
	reg	[31:0] tmp_mem_wdata;
	reg [16:0]  mem_indexed_byte_index; // used for indexed accesses | index in the offset vector register, same size as VLEN parameter
	reg [4:0]  mem_indexed_reg_index; // used for indexed accesses | index of the offset vector register
	reg [2:0]  mem_indexed_sew; // sew of indices in an indexed insn
	reg [3:0]  mem_seg_nfields; // number of segments in a segment load / store
	reg [2:0]  mem_seg_i; // identifies the current field vector
	
	integer offset; // avoid errors
	integer tmp_offset;
	integer offset_incr;
	
	// ARITHMETIC OPERATIONS
	reg instr_arith;
	reg arith_vv;
	reg arith_vi;
	reg arith_vx;
	reg alu_run;
	wire [(64 << NB_LANES) - 1:0] arith_vd;
	wire [(17 << NB_LANES) - 1:0] arith_regi;
	wire [(1 << NB_LANES) - 1:0] arith_res;
	wire arith_done;
	wire arith_instr_valid;
	reg [16:0] arith_remaining;
	reg arith_init;
	reg [2:0] arith_step;

    wire [16:0] tmp_nb_lanes = `min(vl, SHIFTED_NB_LANES);
    wire [4:0] nb_lanes = tmp_nb_lanes[16] ? 5'b10000 :
                          tmp_nb_lanes[15] ? 5'b01111 :
                          tmp_nb_lanes[14] ? 5'b01110 :
                          tmp_nb_lanes[13] ? 5'b01101 :
                          tmp_nb_lanes[12] ? 5'b01100 :
                          tmp_nb_lanes[11] ? 5'b01011 :
                          tmp_nb_lanes[10] ? 5'b01010 :
                          tmp_nb_lanes[9]  ? 5'b01001 :
                          tmp_nb_lanes[8]  ? 5'b01000 :
                          tmp_nb_lanes[7]  ? 5'b00111 :
                          tmp_nb_lanes[6]  ? 5'b00110 :
                          tmp_nb_lanes[5]  ? 5'b00101 :
                          tmp_nb_lanes[4]  ? 5'b00100 :
                          tmp_nb_lanes[3]  ? 5'b00011 :
                          tmp_nb_lanes[2]  ? 5'b00010 :
                          tmp_nb_lanes[1]  ? 5'b00001 :
                                             5'b00000;

	integer lane_num;

	/* wire [SHIFTED_NB_LANES-1:0] arith_mask;
	genvar maski;
	generate
		for (maski = 0; maski < SHIFTED_NB_LANES; maski = maski + 1)
			assign arith_mask[maski] = vm || vregs_v0[(arith_regi[maski*17 +: 17] >> (vsew+3)) + (VLEN >> (vsew+3)) * reg_index];
	endgenerate */

	// MASK INSTRUCTIONS
	reg instr_mask;
	wire instr_mask_unmaskable = instr_mask && pcpi_insn[31:29] == 3'b011;

	reg mask_vv, mask_vx;
	reg instr_vcpop, instr_vfirst, instr_viota, instr_vid;
	wire [VLEN-1:0] vl_mask = {VLEN{1'b1}} >> (VLEN - vl);
	wire [VLEN-1:0] alu_vs2_tmp = instr_vid ? {VLEN{1'b1}} : vregs_rdata2;
	wire [VLEN-1:0] alu_vs2 = (instr_mask) ? (!vm ? alu_vs2_tmp & vl_mask & vregs_v0 : alu_vs2_tmp & vl_mask) : alu_vs2_tmp;

	reg instr_vmove, instr_vmerge, instr_vmv_nr;
	reg instr_vmv_xs, instr_vmv_sx;
	wire [VLEN-1:0] arith_remaining_mask = {VLEN{1'b1}} >> (VLEN > (arith_remaining << (vsew+3)) ? VLEN - (arith_remaining << (vsew+3)) : 0);
	wire [VLEN-1:0] vregs_v0_8, vregs_v0_16, vregs_v0_32, vregs_v0_64;
	reg [2:0] vmv_nr_acc;

	wire instr_vmset = instr_arith && pcpi_insn[31:29] == 3'b011;
	
	genvar geni;
	generate
		for (geni = 0; geni < VLEN8; geni = geni + 1) begin
			assign vregs_v0_8[geni << 3 +: 8] = {8{vregs_v0[geni + (reg_index << VLEN8_LOG)]}};
		end
		for (geni = 0; geni < VLEN16; geni = geni + 1) begin
			assign vregs_v0_16[geni << 4 +: 16] = {16{vregs_v0[geni + (reg_index << VLEN16_LOG)]}};
		end
		for (geni = 0; geni < VLEN32; geni = geni + 1) begin
			assign vregs_v0_32[geni << 5 +: 32] = {32{vregs_v0[geni + (reg_index << VLEN32_LOG)]}};
		end
		for (geni = 0; geni < VLEN64; geni = geni + 1) begin
			assign vregs_v0_64[geni << 6 +: 64] = {64{vregs_v0[geni + (reg_index << VLEN64_LOG)]}};
		end
	endgenerate


	integer loop_i;

	wire [4:0] vregs_raddr1 = !instr_run					? vregs_waddr :
							  instr_vload					? vregs_waddr :
							  instr_viota					? vs1 :
							  ((instr_arith || instr_viota) && arith_done) || ((instr_vmove || instr_vmerge) && reg_index != reg_index_q)   ? vregs_waddr + (!instr_vmset) : // arith op with LMUL > 1
							  (instr_arith || instr_mask)	? vs1 + reg_index :
							  (instr_vstore && mem_sending)	? pcpi_insn[11:7] + reg_index + (mem_seg_i << (vtype[2] ? 0 : vtype[2:0])) :
							  0;

	wire [4:0] vregs_raddr2 = instr_mem_indexed 			? vs2 + mem_indexed_reg_index :
							  instr_viota 					? vs2 :
							  (instr_arith || instr_mask)	? vs2 + reg_index :
							  0;

	reg  [4:0] vregs_waddr_q;
	wire [4:0] vregs_waddr = (instr_vload) 					? pcpi_insn[11:7] + reg_index + (mem_seg_i << (vtype[2] ? 0 : vtype[2:0])) :
							 (instr_vmset)					? vd :
							 (instr_arith | instr_viota)	? vd + reg_index :
							 (instr_mask)				 	? vd :
							 0;

	reg [VLEN-1:0] vregs_wdata_acc, tmp_vregs_wdata;
	reg vregs_wen;
	reg [VLEN-1:0] vregs_wdata;
	wire [VLEN-1:0] vregs_rdata1, vregs_rdata2;

	reg [VLEN-1:0] vregs_v0;
	
	rvv_vregs #(
		.VLEN(VLEN),
		.REGS_INIT_ZERO(REGS_INIT_ZERO)
	) vregs (
		.clk(clk),
		.waddr(vregs_waddr_q),
		.wen(vregs_wen),
		.raddr1(vregs_raddr1),
		.raddr2(vregs_raddr2),
		.wdata(vregs_wdata),
		.rdata1(vregs_rdata1),
		.rdata2(vregs_rdata2)
	);

	always @* begin
		should_trap = 0;

		// vsetvl
		instr_vsetvli = 0;
		instr_vsetivli = 0;
		instr_vsetvl = 0;
		
		// mem
		instr_vload = 0;
		instr_vstore = 0;
		mem_transfer_n = 0;
		instr_mem_unit = 0;
		instr_mem_strided = 0;
		instr_mem_indexed = 0;
		instr_mem_whole_reg = 0;
		mem_sew = 0;
		mem_indexed_sew = 0;
		mem_seg_nfields = 0;

		// arith
		instr_arith = 0;
		arith_vv = 0;
		arith_vi = 0;
		arith_vx = 0;

		// mask
		instr_mask = 0;
		mask_vv = 0;
		mask_vx = 0;
		instr_vcpop = 0;
		instr_vfirst = 0;
		instr_viota = 0;
		instr_vid = 0;

		instr_vmove = 0;
		instr_vmv_nr = 0;
		instr_vmv_xs = 0;
		instr_vmv_sx = 0;
		instr_vmerge = 0;

		// config
		if (resetn && pcpi_insn[14:12] == 3'b111 && pcpi_insn[6:0] == 7'b1010111) begin
			if (pcpi_insn[31] == 1'b0)
				instr_vsetvli = 1;
			else if (pcpi_insn[31:30] == 2'b11)
				instr_vsetivli = 1;
			else if (pcpi_insn[31:25] == 7'b1000000)
				instr_vsetvl = 1;
		end

		// mem
		if (resetn && (pcpi_insn[6:0] == 7'b0000111 || pcpi_insn[6:0] == 7'b0100111)) begin
			instr_vload = !pcpi_insn[5];
			instr_vstore = pcpi_insn[5];

			case (pcpi_insn[27:26]) 
				2'b00: begin
					// unit-stride or whole vector
					instr_mem_whole_reg = pcpi_insn[24:20] == 5'b01000;
					instr_mem_unit = pcpi_insn[24:20] != 5'b01000;
				end
				2'b10:
					// constant stride
					instr_mem_strided = 1;
				2'b01, 2'b11:
					// indexed
					instr_mem_indexed = 1;
			endcase

			mem_seg_nfields = {1'b0, pcpi_insn[31:29]} + 1;
			if (!instr_mem_indexed) begin
				case (pcpi_insn[14:12])
					3'b000: mem_sew = 3'b011;
					3'b101: mem_sew = 3'b100;
					3'b110: mem_sew = 3'b101; 
					3'b111: mem_sew = 3'b110;
				endcase
				should_trap = should_trap || ((pcpi_insn[11:7] + (vl / (VLEN >> mem_sew))) & 8'hF0); // >= 32
			end else begin
				mem_sew = vtype[5:3] + 3;
				case (pcpi_insn[14:12])
					3'b000: mem_indexed_sew = 3'b011;
					3'b101: mem_indexed_sew = 3'b100;
					3'b110: mem_indexed_sew = 3'b101; 
					3'b111: mem_indexed_sew = 3'b110;
				endcase
				should_trap = should_trap || ((pcpi_insn[11:7] + (vl / (VLEN >> mem_sew))) & 8'hF0) || ((pcpi_insn[24:20] + (vl / (VLEN >> mem_indexed_sew))) & 8'hF0); // >= 32
			end
			should_trap = should_trap || (!vtype[2] && (mem_seg_nfields << vtype[1:0]) > 8);

			if (instr_mem_whole_reg) begin
				mem_transfer_n = ({1'b0, pcpi_insn[31:29]} + 1) * (VLEN >> mem_sew);
				`debug_rvv($display("mem_transfer_n : %d", mem_transfer_n);)
			end
			else
				mem_transfer_n = vl;
		end

		// arith
		if (resetn && pcpi_insn[6:0] == 7'b1010111 && (pcpi_insn[14:12] == 3'b000 || pcpi_insn[14:12] == 3'b011 || pcpi_insn[14:12] == 3'b100)) begin
			instr_arith = 1;
			instr_vmove = pcpi_insn[31:26] == 6'b010111 && vm == 1;
			instr_vmerge = pcpi_insn[31:26] == 6'b010111 && vm == 0;
			case (pcpi_insn[14:12])
				3'b000: arith_vv = 1;
				3'b011: arith_vi = 1;
				3'b100: arith_vx = 1;
			endcase
			instr_vmv_nr = pcpi_insn[31:26] == 6'b100111 && vm == 1 && arith_vi == 1;
		end

		// mask
		if (resetn && pcpi_insn[6:0] == 7'b1010111 && (pcpi_insn[14:12] == 3'b010 || pcpi_insn[14:12] == 3'b110)) begin
			instr_mask = 1;
			instr_vcpop = pcpi_insn[31:26] == 6'b010000 && vs1 == 5'b10000;
			instr_vfirst = pcpi_insn[31:26] == 6'b010000 && vs1 == 5'b10001;
			instr_viota = pcpi_insn[31:26] == 6'b010100 && vs1[4:1] == 4'b1000;
			instr_vid = pcpi_insn[31:26] == 6'b010100 && vs1 == 5'b10001;
			mask_vv = pcpi_insn[14:12] == 3'b010;
			mask_vx = pcpi_insn[14:12] == 3'b110;
			instr_vmv_xs = pcpi_insn[31:26] == 6'b010000 && vm == 1 && vs1 == 5'b00000;
			instr_vmv_sx = pcpi_insn[31:26] == 6'b010000 && vm == 1 && vs2 == 5'b00000;
		end
	end

	always @(posedge clk) begin
		pcpi_ready <= 0;
		pcpi_wait <= 0;
		pcpi_wr <= 0;
		pcpi_rd <= instr_vfirst ? 32'hFFFFFFFF : 0;
		pcpi_mem_wen <= 0; // memory access
		pcpi_mem_addr <= pcpi_rs1 + mem_offset_q; // mem addr
		pcpi_mem_op <= 0;
		pcpi_mem_strb <= 0;
		pcpi_mem_ftrans <= 0;
		pcpi_trap_out <= 0;
		
		// if (!instr_run || (instr_arith && arith_remaining <= 1 << NB_LANES && ((instr_arith || instr_mask) ? (vsew+3 <= LANE_WIDTH || arith_step == ((1 << (vsew+3-LANE_WIDTH)) - 1)) : 1)))
		if (!instr_run || ((instr_arith || instr_viota) && arith_done && !instr_vmset) || ((instr_vmove || instr_vmerge || instr_vmv_xs || instr_vmv_sx) && reg_index != reg_index_q))
			vregs_wdata_acc <= vregs_rdata1;

		vregs_wen <= 0;
		vregs_wdata <= 0;
		vregs_waddr_q <= vregs_waddr;

		// arith
		alu_run <= 0;
		
		if (!resetn) begin		
			pcpi_trap_in_q <= 0;
			vstart = 0;
			vxsat = 0;
			vxrm = 0;
			vcsr = 0;
			vl = 0;
			vtype = 32'h8000_0000;

			// vsetvl
			vlmax = 0;
			avl = 0;

			// mem
			byte_index = 0;
			reg_index <= 0;
			reg_index_q <= 0;
			mem_indexed_byte_index <= 0;
			mem_indexed_reg_index <= 0;
			mem_sending = 1;
			last_op = 0;
			mem_stride_mask <= 12'h000;
			mem_stride_i <= 0;
			mem_offset_q = 0;
			tmp_mem_strb = 0;
			tmp_mem_wdata = 0;
			offset <= 0;
			offset_incr = 0;
			mem_seg_i<= 0;

			// arith
			arith_remaining <= 0;
			arith_init <= 1;
			arith_step <= 0;
			vregs_wdata_acc <= 0;
			vmv_nr_acc <= 0;
		end else begin
			reg_index_q <= reg_index;
			/* if (should_trap)
				pcpi_trap_out <= 1; */
			
			// if (should_trap || pcpi_trap_in || pcpi_trap_out) begin
			if (0) begin
				$display("trap");
				/* if (!pcpi_trap_in_q) begin
					`debug_rvv($display("trap taken");)
					if (instr_mem) begin
						vstart <= {2'b00, mem_indexed_reg_index, mem_indexed_byte_index, reg_index, byte_index};
					end
				end */
			end else begin
				/* if (pcpi_trap_in_q) begin
					`debug_rvv($display("context restored");)
					if (instr_mem) begin
						mem_indexed_reg_index 	= vstart[29:25];
						mem_indexed_byte_index 	= vstart[24:15];
						reg_index 			= vstart[14:10];
						byte_index 			= vstart[9:0];
						mem_stride_i = 0;
						if ((byte_index + (VLEN >> mem_sew) * reg_index) >= mem_transfer_n || byte_index >= VLENB || reg_index >= 8)
							 pcpi_trap_out <= 1;
					end
				end */
				if (instr_run) begin
					if (instr_cfg) begin
						if (instr_vsetvli || instr_vsetivli) begin
							vtype[31] = 0; // vill
							vtype[30:8] = {23{1'b0}}; // reserved
							vtype[7] = pcpi_insn[27]; // vma
							vtype[6] = pcpi_insn[26]; // vta
							vtype[5:3] = pcpi_insn[25:23]; // vsew
							vtype[2:0] = pcpi_insn[22:20]; // vlmul
						end else if (instr_vsetvl) begin
							vtype = pcpi_rs2;
						end
						// +3 because vsew starts at 8
						case (vtype[2:0])
							3'b100: vlmax = 0;
							3'b101: vlmax = (VLEN >> (vtype[5:3]+3)) >> 3;
							3'b110: vlmax = (VLEN >> (vtype[5:3]+3)) >> 2;
							3'b111: vlmax = (VLEN >> (vtype[5:3]+3)) >> 1;
							3'b000,
							3'b001,
							3'b010,
							3'b011: vlmax = (VLEN >> (vtype[5:3]+3)) << vtype[2:0];
						endcase;

						if (vlmax == 0) begin
							vtype = 32'h8000_0000;
							vl = 0;
						end else begin
							if (instr_vsetvli || instr_vsetvl) begin
								if (pcpi_insn[19:15] != 0)
									avl = pcpi_rs1;
								else if (pcpi_insn[11:7] != 0)
									avl = 32'hFFFF_FFFF;
								else
									avl = vl;
							end else if (instr_vsetivli)
								avl = {{27{1'b0}}, pcpi_insn[19:15]};
							
							vl = (avl <= vlmax) ? avl : vlmax;
						end
						pcpi_rd <= vl;
						pcpi_wr <= 1;
						pcpi_wait <= 0;
						pcpi_ready <= 1;
					end else if (instr_mem) begin
						if (vl == 0) begin
							pcpi_ready <= 1;
							pcpi_wait <= 0;
						end else begin
						pcpi_mem_op <= 1;
						pcpi_rd <= 0;
						pcpi_wait <= 1;
						pcpi_ready <= 0;
						pcpi_mem_wen <= 0;

						mem_stride_amount = (instr_mem_unit || instr_mem_whole_reg) ? (1 << (mem_sew-3)) : pcpi_rs2;

						if (instr_mem_indexed) begin
							tmp_offset = mem_indexed_byte_index << mem_indexed_sew;
							mem_offset_q = 0;
							case (mem_indexed_sew)
								3'b011: mem_offset_q[0 +: 8] = vregs_rdata2[tmp_offset +: 8]; // 8 bits index
								3'b100: mem_offset_q[0 +: 16] = vregs_rdata2[tmp_offset +: 16]; // 16 bits index
								3'b101: mem_offset_q[0 +: 32] = vregs_rdata2[tmp_offset +: 32]; // 32 bits index
								3'b110: mem_offset_q[0 +: 32] = vregs_rdata2[tmp_offset +: 32]; // 64 bits index but we only keep 32 LSB
							endcase
							mem_offset_q = mem_offset_q + (mem_stride_i << 2);
						end else
							mem_offset_q = mem_stride_amount * ((!instr_mem_whole_reg ? mem_seg_nfields : 1) * (byte_index + reg_index * (VLEN >> mem_sew))) + (mem_stride_i << 2);

						// for segment ops
						mem_offset_q = mem_offset_q + (mem_seg_i << (mem_sew - 3));
						
						if (mem_sending && mem_stride_i == 0) begin
							case (mem_offset_q[1:0] + pcpi_rs1[1:0])
								2'b00: begin // no shift
									case (mem_sew)
										3'b011: mem_stride_mask = 12'h001; // 8 bits
										3'b100: mem_stride_mask = 12'h003; // 16 bits
										3'b101: mem_stride_mask = 12'h00F; // 32 bits
										3'b110: mem_stride_mask = 12'h0FF; // 64 bits
									endcase
								end
								2'b01: begin // shift 1 byte
									case (mem_sew)
										3'b011: mem_stride_mask = 12'h002; // 8 bits
										3'b100: mem_stride_mask = 12'h006; // 16 bits
										3'b101: mem_stride_mask = 12'h01E; // 32 bits
										3'b110: mem_stride_mask = 12'h1FE; // 64 bits
									endcase
								end
								2'b10: begin // shift 2 bytes
									case (mem_sew)
										3'b011: mem_stride_mask = 12'h004; // 8 bits
										3'b100: mem_stride_mask = 12'h00C; // 16 bits
										3'b101: mem_stride_mask = 12'h03C; // 32 bits
										3'b110: mem_stride_mask = 12'h3FC; // 64 bits
									endcase
								end
								2'b11: begin // shift 3 bytes
									case (mem_sew)
										3'b011: mem_stride_mask = 12'h008; // 8 bits
										3'b100: mem_stride_mask = 12'h018; // 16 bits
										3'b101: mem_stride_mask = 12'h078; // 32 bits
										3'b110: mem_stride_mask = 12'h7F8; // 64 bits
									endcase
								end
							endcase
						end

						if (instr_vload) begin
							if (mem_sending) begin
								// send addr to main proc
								pcpi_mem_ftrans <= (byte_index == 0 && reg_index == 0 && mem_stride_i == 0 && mem_seg_i == 0);
								pcpi_mem_wen <= !pcpi_mem_ifetch;
								pcpi_mem_addr <= pcpi_rs1 + mem_offset_q;

								mem_sending <= pcpi_mem_ifetch ? 1 : 0;
								pcpi_wr <= 0;
								pcpi_wait <= 1;
								pcpi_ready <= 0;
							end else if (pcpi_mem_done) begin
								pcpi_mem_ftrans <= 0;
								tmp_vregs_wdata = vregs_rdata1;
								if (mem_stride_mask[{mem_stride_i, 2'b00}]) begin
									tmp_vregs_wdata[(offset) << 3 +: 8] = pcpi_mem_rdata[0 +: 8];
									offset_incr = offset_incr + 1;
								end
								if (mem_stride_mask[{mem_stride_i, 2'b00} + 1]) begin
									tmp_vregs_wdata[((offset + offset_incr) << 3) +: 8] = pcpi_mem_rdata[8 +: 8];
									offset_incr = offset_incr + 1;
								end
								if (mem_stride_mask[{mem_stride_i, 2'b00}+2]) begin
									tmp_vregs_wdata[((offset + offset_incr) << 3) +: 8] = pcpi_mem_rdata[16 +: 8];
									offset_incr = offset_incr + 1;
								end
								if (mem_stride_mask[{mem_stride_i, 2'b00} + 3]) begin
									tmp_vregs_wdata[((offset + offset_incr) << 3) +: 8] = pcpi_mem_rdata[24 +: 8];
									offset_incr = offset_incr + 1;
								end
								if (vm || vregs_v0[(byte_index + (VLEN >> mem_sew) * reg_index)]) begin
									vregs_wen <= 1;
									// vregs_wdata_acc <= tmp_vregs_wdata;
								end
								vregs_wdata <= tmp_vregs_wdata;

								if ((mem_stride_i==0 && mem_stride_mask[7:4] == 0) || (mem_stride_i==1 && mem_stride_mask[11:8] == 0) || mem_stride_i==2) begin
									if (mem_seg_i == mem_seg_nfields - 1 || instr_mem_whole_reg)
										offset <= offset + offset_incr;
									offset_incr = 0;
								end
								
								last_op = (byte_index + (VLEN >> mem_sew) * reg_index) == mem_transfer_n - 1 && ((mem_stride_i==0 && mem_stride_mask[7:4] == 0) || (mem_stride_i==1 && mem_stride_mask[11:8] == 0) || mem_stride_i==2) && (mem_seg_i == mem_seg_nfields - 1 || instr_mem_whole_reg);

								if (!last_op) begin
									// not last transfer

									// update indices
									if ((mem_stride_i==0 && mem_stride_mask[7:4] == 0) || (mem_stride_i==1 && mem_stride_mask[11:8] == 0) || mem_stride_i==2) begin
										mem_stride_i = 0;
										if (mem_seg_i == mem_seg_nfields - 1 || instr_mem_whole_reg) begin
											mem_seg_i <= 0;
											if (byte_index == (VLEN >> mem_sew) - 1) begin
												byte_index = 0;
												offset <= 0;
												reg_index <= reg_index + 1;
											end else begin
												reg_index <= reg_index;
												byte_index = byte_index + 1;
											end
										end else
											mem_seg_i <= mem_seg_i + 1; // TODO ERREUR ?
										// update indexed indices
										if (instr_mem_indexed && (mem_seg_i == mem_seg_nfields - 1)) begin
											if (mem_indexed_byte_index == (VLEN >> mem_indexed_sew) - 1) begin
												mem_indexed_byte_index <= 0;
												mem_indexed_reg_index <= mem_indexed_reg_index + 1;
											end else begin
												mem_indexed_byte_index <= mem_indexed_byte_index + 1;
												mem_indexed_reg_index <= mem_indexed_reg_index;
											end
										end
									end else
										mem_stride_i = mem_stride_i + 1;
								
									pcpi_wait <= 1;
									pcpi_ready <= 0;
								end else begin
									pcpi_wait <= 0;
									pcpi_ready <= 1;
									pcpi_mem_op <= 0;
									// update indices
									byte_index = 0;
									reg_index <= 0;
									mem_indexed_byte_index <= 0;
									mem_indexed_reg_index <= 0;
									offset <= 0;
									offset_incr = 0;
									mem_stride_i = 0;
									mem_seg_i<= 0;
									mem_offset_q = 0;
								end
								
								mem_sending <= 1;
								// outputs
								pcpi_mem_wen <= 0;
								pcpi_mem_addr <= pcpi_rs1 + mem_offset_q;						
								pcpi_wr <= 0;
							end else begin						
								pcpi_ready <= 0;
								pcpi_wait <= 1;
							end
						end else if (instr_vstore) begin
							if (mem_sending) begin
								// send data to main proc
								pcpi_mem_ftrans <= (byte_index == 0 && reg_index == 0 && mem_stride_i == 0 && mem_seg_i == 0);
								
								tmp_mem_strb = 0;
								tmp_mem_wdata = 0;
								if (mem_stride_mask[{mem_stride_i, 2'b00}]) begin
									tmp_offset = (offset + offset_incr) << 3;
									tmp_mem_wdata[0 +: 8] = vregs_rdata1[tmp_offset +: 8];
									tmp_mem_strb[0] = 1'b1;
									// $display("store vregs[%d] = %h", tmp_offset, vregs_rdata1[tmp_offset +: 8]);
									offset_incr = offset_incr + 1;
								end
								if (mem_stride_mask[{mem_stride_i, 2'b00} + 1]) begin
									tmp_offset = (offset + offset_incr) << 3;
									tmp_mem_wdata[8 +: 8] = vregs_rdata1[tmp_offset +: 8];
									tmp_mem_strb[1] = 1'b1;
									// $display("store vregs[%d] = %h", tmp_offset, vregs_rdata1[tmp_offset +: 8]);
									offset_incr = offset_incr + 1;
								end
								if (mem_stride_mask[{mem_stride_i, 2'b00}+2]) begin
									tmp_offset = (offset + offset_incr) << 3;
									tmp_mem_wdata[16 +: 8] = vregs_rdata1[tmp_offset +: 8];
									tmp_mem_strb[2] = 1'b1;
									// $display("store vregs[%d] = %h", tmp_offset, vregs_rdata1[tmp_offset +: 8]);
									offset_incr = offset_incr + 1;
								end
								if (mem_stride_mask[{mem_stride_i, 2'b00} + 3]) begin
									tmp_offset = (offset + offset_incr) << 3;
									tmp_mem_wdata[24 +: 8] = vregs_rdata1[tmp_offset +: 8];
									tmp_mem_strb[3] = 1'b1;
									// $display("store vregs[%d] = %h\n", tmp_offset, vregs_rdata1[tmp_offset +: 8]);
									offset_incr = offset_incr + 1;
								end

								if ((mem_stride_i==0 && mem_stride_mask[7:4] == 0) || (mem_stride_i==1 && mem_stride_mask[11:8] == 0) || mem_stride_i==2) begin
									if (!pcpi_mem_ifetch && mem_seg_i == mem_seg_nfields - 1 || instr_mem_whole_reg)
										offset <= offset + offset_incr;
									offset_incr = 0;
								end
								
								pcpi_mem_strb <= vm || vregs_v0[(byte_index + (VLEN >> mem_sew) * reg_index)] ? tmp_mem_strb : 0;
								pcpi_mem_wdata <= tmp_mem_wdata;

								mem_sending <= pcpi_mem_ifetch ? 1 : 0;
								if (pcpi_mem_ifetch) offset_incr = 0;
								// outputs
								pcpi_mem_wen <= !pcpi_mem_ifetch;
								pcpi_mem_addr <= pcpi_rs1 + mem_offset_q;
								pcpi_wait <= 1;
								pcpi_ready <= 0;
							end else if (pcpi_mem_done) begin
								pcpi_mem_ftrans <= 0;
								last_op = (byte_index + (VLEN >> mem_sew) * reg_index) == mem_transfer_n - 1 && ((mem_stride_i==0 && mem_stride_mask[7:4] == 0) || (mem_stride_i==1 && mem_stride_mask[11:8] == 0) || mem_stride_i==2) && (mem_seg_i == mem_seg_nfields - 1 || instr_mem_whole_reg);

								if (!last_op) begin
									// not last tranfer

									// update indices
									if ((mem_stride_i==0 && mem_stride_mask[7:4] == 0) || (mem_stride_i==1 && mem_stride_mask[11:8] == 0) || mem_stride_i==2) begin
										mem_stride_i = 0;
										if (mem_seg_i == mem_seg_nfields - 1 || instr_mem_whole_reg) begin
											mem_seg_i <= 0;
											if (byte_index == (VLEN >> mem_sew) - 1) begin
												byte_index = 0;
												offset <= 0;
												reg_index <= reg_index + 1;
											end else begin
												reg_index <= reg_index;
												byte_index = byte_index + 1;
											end
										end else
											mem_seg_i <= mem_seg_i + 1;
										// update indexed indices
										if (instr_mem_indexed && (mem_seg_i == mem_seg_nfields - 1)) begin
											if (mem_indexed_byte_index == (VLEN >> mem_indexed_sew) - 1) begin
												mem_indexed_byte_index <= 0;
												mem_indexed_reg_index <= mem_indexed_reg_index + 1;
											end else begin
												mem_indexed_byte_index <= mem_indexed_byte_index + 1;
												mem_indexed_reg_index <= mem_indexed_reg_index;
											end
										end
									end else
										mem_stride_i = mem_stride_i + 1;
										
									pcpi_wait <= 1;
									pcpi_ready <= 0;
								end else begin
									// last transfer
									pcpi_wait <= 0;
									pcpi_ready <= 1;
									pcpi_mem_op <= 0;
									// update indices
									byte_index = 0;
									reg_index <= 0;
									mem_indexed_byte_index <= 0;
									mem_indexed_reg_index <= 0;
									offset <= 0;
									offset_incr = 0;
									mem_stride_i = 0;
									mem_seg_i<= 0;
									mem_offset_q = 0;
								end

								pcpi_mem_wen <= 0;
								pcpi_mem_addr <= pcpi_rs1 + mem_offset_q;
								mem_sending <= 1;
								pcpi_wr <= 0;
							end else begin
								pcpi_ready <= 0;
								pcpi_wait <= 1;
							end
						end
						end
					end else if (instr_vmv_nr) begin
						if (vmv_nr_acc == pcpi_insn[17:15]) begin
							vmv_nr_acc <= 0;
							vregs_wdata <= vregs_rdata2;
							vregs_wen <= 1;
							pcpi_ready <= 1;
							pcpi_wait <= 0;
						end else begin
							vmv_nr_acc <= vmv_nr_acc + 1;
							vregs_wdata <= vregs_rdata2;
							vregs_wen <= 1;
							pcpi_ready <= 0;
							pcpi_wait <= 1;
						end
					end else if (instr_vmove || instr_vmerge || instr_vmv_xs || instr_vmv_sx) begin
						if (instr_vmv_xs) begin
							case (vsew)
								3'b000: pcpi_rd <= {{24{vregs_rdata2[7]}}, vregs_rdata2[0 +: 8]};
								3'b001: pcpi_rd <= {{16{vregs_rdata2[15]}}, vregs_rdata2[0 +: 16]};
								3'b010: pcpi_rd <= vregs_rdata2[0 +: 32];
								3'b011: pcpi_rd <= vregs_rdata2[0 +: 32];
							endcase
							pcpi_wr <= 1;
							pcpi_ready <= 1;
							pcpi_wait <= 0;
						end else if (instr_vmv_sx) begin
							if (vl > 0) begin
								case (vsew)
									3'b000: vregs_wdata <= {vregs_wdata_acc[VLEN-1 :  8], pcpi_rs1[0 +: 8]};
									3'b001: vregs_wdata <= {vregs_wdata_acc[VLEN-1 : 16], pcpi_rs1[0 +: 16]};
									3'b010: vregs_wdata <= {vregs_wdata_acc[VLEN-1 : 32], pcpi_rs1};
									3'b011: vregs_wdata <= {vregs_wdata_acc[VLEN-1 : 64], {32{pcpi_rs1[31]}}, pcpi_rs1};
								endcase
								vregs_wen <= 1;
							end
							pcpi_ready <= 1;
							pcpi_wait <= 0;
						end else if (reg_index_q == reg_index) begin
							if (arith_remaining > 0) begin
								if (arith_vv)
									if (instr_vmove)
										vregs_wdata <= (vregs_wdata_acc & ~arith_remaining_mask) | (vregs_rdata1 & arith_remaining_mask);
									else // vmerge
										case (vsew)
											3'b000: vregs_wdata <= (vregs_wdata_acc & ~arith_remaining_mask) | (vregs_rdata1 & arith_remaining_mask & vregs_v0_8)  | (vregs_rdata2 & arith_remaining_mask & ~vregs_v0_8);
											3'b001: vregs_wdata <= (vregs_wdata_acc & ~arith_remaining_mask) | (vregs_rdata1 & arith_remaining_mask & vregs_v0_16) | (vregs_rdata2 & arith_remaining_mask & ~vregs_v0_16);
											3'b010: vregs_wdata <= (vregs_wdata_acc & ~arith_remaining_mask) | (vregs_rdata1 & arith_remaining_mask & vregs_v0_32) | (vregs_rdata2 & arith_remaining_mask & ~vregs_v0_32);
											3'b011: vregs_wdata <= (vregs_wdata_acc & ~arith_remaining_mask) | (vregs_rdata1 & arith_remaining_mask & vregs_v0_64) | (vregs_rdata2 & arith_remaining_mask & ~vregs_v0_64);
										endcase
										// vregs_wdata <= (vregs_wdata_acc & ~arith_remaining_mask) | (vregs_rdata1 & arith_remaining_mask);
								else begin
									if (instr_vmove)
										case (vsew)
											3'b000: vregs_wdata <= (vregs_wdata_acc & ~arith_remaining_mask) | ({VLEN8{arith_vi ? {{3{pcpi_insn[19]}}, pcpi_insn[19:15]} : pcpi_rs1[0+:8]}} & arith_remaining_mask);
											3'b001: vregs_wdata <= (vregs_wdata_acc & ~arith_remaining_mask) | ({VLEN16{arith_vi ? {{11{pcpi_insn[19]}}, pcpi_insn[19:15]} : pcpi_rs1[0+:16]}} & arith_remaining_mask);
											3'b010: vregs_wdata <= (vregs_wdata_acc & ~arith_remaining_mask) | ({VLEN32{arith_vi ? {{27{pcpi_insn[19]}}, pcpi_insn[19:15]} : pcpi_rs1[0+:32]}} & arith_remaining_mask);
											3'b011: vregs_wdata <= (vregs_wdata_acc & ~arith_remaining_mask) | ({VLEN64{arith_vi ? {{59{pcpi_insn[19]}}, pcpi_insn[19:15]} : {{32{pcpi_rs1[31]}}, pcpi_rs1[0+:8]}}} & arith_remaining_mask);
										endcase
									else // vmerge
										case (vsew)
											3'b000: vregs_wdata <= (vregs_wdata_acc & ~arith_remaining_mask) | ({VLEN8{arith_vi ? {{3{pcpi_insn[19]}}, pcpi_insn[19:15]} : pcpi_rs1[0+:8]}} & arith_remaining_mask & vregs_v0_8) | (vregs_rdata2 & arith_remaining_mask & ~vregs_v0_8);
											3'b001: vregs_wdata <= (vregs_wdata_acc & ~arith_remaining_mask) | ({VLEN16{arith_vi ? {{11{pcpi_insn[19]}}, pcpi_insn[19:15]} : pcpi_rs1[0+:16]}} & arith_remaining_mask & vregs_v0_16) | (vregs_rdata2 & arith_remaining_mask & ~vregs_v0_16);
											3'b010: vregs_wdata <= (vregs_wdata_acc & ~arith_remaining_mask) | ({VLEN32{arith_vi ? {{27{pcpi_insn[19]}}, pcpi_insn[19:15]} : pcpi_rs1[0+:32]}} & arith_remaining_mask & vregs_v0_32) | (vregs_rdata2 & arith_remaining_mask & ~vregs_v0_32);
											3'b011: vregs_wdata <= (vregs_wdata_acc & ~arith_remaining_mask) | ({VLEN64{arith_vi ? {{59{pcpi_insn[19]}}, pcpi_insn[19:15]} : {{32{pcpi_rs1[31]}}, pcpi_rs1[0+:8]}}} & arith_remaining_mask & vregs_v0_64) | (vregs_rdata2 & arith_remaining_mask & ~vregs_v0_64);
										endcase
									// vregs_wdata <= tmp_vregs_wdata;
								end
								vregs_wen <= 1;
							end
							if (arith_init) begin
								arith_remaining <= vl;
								arith_init <= 0;
								reg_index <= 0;
							end else begin
								if (arith_remaining <= (VLEN >> (vsew+3))) begin
									arith_remaining <= 0;
									arith_init <= 1;
									pcpi_ready <= 1;
									pcpi_wait <= 0;
									reg_index <= 0;
								end else begin
									arith_remaining <= arith_remaining > (VLEN >> (vsew+3)) ? arith_remaining - (VLEN >> (vsew+3)) : 0;
									pcpi_ready <= 0;
									pcpi_wait <= 1;
									reg_index <= reg_index + 1;
								end
							end
						end else begin
							pcpi_wait <= 1;
							pcpi_ready <= 0;
						end
					end else if (instr_arith || instr_mask) begin
						alu_run <= !arith_done && (arith_init || (arith_remaining >= 1 << NB_LANES) || arith_step != ((1 << (vsew+3-LANE_WIDTH)) - 1));
						if (alu_run && arith_instr_valid) begin
							tmp_vregs_wdata = vregs_wdata_acc;
							if (instr_vcpop) begin
								if (arith_res[0])
									pcpi_rd <= pcpi_rd + arith_vd[0 +: 32];
							end else if (instr_vfirst) begin
								if (arith_res[0])
									pcpi_rd <= &pcpi_rd ? arith_vd[0 +: 32] : pcpi_rd;
							end else if (instr_vmset) begin
								for (lane_num = 0; lane_num < (1 << NB_LANES); lane_num = lane_num + 1) begin
									if (arith_res[lane_num] && arith_remaining > lane_num && (vm || vregs_v0[arith_regi[lane_num*17 +: 17]])) begin
										tmp_vregs_wdata[arith_regi[lane_num*17 +: 17]] = arith_vd[lane_num<<6];
									end
								end
								if (!arith_done) vregs_wdata_acc <= tmp_vregs_wdata;
								if (!arith_done) vregs_wdata <= tmp_vregs_wdata;
								vregs_wen <= !arith_done;
								pcpi_rd <= 0;
							end else begin
								if (vsew + 3 < LANE_WIDTH || instr_viota) begin // lane larger than vsew (if arith instr, else always true)
									for (lane_num = 0; lane_num < (1 << NB_LANES); lane_num = lane_num + 1) begin
										if (arith_res[lane_num] && arith_remaining > lane_num && (vm || vregs_v0[(arith_regi[lane_num*17 +: 17] >> (vsew+3)) + (VLEN >> (vsew+3)) * reg_index])) begin
											/* if (instr_viota)
												$display("vregs[%d:8] = %h", arith_regi[lane_num*17 +: 17], arith_vd[lane_num << 6 +: (1 << (0+3))]); */
											case (vsew)
												3'b000: begin
													tmp_vregs_wdata[arith_regi[lane_num*17 +: 17] +: (1 << (0+3))] = arith_vd[lane_num << 6 +: (1 << (0+3))];
												end
												3'b001: begin
													tmp_vregs_wdata[arith_regi[lane_num*17 +: 17] +: (1 << (1+3))] = arith_vd[lane_num << 6 +: (1 << (1+3))];
												end
												3'b010: begin
													tmp_vregs_wdata[arith_regi[lane_num*17 +: 17] +: (1 << (2+3))] = arith_vd[lane_num << 6 +: (1 << (2+3))];
												end
												3'b011: begin
													tmp_vregs_wdata[arith_regi[lane_num*17 +: 17] +: (1 << (3+3))] = arith_vd[lane_num << 6 +: (1 << (3+3))];
												end
											endcase
										end
									end
								end else begin
									for (lane_num = 0; lane_num < (1 << NB_LANES); lane_num = lane_num + 1) begin
										if (arith_res[lane_num] && arith_remaining > lane_num && (vm || vregs_v0[(arith_regi[lane_num*17 +: 17] >> (vsew+3)) + (VLEN >> (vsew+3)) * reg_index])) begin
											tmp_vregs_wdata[arith_regi[lane_num*17 +: 17] +: SHIFTED_LANE_WIDTH] = arith_vd[lane_num << 6 +: SHIFTED_LANE_WIDTH];
										end
									end
								end
								if (!arith_done) vregs_wdata_acc <= tmp_vregs_wdata;
								vregs_wdata <= tmp_vregs_wdata;
								vregs_wen <= !arith_done;
								pcpi_rd <= 0;
							end
						end
						if (!arith_instr_valid) begin
							pcpi_ready <= 0;
							pcpi_wait <= 0;
						end else begin
							if (!arith_init && arith_remaining <= 1 << NB_LANES && ((!instr_viota && (instr_arith || instr_mask)) ? (vsew+3 <= LANE_WIDTH || arith_step == ((1 << (vsew+3-LANE_WIDTH)) - 1)) : 1)) begin // all vec done
								reg_index <= 0;
								arith_remaining <= 0;
								pcpi_wait <= 0;
								arith_init <= 1;
								pcpi_ready <= 1;
								pcpi_wr <= instr_vcpop | instr_vfirst;
							end else begin // 1 vec done
								if (arith_done)
									reg_index <= reg_index + 1;
								else
									reg_index <= reg_index;							
								pcpi_wait <= 1;
								pcpi_ready <= 0;
								pcpi_wr <= 0;
							end

							if (arith_init) begin
								// arith_remaining <= instr_mask_unmaskable ? (VLEN >> (vsew+3)) : vl; // whole reg mask op
								// arith_remaining <= instr_mask ? (VLEN >> (vsew+3)) : vl; // whole reg mask op
								arith_remaining <= instr_mask && !instr_viota ? vl >> (vsew+3) : vl; // whole reg mask op
								arith_step <= 0;
								arith_init <= 0;
							end else if (alu_run)
								if (!arith_done)
									if (instr_viota || (vsew+3 <= LANE_WIDTH || arith_step == ((1 << (vsew+3-LANE_WIDTH)) - 1))) begin
										arith_step <= 0;
										arith_remaining <= arith_remaining - (1 << nb_lanes);
									end else /* if (!arith_done) */ begin
										arith_step <= arith_step + 1;
										arith_remaining <= arith_remaining;
									end
						end
					end
				end
			end
			pcpi_trap_in_q <= pcpi_trap_in || pcpi_trap_out;
		end
	end
	
	always @(posedge clk) begin
		if (!resetn) begin
			if (REGS_INIT_ZERO) vregs_v0 <= 0;
		end else
			if (vregs_waddr_q == 0 && vregs_wen)
				vregs_v0 = vregs_wdata;
	end

	rvv_alu_wrapper #(
		.VLEN(VLEN),
		.LANE_WIDTH(LANE_WIDTH),
		.NB_LANES(NB_LANES)
	) valu_w (
		.clk(clk),
		.resetn(resetn),
		.opcode(pcpi_insn[31:26]),
		.instr_mask(instr_mask),
		.run(alu_run),
		.vs1_index(vs1),
		.vs1(alu_vs1),
		.vs2(alu_vs2),
		.vsew(vsew),
		.op_type({alu_vi,alu_vx,alu_vv}),
		.vl(vl),
		.arith_remaining(arith_remaining),
		.vd(arith_vd),
		.regi(arith_regi),
		.res(arith_res),
		.done_out(arith_done),
		.instr_valid(arith_instr_valid),
		.arith_instr_signed(arith_instr_signed)
	);
endmodule

module rvv_vregs #(
	parameter [16:0] VLEN = 17'd 128,
	parameter REGS_INIT_ZERO = 1'b1
) (
	input clk,
	input wen,
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
		if (wen) vregs[waddr] = wdata;
	
	assign rdata1 = vregs[raddr1];
	assign rdata2 = vregs[raddr2];
endmodule
