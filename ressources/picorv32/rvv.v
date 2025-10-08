`ifdef DEBUG_RVV
  `define debug_rvv(debug_command) debug_command
`else
  `define debug_rvv(debug_command)
`endif

/***************************************************************
 * picorv32_pcpi_rvv
 ***************************************************************/

module picorv32_pcpi_rvv #(
	parameter [0:0] ENABLE_REGS_16_31 = 1,
	parameter [0:0] REGS_INIT_ZERO = 0,
	parameter [9:0] VLEN = 10'd 128,
	// arithmetic op
	parameter [1:0] NB_LANES = 2'b01, // 2^NB_LANES lanes used for arith op
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
	localparam integer VLENB = VLEN/8;
	localparam integer regfile_size = (ENABLE_REGS_16_31 ? 32 : 16);
	localparam [7:0] SHIFTED_LANE_WIDTH = 1 << LANE_WIDTH;
	localparam integer VLEN_ARITH_IMM = VLEN-5;
	localparam integer VLEN_ARITH_RS  = VLEN-32;

	assign pcpi_is_rvv_insn = |{instr_cfg, instr_mem, instr_arith};
	assign pcpi_mem_init = instr_mem && !pcpi_mem_op && byte_index == 0 && reg_index == 0 && mem_sending && !pcpi_ready && !pcpi_mem_trans_done;

	// assign pcpi_mem_wen = instr_mem ? !pcpi_mem_ifetch && mem_sending : 0;
	// assign pcpi_mem_addr = pcpi_rs1 + mem_offset_q;

	wire instr_run = pcpi_valid && !pcpi_ready; // 1 if should execute instruction

	reg [31:0] vxsat, vxrm, vcsr, vl, vlmax, vtype;
	reg [31:0] vstart; // reg must be as short as possible
	reg [VLEN-1:0] vregs [regfile_size-1:0] /* synthesis preserve */;

	wire	   vill  = vtype[31];
	wire 	   vma   = vtype[7];
	wire 	   vta   = vtype[6];
	wire [2:0] vsew  = vtype[5:3];
	wire [2:0] vlmul = vtype[2:0];

	wire [4:0] vs1 = pcpi_insn[19:15];
	wire [4:0] vs2 = pcpi_insn[24:20];
	wire [4:0] vd = pcpi_insn[11:7];

	wire [VLEN-1:0] arith_vs1 = arith_vv ? vregs[vs1 + reg_index] :
							arith_vi ? {{VLEN_ARITH_IMM{1'b0}},pcpi_insn[19:15]} :
							arith_vx ? {{VLEN_ARITH_RS{1'b0}},pcpi_rs1} :
							0;

	reg	should_trap;
	reg pcpi_trap_in_q;
	
	// VSETVL
	reg instr_vsetvli, instr_vsetivli, instr_vsetvl;
	wire instr_cfg = |{instr_vsetvli, instr_vsetivli, instr_vsetvl};
	reg [31:0] avl;

	reg [9:0]  byte_index; // index in the vector register, same size as VLEN parameter
	reg [4:0]  reg_index; // index of the vector register

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
	reg	[3:0]  mem_strb_q;
	reg	[31:0] mem_wdata_q;
	reg [9:0]  mem_indexed_byte_index; // used for indexed accesses | index in the offset vector register, same size as VLEN parameter
	reg [4:0]  mem_indexed_reg_index; // used for indexed accesses | index of the offset vector register
	reg [2:0]  mem_indexed_sew; // sew of indices in an indexed insn
	reg [3:0]  mem_seg_nfields; // number of segments in a segment load / store
	reg [2:0]  mem_seg_i; // identifies the current field vector
	reg [4:0]  mem_seg_index; // vector index in vregs, temp variable
	reg [2:0]  mem_seg_n_access;

	reg [VLEN-1:0] temp_vreg; // avoid errors
	integer index; // avoid errors
	integer offset; // avoid errors
	integer tmp_offset;
	integer offset_incr;
	integer for_i;
	
	// ARITHMETIC OPERATIONS
	reg instr_arith;
	reg arith_vv;
	reg arith_vi;
	reg arith_vx;
	reg alu_run;
	wire [(64 << NB_LANES) - 1:0] arith_vd;
	wire [(10 << NB_LANES) - 1:0] arith_regi;
	wire [(1 << NB_LANES) - 1:0] arith_res;
	wire arith_done;
	wire arith_instr_valid;
	reg [31:0] arith_remaining;
	reg arith_init;
	reg [2:0] arith_step;

	wire [3:0] tmp_nb_lanes = `min(VLEN>>(vsew+3), 1 << NB_LANES);
    wire [1:0] nb_lanes = tmp_nb_lanes[3] ? 2'b11 :
                          tmp_nb_lanes[2] ? 2'b10 :
                          tmp_nb_lanes[1] ? 2'b01 :
                          2'b00;

	integer lane_num;
	
	integer init_reg_i;
	
	integer i;
	initial begin
		if (REGS_INIT_ZERO) begin
			for (i = 0; i < regfile_size; i = i+1)
				vregs[i] = 0;
		end
	end


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
				should_trap = should_trap || (pcpi_insn[11:7] + (vl / (VLEN >> mem_sew)) >= regfile_size);
			end else begin
				mem_sew = vtype[5:3] + 3;
				case (pcpi_insn[14:12])
					3'b000: mem_indexed_sew = 3'b011;
					3'b101: mem_indexed_sew = 3'b100;
					3'b110: mem_indexed_sew = 3'b101; 
					3'b111: mem_indexed_sew = 3'b110;
				endcase
				should_trap = should_trap || (pcpi_insn[11:7] + (vl / (VLEN >> mem_sew)) >= regfile_size) || (pcpi_insn[24:20] + (vl / (VLEN >> mem_indexed_sew)) >= regfile_size);
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
		if (resetn && pcpi_insn[6:0] == 7'b1010111 && pcpi_insn[14:12] != 3'b111) begin
			instr_arith = 1;
			case (pcpi_insn[14:12])
				3'b000, 3'b001, 3'b010: arith_vv = 1;
				3'b011: arith_vi = 1;
				3'b100, 3'b110: arith_vx = 1;
				3'b101: should_trap = 1; // float op
			endcase
		end
	end

	always @(posedge clk) begin
		pcpi_ready <= 0;
		pcpi_wait <= 0;
		pcpi_wr <= 0;
		pcpi_rd <= 'bx;
		pcpi_mem_wen <= 0; // memory access
		pcpi_mem_addr <= pcpi_rs1 + mem_offset_q; // mem addr
		pcpi_mem_op <= 0;
		pcpi_mem_strb <= 0;
		pcpi_mem_ftrans <= 0;
		pcpi_trap_out <= 0;
		
		// arith
		alu_run <= 0;
		
		if (!resetn) begin
		
			for (init_reg_i = 0; init_reg_i < regfile_size; init_reg_i = init_reg_i + 1)
				vregs[init_reg_i] <= 0;
		
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
			reg_index = 0;
			mem_indexed_byte_index = 0;
			mem_indexed_reg_index = 0;
			mem_sending = 1;
			last_op <= 0;
			mem_stride_mask <= 12'h000;
			mem_stride_i <= 0;
			mem_offset_q <= 0;
			mem_strb_q <= 0;
			mem_wdata_q <= 0;
			offset <= 0;
			offset_incr = 0;
			mem_seg_i = 0;
			mem_seg_index = 0;
			mem_seg_n_access = 0;

			// arith
			arith_remaining <= 0;
			arith_init <= 1;
			arith_step <= 0;
		end else begin
			/* if (should_trap)
				pcpi_trap_out <= 1; */
			
			if (should_trap || pcpi_trap_in || pcpi_trap_out) begin
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
						pcpi_mem_op <= 1;
						pcpi_rd <= 'bx;
						pcpi_wait <= 1;
						pcpi_ready <= 0;
						pcpi_mem_wen <= 0;

						mem_stride_amount = (instr_mem_unit || instr_mem_whole_reg) ? (1 << (mem_sew-3)) : pcpi_rs2;

						if (instr_mem_indexed) begin
							temp_vreg = vregs[pcpi_insn[24:20] + mem_indexed_reg_index];
							tmp_offset = mem_indexed_byte_index << mem_indexed_sew;
							mem_offset_q = 0;
							case (mem_indexed_sew)
								3'b011: mem_offset_q[0 +: 8] = temp_vreg[tmp_offset +: 8]; // 8 bits index
								3'b100: mem_offset_q[0 +: 16] = temp_vreg[tmp_offset +: 16]; // 16 bits index
								3'b101: mem_offset_q[0 +: 32] = temp_vreg[tmp_offset +: 32]; // 32 bits index
								3'b110: mem_offset_q[0 +: 32] = temp_vreg[tmp_offset +: 32]; // 64 bits index but we only keep 32 LSB
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
								// pcpi_mem_wen <= 1;
								pcpi_mem_wen <= !pcpi_mem_ifetch;
								pcpi_mem_addr <= pcpi_rs1 + mem_offset_q;

								mem_sending <= pcpi_mem_ifetch ? 1 : 0;
								pcpi_wr <= 0;
								pcpi_wait <= 1;
								pcpi_ready <= 0;
							end else if (pcpi_mem_done) begin
								pcpi_mem_ftrans <= 0;
								index = pcpi_insn[11:7] + reg_index + (mem_seg_i << (vtype[2] ? 0 : vtype[2:0]));
								
								if (mem_stride_mask[{mem_stride_i, 2'b00}]) begin
									`debug_rvv($display("byte0");)
									tmp_offset = (offset + offset_incr) << 3; // convert from byte to bit index
									vregs[index][tmp_offset +: 8] <= pcpi_mem_rdata[0 +: 8];
									offset_incr = offset_incr + 1;
								end
								if (mem_stride_mask[{mem_stride_i, 2'b00} + 1]) begin
									`debug_rvv($display("byte1");)
									tmp_offset = (offset + offset_incr) << 3; // convert from byte to bit index
									vregs[index][tmp_offset +: 8] <= pcpi_mem_rdata[8 +: 8];
									offset_incr = offset_incr + 1;
								end
								if (mem_stride_mask[{mem_stride_i, 2'b00}+2]) begin
									`debug_rvv($display("byte2");)
									tmp_offset = (offset + offset_incr) << 3; // convert from byte to bit index
									vregs[index][tmp_offset +: 8] <= pcpi_mem_rdata[16 +: 8];
									offset_incr = offset_incr + 1;
								end
								if (mem_stride_mask[{mem_stride_i, 2'b00} + 3]) begin
									`debug_rvv($display("byte3");)
									tmp_offset = (offset + offset_incr) << 3; // convert from byte to bit index
									vregs[index][tmp_offset +: 8] <= pcpi_mem_rdata[24 +: 8];
									offset_incr = offset_incr + 1;
								end

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
												reg_index = reg_index + 1;
											end else begin
												reg_index = reg_index;
												byte_index = byte_index + 1;
											end
										end else
											mem_seg_i <= mem_seg_i + 1; // TODO ERREUR ?
										// update indexed indices
										if (instr_mem_indexed && (mem_seg_i == mem_seg_nfields - 1)) begin
											if (mem_indexed_byte_index == (VLEN >> mem_indexed_sew) - 1) begin
												mem_indexed_byte_index = 0;
												mem_indexed_reg_index = mem_indexed_reg_index + 1;
											end else begin
												mem_indexed_byte_index = mem_indexed_byte_index + 1;
												mem_indexed_reg_index = mem_indexed_reg_index;
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
									reg_index = 0;
									mem_indexed_byte_index = 0;
									mem_indexed_reg_index = 0;
									offset <= 0;
									offset_incr = 0;
									mem_stride_i = 0;
									mem_seg_i = 0;
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
								index = pcpi_insn[11:7] + reg_index + (mem_seg_i << (vtype[2] ? 0 : vtype[2:0]));

								temp_vreg = vregs[index];
								mem_strb_q = 0;
								mem_wdata_q = 0;
								if (mem_stride_mask[{mem_stride_i, 2'b00}]) begin
									tmp_offset = (offset + offset_incr) << 3;
									mem_wdata_q[0 +: 8] = temp_vreg[tmp_offset +: 8];
									mem_strb_q[0] = 1'b1;
									offset_incr = offset_incr + 1;
								end
								if (mem_stride_mask[{mem_stride_i, 2'b00} + 1]) begin
									tmp_offset = (offset + offset_incr) << 3;
									mem_wdata_q[8 +: 8] = temp_vreg[tmp_offset +: 8];
									mem_strb_q[1] = 1'b1;
									offset_incr = offset_incr + 1;
								end
								if (mem_stride_mask[{mem_stride_i, 2'b00}+2]) begin
									tmp_offset = (offset + offset_incr) << 3;
									mem_wdata_q[16 +: 8] = temp_vreg[tmp_offset +: 8];
									mem_strb_q[2] = 1'b1;
									offset_incr = offset_incr + 1;
								end
								if (mem_stride_mask[{mem_stride_i, 2'b00} + 3]) begin
									tmp_offset = (offset + offset_incr) << 3;
									mem_wdata_q[24 +: 8] = temp_vreg[tmp_offset +: 8];
									mem_strb_q[3] = 1'b1;
									offset_incr = offset_incr + 1;
								end

								if ((mem_stride_i==0 && mem_stride_mask[7:4] == 0) || (mem_stride_i==1 && mem_stride_mask[11:8] == 0) || mem_stride_i==2) begin
									if (!pcpi_mem_ifetch && mem_seg_i == mem_seg_nfields - 1 || instr_mem_whole_reg)
										offset <= offset + offset_incr;
									offset_incr = 0;
								end

								pcpi_mem_strb <= mem_strb_q;
								pcpi_mem_wdata <= mem_wdata_q;

								mem_sending <= pcpi_mem_ifetch ? 1 : 0;
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
												reg_index = reg_index + 1;
											end else begin
												reg_index = reg_index;
												byte_index = byte_index + 1;
											end
										end else
											mem_seg_i <= mem_seg_i + 1;
										// update indexed indices
										if (instr_mem_indexed && (mem_seg_i == mem_seg_nfields - 1)) begin
											if (mem_indexed_byte_index == (VLEN >> mem_indexed_sew) - 1) begin
												mem_indexed_byte_index = 0;
												mem_indexed_reg_index = mem_indexed_reg_index + 1;
											end else begin
												mem_indexed_byte_index = mem_indexed_byte_index + 1;
												mem_indexed_reg_index = mem_indexed_reg_index;
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
									reg_index = 0;
									mem_indexed_byte_index = 0;
									mem_indexed_reg_index = 0;
									offset <= 0;
									offset_incr = 0;
									mem_stride_i = 0;
									mem_seg_i = 0;
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
					end else if (instr_arith) begin
						alu_run <= !arith_done && (((arith_init ? vl : arith_remaining) >= 1 << NB_LANES) || arith_step != ((1 << (vsew+3-LANE_WIDTH)) - 1));
						if (alu_run && arith_instr_valid) begin
							temp_vreg = vregs[vd + reg_index];
							if (vsew + 3 < LANE_WIDTH) begin // lane larger than vsew
								for (lane_num = 0; lane_num < (1 << NB_LANES); lane_num = lane_num + 1) begin
									if (arith_res[lane_num] && arith_remaining > lane_num) begin
										case (vsew)
											3'b000:
												temp_vreg[arith_regi[lane_num*10 +: 10] +: (1 << (0+3))] = arith_vd[lane_num << 6 +: (1 << (0+3))];
											3'b001:
												temp_vreg[arith_regi[lane_num*10 +: 10] +: (1 << (1+3))] = arith_vd[lane_num << 6 +: (1 << (1+3))];
											3'b010:
												temp_vreg[arith_regi[lane_num*10 +: 10] +: (1 << (2+3))] = arith_vd[lane_num << 6 +: (1 << (2+3))];
											3'b011:
												temp_vreg[arith_regi[lane_num*10 +: 10] +: (1 << (3+3))] = arith_vd[lane_num << 6 +: (1 << (3+3))];
										endcase
									end
								end
							end else begin
								for (lane_num = 0; lane_num < (1 << NB_LANES); lane_num = lane_num + 1) begin
									if (arith_res[lane_num] && arith_remaining > lane_num) begin
										temp_vreg[arith_regi[lane_num*10 +: 10] +: SHIFTED_LANE_WIDTH] = arith_vd[lane_num << 6 +: SHIFTED_LANE_WIDTH];
									end
								end
							end

							vregs[vd + reg_index] = temp_vreg;
						end
						if (!arith_instr_valid) begin
							pcpi_ready <= 0;
							pcpi_wait <= 0;
						end else begin
							if (!arith_init && arith_remaining <= 1 << NB_LANES && (vsew+3 <= LANE_WIDTH || arith_step == ((1 << (vsew+3-LANE_WIDTH)) - 1))) begin // all vec done
								reg_index <= 0;
								arith_remaining <= 0;
								pcpi_wait <= 0;
								arith_init <= 1;
								pcpi_ready <= 1;
							end else begin // 1 vec done
								if (arith_done)
									reg_index <= reg_index + 1;
								else
									reg_index <= reg_index;							
								pcpi_wait <= 1;
								pcpi_ready <= 0;
							end

							if (arith_init) begin
								arith_remaining <= vl;
								arith_step <= 0;
								arith_init <= 0;
							end else if (alu_run)
								if (vsew+3 <= LANE_WIDTH || arith_step == ((1 << (vsew+3-LANE_WIDTH)) - 1)) begin
									arith_step <= 0;
									arith_remaining <= arith_remaining - (1 << nb_lanes);
								end else begin
									arith_step <= arith_step + 1;
									arith_remaining <= arith_remaining;
								end

							pcpi_rd <= 0;
							pcpi_wr <= 0;
						end
					end
				end
			end
			pcpi_trap_in_q <= pcpi_trap_in || pcpi_trap_out;
		end
	end

	rvv_alu_wrapper #(
		.VLEN(VLEN),
		.LANE_WIDTH(LANE_WIDTH),
		.NB_LANES(NB_LANES)
	) valu_w (
		.clk(clk),
		.resetn(resetn),
		.opcode(pcpi_insn[31:26]),
		.run(alu_run),
		.vs1(arith_vs1),
		.vs2(vregs[vs2 + reg_index]),
		.vsew(vsew),
		.op_type({arith_vi,arith_vx,arith_vv}),
		.vd(arith_vd),
		.regi(arith_regi),
		.res(arith_res),
		.done_out(arith_done),
		.instr_valid(arith_instr_valid)
	);
endmodule
