always @* begin
		// vsetvl
		instr_vsetvli = 0;
		instr_vsetivli = 0;
		instr_vsetvl = 0;
		
		// mem
		instr_vload = 0;
		instr_vstore = 0;
		mem_transfer_n = 0;
		mem_last_transfer_len = 0;
		instr_mem_unit = 0;
		instr_mem_const = 0;
		mem_sew = 0;
		// remaining_mem_transfer = 0;
		// current_mem_addr = 0;
		// current_reg_byte = 0;

		// config
		if (resetn && pcpi_insn[14:12] == 3'b111 && pcpi_insn[6:0] == 7'b1010111) begin
			if (pcpi_insn[31] == 1'b0)
				instr_vsetvli = 1;
			else if (pcpi_insn[31:30] == 2'b11)
				instr_vsetivli = 1;
			else if (pcpi_insn[31:25] == 7'b1000000)
				instr_vsetvl = 1;
		end

		// mem decod
		if (resetn && (pcpi_insn[6:0] == 7'b0000111 || pcpi_insn[6:0] == 7'b0100111)) begin
			instr_vload = !pcpi_insn[5];
			instr_vstore = pcpi_insn[5];

			// TODO: only works for unit-stride addressing mode
			case (pcpi_insn[27:26]) 
				2'b00: begin
					// unit-stride
					instr_mem_unit = 1;
					instr_mem_const = 0;
					case (pcpi_insn[14:12])
						3'b000: begin mem_transfer_n = vl >> 2; mem_last_transfer_len = vl[1:0]; end // 8 bits elem
						3'b101: begin mem_transfer_n = vl >> 1; mem_last_transfer_len = vl[0] << 1; end // 16 bits elem
						3'b110: begin mem_transfer_n = vl; mem_last_transfer_len = 0; end // 32 bits elem
						3'b111: begin mem_transfer_n = vl << 1; mem_last_transfer_len = 0; end // 64 bits elem
					endcase
					mem_sew = 3'b101;
					if (mem_last_transfer_len != 0)
						mem_transfer_n += 1;
				end
				2'b10: begin
					// constant stride
					instr_mem_unit = 0;
					instr_mem_const = 1;
					mem_transfer_n = pcpi_insn[14:12] == 3'b111 ? vl << 1 : vl;
					case (pcpi_insn[14:12])
						3'b000: mem_sew = 3'b011;
						3'b101: mem_sew = 3'b100;
						3'b110: mem_sew = 3'b101; 
						3'b111: mem_sew = 3'b110;
					endcase
				end
			endcase
		end
	end

	always @(posedge clk) begin
		pcpi_ready <= 0;
		pcpi_wait <= 0;
		pcpi_wr <= 0;
		pcpi_rd <= 'bx;
		// pcpi_mem_load <= 0; // 1 if vector memory load
		// pcpi_mem_store <= 0; // 1 if vector memory store
		pcpi_mem_wen <= 0; // memory access
		pcpi_mem_base <= 0; // base reg addr
		pcpi_mem_offset <= 0; // mem addr offset
		pcpi_mem_op <= 0;
		pcpi_mem_ftrans <= 0;
		
		if (!resetn) begin
			vstart = 0;
			vxsat = 0;
			vxrm = 0;
			vcsr = 0;
			vl = 0;
			vtype = 32'h8000_0000;
			// vsetvl
			vlmax = 0;
			avl = 0;
			mem_byte_index = 0;
			mem_reg_index = 0;
			mem_sending = 1;
			mem_not_last_transfer <= 0;
			mem_stride_mask <= 12'h000;
			mem_stride_i <= 0;
			mem_offset_q <= 0;
		end else if (instr_run) begin
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
					3'b100: begin vtype[31] = 1; vlmax = 0; end
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
				/* $display("\n------------------");
				$display("RVV_insn  : %b", pcpi_insn);
				$display("RVV_vtype : %b", vtype);
				$display("RVV_vlmax : %b", vlmax);
				$display("RVV_tmp : %b", ($unsigned(-$signed(vtype[2:0]))));
				$display("------------------"); */
			end else if (mem_instr) begin
				pcpi_mem_op <= 1;
				if (instr_vload) begin
					pcpi_rd <= 'bx;
					pcpi_wait <= 1;
					pcpi_ready <= 0;
					pcpi_mem_wen <= 0;
					if (mem_sending) begin
						// send addr to main proc
						pcpi_mem_ftrans <= (mem_byte_index == 0 && mem_reg_index == 0 && mem_stride_i == 0);
						// pcpi_mem_load <= 1;
						pcpi_mem_wen <= 1;
						pcpi_mem_base <= pcpi_rs1;
						if (instr_mem_unit)
							pcpi_mem_offset <= (mem_byte_index << 2) + (mem_reg_index << (VLEN >> mem_sew));
						else if (instr_mem_const) begin
							if (mem_stride_i == 0) begin
								mem_offset_q = (pcpi_rs2 * (mem_byte_index + (mem_reg_index * (VLEN >> mem_sew)))) + (mem_stride_i << 2);
								case (mem_offset_q[1:0])
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
								pcpi_mem_offset <= mem_offset_q;
								$display("mem_offset_q : %d", mem_offset_q);
							end
						end

						mem_sending <= pcpi_mem_ifetch ? 1 : 0;
						pcpi_wr <= 0;
						pcpi_wait <= 1;
						pcpi_ready <= 0;
					end else if (pcpi_mem_done) begin
						pcpi_mem_ftrans <= 0;
						index = pcpi_insn[11:7] + mem_reg_index;
						if (instr_mem_unit) begin
							offset = mem_byte_index << 5;
							mem_not_last_transfer = mem_reg_index * (VLEN >> 5) + mem_byte_index < mem_transfer_n - 1;
						end	else if (instr_mem_const) begin
							offset = mem_byte_index << (mem_sew-2);
							mem_not_last_transfer = (mem_byte_index + (mem_reg_index * (VLEN >> mem_sew))) < vl - 1;
						end

						if (mem_not_last_transfer) begin
							// not last transfer
							temp_vreg = vregs[index];
							if (instr_mem_unit)
								temp_vreg[offset +: 32] = pcpi_mem_rdata;
							else if (instr_mem_const) begin
								if (mem_stride_mask[mem_stride_i<<2]) begin
									$display("byte 1 | val : %b", mem_stride_mask[mem_stride_i<<2]);
									tmp_offset = offset << 2;
									temp_vreg[tmp_offset +: 8] = pcpi_mem_rdata[0 +: 8];
								end
								if (mem_stride_mask[(mem_stride_i<<2) + 1]) begin
									$display("byte 2 | val : %b", mem_stride_mask[(mem_stride_i<<2)+1]);
									tmp_offset = offset << 2;
									temp_vreg[tmp_offset +: 8] = pcpi_mem_rdata[8 +: 8];
								end
								if (mem_stride_mask[(mem_stride_i<<2)+2]) begin
									$display("byte 3 | val : %b", mem_stride_mask[(mem_stride_i<<2)+2]);
									tmp_offset = offset << 2;
									temp_vreg[tmp_offset +: 8] = pcpi_mem_rdata[16 +: 8];
								end
								if (mem_stride_mask[(mem_stride_i<<2) + 3]) begin
									$display("byte 4 | val : %b", mem_stride_mask[(mem_stride_i<<2)+3]);
									tmp_offset = offset << 2;
									temp_vreg[tmp_offset +: 8] = pcpi_mem_rdata[24 +: 8];
								end
								$display("mem_stride_mask : %h | mem_stride_i : %b", mem_stride_mask, mem_stride_i);
								$display("pcpi_mem_rdata : %h", pcpi_mem_rdata);
								$display("tmp_offset : %d | offset : %d", tmp_offset, offset);
								$display("temp_vreg : %h", temp_vreg);
							end
							vregs[index] = temp_vreg;

							// update indices
							if ((instr_mem_const && ((mem_stride_i==0 && mem_stride_mask[1] == 0) || (mem_stride_i==1 && mem_stride_mask[2] == 0) || mem_stride_i==2)) || instr_mem_unit) begin
								mem_stride_i = 0;
								if ((instr_mem_unit && (mem_byte_index+1) << 5 == VLEN) || (instr_mem_const && ((offset << 2) + (1 << mem_sew)) == VLEN)) begin
									mem_byte_index = 0;
									mem_reg_index += 1;
								end else begin
									mem_byte_index += 1;
									mem_reg_index = mem_reg_index;
								end
							end else
								mem_stride_i += 1;
						
							pcpi_wait <= 1;
							pcpi_ready <= 0;
						end else begin
							// last transfer
							temp_vreg = vregs[index];
							tmp_offset = offset << 2;
							if (instr_mem_const) begin
								if (mem_stride_mask[mem_stride_i<<2]) begin
									$display("byte 1 | val : %b", mem_stride_mask[mem_stride_i<<2]);
									tmp_offset = offset << 2;
									temp_vreg[tmp_offset +: 8] = pcpi_mem_rdata[0 +: 8];
								end
								if (mem_stride_mask[(mem_stride_i<<2) + 1]) begin
									$display("byte 2 | val : %b", mem_stride_mask[(mem_stride_i<<2)+1]);
									tmp_offset = offset << 2;
									temp_vreg[tmp_offset +: 8] = pcpi_mem_rdata[8 +: 8];
								end
								if (mem_stride_mask[(mem_stride_i<<2)+2]) begin
									$display("byte 3 | val : %b", mem_stride_mask[(mem_stride_i<<2)+2]);
									tmp_offset = offset << 2;
									temp_vreg[tmp_offset +: 8] = pcpi_mem_rdata[16 +: 8];
								end
								if (mem_stride_mask[(mem_stride_i<<2) + 3]) begin
									$display("byte 4 | val : %b", mem_stride_mask[(mem_stride_i<<2)+3]);
									tmp_offset = offset << 2;
									temp_vreg[tmp_offset +: 8] = pcpi_mem_rdata[24 +: 8];
								end
							end else if (instr_mem_unit)
								case (mem_last_transfer_len)
									2'b00: temp_vreg[offset +:32] = pcpi_mem_rdata[31:0];
									2'b01: temp_vreg[offset +:8] = pcpi_mem_rdata[7:0];
									2'b10: temp_vreg[offset +:16] = pcpi_mem_rdata[15:0];
									2'b11: temp_vreg[offset +:24] = pcpi_mem_rdata[23:0];
								endcase
							vregs[index] = temp_vreg;

							pcpi_wait <= 0;
							pcpi_ready <= 1;
							pcpi_mem_op <= 0;
							// update indices
							mem_byte_index = 0;
							mem_reg_index = 0;
						end

						$display("v1 : %h", vregs[1]);
						$display("v2 : %h", vregs[2]);
						$display("v3 : %h", vregs[3]);
						$display("v4 : %h", vregs[4]);
						$display("v5 : %h", vregs[5]);
						$display("v6 : %h", vregs[6]);
						$display("v7 : %h", vregs[7]);
						$display("v8 : %h\n", vregs[8]);
						
						mem_sending <= 1;
						// outputs
						// pcpi_mem_load <= 1;
						pcpi_mem_wen <= 0;
						pcpi_mem_base <= 0;
						pcpi_mem_offset <= 0;						
						pcpi_wr <= 0;
					end
				end else if (instr_vstore) begin
					pcpi_rd <= 'bx;
					pcpi_wait <= 1;
					pcpi_ready <= 0;
					pcpi_mem_wen <= 0;
					if (mem_sending) begin
						pcpi_mem_ftrans <= (mem_byte_index == 0 && mem_reg_index == 0 && mem_stride_i == 0);
						// send data to main proc
						index = pcpi_insn[11:7] + mem_reg_index;
						offset = mem_byte_index << 5;
						mem_not_last_transfer = mem_reg_index * (VLEN >> 5) + mem_byte_index < mem_transfer_n - 1;
						if (mem_not_last_transfer) begin
							// not last transfer
							pcpi_mem_wdata = vregs[index][offset +: 32];
							pcpi_mem_strb = 4'b1111;
						end else begin
							// last transfer
							case (mem_last_transfer_len)
								2'b00: begin
									pcpi_mem_wdata = vregs[index][offset +: 32];
									pcpi_mem_strb = 4'b1111;
								end
								2'b01: begin
									pcpi_mem_wdata = {24'h000000, vregs[index][offset +: 8]};
									pcpi_mem_strb = 4'b0001;
								end
								2'b10: begin
									pcpi_mem_wdata = {16'h0000, vregs[index][offset +: 16]};
									pcpi_mem_strb = 4'b0011;
								end
								2'b11: begin
									pcpi_mem_wdata = {8'h00, vregs[index][offset +: 24]};
									pcpi_mem_strb = 4'b0111;
								end
							endcase
						end						
						mem_sending <= pcpi_mem_ifetch ? 1 : 0;
						// outputs
						// pcpi_mem_store <= 1;
						pcpi_mem_wen <= 1;
						pcpi_mem_base <= pcpi_rs1;
						pcpi_mem_offset <= (mem_byte_index << 2) + (mem_reg_index << (VLEN >> 5));
						pcpi_wait <= 1;
						pcpi_ready <= 0;
					end else if (pcpi_mem_done) begin
						pcpi_mem_ftrans <= 0;
						if (mem_not_last_transfer) begin
							// not last tranfer
							pcpi_wait <= 1;
							pcpi_ready <= 0;
							// update indices
							if ((mem_byte_index+1) << 5 == VLEN) begin
								mem_byte_index = 0;
								mem_reg_index += 1;
							end else begin
								mem_byte_index += 1;
								mem_reg_index = mem_reg_index;
							end
						end else begin
							// last transfer
							pcpi_wait <= 0;
							pcpi_ready <= 1;
							// update indices
							mem_byte_index = 0;
							mem_reg_index = 0;
						end

						// pcpi_mem_store <= 1;
						pcpi_mem_wen <= 0;
						pcpi_mem_base <= 0;
						pcpi_mem_offset <= 0;
						mem_sending <= 1;
						pcpi_wr <= 0;
					end
				end
			end
		end
	end