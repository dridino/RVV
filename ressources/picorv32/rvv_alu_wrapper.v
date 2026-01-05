`define min(a,b) (a < b ? a : b)

module rvv_alu_wrapper #(
    parameter [16:0] VLEN = 17'd 128,
    parameter [2:0] LANE_WIDTH = 3'b011,
    parameter integer NB_LANES = 1
) (
    input                               clk, resetn,
    input       [5:0]                   opcode,
    input                               instr_mask,
    input                               run,
    input       [4:0]                   vs1_index,
    input       [VLEN-1:0]              vs1,
    input       [VLEN-1:0]              vs2,
    input       [2:0]                   vsew,
    input       [2:0]                   op_type,
    input       [16:0]                  vl,
    input       [16:0]                  arith_remaining,

    output      [(64<<NB_LANES) - 1:0]  vd,
    output      [(17<<NB_LANES) - 1:0]  regi,
    output      [(1<<NB_LANES) - 1:0]   res,
    output                              done_out,
    output                              instr_valid,
    output                              arith_instr_signed
);
    localparam SHIFTED_LANE_WIDTH = 1 << LANE_WIDTH;
    localparam SHIFTED_NB_LANES = 1 << NB_LANES;
    localparam integer VLEN_SIZE = $clog2(VLEN);
	localparam [2:0] VV = 3'b001;
	localparam [2:0] VX = 3'b010;
	localparam [2:0] VI = 3'b100;

    reg [16:0] byte_i;
    reg [3:0] in_reg_offset;

    reg done;
    assign done_out = done;

    wire [(17<<NB_LANES)-1 : 0] index;
    assign regi = index;

    wire [(SHIFTED_NB_LANES)-1 : 0] runs;
    wire [(SHIFTED_NB_LANES)-1 : 0] mask_couts;
    wire [(64<<NB_LANES)-1 : 0] vds;

    wire [SHIFTED_NB_LANES-1 : 0] arith_instr_signed_v;
    assign arith_instr_signed = arith_instr_signed_v[0];

    assign res = runs;

    reg [VLEN_SIZE-1:0] viota_acc, viota_acc_tmp, viota_acc_tmp_assign;

    wire instr_viota = (opcode == 6'b010100 && vs1_index[4:1] == 4'b1000); // viota | vid

    integer loopi;
    genvar loop_i;
    generate
        for (loop_i = 0; loop_i < (1<<NB_LANES); loop_i = loop_i + 1) begin
            // viota_acc_tmp_assign = loop_i == 0 ? viota_acc : (viota_acc_tmp_assign + vds[64*(i-1)]);
            assign runs[loop_i] = run && arith_remaining > loop_i;
            assign vd[(64*loop_i) +: 64] = (opcode == 6'b010000 && vs1_index == 5'b10000) ? (loop_i == 0 ? {32'h00000000, sumN(vds)} : 0) :
                                           (opcode == 6'b010000 && vs1_index == 5'b10001) ? (loop_i == 0 ? {32'h00000000, minN(vds)} : 0) : // vfirst
                                           (opcode == 6'b010100 && vs1_index == 5'b00001) ? (loop_i == 0 ? vds[(64*loop_i) +: 64] : (mask_couts[loop_i-1] ? vds[(64*loop_i) +: 64] : 0)) :
                                           (opcode == 6'b010100 && vs1_index == 5'b00011) ? (loop_i == 0 ? vds[(64*loop_i) +: 64] : (mask_couts[loop_i-1] ? vds[(64*loop_i) +: 64] : 0)) :
                                           (opcode == 6'b010100 && vs1_index == 5'b00010) ? (loop_i == 0 ? vds[(64*loop_i) +: 64] : (mask_couts[loop_i-1] ? 0 : vds[(64*loop_i) +: 64])) :
                                           instr_viota                                    ? (viota_acc_fn(vds, loop_i, viota_acc)) : // viota | vid
                                                                                            vds[(64*loop_i) +: 64];
        end
    endgenerate

    wire [(SHIFTED_NB_LANES)-1 : 0] instr_valids;
    assign instr_valid = instr_valids[0];

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

    reg [2:0] viota_incr;

    always @(posedge clk) begin
        if (!resetn) begin
            byte_i <= 0;
            done <= 0;
            viota_acc <= 0;
	    viota_incr <= 0;
        end else if (run) begin
            if (!done) begin
            if (instr_viota) begin // viota | vid
                viota_acc_tmp = viota_acc;
                for (loopi = 0; loopi < SHIFTED_NB_LANES; loopi = loopi + 1)
		            if (runs[loopi])
                        viota_acc_tmp = viota_acc_tmp + vds[64*loopi];
		        viota_acc <= viota_acc_tmp;
            end
                if (vsew+3 <= LANE_WIDTH) begin
                    done <= byte_i + (1 << (nb_lanes)) >= (instr_mask && !(instr_viota) ? vl : `min(vl, (VLEN >> (vsew + 3))));
                end else begin
                    done <= byte_i + (1 << (nb_lanes)) >= (instr_mask && !(instr_viota) ? vl : `min(vl, (VLEN >> (vsew + 3)))) && in_reg_offset == ((1 << (vsew+3-LANE_WIDTH)) - 1);
                end

                if (vsew + 3 <= LANE_WIDTH || in_reg_offset == (vsew + 3 <= LANE_WIDTH ? 0 : (1 << (vsew+3-LANE_WIDTH)) - 1) || (instr_viota)) begin
                    in_reg_offset <= 0;
                    byte_i <= byte_i + (1<<nb_lanes);
                end else
                    in_reg_offset <= in_reg_offset + 1;
            end else begin
                viota_incr <= viota_incr + 1;
		done <= 0;
            end
        end else begin
	    // if (arith_remaining == 0 && !(opcode == 6'b010100 && vs1_index == 5'b10000))
            byte_i <= 0;
            in_reg_offset <= 0;
	    if (arith_remaining == 0) viota_incr <= 0;
            done <= 0;
            if (arith_remaining == 0)
                viota_acc <= 0;
        end
    end

    generate
        for (loop_i = 0; loop_i < SHIFTED_NB_LANES; loop_i = loop_i + 1) begin
            rvv_alu #(
                .VLEN (VLEN),
                .LANE_WIDTH (LANE_WIDTH),
                .LANE_I (loop_i)
            ) valu0 (
                .clk(clk),
                .resetn(resetn),
                .nb_lanes(nb_lanes),
                .opcode(opcode),
                .instr_mask(instr_mask),
                .run(runs[loop_i]),
                .vs1_index(vs1_index),
                .vs1_in(vs1),
                .vs2_in(vs2),
                .vsew(vsew),
                .op_type(op_type),
		        .byte_i((viota_incr > 0 ? (((VLEN >> (vsew+3)) * viota_incr) /*+ 1*/) : 0) + byte_i),
                .in_reg_offset(in_reg_offset),
                .vd(vds[(64*loop_i) +: 64]),
                .mask_cout(mask_couts[loop_i]),
                .index(index[(17*loop_i) +: 17]),
                .instr_valid(instr_valids[loop_i]),
                .instr_signed(arith_instr_signed_v[loop_i])
            );
        end
    endgenerate

    function automatic [31:0] sumN;
        input [SHIFTED_NB_LANES*64-1:0] in;
        integer i;
        reg [31:0] acc;
        begin
            acc = 0;
            for (i = 0; i < SHIFTED_NB_LANES; i = i + 1)
                acc = acc + in[i*64 +: 32];
            sumN = acc;
        end
    endfunction

    function automatic [31:0] minN;
        input [SHIFTED_NB_LANES*64-1:0] in;
        integer i;
        reg [31:0] acc;
        begin
            acc = 32'hFFFFFFFF;
            for (i = 0; i < SHIFTED_NB_LANES; i = i + 1)
                if (&acc & |in[i*64 +: SHIFTED_LANE_WIDTH])
                    acc = &(in[i*64 +: SHIFTED_LANE_WIDTH]) ? 32'hFFFFFFFF : in[i*64 +: 32];
            minN = acc;
        end
    endfunction

    function automatic [63:0] viota_acc_fn;
        input [SHIFTED_NB_LANES*64-1:0] vds;
        input integer i;
        input [VLEN_SIZE-1:0] viota_acc;
        integer loopi;
        reg [VLEN_SIZE-1:0] acc;
        begin
            acc = viota_acc;
            for (loopi = 0; loopi < i; loopi = loopi + 1) begin
                acc = acc + vds[loopi*64 +: 64];
                // $display("i : %d, loopi : %d, acc : %d, ret : %d", i, loopi, viota_acc, acc);
            end
            viota_acc_fn = acc;
        end
    endfunction

endmodule
