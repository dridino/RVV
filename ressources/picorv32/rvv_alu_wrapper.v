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
    output                              instr_valid
);
    localparam SHIFTED_LANE_WIDTH = 1 << LANE_WIDTH;
    localparam SHIFTED_NB_LANES = 1 << NB_LANES;
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
    wire [(64<<NB_LANES)-1 : 0] vds;

    assign res = runs;

    genvar loop_i;
    generate
        for (loop_i = 0; loop_i < (1<<NB_LANES); loop_i = loop_i + 1) begin
            assign runs[loop_i] = run && arith_remaining > loop_i;
            assign vd[(64*loop_i) +: 64] = (opcode == 6'b010000 && vs1_index == 5'b10000) ? (loop_i == 0 ? {32'h00000000, sumN(vds)} : 0) :
                                           (opcode == 6'b010000 && vs1_index == 5'b10001) ? (loop_i == 0 ? {32'h00000000, minN(vds)} : 0) :
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

    always @(posedge clk) begin
        if (!resetn) begin
            byte_i <= 0;
            done <= 0;
        end else if (run) begin
            if (!done) begin
                if (vsew+3 <= LANE_WIDTH) begin
                    done <= byte_i + (1 << (nb_lanes)) >= (instr_mask ? vl /* >> (vsew+3) */ : `min(vl, (VLEN >> (vsew + 3))));
                end else begin
                    done <= byte_i + (1 << (nb_lanes)) >= (instr_mask ? vl /* >> (vsew+3) */ : `min(vl, (VLEN >> (vsew + 3)))) && in_reg_offset == ((1 << (vsew+3-LANE_WIDTH)) - 1);
                end

                if (vsew + 3 < LANE_WIDTH || in_reg_offset == (vsew + 3 <= LANE_WIDTH ? 0 : (1 << (vsew+3-LANE_WIDTH)) - 1)) begin
                    in_reg_offset <= 0;
                    byte_i <= byte_i + (1<<nb_lanes);
                end else
                    in_reg_offset <= in_reg_offset + 1;
            end else begin
                done <= 0;
            end
        end else begin
            byte_i <= 0;
            in_reg_offset <= 0;
            done <= 0;
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
                .byte_i(byte_i),
                .in_reg_offset(in_reg_offset),
                .vd(vds[(64*loop_i) +: 64]),
                .index(index[(17*loop_i) +: 17]),
                .instr_valid(instr_valids[loop_i])
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
                if (&acc)
                    acc = &(in[i*64 +: SHIFTED_LANE_WIDTH]) ? 32'hFFFFFFFF : in[i*64 +: 32];
            minN = acc;
        end
    endfunction

endmodule