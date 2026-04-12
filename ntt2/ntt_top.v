// =============================================================
// NTT_TOP — 4-BFU KYBER-512 NTT
// =============================================================
// Architecture:
//   4 memory banks (64×16 each, indexed by addr[1:0])
//   4 butterfly units
//   1 twiddle ROM (shared — same zeta for all BFUs in a block)
//
// Banking: logical_addr[1:0] selects bank, [7:2] = physical addr
//   Bank 0: addresses 0, 4, 8, ..., 252
//   Bank 1: addresses 1, 5, 9, ..., 253
//   Bank 2: addresses 2, 6, 10, ..., 254
//   Bank 3: addresses 3, 7, 11, ..., 255
//
// Normal mode (len >= 4):
//   BFU_k accesses bank k exclusively (port A = "a" side, port B = "b" side)
//   Conflict-free because len%4 == 0 → (addr+k)[1:0] == (addr+len+k)[1:0]
//
// Crossbar mode (len = 2, len2_mode = 1):
//   BFU0: a_in from bank0.portA, b_in from bank2.portA (crossbar)
//   BFU1: a_in from bank1.portA, b_in from bank3.portA (crossbar)
//   BFU2/BFU3: idle (writes gated)
// =============================================================
module ntt_top (
    input  wire        clk,
    input  wire        rst,
    input  wire        start,
    output wire        done,
    input  wire        ext_we,
    input  wire [7:0]  ext_addr_w,
    input  wire signed [15:0] ext_data_w,
    input  wire [7:0]  ext_addr_r,
    output wire signed [15:0] ext_data_r
);

    // =========================================================
    // Controller
    // =========================================================
    wire [7:0] ctrl_addr_a, ctrl_addr_b;
    wire [7:0] ctrl_addr_wa, ctrl_addr_wb;
    wire       ctrl_we;
    wire [7:0] ctrl_zeta_addr;
    wire       len2_mode;

    ntt_controller ctrl (
        .clk(clk), .reset(rst), .start(start), .done(done),
        .addr_a(ctrl_addr_a), .addr_b(ctrl_addr_b),
        .addr_write_a(ctrl_addr_wa), .addr_write_b(ctrl_addr_wb),
        .we(ctrl_we), .zeta_addr(ctrl_zeta_addr), .len2_mode(len2_mode)
    );

    // =========================================================
    // Twiddle ROM (shared)
    // =========================================================
    wire signed [15:0] twiddle;
    twiddle_rom rom (.clk(clk), .addr(ctrl_zeta_addr), .zeta(twiddle));

    // =========================================================
    // Derived addresses for 4 BFUs (logical, 8-bit)
    // =========================================================
    wire [7:0] ra0 = ctrl_addr_a;          wire [7:0] rb0 = ctrl_addr_b;
    wire [7:0] ra1 = ctrl_addr_a + 8'd1;   wire [7:0] rb1 = ctrl_addr_b + 8'd1;
    wire [7:0] ra2 = ctrl_addr_a + 8'd2;   wire [7:0] rb2 = ctrl_addr_b + 8'd2;
    wire [7:0] ra3 = ctrl_addr_a + 8'd3;   wire [7:0] rb3 = ctrl_addr_b + 8'd3;

    wire [7:0] wa0 = ctrl_addr_wa;          wire [7:0] wb0 = ctrl_addr_wb;
    wire [7:0] wa1 = ctrl_addr_wa + 8'd1;   wire [7:0] wb1 = ctrl_addr_wb + 8'd1;
    wire [7:0] wa2 = ctrl_addr_wa + 8'd2;   wire [7:0] wb2 = ctrl_addr_wb + 8'd2;
    wire [7:0] wa3 = ctrl_addr_wa + 8'd3;   wire [7:0] wb3 = ctrl_addr_wb + 8'd3;

    // =========================================================
    // External access helpers
    // =========================================================
    wire [1:0] ext_w_bank = ext_addr_w[1:0];
    wire [1:0] ext_r_bank = ext_addr_r[1:0];

    // =========================================================
    // Butterfly outputs
    // =========================================================
    wire signed [15:0] bf0_ao, bf0_bo;
    wire signed [15:0] bf1_ao, bf1_bo;
    wire signed [15:0] bf2_ao, bf2_bo;
    wire signed [15:0] bf3_ao, bf3_bo;

    // =========================================================
    // Bank outputs
    // =========================================================
    wire signed [15:0] bk0_da, bk0_db;
    wire signed [15:0] bk1_da, bk1_db;
    wire signed [15:0] bk2_da, bk2_db;
    wire signed [15:0] bk3_da, bk3_db;

    // =========================================================
    // BANK 0 — addresses with [1:0] == 2'b00
    // =========================================================
    // Port A: ext write/read OR BFU0 "a" side (same in both modes)
    // Port B: BFU0 "b" side (normal mode only)
    wire b0_wea = (ext_we && ext_w_bank == 2'd0) || (ctrl_we && !ext_we);
    wire b0_web = ctrl_we && !ext_we && !len2_mode;

    wire [5:0] b0_aa = (ext_we && ext_w_bank == 2'd0) ? ext_addr_w[7:2] :
                       (done   && ext_r_bank == 2'd0) ? ext_addr_r[7:2] :
                       ctrl_we                        ? wa0[7:2] :
                                                        ra0[7:2];

    wire [5:0] b0_ab = ctrl_we ? wb0[7:2] : rb0[7:2];

    wire signed [15:0] b0_da_in = (ext_we && ext_w_bank == 2'd0) ? ext_data_w : bf0_ao;
    wire signed [15:0] b0_db_in = bf0_bo;

    memory_bank bank0 (
        .clk(clk),
        .we_a(b0_wea), .addr_a(b0_aa), .din_a(b0_da_in), .dout_a(bk0_da),
        .we_b(b0_web), .addr_b(b0_ab), .din_b(b0_db_in), .dout_b(bk0_db)
    );

    // =========================================================
    // BANK 1 — addresses with [1:0] == 2'b01
    // =========================================================
    wire b1_wea = (ext_we && ext_w_bank == 2'd1) || (ctrl_we && !ext_we);
    wire b1_web = ctrl_we && !ext_we && !len2_mode;

    wire [5:0] b1_aa = (ext_we && ext_w_bank == 2'd1) ? ext_addr_w[7:2] :
                       (done   && ext_r_bank == 2'd1) ? ext_addr_r[7:2] :
                       ctrl_we                        ? wa1[7:2] :
                                                        ra1[7:2];

    wire [5:0] b1_ab = ctrl_we ? wb1[7:2] : rb1[7:2];

    wire signed [15:0] b1_da_in = (ext_we && ext_w_bank == 2'd1) ? ext_data_w : bf1_ao;
    wire signed [15:0] b1_db_in = bf1_bo;

    memory_bank bank1 (
        .clk(clk),
        .we_a(b1_wea), .addr_a(b1_aa), .din_a(b1_da_in), .dout_a(bk1_da),
        .we_b(b1_web), .addr_b(b1_ab), .din_b(b1_db_in), .dout_b(bk1_db)
    );

    // =========================================================
    // BANK 2 — addresses with [1:0] == 2'b10
    // =========================================================
    // Normal: BFU2 "a" side (port A) + "b" side (port B)
    // Len2:   BFU0 "b" side via crossbar (port A only)
    wire b2_wea = (ext_we && ext_w_bank == 2'd2) || (ctrl_we && !ext_we);
    wire b2_web = ctrl_we && !ext_we && !len2_mode;

    // Port A address: normal=BFU2 "a", len2=BFU0 "b" (crossbar)
    wire [5:0] b2_aa_normal = ctrl_we ? wa2[7:2] : ra2[7:2];
    wire [5:0] b2_aa_len2   = ctrl_we ? wb0[7:2] : rb0[7:2];

    wire [5:0] b2_aa = (ext_we && ext_w_bank == 2'd2) ? ext_addr_w[7:2] :
                       (done   && ext_r_bank == 2'd2) ? ext_addr_r[7:2] :
                       len2_mode                      ? b2_aa_len2 :
                                                        b2_aa_normal;

    wire [5:0] b2_ab = ctrl_we ? wb2[7:2] : rb2[7:2];

    // Port A data: normal=BFU2 a_out, len2=BFU0 b_out (crossbar)
    wire signed [15:0] b2_da_in = (ext_we && ext_w_bank == 2'd2) ? ext_data_w :
                                  len2_mode                      ? bf0_bo :
                                                                   bf2_ao;
    wire signed [15:0] b2_db_in = bf2_bo;

    memory_bank bank2 (
        .clk(clk),
        .we_a(b2_wea), .addr_a(b2_aa), .din_a(b2_da_in), .dout_a(bk2_da),
        .we_b(b2_web), .addr_b(b2_ab), .din_b(b2_db_in), .dout_b(bk2_db)
    );

    // =========================================================
    // BANK 3 — addresses with [1:0] == 2'b11
    // =========================================================
    wire b3_wea = (ext_we && ext_w_bank == 2'd3) || (ctrl_we && !ext_we);
    wire b3_web = ctrl_we && !ext_we && !len2_mode;

    wire [5:0] b3_aa_normal = ctrl_we ? wa3[7:2] : ra3[7:2];
    wire [5:0] b3_aa_len2   = ctrl_we ? wb1[7:2] : rb1[7:2];

    wire [5:0] b3_aa = (ext_we && ext_w_bank == 2'd3) ? ext_addr_w[7:2] :
                       (done   && ext_r_bank == 2'd3) ? ext_addr_r[7:2] :
                       len2_mode                      ? b3_aa_len2 :
                                                        b3_aa_normal;

    wire [5:0] b3_ab = ctrl_we ? wb3[7:2] : rb3[7:2];

    wire signed [15:0] b3_da_in = (ext_we && ext_w_bank == 2'd3) ? ext_data_w :
                                  len2_mode                      ? bf1_bo :
                                                                   bf3_ao;
    wire signed [15:0] b3_db_in = bf3_bo;

    memory_bank bank3 (
        .clk(clk),
        .we_a(b3_wea), .addr_a(b3_aa), .din_a(b3_da_in), .dout_a(bk3_da),
        .we_b(b3_web), .addr_b(b3_ab), .din_b(b3_db_in), .dout_b(bk3_db)
    );

    // =========================================================
    // External read output (mux by bank)
    // =========================================================
    assign ext_data_r = (ext_r_bank == 2'd0) ? bk0_da :
                        (ext_r_bank == 2'd1) ? bk1_da :
                        (ext_r_bank == 2'd2) ? bk2_da :
                                               bk3_da;

    // =========================================================
    // Butterfly 0 — bank 0 (always active)
    // Normal: a_in=bank0.A, b_in=bank0.B
    // Len2:   a_in=bank0.A, b_in=bank2.A (crossbar)
    // =========================================================
    butterfly bf0 (
        .clk(clk), .rst(rst),
        .a_in(bk0_da),
        .b_in(len2_mode ? bk2_da : bk0_db),
        .w_in(twiddle),
        .a_out(bf0_ao), .b_out(bf0_bo)
    );

    // =========================================================
    // Butterfly 1 — bank 1 (always active)
    // Normal: a_in=bank1.A, b_in=bank1.B
    // Len2:   a_in=bank1.A, b_in=bank3.A (crossbar)
    // =========================================================
    butterfly bf1 (
        .clk(clk), .rst(rst),
        .a_in(bk1_da),
        .b_in(len2_mode ? bk3_da : bk1_db),
        .w_in(twiddle),
        .a_out(bf1_ao), .b_out(bf1_bo)
    );

    // =========================================================
    // Butterfly 2 — bank 2 (idle in len2 mode)
    // =========================================================
    butterfly bf2 (
        .clk(clk), .rst(rst),
        .a_in(bk2_da), .b_in(bk2_db), .w_in(twiddle),
        .a_out(bf2_ao), .b_out(bf2_bo)
    );

    // =========================================================
    // Butterfly 3 — bank 3 (idle in len2 mode)
    // =========================================================
    butterfly bf3 (
        .clk(clk), .rst(rst),
        .a_in(bk3_da), .b_in(bk3_db), .w_in(twiddle),
        .a_out(bf3_ao), .b_out(bf3_bo)
    );

endmodule
