// =============================================================
// NTT CONTROLLER — 4 BFUs (2-BFU fallback for len=2)
// =============================================================
// For len >= 4: 4 BFUs process j, j+1, j+2, j+3 in parallel.
//   j advances by 4 each iteration.
//   Banking by addr[1:0] guarantees zero conflicts (len%4==0).
//
// For len = 2: 2 BFUs process j, j+1 in parallel (crossbar mode).
//   j advances by 2 (but len=2 means 1 iteration per block).
//   BFU2/BFU3 idle. len2_mode output enables crossbar in ntt_top.
//
// Pipeline: BRAM(1) + butterfly(5) = 6 wait cycles
//
// Cycle count:
//   6 stages × 32 iter + 1 stage × 64 iter = 256 iter × 9 ≈ 2304
// =============================================================
module ntt_controller (
    input  wire       clk,
    input  wire       reset,
    input  wire       start,

    output reg  [7:0] addr_a,       // Base address (BFU0: s+j)
    output reg  [7:0] addr_b,       // Pair address (BFU0: s+j+len)

    output reg        we,
    output reg  [7:0] addr_write_a,
    output reg  [7:0] addr_write_b,

    output reg  [7:0] zeta_addr,
    output reg        done,
    output wire       len2_mode     // High when len==2 (2-BFU crossbar)
);

    parameter N = 256;

    localparam IDLE  = 3'd0;
    localparam READ  = 3'd1;
    localparam WAIT  = 3'd2;
    localparam WRITE = 3'd3;
    localparam NEXT  = 3'd4;
    localparam DONE  = 3'd5;

    reg [2:0] state;
    reg [7:0] len;
    reg [7:0] start_idx;
    reg [7:0] j;
    reg [7:0] k;
    reg [2:0] pipe_cnt;

    wire [8:0] next_block = start_idx + (len << 1);

    assign len2_mode = (len == 8'd2);

    // Check if more butterflies remain in current block
    wire more_in_block = len2_mode ? (j + 8'd2 < len)
                                   : (j + 8'd4 < len);

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state     <= IDLE;
            len       <= 128;
            start_idx <= 0;
            j         <= 0;
            k         <= 0;
            we        <= 0;
            done      <= 0;
            pipe_cnt  <= 0;
            zeta_addr <= 0;
            addr_a    <= 0;
            addr_b    <= 0;
            addr_write_a <= 0;
            addr_write_b <= 0;
        end else begin
            case (state)

                IDLE: begin
                    done <= 0;
                    we   <= 0;
                    if (start) begin
                        len       <= 128;
                        start_idx <= 0;
                        j         <= 0;
                        k         <= 1;
                        state     <= READ;
                    end
                end

                READ: begin
                    addr_a    <= start_idx + j;
                    addr_b    <= start_idx + j + len;
                    zeta_addr <= k;
                    we        <= 0;
                    pipe_cnt  <= 0;
                    state     <= WAIT;
                end

                WAIT: begin
                    pipe_cnt <= pipe_cnt + 1;
                    if (pipe_cnt == 3'd5)
                        state <= WRITE;
                end

                WRITE: begin
                    addr_write_a <= start_idx + j;
                    addr_write_b <= start_idx + j + len;
                    we    <= 1;
                    state <= NEXT;
                end

                NEXT: begin
                    we <= 0;
                    if (more_in_block) begin
                        j     <= len2_mode ? j + 8'd2 : j + 8'd4;
                        state <= READ;
                    end else begin
                        j <= 0;
                        k <= k + 1;

                        if (next_block < N) begin
                            start_idx <= next_block[7:0];
                            state     <= READ;
                        end else begin
                            start_idx <= 0;
                            if (len > 2) begin
                                len   <= len >> 1;
                                state <= READ;
                            end else begin
                                state <= DONE;
                            end
                        end
                    end
                end

                DONE: begin
                    done <= 1;
                end

                default: state <= IDLE;

            endcase
        end
    end

endmodule
