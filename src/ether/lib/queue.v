// Copyright (c) 2021-2024 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

`resetall
`default_nettype none

module queue #(
    parameter DATA_WIDTH = 8,
    parameter DEPTH = 8
)
(
    input  wire                   clk,
    input  wire                   rst_n,

    input  wire                   wr_en,
    input  wire [DATA_WIDTH-1:0]  din,
    output wire                   full,
    output wire                   almost_full,

    input  wire                   rd_en,
    output wire [DATA_WIDTH-1:0]  dout,
    output wire                   empty
);

    localparam PTR_WIDTH = $clog2(DEPTH);

    reg [DATA_WIDTH-1:0] regs[0:DEPTH-1];
    reg [PTR_WIDTH:0] wp;
    reg [PTR_WIDTH:0] rp;
    wire [PTR_WIDTH:0] wp_next;

    assign wp_next = wp + 1;
    assign full = (wp[PTR_WIDTH] != rp[PTR_WIDTH]) & (wp[PTR_WIDTH-1:0] == rp[PTR_WIDTH-1:0]);
    assign almost_full = (wp_next[PTR_WIDTH] != rp[PTR_WIDTH]) & (wp_next[PTR_WIDTH-1:0] == rp[PTR_WIDTH-1:0]);
    assign empty = (wp == rp);
    assign dout = empty ? {DATA_WIDTH{1'b0}} : regs[rp[PTR_WIDTH-1:0]];

    `ifdef USE_SYNC_RESET
        always @(posedge clk) begin
            if (!rst_n) begin
                wp <= 0;
                rp <= 0;
            end else begin
                if (wr_en & !full) begin
                    regs[wp[PTR_WIDTH-1:0]] <= din;
                    wp <= wp + 1;
                end
                if (rd_en & !empty) begin
                    rp <= rp + 1;
                end
            end
        end
    `else
        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                wp <= 0;
                rp <= 0;
            end else begin
                if (wr_en & !full) begin
                    regs[wp[PTR_WIDTH-1:0]] <= din;
                    wp <= wp + 1;
                end
                if (rd_en & !empty) begin
                    rp <= rp + 1;
                end
            end
        end
    `endif

`ifdef FORMAL
    // preparation for using $past
    reg f_past_valid;
    initial f_past_valid = 1'b0;
    always @(posedge clk)
        f_past_valid <= 1'b1;

    // reset
    initial assume(~rst_n);
    always @(posedge clk)
        if (!f_past_valid)
            assume(~rst_n);

    wire [PTR_WIDTH:0] f_count;
    assign f_count = (wp[PTR_WIDTH] == rp[PTR_WIDTH]) ? (wp[PTR_WIDTH-1:0] - rp[PTR_WIDTH-1:0]) : (wp[PTR_WIDTH-1:0] + DEPTH - rp[PTR_WIDTH-1:0]);

    always @(posedge clk) begin
        if (f_past_valid) begin
            if ($past(~rst_n)) begin
                empty_after_reset: assert(empty);
            end else begin
                if ($past(almost_full & wr_en & ~rd_en))
                    enqueue_on_almost_full: assert(full);

                if ($past(full & rd_en & ~wr_en))
                    dequeue_on_full: assert(almost_full);

                if ($past(almost_full & rd_en & ~wr_en))
                    dequeue_on_almost_full: assert(~almost_full);

                if ($past(full & wr_en & ~rd_en))
                    enqueue_on_full: assert(full);

                if ($past(empty & rd_en & ~wr_en))
                    dequeue_on_empty: assert(empty);

                excl_full_and_empty: assert (!(full & empty));

                excl_almost_full_and_full: assert (!(almost_full & full));

                count_not_over_depth: assert(f_count <= DEPTH);
                if (empty)
                    count_is_zero_if_empty: assert(f_count == 0);
                if (full)
                    count_is_depth_if_full: assert(f_count == DEPTH);
                if (almost_full)
                    count_is_depth_minus_one_if_almost_full: assert(f_count == DEPTH - 1);
            end
        end
    end

    always @(posedge clk) begin
        cover(empty);
        cover(almost_full);
        cover(full);
    end
`endif

endmodule

`default_nettype wire
