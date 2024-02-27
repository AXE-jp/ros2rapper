// Copyright (c) 2021-2024 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

`default_nettype none

module ram_1rw #(parameter DEPTH=64, DWIDTH=32) (
    i_clk, i_rst_n, i_cs_n, i_we_n, i_wmask, i_addr, i_wdata, o_rdata
);

    localparam AWIDTH = $clog2(DEPTH);

    input wire i_clk;
    input wire i_rst_n;
    input wire i_cs_n;
    input wire i_we_n;
    input wire [DWIDTH/8-1:0] i_wmask;
    input wire [AWIDTH-1:0] i_addr;
    input wire [DWIDTH-1:0] i_wdata;
    output wire [DWIDTH-1:0] o_rdata;

    reg [DWIDTH-1:0] r_rdata;
    (* ram_style = "block" *) reg [DWIDTH-1:0] mem[0:2**AWIDTH-1];

    assign o_rdata = r_rdata;

    `ifdef TARGET_XILINX
    if (DWIDTH == 8) begin
        always @(posedge i_clk) begin
            if (!i_cs_n) begin
                if (!i_we_n) begin
                    if (i_wmask[0]) mem[i_addr][7:0] <= i_wdata[7:0];
                end
                r_rdata <= mem[i_addr];
            end
        end
    end else if (DWIDTH == 16) begin
        always @(posedge i_clk) begin
            if (!i_cs_n) begin
                if (!i_we_n) begin
                    if (i_wmask[0]) mem[i_addr][7:0] <= i_wdata[7:0];
                    if (i_wmask[1]) mem[i_addr][15:8] <= i_wdata[15:8];
                end
                r_rdata <= mem[i_addr];
            end
        end
    end else if (DWIDTH == 32) begin
        always @(posedge i_clk) begin
            if (!i_cs_n) begin
                if (!i_we_n) begin
                    if (i_wmask[0]) mem[i_addr][7:0] <= i_wdata[7:0];
                    if (i_wmask[1]) mem[i_addr][15:8] <= i_wdata[15:8];
                    if (i_wmask[2]) mem[i_addr][23:16] <= i_wdata[23:16];
                    if (i_wmask[3]) mem[i_addr][31:24] <= i_wdata[31:24];
                end
                r_rdata <= mem[i_addr];
            end
        end
    end
    `else
    if (DWIDTH == 8) begin
        always @(posedge i_clk or negedge i_rst_n) begin
            if (!i_rst_n) begin
                r_rdata <= 0;
            end else begin
                if (!i_cs_n) begin
                    if (!i_we_n) begin
                        if (i_wmask[0]) mem[i_addr][7:0] <= i_wdata[7:0];
                    end
                    r_rdata <= mem[i_addr];
                end
            end
        end
    end else if (DWIDTH == 16) begin
        always @(posedge i_clk or negedge i_rst_n) begin
            if (!i_rst_n) begin
                r_rdata <= 0;
            end else begin
                if (!i_cs_n) begin
                    if (!i_we_n) begin
                        if (i_wmask[0]) mem[i_addr][7:0] <= i_wdata[7:0];
                        if (i_wmask[1]) mem[i_addr][15:8] <= i_wdata[15:8];
                    end
                    r_rdata <= mem[i_addr];
                end
             end
        end
    end else if (DWIDTH == 32) begin
        always @(posedge i_clk or negedge i_rst_n) begin
            if (!i_rst_n) begin
                r_rdata <= 0;
            end else begin
                if (!i_cs_n) begin
                    if (!i_we_n) begin
                        if (i_wmask[0]) mem[i_addr][7:0] <= i_wdata[7:0];
                        if (i_wmask[1]) mem[i_addr][15:8] <= i_wdata[15:8];
                        if (i_wmask[2]) mem[i_addr][23:16] <= i_wdata[23:16];
                        if (i_wmask[3]) mem[i_addr][31:24] <= i_wdata[31:24];
                    end
                    r_rdata <= mem[i_addr];
                end
            end
        end
    end
    `endif
endmodule

`default_nettype wire
