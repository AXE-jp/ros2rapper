/*

Copyright (c) 2019 Alex Forencich

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

// Language: Verilog 2001

`include "ros2_ether_config.vh"

`resetall
`default_nettype none

/*
 * MII PHY interface
 */
module mii_phy_if (
    input  wire        rst_n,

    /*
     * MII interface to MAC
     */
    output wire        mac_mii_rx_clk,
    output wire        mac_mii_rx_rst,
    output wire [3:0]  mac_mii_rxd,
    output wire        mac_mii_rx_dv,
    output wire        mac_mii_rx_er,
    output wire        mac_mii_tx_clk,
    output wire        mac_mii_tx_rst,
    input  wire [3:0]  mac_mii_txd,
    input  wire        mac_mii_tx_en,
    input  wire        mac_mii_tx_er,

    /*
     * MII interface to PHY
     */
    input  wire        phy_mii_rx_clk,
    input  wire [3:0]  phy_mii_rxd,
    input  wire        phy_mii_rx_dv,
    input  wire        phy_mii_rx_er,
    input  wire        phy_mii_tx_clk,
    output wire [3:0]  phy_mii_txd,
    output wire        phy_mii_tx_en,
    output wire        phy_mii_tx_er
);

ssio_sdr_in #
(
    .WIDTH(6)
)
rx_ssio_sdr_inst (
    .input_clk(phy_mii_rx_clk),
    .input_d({phy_mii_rxd, phy_mii_rx_dv, phy_mii_rx_er}),
    .output_clk(mac_mii_rx_clk),
    .output_q({mac_mii_rxd, mac_mii_rx_dv, mac_mii_rx_er}),
    .rst_n(rst_n)
);

(* IOB = "TRUE" *)
reg [3:0] phy_mii_txd_reg;
(* IOB = "TRUE" *)
reg phy_mii_tx_en_reg, phy_mii_tx_er_reg;

assign phy_mii_txd = phy_mii_txd_reg;
assign phy_mii_tx_en = phy_mii_tx_en_reg;
assign phy_mii_tx_er = phy_mii_tx_er_reg;

always @(posedge mac_mii_tx_clk or negedge rst_n) begin
    if (!rst_n) begin
        phy_mii_txd_reg <= 4'd0;
        phy_mii_tx_en_reg <= 1'b0;
        phy_mii_tx_er_reg <= 1'b0;
    end else begin
        phy_mii_txd_reg <= mac_mii_txd;
        phy_mii_tx_en_reg <= mac_mii_tx_en;
        phy_mii_tx_er_reg <= mac_mii_tx_er;
    end
end

`ifdef TARGET_XILINX
    BUFG
    mii_bufg_inst (
        .I(phy_mii_tx_clk),
        .O(mac_mii_tx_clk)
    );
`else
    assign mac_mii_tx_clk = phy_mii_tx_clk;
`endif

// reset sync
reg [3:0] tx_rst_reg;
assign mac_mii_tx_rst = tx_rst_reg[0];

always @(posedge mac_mii_tx_clk or negedge rst_n) begin
    if (!rst_n) begin
        tx_rst_reg <= 4'hf;
    end else begin
        tx_rst_reg <= {1'b0, tx_rst_reg[3:1]};
    end
end

reg [3:0] rx_rst_reg;
assign mac_mii_rx_rst = rx_rst_reg[0];

always @(posedge mac_mii_rx_clk or negedge rst_n) begin
    if (!rst_n) begin
        rx_rst_reg <= 4'hf;
    end else begin
        rx_rst_reg <= {1'b0, rx_rst_reg[3:1]};
    end
end

endmodule

`resetall
