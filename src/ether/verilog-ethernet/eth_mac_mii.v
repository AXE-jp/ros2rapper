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
 * 10M/100M Ethernet MAC with MII interface
 */
module eth_mac_mii #
(
    parameter ENABLE_PADDING = 1,
    parameter MIN_FRAME_LENGTH = 64
)
(
    input  wire        rst_n,
    output wire        rx_clk,
    output wire        rx_rst,
    output wire        tx_clk,
    output wire        tx_rst,

    /*
     * AXI input
     */
    input  wire [7:0]  tx_axis_tdata,
    input  wire        tx_axis_tvalid,
    output wire        tx_axis_tready,
    input  wire        tx_axis_tlast,
    input  wire        tx_axis_tuser,

    /*
     * AXI output
     */
    output wire [7:0]  rx_axis_tdata,
    output wire        rx_axis_tvalid,
    output wire        rx_axis_tlast,
    output wire        rx_axis_tuser,

    /*
     * MII interface
     */
    input  wire        mii_rx_clk,
    input  wire [3:0]  mii_rxd,
    input  wire        mii_rx_dv,
    input  wire        mii_rx_er,
    input  wire        mii_tx_clk,
    output wire [3:0]  mii_txd,
    output wire        mii_tx_en,
    output wire        mii_tx_er,

    /*
     * Status
     */
    output wire        tx_start_packet,
    output wire        tx_error_underflow,
    output wire        rx_start_packet,
    output wire        rx_error_bad_frame,
    output wire        rx_error_bad_fcs,

    /*
     * Configuration
     */
    input  wire [7:0]  ifg_delay
);

wire [3:0]  mac_mii_rxd;
wire        mac_mii_rx_dv;
wire        mac_mii_rx_er;
wire [7:0]  mac_mii_txd_full;
wire [3:0]  mac_mii_txd = mac_mii_txd_full[3:0];
wire        mac_mii_tx_en;
wire        mac_mii_tx_er;

mii_phy_if mii_phy_if_inst (
    .rst_n(rst_n),

    .mac_mii_rx_clk(rx_clk),
    .mac_mii_rx_rst(rx_rst),
    .mac_mii_rxd(mac_mii_rxd),
    .mac_mii_rx_dv(mac_mii_rx_dv),
    .mac_mii_rx_er(mac_mii_rx_er),
    .mac_mii_tx_clk(tx_clk),
    .mac_mii_tx_rst(tx_rst),
    .mac_mii_txd(mac_mii_txd),
    .mac_mii_tx_en(mac_mii_tx_en),
    .mac_mii_tx_er(mac_mii_tx_er),

    .phy_mii_rx_clk(mii_rx_clk),
    .phy_mii_rxd(mii_rxd),
    .phy_mii_rx_dv(mii_rx_dv),
    .phy_mii_rx_er(mii_rx_er),
    .phy_mii_tx_clk(mii_tx_clk),
    .phy_mii_txd(mii_txd),
    .phy_mii_tx_en(mii_tx_en),
    .phy_mii_tx_er(mii_tx_er)
);

eth_mac_1g #(
    .ENABLE_PADDING(ENABLE_PADDING),
    .MIN_FRAME_LENGTH(MIN_FRAME_LENGTH)
)
eth_mac_1g_inst (
    .tx_clk(tx_clk),
    .tx_rst(tx_rst),
    .rx_clk(rx_clk),
    .rx_rst(rx_rst),
    .tx_axis_tdata(tx_axis_tdata),
    .tx_axis_tvalid(tx_axis_tvalid),
    .tx_axis_tready(tx_axis_tready),
    .tx_axis_tlast(tx_axis_tlast),
    .tx_axis_tuser(tx_axis_tuser),
    .rx_axis_tdata(rx_axis_tdata),
    .rx_axis_tvalid(rx_axis_tvalid),
    .rx_axis_tlast(rx_axis_tlast),
    .rx_axis_tuser(rx_axis_tuser),
    .gmii_rxd({mac_mii_rxd, mac_mii_rxd}),
    .gmii_rx_dv(mac_mii_rx_dv),
    .gmii_rx_er(mac_mii_rx_er),
    .gmii_txd(mac_mii_txd_full),
    .gmii_tx_en(mac_mii_tx_en),
    .gmii_tx_er(mac_mii_tx_er),
    .tx_ptp_ts(96'b0),
    .rx_ptp_ts(96'b0),
    .tx_axis_ptp_ts(),
    .tx_axis_ptp_ts_tag(),
    .tx_axis_ptp_ts_valid(),
    .rx_clk_enable(1'b1),
    .tx_clk_enable(1'b1),
    .rx_mii_select(1'b1),
    .tx_mii_select(1'b1),
    .tx_start_packet(tx_start_packet),
    .tx_error_underflow(tx_error_underflow),
    .rx_start_packet(rx_start_packet),
    .rx_error_bad_frame(rx_error_bad_frame),
    .rx_error_bad_fcs(rx_error_bad_fcs),
    .ifg_delay(ifg_delay)
);

endmodule

`resetall
