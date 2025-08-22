// Copyright (c) 2021-2025 AXE, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later

`resetall

`include "ros2_config.vh"

module top(
    input  wire       clk,
    input  wire       rst_n,

    output wire       phy_ref_clk,
    input  wire       phy_rx_clk,
    input  wire [3:0] phy_rxd,
    input  wire       phy_rx_dv,
    input  wire       phy_rx_er,
    input  wire       phy_tx_clk,
    output wire [3:0] phy_txd,
    output wire       phy_tx_en,
    output wire       phy_rst_n,

    output wire       led4,
    output wire       led5
);

    wire ros2_clk;
    wire clk_25MHz;
    clk_wiz_0 clk_wiz_inst (
        // Clock out ports
        .ros2_clk(ros2_clk),     // output ros2_clk
        .clk_25MHz(clk_25MHz),     // output clk_25MHz
        // Status and control signals
        .resetn(rst_n), // input resetn
        .locked(),       // output locked
        // Clock in ports
        .clk_in1(clk)      // input clk_in1
    );

    wire [`ROS2_MAX_APP_DATA_LEN*8-1:0] pub_app_data;
    wire pub_app_data_ap_vld;
    wire pub_app_data_ap_ack;
    wire [15:0] pub_data_seed;
    hls_pub_0 hls_pub_inst (
        .pub_app_data_ap_vld(pub_app_data_ap_vld),  // output wire pub_app_data_ap_vld
        .pub_app_data_ap_ack(pub_app_data_ap_ack),  // input wire pub_app_data_ap_ack
        .ap_clk(ros2_clk),                            // input wire ap_clk
        .ap_rst_n(rst_n),                        // input wire ap_rst_n
        .pub_data_seed(pub_data_seed),              // input wire [15 : 0] pub_data_seed
        .pub_app_data(pub_app_data)                // output wire [8191 : 0] pub_app_data
    );

    wire sub_data_result;
    wire sub_data_result_ap_vld;
    wire sub_data_result_ap_ack;
    wire [$clog2(`ROS2_MAX_APP_DATA_LEN)-2:0] sub_app_data_address0;
    wire sub_app_data_ce0;
    wire [15:0] sub_app_data_q0;
    hls_sub_0 hls_sub_inst (
        .sub_app_data_ce0(sub_app_data_ce0),              // output wire sub_app_data_ce0
        .sub_data_result_ap_vld(sub_data_result_ap_vld),  // output wire sub_data_result_ap_vld
        .sub_data_result_ap_ack(sub_data_result_ap_ack),  // input wire sub_data_result_ap_ack
        .ap_clk(ros2_clk),                                  // input wire ap_clk
        .ap_rst_n(rst_n),                              // input wire ap_rst_n
        .sub_app_data_address0(sub_app_data_address0),    // output wire [8 : 0] sub_app_data_address0
        .sub_app_data_q0(sub_app_data_q0),                // input wire [15 : 0] sub_app_data_q0
        .sub_data_result(sub_data_result)                // output wire sub_data_result
    );

    wire [$clog2(`ROS2_MAX_APP_DATA_LEN)-1:0] ros2_sub_app_data_addr;
    wire ros2_sub_app_data_ce;
    wire ros2_sub_app_data_we;
    wire [7:0] ros2_sub_app_data_wdata;
    ros2_module #(
        .ROS2CLK_HZ(80_000_000)
    )
    ros2_inst (
        .clk(ros2_clk),
        .rst_n(rst_n),
        .clk_25mhz(clk_25MHz),

        .phy_ref_clk(phy_ref_clk),
        .phy_rx_clk(phy_rx_clk),
        .phy_rxd(phy_rxd),
        .phy_rx_dv(phy_rx_dv),
        .phy_rx_er(phy_rx_er),
        .phy_tx_clk(phy_tx_clk),
        .phy_txd(phy_txd),
        .phy_tx_en(phy_tx_en),
        .phy_rst_n(phy_rst_n),

        .led4(led4),
        .led5(led5),

        .ros2_pub_app_data(pub_app_data),
        .ros2_pub_app_data_ap_vld(pub_app_data_ap_vld),
        .ros2_pub_app_data_ap_ack(pub_app_data_ap_ack),
        .pub_data_seed(pub_data_seed),

        .ros2_sub_app_data_addr(ros2_sub_app_data_addr),
        .ros2_sub_app_data_ce(ros2_sub_app_data_ce),
        .ros2_sub_app_data_we(ros2_sub_app_data_we),
        .ros2_sub_app_data_wdata(ros2_sub_app_data_wdata),

        .sub_data_result(sub_data_result),
        .sub_data_result_ap_vld(sub_data_result_ap_vld),
        .sub_data_result_ap_ack(sub_data_result_ap_ack)
    );

    blk_mem_gen_0 blk_mem_inst (
        .clka(ros2_clk),    // input wire clka
        .ena(ros2_sub_app_data_ce),      // input wire ena
        .wea(ros2_sub_app_data_we),      // input wire [0 : 0] wea
        .addra(ros2_sub_app_data_addr),  // input wire [9 : 0] addra
        .dina(ros2_sub_app_data_wdata),    // input wire [7 : 0] dina
        .clkb(ros2_clk),    // input wire clkb
        .enb(sub_app_data_ce0),      // input wire enb
        .addrb(sub_app_data_address0),  // input wire [8 : 0] addrb
        .doutb(sub_app_data_q0)  // output wire [15 : 0] doutb
    );

endmodule

`resetall
