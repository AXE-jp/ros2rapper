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
    // --- Clock & Reset
    wire clk_int;
    wire clk_25mhz_int;
    wire rst_n_int;
    wire mmcm_locked;
    wire mmcm_clkfb;

    assign phy_ref_clk = clk_25mhz_int;

`ifdef ROS2RAPPER_HLS_VITIS
    localparam ROS2CLK_HZ = 80_000_000;
`elsif ROS2RAPPER_HLS_CWB
    localparam ROS2_CLK_HZ = 40_000_000;
`endif
    MMCME2_BASE #(
        .BANDWIDTH("OPTIMIZED"),
`ifdef ROS2RAPPER_HLS_VITIS
        .CLKOUT0_DIVIDE_F(12.5),
`elsif ROS2RAPPER_HLS_CWB
        .CLKOUT0_DIVIDE_F(25.0),
`endif
        .CLKOUT0_DUTY_CYCLE(0.5),
        .CLKOUT0_PHASE(0),
        .CLKOUT1_DIVIDE(40),
        .CLKOUT1_DUTY_CYCLE(0.5),
        .CLKOUT1_PHASE(0),
        .CLKOUT2_DIVIDE(1),
        .CLKOUT2_DUTY_CYCLE(0.5),
        .CLKOUT2_PHASE(0),
        .CLKOUT3_DIVIDE(1),
        .CLKOUT3_DUTY_CYCLE(0.5),
        .CLKOUT3_PHASE(0),
        .CLKOUT4_DIVIDE(1),
        .CLKOUT4_DUTY_CYCLE(0.5),
        .CLKOUT4_PHASE(0),
        .CLKOUT5_DIVIDE(1),
        .CLKOUT5_DUTY_CYCLE(0.5),
        .CLKOUT5_PHASE(0),
        .CLKOUT6_DIVIDE(1),
        .CLKOUT6_DUTY_CYCLE(0.5),
        .CLKOUT6_PHASE(0),
        .CLKFBOUT_MULT_F(10),
        .CLKFBOUT_PHASE(0),
        .DIVCLK_DIVIDE(1),
        .REF_JITTER1(0.010),
        .CLKIN1_PERIOD(10.0),
        .STARTUP_WAIT("FALSE"),
        .CLKOUT4_CASCADE("FALSE")
    )
    clk_mmcm_inst (
        .CLKIN1(clk),
        .CLKFBIN(mmcm_clkfb),
        .RST(~rst_n),
        .PWRDWN(1'b0),
        .CLKOUT0(clk_int),
        .CLKOUT0B(),
        .CLKOUT1(clk_25mhz_int),
        .CLKOUT1B(),
        .CLKOUT2(),
        .CLKOUT2B(),
        .CLKOUT3(),
        .CLKOUT3B(),
        .CLKOUT4(),
        .CLKOUT5(),
        .CLKOUT6(),
        .CLKFBOUT(mmcm_clkfb),
        .CLKFBOUTB(),
        .LOCKED(mmcm_locked)
    );

    reg [3:0] sync_rst_reg;
    assign rst_n_int = sync_rst_reg[3];

    always @(posedge clk_int or negedge rst_n) begin
        if (!rst_n) begin
            sync_rst_reg <= 0;
        end else begin
            sync_rst_reg <= {sync_rst_reg[2:0], mmcm_locked};
        end
    end

    wire pub_app_data_ap_start;
    wire pub_app_data_ap_ready;
    wire [$clog2(`ROS2_MAX_APP_DATA_LEN)-2:0] pub_app_data_addr;
    wire pub_app_data_ce;
    wire pub_app_data_we;
    wire [15:0] pub_app_data_wdata;
    wire [15:0] pub_data_seed;
    hls_pub_0 hls_pub_inst (
        .ap_clk(clk_int),
        .ap_rst_n(rst_n_int),
        .ap_start(pub_app_data_ap_start),
        .ap_ready(pub_app_data_ap_ready),
        .ap_idle(),
        .ap_done(),
        .pub_app_data_address0(pub_app_data_addr),
        .pub_app_data_ce0(pub_app_data_ce),
        .pub_app_data_we0(pub_app_data_we),
        .pub_app_data_d0(pub_app_data_wdata),
        .pub_data_seed(pub_data_seed)
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
        .ap_clk(clk_int),                                  // input wire ap_clk
        .ap_rst_n(rst_n_int),                              // input wire ap_rst_n
        .sub_app_data_address0(sub_app_data_address0),    // output wire [8 : 0] sub_app_data_address0
        .sub_app_data_q0(sub_app_data_q0),                // input wire [15 : 0] sub_app_data_q0
        .sub_data_result(sub_data_result)                // output wire sub_data_result
    );

    wire [$clog2(`ROS2_MAX_APP_DATA_LEN)-1:0] ros2_sub_app_data_addr;
    wire ros2_sub_app_data_ce;
    wire ros2_sub_app_data_we;
    wire [7:0] ros2_sub_app_data_wdata;
    ros2_module #(
        .ROS2CLK_HZ(ROS2CLK_HZ)
    )
    ros2_inst (
        .clk(clk_int),
        .rst_n(rst_n_int),
        .clk_25mhz(clk_25mhz_int),

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

        .ros2_pub_app_data_ap_start(pub_app_data_ap_start),
        .ros2_pub_app_data_ap_ready(pub_app_data_ap_ready),
        .ros2_pub_app_data_addr0(pub_app_data_addr),
        .ros2_pub_app_data_ce0(pub_app_data_ce),
        .ros2_pub_app_data_we0(pub_app_data_we),
        .ros2_pub_app_data_wdata0(pub_app_data_wdata),
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
        .clka(clk_int),    // input wire clka
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
